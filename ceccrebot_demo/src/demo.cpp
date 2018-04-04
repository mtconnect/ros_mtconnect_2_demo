/*
Copyright 2018 Southwest Research Institute

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <ceccrebot_demo/demo.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/GetMotionPlan.h>

static const std::string MTCONNECT_WORK_ACTION = "work";

bool ceccrebot_demo::loadConfig(ros::NodeHandle &nh, ceccrebot_demo::Config &cfg)
{
  return true;
}

ceccrebot_demo::Demo::Demo(ros::NodeHandle &nh, ros::NodeHandle &nhp) :
  mtconnect_server_(MTCONNECT_WORK_ACTION, false),
  curr_work_(nullptr)
{
  // reading parameters
  if(loadConfig(nhp, cfg_))
  {
    ROS_INFO_STREAM("Parameters successfully read");
  }
  else
  {
    throw std::runtime_error("Failed to load parameters");
  }

  // moveit interface
  move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(cfg_.arm_group_name);
  move_group_ptr_->setPlannerId("RRTConnectkConfigDefault");

  // motion plan client
  motion_plan_client_ = nh.serviceClient<moveit_msgs::GetMotionPlan>(cfg_.motion_plan_service);

  // marker publisher (rviz visualization)
  marker_publisher_ = nh.advertise<visualization_msgs::Marker>(cfg_.marker_topic, 1);

  // grasp action client (vacuum gripper)
  grasp_action_client_ptr_ = GraspActionClientPtr(new GraspActionClient(cfg_.grasp_action_name, true));

  // waiting to establish connections
  while(ros::ok() && ! grasp_action_client_ptr_->waitForServer(ros::Duration(2.0f)))
  {
    ROS_INFO_STREAM("Waiting for grasp action server: " << cfg_.grasp_action_name);
  }

  if (! ros::ok())
    throw std::runtime_error("ROS has shutdown");

  //start the MTConnect action interface
  //  assume these callbacks are being invoked in their own threads
  mtconnect_server_.registerGoalCallback(boost::bind(&Demo::mtconnect_work_available, this));
  mtconnect_server_.registerPreemptCallback(boost::bind(&Demo::mtconnect_work_preempted, this));
  mtconnect_server_.start();
}

void ceccrebot_demo::Demo::run()
{
  // move to a "clear" position
  go_to_pose("home");

  while (ros::ok())
  {
    std::unique_lock<std::mutex> lock(mutex_);
    work_available_condition_.wait_for(lock, std::chrono::seconds{2});

    //Holds mutex until work completes
    if (curr_work_ != nullptr)
      work_dispatch(curr_work_);
  }
}

void ceccrebot_demo::Demo::mtconnect_work_available()
{
  mtconnect_bridge::DeviceWorkGoal::ConstPtr goal = mtconnect_server_.acceptNewGoal();

  //Replace any current work and notify main thread
  //TODO: a more formal preempting strategy
  std::lock_guard<std::mutex> guard(mutex_);
  curr_work_ = goal;
  work_available_condition_.notify_one();
}

void ceccrebot_demo::Demo::mtconnect_work_preempted()
{
}

void ceccrebot_demo::Demo::work_dispatch(mtconnect_bridge::DeviceWorkGoal::ConstPtr work)
{
  if (work->type == "move")
  {
    go_to_pose(work->data);
  }
}

void ceccrebot_demo::Demo::go_to_pose(const std::string &pose_name)
{
  move_group_ptr_->setNamedTarget(pose_name);
  move_group_ptr_->setPlanningTime(5.0);

  bool success = (bool) move_group_ptr_->move();
  if(success)
  {
    ROS_INFO_STREAM("Move " << pose_name << " Succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("Move " << pose_name << " Failed");
    throw std::runtime_error("movement failed");
  }
}

void ceccrebot_demo::Demo::cmd_gripper(bool close)
{
  // set the corresponding gripper action in the "grasp_goal" object.
  control_msgs::GripperCommandGoal grasp_goal;
  grasp_goal.command.max_effort = cfg_.gripper_effort;
  if (close)
    grasp_goal.command.position = cfg_.gripper_close_position;
  else
    grasp_goal.command.position = cfg_.gripper_open_position;

  grasp_action_client_ptr_->sendGoal(grasp_goal);
  if (grasp_action_client_ptr_->waitForResult(ros::Duration(4.0f)))
  {
    if (close)
      ROS_INFO_STREAM("Gripper closed");
    else
      ROS_INFO_STREAM("Gripper opened");
  }
  else
  {
    throw std::runtime_error(std::string("Gripper failed to ") + (close ? "close" : "open"));
  }
}

bool ceccrebot_demo::Demo::create_motion_plan(
    const geometry_msgs::PoseStamped &pose_target,
    const moveit_msgs::RobotState &start_robot_state,
    moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
	// constructing motion plan goal constraints
	std::vector<double> position_tolerances(3, 0.01f);
	std::vector<double> orientation_tolerances(3, 0.01f);
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
      cfg_.wrist_link_name,
      pose_target,
      position_tolerances,
			orientation_tolerances);

	// creating motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
	moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;
	req.start_state = start_robot_state;
	req.start_state.is_diff = true;
	req.group_name = cfg_.arm_group_name;
	req.goal_constraints.push_back(pose_goal);
	req.allowed_planning_time = 5.0;
	req.num_planning_attempts = 1;

	// request motion plan
	bool success = false;
	if(motion_plan_client_.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
	{
		// saving motion plan results
		plan.start_state_ = res.trajectory_start;
		plan.trajectory_ = res.trajectory;
		success = true;
	}

	return success;
}

void ceccrebot_demo::Demo::pick(const std::vector<geometry_msgs::PoseStamped>& pick_poses)
{
	  move_group_ptr_->setEndEffectorLink(cfg_.wrist_link_name);
	  move_group_ptr_->setPlanningTime(5.0);
	  move_group_ptr_->setPoseReferenceFrame(cfg_.world_frame_id);

	  // move the robot to each wrist pick pose
	  for(unsigned int i = 0; i < pick_poses.size(); i++)
	  {
	  	moveit_msgs::RobotState robot_state;
      moveit::planning_interface::MoveGroupInterface::Plan plan;

      //plan and move
	    bool success = create_motion_plan(pick_poses[i], robot_state, plan) &&
                     move_group_ptr_->execute(plan);

	    // verifying move completion
	    if(success)
	    {
	      ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
	    }
	    else
	    {
	      ROS_ERROR_STREAM("Pick Move " << i <<" Failed");
        throw std::runtime_error("pick failed");
	    }

	    if(i == 1) //About to grab part
	    {
	      cmd_gripper(true);
	    }
	  }
}

void ceccrebot_demo::Demo::place(const std::vector<geometry_msgs::PoseStamped>& place_poses)
{
  move_group_ptr_->setEndEffectorLink(cfg_.wrist_link_name);
  move_group_ptr_->setPoseReferenceFrame(cfg_.world_frame_id);
  move_group_ptr_->setPlanningTime(5.0);

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
  	moveit_msgs::RobotState robot_state;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    //plan and move
    bool success = create_motion_plan(place_poses[i], robot_state, plan) &&
                   move_group_ptr_->execute(plan);

    if(success)
    {
      ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("Place Move " << i <<" Failed");
      throw std::runtime_error("place failed");
    }

    if(i == 1) //Part ready to be released
    {
      cmd_gripper(false);
    }
  }
}

/**
 * Define the sequence of pick moves to grab a part.
 * \param pose - The location of the part with the z-axis pointing towards the robot
 */
/*
std::vector<geometry_msgs::PoseStamped> ceccrebot_demo::Demo::create_pick_moves(geometry_msgs::PoseStamped &pose)
{
  if (pose.header.frame_id != cfg_.world_frame_id)
    throw std::runtime_error("Pick location not given in world coordinates");

  //Create pose with detected object's position but default orientation
  geometry_msgs::PoseStamped pick_pose;
  pick_pose.header = pose.header;
  pick_pose.pose.position = pose.pose.position;
  pick_pose.pose.orientation = tf::createQuaternionMsgFromYaw(cfg_.PICK_YAW);

  ROS_INFO("World pick pose: position = (%f,%f,%f), orientation = (%f,%f,%f,%f)",
    pick_pose.pose.position.x,
    pick_pose.pose.position.y,
    pick_pose.pose.position.z,
    pick_pose.pose.orientation.x,
    pick_pose.pose.orientation.y,
    pick_pose.pose.orientation.z,
    pick_pose.pose.orientation.w);

  // flip pose to point z axis at object
  // tcp should have z axis pointing at object during pick
  tf::Transform flip_about_x(tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), M_PI), tf::Vector3(0, 0, 0));
  geometry_msgs::PoseStamped tcp_pick_pose = transformPose(pick_pose, flip_about_x);

  // create all the poses for tcp's pick motion (approach, pick and retreat)
  std::vector<geometry_msgs::PoseStamped> tcp_pick_poses = create_manipulation_poses(
      tcp_pick_pose,
      cfg_.RETREAT_DISTANCE,
      cfg_.APPROACH_DISTANCE);

  tf::StampedTransform tcp_to_wrist_tf;
  transform_listener_.waitForTransform(cfg_.TCP_LINK_NAME, cfg_.wrist_link_name,ros::Time::now(),ros::Duration(3.0f));
  transform_listener_.lookupTransform(cfg_.TCP_LINK_NAME, cfg_.wrist_link_name, ros::Time(0.0f), tcp_to_wrist_tf);

  // wrist poses
  std::vector<geometry_msgs::PoseStamped> wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_pick_poses);

  // printing some results
  ROS_INFO_STREAM("tcp position at pick: " << tcp_pick_pose.pose.position);
  ROS_INFO_STREAM("wrist position at pick: " << wrist_pick_poses.at(1).pose.position);

  return wrist_pick_poses;
}
*/

/*
std::vector<geometry_msgs::PoseStamped> ceccrebot_demo::Demo::create_place_moves()
{
  geometry_msgs::PoseStamped place_pose;
  place_pose.header.frame_id = cfg_.world_frame_id;
  place_pose.header.stamp = ros::Time::now();

  place_pose.pose.position.x = cfg_.BOX_PLACE_POS.x();
  place_pose.pose.position.y = cfg_.BOX_PLACE_POS.y();
  place_pose.pose.position.z = cfg_.BOX_PLACE_POS.z();
  place_pose.pose.orientation = tf::createQuaternionMsgFromYaw(cfg_.BOX_PLACE_YAW);

  //tcp should have z axis pointing at place position
  tf::Transform flip_about_x(tf::Quaternion(tf::Vector3(1.0, 0.0, 0.0), M_PI), tf::Vector3(0, 0, 0));
  geometry_msgs::PoseStamped tcp_place_pose = transformPose(place_pose, flip_about_x);

  std::vector<geometry_msgs::PoseStamped> tcp_place_poses = create_manipulation_poses(
      tcp_place_pose,
      cfg_.RETREAT_DISTANCE,
      cfg_.APPROACH_DISTANCE);

  tf::StampedTransform tcp_to_wrist_tf;
  transform_listener_.waitForTransform(cfg_.TCP_LINK_NAME, cfg_.wrist_link_name, ros::Time(0.0f), ros::Duration(3.0f));
  transform_listener_.lookupTransform(cfg_.TCP_LINK_NAME, cfg_.wrist_link_name, ros::Time(0.0f), tcp_to_wrist_tf);

  std::vector<geometry_msgs::PoseStamped> wrist_place_poses = transform_from_tcp_to_wrist(
      tcp_to_wrist_tf,
      tcp_place_poses);

  // printing results
  ROS_INFO_STREAM("tcp position at place: " << tcp_place_pose.pose.position);
  ROS_INFO_STREAM("tcp orientation at place: " << tcp_place_pose.pose.orientation);
  ROS_INFO_STREAM("wrist position at place: " << wrist_place_poses.at(1).pose.position);

  return wrist_place_poses;
}
*/
