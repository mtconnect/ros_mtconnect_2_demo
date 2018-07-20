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
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <xmlrpcpp/XmlRpcException.h>

static const std::string MTCONNECT_WORK_ACTION = "work";
static const std::string GRIPPER_OPEN_SERVICE = "gripper_open";
static const std::string GRIPPER_CLOSE_SERVICE = "gripper_close";

inline double deg_to_rad(double d) {return d * 3.141592/180.0;}

bool ceccrebot_demo::loadConfig(ros::NodeHandle &nh, ceccrebot_demo::Config &cfg)
{
  nh.param<std::string>("world_frame_id", cfg.world_frame_id, cfg.world_frame_id);
  nh.param<std::string>("arm_group_name", cfg.arm_group_name, cfg.arm_group_name);
  nh.param<std::string>("wrist_link_name", cfg.wrist_link_name, cfg.wrist_link_name);
  nh.param<std::string>("motion_plan_service", cfg.motion_plan_service, cfg.motion_plan_service);
  nh.param<std::string>("marker_topic", cfg.marker_topic, cfg.marker_topic);
  return true;
}

void ceccrebot_demo::loadPoses(XmlRpc::XmlRpcValue &param, std::map<std::string, JointPose> &robot_poses)
{
  try
  {
    std::vector<std::string> joint_names;
    XmlRpc::XmlRpcValue joints_param = param["joints"];
    for (int i=0; i < joints_param.size(); ++i)
    {
      joint_names.push_back(joints_param[i]);
    }
    XmlRpc::XmlRpcValue positions_param = param["positions"];
    for (auto it = positions_param.begin(); it != positions_param.end(); ++it)
    {
      std::string pose_name = it->first;
      XmlRpc::XmlRpcValue joint_values = it->second;

      if (joint_names.size() != joint_values.size())
        throw std::runtime_error(
            "pose '" + pose_name + "': mismatch with number of joints, expected " + std::to_string(joint_names.size()));

      std::map<std::string, double> pose;
      for (int i=0; i < joint_names.size(); ++i)
      {
        pose[joint_names[i]] = deg_to_rad(joint_values[i]);
      }
      robot_poses[pose_name] = pose;
    }
  }
  catch (XmlRpc::XmlRpcException &ex)
  {
    throw std::runtime_error("XmlRpc error: " + ex.getMessage());
  }
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

  XmlRpc::XmlRpcValue poses_param;
  if (! nhp.getParam("poses", poses_param))
    throw std::runtime_error("Required parameter 'poses' not found");
  loadPoses(poses_param, robot_poses_);

  //direct URScript interface
  robot_raw_interface_ = nh.advertise<std_msgs::String>("ur_driver/URScript", 10);

  // moveit interface
  move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(cfg_.arm_group_name);
  move_group_ptr_->setPlannerId("RRTConnectkConfigDefault");

  // motion plan client
  motion_plan_client_ = nh.serviceClient<moveit_msgs::GetMotionPlan>(cfg_.motion_plan_service);

  // marker publisher (rviz visualization)
  marker_publisher_ = nh.advertise<visualization_msgs::Marker>(cfg_.marker_topic, 1);

  gripper_open_srv_ = nh.serviceClient<std_srvs::Trigger>(GRIPPER_OPEN_SERVICE);
  if (! gripper_open_srv_.waitForExistence(ros::Duration(3.0)))
  {
    ROS_WARN("Gripper open service not available");
    //throw std::runtime_error("Gripper open service not available");
  }
  gripper_close_srv_ = nh.serviceClient<std_srvs::Trigger>(GRIPPER_CLOSE_SERVICE);
  if (! gripper_close_srv_.waitForExistence(ros::Duration(3.0)))
  {
    ROS_WARN("Gripper close service not available");
    //throw std::runtime_error("Gripper close service not available");
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
  //TODO: a more formal preempting strategy, the client may decide to cancel
  std::lock_guard<std::mutex> guard(mutex_);
  curr_work_ = goal;
  work_available_condition_.notify_one();
}

void ceccrebot_demo::Demo::mtconnect_work_preempted()
{
}

void ceccrebot_demo::Demo::work_dispatch(mtconnect_bridge::DeviceWorkGoal::ConstPtr work)
{
  //TODO: send feedback: performing work
  try
  {
    if (work->type == "move")
    {
      ROS_INFO_STREAM("Received work: move to " << work->data);
      go_to_pose(work->data);
    }
    if (work->type == "gripper")
    {
      ROS_INFO_STREAM("Received work: command gripper to " << work->data);
      cmd_gripper(work->data);
    }
    mtconnect_server_.setSucceeded();
  }
  catch (const std::runtime_error &ex)
  {
    ROS_ERROR_STREAM("Work execution failed: " << ex.what());
    mtconnect_server_.setAborted();
  }
  curr_work_ = nullptr;
}

void ceccrebot_demo::Demo::go_to_pose(const std::string &pose_name)
{
  if (robot_poses_.count(pose_name) == 0)
    throw std::runtime_error("Move target '" + pose_name + "' is unknown");

  move_group_ptr_->setJointValueTarget(robot_poses_[pose_name]);
  move_group_ptr_->setPlanningTime(5.0);

  bool success = (bool) move_group_ptr_->move();
  //stop_robot();
  if(success)
  {
    ROS_INFO_STREAM("Move " << pose_name << " Succeeded");
  }
  else
  {
    throw std::runtime_error("Failed to move to " + pose_name);
  }
}

void ceccrebot_demo::Demo::cmd_gripper(const std::string &cmd)
{
  if (cmd == "open")
  {
    std_srvs::Trigger srv_data;
    if (! gripper_open_srv_.call(srv_data))
    {
      throw std::runtime_error("Gripper open failed for an unknown reason");
    }
    if (! srv_data.response.success)
    {
      std::ostringstream ss;
      ss << "Gripper open failed: " << srv_data.response.message;
      throw std::runtime_error(ss.str());
    }
  }
  else if (cmd == "close")
  {
    std_srvs::Trigger srv_data;
    if (! gripper_open_srv_.call(srv_data))
    {
      throw std::runtime_error("Gripper close failed for an unknown reason");
    }
    if (! srv_data.response.success)
    {
      std::ostringstream ss;
      ss << "Gripper close failed: " << srv_data.response.message;
      throw std::runtime_error(ss.str());
    }
  }
  else
  {
    std::ostringstream ss;
    ss << "Unrecognized gripper command data: " << cmd;
    throw std::runtime_error(ss.str());
  }
}

void ceccrebot_demo::Demo::stop_robot() const
{
  std::string stop_cmd = "stopj(1)";

  std_msgs::String msg;
  msg.data = stop_cmd;
  robot_raw_interface_.publish(msg);
  ROS_WARN("Sent robot stop");
}

bool ceccrebot_demo::Demo::create_motion_plan(
    const geometry_msgs::PoseStamped &pose_target,
    const moveit_msgs::RobotState &start_robot_state,
    moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
	// constructing motion plan goal constraints
	double joint_tolerance = 0.05;
  //const moveit::core::RobotState robot_state;
  const robot_model::JointModelGroup * joint_model_group = move_group_ptr_->getRobotModel()->getJointModelGroup(cfg_.arm_group_name);
	//moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(
      //start_robot_state,
      //joint_model_group,
      //joint_tolerance);

	// creating motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
	moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;
	req.start_state = start_robot_state;
	req.start_state.is_diff = true;
	req.group_name = cfg_.arm_group_name;
	//req.goal_constraints.push_back(joint_goal);
	req.allowed_planning_time = 5.0;
	req.num_planning_attempts = 10;

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
	      cmd_gripper("close");
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
      cmd_gripper("open");
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
