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
#ifndef __CECCREBOT_DEMO_DEMO_H__
#define __CECCREBOT_DEMO_DEMO_H__

#include <condition_variable>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/GripperCommandAction.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <mtconnect_bridge/DeviceWorkAction.h>
#include <xmlrpcpp/XmlRpcValue.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GraspActionClient;
typedef std::shared_ptr<GraspActionClient> GraspActionClientPtr;
typedef std::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef std::map<std::string, double> JointPose;

namespace ceccrebot_demo
{

struct Config
{
  std::string world_frame_id = "world";
  std::string arm_group_name = "arm";
  std::string wrist_link_name = "wrist";
  std::string motion_plan_service = "plan";
  std::string marker_topic = "markers";
  std::string grasp_action_name = "grasp";
  double planning_time = 10.0; //seconds
};

struct PositionAndSpeed
{
  JointPose position;
  double speed_factor;
};

bool loadConfig(ros::NodeHandle &nh, Config &cfg);
void loadPoses(XmlRpc::XmlRpcValue &param, std::map<std::string, PositionAndSpeed> &robot_poses);

class Demo
{
public:
  Demo(ros::NodeHandle &nh, ros::NodeHandle &nhp);

  typedef actionlib::SimpleActionServer<mtconnect_bridge::DeviceWorkAction> MTConnectWorkServer;
  void run();

  void work_dispatch(mtconnect_bridge::DeviceWorkGoal::ConstPtr work);

  //std::vector<geometry_msgs::PoseStamped> create_pick_moves(geometry_msgs::PoseStamped &pose);
  //std::vector<geometry_msgs::PoseStamped> create_place_moves();
  void pick(const std::vector<geometry_msgs::PoseStamped>& pick_poses);
  void place(const std::vector<geometry_msgs::PoseStamped>& place_poses);
  void go_to_pose(const std::string &pose_name);

protected:
  void mtconnect_work_available();
  void mtconnect_work_preempted();

  void cmd_gripper(const std::string &data);
  void stop_robot() const;

  bool create_motion_plan(
      const geometry_msgs::PoseStamped &pose_target,
      const moveit_msgs::RobotState &start_robot_state,
      moveit::planning_interface::MoveGroupInterface::Plan &plan);

  //void show_box(bool show);

private:
  Config cfg_;
  ros::Publisher marker_publisher_;

  MTConnectWorkServer mtconnect_server_;
  mtconnect_bridge::DeviceWorkGoal::ConstPtr curr_work_;
  std::mutex mutex_;
  std::condition_variable work_available_condition_;

  ros::Publisher robot_raw_interface_;
  ros::ServiceClient motion_plan_client_;
  ros::ServiceClient gripper_open_srv_;
  ros::ServiceClient gripper_close_srv_;

  MoveGroupPtr move_group_ptr_;
  std::map<std::string, PositionAndSpeed> robot_poses_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
};

} //end namespace

#endif
