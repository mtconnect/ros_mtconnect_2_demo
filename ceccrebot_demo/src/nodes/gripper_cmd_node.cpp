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
#include <atomic>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>
#include <std_srvs/Trigger.h>
#include <xmlrpcpp/XmlRpcValue.h>

static constexpr double TIMEOUT = 5.0; //seconds

namespace ceccrebot_demo
{

struct Config
{
  int gripper_close_position = 255;
  int gripper_open_position = 0;
  int gripper_speed = 50;
  int gripper_force = 255;
  int scissor_position = 150;
};

enum class GripperStatus
{
  OPEN,
  CLOSED
};

bool loadConfig(ros::NodeHandle &nh, Config &cfg);

class GripperControl
{
public:
  GripperControl(ros::NodeHandle &nh, ros::NodeHandle &nhp, ros::NodeHandle &srv_nh);

  void inputCallback(robotiq_s_model_control::SModel_robot_input::ConstPtr msg);
  bool activate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  bool open(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
  bool close(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

protected:
  std_srvs::Trigger::Response runCommand(const robotiq_s_model_control::SModel_robot_output &msg) const;
  bool waitMotionStart(double timeout) const;
  bool waitMotionStop(double timeout) const;

private:
  Config cfg_;

  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;
  ros::ServiceServer activate_srv_;
  ros::ServiceServer open_srv_;
  ros::ServiceServer close_srv_;

  robotiq_s_model_control::SModel_robot_input::ConstPtr curr_input_;
  std::atomic<bool> stationary_;
};

GripperControl::GripperControl(ros::NodeHandle &nh, ros::NodeHandle &nhp, ros::NodeHandle &srv_nh) :
  curr_input_(new robotiq_s_model_control::SModel_robot_input),
  stationary_(true)
{
  if (! loadConfig(nh, cfg_))
    throw std::runtime_error("Failed to load node configuration");

  // I/O with device
  input_sub_ = nh.subscribe("input", 10, &GripperControl::inputCallback, this);
  output_pub_ = nh.advertise<robotiq_s_model_control::SModel_robot_output>("output", 10);

  // External interfaces
  activate_srv_ = srv_nh.advertiseService("activate", &GripperControl::activate, this);
  open_srv_ = srv_nh.advertiseService("open", &GripperControl::open, this);
  close_srv_ = srv_nh.advertiseService("close", &GripperControl::close, this);
}

bool loadConfig(ros::NodeHandle &nh, Config &cfg)
{
  return true;
}

robotiq_s_model_control::SModel_robot_output outputMsgFromConfig(const Config &cfg)
{
  robotiq_s_model_control::SModel_robot_output msg;
  msg.rACT = 1;
  msg.rGTO = 1;
  msg.rICF = 0;
  msg.rICS = 1;
  msg.rSPA = cfg.gripper_speed;
  msg.rFRA = cfg.gripper_force;
  msg.rPRS = cfg.scissor_position;
  return msg;
}

void GripperControl::inputCallback(robotiq_s_model_control::SModel_robot_input::ConstPtr msg)
{
  curr_input_ = msg;
  if (msg->gGTO == 1)
    stationary_ = (msg->gSTA != 0);
}

bool GripperControl::activate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  return true;
}

bool GripperControl::open(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  auto out_msg = outputMsgFromConfig(cfg_);
  out_msg.rPRA = cfg_.gripper_open_position;
  resp = runCommand(out_msg);
  return true;
}

bool GripperControl::close(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
  auto out_msg = outputMsgFromConfig(cfg_);
  out_msg.rPRA = cfg_.gripper_close_position;
  resp = runCommand(out_msg);
  return true;
}

std_srvs::Trigger::Response GripperControl::runCommand(const robotiq_s_model_control::SModel_robot_output &msg) const
{
  std_srvs::Trigger::Response resp;
  output_pub_.publish(msg);
  if (! waitMotionStart(TIMEOUT))
  {
    resp.success = false;
    resp.message = "failed to start motion";
    return resp;
  }
  if (! waitMotionStop(TIMEOUT))
  {
    resp.success = false;
    resp.message = "failed to complete motion";
    return resp;
  }
  resp.success = true;
  return resp;
}

bool GripperControl::waitMotionStart(double timeout) const
{
  ros::Time t0 = ros::Time::now();
  while ((ros::Time::now() - t0).toSec() < timeout)
  {
    if (! stationary_)
    {
      ROS_INFO("Wait start: Moving");
      return true;
    }
    else
    {
      ROS_INFO("Wait start: Static");
    }
    ros::Duration(0.05).sleep();
  }
  return false;
}

bool GripperControl::waitMotionStop(double timeout) const
{
  ros::Time t0 = ros::Time::now();
  while ((ros::Time::now() - t0).toSec() < timeout)
  {
    if (stationary_)
    {
      ROS_INFO("Wait stop: Static");
      return true;
    }
    else
    {
      ROS_INFO("Wait stop: Moving");
    }
    ros::Duration(0.05).sleep();
  }
  return false;
}

} //end namespace

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "gripper_control");
  ros::NodeHandle nh, nhp("~");
  ros::AsyncSpinner spinner(2);

  //Service callbacks need to be asynchronous with rest of system
  ros::CallbackQueue srv_queue;
  ros::NodeHandle srv_nh;
  srv_nh.setCallbackQueue(&srv_queue);
  ros::AsyncSpinner srv_spinner(2, &srv_queue);

  try
  {
    ceccrebot_demo::GripperControl n(nh, nhp, srv_nh);

    spinner.start();
    srv_spinner.start();
    ros::waitForShutdown();
  }
  catch (const std::runtime_error &ex)
  {
    ROS_ERROR_STREAM("gripper_control node: " << ex.what());
  }
  return 0;
}
