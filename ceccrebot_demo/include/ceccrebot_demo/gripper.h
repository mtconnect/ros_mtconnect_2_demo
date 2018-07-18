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
#ifndef __CECCREBOT_DEMO_GRIPPER_H__
#define __CECCREBOT_DEMO_GRIPPER_H__

#include <ros/ros.h>
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace ceccrebot_demo
{

struct Config
{
  double gripper_close_position = 0.0;
  double gripper_open_position = 100.0;
};

class Gripper
{
public:
  Gripper(ros::NodeHandle &nh, ros::NodeHandle &nhp);

  static bool loadConfig(ros::NodeHandle &nh, Config &cfg);

  void inputCallback(robotiq_s_model_control::SModel_robot_input::ConstPtr msg);

  void open() const;
  void close() const;

protected:
  Config cfg_;
  robotiq_s_model_control::SModel_robot_input::ConstPtr curr_input_;
};

} //end namespace

#endif
