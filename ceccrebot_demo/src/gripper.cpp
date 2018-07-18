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
#include <ceccrebot_demo/gripper.h>

ceccrebot_demo::Gripper::Gripper(ros::NodeHandle &nh, ros::NodeHandle &nhp) :
  curr_input_(new robotiq_s_model_control::SModel_robot_input)
{
}

bool ceccrebot_demo::Gripper::loadConfig(ros::NodeHandle &nh, Config &cfg)
{
  return true;
}


void ceccrebot_demo::Gripper::inputCallback(robotiq_s_model_control::SModel_robot_input::ConstPtr msg)
{
  curr_input_ = msg;
}

void ceccrebot_demo::Gripper::open() const
{
}

void ceccrebot_demo::Gripper::close() const
{
}
