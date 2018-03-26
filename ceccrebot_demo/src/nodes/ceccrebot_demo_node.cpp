/*
Copyright 2017 Southwest Research Institute

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

int main(int argc, char * argv[])
{
  // ros initialization
  ros::init(argc, argv, "ceccrebot_demo");
  ros::NodeHandle nh, nhp("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  try
  {
    ceccrebot_demo::Demo demo(nh, nhp);
    demo.run();
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Fatal error: " << e.what());
    return -1;
  }
  return 0;
}
