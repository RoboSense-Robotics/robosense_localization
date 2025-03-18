/******************************************************************************
Copyright 2025 RoboSense Technology Co., Ltd

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 *****************************************************************************/
 
#include <memory>
#include <signal.h>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2_interface.h"

int main (int argc, char** argv) {
    // Set glog.
    FLAGS_colorlogtostderr = true;
    // Initialize ROS2.
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("msf_localization");
    Ros2Interface ros_interface(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}