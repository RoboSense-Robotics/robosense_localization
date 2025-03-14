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