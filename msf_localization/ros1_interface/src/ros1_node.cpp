#include <memory>
#include <signal.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "ros1_interface.h"

int main (int argc, char** argv) {
    // Set glog.
    FLAGS_colorlogtostderr = true;
    // Initialize ros.
    ros::init(argc, argv, "msf_localization");
    ros::NodeHandle nh;
    RosInterface localizer(nh);
    ros::spin();
    return 1;
}