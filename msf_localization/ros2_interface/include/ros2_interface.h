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

#pragma once
#include <thread>
#include <atomic>
#include <fstream>
#include <memory>
#include <map>
#include <mutex>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iomanip>
#include <glog/logging.h>
#include <iostream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>      
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "eskf_interface.h"

#include "type.h"

class Ros2Interface {
public:
    Ros2Interface(std::shared_ptr<rclcpp::Node> node);
    ~Ros2Interface();
    //msg callback
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);
    void canOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr); //user self adapt
    void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);
    //thread
    void waitStart();
    void pubState();
    //sub   
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr can_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
    //pub
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr loc_odom_pub_;
    inline rclcpp::Time get_ros_time(double timestamp)
    {
        int32_t sec = std::floor(timestamp);
        auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
        uint32_t nanosec = nanosec_d;
        return rclcpp::Time(sec, nanosec);
    }
private:
    std::shared_ptr<rclcpp::Node> nh_;
    void stateToRos(const Robosense::State& state);
    void trimOdomQueue(double cur_time);
    //eskf 
    std::unique_ptr<Robosense::EskfInterface> eskf_interface_ptr_;
    //pub
    bool pub_enable_ = true;
    nav_msgs::msg::Path fuse_path_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> fuse_tf_broadcaster_;     
    geometry_msgs::msg::TransformStamped fuse_transform_;
    //msg receive flag
    std::atomic<bool>imu_ready_{false};
    std::atomic<bool>can_ready_{false};
    std::atomic<bool>lidar_ready_{false};
    std::atomic<bool>started_{false};
    // init msg
    Robosense::ImuDataPtr init_imu_ptr_;
    Robosense::CanOdomDataPtr init_can_ptr_;
    Robosense::LidarOdomDataPtr init_lidar_ptr_;
    // buffer mutex
    std::mutex can_odom_buff_mutex_;
    std::map<double, Robosense::CanOdomDataPtr> can_odom_buff_;
    // can vel calibration ratio
    double can_vel_ratio_;
    std::string meta_config_;
    // debug
    bool pub_traj_{true};
    bool can_update_{false};
    // imu to vehicle transform
    Eigen::Matrix3d rotation_imu_vehicle_;
    Eigen::Vector3d t_imu_vehicle_;
    std::atomic<int> init_imu_num_{1};
    // mean gyr and acc when static
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_acc_ = {0,0,9.8013474249};
    // can speed m/s
    double can_speed_mps_;
    double g_ = 9.8013474249;
    // car moving status
    std::atomic<Robosense::CarStatus> cur_car_status_{Robosense::stop};
    // thread
    std::thread wait_start_thread_;
    std::thread pub_state_thread_;


};
