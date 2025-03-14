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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "eskf_interface.h"
#include "type.h"

class Ros1Interface {
public:
    Ros1Interface(ros::NodeHandle& nh);
    ~Ros1Interface();
    //msg callback
    void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    void chassisCallback(const ros_adapter::HunterStatus::ConstPtr &msg_ptr);
    void canOdomCallback(const nav_msgs::OdometryConstPtr& msg_ptr); //user self adapt
    void lidarOdomCallback(const nav_msgs::OdometryConstPtr& msg_ptr);
    //thread
    void waitStart();
    void pubState();
    //sub   
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Subscriber can_odom_sub_;
    ros::Subscriber lidar_odom_sub_;
    //pub
    ros::Publisher traj_pub_;
    ros::Publisher loc_odom_pub_;
private:
    void stateToRos(const Robosense::State& state);
    void trimOdomQueue(double cur_time);
    //eskf 
    std::unique_ptr<Robosense::EskfInterface> eskf_interface_ptr_;
    //pub
    bool pub_enable_ = true;
    nav_msgs::Path fuse_path_;
    tf2_ros::TransformBroadcaster fuse_tf_broadcaster_;
    geometry_msgs::TransformStamped fuse_transform_;
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
    std::deque<Robosense::ImuDataPtr>imu_deque_;
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
