
#include "lidar_localization.h"
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS1 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#endif

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#endif

std::shared_ptr<LidarLocalization> lidar_localization_ptr(nullptr);
#ifdef ROS1 
ros::Publisher lidar_pose_pub;
#endif

#ifdef ROS2 
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_pose_pub;
#endif


#ifdef ROS1
Pose odom2pose(const nav_msgs::Odometry &odom) {
  Pose pose;
  pose.timestamp = odom.header.stamp.toSec();
  pose.xyz << odom.pose.pose.position.x, odom.pose.pose.position.y,
      odom.pose.pose.position.z;
  pose.q.x() = odom.pose.pose.orientation.x;
  pose.q.y() = odom.pose.pose.orientation.y;
  pose.q.z() = odom.pose.pose.orientation.z;
  pose.q.w() = odom.pose.pose.orientation.w;
  return pose;
}
#endif

#ifdef ROS2
Pose odom2pose(const nav_msgs::msg::Odometry::SharedPtr &odom) {
  Pose pose;
  double timestamp = odom->header.stamp.sec + odom->header.stamp.nanosec * 1e-9;
  pose.timestamp = timestamp;
  pose.xyz << odom->pose.pose.position.x, 
             odom->pose.pose.position.y, 
             odom->pose.pose.position.z;
  pose.q.x() = odom->pose.pose.orientation.x;
  pose.q.y() = odom->pose.pose.orientation.y;
  pose.q.z() = odom->pose.pose.orientation.z;
  pose.q.w() = odom->pose.pose.orientation.w;
  return pose;
}
#endif

#ifdef ROS1
void relPoseCallback(const nav_msgs::OdometryConstPtr &msg) {
  auto pose = odom2pose(*msg);
  lidar_localization_ptr->addRelPose(pose);
}

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
  pcl::PointCloud<RsPointXYZIRT>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT>);
  pcl::fromROSMsg(*msg, *cloud);
  cloud->header.stamp = cloud->points.back().timestamp*1e6; 
  lidar_localization_ptr->addLidarData(cloud);
}
#endif

#ifdef ROS2
void relPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto pose = odom2pose(msg);
  lidar_localization_ptr->addRelPose(pose);
}

void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<RsPointXYZIRT>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT>);
  pcl::fromROSMsg(*msg, *cloud);
  cloud->header.stamp = cloud->points.back().timestamp * 1e6;
  lidar_localization_ptr->addLidarData(cloud);
}
#endif

int main(int argc, char **argv) {
#ifdef ROS1
  ros::init(argc, argv, "lidar_localization_node");
  ros::NodeHandle nh;

  std::string config_file = std::string(PROJECT_DIR) + "/config/config.yaml";
  std::string lidar_topic = "";
  std::string rel_pose_topic = "";
  std::string abs_pose_topic = "";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(config_file);
    lidar_topic = config_node["lidar_topic"].as<std::string>();
    rel_pose_topic = config_node["rel_pose_topic"].as<std::string>();
    abs_pose_topic = config_node["abs_pose_topic"].as<std::string>();
  } catch (...) {
    std::cout << "config file load failed!" << std::endl;
    return -1;
  }
  lidar_localization_ptr = std::make_shared<LidarLocalization>(config_node);
  lidar_pose_pub = nh.advertise<nav_msgs::Odometry>("/lidar_pose_xyz", 10);
  auto pose_func = [&](const Pose &pose) {
    nav_msgs::Odometry odom;
    Eigen::Vector3d xyz(pose.xyz.x(), pose.xyz.y(),
                        pose.xyz.z());
    odom.header.stamp = ros::Time(pose.timestamp);
    odom.header.frame_id = "rslidar";
    /////加入status
    odom.pose.covariance[0] = static_cast<double>(pose.status_code); // status
    odom.pose.pose.position.x = pose.xyz.x();
    odom.pose.pose.position.y = pose.xyz.y();
    odom.pose.pose.position.z = pose.xyz.z();
    odom.pose.pose.orientation.x = pose.q.x();
    odom.pose.pose.orientation.y = pose.q.y();
    odom.pose.pose.orientation.z = pose.q.z();
    odom.pose.pose.orientation.w = pose.q.w();
    lidar_pose_pub.publish(odom);
  };
  lidar_localization_ptr->registerCallback(pose_func);

  ros::Subscriber lidar_sub = nh.subscribe(lidar_topic, 10, lidarCallback);
  ros::Subscriber rel_pose_sub =
      nh.subscribe(rel_pose_topic, 100, relPoseCallback);

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();
  return 0;
#endif

#ifdef ROS2
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("lidar_localization_node");

  std::string config_file = std::string(PROJECT_DIR) + "/config/config.yaml";

  std::string lidar_topic = "";
  std::string rel_pose_topic = "";
  std::string abs_pose_topic = "";
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(config_file);
    lidar_topic = config_node["lidar_topic"].as<std::string>();
    rel_pose_topic = config_node["rel_pose_topic"].as<std::string>();
    abs_pose_topic = config_node["abs_pose_topic"].as<std::string>();
  } catch (...) {
    std::cout << "config file load failed!" << std::endl;
    return -1;
  }

  lidar_localization_ptr = std::make_shared<LidarLocalization>(config_node);

  lidar_pose_pub = nh->create_publisher<nav_msgs::msg::Odometry>("/lidar_pose_xyz", 10);
  auto pose_func = [&](const Pose &pose) {
    nav_msgs::msg::Odometry odom;
    Eigen::Vector3d xyz(pose.xyz.x(), pose.xyz.y(),
                        pose.xyz.z());
    odom.header.stamp = rclcpp::Time(pose.timestamp * 1e9); // 将秒转为纳秒
    odom.header.frame_id = "rslidar";
    /////加入status
    odom.pose.covariance[0] = static_cast<double>(pose.status_code); // status
    odom.pose.pose.position.x = pose.xyz.x();
    odom.pose.pose.position.y = pose.xyz.y();
    odom.pose.pose.position.z = pose.xyz.z();
    odom.pose.pose.orientation.x = pose.q.x();
    odom.pose.pose.orientation.y = pose.q.y();
    odom.pose.pose.orientation.z = pose.q.z();
    odom.pose.pose.orientation.w = pose.q.w();
    lidar_pose_pub->publish(odom);
  };
  lidar_localization_ptr->registerCallback(pose_func);

  auto lidar_sub = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, 10, lidarCallback);

  auto rel_pose_sub = nh->create_subscription<nav_msgs::msg::Odometry>(
      rel_pose_topic, 100, relPoseCallback);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(nh);
  executor.spin();
  rclcpp::shutdown();
  return 0;

#endif

}