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

#ifndef LIDAR_LOCALIZATION_H
#define LIDAR_LOCALIZATION_H

#include <mutex>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <queue>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <pcl/features/normal_3d.h>  // 用于NormalEstimation
#include <pcl/search/kdtree.h>  
#include "data_type.hpp"
#include "localization.hpp"
#include "utility.hpp"

#include "lidar_matcher.h"

class LidarLocalization
{
public:
  LidarLocalization() = default;
  LidarLocalization(const YAML::Node& config_node);

  // 添加相对位姿
  void addRelPose(const Pose& pose);

  // 添加激光雷达数据
  void addLidarData(const pcl::PointCloud<RsPointXYZIRT>::Ptr& cloud_ptr);

  void registerCallback(const std::function<void(const Pose&)>& callback)
  {
    callbacks_.emplace_back(callback);
  }

private:
  YAML::Node config_node_;
  STATUS status_ = STATUS::IDLE;
  std::shared_ptr<LidarMatcher> lidar_matcher_;
  // map
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_ptr_;
  pcl::PointCloud<PointT>::Ptr map_cloud_ptr_;
  std::string map_path_;
  size_t input_cloud_size_thr_{1000};
  // 初始化地图
  bool initMap();

  void process();

  void semanticFilter(const PointCloudT::Ptr& undistorted_cloud, PointCloudT::Ptr& semantic_cloud);

  bool undistortPointCloud(const pcl::PointCloud<RsPointXYZIRT>::Ptr lidar_cloud, PointCloudT::Ptr& undistorted_cloud,
                           Eigen::Affine3d& pose_undistort);
  bool forwardPropagate(const Pose& last_pose, double t, Pose& pose_at_t,Pose& origin_pose_at_t);

  PointCloudT::Ptr voxelGridFilter(const PointCloudT::Ptr& cloud_ptr, float leaf_size);

  // frame
  pcl::PointCloud<PointT>::Ptr undistor_cloud_;
  Pose last_lidar_pose_;

  // rel pose
  std::map<double, Pose> rel_poses_map_;
  std::mutex rel_mutex_;
  // init position and orientation in map
  Eigen::Vector3d init_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond init_orientation_ = Eigen::Quaterniond::Identity();

  double MATCHING_CHECK_RESIDUAL_THRESH = 0.3;

  // calib
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();

  // callback
  std::vector<std::function<void(const Pose&)>> callbacks_;

  // is_pub_cloud
  bool is_pub_cloud_{true};
  bool is_pub_map_{false};
  // blind_distance_
  double blind_distance_{2.0};
};

#endif  // LIDAR_LOCALIZATION_H