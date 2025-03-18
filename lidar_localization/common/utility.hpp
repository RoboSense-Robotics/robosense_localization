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
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include "data_type.hpp"
template <typename PointT>
bool removeNaNFromPCLCloud(typename pcl::PointCloud<PointT>::Ptr in) {
  std::vector<int> indices;
  in->is_dense = false;
  pcl::removeNaNFromPointCloud(*in, *in, indices);
  if (in->points.size() == 0) {
    std::cerr << "ERROR: Point cloud is empty! " << std::endl;
    return false;
  }
  return true;
}

inline Eigen::Affine3d poseInterp(double t, Pose const &aff1,
                                  Pose const &aff2) {
  /* assume here t1 <= t <= t2 */
  double t1 = aff1.timestamp;
  double t2 = aff2.timestamp;
  double alpha = 0.0;
  if (t2 != t1) {
    alpha = (t - t1) / (t2 - t1);
  }

  Eigen::Quaternion<double> rot1, rot2;

  rot1.w() = aff1.q.w();
  rot1.x() = aff1.q.x();
  rot1.y() = aff1.q.y();
  rot1.z() = aff1.q.z();

  rot2.w() = aff2.q.w();
  rot2.x() = aff2.q.x();
  rot2.y() = aff2.q.y();
  rot2.z() = aff2.q.z();

  Eigen::Vector3d trans1, trans2;

  trans1.x() = aff1.xyz.x();
  trans1.y() = aff1.xyz.y();
  trans1.z() = aff1.xyz.z();

  trans2.x() = aff2.xyz.x();
  trans2.y() = aff2.xyz.y();
  trans2.z() = aff2.xyz.z();

  Eigen::Affine3d result;
  result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
  result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();

  return result;
}

inline bool
pointsUndistort(const pcl::PointCloud<RsPointXYZIRT>::Ptr &cloud_ptr,
                const Pose &pre_odom, const Pose &cur_odom,
                double undistort_time, bool com_to_end = true) {
  if (pre_odom.timestamp > cur_odom.timestamp) {
    std::cout << " undistort return " << std::endl;
    return false;
  }
  Eigen::Affine3d dst_pose;
  if (com_to_end) {
    dst_pose.translation() = cur_odom.xyz;
    dst_pose.linear() = cur_odom.q.toRotationMatrix();
  } else {
    dst_pose = poseInterp(undistort_time, pre_odom, cur_odom);
  }

  Eigen::Matrix4d dst_pose_inverse = dst_pose.matrix().inverse();
  for (size_t i = 0; i < cloud_ptr->size(); ++i) {
    auto &pt = cloud_ptr->points[i];
    auto pt_pose = poseInterp(pt.timestamp, pre_odom, cur_odom);
    auto t_mat = dst_pose_inverse * pt_pose.matrix();

    Eigen::Affine3d transformation;
    transformation.matrix() = t_mat;
    Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
    pcl::transformPoint(pt1, pt1, transformation);
    pt.x = pt1.x();
    pt.y = pt1.y();
    pt.z = pt1.z();
  }
  return true;
}

inline bool
pointsUndistort(const pcl::PointCloud<RsPointXYZIRT>::Ptr &cloud_ptr,
                const pcl::PointCloud<PointT>::Ptr &outcloud,
                const Pose &pre_odom, const Pose &cur_odom,
                double undistort_time, Eigen::Affine3d &dst_pose,
                bool com_to_end = true) {
  if (pre_odom.timestamp > cur_odom.timestamp) {
    std::cout << " undistort return " << std::endl;
    return false;
  }
  if (com_to_end) {
    dst_pose.translation() = cur_odom.xyz;
    dst_pose.linear() = cur_odom.q.toRotationMatrix();
  } else {
    dst_pose = poseInterp(undistort_time, pre_odom, cur_odom);
  }
  Eigen::Matrix4d dst_pose_inv = dst_pose.matrix().inverse();
  int pt_count = 0;

  double last_time = 0.0;
  Eigen::Affine3d pt_pose;
  Eigen::Matrix4d t_mat;

  for (size_t i = 0; i < cloud_ptr->size(); ++i) {

    PointT added_pt;
    double timestamp = cloud_ptr->points[i].timestamp;

    if (timestamp - last_time > 2e-5) {
      pt_pose = poseInterp(timestamp, pre_odom, cur_odom);
      t_mat = dst_pose_inv * pt_pose.matrix();
      last_time = timestamp;
    }

    Eigen::Affine3d transformation;
    transformation.matrix() = t_mat;
    Eigen::Vector3d pt1(cloud_ptr->points[i].x, cloud_ptr->points[i].y,
                        cloud_ptr->points[i].z);
    pcl::transformPoint(pt1, pt1, transformation);
    added_pt.x = pt1.x();
    added_pt.y = pt1.y();
    added_pt.z = pt1.z();
    added_pt.intensity = cloud_ptr->points[i].intensity;
    outcloud->points.emplace_back(added_pt);
  }
  return true;
}

inline bool
removeDynamicAndDownsample(const pcl::PointCloud<RsPointXYZIRT>::Ptr &cloud_ptr,
                           const pcl::PointCloud<PointT>::Ptr &outcloud,
                           std::vector<int> &instance, int &point_filter_num) {
  int pt_count = 0;
  for (size_t i = 0; i < cloud_ptr->size(); ++i) {
    if (instance[i] != -1)
      continue;
    pt_count++;
    PointT added_pt;
    if (pt_count % point_filter_num == 0) {
      added_pt.x = cloud_ptr->points[i].x;
      added_pt.y = cloud_ptr->points[i].y;
      added_pt.z = cloud_ptr->points[i].z;
      added_pt.intensity = cloud_ptr->points[i].intensity;
      outcloud->points.emplace_back(added_pt);
    }
  }
  std::cout << "outcloud->size : " << outcloud->size() << std::endl;
  return true;
}

inline bool
removeDynamicAndDownsample(const pcl::PointCloud<RsPointXYZIRT>::Ptr &cloud_ptr,
                           const pcl::PointCloud<RsPointXYZIRT>::Ptr &outcloud,
                           std::vector<int> &instance, int &point_filter_num) {
  int pt_count = 0;
  for (size_t i = 0; i < cloud_ptr->size(); ++i) {
    if (instance[i] != -1)
      continue;
    if (pt_count++ % point_filter_num == 0) {
      outcloud->points.emplace_back(cloud_ptr->points[i]);
    }
  }
  std::cout << "outcloud->size : " << outcloud->size() << std::endl;
  return true;
}

template <typename T>
inline typename pcl::PointCloud<T>::Ptr
passthroughFilter(const typename pcl::PointCloud<T>::Ptr &cloud, double x_min,
                  double x_max, double y_min, double y_max) {
  typename pcl::PointCloud<T>::Ptr cloud_filtered(new
                                                  typename pcl::PointCloud<T>);

  pcl::PassThrough<T> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*cloud_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*cloud_filtered);

  return cloud_filtered;
}

template <typename T>
inline typename pcl::PointCloud<T>::Ptr
voxelGridFilter(const typename pcl::PointCloud<T>::Ptr &cloud,
                double xy_leaf_size = 0.5, double z_leaf_size = 0.5) {
  typename pcl::PointCloud<T>::Ptr cloud_filtered(
      new typename pcl::PointCloud<T>());

  pcl::VoxelGrid<T> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(xy_leaf_size, xy_leaf_size, z_leaf_size);
  sor.filter(*cloud_filtered);

  return cloud_filtered;
}

template <typename T>
inline typename pcl::PointCloud<T>::Ptr
uniformSample(const typename pcl::PointCloud<T>::Ptr &cloud,
              double leaf_size = 0.5) {
  pcl::UniformSampling<T> us;

  us.setInputCloud(cloud);
  us.setRadiusSearch(leaf_size);
  typename pcl::PointCloud<T>::Ptr cloud_filtered(new
                                                  typename pcl::PointCloud<T>);
  us.filter(*cloud_filtered);

  return cloud_filtered;
}

inline int fromCalibFileToPose(const std::string lidar_topic,
                               const std::string calibFilePath, double &x,
                               double &y, double &z, double &roll,
                               double &pitch, double &yaw) {
  // AERROR << "RUN HERE ----";
  YAML::Node configNode;
  try {
    configNode = YAML::LoadFile(calibFilePath);
    const YAML::Node sensorsNode = configNode["sensors"];
    const YAML::Node lidarsNode = sensorsNode["lidar"];

    for (size_t i = 0; i < lidarsNode.size(); ++i) {
      const YAML::Node &lidarNode = lidarsNode[i];
      const std::string &topic = lidarNode["topic"].as<std::string>();
      // AERROR << "topic = " << topic;
      if (topic == lidar_topic) {
        const YAML::Node calibNode = lidarNode["calibration"];
        x = calibNode["x"].as<double>();
        y = calibNode["y"].as<double>();
        z = calibNode["z"].as<double>();
        roll = calibNode["roll"].as<double>();
        pitch = calibNode["pitch"].as<double>();
        yaw = calibNode["yaw"].as<double>();

        break;
      }
    }

  } catch (...) {
    return -1;
  }

  return 0;
}

class RSTicToc {
 public:
  RSTicToc(std::string name) { name_ = name; }

  ~RSTicToc() {}

  void tic() {
    if (!start_) {
      start_ = true;
      gettimeofday(&start_t_, NULL);
    }
  }

  void toc() {
    if (start_) {
      gettimeofday(&finish_t_, NULL);
      dur_ms_ =
          (finish_t_.tv_sec - start_t_.tv_sec) * 1000.0 +
          static_cast<double>(finish_t_.tv_usec - start_t_.tv_usec) / 1000.0;
      std::cout << "========> " << name_ << " time consuming : " << dur_ms_
                << "ms" << std::endl;
    }
  }

  double tocAndGetTime() {
    if (start_) {
      gettimeofday(&finish_t_, NULL);
      dur_ms_ =
          (finish_t_.tv_sec - start_t_.tv_sec) * 1000.0 +
          static_cast<double>(finish_t_.tv_usec - start_t_.tv_usec) / 1000.0;
      // std::cout << "========> " << name_ << " time consuming : " << dur_ms_
      // << "ms" << std::endl;
    }
    return getTime();
  }

  double getTime() { return dur_ms_; }

 private:
  std::string name_;
  bool start_{false};
  struct timeval start_t_;
  struct timeval finish_t_;
  double dur_ms_;
};

/** @brief check whether given param is between the given lower and upper
 * boundary for fool-proofing
 *  @param[in] param parameter to check
 *  @param[in] name
 *  @param[in] method comparison method
 *  @param[in] min lower boundary
 *  @param[in] max upper boundary
 */
template <typename T>
void checkParamValidity(T& param, std::string name, int method = 0,
                        T min = -std::numeric_limits<T>::max(),
                        T max = std::numeric_limits<T>::max()) {
  bool param_is_valid = true;
  if (method == 0) {
    if (param > max) {
      LERROR << "===Error Parameter=== " << name
             << " should not exceed upper boundary: " << max
             << ", given is: " << param << REND;
      param_is_valid = false;
    } else if (param <= min) {
      LERROR << "===Error Parameter=== " << name
             << " should not exceed lower boundary: " << min
             << ", given is: " << param << REND;
      param_is_valid = false;
    }
  } else if (method == 1) {
    if (param > max) {
      LERROR << "===Error Parameter=== " << name
             << " should not exceed upper boundary: " << max
             << ", given is: " << param << REND;
      param_is_valid = false;
    }
  } else if (method == 2) {
    if (param <= min) {
      LERROR << "===Error Parameter=== " << name
             << " should not exceed lower boundary: " << min
             << ", given is: " << param << REND;
      param_is_valid = false;
    }
  } else
    LERROR << "===Error method=== " << REND;

  // do something
  if (!param_is_valid) {
    exit(EXIT_FAILURE);
  }
}