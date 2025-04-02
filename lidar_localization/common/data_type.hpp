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
#ifndef COMMON_DATA_TYPE_H
#define COMMON_DATA_TYPE_H
#include <fstream>
#include <iomanip>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

enum class STATUS /* status aft receive a lidar frame */
{
  IDLE = 0,
  LOW_ACCURACY = 1,
  NORMAL = 2,
  LOST = 3,
  NO_ENOUGH_MAP = 4,
  LOW_ACCURACY_RPZ = 5,
};

struct Pose {
  std::string source = "lidar";
  std::string status = "Null";
  STATUS status_code = STATUS::IDLE;
  double timestamp = 0; /* lidar timestamp, s */

  double enu_heading = 0;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Pose() = default;
  Pose(const Eigen::Matrix4d &T, double _timestamp) {
    xyz = T.block<3, 1>(0, 3);
    q = Eigen::Quaterniond(T.block<3, 3>(0, 0));
    q.normalize();
    timestamp = _timestamp;
  }

  void setStatus(STATUS _status) {
    status_code = _status;
    switch (status_code) {
    case STATUS::IDLE:
      status = "IDLE";
    case STATUS::LOW_ACCURACY:
      status = "LOW_ACCURACY";
    case STATUS::LOST:
      status = "LOST";
    case STATUS::NORMAL:
      status = "NORMAL";
    case STATUS::NO_ENOUGH_MAP:
      status = "NO_ENOUGH_MAP";
    case STATUS::LOW_ACCURACY_RPZ:
      status = "LOW_ACCURACY_RPZ";
    }
  }

  void print(const std::string &prefix = "") const {
    std::cout << "------------------" << prefix << "------------------"
              << std::endl;
    std::cout << "timestamp : " << std::fixed << timestamp << std::endl;
    std::cout << "xyz       : " << xyz.transpose() << std::endl;
    std::cout << "q         : " << q.coeffs().transpose() << std::endl;
    std::cout << "--------------------------------------------------"
              << std::endl;
  }

  Eigen::Matrix4d transform() const {
    Eigen::Affine3d T = Eigen::Translation3d(xyz.x(), xyz.y(), xyz.z()) * q;
    return T.matrix();
  }

  void updatePose(Pose &delta_pose) {
    q = delta_pose.q * q;
    xyz = delta_pose.q * xyz + delta_pose.xyz;
  }

  void updatePoseRight(Pose &delta_pose) {
    xyz = q * delta_pose.xyz + xyz;
    q = q * delta_pose.q;
  }

  Pose inverse() {
    Pose pose_inv;
    pose_inv.q = q.inverse();
    pose_inv.xyz = -(pose_inv.q * xyz);
    pose_inv.timestamp = timestamp;
    pose_inv.source = source;
    return pose_inv;
  }

  void savePose(std::string path) {
    std::ofstream ofs;
    ofs.open(path, std::ios::out | std::ios::app);

    ofs << std::fixed << std::setprecision(6) << timestamp << " " << xyz.x()
        << " " << xyz.y() << " " << xyz.z() << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << "\n";
    ofs.close();
  }
};

struct Velocity {
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();  /* vel x y z */
  Eigen::Vector3d angular = Eigen::Vector3d::Zero(); /* vel roll, pitch, yaw */
};

struct RsPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    RsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        std::uint16_t, ring, ring)(double, timestamp, timestamp))

typedef pcl::PointXYZRGB MapPointT;
typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
PCL_INSTANTIATE_PointCloud(PointT)

    inline void RSCloud2PCLCloud(
        const pcl::PointCloud<RsPointXYZIRT>::Ptr &rs_cloud,
        pcl::PointCloud<PointT>::Ptr &pcl_cloud) {
  pcl_cloud->header = rs_cloud->header;
  for (size_t i = 0; i < rs_cloud->size(); ++i) {

    PointT added_pt;

    added_pt.x = rs_cloud->points[i].x;
    added_pt.y = rs_cloud->points[i].y;
    added_pt.z = rs_cloud->points[i].z;
    // added_pt.intensity = rs_cloud->points[i].intensity;
    pcl_cloud->points.emplace_back(added_pt);
  }
}

struct Target {
 public:
  // get from map
  double x;
  double y;
  double z;
  Eigen::Vector3d pos;
  Eigen::Vector3d normal;
  PointCloudT::Ptr normal_pointcloud;
  // estimated
  Eigen::Vector3d estimated_normal;
  PointCloudT::Ptr estimated_normal_pointcloud;

 public:
  Target() : x(0), y(0), z(0) {}
  Target(const double& _x, const double& _y, const double& _z)
      : x(_x), y(_y), z(_z) {
    pos = Eigen::Vector3d(x, y, z);
  }
  Target(const Eigen::Vector3d& _target)
      : x(_target.x()), y(_target.y()), z(_target.z()) {
    pos = Eigen::Vector3d(x, y, z);
  }
  void setNormal(const Eigen::Vector3d& _normal) {
    normal = _normal.normalized();
  }
  void setEstimatedNormal(const Eigen::Vector3d& _normal) {
    estimated_normal = _normal.normalized();
  }
  // void setCurvature(const float& _curvature)
  // {
  //   curvature = _curvature;
  // }

  void setNormalCloud(int intensity = 255, int num = 20, double step = 0.05) {
    normal_pointcloud.reset(new PointCloudT);
    PointT p_tmp;
    double t = 0;
    for (int i = 0; i < num; i++) {
      p_tmp.x = x + normal.x() * t;
      p_tmp.y = y + normal.y() * t;
      p_tmp.z = z + normal.z() * t;
      p_tmp.intensity = intensity;

      normal_pointcloud->points.push_back(p_tmp);
      t += step;
    }
  }

  void setEstimatedNormalCloud(int intensity = 255, int num = 20,
                               double step = 0.05) {
    estimated_normal_pointcloud.reset(new PointCloudT);
    PointT p_tmp;
    double t = 0;
    for (int i = 0; i < num; i++) {
      p_tmp.x = x + estimated_normal.x() * t;
      p_tmp.y = y + estimated_normal.y() * t;
      p_tmp.z = z + estimated_normal.z() * t;
      p_tmp.intensity = intensity;

      estimated_normal_pointcloud->points.push_back(p_tmp);
      t += step;
    }
  }
};
#endif