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
#include <array>
#include <Eigen/Geometry>
#include "data_type.hpp"

template <typename T>
inline Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 3, 1> eulerAnglesZYX(T q_in)
{
  typedef typename std::remove_reference<T>::type::Scalar Scalar;

  Eigen::Matrix<Scalar, 4, 1> q = q_in.normalized().coeffs();

  Scalar s = -2 * (q(0) * q(2) - q(3) * q(1));
  if (s > 1)
    s = 1;
  return (Eigen::Matrix<Scalar, 3, 1>()
              << atan2f(2 * (q(0) * q(1) + q(3) * q(2)), q(3) * q(3) + q(0) * q(0) - q(1) * q(1) - q(2) * q(2)),
          asin(s), atan2(2 * (q(1) * q(2) + q(3) * q(0)), q(3) * q(3) - q(0) * q(0) - q(1) * q(1) + q(2) * q(2)))
      .finished();
};

/**
* Convert rpy euler angles to an rotation vector
* Input values are in radians
*/
inline std::array<double, 3> rpy2rotvec(double roll, double pitch, double yaw)
{
  Eigen::Vector3d roll_pitch_yaw(roll, pitch, yaw);
  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  auto rot = q.vec(); /* (q.x, q.y, q.z) */
  double angle = 2 * std::acos(q.w());
  Eigen::Vector3d rot_vec = rot.normalized() * angle;
  return {rot_vec(0), rot_vec(1), rot_vec(2)};
}

/* TODO:  Check whether the translation is before or after the rotation */  /* ??? what does this comment mean? */
inline Eigen::Matrix4d eulerAngleToTransMatrix(const double& x, const double& y, const double& z,
                                                              const double& roll, const double& pitch,
                                                              const double& yaw)
{
  Eigen::Translation3d translation(x, y, z);
  Eigen::AngleAxisd angleaxis_roll(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd angleaxis_pitch(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd angleaxis_yaw(yaw, Eigen::Vector3d::UnitZ());
  return (translation * angleaxis_yaw * angleaxis_pitch * angleaxis_roll).matrix();
}

inline Eigen::Matrix4d arrayToTransMatrix(const std::array<double, 6>& arr)
{
  return eulerAngleToTransMatrix(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5]);
}

inline std::array<double, 6> mat4dToArray(const Eigen::Matrix4d& mat)
{
  std::array<double, 6> ret;
  ret[0] = mat(0,3);
  ret[1] = mat(1,3);
  ret[2] = mat(2,3);
  
  Eigen::Quaterniond q(mat.block<3,3>(0,0));
  auto ypr = eulerAnglesZYX(q);
  ret[3] = ypr[2];
  ret[4] = ypr[1];
  ret[5] = ypr[0];

  return ret;
}

inline std::array<double, 3> mat3dToArray(const Eigen::Matrix3d& mat)
{
  std::array<double, 3> ret;
  Eigen::Quaterniond q(mat.block<3,3>(0,0));
  auto ypr = eulerAnglesZYX(q);
  ret[0] = ypr[2];
  ret[1] = ypr[1];
  ret[2] = ypr[0];

  return ret;
}

inline Eigen::Matrix3d eulerAngleToRotMat(const double& roll, const double& pitch, const double& yaw)
{
  Eigen::AngleAxisd angleaxis_roll(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd angleaxis_pitch(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd angleaxis_yaw(yaw, Eigen::Vector3d::UnitZ());
  return (angleaxis_yaw * angleaxis_pitch * angleaxis_roll).matrix();
}

inline Eigen::Matrix3d arrayToTransMatrix(const std::array<double, 3>& arr)
{
  return eulerAngleToRotMat(arr[0], arr[1], arr[2]);
}

inline std::array<double, 6> poseToArray(const Pose& pose)
{
  std::array<double, 6> ret;
  ret[0] = pose.xyz.x();
  ret[1] = pose.xyz.y();
  ret[2] = pose.xyz.z();

  auto ypr = eulerAnglesZYX(pose.q);
  ret[3] = ypr[2];
  ret[4] = ypr[1];
  ret[5] = ypr[0];
  return ret;
}

inline Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond& q)
{
  double roll, pitch, yaw;
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
  pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
  pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
  return Eigen::Vector3d(roll, pitch, yaw);
}