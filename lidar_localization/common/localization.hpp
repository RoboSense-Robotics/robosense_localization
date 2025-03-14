#pragma once
#include <Eigen/Dense>
#include <pcl/common/transforms.h>

#include "data_type.hpp"

#define RSRESET "\033[0m"
#define RSBOLDRED "\033[1m\033[31m"     /* Bold Red */
#define RSBOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define RSBOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define RSBOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define RSBOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define LINFO (std::cout << RSBOLDGREEN)
#define LWARNING (std::cout << RSBOLDYELLOW)
#define LERROR (std::cout << RSBOLDRED)
#define LDEBUG (std::cout << RSBOLDCYAN)
#define LTITLE (std::cout << RSBOLDMAGENTA)
#define END (std::endl)
#define REND "\033[0m" << std::endl
#define GETBIT(X, Y) ((X) >> (Y)&1)
#define NAME(x) (#x)

const double PI = M_PI;
const double D2R = 0.01745329;
const double R2D = 57.295779515;

struct AlignInfo
{
  // preprocess
  int origin_source_size = -1;
  int gnd_size = -1;
  int non_gnd_size = -1;
  int scan_non_gnd_size = -1;
  int target_cloud_size = -1;
  int target_gnd_size = -1;
  int target_non_gnd_size = -1;

  int iter_times = 0;

  // recall = valid / total
  // matching
  uint valid_pair_num = 0;
  double valid_pair_ratio = 0;
  // checking
  uint valid_gnd_num = 0;
  uint valid_non_gnd_num = 0;
  double ceres_valid_pair_ratio = 0;
  double ceres_valid_gnd_pair_ratio = 0;
  double ceres_valid_non_gnd_pair_ratio = 0;
  double ceres_valid_scan_non_gnd_pair_ratio = 0;
  std::vector<int> align_source_indices_;
  std::vector<int> align_source_rel_indices_;
  std::vector<int> valid_source_indices_;
  std::vector<double> valid_source_residuals_;

  int non_gnd_occupy_grid_num = 0;
  int gnd_occupy_grid_num = 0;

  // residual and cost
  double point_pair_distance_residual = -1;
  double gnd_point_pair_distance_residual = -1;
  double non_gnd_point_pair_distance_residual = -1;
  double scan_non_gnd_pair_distance_residual = -1;

  // ceres
  double init_cost = -1;
  double final_cost = -1;
  std::vector<double> residuals;
  int ceres_num_residuals = -1;
};

inline bool getVelFromPose(const Pose& pose_prev, const Pose& pose_next, Velocity& vel)
{
  double delta_t = pose_next.timestamp - pose_prev.timestamp;
  vel.linear = (pose_next.xyz - pose_prev.xyz) / delta_t;

  Eigen::Matrix3d R_prev = pose_prev.q.toRotationMatrix();
  Eigen::Matrix3d R_next = pose_next.q.toRotationMatrix();
  Eigen::Matrix3d R_delta = R_prev.transpose() * R_next;
  Eigen::AngleAxisd rot = Eigen::AngleAxisd().fromRotationMatrix(R_delta);
  Eigen::Vector3d rot_vec = rot.angle() * rot.axis();
  vel.angular = rot_vec / delta_t;
  return true;
}

inline bool getVelFromPoseQueue(const std::map<double, Pose>& queue, double t, Velocity& vel)
{
  if (queue.size() < 2)
  {
    std::cout << "WARN: cannot get vel from queue that is smaller than 2" << std::endl;
    return false;
  }

  auto rel_next = queue.lower_bound(t);
  decltype(rel_next) rel_prev;

  if (rel_next == queue.end()) /* t is newer than latest element in queue */
  {
    rel_next = std::prev(rel_next);
    rel_prev = std::prev(rel_next);
  }
  else if (rel_next == queue.begin()) /* t is older than the earliest element in queue */
  {
    rel_prev = rel_next;
    rel_next = std::next(rel_prev);
  }
  else
  {
    rel_prev = std::prev(rel_next);
  }

  return getVelFromPose(rel_prev->second, rel_next->second, vel);
}

inline Eigen::Matrix4d deltaTransform(const Velocity& local_vel, double delta_t)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Vector3d rot_vec = local_vel.angular * delta_t;
  Eigen::AngleAxisd delta_R(rot_vec.norm(), rot_vec.normalized());
  Eigen::Vector3d v2 = delta_R * local_vel.linear;
  T.block<3, 1>(0, 3) = (local_vel.linear + v2) * 0.5 * delta_t;
  T.block<3, 3>(0, 0) = delta_R.toRotationMatrix();
  return T;
}



inline bool interpolate(std::map<double, Pose>& queue, double t, Pose& pose_at_t)
{
  if (queue.size() < 2)
  {
    std::cout << "WARN: Interpolate queue size less than 2" << std::endl;
    return false;
  }

  const double TIME_DIFF_THRESH = 1e-3;
  auto rel_next = queue.lower_bound(t);
  std::string pose_source = rel_next->second.source;


  if (rel_next == queue.end()) /* t is newer than latest element in queue */
  {
    if (std::fabs(t - queue.rbegin()->first) < TIME_DIFF_THRESH)
    {
      pose_at_t = queue.rbegin()->second;
    }
    else /* interpolate outside right bound */
    {
      rel_next = std::prev(rel_next);
      auto rel_prev = std::prev(rel_next);

      Velocity vel;
      getVelFromPose(rel_prev->second, rel_next->second,
                     vel);  // vel in global frame
      Velocity local_vel;
      local_vel.linear = rel_next->second.q.inverse() * vel.linear;
      local_vel.angular = rel_next->second.q.inverse() * vel.angular;
      Eigen::Matrix4d T_at_t = rel_next->second.transform() * deltaTransform(local_vel, t - rel_next->first);
      pose_at_t = Pose(T_at_t, t);

    }
  }
  else if (rel_next == queue.begin()) /* t is earlier than the first element in queue */
  {
    if (std::fabs(t - queue.begin()->first) < TIME_DIFF_THRESH)
    {
      pose_at_t = queue.begin()->second;
    }
    else /* interpolate outside left bound */
    {
      auto rel_prev = rel_next;
      rel_next = std::next(rel_next);

      Velocity vel;
      getVelFromPose(rel_prev->second, rel_next->second, vel);
      Velocity local_vel;
      local_vel.linear = rel_prev->second.q.inverse() * vel.linear;
      local_vel.angular = rel_prev->second.q.inverse() * vel.angular;
      Eigen::Matrix4d T_at_t = rel_prev->second.transform() * deltaTransform(local_vel, rel_prev->first - t).inverse();
      pose_at_t = Pose(T_at_t, t);

    }
  }
  else if (std::fabs(t - rel_next->first) < TIME_DIFF_THRESH)
  {
    pose_at_t = rel_next->second;
    return true;
  }
  else /* interpolate */
  {
    auto rel_prev = std::prev(rel_next);

    Pose pose_fixed = rel_next->second;
    double ratio = (t - rel_prev->first) / (rel_next->first - rel_prev->first);
    pose_at_t.xyz = (1 - ratio) * pose_fixed.xyz + ratio * rel_next->second.xyz;
    pose_at_t.q = rel_prev->second.q.slerp(ratio, rel_next->second.q);
  }

  pose_at_t.source = rel_next->second.source;
  pose_at_t.timestamp = t;
  return true;
}