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

#ifndef LIDAR_MATCHER_H
#define LIDAR_MATCHER_H

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <omp.h>

#include "data_type.hpp"
#include "localization.hpp"
#include "utility.hpp"
#include "yaml_reader.hpp"
#ifndef USE_EIGEN_OPTIMIZATION
#include "nonlinear_solver.hpp"
#else 
#include "eigen_nonlinear_solver.hpp"
#endif

enum class LidarMatchingStatus {
  kNone = 0,
  kInit = 1,
  kOngoing = 2,
  kCompleteSuccess = 3,
  kCompleteError = 4
};

enum class LidarMatchingDetail {
  kNone = 0,
  kSuccess,
  // input error
  kNoEnoughSourcePoints, // low lidar cloud size (<100)
  kNoEnoughMapPoints,    // low map cloud size (<80000)
  kInvalidPose,          // TODO
  // knn searching error
  kNoEnoughKNNPoints, // TODO
  kNoEnoughPointPair, // low valid pair size (<500)
  // solver error
  kSolveFailed,
  kNotConverged,
};

/* this is the base class for lidar matchers, defining interfaces*/
class LidarMatcher {
public:
  LidarMatcher() = default;
  LidarMatcher(const YAML::Node &config_node){config_node_ = config_node;};
  virtual ~LidarMatcher() = default;

  virtual bool align(const PointCloudT::Ptr &source_cloud,
                     const pcl::KdTreeFLANN<PointT>::Ptr kd_tree,
                     const PointCloudT::Ptr &map_cloud, const Pose &init_pose,
                     Pose &result_pose,double &lidar_time) = 0;

  AlignInfo getAlignInfo()                       
  {
    AlignInfo align_info_copy = align_info_;
    align_info_ = AlignInfo();
    return align_info_copy;
  }

  void setLidarMatchingStatus(LidarMatchingStatus set_value) {
    matching_status_ = set_value;
  }
  void setLidarMatchingDetail(LidarMatchingDetail set_value) {
    matching_detail_ = set_value;
  }

  void addCloudInfoToVec(std::vector<int> &src_cloud_id_ve,
                         PointCloudT::Ptr &tgt_cloud_info_vec, uint &insert_loc,
                         const int &src_point_id, const PointT &tgt_point,
                         const Eigen::Vector3d &tgt_normal);
  void allocate() {
    source_cloud_.reset(new PointCloudT());
    ground_source_cloud_.reset(new PointCloudT());
    aligned_source_cloud_.reset(new PointCloudT());
    target_cloud_info_.reset(new PointCloudT());
  }

  void reset() {
    total_delta_pose_.q.setIdentity();
    total_delta_pose_.xyz.setZero();
    delta_pose_.q.setIdentity();
    delta_pose_.xyz.setZero();
  }

protected:
  bool validation();
  void searchPairLoopOMP();
  bool loadConfig(const YAML::Node &config_node);
  bool preprocess(const PointCloudT::Ptr &source_cloud,
                  const pcl::KdTreeFLANN<PointT>::Ptr kd_tree,
                  const Pose &init_pose);
  double valid_threshold_;
  YAML::Node config_node_;
    // cloud
  PointCloudT::Ptr source_cloud_;
  PointCloudT::Ptr ground_source_cloud_;
  PointCloudT::Ptr aligned_source_cloud_; // aligned to target cloud
  PointCloudT::Ptr target_cloud_info_;    // store point
  std::vector<int> valid_source_id_vec_; // store point id (within the cloud)
  pcl::KdTreeFLANN<PointT>::Ptr kd_tree_;
  PointCloudT::Ptr map_cloud_;
  // input (init) pose
  Pose lidar_pose_;
  // temp pose
  Pose delta_pose_;
  Pose total_delta_pose_;
  // status code
  LidarMatchingStatus matching_status_ = LidarMatchingStatus::kNone;
  LidarMatchingDetail matching_detail_ = LidarMatchingDetail::kNone;
  /// config variables
  int min_map_size_ = 80000;
  int min_pair_size_ = 500;
  double leaf_size_ = 0.5;
  double z_leaf_size_ = 0.5;
  int point_downsample_num_ = 5;
  double blind_distance_ = 3;
  double blind_distance_max_ = 100;
  double valid_residual_thresh_ = 0.13;
  // check matching validity
  double checking_valid_ratio_thresh_ = 0.3;
  // check matching convergence
  double TF_angle_delta_thresh_ = 0.2;
  double ave_final_cost_thresh_ = 0.02;
  int iterations_ = 10;
  // estimatePlane
  int plane_points_num_ = 1;
  // solver options
  double loss_para_ = 0.1;
  bool minimizer_progress_to_stdout_ = false;
  int solver_num_threads_ = 4;
  int solver_max_num_iterations_ = 10;
  double initial_trust_region_radius_ = 1e4;
  double function_tolerance_ = 1e-6;
  // limit delta lidar pose
  bool check_delta_pose_ = true;
  double max_yaw_speed_per_sec_ = 50; // deg/s
  double max_xy_speed_per_sec_ = 30;  // m/s

  AlignInfo align_info_;

  // feature
  int min_point_num_ = 3;
  double min_angle_thresh_ = 8.; // deg


  int max_pair_size_{800};
};

#ifndef USE_EIGEN_OPTIMIZATION
class LidarMatcherCeres : public LidarMatcher {
public:
  LidarMatcherCeres() = default;
  LidarMatcherCeres(const YAML::Node &config_node)
      : LidarMatcher(config_node) {
    allocate();
    loadConfig(config_node_);
    setLidarMatchingStatus(LidarMatchingStatus::kInit);
    setSolverOptions();
  };
  virtual ~LidarMatcherCeres() = default;
  
  bool align(const PointCloudT::Ptr &source_cloud,
             const pcl::KdTreeFLANN<PointT>::Ptr kd_tree,
             const PointCloudT::Ptr &map_cloud, const Pose &init_pose,
             Pose &result_pose,double &lidar_time) override;

private:
  // iteration loop
  bool addResidualBlocksLoop(const std::shared_ptr<CeresSolver<7>> &solver);
  bool solve(const std::shared_ptr<CeresSolver<7>> &solver);
  void updateTempResult(const std::shared_ptr<CeresSolver<7>> &solver,
                        const int &iter_times);
  void updateFinalResult(const std::shared_ptr<CeresSolver<7>> &solver,
                         const Pose &init_pose, Pose &result_pose);
  /// chore functions
  void setSolverOptions() {
    ceres_op_.minimizer_progress_to_stdout = minimizer_progress_to_stdout_;
    ceres_op_.linear_solver_type = ceres::DENSE_QR;
    ceres_op_.max_num_iterations = solver_max_num_iterations_;
    ceres_op_.num_threads = solver_num_threads_;
    ceres_op_.initial_trust_region_radius = initial_trust_region_radius_;
    ceres_op_.function_tolerance = function_tolerance_; 
    ceres_op_.gradient_tolerance = 1e-4 * ceres_op_.function_tolerance;
  } 


private:
  // solver
  ceres::Solver::Options ceres_op_;
};
#else
class LidarMatcherEigen : public LidarMatcher {
public:
  LidarMatcherEigen() = default;
  LidarMatcherEigen(const YAML::Node &config_node)
      : LidarMatcher(config_node) {
    allocate();
    loadConfig(config_node_);
    setLidarMatchingStatus(LidarMatchingStatus::kInit);
  };
  virtual ~LidarMatcherEigen() = default;
  bool align(const PointCloudT::Ptr &source_cloud,
             const pcl::KdTreeFLANN<PointT>::Ptr kd_tree,
             const PointCloudT::Ptr &map_cloud, const Pose &init_pose,
             Pose &result_pose,double &lidar_time) override;

private:
  //// Eigen optimization
  bool addResidualBlocksLoopEigen(const std::shared_ptr<EigenSolver<7>> &solver);
  bool solveEigen(const std::shared_ptr<EigenSolver<7>> &solver);
  void updateTempResultEigen(const std::shared_ptr<EigenSolver<7>> &solver,const int &iter_times);
  void updateFinalResultEigen(const std::shared_ptr<EigenSolver<7>> &solver,const Pose &init_pose, Pose &result_pose);
};
#endif

#endif // LIDAR_MATCHER_H