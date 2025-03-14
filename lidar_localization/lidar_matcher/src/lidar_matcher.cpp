#include "lidar_matcher.h"
#ifdef USE_ROS_MODE
#ifdef ROS1
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#endif

#ifdef ROS2
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#endif

#endif // only for DEBUGGING

/************ utility functions  *********************/
Pose vec2Pose(Eigen::Matrix<double, 7, 1> &&ret) {
  Pose pose;
  pose.xyz = Eigen::Vector3d(ret[0], ret[1], ret[2]);
  pose.q = Eigen::Quaterniond(ret[6], ret[3], ret[4], ret[5]);
  return pose;
}

bool LidarMatcher::preprocess(const PointCloudT::Ptr &origin_source_cloud,
                                   const pcl::KdTreeFLANN<PointT>::Ptr kd_tree,
                                   const Pose &init_pose) {
  align_info_.target_cloud_size = map_cloud_->points.size();
  lidar_pose_ = init_pose;
  // set global index, and mark current scan
  PointCloudT::Ptr organized_source_cloud(new PointCloudT);
  *organized_source_cloud = *origin_source_cloud;
  for (int i = 0; i < organized_source_cloud->points.size(); ++i) {
    auto &pt = organized_source_cloud->points[i];
    if (pt.curvature == 1) {
      pt.curvature = float(i) + 0.1; // current scan point
    } else {
      pt.curvature = i; // local_map point
    }
  }

  if (organized_source_cloud->points.size() < 800) // TODO //800
  {
    LERROR << "align : NO_ENOUGH_SOURCE_POINTS" << REND;
    setLidarMatchingDetail(LidarMatchingDetail::kNoEnoughSourcePoints);
    setLidarMatchingStatus(LidarMatchingStatus::kCompleteError);
    return false;
  }
  align_info_.origin_source_size = organized_source_cloud->points.size();
  pcl::transformPointCloud(*organized_source_cloud, *source_cloud_,
                           lidar_pose_.transform());

  return true;
}


inline int areVectorsParallel(const Eigen::Vector3d &v1,
                              const Eigen::Vector3d &v2, double threash = 10.) {
  // 使用点积判断
  double dotProduct = v1.dot(v2);
  double theta = acos(dotProduct / (v1.norm() * v2.norm())) * 180 / M_PI;
 
  if (abs(theta) < threash) {
    return 1; 
  }

  return 0; 
}

void LidarMatcher::searchPairLoopOMP() {  
  align_info_.valid_pair_num = 0;  
  int source_size = source_cloud_->points.size();  
  align_info_.align_source_indices_.clear();  
  align_info_.align_source_indices_.reserve(source_size);  
  align_info_.align_source_rel_indices_.clear();  
  align_info_.align_source_rel_indices_.reserve(source_size);  
  align_info_.non_gnd_size = 0;  
  align_info_.gnd_size = 0;  
  align_info_.scan_non_gnd_size = 0;  
  valid_source_id_vec_.clear();  
  valid_source_id_vec_.reserve(source_size);  
  target_cloud_info_->clear();
  target_cloud_info_->points.resize(source_size);
  // Use OpenMP to parallelize the loop  
  omp_set_num_threads(4);
  std::vector<int> is_selected(source_size ,0);
  std::vector<PointT> neightbor(source_size); 
  std::vector<Eigen::Vector3d> normal(source_size);
  int pair_cnt = 0;
  #pragma omp parallel for
  for (int i = 0; i < source_size; i++) {  
    auto &source_p = source_cloud_->points[i];  
 
    std::vector<int> idx;  
    std::vector<float> srqdis;  
    kd_tree_->radiusSearch(source_p, 0.3, idx, srqdis,min_point_num_);  
    if (idx.size() < min_point_num_) {
      continue;
    }

    int is_valid = 1;
    for (int j = 0; j < idx.size() - 1; j++) {
      for (int k = j + 1; k < idx.size(); k++) {
        const auto &p1 = map_cloud_->points[idx[j]];
        const auto &p2 = map_cloud_->points[idx[k]];
        Eigen::Vector3d v1(p1.normal_x, p1.normal_y, p1.normal_z);
        Eigen::Vector3d v2(p2.normal_x, p2.normal_y, p2.normal_z);
        if (areVectorsParallel(v1, v2,min_angle_thresh_) == 0) {
          is_valid = 0;
          break;
        }
      }
      if (is_valid == 0) {
        break;
      }
    }
    if (is_valid == 0) {
      continue;
    }

    is_selected[i] = 1;
    auto nn_point = map_cloud_->points[idx[0]];
    neightbor[i] = nn_point;
    normal[i] = Eigen::Vector3d{nn_point.normal_x, nn_point.normal_y, nn_point.normal_z};
  }
 
  for (int i = 0; i < source_size; i++)
  { 
    auto &source_p = source_cloud_->points[i];
    if (is_selected[i] == 0)
      continue;

    auto &nn_point = neightbor[i];
    auto &estimated_normal = normal[i];
    addCloudInfoToVec(valid_source_id_vec_, target_cloud_info_,
                      align_info_.valid_pair_num, i, nn_point,
                      estimated_normal);
    // Record absolute and relative indices
    align_info_.align_source_indices_.push_back((int)source_cloud_->points[i].curvature);
    align_info_.align_source_rel_indices_.push_back(i);

    if (source_p.intensity == 2)
    {
      align_info_.non_gnd_size++;
      // Current scan non-ground point
      if (source_p.curvature != (int)source_p.curvature)
      {
        align_info_.scan_non_gnd_size++;
      }
    }
    else if (source_p.intensity < 0)
    {
      align_info_.gnd_size++;
    }
  }
}  

bool LidarMatcher::validation() {
  double TF_score = Eigen::AngleAxisd(delta_pose_.q).angle() * R2D;
  bool TF_is_valid = TF_score < TF_angle_delta_thresh_;
  bool final_cost_is_valid = align_info_.final_cost < ave_final_cost_thresh_;
  if (align_info_.iter_times == iterations_) {
    LWARNING << "validation : NOT_CONVERGED at max iterations" << REND;
  }
  return TF_is_valid && final_cost_is_valid;
}

// load config from yaml
bool LidarMatcher::loadConfig(const YAML::Node &config_node) {
  if (!config_node) {
    LERROR << "loadConfig : config_node is empty" << REND;
    return false;
  }
  // source cloud filter
  yamlRead<int>(config_node, "min_map_size", min_map_size_, 80000);
  yamlRead<int>(config_node, "min_pair_size", min_pair_size_, 500);
  yamlRead<int>(config_node, "max_pair_size", max_pair_size_, 800);
  yamlRead<double>(config_node, "leaf_size", leaf_size_, 0.5);
  yamlRead<double>(config_node, "z_leaf_size", z_leaf_size_, 0.5);
  yamlRead<int>(config_node, "point_downsample_num", point_downsample_num_, 8);
  yamlRead<double>(config_node, "blind_distance", blind_distance_, 10);
  yamlRead<double>(config_node, "blind_distance_max", blind_distance_max_, 100);
  // check matching convergence
  yamlRead<int>(config_node, "iterations", iterations_, 10);
  yamlRead<double>(config_node, "validation_TF_angle_delta_thresh",
                   TF_angle_delta_thresh_, 0.2);
  yamlRead<double>(config_node, "validation_ave_final_cost_thresh",
                   ave_final_cost_thresh_, 0.02);
  // estimatePlane
  yamlRead<int>(config_node, "plane_points_num", plane_points_num_, 1);
  // feature: del invalid pts
  yamlRead<int>(config_node, "min_point_num", min_point_num_, 3);
  yamlRead<double>(config_node, "min_angle", min_angle_thresh_, 8.0);

  // ceres solver options
  yamlRead<double>(config_node, "loss_para", loss_para_, 0.1);
  yamlRead<bool>(config_node, "minimizer_progress_to_stdout",
                 minimizer_progress_to_stdout_, false);
  yamlRead<int>(config_node, "num_threads", solver_num_threads_, 4);
  yamlRead<int>(config_node, "max_num_iterations", solver_max_num_iterations_,
                10);
  yamlRead<double>(config_node, "initial_trust_region_radius",
                   initial_trust_region_radius_, 1e4);
  yamlRead<double>(config_node, "function_tolerance", function_tolerance_,
                   1e-3);
  // output limits
  yamlRead<double>(config_node, "max_yaw_speed_per_sec", max_yaw_speed_per_sec_,
                   30); ////50
  yamlRead<double>(config_node, "max_xy_speed_per_sec", max_xy_speed_per_sec_,
                   6); ////30

  // fool proofing, check validation
  checkParamValidity(min_map_size_, NAME(min_map_size_), 0, 0, 150000);
  checkParamValidity(min_pair_size_, NAME(min_pair_size_), 0, 0, 3000);
  checkParamValidity(leaf_size_, NAME(leaf_size_), 0, 0.001, 5.0);
  checkParamValidity(blind_distance_, NAME(blind_distance_), 0, 0.0, 50.0);
  checkParamValidity(blind_distance_max_, NAME(blind_distance_max_), 0, 50.0,
                     200.0);
  checkParamValidity(iterations_, NAME(iterations_), 0, 3, 30);
  checkParamValidity(TF_angle_delta_thresh_, NAME(TF_angle_delta_thresh_), 0,
                     0.0, 10.0);
  checkParamValidity(ave_final_cost_thresh_, NAME(ave_final_cost_thresh_), 0,
                     0.0, 10.0);
  checkParamValidity(plane_points_num_, NAME(plane_points_num_), 0, 0, 100);

  return true;
}

/** @brief save cloud info (pos and normal) of valid pair into certain
 * position of a fixed length cloud, so as to avoid use omp critical
 */
void LidarMatcher::addCloudInfoToVec(std::vector<int> &src_cloud_id_vec,
                                          PointCloudT::Ptr &tgt_cloud_info_vec,
                                          uint &insert_loc,
                                          const int &src_point_id,
                                          const PointT &tgt_point,
                                          const Eigen::Vector3d &tgt_normal) {
  src_cloud_id_vec.emplace_back(src_point_id); // = src_point_id;

  tgt_cloud_info_vec->points[insert_loc].x = tgt_point.x;
  tgt_cloud_info_vec->points[insert_loc].y = tgt_point.y;
  tgt_cloud_info_vec->points[insert_loc].z = tgt_point.z;
  tgt_cloud_info_vec->points[insert_loc].normal_x = tgt_normal.x();
  tgt_cloud_info_vec->points[insert_loc].normal_y = tgt_normal.y();
  tgt_cloud_info_vec->points[insert_loc].normal_z = tgt_normal.z();
  ++insert_loc;
}

#ifndef USE_EIGEN_OPTIMIZATION
bool LidarMatcherCeres::align(const PointCloudT::Ptr &source_cloud,
                              const pcl::KdTreeFLANN<PointT>::Ptr kd_tree,
                              const PointCloudT::Ptr &map_cloud,
                              const Pose &init_pose, Pose &result_pose,double &lidar_time) {
  map_cloud_ = map_cloud;
  kd_tree_ = kd_tree;
  int iter_cnts = 0;
  double ceres_opt_times = 0.0;
  if (!preprocess(source_cloud, kd_tree_, init_pose)) {
    return false;
  }
  for (int iter_times = 0; iter_times < iterations_; ++iter_times) {
    setLidarMatchingStatus(LidarMatchingStatus::kOngoing);
    align_info_.iter_times = iter_times + 1;
    searchPairLoopOMP();
    // add valid point pair to solver
    RSTicToc time_c1{"ceres solver"};
    time_c1.tic();
    std::shared_ptr<CeresSolver<7>> solver(
        new SolvePoseFromPointToPlaneCeresSophus());
    if (!addResidualBlocksLoop(solver)) {
      return false;
    }

    if (!solve(solver)) {
      return false;
    }
    time_c1.toc();

    iter_cnts = iter_times+1;
    ceres_opt_times +=time_c1.getTime();
    updateTempResult(solver, iter_times);

    if (validation()) {
      updateFinalResult(solver, init_pose, result_pose);
      setLidarMatchingStatus(LidarMatchingStatus::kCompleteSuccess);
      break;
    } else 
    {
      if (align_info_.iter_times == iterations_) {
        updateFinalResult(solver, init_pose, result_pose);
      }
      setLidarMatchingDetail(LidarMatchingDetail::kNotConverged);
    }
  }
  std::cout << std::fixed << lidar_time << "," << iter_cnts << "," << ceres_opt_times << "," <<result_pose.xyz.transpose() << "," << result_pose.q.coeffs().transpose() <<std::endl;
  iter_cnts = 0;
  ceres_opt_times = 0.0;
  
  reset();
  result_pose.timestamp = init_pose.timestamp;
  return true;
}

bool LidarMatcherCeres::addResidualBlocksLoop(
    const std::shared_ptr<CeresSolver<7>> &solver) {
  std::map<double, std::pair<Target, Target>> valid_pair_map;

  for (int i = 0; i < align_info_.valid_pair_num; i++) {
    auto &source_p = source_cloud_->points[valid_source_id_vec_[i]];
    if (std::isnan(source_p.x) || std::isnan(source_p.y) ||
        std::isnan(source_p.z)) {
      continue;
    }
    auto &target_p = target_cloud_info_->points[i];
    if (std::isnan(target_p.x) || std::isnan(target_p.y) ||
        std::isnan(target_p.z)) {
      continue;
    }
    if (std::isnan(target_p.normal_x) || std::isnan(target_p.normal_x) ||
        std::isnan(target_p.normal_z)) {
      continue;
    }
    Target source_t(source_p.x, source_p.y, source_p.z);
    Target target_t(target_p.x, target_p.y, target_p.z);
    target_t.setNormal(Eigen::Vector3d{target_p.normal_x, target_p.normal_y,
                                       target_p.normal_z});

    // 常规方法，不排序
    solver->addData(source_t, target_t);
  }

  align_info_.valid_pair_ratio =
      (double)align_info_.valid_pair_num / source_cloud_->points.size();

  std::cout << "valid_pair_num: " << align_info_.valid_pair_num << std::endl;

  if (align_info_.valid_pair_num < min_pair_size_) {
    LERROR << "align : NO_ENOUGH_POINT_PAIR: " << align_info_.valid_pair_num
           << REND;
    setLidarMatchingDetail(LidarMatchingDetail::kNoEnoughPointPair);
    setLidarMatchingStatus(LidarMatchingStatus::kCompleteError);
    return false;
  }
  return true;
}

bool LidarMatcherCeres::solve(const std::shared_ptr<CeresSolver<7>> &solver) {
  if (!solver->solve(ceres_op_)) {
    LERROR << "align : SOLVE_FAILED" << REND;
    setLidarMatchingDetail(LidarMatchingDetail::kSolveFailed);
    setLidarMatchingStatus(LidarMatchingStatus::kCompleteError);
    return false;
  } else {
    // TODO large solving result
    if (0) {
      setLidarMatchingDetail(LidarMatchingDetail::kSolveFailed);
      setLidarMatchingStatus(LidarMatchingStatus::kCompleteError);
      return false;
    }
    return true;
  }
}

void LidarMatcherCeres::updateTempResult(
    const std::shared_ptr<CeresSolver<7>> &solver, const int &iter_times) {
  auto summary = solver->getSummary();
  if (iter_times == 0) {
    align_info_.init_cost = summary.initial_cost / align_info_.valid_pair_num;
  }

  delta_pose_ = vec2Pose(solver->getResult());
  total_delta_pose_.updatePose(delta_pose_);

  // update init pose and source cloud
  pcl::transformPointCloud(*source_cloud_, *source_cloud_,
                           delta_pose_.transform());
  aligned_source_cloud_ = source_cloud_;
}

void LidarMatcherCeres::updateFinalResult(
    const std::shared_ptr<CeresSolver<7>> &solver, const Pose &init_pose,
    Pose &result_pose) {
  // 更新位姿
  lidar_pose_.updatePose(total_delta_pose_); 
  static double last_lidar_pose_time_ = init_pose.timestamp;
  static bool first_frame_ = true;

  double time_diff = init_pose.timestamp - last_lidar_pose_time_ + 1e-6;
  if (first_frame_) {
    time_diff = 0.1;
    first_frame_ = false;
  }

  last_lidar_pose_time_ = init_pose.timestamp;

  if (time_diff < -0.15) {
    LWARNING << "updateFinalResult: discontinuous scans, delta Lidar time "
             << time_diff << "s" << REND;
  }

  result_pose = lidar_pose_;
  // result_pose.print("result_pose");

  if (check_delta_pose_) {
    double total_delta_angle =
        Eigen::AngleAxisd(total_delta_pose_.q).angle() * R2D;
    bool yaw_condition =
        abs(total_delta_angle) / time_diff > max_yaw_speed_per_sec_;
    double total_delta_xy = total_delta_pose_.xyz.norm();
    bool xy_condition = abs(total_delta_xy) / time_diff > max_xy_speed_per_sec_;

    if (yaw_condition || xy_condition) {
      LERROR << "updateFinalResult: Large delta Lidar pose, delta yaw: "
             << total_delta_angle << " , delta xy dis: " << total_delta_xy
             << REND;
      result_pose = init_pose;

      align_info_.ceres_valid_pair_ratio = 0.0;
      align_info_.point_pair_distance_residual = 1.0;

      return;
    }
  }

  // 计算重叠率打分
  solver->getResidual(align_info_.final_cost, &align_info_.residuals);
  align_info_.final_cost /= align_info_.valid_pair_num;

  int total_num = align_info_.residuals.size();
  int valid_num = total_num;
  int valid_gnd_num = 0;
  int valid_non_gnd_num = 0;
  int valid_scan_non_gnd_num = 0;
  if (total_num) {
    for (int i = 0; i < total_num; ++i) {
      auto &valid_pt = source_cloud_->points[valid_source_id_vec_[i]];
      bool is_gnd_point = valid_pt.intensity < 0;
      bool is_non_gnd_point = valid_pt.intensity == 2;
      bool is_scan_non_gnd = valid_pt.curvature != (int)valid_pt.curvature;

      // 有效点
      if (align_info_.residuals[i] > valid_residual_thresh_ ||
          align_info_.residuals[i] < -valid_residual_thresh_) {
        valid_num--;
      } else {
        if (is_gnd_point) {
          ++valid_gnd_num;
        } else if (is_non_gnd_point) {
          ++valid_non_gnd_num;
          if (is_scan_non_gnd) {
            ++valid_scan_non_gnd_num;
          }
        }
      }

      // 统计残差
      double abs_res = fabs(align_info_.residuals[i]);
      if (abs_res < 1.7 * valid_residual_thresh_) {
        align_info_.valid_source_indices_.push_back(
            align_info_.align_source_indices_[i]);
        align_info_.valid_source_residuals_.push_back(abs_res); // for showing
      }
      align_info_.point_pair_distance_residual += abs_res;
      valid_pt.normal_x = abs_res;

      if (is_gnd_point) {
        align_info_.gnd_point_pair_distance_residual += abs_res;
      } else if (is_non_gnd_point) {
        align_info_.non_gnd_point_pair_distance_residual += abs_res;
        if (is_scan_non_gnd) {
          align_info_.scan_non_gnd_pair_distance_residual += abs_res;
        }
      }
    }

    // 通过有效点计算重叠率
    // 所有点重叠率
    align_info_.ceres_valid_pair_ratio = (double)valid_num / total_num;
    align_info_.point_pair_distance_residual /= total_num;
    // 地面重叠率
    if (align_info_.gnd_size) {
      align_info_.ceres_valid_gnd_pair_ratio =
          (double)valid_gnd_num / align_info_.gnd_size;
      align_info_.gnd_point_pair_distance_residual /= align_info_.gnd_size;
    }
    // 非地面重叠率
    align_info_.ceres_valid_non_gnd_pair_ratio =
        (double)valid_non_gnd_num / align_info_.non_gnd_size;
    align_info_.non_gnd_point_pair_distance_residual /=
        align_info_.non_gnd_size;
    // 雷达原始scan非地面重叠率
    if (align_info_.scan_non_gnd_size) {
      align_info_.ceres_valid_scan_non_gnd_pair_ratio =
          (double)valid_scan_non_gnd_num / align_info_.scan_non_gnd_size;
      align_info_.scan_non_gnd_pair_distance_residual /=
          align_info_.scan_non_gnd_size;
    }
  }
}
#else
//********************Eigen solver***********************//
bool LidarMatcherEigen::align(const PointCloudT::Ptr &source_cloud,
                              const pcl::KdTreeFLANN<PointT>::Ptr kd_tree,
                              const PointCloudT::Ptr &map_cloud,
                              const Pose &init_pose, Pose &result_pose,double &lidar_time) {
  map_cloud_ = map_cloud;
  kd_tree_ = kd_tree;
  int iter_cnts = 0;
  double eigen_opt_times = 0.0;
  if (!preprocess(source_cloud, kd_tree_, init_pose)) {
    return false;
  }
  for (int iter_times = 0; iter_times < iterations_; ++iter_times) {
    setLidarMatchingStatus(LidarMatchingStatus::kOngoing);
    align_info_.iter_times = iter_times + 1;
    searchPairLoopOMP();
    // add valid point pair to solver
    RSTicToc time_c1{"eigen solver"};
    time_c1.tic();
    std::shared_ptr<EigenSolver<7>> solver(new SolvePoseFromPointToPlaneEigen());
    if (!addResidualBlocksLoopEigen(solver)) {
      return false;
    }

    if (!solveEigen(solver)) {
      return false;
    }
    time_c1.toc();

    iter_cnts = iter_times+1;
    eigen_opt_times +=time_c1.getTime();
    updateTempResultEigen(solver, iter_times);

    if (validation()) {
      updateFinalResultEigen(solver, init_pose, result_pose);
      setLidarMatchingStatus(LidarMatchingStatus::kCompleteSuccess);
      break;
    } else 
    {
      if (align_info_.iter_times == iterations_) {
        updateFinalResultEigen(solver, init_pose, result_pose);
      }
      setLidarMatchingDetail(LidarMatchingDetail::kNotConverged);
    }
  }
  std::cout << std::fixed << lidar_time << "," << iter_cnts << "," << eigen_opt_times << "," <<result_pose.xyz.transpose() << "," << result_pose.q.coeffs().transpose() <<std::endl;
  iter_cnts = 0;
  eigen_opt_times = 0.0;
  
  reset();
  result_pose.timestamp = init_pose.timestamp;
  return true;
}

bool LidarMatcherEigen::addResidualBlocksLoopEigen(const std::shared_ptr<EigenSolver<7>> &solver) {
  std::map<double, std::pair<Target, Target>> valid_pair_map;

  for (int i = 0; i < align_info_.valid_pair_num; i++) {
    auto &source_p = source_cloud_->points[valid_source_id_vec_[i]];
    if (std::isnan(source_p.x) || std::isnan(source_p.y) ||
        std::isnan(source_p.z)) {
      continue;
    }
    auto &target_p = target_cloud_info_->points[i];
    if (std::isnan(target_p.x) || std::isnan(target_p.y) ||
        std::isnan(target_p.z)) {
      continue;
    }
    if (std::isnan(target_p.normal_x) || std::isnan(target_p.normal_x) ||
        std::isnan(target_p.normal_z)) {
      continue;
    }
    Target source_t(source_p.x, source_p.y, source_p.z);
    Target target_t(target_p.x, target_p.y, target_p.z);
    target_t.setNormal(Eigen::Vector3d{target_p.normal_x, target_p.normal_y,
                                       target_p.normal_z});

    solver->addData(source_t, target_t);
  }

  align_info_.valid_pair_ratio =
      (double)align_info_.valid_pair_num / source_cloud_->points.size();

  if (align_info_.valid_pair_num < min_pair_size_) {
    LERROR << "align : NO_ENOUGH_POINT_PAIR: " << align_info_.valid_pair_num
           << REND;
    setLidarMatchingDetail(LidarMatchingDetail::kNoEnoughPointPair);
    setLidarMatchingStatus(LidarMatchingStatus::kCompleteError);
    return false;
  }
  return true;
}

bool LidarMatcherEigen::solveEigen(const std::shared_ptr<EigenSolver<7>> &solver) {
  if (!solver->solve()) {
    LERROR << "align : SOLVE_FAILED" << REND;
    setLidarMatchingDetail(LidarMatchingDetail::kSolveFailed);
    setLidarMatchingStatus(LidarMatchingStatus::kCompleteError);
    return false;
  } 
    return true;
}

 void LidarMatcherEigen::updateTempResultEigen(const std::shared_ptr<EigenSolver<7>> &solver,const int &iter_times) {
  delta_pose_ = vec2Pose(solver->getResult());
  total_delta_pose_.updatePose(delta_pose_);

  // update init pose and source cloud
  pcl::transformPointCloud(*source_cloud_, *source_cloud_,
                           delta_pose_.transform());
  aligned_source_cloud_ = source_cloud_;
 }

 void LidarMatcherEigen::updateFinalResultEigen(const std::shared_ptr<EigenSolver<7>> &solver,const Pose &init_pose, Pose &result_pose) {
  lidar_pose_.updatePose(total_delta_pose_); // T_Wn_L = T_total * T_W0_L
  static double last_lidar_pose_time_ = init_pose.timestamp;
  static bool first_frame_ = true;

  double time_diff = init_pose.timestamp - last_lidar_pose_time_ + 1e-6;
  if (first_frame_) {
    time_diff = 0.1;
    first_frame_ = false;
  }

  last_lidar_pose_time_ = init_pose.timestamp;
  if (time_diff < -0.15) {
    LWARNING << "updateFinalResult: discontinuous scans, delta Lidar time "
             << time_diff << "s" << REND;
  }

  result_pose = lidar_pose_;

  if (check_delta_pose_) {
    double total_delta_angle =
        Eigen::AngleAxisd(total_delta_pose_.q).angle() * R2D;
    bool yaw_condition =
        abs(total_delta_angle) / time_diff > max_yaw_speed_per_sec_;
    double total_delta_xy = total_delta_pose_.xyz.norm();
    bool xy_condition = abs(total_delta_xy) / time_diff > max_xy_speed_per_sec_;

    if (yaw_condition || xy_condition) {
      LERROR << "updateFinalResult: Large delta Lidar pose, delta yaw: "
             << total_delta_angle << " , delta xy dis: " << total_delta_xy
             << REND;
      result_pose = init_pose;

      align_info_.ceres_valid_pair_ratio = 0.0;
      align_info_.point_pair_distance_residual = 1.0;

      return;
    }
  }

  // 计算重叠率打分
  solver->getResidual(align_info_.final_cost, &align_info_.residuals);
  align_info_.final_cost /= align_info_.valid_pair_num;

  int total_num = align_info_.residuals.size();
  int valid_num = total_num;

  if (total_num) {
    for (int i = 0; i < total_num; ++i) {
      auto &valid_pt = source_cloud_->points[valid_source_id_vec_[i]];

      // 有效点
      if (align_info_.residuals[i] > valid_residual_thresh_ ||
          align_info_.residuals[i] < -valid_residual_thresh_) {
        valid_num--;
      } 
      // 统计残差
      double abs_res = fabs(align_info_.residuals[i]);
      if (abs_res < 1.7 * valid_residual_thresh_) {
        align_info_.valid_source_indices_.push_back(
            align_info_.align_source_indices_[i]);
        align_info_.valid_source_residuals_.push_back(abs_res); // for showing
      }
      align_info_.point_pair_distance_residual += abs_res;
      valid_pt.normal_x = abs_res;

    }
    align_info_.ceres_valid_pair_ratio = (double)valid_num / total_num;
    align_info_.point_pair_distance_residual /= total_num;
  }
 }
#endif