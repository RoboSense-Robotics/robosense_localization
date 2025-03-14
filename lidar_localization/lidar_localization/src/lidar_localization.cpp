#include "lidar_localization.h"
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS1 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#endif

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#endif

LidarLocalization::LidarLocalization(const YAML::Node &config_node)
  : config_node_(config_node) {
  std::string lidar_topic = config_node_["lidar_topic"].as<std::string>();
  is_pub_cloud_ = config_node["is_pub_cloud"].as<bool>();
  input_cloud_size_thr_ = config_node["input_cloud_size_thr"].as<size_t>();
  blind_distance_ = config_node["lidar_matcher"]["blind_distance"].as<double>();

  std::vector<double> lidar_vehicle_xyz = config_node_["lidar_vehicle_xyz"].as<std::vector<double>>();
  std::vector<double> lidar_vehicle_rpy = config_node_["lidar_vehicle_rpy"].as<std::vector<double>>();
  auto rotation_lidar_vehicle_ = Eigen::AngleAxisd(lidar_vehicle_rpy[2], Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(lidar_vehicle_rpy[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(lidar_vehicle_rpy[0], Eigen::Vector3d::UnitX()).toRotationMatrix();
  T_base_lidar_.block<3, 3>(0, 0) = rotation_lidar_vehicle_;
  T_base_lidar_.block<3, 1>(0, 3) = Eigen::Vector3d(lidar_vehicle_xyz[0], lidar_vehicle_xyz[1], lidar_vehicle_xyz[2]);
  // init pose
  std::vector<double> init_xyz = config_node_["init_position"].as<std::vector<double>>();
  std::vector<double> init_rpy = config_node_["init_euler"].as<std::vector<double>>();
  init_orientation_ = Eigen::AngleAxisd(init_rpy[2], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(init_rpy[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(init_rpy[0], Eigen::Vector3d::UnitX());
  init_position_ = Eigen::Vector3d(init_xyz[0], init_xyz[1], init_xyz[2]);
  std::cout << "-------------------- Configuration --------------------"<<std::endl;
  std::cout << "is_pub_cloud_: " <<is_pub_cloud_ << std::endl;
  std::cout << "input_cloud_size_thr: " << input_cloud_size_thr_ << std::endl;
  std::cout << "blind_distance_: " << blind_distance_ << std::endl;
  std::cout << "-------------------- Calibration --------------------"<<std::endl;
  std::cout << "T_base_lidar_: \n" << T_base_lidar_ << std::endl;
  std::cout << "-------------------- Initialization --------------------"<<std::endl;
  std::cout << "init_position_: \n" << init_position_.transpose() << std::endl;
  std::cout << "init_orientation_: \n" << init_orientation_.matrix() << std::endl;
  std::cout << "-------------------------------------------------------"<<std::endl;

  bool init_map = initMap();
  status_ = STATUS::LOST;
#ifndef USE_EIGEN_OPTIMIZATION
  lidar_matcher_ = std::make_shared<LidarMatcherCeres>(config_node_["lidar_matcher"]);
#else
  lidar_matcher_ = std::make_shared<LidarMatcherEigen>(config_node_["lidar_matcher"]);
#endif
}

// 添加相对位姿
void LidarLocalization::addRelPose(const Pose &pose) {
  rel_mutex_.lock();
  // 将位姿插入相对位姿映射中
  rel_poses_map_.insert(std::make_pair(pose.timestamp, pose));
  rel_mutex_.unlock();
}

// 添加激光雷达数据: 主流程
void LidarLocalization::addLidarData(const pcl::PointCloud<RsPointXYZIRT>::Ptr &lidar_cloud) {
  double lidar_time = lidar_cloud->header.stamp * 1e-6;
  std::cout << "lidar timestamp :"<< std::fixed << lidar_time << std::endl;
  // 变换到车体
  pcl::PointCloud<RsPointXYZIRT>::Ptr lidar_cloud_base(new pcl::PointCloud<RsPointXYZIRT>);
  pcl::transformPointCloud(*lidar_cloud, *lidar_cloud_base, T_base_lidar_);
  pcl::PointCloud<RsPointXYZIRT>::Ptr lidar_cloud_base_ptr(new pcl::PointCloud<RsPointXYZIRT>);
  //check input lidar_cloud_base->points.size()
  if (lidar_cloud_base->points.size() < input_cloud_size_thr_) {
    LERROR << "addLidarData : INPUT CLOUD SIZE SMALL TAHN 1000: " << lidar_cloud_base->points.size()
           << REND;
    return;
  }
  //////////////////////////////////////
  lidar_cloud_base_ptr->reserve(lidar_cloud_base->points.size());
  lidar_cloud_base_ptr->header = lidar_cloud_base->header;
  for (int i = 0; i < lidar_cloud_base->points.size(); i++) {
    auto pt = lidar_cloud_base->points[i];
    if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z < blind_distance_*blind_distance_ || pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > 900.0 || pt.z > 10) {
      continue;
    }
    lidar_cloud_base_ptr->points.emplace_back(pt);
  }
  Pose result_pose;
  result_pose.timestamp = lidar_time;

  // 判断是否存在相对位姿及时间戳是否可以匹配
  rel_mutex_.lock();
  if (rel_poses_map_.size() < 2 || lidar_time < rel_poses_map_.begin()->first) {
    std::cout << "LiDAR earlier than first rel_tf, skip this scan" << std::endl;
    rel_mutex_.unlock();
    return;
  }
  rel_mutex_.unlock();
  // 点云去畸变
  PointCloudT::Ptr undistorted_cloud(new PointCloudT);
  Eigen::Affine3d pose_undistort;
  if (!undistortPointCloud(lidar_cloud_base_ptr, undistorted_cloud, pose_undistort))
    return;
  // 去除nan点
  undistorted_cloud->is_dense = false;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*undistorted_cloud, *undistorted_cloud, indices);
  undistorted_cloud->height = 1;
  undistorted_cloud->width = undistorted_cloud->points.size();
  PointCloudT::Ptr semantic_cloud(new PointCloudT);
  semanticFilter(undistorted_cloud, semantic_cloud);
  AlignInfo matcher_align_info;
  std::cout << "semantic_cloud size: " << semantic_cloud->points.size() << std::endl;
  switch (status_) {
    case STATUS::IDLE:
    case STATUS::LOST: {
      // 初始化
      Pose init_lidar_pose;
      init_lidar_pose.xyz = init_position_;
      init_lidar_pose.q = init_orientation_;
      init_lidar_pose.timestamp = lidar_time;
      bool ret_align =
          lidar_matcher_->align(semantic_cloud, kdtree_ptr_, map_cloud_ptr_,
                                init_lidar_pose, result_pose, lidar_time);
      std::cout << "init_lidar_pose: \n" << result_pose.transform() << std::endl;

      last_lidar_pose_ = result_pose;
      last_lidar_pose_.timestamp = lidar_time;
      last_lidar_pose_.q = result_pose.q;
      last_lidar_pose_.xyz = result_pose.xyz;
      status_ = STATUS::LOW_ACCURACY;
      /////加入status
      result_pose.setStatus(status_);
      //////////
      break;
    }
    case STATUS::LOW_ACCURACY:
    case STATUS::LOW_ACCURACY_RPZ:
    case STATUS::NORMAL: {
      Pose init_pose;
      Pose origin_init_pose;
      //init_pose: 从last_lidar_pose_外推的位姿 origin_init_pose: 原始相对定位pose
      if (forwardPropagate(last_lidar_pose_, lidar_time, init_pose, origin_init_pose)) {
        RSTicToc time_c0{"cloud align"};
        time_c0.tic();
        bool ret_align = lidar_matcher_->align(
            semantic_cloud, kdtree_ptr_, map_cloud_ptr_, init_pose, result_pose,lidar_time);
        matcher_align_info = lidar_matcher_->getAlignInfo();
        double match_score = matcher_align_info.point_pair_distance_residual; ///残差
        double valid_pair_ratio = matcher_align_info.ceres_valid_pair_ratio;
        std::cout << "valid_pair_ratio: " << valid_pair_ratio << std::endl;
        bool ret_score = match_score < MATCHING_CHECK_RESIDUAL_THRESH;
        bool valid_pair_score  = valid_pair_ratio > 0.9;
        time_c0.toc();
        std::cout << "ret_align: " << ret_align << " ret_score: " <<  ret_score <<" valid_pair_score: " << valid_pair_score << std::endl;
        if (ret_align && ret_score && valid_pair_score) {
          status_ = STATUS::NORMAL;
          result_pose.setStatus(status_);
          last_lidar_pose_ = result_pose;
        } else {
          status_ = STATUS::LOW_ACCURACY;
          result_pose.setStatus(status_);
          last_lidar_pose_ = origin_init_pose;
          result_pose = last_lidar_pose_;
        }
      }
      break;
    }
  }
#ifdef ROS1
  static ros::NodeHandle nh_;
  static ros::Publisher pub_cloud =
      nh_.advertise<sensor_msgs::PointCloud2>("/lidar_world_cloud", 1);
  sensor_msgs::PointCloud2 world_cloud_msg;
  PointCloudT::Ptr world_cloud_ptr(new PointCloudT());
  auto T = result_pose.transform();
  pcl::transformPointCloud(*semantic_cloud, *world_cloud_ptr, T.cast<float>());
  pcl::toROSMsg(*world_cloud_ptr, world_cloud_msg);
  world_cloud_msg.header.stamp = ros::Time::now();
  world_cloud_msg.header.frame_id = "rslidar";
  pub_cloud.publish(world_cloud_msg);
#endif

#ifdef ROS2
  if (is_pub_cloud_) {
    static auto nh_ = rclcpp::Node::make_shared("align_pc_pub_node");
    static auto pub_cloud = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_world_cloud", 1);
    sensor_msgs::msg::PointCloud2 world_cloud_msg;
    PointCloudT::Ptr world_cloud_ptr(new PointCloudT());
    auto T = result_pose.transform();
    pcl::transformPointCloud(*semantic_cloud, *world_cloud_ptr, T.cast<float>());
    pcl::toROSMsg(*world_cloud_ptr, world_cloud_msg);
    world_cloud_msg.header.stamp = rclcpp::Clock().now();
    world_cloud_msg.header.frame_id = "rslidar";
    pub_cloud->publish(world_cloud_msg);
  }
#endif

  for (const auto &cb : callbacks_) {
    cb(result_pose);
  }
}

bool LidarLocalization::initMap() {
  // 加载地图
  map_path_ = PROJECT_DIR + config_node_["map_path"].as<std::string>();
  pcl::PointCloud<PointT>::Ptr map_ptr(new pcl::PointCloud<PointT>);
  map_cloud_ptr_.reset(new pcl::PointCloud<PointT>());
  if (pcl::io::loadPCDFile<PointT>(map_path_, *map_ptr) == -1) {
    std::cout << "Failed to load map: " << map_path_ << std::endl;
    return false;
  }
  std::cout << "Loaded map: " << map_path_ << std::endl;
  map_ptr->is_dense = false;
  map_ptr->height = 1;
  map_ptr->width = map_ptr->points.size();
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*map_ptr, *map_ptr, indices);
  // 地图下采样
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(map_ptr);
  voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  voxel_filter.filter(*map_cloud_ptr_);
  map_cloud_ptr_->height = 1;
  map_cloud_ptr_->width = map_cloud_ptr_->points.size();
  // 构建地图KDtree
  kdtree_ptr_.reset(new pcl::KdTreeFLANN<PointT>());
  kdtree_ptr_->setInputCloud(map_cloud_ptr_);
  std::cout << "map size: " << map_cloud_ptr_->size() << std::endl;
  return true;
}

bool LidarLocalization::undistortPointCloud(
    const pcl::PointCloud<RsPointXYZIRT>::Ptr lidar_cloud,
    PointCloudT::Ptr &undistorted_cloud, Eigen::Affine3d &pose_undistort) {
  rel_mutex_.lock();
  if (rel_poses_map_.size() < 2) {
    std::cout << "No enough relative pose, skip this scan" << std::endl;
    rel_mutex_.unlock();
    return false;
  }

  Pose rel_tf_begin;
  Pose rel_tf_end;
  bool ret1 = interpolate(rel_poses_map_, lidar_cloud->points.front().timestamp,
                          rel_tf_begin);
  bool ret2 = interpolate(rel_poses_map_, lidar_cloud->points.back().timestamp,
                          rel_tf_end);
  rel_mutex_.unlock();

  Eigen::Affine3d dst = Eigen::Affine3d::Identity();
  if (ret1 && ret2) {
    pointsUndistort(lidar_cloud, undistorted_cloud, rel_tf_begin, rel_tf_end, 0,
                    dst, true);
    undistorted_cloud->header = lidar_cloud->header;
    pose_undistort = dst;
    return true;
  } else {
    return false;
  }
}

void LidarLocalization::semanticFilter(const PointCloudT::Ptr &undistorted_cloud, PointCloudT::Ptr &semantic_cloud) {
  for (int i = 0; i < undistorted_cloud->points.size(); ++i) {
    auto &pt = undistorted_cloud->points[i];
    pt.curvature = 1; // record as new points (current scan)
    if (pt.z < 0.2) {
      pt.intensity = -1; // ground
      if (pt.y > -10 && pt.y < 10 && pt.x > -50 && pt.x < 150) {
        pt.intensity = -2; // special ground
      }
    } else {
      if (pt.z > 1.5) {
        pt.intensity = 2;
      }
      else
        pt.intensity = 1; // non ground
    }
    semantic_cloud->points.push_back(pt);
  }

  semantic_cloud = voxelGridFilter(semantic_cloud, 0.2);
  semantic_cloud->header = undistorted_cloud->header;
  semantic_cloud->height = 1;
  semantic_cloud->width = semantic_cloud->points.size();
}

bool LidarLocalization::forwardPropagate(const Pose &last_pose, double t,
                                         Pose &pose_at_t,Pose& origin_pose_at_t) {
  if (last_pose.timestamp > t)
    return false;

  rel_mutex_.lock();
  Pose rel_pose_begin;
  if (!interpolate(rel_poses_map_, last_pose.timestamp, rel_pose_begin)) {
    std::cout << "FP: failed getting rel_pose_begin" << std::endl;
    rel_mutex_.unlock();
    return false;
  }

  Pose rel_pose_t;
  if (!interpolate(rel_poses_map_, t, rel_pose_t)) {
    std::cout << "FP: failed getting rel_pose_t" << std::endl;
    rel_mutex_.unlock();
    return false;
  }
  rel_mutex_.unlock();

  auto T_b = rel_pose_begin.transform();
  auto T_t = rel_pose_t.transform();
  auto T = T_b.inverse() * T_t;

  Eigen::Matrix4d T_pose_at_t = last_pose.transform() * T;
  pose_at_t = Pose(T_pose_at_t, t);
  pose_at_t.timestamp = t;
  origin_pose_at_t = rel_pose_t;
  origin_pose_at_t.timestamp = t;
  return true;
}

PointCloudT::Ptr LidarLocalization::voxelGridFilter(const PointCloudT::Ptr &cloud_ptr,
                                   float leaf_size) {
  pcl::VoxelGrid<PointT> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid_filter.setInputCloud(cloud_ptr);
  PointCloudT::Ptr filtered_ptr(new PointCloudT);
  voxel_grid_filter.filter(*filtered_ptr);
  return filtered_ptr;
}

