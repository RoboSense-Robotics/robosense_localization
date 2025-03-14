#ifndef type_h_
#define type_h_

#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace Robosense {

enum SensorType{
    None = 0,
    Imu = 1,
    GpsPos = 2,
    CanOdom = 3,
    LidarOdom = 4
};

enum CarStatus{
    stop = 0,
    forward = 1,
    backward = 2,
};

struct SensorData{
    double timestamp;
    SensorType type = None;
    CarStatus cur_car_status;
};
using SensorDataPtr = std::shared_ptr<SensorData>;

struct ImuData : public SensorData { 
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d imu_fake_vel =  Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
};
using ImuDataPtr = std::shared_ptr<ImuData>;


struct CanOdomData : public SensorData{
    Eigen::Vector3d vel;
};
using CanOdomDataPtr = std::shared_ptr<CanOdomData>;

struct LidarOdomData : public SensorData{
    Eigen::Matrix4d pos = Eigen::Matrix4d::Identity();
    double lidar_status;
};
using LidarOdomDataPtr = std::shared_ptr<LidarOdomData>;

struct State {
    double timestamp;

    Eigen::Vector3d p_IinG;     
    Eigen::Vector3d v_IinG;    
    Eigen::Matrix3d R_G2I;     // rotation from Global frame to IMU frame.
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Vector3d can_odom_vel;
    Eigen::Vector3d acc_bias;  
    Eigen::Vector3d gyro_bias; 
    // State Covariance.
    Eigen::Matrix<double, 15, 15> cov;
    SensorType type = None;

};

}  // Robosense
#endif