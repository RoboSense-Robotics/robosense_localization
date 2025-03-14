#pragma once
#include <fstream>
#include <atomic>
#include <iomanip>
#include <Eigen/Core>
#include <mutex>
#include <deque>
#include <queue>
#include <map>
#include <condition_variable>
#include <thread>
#include "type.h"
#include "eskf_helper.h"
#include <iostream>
#include <filesystem>
namespace Robosense {

class EskfInterface {
public:
    EskfInterface(const double acc_noise, const double gyro_noise,
                            const double acc_bias_noise, const double gyro_bias_noise,
                            const double can_odom_noise, const double lidar_t_noise,
                            const double lidar_R_noise, const Eigen::Vector3d& gravity);
    ~EskfInterface();

    void addSensorBuff(SensorDataPtr sensor_data_ptr);
    void eskfThread();
    void start();
    void stop();
    void stateInit(const ImuDataPtr first_imu_ptr, const CanOdomDataPtr first_can_ptr);
    bool getFusionState(State& state);
    void ifCanUpdate(const bool can_update){can_update_ = can_update;};
    void setInitSigma(const double p_init_sigma, const double v_init_sigma,
                                const double R_init_sigma, const double ba_init_sigma,
                                const double bg_init_sigma);
    void setInitPose(const Eigen::Vector3d& init_pose, const Eigen::Matrix3d& init_R);
    Eigen::Vector3d gravity_{Eigen::Vector3d(0, 0, -9.801)};
    State state_;
    bool localization_enable_ = false;
    std::condition_variable data_cond_;
    std::map<double,State> fusion_states_;
private:
    std::shared_ptr<EskfHelper> eskf_helper_;
    std::map<double,SensorDataPtr> sensor_buff_;
    std::queue<double> sensor_time_buff_;
    State pub_state_;
    std::mutex sensor_buff_mutex_;
    std::mutex state_buff_mutex_;
    std::mutex pub_state_mutex;
    std::thread eskf_thread_;
    std::atomic<double> update_start_time_{std::numeric_limits<double>::infinity()};
    std::atomic<double> latest_state_time_ {std::numeric_limits<double>::infinity()};
    std::atomic<double> oldest_state_time_ {0};
    double last_state_timestamp_;
    bool can_update_ = false;
    double p_init_sigma_;
    double v_init_sigma_;
    double R_init_sigma_;
    double ba_init_sigma_;
    double bg_init_sigma_;
    Eigen::Vector3d init_pose_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d init_R_{Eigen::Matrix3d::Identity()};
};

} 