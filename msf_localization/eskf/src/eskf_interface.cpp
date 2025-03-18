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
 
#include <limits>
#include <glog/logging.h>
#include <algorithm>
#include "eskf_interface.h"
#include <stdlib.h>

namespace Robosense {

EskfInterface::EskfInterface(const double acc_noise, const double gyro_noise,
                                const double acc_bias_noise, const double gyro_bias_noise,
                                const double can_odom_noise, const double lidar_t_noise,
                                const double lidar_R_noise, const Eigen::Vector3d& gravity){
    eskf_helper_ = std::make_shared<EskfHelper>(acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, can_odom_noise, lidar_t_noise, lidar_R_noise, gravity);
}

void EskfInterface::addSensorBuff(SensorDataPtr sensor_data_ptr) {
    if (sensor_data_ptr->timestamp < oldest_state_time_.load()) {
        LOG(WARNING) << "The sensor: " << sensor_data_ptr->type
                    << " is eariler than the first state, it will be discarded" 
                    << std::fixed << sensor_data_ptr->timestamp 
                    << " oldest_state_time_:" << oldest_state_time_;
        return;
    }

    // If a observation is 1 secs before the latest we think it is too old
    if (sensor_data_ptr->timestamp < latest_state_time_.load() - 0.5) {
        LOG(WARNING) << "Sensor msg type ID:" << sensor_data_ptr->type << " delay over 0.5s so discarded";
        return;
    }

    std::unique_lock<std::mutex> lock(sensor_buff_mutex_);
    sensor_buff_[sensor_data_ptr->timestamp] = sensor_data_ptr;
    double update_time = sensor_data_ptr->timestamp;
    sensor_time_buff_.push(update_time);
    data_cond_.notify_one();
}

void EskfInterface::eskfThread() {
    while (true) {
        // wait condition_variable
        std::unique_lock<std::mutex> sensor_lock(sensor_buff_mutex_);
        data_cond_.wait(sensor_lock, [this]() {
            return (!sensor_time_buff_.empty() || !localization_enable_);
        });

        if (!localization_enable_) {
            sensor_lock.unlock();
            LOG(INFO) << "localization finish";
            break;
        }

        update_start_time_.store(sensor_time_buff_.front());
        sensor_time_buff_.pop();
        
        if (sensor_buff_.rbegin()->first - update_start_time_ > 0.3)
            continue;

        double trim_stop_time = 0;
        trim_stop_time = sensor_buff_.rbegin()->second->timestamp - 100.0;
        
        std::unique_lock<std::mutex> state_lock(state_buff_mutex_);
        auto it = fusion_states_.lower_bound(update_start_time_);
        if (it == fusion_states_.begin()) {
            continue;
        }
        
        if (it != fusion_states_.end() && it != fusion_states_.begin()) {
            fusion_states_.erase(it, fusion_states_.end());
        }

        std::map<double, SensorDataPtr> sensors_for_update;
        for (auto it = sensor_buff_.find(update_start_time_); it != sensor_buff_.end(); ++it)
            sensors_for_update[it->first] = it->second;

        if (sensors_for_update.empty())
            LOG(ERROR) << "Eskf Cannot find the state for updating, this should never happen";

        for (auto sensor : sensors_for_update) {
            State new_state;
            SensorDataPtr& sensor_ptr = sensor.second;

            switch (sensor_ptr->type) {
                case SensorType::Imu: {
                    auto it = fusion_states_.upper_bound(sensor_ptr->timestamp);
                    if (it != fusion_states_.end()) {
                        LOG(ERROR) << "fusion_states have larger timestamp than sensor timestamp this should not happen";
                    }
                    if (it == fusion_states_.begin()) {
                        LOG(ERROR) << "imu first time: " << fusion_states_.begin()->first 
                                 << " senor time : " << sensor_ptr->timestamp;
                        continue;
                    }
                    it--;
                    new_state = it->second;
                    new_state.type = sensor_ptr->type;
                    new_state.timestamp = sensor_ptr->timestamp;
                    new_state.acc = std::static_pointer_cast<ImuData>(sensor_ptr)->acc;
                    new_state.gyro = std::static_pointer_cast<ImuData>(sensor_ptr)->gyro;
                    new_state.gyro_bias = std::static_pointer_cast<ImuData>(sensor_ptr)->gyro_bias;
                    if (sensor_ptr->cur_car_status == CarStatus::stop) {
                        fusion_states_[new_state.timestamp] = new_state;
                        break;
                    }
                    if (can_update_) 
                        eskf_helper_->predictAndUpdate(sensor_ptr, it->second, new_state);
                    else 
                        eskf_helper_->imuOdomPredict(sensor_ptr, it->second, new_state);
                    
                    fusion_states_[new_state.timestamp] = new_state;
                    break;
                }

                case SensorType::LidarOdom: {
                    auto it = fusion_states_.upper_bound(sensor_ptr->timestamp);
                    if (it != fusion_states_.end()) {
                        LOG(ERROR) << "fusion_states have larger timestamp than sensor timestamp this should not happen";
                    }
                    if (it == fusion_states_.begin()) {
                        LOG(ERROR) << "first time: " << fusion_states_.begin()->first
                               << " senor time : " << sensor_ptr->timestamp;
                        continue;
                    }
                    it--;
                    
                    new_state = it->second;
                    new_state.type = sensor_ptr->type;
                    new_state.timestamp = sensor_ptr->timestamp;
                    
                    if (sensor_ptr->cur_car_status == CarStatus::stop) {
                        new_state.v_IinG.setZero();
                        fusion_states_[new_state.timestamp] = new_state;
                        break;
                    }
                    
                    double dt = sensor_ptr->timestamp - it->first;
                    if (dt > 0.001) {
                        eskf_helper_->fastPredict(sensor_ptr->timestamp, it->second, new_state);
                    }
                    new_state.timestamp = sensor_ptr->timestamp;

                    eskf_helper_->lidarUpdatePos(sensor_ptr, it->second, new_state);
                    fusion_states_[new_state.timestamp] = new_state;
                    break;
                }

                default:
                    LOG(ERROR) << "sensor type is none this should never happen";
            }
        }

        latest_state_time_ = sensors_for_update.rbegin()->first;

        if (sensor_buff_[update_start_time_]->type == SensorType::Imu) {
            std::unique_lock<std::mutex> pub_state_lock(pub_state_mutex);
            pub_state_ = fusion_states_[update_start_time_];
        }

        /* trimObservations */
        if (trim_stop_time > sensor_buff_.begin()->first) {
            auto it = sensor_buff_.upper_bound(trim_stop_time);
            --it;
            sensor_buff_.erase(sensor_buff_.begin(), it);
        }

        { /* trimFusionStates */
            if (trim_stop_time > fusion_states_.begin()->first) {
                auto itr = fusion_states_.upper_bound(trim_stop_time);
                --itr;
                fusion_states_.erase(fusion_states_.begin(), itr);
            }
            oldest_state_time_.store(fusion_states_.begin()->first);
        }
    }
    return;
}

void EskfInterface::setInitSigma(const double p_init_sigma, const double v_init_sigma,
                                const double R_init_sigma, const double ba_init_sigma,
                                const double bg_init_sigma){
    p_init_sigma_ = p_init_sigma;
    v_init_sigma_ = v_init_sigma;
    R_init_sigma_ = R_init_sigma;
    ba_init_sigma_ = ba_init_sigma;
    bg_init_sigma_ = bg_init_sigma;
}

void EskfInterface::setInitPose(const Eigen::Vector3d& init_pose, const Eigen::Matrix3d& init_R){
    init_pose_ = init_pose;
    init_R_ = init_R;
}

void EskfInterface::stateInit(const ImuDataPtr first_imu_ptr, const CanOdomDataPtr first_can_ptr) {
    State init_state;
    init_state.timestamp = std::max({first_imu_ptr->timestamp, first_can_ptr->timestamp});
    
    std::unique_lock<std::mutex> state_lock(state_buff_mutex_);
    fusion_states_.clear();
    sensor_buff_.clear();
    latest_state_time_.store(init_state.timestamp);
    oldest_state_time_.store(init_state.timestamp);
    
    init_state.p_IinG = init_pose_;
    init_state.R_G2I = init_R_;
    init_state.v_IinG = init_state.R_G2I * first_can_ptr->vel;
    init_state.acc_bias.setZero();
    init_state.gyro_bias.setZero();
    init_state.gyro = first_imu_ptr->gyro - first_imu_ptr->gyro_bias;
    init_state.acc = first_imu_ptr->acc;

    // Set covariance.
    init_state.cov.setZero();
    //position
    init_state.cov.block<3, 3>(0, 0) << Eigen::Matrix3d::Identity() * p_init_sigma_ * p_init_sigma_;
    //velocity
    init_state.cov.block<3, 3>(3, 3) << Eigen::Matrix3d::Identity() * v_init_sigma_ * v_init_sigma_ ;
    //rotation
    init_state.cov.block<3, 3>(6, 6) << Eigen::Matrix3d::Identity() * R_init_sigma_ * R_init_sigma_ ;
    // Acc bias.
    init_state.cov.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * ba_init_sigma_ * ba_init_sigma_ ;
    // Gyro bias.
    init_state.cov.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * bg_init_sigma_ * bg_init_sigma_;
    
    fusion_states_.emplace(init_state.timestamp, init_state);
    pub_state_ = fusion_states_[init_state.timestamp];
    state_lock.unlock();    
}

bool EskfInterface::getFusionState(State& state) {
    if (!localization_enable_)
        return false;
        
    std::unique_lock<std::mutex> pub_state_lock(pub_state_mutex);
    state = pub_state_;
    if (state.timestamp == last_state_timestamp_)
        return false;
        
    last_state_timestamp_ = state.timestamp;
    return true;
}

void EskfInterface::start() {
    localization_enable_ = true;
    auto eskf_func_thread = [this]() {
        eskfThread();
    };
    eskf_thread_ = std::thread(eskf_func_thread);
}

void EskfInterface::stop() {
    localization_enable_ = false;
    data_cond_.notify_all();
    if (eskf_thread_.joinable())
        eskf_thread_.join();
    return;
}


EskfInterface::~EskfInterface() {
    localization_enable_ = false;
    data_cond_.notify_all();
}

}  // namespace ImuGpsLocalization
