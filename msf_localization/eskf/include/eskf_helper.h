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
#include <eigen3/Eigen/Core>
#include "type.h"

constexpr double R2D = 180. / M_PI;
constexpr double D2R = M_PI / 180.;
namespace Robosense {

class EskfHelper {
public:
    EskfHelper(const double acc_noise, const double gyro_noise,
               const double acc_bias_noise, const double gyro_bias_noise,
               const double can_odom_noise, const double lidar_t_noise,
               const double lidar_R_noise, const Eigen::Vector3d& gravity);

    void lidarUpdatePos(SensorDataPtr lidar_pos_ptr, State& state, State& new_state);
    void addDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State& new_state);           
    void fastPredict(double t, State& state, State& new_state);     
    void predictAndUpdate(SensorDataPtr cur_imu, State& state, State& new_state);
    void imuOdomPredict(SensorDataPtr cur_imu, State& state, State& new_state);

private:
    double acc_noise_;
    double gyro_noise_;
    double acc_bias_noise_;
    double gyro_bias_noise_;
    double can_odom_noise_;
    double lidar_t_noise_;
    double lidar_R_noise_;
    Eigen::Vector3d gravity_;
};

inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}

}  