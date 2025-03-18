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
 
#include "eskf_helper.h"

namespace Robosense {
EskfHelper::EskfHelper(const double acc_noise, const double gyro_noise,
               const double acc_bias_noise, const double gyro_bias_noise,
               const double can_odom_noise, const double lidar_t_noise,
               const double lidar_R_noise, const Eigen::Vector3d& gravity)
    : acc_noise_(acc_noise), gyro_noise_(gyro_noise), acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
      can_odom_noise_(can_odom_noise), lidar_t_noise_(lidar_t_noise), lidar_R_noise_(lidar_R_noise), gravity_(gravity){}


void EskfHelper::lidarUpdatePos(SensorDataPtr lidar_pos_ptr, State& state, State& new_state){
    Eigen::Matrix<double, 6, 1> residual;
    Eigen::Matrix<double, 6, 15> H;
    Eigen::Vector3d t = std::static_pointer_cast<LidarOdomData>(lidar_pos_ptr)->pos.block<3, 1>(0, 3);
    Eigen::Matrix3d R = std::static_pointer_cast<LidarOdomData>(lidar_pos_ptr)->pos.block<3, 3>(0, 0);
    residual.block<3, 1>(0, 0) = t - state.p_IinG;
    Eigen::Vector3d residual_vehicle ;
    residual_vehicle = state.R_G2I.inverse() * residual.block<3, 1>(0, 0);
    Eigen::AngleAxisd res_rotation_vec(state.R_G2I.inverse() * R);
    residual.block<3, 1>(3, 0) = res_rotation_vec.angle() * res_rotation_vec.axis();

    H.setZero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

    // ESKF
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity();
    Q.block<3, 3>(0, 0) *= lidar_t_noise_;
    Q.block<3, 3>(3, 3) *= lidar_R_noise_;
    //AINFO << "lidar update residual: " << residual.transpose();
    const Eigen::MatrixXd& P = state.cov;
    const Eigen::MatrixXd K = P * H.transpose() *
                              (H * P * H.transpose() + Q).inverse();
    const Eigen::VectorXd delta_x = K * residual;
    addDeltaToState(delta_x, new_state);
    const Eigen::MatrixXd I_KH =
        Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    new_state.cov = I_KH * P * I_KH.transpose() + K * Q * K.transpose();

    // ESKF reset
    Eigen::Matrix<double, 15, 15> G = Eigen::Matrix<double, 15, 15>::Identity();
    G.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() -
                          GetSkewMatrix(0.5 * delta_x.block<3, 1>(6, 0));
    new_state.cov = G * new_state.cov * G.transpose();
    new_state.timestamp = lidar_pos_ptr->timestamp;
}

void EskfHelper::addDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State& new_state){
    new_state.p_IinG += delta_x.block<3, 1>(0, 0);
    new_state.v_IinG += delta_x.block<3, 1>(3, 0);
    new_state.acc_bias += delta_x.block<3, 1>(9, 0);
    new_state.gyro_bias += delta_x.block<3, 1>(12, 0);
    new_state.R_G2I = new_state.R_G2I *
                      Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(),
                                        delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
}    

void EskfHelper::fastPredict(double t, State& state, State& new_state){
    double dt = t - state.timestamp;
    const Eigen::Vector3d gyro_unbias = state.gyro - state.gyro_bias;
    const Eigen::Vector3d acc_unbias = state.acc - state.acc_bias;
    const Eigen::Vector3d delta_angle_axis = gyro_unbias * dt;
    const Eigen::Vector3d half_delta_angle_axis = gyro_unbias * dt * 0.5;
    const Eigen::Vector3d vel = state.v_IinG;
    new_state.R_G2I = state.R_G2I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
    Eigen::Matrix3d half_R_G2I = state.R_G2I * Eigen::AngleAxisd(half_delta_angle_axis.norm(),
                                        half_delta_angle_axis.normalized()).toRotationMatrix();
    auto v_k1 = state.R_G2I * acc_unbias + gravity_;
    auto v_k2 = half_R_G2I * acc_unbias + gravity_;
    auto v_k3 = v_k2;  
    auto v_k4 = new_state.R_G2I * acc_unbias + gravity_;
    auto v_next = vel + dt * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4) / 6;
    new_state.v_IinG = v_next;
    auto p_k1 = vel;
    auto p_k2 = vel + 0.5 * dt * v_k1;
    auto p_k3 = vel + 0.5 * dt * v_k2;
    auto p_k4 = vel + dt * v_k3;
    auto p_next = state.p_IinG + dt * (p_k1 + 2 * p_k2 + 2 * p_k3 + p_k4) / 6;
    new_state.p_IinG = p_next;
}

void EskfHelper::predictAndUpdate(SensorDataPtr cur_imu, State& state, State& new_state){
 // Delta Time.
    const double delta_t =cur_imu->timestamp - state.timestamp;
    const double delta_t2 = delta_t * delta_t;
    new_state.timestamp = cur_imu->timestamp;
    // Acc and gyro.
    const Eigen::Vector3d acc_unbias = 0.5 * (state.acc + std::static_pointer_cast<ImuData>(cur_imu)->acc) - state.acc_bias;
    const Eigen::Vector3d gyro_unbias =  0.5 * (state.gyro + std::static_pointer_cast<ImuData>(cur_imu)->gyro) - state.gyro_bias;
    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
    new_state.R_G2I = state.R_G2I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();

    Eigen::Vector3d vel = state.v_IinG;
    auto v_k1 = state.R_G2I * (state.acc -  state.acc_bias) + gravity_;
    auto v_k2 =
        (state.R_G2I * (state.acc - state.acc_bias) + new_state.R_G2I * (new_state.acc - state.acc_bias)) / 2 + gravity_;
    // auto v_k2 = R_half * (mid_acc - acc_bias) + g;
    auto v_k3 = v_k2;  
    auto v_k4 = new_state.R_G2I * (new_state.acc- state.acc_bias) + gravity_;
    auto v_next = vel + delta_t * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4) / 6;
    new_state.v_IinG = v_next;

  // position integration
    auto p_k1 = vel;
    auto p_k2 = vel + 0.5 * delta_t * v_k1;
    auto p_k3 = vel + 0.5 * delta_t * v_k2;
    auto p_k4 = vel + delta_t * v_k3;
    Eigen::Vector3d position = state.p_IinG;
    auto p_next = position + delta_t * (p_k1 + 2 * p_k2 + 2 * p_k3 + p_k4) / 6;
    new_state.p_IinG = p_next;

    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
    // Fx.block<3, 3>(3, 6)   = - state.R_G2I * GetSkewMatrix(acc_unbias*1e12) * delta_t;
    Fx.block<3, 3>(3, 6)   = - state.R_G2I * GetSkewMatrix(acc_unbias) * delta_t;
    Fx.block<3, 3>(3, 9)   = - new_state.R_G2I * delta_t;
    Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
    Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> QI = Eigen::Matrix<double, 12, 12>::Zero();
    QI.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    QI.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    QI.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    QI.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();
    new_state.cov = Fx * state.cov * Fx.transpose() + Fi * QI * Fi.transpose();
     
    //UPDATE 
    Eigen::Matrix<double, 3, 15> H = Eigen::Matrix<double,3,15>::Zero();

    Eigen::Matrix<double,3,1> residual;
    const Eigen::Vector3d& v_IinG  = new_state.v_IinG;
    const Eigen::Matrix3d& R_G2I  = new_state.R_G2I;
    residual = std::static_pointer_cast<ImuData>(cur_imu)->imu_fake_vel - R_G2I.transpose()*v_IinG;
    H.block<3, 3>(0, 3) = R_G2I.transpose();
    H.block<3, 3>(0, 6) = GetSkewMatrix(R_G2I.transpose()*v_IinG);
    Eigen::Matrix<double, 3, 3> QV;
    QV = Eigen::Matrix<double, 3, 3>::Identity()* can_odom_noise_ ;

    const Eigen::MatrixXd& P = new_state.cov;
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + QV).inverse();
    Eigen::Matrix<double, 15, 1> delta_x = K * residual;

    addDeltaToState(delta_x, new_state);
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    new_state.cov = I_KH * P * I_KH.transpose() + K * QV * K.transpose();

    //ESKF reset
    Eigen::Matrix<double, 15, 15> G = Eigen::Matrix<double, 15, 15>::Identity();
    G.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - GetSkewMatrix(0.5*delta_x.block<3,1>(6,0));
    new_state.cov = G*new_state.cov*G.transpose();
    return ;
}

void EskfHelper::imuOdomPredict(SensorDataPtr cur_imu, State& state, State& new_state){
    // Delta Time.
    const double delta_t =cur_imu->timestamp - state.timestamp;
    const double delta_t2 = delta_t * delta_t;
    new_state.timestamp = cur_imu->timestamp;
    const Eigen::Vector3d gyro_unbias =  0.5 * (state.gyro + std::static_pointer_cast<ImuData>(cur_imu)->gyro) - state.gyro_bias;

    new_state.acc = std::static_pointer_cast<ImuData>(cur_imu)->acc;
    new_state.gyro = std::static_pointer_cast<ImuData>(cur_imu)->gyro;
    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
    new_state.R_G2I = state.R_G2I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();

    new_state.v_IinG = new_state.R_G2I * std::static_pointer_cast<ImuData>(cur_imu)->imu_fake_vel;
    new_state.can_odom_vel = std::static_pointer_cast<ImuData>(cur_imu)->imu_fake_vel;
    new_state.p_IinG = state.p_IinG + (state.v_IinG + new_state.v_IinG) * 0.5f * delta_t;

    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6)   = - state.R_G2I*GetSkewMatrix(std::static_pointer_cast<ImuData>(cur_imu)->imu_fake_vel)* delta_t;
    Fx.block<3, 3>(9, 9)   = Eigen::Matrix3d::Zero();

    Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
    Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> QI = Eigen::Matrix<double, 12, 12>::Zero();
    QI.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    QI.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    QI.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    QI.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

    new_state.cov = Fx * state.cov * Fx.transpose() + Fi * QI * Fi.transpose();
}

}