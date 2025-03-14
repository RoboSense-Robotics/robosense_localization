#include "ros1_interface.h"

Ros1Interface::Ros1Interface(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, can_odom_noise, lidar_t_noise, lidar_R_noise;
    double p_init_sigma, v_init_sigma, R_init_sigma, ba_init_sigma, bg_init_sigma;
    std::string imu_topic, can_topic, lidar_odom_topic, loc_odom_topic ;
    double x_imu_vehicle, y_imu_vehicle, z_imu_vehicle, roll_imu_vehicle, pitch_imu_vehicle, yaw_imu_vehicle;
    // set meta path
    std::string PROJECT_PATH_ = PROJECT_PATH;
    std::string meta_path = PROJECT_PATH_ + "config/";
    std::string loc_param_path = meta_path + "loc_param.yaml";

    YAML::Node loc_cfg = YAML::LoadFile(loc_param_path);
    // get loc param from meta
    imu_topic = loc_cfg["imu_topic"].as<std::string>();
    can_topic = loc_cfg["can_topic"].as<std::string>();
    loc_odom_topic = loc_cfg["loc_odom_topic"].as<std::string>();
    lidar_odom_topic = loc_cfg["lidar_odom_topic"].as<std::string>();

    acc_noise = loc_cfg["acc_noise"].as<double>();
    gyro_noise = loc_cfg["gyro_noise"].as<double>();
    acc_bias_noise = loc_cfg["acc_bias_noise"].as<double>();
    gyro_bias_noise = loc_cfg["gyro_bias_noise"].as<double>();
    can_odom_noise = loc_cfg["can_odom_noise"].as<double>();
    lidar_t_noise = loc_cfg["lidar_t_noise"].as<double>();
    lidar_R_noise = loc_cfg["lidar_R_noise"].as<double>();
    p_init_sigma = loc_cfg["p_init_sigma"].as<double>();
    v_init_sigma = loc_cfg["v_init_sigma"].as<double>();
    R_init_sigma = loc_cfg["R_init_sigma"].as<double>();
    ba_init_sigma = loc_cfg["ba_init_sigma"].as<double>();
    bg_init_sigma = loc_cfg["bg_init_sigma"].as<double>();
    g_ = loc_cfg["gravity"].as<double>();
    can_update_ = loc_cfg["can_update"].as<bool>();
    pub_traj_ = loc_cfg["pub_traj"].as<bool>();
    // get calib param from meta
    x_imu_vehicle = loc_cfg["x_imu_vehicle"].as<double>();
    y_imu_vehicle = loc_cfg["y_imu_vehicle"].as<double>();
    z_imu_vehicle = loc_cfg["z_imu_vehicle"].as<double>();
    roll_imu_vehicle = loc_cfg["roll_imu_vehicle"].as<double>();
    pitch_imu_vehicle = loc_cfg["pitch_imu_vehicle"].as<double>();
    yaw_imu_vehicle = loc_cfg["yaw_imu_vehicle"].as<double>();
    Eigen::Vector3d gravity_vec(0, 0, -g_);
    std::cout << "-------------------- Configuration --------------------"<<std::endl;
    std::cout << "IMU Topic: " << imu_topic<<std::endl;
    std::cout << "CAN Topic: " << can_topic<<std::endl;
    std::cout << "Lidar Odometry Topic: " << lidar_odom_topic<<std::endl;
    std::cout << "Localization Odometry Topic: " << loc_odom_topic<<std::endl;
    std::cout << "-------------------------------------------------------"<<std::endl;
    std::cout << "Accelerometer Noise: " << acc_noise<<std::endl;
    std::cout << "Gyroscope Noise: " << gyro_noise<<std::endl;
    std::cout << "Accelerometer Bias Noise: " << acc_bias_noise<<std::endl;
    std::cout << "Gyroscope Bias Noise: " << gyro_bias_noise<<std::endl;
    std::cout << "CAN Odometry Noise: " << can_odom_noise<<std::endl;
    std::cout << "Lidar Translation Noise: " << lidar_t_noise<<std::endl;
    std::cout << "Lidar Rotation Noise: " << lidar_R_noise<<std::endl;
    std::cout << "p_init_sigma: " << p_init_sigma<<std::endl;
    std::cout << "v_init_sigma: " << v_init_sigma<<std::endl;
    std::cout << "R_init_sigma: " << R_init_sigma<<std::endl;
    std::cout << "ba_init_sigma: " << ba_init_sigma<<std::endl;
    std::cout << "bg_init_sigma: " << bg_init_sigma<<std::endl;
    std::cout << "Gravity: " << g_<<std::endl;
    std::cout << "-------------------------------------------------------"<<std::endl;
    std::cout << "IMU Calibration Parameters:"<<std::endl;
    std::cout << "x: " << x_imu_vehicle<<std::endl;
    std::cout << "y: " << y_imu_vehicle<<std::endl;
    std::cout << "z: " << z_imu_vehicle<<std::endl;
    std::cout << "roll: " << roll_imu_vehicle<<std::endl;
    std::cout << "pitch: " << pitch_imu_vehicle<<std::endl;
    std::cout << "yaw: " << yaw_imu_vehicle<<std::endl;
    std::cout << "can_update: " << can_update_<<std::endl;
    std::cout << "pub_traj: " << pub_traj_<<std::endl;
    std::cout << "-------------------------------------------------------"<<std::endl;
    rotation_imu_vehicle_ = Eigen::AngleAxisd(yaw_imu_vehicle, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch_imu_vehicle, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll_imu_vehicle, Eigen::Vector3d::UnitX());
    t_imu_vehicle_ << x_imu_vehicle, y_imu_vehicle, z_imu_vehicle;
    // Initialization EskfInterface    
    eskf_interface_ptr_ = std::make_unique<Robosense::EskfInterface>(acc_noise, gyro_noise, 
                    acc_bias_noise, gyro_bias_noise, can_odom_noise, lidar_t_noise, lidar_R_noise, gravity_vec);
    eskf_interface_ptr_->setInitSigma(p_init_sigma, v_init_sigma, R_init_sigma, ba_init_sigma, bg_init_sigma);
    eskf_interface_ptr_->ifCanUpdate(can_update_);
    can_odom_sub_ = nh.subscribe(can_topic, 1000, &Ros1Interface::chassisCallback, this);
    imu_sub_ = nh.subscribe(imu_topic, 1000, &Ros1Interface::imuCallback, this);
    lidar_odom_sub_ = nh.subscribe(lidar_odom_topic, 1000, &Ros1Interface::lidarOdomCallback, this);
    traj_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
    loc_odom_pub_ = nh.advertise<nav_msgs::Odometry>(loc_odom_topic, 10);

    auto func1 = [this]() { waitStart(); };
    wait_start_thread_ = std::thread(func1);
    auto func2 = [this]() { pubState(); };
    pub_state_thread_ = std::thread(func2);

}

void Ros1Interface::waitStart() {
    while (true) {
        if (imu_ready_.load() && can_ready_.load() ) {
            eskf_interface_ptr_->stateInit(init_imu_ptr_, init_can_ptr_);
            started_.store(true);
            break;
        }
        usleep(1000);
    }
    LOG(WARNING) << "rs localization start!";
    eskf_interface_ptr_->start();
}

void Ros1Interface::pubState() {
    while (pub_enable_) {
        Robosense::State fusion_state;
        if (started_.load() && eskf_interface_ptr_->getFusionState(fusion_state)) {
            //pub localization odom 
            nav_msgs::Odometry loc_odom;
            loc_odom.header.frame_id = "rslidar";
            loc_odom.header.stamp = ros::Time().fromSec(fusion_state.timestamp);
            loc_odom.pose.pose.position.x = fusion_state.p_IinG[0];
            loc_odom.pose.pose.position.y = fusion_state.p_IinG[1];
            loc_odom.pose.pose.position.z = fusion_state.p_IinG[2];
            Eigen::Quaterniond G_q_I(fusion_state.R_G2I);
            G_q_I = G_q_I.normalized();
            loc_odom.pose.pose.orientation.x = G_q_I.x();
            loc_odom.pose.pose.orientation.y = G_q_I.y();
            loc_odom.pose.pose.orientation.z = G_q_I.z();
            loc_odom.pose.pose.orientation.w = G_q_I.w();
            loc_odom_pub_.publish(loc_odom);
            //pub traj for debug 
            if (pub_traj_) {
                stateToRos(fusion_state);
                traj_pub_.publish(fuse_path_);
                fuse_tf_broadcaster_.sendTransform(fuse_transform_);
            }
        }
        usleep(3000);
    }
}

Ros1Interface::~Ros1Interface() {
    pub_enable_ = false;
    if (pub_state_thread_.joinable()) {
        pub_state_thread_.join();
    }
    eskf_interface_ptr_->stop();
}

void Ros1Interface::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    if (!can_odom_buff_.size()) {
        return;
    }
    auto sensor_ptr = std::make_shared<Robosense::ImuData>();
    sensor_ptr->type = Robosense::Imu;
    sensor_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    sensor_ptr->acc << imu_msg_ptr->linear_acceleration.x,
                        imu_msg_ptr->linear_acceleration.y,
                        imu_msg_ptr->linear_acceleration.z;
    
    sensor_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                        imu_msg_ptr->angular_velocity.y,
                        imu_msg_ptr->angular_velocity.z;
    sensor_ptr->acc = rotation_imu_vehicle_ * sensor_ptr->acc;
    sensor_ptr->gyro = rotation_imu_vehicle_ * sensor_ptr->gyro;

    if (cur_car_status_ == Robosense::stop) {
        if (init_imu_num_ < std::numeric_limits<int>::max()) {
            mean_gyr_ += (sensor_ptr->gyro - mean_gyr_) / init_imu_num_;
            mean_acc_ += (sensor_ptr->acc - mean_acc_) / init_imu_num_;
            init_imu_num_++;
        }
    }
    sensor_ptr->cur_car_status = cur_car_status_;
    sensor_ptr->gyro_bias = mean_gyr_;

    imu_deque_.push_back(sensor_ptr);
    auto sensor_ptr_front = imu_deque_.front();
    //interpolate can odom vel at imu timestamp
    if (can_odom_buff_.size() > 0) {
        double t1, t2, v1, v2, vx;
        std::unique_lock<std::mutex> lock(can_odom_buff_mutex_);
        auto itr = can_odom_buff_.lower_bound(sensor_ptr_front->timestamp);
        if (itr == can_odom_buff_.end()) {
            vx = can_odom_buff_.rbegin()->second->vel[0];
        } else {
            t2 = itr->second->timestamp;
            v2 = itr->second->vel[0];
            if (itr != can_odom_buff_.begin()) {
                itr--;
                t1 = itr->second->timestamp;
                v1 = itr->second->vel[0];
                vx = (v2 - v1) / (double)(t2 - t1) * (double)(sensor_ptr_front->timestamp - t1) + v1;
            } else
                vx = v2;
            imu_deque_.pop_front();
        }
        sensor_ptr_front->imu_fake_vel << vx, 0.0, 0.0;
    }

    if (!started_.load()) {
        init_imu_ptr_ = sensor_ptr_front;
        imu_ready_.store(true);
        return;
    } else {
        eskf_interface_ptr_->addSensorBuff(sensor_ptr_front);
    }
}


void Ros1Interface::chassisCallback(const ros_adapter::HunterStatus::ConstPtr& msg_ptr) {
    can_speed_mps_ = msg_ptr->linear_velocity;
    if (abs(can_speed_mps_) < 1e-4){
        cur_car_status_ = Robosense::stop;
    }
    else if (can_speed_mps_ > 1e-4)
        cur_car_status_ = Robosense::forward;
    else
        cur_car_status_ = Robosense::backward;
    Robosense::CanOdomDataPtr can_odom_data_ptr = std::make_shared<Robosense::CanOdomData>();
    can_odom_data_ptr->timestamp = msg_ptr->header.stamp.toSec();
    can_odom_data_ptr->vel << can_speed_mps_, 0.0, 0.0;
    std::unique_lock<std::mutex> lock(can_odom_buff_mutex_);
    can_odom_buff_[can_odom_data_ptr->timestamp] = can_odom_data_ptr;
    trimOdomQueue(can_odom_data_ptr->timestamp);
    if (!started_.load()) {
        init_can_ptr_ = can_odom_data_ptr;
        can_ready_.store(true);
        return;
    }
}
//user self adapt
void Ros1Interface::canOdomCallback(const nav_msgs::OdometryConstPtr& msg_ptr) {
    can_speed_mps_ = msg_ptr->twist.twist.linear.x;
    if (abs(can_speed_mps_) < 1e-4){
        cur_car_status_ = Robosense::stop;
    }
    else if (can_speed_mps_ > 1e-4)
        cur_car_status_ = Robosense::forward;
    else
        cur_car_status_ = Robosense::backward;
    Robosense::CanOdomDataPtr can_odom_data_ptr = std::make_shared<Robosense::CanOdomData>();
    can_odom_data_ptr->timestamp = msg_ptr->header.stamp.toSec();
    can_odom_data_ptr->vel << can_speed_mps_, 0.0, 0.0;
    std::unique_lock<std::mutex> lock(can_odom_buff_mutex_);
    can_odom_buff_[can_odom_data_ptr->timestamp] = can_odom_data_ptr;
    trimOdomQueue(can_odom_data_ptr->timestamp);
    if (!started_.load()) {
        init_can_ptr_ = can_odom_data_ptr;
        can_ready_.store(true);
        return;
    }
}

void Ros1Interface::lidarOdomCallback(const nav_msgs::OdometryConstPtr& msg_ptr) {
    auto lidar_odom_ptr = std::make_shared<Robosense::LidarOdomData>();
    lidar_odom_ptr->type = Robosense::LidarOdom;
    lidar_odom_ptr->timestamp = msg_ptr->header.stamp.toSec();
    lidar_odom_ptr->cur_car_status = cur_car_status_;
    Eigen::Matrix4d lidar_update_transform = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q;
    Eigen::Vector3d p{msg_ptr->pose.pose.position.x,
                      msg_ptr->pose.pose.position.y,
                      msg_ptr->pose.pose.position.z};
    q.x() = msg_ptr->pose.pose.orientation.x;
    q.y() = msg_ptr->pose.pose.orientation.y;
    q.z() = msg_ptr->pose.pose.orientation.z;
    q.w() = msg_ptr->pose.pose.orientation.w;
    double lidar_status = msg_ptr->pose.covariance[0];
    lidar_update_transform.block<3, 3>(0, 0) = q.matrix();
    lidar_update_transform.block<3, 1>(0, 3) = p;
    lidar_odom_ptr->pos = lidar_update_transform;
    lidar_odom_ptr->lidar_status = lidar_status;
    if (lidar_status == 2.) {
        eskf_interface_ptr_->addSensorBuff(lidar_odom_ptr);
        init_lidar_ptr_ = lidar_odom_ptr;
        lidar_ready_.store(true);
        LOG(INFO) << "add lidar at :" << std::fixed << lidar_odom_ptr->timestamp;
    }
}

void Ros1Interface::stateToRos(const Robosense::State& state) {
    fuse_path_.header.frame_id = "rslidar";
    fuse_path_.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header = fuse_path_.header;

    pose.pose.position.x = state.p_IinG[0];
    pose.pose.position.y = state.p_IinG[1];
    pose.pose.position.z = state.p_IinG[2];

    Eigen::Quaterniond G_q_I(state.R_G2I);
    G_q_I = G_q_I.normalized();
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    fuse_path_.poses.push_back(pose);
    // pub transform
    fuse_transform_.header.frame_id = "rslidar";
    fuse_transform_.child_frame_id = "fuse_data_tf";
    fuse_transform_.header.stamp = ros::Time::now();
    fuse_transform_.transform.translation.x = state.p_IinG[0];
    fuse_transform_.transform.translation.y = state.p_IinG[1];
    fuse_transform_.transform.translation.z = state.p_IinG[2];
    fuse_transform_.transform.rotation.x = G_q_I.x();
    fuse_transform_.transform.rotation.y = G_q_I.y();
    fuse_transform_.transform.rotation.z = G_q_I.z();
    fuse_transform_.transform.rotation.w = G_q_I.w();
}

void Ros1Interface::trimOdomQueue(double cur_time) {
    static const double MAX_DURATION = 100;
    while (true) {
        if (can_odom_buff_.size() <= 2)
            break;
        auto itr = can_odom_buff_.begin();
        if (itr->first < cur_time - MAX_DURATION)
            can_odom_buff_.erase(itr);
        else
            break;
    }
}
