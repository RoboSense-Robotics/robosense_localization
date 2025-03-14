# localization

[README](README.md) | [中文文档](README_CN.md)

## 1. 简介
**localization** 用于提供实时的相对定位信息或全局定位信息,该项目包含了以下两个模块：

`msf_localization` 是基于**ESKF**框架的融合定位模块，可提供只基于**imu+轮速**的相对定位信息以及融合**点云定位结果**的全局定位信息，该模块也可接入其他传感器观测进行后续拓展

`lidar_localization` 是基于点云地图的点云定位模块，使用ceres进行优化，使用pcl进行点云处理

## 2. 前置依赖

此项目基于 `ros2 humble` + `python 3.8.10` 进行开发测试

`msf_localization` 模块和 `lidar_localization` 模块依赖 `ros2` 以及 `package.xml` 中定义的依赖项

`msf_localization` 依赖glog记录log信息 

**Ubuntu:** 
```bash
sudo apt install libgoogle-glog-dev
```

### 2.1 安装 ros2

根据您的操作系统选择 [官方教程](https://fishros.org/doc/ros2/humble/Installation.html) 中的指定内容进行执行

### 2.2 安装 python3 [可选]

您可以遵循您正在使用的平台的 Python 安装教程来安装 Python3，推荐使用 `3.8.10` 以上的版本

* **Windows:** 下载并执行 https://www.python.org/ftp/python/3.8.10/python-3.8.10-amd64.exe 即可完成安装


* **Ubuntu:** 
```bash 
sudo apt-get install -y python3 python3-pip
```


## 3. 安装部署

### 3.1 代码拉取

您可以创建一个新的文件夹或进入您现有的 `ros2` 工作空间，执行以下命令将代码拉取到工作空间内

```bash
git clone git@gitlab.robosense.cn:super_sensor_sdk/ros2_sdk/localization.git -b main
```
### 3.3 编译 localization

在您的工作空间下执行以下命令来编译安装 `localization`

```bash
colcon build --symlink-install 
```

编译安装完成后，推荐刷新一下工作空间的 `bash profile`，确保组件功能正常

```bash
source install/setup.bash
```

## 4. 使用方式

### 4.1 运行定位节点

运行融合定位节点

```bash
ros2 run msf_localization msf_localization_node 
```

运行点云定位节点
```bash
ros2 run lidar_localization lidar_localization_node
```

### 4.2 Rviz可视化

```bash
rviz2 -d msf_localization/rviz/rviz2_config.rviz
```
### 4.3 所需传感器消息

* `msf_localization` : imu消息， 轮速消息， 点云定位结果（可选）， 用户拓展的其他传感器消息（可选）

* `lidar_localization` : lidar点云消息， 融合定位结果

### 4.4 融合定位初始化

初始位置默认是 `[0, 0, 0]`， 初始姿态默认是单位矩阵， 用户若已知初始时刻在点云地图中的位姿，或能通过rtk等外部设备获取到初始时刻的位姿， 可通过修改配置文件

`msf_localization/config/loc_param.yaml`：

初始位置: `x_init, y_init, z_init`

初始姿态: `roll_init, pitch_init, yaw_init`


### 4.5 准备数据集

* `点云地图` : 由某些开源建图软件如fast-lio或其他建图软件生成的点云地图pcd文件

* `示例数据路径` : 

## 5. 功能说明

### 5.1 配置文件

融合定位相关配置文件：msf_localization/config/loc_param.yaml

点云定位相关配置文件：lidar_localization/config/config.yaml

## 6. FAQ

[Create New Issue](http://gitlab.robosense.cn/super_sensor_sdk/ros2_sdk/localization/-/issues/new)
