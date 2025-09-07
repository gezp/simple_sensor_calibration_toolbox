# simple sensor calibration toolbox (SSCT)

[![Build and Test](https://github.com/gezp/sensor_calibration/actions/workflows/ci.yml/badge.svg?branch=humble)](https://github.com/gezp/sensor_calibration/actions/workflows/ci.yml)

该项目基于ROS2平台实现了部分传感器标定功能，主要以学习为目的，面向初学者学习入门，因此代码侧重于可读性，及扩展性，尽可能将代码进行解耦，并遵循ROS2的项目规范，以及代码风格。

本项目主要面向以下场景的传感器标定：

* 传感器内参标定（sensor_intrinsics）
* 传感器到传感器之间的外参标定（sensor2sensor）

标定文件的存储形式：

* 采用YAML文件，数据格式详见[Calibration File Format](doc/calibration_file_format.md)文档

## Quick start

环境要求

* ROS版本: `Humble`

下载源码及安装依赖

```
# 使用git下载代码到ROS2工作空间的src目录
# mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/gezp/sensor_calibration.git
# 进入ROS2工作空间, 安装依赖
# cd  ~/ros2_ws
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
```

编译

```
colcon build --symlink-install
```

## Plan

传感器内参标定

- [x] 实现针孔相机内参标定: `ssct_camera_intrinsic_calib`
- [ ] 实现鱼眼相机内参标定: `ssct_camera_intrinsic_calib`
- [ ] 实现全景相机内参标定: `ssct_camera_intrinsic_calib`

sensor2sensor外参标定

- [ ] 实现手动lidar2lidar外参标定: `ssct_lidar_lidar_calib`
- [ ] 实现基于ICP的自动lidar2lidar外参标定: `ssct_lidar_lidar_calib`
- [x] 实现手动lidar2camera外参标定: `ssct_lidar_cam_manual_calib`


## Acknowledgement

本项目参考了许多其它类似项目，并参考并引用了部分代码，这里向以下项目的作者表示感谢！

* https://github.com/PJLab-ADG/SensorsCalibration
* https://github.com/tier4/CalibrationTools
* https://github.com/ethz-asl/kalibr
