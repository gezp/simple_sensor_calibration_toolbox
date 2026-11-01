# ssct_cam_imu_calib

相机-IMU 外参在线标定工具，通过 `config/calibration.yaml` 的 `calibrator_type` 选择标定器：

* `kalibr_calibrator`：Kalibr 风格的连续时间批量优化，用 B 样条位姿轨迹联合优化相机-IMU 外参、时间偏移、重力、IMU 零偏和位姿控制点。
* `handeye_calibrator`：用 IMU 静止段之间的相对位姿做 `A * X = X * B` 手眼标定，速度快，作为初值或快速检查。

标定板使用 OpenCV ArUco / ChArUco，不依赖 Kalibr 的 AprilGrid。相机模型支持 `pinhole`、`pinhole_radtan`、`ftheta_kb`。

## Quick Start

手机与 PC 连接到同一网络，并设置相同的 `ROS_DOMAIN_ID`。demo 订阅 `/camera/image_raw/compressed` 和 `/imu/data`：

```bash
ros2 launch ssct_cam_imu_calib phone_calib.launch.py
```

生成 ChArUco 标定板：

```bash
python3 ssct_cam_imu_calib/scripts/generate_charuco_board.py --output charuco.png
```

打印时保持 `square_length` 与实际尺寸一致，并保证整个板面平整。

Web 客户端：

```bash
ros2 launch ssct_calib_client web_client.launch.py mode:=cam_imu_calib
```

操作步骤：

1. 点击 `connect`，选择标定 topic。
2. 点击 `start`。
3. 保持标定板在图像中可见，充分激励相机的各个旋转轴。
4. 采集时长、关键帧数和旋转激励达到要求后自动优化，成功后点击 `save result` 保存。

默认配置使用 `kalibr_calibrator`。改用 `handeye_calibrator` 时把 `calibrator_type` 改为 `handeye_calibrator`，采集流程见下文。

## kalibr_calibrator

采集流程：

1. 不要求初始静止，点击 `start` 后直接开始采集。
2. 保持标定板在图像中可见，充分激励相机的各个旋转轴；平直缓慢的轨迹无法观测完整外参和时间偏移。
3. 采集时长、关键帧数和旋转激励达到要求后自动优化，成功后点击 `save result` 保存。

配置：

* `kalibr_calibrator.aruco_detector`：`charuco` 或 `gridboard` 的板参数和 ArUco 检测参数，板参数必须与实际打印板一致。
* `kalibr_calibrator.min_image_interval`：处理图像的最小间隔（秒）。
* `kalibr_calibrator.min_collection_duration`：自动开始优化的最短采集时长。
* `kalibr_calibrator.min_keyframes` / `min_rotation_deg`：观测筛选条件，在进入优化器之前由采集层检查。
* `kalibr_calibrator.optimizer`：优化器配置段，包括迭代次数、样条间隔、噪声、先验、时间偏移和重力估计开关。重力模长先验固定为 `9.80655`，与开源 Kalibr 默认一致。IMU 噪声密度读 `initial_calibration.yaml`。

初值流程与开源 Kalibr 一致，不依赖静止段：

1. 先用相机角速度和陀螺角速度的模长做时间偏移搜索（模长对旋转不变，不需外参旋转）；
2. 再用陀螺和相机角速度做小优化，得到相机-IMU 旋转和陀螺零偏；
3. 重力方向由世界系平均比力确定，模长固定为 `9.80655`；
4. 用标定板位姿和当前外参构造位姿样条初值，平移初值为零。

`resource/initial_calibration.yaml` 必须包含相机内参和 IMU 内参（含 `gyro_noise_density` / `accel_noise_density`），缺失时节点启动失败。文件中的相机-IMU 外参不参与 Kalibr 初始化。

## handeye_calibrator

采集流程：

1. 点击 `start` 后保持标定板可见，在多个不同位姿静止停留，每个位姿至少 `static_imu_collector.min_duration`（默认 0.5 s），位姿之间要有旋转变化。
2. 每个静止段独立估计本段的陀螺零偏和重力，作为从该段出发的 A→B 积分先验；各段互不依赖。
3. 相邻静止段的标定板观测间隔需小于 `max_pair_interval`（默认 5 s），有效对数达到 `min_pairs`。
4. 有效对数达到 `min_pairs` 后自动优化，成功后点击 `save result` 保存。

配置：

* `handeye_calibrator.static_imu_collector`：滑动窗口静止段采集参数（`window`、`gyro_std`、`accel_std`、`gravity_tolerance`、`min_duration`），用于构建 A→B 对；每段均值陀螺/加速度作为从该段出发的积分先验，各段独立。每个静止段取中心附近的标定板观测，相邻段组成一对 A→B。静止段由运动结束并封存在最后一个静止样本，长度达到 `min_duration` 才提交；提交后采集器自动重置并开始寻找下一段，无需手动 `reset`。
* `handeye_calibrator.max_pair_interval` / `min_pairs`：A→B 两次观测的最大间隔（默认 5 s，超时丢弃该对）和最少有效对数。等待图像的静止段队列固定上限 5 个，超出丢弃最旧的段。
* `handeye_calibrator.min_rotation_deg`：构建 pair 时过滤旋转不足的相邻静止段（不满足直接丢弃，不计入 `min_pairs`）。
* `handeye_calibrator.min_image_interval`：处理图像的最小间隔（秒）。
* `handeye_calibrator.optimizer`：一致性阈值和最大对数。手眼法不估计时间偏移（输出 0），平移来自每对相对运动的积分。

## 结果

输出沿用 `ssct_common` 的标定文件格式，保存相机内参、IMU 内参和外参：

```yaml
transforms:
    camera_tf_imu:
        frame_id: camera
        child_frame_id: imu
        translation: [...]
        rotation_xyzw: [...]
        time_offset: 0.001
```

* `T_camera_imu` 把 IMU 坐标系下的点变换到相机坐标系。
* 时间偏移约定：`t_imu = t_img + time_offset`，单位秒，正值表示图像时间戳滞后。

## 注意

* `mei` / `mei_radtan` 相机模型暂不支持。
* 两种方法都需要充分的旋转激励；纯平移或静止数据无法观测完整外参。
* `handeye_calibrator` 依赖多个静止段之间的相对运动，静止时标定板必须在图像中可见；`kalibr_calibrator` 不要求静止。
* 标定板必须保持静止，移动标定板等价于相机-IMU 之外多了一个未知位姿。
* 优化使用归一化去畸变点，因此 `pinhole_radtan` 和 `ftheta_kb` 共用同一套重投影残差。

## Reference

* Kalibr 开源实现：<https://github.com/ethz-asl/kalibr>
