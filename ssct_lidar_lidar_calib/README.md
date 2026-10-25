# ssct_lidar_lidar_calib

Lidar-Lidar 外参标定工具，支持手动与自动标定。

标定节点负责两路点云的时间配对、发布标定初值并保存结果。手动模式的双点云显示与外参调整在 Web 浏览器中完成；自动模式基于地面约束、yaw 搜索和法向 ICP 直接求解外参。

## Quick Start

创建数据存放文件夹

```bash
mkdir ~/calibration_data
```

准备 demo 数据

* 在 `~/calibration_data/SensorsCalibration/lidar2lidar` 下准备 `top`、`left`、`right` 三个目录，每个目录放一份或多份 `.pcd` 点云（数据可参考 [SensorsCalibration](https://github.com/PJLab-ADG/SensorsCalibration) 的 lidar2lidar 数据集）。
* 初始外参 `config/initial_calibration.yaml` 默认对应 demo 数据的 `top_center_lidar`、`left_lidar`、`right_lidar`。

运行手动标定 demo

```bash
ros2 launch ssct_lidar_lidar_calib manual_calib_demo.launch.py source:=left
```

`source` 可选 `left` 或 `right`。

另开终端启动 Web 客户端

```bash
ros2 launch ssct_calib_client web_client.launch.py mode:=lidar_lidar_manual_calib
```

浏览器打开 `http://<host>:8080`，在 `激光雷达手动标定` 模式下：

1. 点击 `connect`，选择标定 topic。
2. 点击 `start`，等待配对的 target/source 点云。
3. 使用平移/旋转按钮调整外参，source 点云会实时变换到 target 坐标系；可调点大小、着色和透明度。
4. 点击 `save result`，把当前外参写入结果文件。

### 自动标定

```bash
ros2 launch ssct_lidar_lidar_calib demo.launch.py source:=left
```

另开终端启动通用 Web 客户端（默认 `generic` 模式）：

```bash
ros2 launch ssct_calib_client web_client.launch.py
```

浏览器打开 `http://<host>:8080`，`connect` 到 `ssct_lidar_lidar_calib` topic：

1. 点击 `start`，节点等待一对同步点云。
2. 自动求解完成后状态进度为 99，点击 `save result` 写入结果文件。

自动标定流程：

1. RANSAC 提取 target/source 地面平面。
2. 对齐两地面法向量，并按 target 地面高度修正 source 外参。
3. 以 yaw 为唯一变量由粗到细搜索，最近邻距离最小者为最优 yaw。
4. 法向 ICP 精配准，得到最终 `T_target_source`。

## 配置

手动标定配置文件为 `config/manual_calibration.yaml`：

```yaml
sync:
    source_time_offset: 0.0  # 秒，t_source_corrected = t_source + offset
    max_time_diff: 0.05     # 秒，超过则视为不同步
    timeout: 5.0            # 秒，采集超时
```

自动标定配置文件为 `config/calibration.yaml`：

```yaml
sync:
    source_time_offset: 0.0
    max_time_diff: 0.05
    timeout: 5.0
simple_calibrator:
    near_range: 1.0                          # 米，地面提取前剔除 source 原点附近点
    ground_distance_threshold: 0.2           # 米，地面 RANSAC 内点阈值
    ground_min_points: 100                   # 地面/非地面最少点数
    yaw_search_range: 10.0                   # 度，初始 yaw 搜索范围
    yaw_step: 5.0                            # 度，初始 yaw 搜索步长
    yaw_iterations: 5                        # yaw 由粗到细搜索轮数
    normal_k_search: 40                      # 法向估计近邻数
    icp_max_iterations: 10
    icp_max_correspondence_distance: 0.3     # 米
```

* 两路点云使用 header stamp 配对，取时间差最小的一对；不同源时用 `source_time_offset` 补偿。
* 初始外参由 `config/initial_calibration.yaml` 提供，方向为 target -> source，web 端按 `T_target_source` 变换 source 点云。

## Topic

| Topic | 说明 |
|---|---|
| `~/status` | 标定状态，`sensor_topic`/`child_sensor_topic` 指向目标/源点云 |
| `~/command` | 标定命令 |
| `~/initial_params` | 标定初值（初始外参），latched |
| `~/result_params` | 标定结果（手动由 Web 发布，自动求解后发布） |
| `~/target_pointcloud` | 配对后的目标点云 |
| `~/source_pointcloud` | 配对后的源点云 |

手动标定前缀为 `/calibration/ssct_lidar_lidar_manual_calib/<target>_tf_<source>`，自动标定前缀为 `/calibration/ssct_lidar_lidar_calib/<target>_tf_<source>`。自动模式求解完成后会发布 `~/result_params`。标定结果默认保存在 `~/calibration_data/result.yaml`。

## Reference

* https://github.com/PJLab-ADG/SensorsCalibration/tree/master/lidar2lidar/manual_calib
* https://github.com/PJLab-ADG/SensorsCalibration/tree/master/lidar2lidar/auto_calib
