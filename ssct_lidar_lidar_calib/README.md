# ssct_lidar_lidar_calib

Lidar-Lidar 外参手动标定工具。

标定节点负责两路点云的时间配对、发布标定初值并保存结果；双点云显示与外参调整在 Web 浏览器中完成。

## Quick Start

创建数据存放文件夹

```bash
mkdir ~/calibration_data
```

准备 demo 数据

* 在 `~/calibration_data/SensorsCalibration/lidar2lidar` 下准备 `top`、`left`、`right` 三个目录，每个目录放一份或多份 `.pcd` 点云（数据可参考 [SensorsCalibration](https://github.com/PJLab-ADG/SensorsCalibration) 的 lidar2lidar 数据集）。
* 初始外参 `config/initial_calibration.yaml` 默认对应 demo 数据的 `top_center_lidar`、`left_lidar`、`right_lidar`。

运行标定 demo

```bash
ros2 launch ssct_lidar_lidar_calib manual_calib_demo.launch.py source:=left
```

`source` 可选 `left` 或 `right`。

另开终端启动 Web 客户端

```bash
ros2 launch ssct_calib_client web_client.launch.py mode:=lidar_lidar_manual
```

浏览器打开 `http://<host>:8080`，在 `激光雷达手动标定` 模式下：

1. 点击 `connect`，选择标定 topic。
2. 点击 `start`，等待配对的 target/source 点云。
3. 使用平移/旋转按钮调整外参，source 点云会实时变换到 target 坐标系；可调点大小、着色和透明度。
4. 点击 `save result`，把当前外参写入结果文件。

## 配置

标定配置文件为 `config/calibrator.yaml`。

```yaml
sync:
    source_time_offset: 0.0  # 秒，t_source_corrected = t_source + offset
    max_time_diff: 0.05     # 秒，超过则视为不同步
    timeout: 5.0            # 秒，采集超时
```

* 两路点云使用 header stamp 配对，取时间差最小的一对；不同源时用 `source_time_offset` 补偿。
* 初始外参由 `config/initial_calibration.yaml` 提供，方向为 target -> source，web 端按 `T_target_source` 变换 source 点云。

## Topic

| Topic | 说明 |
|---|---|
| `~/status` | 标定状态，`sensor_topic`/`child_sensor_topic` 指向目标/源点云 |
| `~/command` | 标定命令 |
| `~/initial_params` | 标定初值（初始外参），latched |
| `~/result_params` | 保存时的标定结果 |
| `~/target_pointcloud` | 配对后的目标点云 |
| `~/source_pointcloud` | 配对后的源点云 |

`~/` 前缀为 `/calibration/ssct_lidar_lidar_calib/<target>_tf_<source>`。标定结果默认保存在 `~/calibration_data/result.yaml`。

## Reference

* https://github.com/PJLab-ADG/SensorsCalibration/tree/master/lidar2lidar/manual_calib
