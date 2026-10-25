# ssct_lidar_cam_calib

Lidar-Camera 外参手动标定工具。

标定节点负责图像与点云的时间配对、图像去畸变、发布标定初值并保存结果；投影叠加与位姿调整在 Web 浏览器中完成。

## Quick Start

创建数据存放文件夹

```bash
mkdir ~/calibration_data
```

准备 demo 数据

* 使用 [SensorsCalibration](https://github.com/PJLab-ADG/SensorsCalibration) 的数据，从 Github 上下载仓库，分别复制其中的 `lidar2camera/manual_calib/data` 目录中的图像和点云文件到本地的 `~/calibration_data/SensorsCalibration/lidar2camera` 中 `camera` 和 `lidar` 目录即可。

运行标定 demo

```bash
ros2 launch ssct_lidar_cam_calib manual_calib_demo.launch.py
```

另开终端启动 Web 客户端

```bash
ros2 launch ssct_calib_client web_client.launch.py mode:=lidar_cam_manual_calib
```

浏览器打开 `http://<host>:8080`，在 `激光相机手动标定` 模式下：

1. 点击 `connect`，选择标定 topic。
2. 点击 `start`，等待去畸变图像和配对点云。
3. 使用平移/旋转按钮调整外参，点云会实时叠加到图像上；可调点大小、深度/强度着色和透明度。
4. 点击 `save result`，把当前外参写入结果文件。

## 配置

标定配置文件为 `config/calibrator.yaml`。

```yaml
sync:
    image_time_offset: 0.0  # 秒，t_cam_corrected = t_cam + offset
    max_time_diff: 0.05     # 秒，超过则视为不同步
    timeout: 5.0            # 秒，采集超时
```

* 去畸变模型由源相机模型内置决定：`pinhole_radtan` 转 `pinhole`，`ftheta_kb`（别名 `fisheye`）保持 `ftheta_kb`；`mei_radtan` 不支持。
* 点云与图像使用 header stamp 配对；两路时间必须同源，不同源时用 `image_time_offset` 补偿。

## Topic

| Topic | 说明 |
|---|---|
| `~/status` | 标定状态，`sensor_topic`/`child_sensor_topic` 指向去畸变图和配对点云 |
| `~/command` | 标定命令 |
| `~/initial_params` | 标定初值（目标相机模型 + 初始外参），latched |
| `~/result_params` | 保存时的标定结果 |
| `~/image` | 去畸变图 |
| `~/pointcloud` | 配对点云 |

标定结果默认保存在 `~/calibration_data/result.yaml`。

## Reference

* https://github.com/PJLab-ADG/SensorsCalibration/tree/master/lidar2camera/manual_calib
