# ssct_camera_intrinsic_calib

这个包提供了相机内参标定的工具，通过配置文件`config/calibrator.yaml`中的`calibrator_type`选择标定器：

* `opencv_calibrator`：标定`pinhole_radtan`相机模型（针孔 + 径向/切向畸变），基于`cv::calibrateCamera`，适用于普通镜头。
* `opencv_fisheye_calibrator`：标定`ftheta_kb`鱼眼相机模型（Kannala-Brandt），基于`cv::fisheye::calibrate`。
* `opencv_omnidir_calibrator`：标定`mei_radtan`全景/全向相机模型（Mei），基于`cv::omnidir::calibrate`。

每个标定器的配置写在与其同名的配置段中，例如 `opencv_calibrator:`。目前仅支持棋盘格标定板。

## Quick Start

创建数据存放文件夹

```bash
mkdir ~/calibration_data
```

准备demo数据

* 使用[SensorsCalibratio](https://github.com/PJLab-ADG/SensorsCalibration)的数据，从Github上下载仓库，复制其中的`camera_intrinsic/intrinsic_calib/data`目录到本地的`~/calibration_data/SensorsCalibration/camera_intrinsic`中即可。

运行标定demo程序

```bash
ros2 launch ssct_camera_intrinsic_calib demo.launch.py 
```

* 默认情况下需要使用 Web 客户端启动标定流程，如果想要标定程序自动开启标定流程并保存结果，并在浏览器中查看进度，可设置`calibration_node`的参数`autostart`为`True`。
* 标定的配置文件为`config/calibrator.yaml`，可以修改标定板的配置，相机的图像尺寸，误差阈值等参数。
* 标定结果保存在`~/calibration_data/result.yaml`中。

另开终端启动 Web 客户端，浏览器打开 `http://<host>:8080`：

```bash
ros2 launch ssct_calib_client web_client.launch.py mode:=camera_intrinsic_calib
```

## Reference

* https://github.com/PJLab-ADG/SensorsCalibration/tree/master/camera_intrinsic/intrinsic_calib
