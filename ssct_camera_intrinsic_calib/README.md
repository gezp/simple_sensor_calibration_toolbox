# ssct_camera_intrinsic_calib

这个包提供了相机内参标定的工具，目前已经实现的`calibrator`有：

* `pinhole_calibrator` ：标定`pinhole_radtan`相机模型，适用于镜头畸变较小的相机。
  * 目前仅支持棋盘格标定板。
* `fisheye_calibrator` (TODO)

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

* 默认情况下需要使用GUI客户端启动标定流程，如果想要标定程序自动开启标定流程并保存结果，并不使用GUI，可设置`calibration_node`的参数`autostart`为`True`。
* 标定的配置文件为`config/calibrator.yaml`，可以修改标定板的配置，相机的图像尺寸，误差阈值等参数。
* 标定结果保存在`~/calibration_data/esult.yaml`中。

运行标定GUI客户端

```bash
ros2 run ssct_camera_intrinsic_calib calibration_client.py
```

* 基于Qt的图形化界面，可以查看标定过程的标定信息，以及控制标定程序的流程。

## Reference

* https://github.com/PJLab-ADG/SensorsCalibration/tree/master/camera_intrinsic/intrinsic_calib
