# ssct_lidar_cam_manual_calib

这个包提供了Lidar-Camera外参手动标定的工具。


## Quick Start

创建数据存放文件夹

```bash
mkdir ~/calibration_data
```

准备demo数据

* 使用[SensorsCalibration](https://github.com/PJLab-ADG/SensorsCalibration)的数据，从Github上下载仓库，分别复制其中的`lidar2camera/manual_calib/data`目录中的图像和点云文件到到本地的`~/calibration_data/SensorsCalibration/lidar2camera`中`camera`和`lidar`目录即可。

运行标定demo程序

```bash
ros2 launch ssct_lidar_cam_manual_calib demo.launch.py 
```

* 标定的配置文件为`config/calibrator.yaml`，可以修改Lidar投影等参数。
* 初始的demo标定文件为`config/initial_calibration.yaml`。
* 标定结果保存在`~/calibration_data/result.yaml`中。

运行标定GUI客户端

```bash
ros2 run ssct_lidar_cam_manual_calib calibration_client.py
```

* 基于Qt的图形化界面，可以查看标定过程的标定信息，以及控制标定程序的流程。

## Reference

* https://github.com/PJLab-ADG/SensorsCalibration/tree/master/lidar2camera/manual_calib
