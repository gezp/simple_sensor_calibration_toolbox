# Calibration File Format

该项目的标定结果采用yaml文件形式进行存储，目前支持的标定结果类型有：

* 相机内参
* 传感器与传感器之间的外参

其中相机内参保存在`cameras`列表中，传感器外参保存在`sensor_pair_transoforms`列表中。样例如下：

```yaml
cameras:
    camera1:
        frame_id: camera1
        height: 1200
        width: 1920
        type: pinhole_radtan
        intrinsics: [1057.79, 1059.8, 962.78, 581.29]
        distortion_coeffs: [-0.149116, 0.09615, -0.000526577, -0.000567049, -0.022971]
imus:
    imu1:
        frame_id: imu1
        accel_matrix: [1, 0, 0, 0, 1, 0, 0, 0, 1]
        accel_offset: [0, 0, 0]
        accel_noise_density: [1.86e-03, 1.86e-03, 1.86e-03]
        accel_random_walk: [4.33e-04, 4.33e-04, 4.33e-04]
        gyro_matrix: [1, 0, 0, 0, 1, 0, 0, 0, 1]
        gyro_offset: [0, 0, 0]
        gyro_noise_density: [1.87e-04, 1.87e-04, 1.87e-04]
        gyro_random_walk: [2.66e-05, 2.66e-05, 2.66e-05]
transforms:
    transform1:
        frame_id: camera1
        child_frame_id: lidar1
        translation: [0.07008565, -0.01771023, 0.00399246]
        rotation: [0.0 ,0.0 ,0.0 ,1.0] # x,y,z,w
    transform2:
        frame_id: lidar1
        child_frame_id: lidar2
        translation: [0.07008565, -0.01771023, 0.00399246]
        rotation: [0.0 ,0.0 ,0.0 ,1.0] # x,y,z,w
```

## 相机内参说明

相机内参包括以下三个量

* frame_id：传感器的frame id
* height，width：标定相机的图片分辨率
* type：采用的相机模型类型，为一个字符串
* intrinsics：相机内参系数，为一个float数组
* distortion_coeffs：畸变参数系数，为一个float数组

由于不同的相机模型，其内参系数和畸变参数的组成及含义也不同，目前仅支持以下几种相机模型类型:

* `pinhole_radtan `，`pinhole_equidistant `，`omni_radtan`

> 这里的相机模型可能与其它资料的定义不同，这里等价于某些资料中的相机模型+畸变模型。

`pinhole_radtan` (简写为`pinhole`)相机模型参数说明：

* intrinsics: $[f_x, f_y, c_x, c_y]$
* distortion_coeffs: $[k_1, k_2, p_1, p_2, k_3]$ ，其中k为径向畸变参数，p为切向畸变参数。
* 参考：https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html

`pinhole_equidistant` (简写为`fisheye`)相机模型参数说明：

* intrinsics: $[f_x, f_y, c_x, c_y]$
* distortion_coeffs: $[\theta_1, \theta_2, \theta_3, \theta_4]$ 
* 参考：https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html

`omni_radtan` (简写为`omni`或`omnidir`)相机模型参数说明：

* intrinsics: $[\xi, f_x, f_y, c_x, c_y]$
* distortion_coeffs: $[k_1, k_2, p_1, p_2, k_3]$ ，其中k为径向畸变参数，p为切向畸变参数。
* 参考：https://docs.opencv.org/4.x/dd/d12/tutorial_omnidir_calib_main.html


## Imu内参说明

IMU内参包括加速度计accel和陀螺仪gyro两组数据

* frame_id：传感器的frame id
* accel_matrix，accel_offset：速度计accel的仿射变换模型参数，修正测量值。
* accel_noise_density，accel_random_walk：速度计accel的白噪声以及零偏不稳定（随机游走）噪声。
* gyro_matrix，gyro_offset：陀螺仪gyro的仿射变换模型参数，修正测量值。
* gyro_noise_density，gyro_random_walk：陀螺仪gyro的白噪声以及零偏不稳定（随机游走）噪声。

速度计accel和陀螺仪gyro采用仿射变换建立确定性误差模型，修正scale,misalignment等误差，测量值修正公式如下：
$$
\hat x_{accel} = A_{accel}x_{accel} + b_{accel} \\
\hat x_{gyro} = A_{gyro}x_{gyro} + b_{gyro} \\
$$

* 其中$x$是原始测量值，$\hat x$是标定修正后的值，A对应matrix参数，b对应offset参数

random_walk和noise_density是随机误差（不确定性误差）模型参数，一般使用allan方差分析得到
* 参考：https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model


## 传感器外参说明

传感器的外参包括以下四个量

* frame_id：传感器的frame id
* child_frame_id：另一个传感器的frame id
* translation：外参坐标变换中的平移部分
* rotation：外参坐标变换中的旋转部分，采用四元数表示，顺序为`[x,y,z,w]`

这里标定出的坐标变换表示为 $T_\text{frame id - child frame id}$，等价以下两种说法：

* 在`frame_id`传感器坐标系下，`child_frame_id`的位姿。
* `child_frame_id`坐标系到`frame_id`坐标系下的位姿变换。
