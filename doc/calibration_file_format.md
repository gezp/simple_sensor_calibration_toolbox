# Calibration File Format

该项目的标定结果采用yaml文件形式进行存储，目前支持的标定结果类型有：

* 相机内参
* 传感器与传感器之间的外参

其中相机内参保存在`cameras`列表中，传感器外参保存在`sensor_pair_transoforms`列表中。样例如下：

```yaml
cameras:
    camera1:
        frame_id: camera1
        type: pinhole_radtan
        intrinsics: [461.629, 460.152, 362.680, 246.049]
        distortion_coeffs: [-0.27695497, 0.06712482, 0.00087538, 0.00011556, 0.0]
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

## 传感器外参说明

传感器的外参包括以下四个量

* frame_id：传感器的frame id
* child_frame_id：另一个传感器的frame id
* translation：外参坐标变换中的平移部分
* rotation：外参坐标变换中的旋转部分，采用四元数表示，顺序为`[x,y,z,w]`

这里标定出的坐标变换表示为 $T_\text{frame id - child frame id}$，等价以下两种说法：

* 在`frame_id`传感器坐标系下，`child_frame_id`的位姿。
* `child_frame_id`坐标系到`frame_id`坐标系下的位姿变换。

## 相机内参说明

相机内参包括以下三个量

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

