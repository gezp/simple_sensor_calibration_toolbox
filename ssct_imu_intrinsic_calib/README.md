# ssct_imu_intrinsic_calib

## 总览

IMU 内参在线标定工具。一个 `calibration_node`，通过配置项 `calibrator_type` 选择标定方法：

| `calibrator_type`   | 标定内容                           |
| --------------------- | ---------------------------------- |
| `allan_calibrator`  | `noise_density`、`random_walk` |
| `imu_tk_calibrator` | accel/gyro 的 `matrix`、`bias` |

两种标定相互独立，每次保存只写本次标定得到的字段，输出文件整体覆盖。需要同时得到随机误差和确定性内参时，分别运行并保存到不同文件。

输出沿用 `ssct_common` 的标定文件格式：

```yaml
imus:
    imu1:
        frame_id: imu
        accel_matrix: [...]
        accel_bias: [...]
        accel_noise_density: [...]
        accel_random_walk: [...]
        gyro_matrix: [...]
        gyro_bias: [...]
        gyro_noise_density: [...]
        gyro_random_walk: [...]
```

| 字段                                | Allan  | IMU-TK |
| ----------------------------------- | ------ | ------ |
| `accel_matrix` / `gyro_matrix`  | 单位阵 | 估计值 |
| `accel_bias` / `gyro_bias`      | 零     | 估计值 |
| `noise_density` / `random_walk` | 估计值 | 零     |

## Quick Start

**Demo 数据**

包内自带测试数据 `resource/uncalibrated_imu_data.csv`，来自 [imu_tk test_data](https://github.com/Kyle-ak/imu_tk/tree/master/bin/test_data)，为其中部分数据的裁剪版本，通过 `dummy_imu_node` 按指定频率回放，无需额外准备数据。支持普通 IMU 数据和原始计数数据。

`demo.launch.py` 使用 IMU-TK 配置，启动回放节点和标定节点：

```bash
ros2 launch ssct_imu_intrinsic_calib demo.launch.py
ros2 launch ssct_calib_client web_client.launch.py mode:=imu_intrinsic_calib
```

* Web 客户端默认监听 `0.0.0.0:8080`，浏览器打开 `http://localhost:8080`，页面中点击开始标定。

**真实传感器**

```bash
# Allan 方差
ros2 launch ssct_imu_intrinsic_calib allan_calib.launch.py

# IMU-TK
ros2 launch ssct_imu_intrinsic_calib tk_imu_calib.launch.py

# Web 客户端
ros2 launch ssct_calib_client web_client.launch.py mode:=imu_intrinsic_calib
```

Web 客户端同时显示传感器 topic 的实时 IMU 曲线和标定 debug 图，两个 launch 文件默认订阅 `/imu/data`。

## Allan 方差标定

保持 IMU 完全静止，使用 Allan 方差估计随机误差参数。相关代码参考：[Kalibr](https://github.com/ethz-asl/kalibr)。

### 标定流程

1. 启动 `allan_calib.launch.py`，在 Web 客户端点击 `connect` 并选择标定 topic。
2. 保持 IMU 完全静止，点击 `start`。
3. 每隔 `plot_interval` 自动计算一次 Allan 方差并刷新曲线，此时结果仅用于显示，不能保存。
4. 采集时长达到 `duration` 后自动计算最终结果并结束采集。
5. 点击 `save result` 保存。

### 采集要求

- 传感器和线缆固定牢靠，采集全程保持完全静止。
- 不要碰桌子、移动或敲击传感器。
- 采样率稳定、时间戳递增；采集期间不要重启节点或传感器。
- `duration` 建议至少 300 秒；random walk 更依赖长时间数据，条件允许时使用数小时数据。

### 配置

`config/allan_calibration.yaml`：

* `calibrator_type`：`allan_calibrator`。
* `allan_calibrator.duration`：必须采集的静止时长，单位秒；达到后计算最终结果并结束。
* `allan_calibrator.plot_interval`：采集期间每隔多少秒计算一次并刷新曲线；中间结果不能保存。
* `allan_calibrator.num_clusters`：Allan 曲线 cluster 数量，log 间隔。
* `allan_calibrator.min_samples`：最少有效样本数，不足时最终计算失败。

默认 `duration` 为 30 分钟、`plot_interval` 为 5 分钟：5、10、15、20、25 分钟显示中间曲线，30 分钟输出最终结果并结束。

### 算法

1. 用重叠 Allan 方差计算每条轴的 Allan deviation 曲线。
2. 白噪声区拟合斜率 `-1/2`，得到 noise density。
3. 随机游走区拟合斜率 `+1/2`，得到 random walk。
4. 若数据中没有明显的随机游走上升区，random walk 保存为零，状态信息会给出提示。

字段单位：

* `accel_noise_density`：`m/s²/√Hz`；采样周期 `dt` 下单样本标准差为 `noise_density / √dt`
* `accel_random_walk`：`m/s³/√Hz`；`t` 秒后零偏标准差为 `random_walk * √t`，单步增量标准差为 `random_walk * √dt`
* `gyro_noise_density`：`rad/s/√Hz`；采样周期 `dt` 下单样本标准差为 `noise_density / √dt`
* `gyro_random_walk`：`rad/s²/√Hz`；`t` 秒后零偏标准差为 `random_walk * √t`，单步增量标准差为 `random_walk * √dt`

### 输出

更新 `noise_density`、`random_walk`；`matrix` 写单位阵，`bias` 写零。

## IMU-TK 标定

使用 IMU-TK 多位置法估计确定性内参，无需转台，相关代码参考：[imu_tk](https://github.com/Kyle-ak/imu_tk)。

### 标定流程

1. 启动 `tk_imu_calib.launch.py` 或 `demo.launch.py`，在 Web 客户端点击 `connect` 并选择标定 topic。
2. 点击 `start`，依次转动到至少 `min_intervals` 个不同姿态；状态信息会提示当前应保持还是换位置，以及需要保持的秒数。
3. 采集到足够的静止姿态后自动计算，状态变为成功。
4. 点击 `save result` 保存。

### 采集要求

- 传感器和线缆固定牢靠，采集时避免碰桌子或拖动线缆。
- 每个姿态保持静止 `static_window + interval_samples / sample_rate` 秒；默认 `0.5 + 100 / 100 = 1.5` 秒。
- 新姿态与所有已采集姿态的重力方向夹角至少 `15°`，重复姿态不计入。
- 至少采集 `min_intervals` 个不同姿态，默认 12 个；需要更高精度时增大该值，例如 15 或 20。
- 状态里的 `static poses: N / M` 表示已采集的有效姿态数和目标数，第一个姿态也会立即计入。

### 状态提示

| 提示                                             | 含义                            | 操作                               |
| ------------------------------------------------ | ------------------------------- | ---------------------------------- |
| `KEEP STILL 0.8 / 1.5 s`                       | 正在静止倒计时                  | 保持不动，等待本姿态采集完成       |
| `MOVE NOW: pose N / M captured ...`            | 当前姿态有效并已计入            | 转动到下一个姿态，再保持静止       |
| `MOVE: rotate >15 deg ...`                     | 检测到正在转动，尚未静止        | 转到与之前不同的方向后保持静止     |
| `INVALID POSE: same orientation ...`           | 静止有效，但与已有姿态重复      | 重新摆放到夹角超过 `15°` 的姿态 |
| `INVALID STATIC: re-place ...`                 | 检测到静止，但姿态无效          | 重新摆放传感器并保持静止           |
| `NO STATIC POSE: check mounting/vibration ...` | 超过 3 倍保持时间仍未检测到静止 | 检查安装和振动，重新摆放后保持静止 |

无效姿态不会一直等待：静止时长达到要求后仍无效会立即提示 `INVALID POSE`；没有检测到静止超过约 4.5 秒会提示 `NO STATIC POSE`。已经成功的标定不会再重复计算，直接等待保存。

### 配置

`config/imu_tk_calibration.yaml`：

* `calibrator_type`：`imu_tk_calibrator`。
* `imu_tk_calibrator.gravity_magnitude`：当地重力加速度，默认 `9.81744`。
* `imu_tk_calibrator.static_window`：静止区间检测窗口，默认 0.5 秒。
* `imu_tk_calibrator.interval_samples`：每个静止姿态取用的样本数，默认 100。
* `imu_tk_calibrator.min_intervals`：最少静止姿态数，默认 12；增大该值可采集更多姿态提高精度。

### 算法

1. 用加速度计方差检测静止区间：以全程最安静窗口的方差为噪声基准，遍历阈值倍数，取加速度残差最小的结果；重复姿态只计一次。
2. 加速度计模型 `a = T * K * (raw - b)`，其中 `T` 为下三角安装误差矩阵，`K` 为尺度矩阵，使用重力模长作为约束。支持原始计数输入，偏置和尺度由数据自动初始化。
3. 陀螺仪模型 `w = T * K * (raw - b)`，使用相邻静止姿态之间的重力方向变化约束陀螺积分；所有静止区间的陀螺均值估计零偏。
4. 保存形式为 `calibrated = matrix * (raw - bias)`，`bias` 与原始测量值同单位。

### 输出

更新 `accel_matrix`、`accel_bias`、`gyro_matrix`、`gyro_bias`；`noise_density`、`random_walk` 写零，需单独做 Allan 标定。
