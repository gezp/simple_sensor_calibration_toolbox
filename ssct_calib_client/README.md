# ssct_calib_client

基于 Web 的标定客户端。通过 ROS2 topic 与标定节点交互，不负责启动或停止节点。

## 依赖

```bash
sudo apt install python3-aiohttp
```

## 启动

```bash
ros2 launch ssct_calib_client web_client.launch.py
```

浏览器打开 `http://<host>:8080`。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `host` | `0.0.0.0` | 监听地址 |
| `port` | `8080` | HTTP 端口 |
| `mode` | `generic` | 初始模式：`generic` / `camera_intrinsic` / `lidar_cam_manual` / `lidar_lidar_manual` |

## 模式

| 模式 | 标题栏 |
|---|---|
| `generic` | 通用标定 |
| `camera_intrinsic` | 相机内参标定 |
| `lidar_cam_manual` | 激光相机手动标定 |
| `lidar_lidar_manual` | 激光雷达手动标定 |

点击 `connect` 选择 topic，点击 `disconnect` 断开。切换模式会重置当前连接和状态。通用标定模式下连接已实现的标定类型时，自动切换到对应模式。

## 扩展

新增 `webui/modes/<id>/index.js`，导出 `meta`（含 `label`、`calibrationType`）和 `mount`，并在 `scripts/web_client.py` 的 `MODES` 中注册，顺序即列表顺序。

## 安全

客户端不做鉴权。默认监听 `0.0.0.0`，同一局域网的任何人都能控制标定并覆盖结果文件，仅可在可信网络使用。
