# cuvslam_rgbd

这是一个面向 CPU/RVV 版本 cuVSLAM 的 ROS 2 RGB-D 里程计封装。

节点使用对齐后的彩色图像、深度图像、相机标定信息以及相机到机器人本体的 TF，可选接入合并后的 IMU，不依赖任何参考里程计，也不会执行轨迹对比、误差评估或结果文件输出。

## 功能

节点订阅：

- `/camera/color/image_raw`：彩色图像
- `/camera/aligned_depth_to_color/image_raw`：对齐到彩色相机的深度图像
- `/camera/color/camera_info`：彩色相机标定信息
- `/tf` 和 `/tf_static`：相机到机器人本体的外参

节点发布：

- `/cuvslam/odom`（`nav_msgs/msg/Odometry`）：cuVSLAM 里程计
- `/cuvslam/tracking`（`std_msgs/msg/Bool`）：当前帧是否输出有效位姿（启用 IMU 后可包含短时惯性恢复）

节点启动时通过 TF 获取 `rig_frame <- camera_color_optical_frame` 变换，并用它构建 cuVSLAM 相机 Rig。默认不发布里程计 TF，避免与底盘或其他定位节点产生 TF 冲突。

## 编译

系统 Python 环境需要已经安装可用的 `cuvslam` Python 包。

```bash
pip install cuvslam --break-system-packages
```

```bash
source /opt/ros/humble/setup.bash
source ~/jdbot_ws/install/setup.bash
cd ~/jdbot_ws
colcon build --symlink-install --packages-select cuvslam_rgbd
```

编译完成后重新加载工作空间：

```bash
source ~/jdbot_ws/install/setup.bash
```

## 启动

```bash
source ~/jdbot_ws/install/setup.bash
ros2 launch cuvslam_rgbd cuvslam_rgbd.launch.py
```

## 参数配置

所有配置均作为 launch 参数提供，不再使用 YAML 配置文件。例如：

```bash
ros2 launch cuvslam_rgbd cuvslam_rgbd.launch.py \
  output_odom_topic:=/visual_odom \
  rig_frame:=base_footprint \
  odom_frame:=visual_odom \
  depth_scale_factor:=1000.0 \
  sync_slop_s:=0.03 \
  publish_tf:=false
```

使用下面的命令查看全部参数及其默认值：

```bash
ros2 launch cuvslam_rgbd cuvslam_rgbd.launch.py --show-args
```

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `node_name` | `cuvslam_rgbd` | ROS 2 节点名称 |
| `color_topic` | `/camera/color/image_raw` | 彩色图像话题 |
| `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | 对齐后的深度图像话题 |
| `camera_info_topic` | `/camera/color/camera_info` | 相机标定信息话题 |
| `output_odom_topic` | `/cuvslam/odom` | 里程计输出话题 |
| `tracking_topic` | `/cuvslam/tracking` | 跟踪状态输出话题 |
| `rig_frame` | `base_footprint` | cuVSLAM Rig 对应的机器人坐标系 |
| `odom_frame` | `cuvslam_odom` | 里程计世界坐标系 |
| `depth_scale_factor` | `1000.0` | 深度值到米的换算比例；毫米深度图使用 1000 |
| `sync_slop_s` | `0.025` | 彩色与深度图像允许的最大时间差，单位为秒 |
| `sync_queue_size` | `30` | 图像同步队列长度 |
| `qos_depth` | `40` | 输入图像 QoS 队列深度 |
| `report_interval_s` | `5.0` | 运行状态日志输出周期，单位为秒 |
| `publish_tf` | `false` | 是否发布 `odom_frame -> rig_frame` TF |

影响订阅关系或 cuVSLAM Tracker 初始化的参数，需要在启动节点时指定；修改后应重新启动节点。

## RealSense D415 启动示例

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=/ \
  enable_color:=true \
  enable_depth:=true \
  rgb_camera.color_profile:=640,480,15 \
  depth_module.depth_profile:=640,480,15 \
  align_depth.enable:=true \
  enable_sync:=true
```

相机和机器人本体之间必须存在完整的 TF 链，例如：

```text
base_footprint -> base_link -> camera_link -> camera_color_optical_frame
```

## 输出检查

查看里程计：

```bash
ros2 topic echo /cuvslam/odom
```

查看输出频率：

```bash
ros2 topic hz /cuvslam/odom
```

查看跟踪状态：

```bash
ros2 topic echo /cuvslam/tracking
```

## CPU/RVV RGB-D + IMU

需要安装支持 CPU/RVV RGB-D IMU 融合的 cuvslam 后端。默认关闭 IMU，启用方式：

```bash
ros2 launch cuvslam_rgbd cuvslam_rgbd.launch.py \
  enable_imu:=true imu_topic:=/camera/imu imu_frequency:=200.0
```

RealSense 需要 `enable_gyro:=true enable_accel:=true unite_imu_method:=2`。
输入为合并后的 `sensor_msgs/Imu`，使用角速度和包含重力的加速度，不使用 orientation。
必须存在 `rig_frame <- IMU header.frame_id` 的 TF。启动后保持静止至少 1 秒，
日志中的 `IMU initialized=True` 表示初始化完成。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `enable_imu` | `false` | 启用后端 IMU 融合 |
| `imu_topic` | `/camera/imu` | 合并后的 IMU 话题 |
| `imu_frequency` | `200.0` | IMU 实际采样频率，Hz |
| `imu_queue_size` | `2000` | IMU 订阅和缓存上限 |
| `imu_wait_s` | `0.15` | 图像等候 IMU 时间戳赶上的最长时间，秒 |
| `gyroscope_noise_density` | `0.006067337` | gyro 噪声密度 |
| `gyroscope_random_walk` | `0.00003621195` | gyro bias 随机游走 |
| `accelerometer_noise_density` | `0.033621979` | accel 噪声密度 |
| `accelerometer_random_walk` | `0.0009825659` | accel bias 随机游走 |

噪声默认值来自后端 RealSense 示例，有独立标定结果时应替换。
节点按时间戳顺序串行提交 IMU 和图像，运行日志包含 IMU 提交、丢弃和图像丢弃计数。
影响订阅与 Tracker 初始化的参数需要在启动时指定，修改后重启节点。

当前为实验功能，仍存在位置与航向漂移。短时惯性恢复最多持续 0.25 秒，
持续视觉丢失超出恢复窗口后需重启跟踪；此扩展不提供全局重定位。
