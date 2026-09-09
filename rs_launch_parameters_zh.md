# `rs_launch.py` 参数中文说明

本文整理自 https://github.com/realsenseai/realsense-ros.git 下的 `rs_launch.py`。表中的“默认值”保持源码写法；命令行使用示例：

```bash
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true
```

> 说明：布尔参数通常取 `true` 或 `false`；流配置（profile）通常写成 `宽度,高度,帧率`，例如 `640,480,30`。默认值 `0,0,0` 表示不指定具体配置，由设备/驱动选择可用配置。`''` 表示空字符串，即未指定。

## 设备与启动设置

| 参数 | 默认值 | 中文说明 |
|---|---:|---|
| `camera_name` | `camera` | 相机节点的唯一名称。 |
| `camera_namespace` | `camera` | 相机节点使用的 ROS 命名空间。 |
| `serial_no` | `''` | 按设备序列号选择相机；为空时不按序列号筛选。多相机环境中可用它绑定指定设备。 |
| `usb_port_id` | `''` | 按 USB 端口 ID 选择相机；为空时不按端口筛选。 |
| `device_type` | `''` | 按设备型号/类型选择相机；为空时不按类型筛选。 |
| `config_file` | `''` | YAML 参数配置文件路径。文件中的参数会连同启动参数传给相机节点。 |
| `json_file_path` | `''` | RealSense 高级模式 JSON 配置文件路径，用于加载设备高级配置。 |
| `initial_reset` | `false` | 启动时是否先对设备执行硬件复位；设备连接异常时可能有用，但会增加启动时间。 |
| `accelerate_gpu_with_glsl` | `false` | 是否使用 GLSL 启用 GPU 加速；需要系统和构建环境支持。 |
| `rosbag_filename` | `''` | 将 RealSense bag 文件作为数据源；为空时使用实体设备。 |
| `rosbag_loop` | `false` | 播放 RealSense bag 文件时是否循环播放。 |
| `log_level` | `info` | 节点日志级别，可用值为 `DEBUG`、`INFO`、`WARN`、`ERROR`、`FATAL`（通常不区分大小写）。 |
| `output` | `screen` | 节点输出位置：`screen` 输出到终端，`log` 写入 ROS 日志。 |
| `wait_for_device_timeout` | `-1.` | 等待设备连接的超时时间，单位为秒；`-1` 表示持续等待、不超时。 |
| `reconnect_timeout` | `6.` | 设备断开后，相邻两次重连尝试之间的等待时间，单位为秒。 |

## 彩色、深度与红外流

| 参数 | 默认值 | 中文说明 |
|---|---:|---|
| `enable_color` | `true` | 是否启用彩色图像流。 |
| `rgb_camera.color_profile` | `0,0,0` | 彩色流配置，格式为 `宽度,高度,帧率`；`0,0,0` 表示使用驱动/设备默认配置。 |
| `rgb_camera.color_format` | `RGB8` | 彩色流像素格式。默认使用 8 位 RGB 三通道格式。 |
| `rgb_camera.enable_auto_exposure` | `true` | 是否为彩色图像启用自动曝光。关闭后可使用设备支持的手动曝光设置。 |
| `enable_depth` | `true` | 是否启用深度图像流。 |
| `enable_infra` | `false` | 是否启用索引为 0 的红外流（infra0）；具体支持情况取决于设备。 |
| `enable_infra1` | `false` | 是否启用索引为 1 的红外流（infra1）。 |
| `enable_infra2` | `false` | 是否启用索引为 2 的红外流（infra2）。 |
| `depth_module.depth_profile` | `0,0,0` | 深度流配置，格式为 `宽度,高度,帧率`；`0,0,0` 表示使用默认配置。 |
| `depth_module.depth_format` | `Z16` | 深度流像素格式；`Z16` 为常用的 16 位深度格式。 |
| `depth_module.infra_profile` | `0,0,0` | 红外流（infra0/1/2）共用的配置，格式为 `宽度,高度,帧率`。 |
| `depth_module.infra_format` | `RGB8` | infra0 红外流的像素格式。 |
| `depth_module.infra1_format` | `Y8` | infra1 红外流的像素格式；`Y8` 为 8 位灰度格式。 |
| `depth_module.infra2_format` | `Y8` | infra2 红外流的像素格式；`Y8` 为 8 位灰度格式。 |
| `depth_module.color_profile` | `0,0,0` | D405 深度模组彩色流配置，格式为 `宽度,高度,帧率`。 |
| `depth_module.color_format` | `RGB8` | D405 深度模组彩色流的像素格式。 |
| `depth_module.exposure` | `8500` | 深度模组的手动曝光值；有效范围和单位由具体设备决定。自动曝光开启时，该值通常不会直接生效。 |
| `depth_module.gain` | `16` | 深度模组的手动增益值；有效范围取决于设备。自动曝光开启时，该值通常不会直接生效。 |
| `depth_module.enable_auto_exposure` | `true` | 是否为深度图像启用自动曝光。 |
| `enable_sync` | `false` | 是否同步各传感器数据流，使不同流按时间戳配对。 |
| `depth_module.inter_cam_sync_mode` | `0` | 多相机硬件同步模式：`0` 默认、`1` 主相机（Master）、`2` 从相机（Slave）。仅在支持该功能的设备上有效。 |
| `enable_rgbd` | `false` | 是否发布组合的 RGB-D 主题；通常需要彩色流和深度流，并配合深度对齐使用。 |
| `clip_distance` | `-2.` | 深度裁剪距离，单位为米；超过该距离的深度像素会被裁剪。负值表示禁用裁剪。 |

## HDR 设置

| 参数 | 默认值 | 中文说明 |
|---|---:|---|
| `depth_module.hdr_enabled` | `false` | 是否启用深度模组 HDR。与 HDR 合并滤镜配合使用。 |
| `depth_module.exposure.1` | `7500` | HDR 序列中第一组曝光值。 |
| `depth_module.gain.1` | `16` | HDR 序列中第一组增益值。 |
| `depth_module.exposure.2` | `1` | HDR 序列中第二组曝光值。 |
| `depth_module.gain.2` | `16` | HDR 序列中第二组增益值。 |
| `hdr_merge.enable` | `false` | 是否启用 HDR 合并滤镜，将 HDR 序列帧合成为深度结果。 |

## IMU 与运动数据

| 参数 | 默认值 | 中文说明 |
|---|---:|---|
| `enable_gyro` | `false` | 是否启用陀螺仪数据流。 |
| `enable_accel` | `false` | 是否启用加速度计数据流。 |
| `enable_motion` | `false` | 是否为 DDS 设备启用统一的运动（IMU）数据流。 |
| `gyro_fps` | `0` | 陀螺仪采样率（Hz）；`0` 表示由驱动/设备选择默认值。 |
| `accel_fps` | `0` | 加速度计采样率（Hz）；`0` 表示由驱动/设备选择默认值。 |
| `motion_fps` | `0` | DDS 设备运动流的每秒采样数；`0` 表示使用默认值。 |
| `unite_imu_method` | `0` | IMU 数据合并方式：`0` 不合并，`1` 复制（copy），`2` 线性插值（linear_interpolation）。用于将不同频率的陀螺仪和加速度计数据组合到统一 IMU 消息中。 |
| `angular_velocity_cov` | `0.01` | IMU 消息中角速度的协方差值，用于表达测量不确定性。 |
| `linear_accel_cov` | `0.01` | IMU 消息中线加速度的协方差值，用于表达测量不确定性。 |

## TF、诊断与坐标系

| 参数 | 默认值 | 中文说明 |
|---|---:|---|
| `diagnostics_period` | `0.0` | 发布诊断信息的周期/频率控制值；源码说明中 `0` 表示禁用。 |
| `publish_tf` | `true` | 是否发布传感器间的静态和动态 TF 坐标变换。 |
| `tf_publish_rate` | `0.0` | 动态 TF 发布频率，单位为 Hz；`0.0` 表示不周期发布动态 TF。静态 TF 不受此频率影响。 |
| `base_frame_id` | `link` | 传感器 TF 树的根坐标系名称。最终名称还会结合相机名/前缀规则生成。 |
| `tf_prefix` | 空字符串 | 添加到所有 frame ID 前面的前缀，适合多相机系统避免坐标系重名。 |

## 点云与图像后处理滤镜

| 参数 | 默认值 | 中文说明 |
|---|---:|---|
| `pointcloud.enable` | `false` | 是否根据深度数据生成并发布点云。 |
| `pointcloud.stream_filter` | `2` | 点云纹理所使用的数据流类型编号；默认 `2` 通常对应彩色流。可用编号取决于 librealsense 的流类型定义。 |
| `pointcloud.stream_index_filter` | `0` | 点云纹理流的索引；用于同一类型存在多个流时指定具体流。 |
| `pointcloud.ordered_pc` | `false` | 是否生成有序点云。启用时通常保留与深度图一致的二维组织结构，无效点也会保留占位。 |
| `pointcloud.allow_no_texture_points` | `false` | 是否保留无法映射到纹理图像的点；`false` 表示过滤这些点。 |
| `align_depth.enable` | `false` | 是否启用深度对齐滤镜，将深度图对齐到目标图像流（通常为彩色流）。 |
| `colorizer.enable` | `false` | 是否启用深度着色滤镜，把深度值转换为便于观察的彩色图像。 |
| `decimation_filter.enable` | `false` | 是否启用抽取滤镜，通过降低深度图分辨率减少数据量。 |
| `decimation_filter.filter_magnitude` | `2` | 抽取滤镜倍率；数值越大，输出分辨率通常越低。有效范围由 librealsense/设备版本决定。 |
| `rotation_filter.enable` | `false` | 是否启用图像旋转滤镜。 |
| `rotation_filter.rotation` | `0.0` | 旋转角度，可取 `0.0`、`90.0`、`-90.0` 或 `180.0`。 |
| `spatial_filter.enable` | `false` | 是否启用空间滤镜，在单帧邻域内平滑深度并降低噪声。 |
| `temporal_filter.enable` | `false` | 是否启用时间滤镜，利用连续帧信息平滑深度并提高稳定性。 |
| `disparity_filter.enable` | `false` | 是否启用深度/视差转换滤镜。某些深度滤波流程会在视差域中处理数据。 |
| `hole_filling_filter.enable` | `false` | 是否启用孔洞填充滤镜，填补深度图中的部分无效像素。 |

## 安全、标记点云与占用数据

| 参数 | 默认值 | 中文说明 |
|---|---:|---|
| `occupancy_max_range` | `2.5` | 占用栅格射线投射的最大可信距离，单位为米；超过此距离的单元格记为未知（`-1`）。 |
| `enable_safety` | `false` | 是否启用安全数据流；仅适用于支持该流的设备。 |
| `safety_camera.safety_mode` | `0` | 安全相机模式：`0` 运行（Run）、`1` 待机（Standby）、`2` 服务（Service）。 |
| `enable_labeled_point_cloud` | `false` | 是否启用带标签点云数据流；仅适用于支持该功能的设备。 |
| `depth_mapping_camera.labeled_point_cloud_profile` | `0,0,0` | 带标签点云流配置，格式为 `宽度,高度,帧率`。 |
| `enable_occupancy` | `false` | 是否启用占用数据流；仅适用于支持该功能的设备。 |
| `depth_mapping_camera.occupancy_profile` | `0,0,0` | 占用数据流配置，格式为 `宽度,高度,帧率`。 |

## 配置文件与参数优先级

`rs_launch.py` 会先声明上述启动参数，再把启动参数和 `config_file` 指定的 YAML 内容一起传给 `realsense2_camera_node`。节点参数列表中启动参数在前、YAML 参数在后，因此同名项通常以 YAML 文件中的值为准。

启动脚本还会检查命令行和 YAML 中的参数名；不在上述 80 个参数中的名称会打印黄色警告。节点最终以普通 ROS 节点还是生命周期节点启动，由 `realsense2_camera/config/global_settings.yaml` 中的 `use_lifecycle_node` 决定。

## 使用示例

启用 640×480、30 FPS 的彩色和深度流，并开启对齐及点云：

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640,480,30 \
  depth_module.depth_profile:=640,480,30 \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

按序列号启动指定相机，并为 TF 坐标系添加前缀：

```bash
ros2 launch realsense2_camera rs_launch.py \
  serial_no:="'123456789'" \
  camera_name:=camera_1 \
  camera_namespace:=camera_1 \
  tf_prefix:=camera_1_
```

> 注意：相机型号、固件和 USB 带宽不同，可用的分辨率、帧率、格式及功能也不同。若设置设备不支持的组合，节点可能无法启动相应数据流。
