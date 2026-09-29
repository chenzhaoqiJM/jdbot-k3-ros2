# k3_rgbd_odometry

面向 K3 RISC-V 与 RealSense D415 的轻量 ROS 2 纯 RGB-D 里程计原型。核心不依赖
OpenCV、PCL、Ceres 或 TBB，仅使用 Eigen 与 ROS 2/TF2。

## 当前状态

- 可在 amd64 与 K3 riscv64 上以 Release 模式编译。
- 支持 `rgb8`、`bgr8`、`mono8` 彩色输入与 `16UC1` 对齐深度。
- 自动从 TF 获取 `base_footprint <- camera_color_optical_frame` 外参。
- 发布平面约束的里程计、TF、跟踪状态与内点数。
- K3 真机 640×480@15 Hz 下，实测跟踪与 odom 输出约 `14.3–14.9 Hz`；
  20 秒逐秒 CPU 峰值 `38%`，内存约 `35–36 MB`，低于单核 100% 上限。
- 已加入轻量关键帧、BRIEF 描述子、RGB-D SE(2) RANSAC、位姿图优化和优化轨迹发布。
- 三份闭环数据的完整优化图平移 RMSE 为 `0.132 / 0.230 / 0.247 m`，均优于
  同包 cuVSLAM 的 `0.155 / 0.247 / 0.262 m`。

## 算法结构

1. 自实现灰度转换和四层图像金字塔；
2. 分块 Shi–Tomasi 角点；
3. 粗层小窗口块匹配与亚像素 KLT；
4. 深度恢复 3D，对应点 RANSAC 刚体初值；
5. 鲁棒重投影与地面车 `x/y/yaw` 三自由度优化；
6. 短关键帧相对跟踪、按累计视觉行程自动切换逐帧模式，并补偿短时跟踪缺口；
7. 异常增量拒绝、协方差与跟踪质量输出；
8. 按位移/转角创建持久关键帧，自研 256-bit BRIEF 描述子检索；
9. 由匹配特征的 RGB-D 三维点执行 SE(2) RANSAC，验证后加入小型位姿图；
10. 发布经过回环优化的完整关键帧 `nav_msgs/Path`。

源码中保留了实验性的直接光度、深度 ICP 与虚拟二维扫描校正函数，便于继续消融；
生产路径目前没有串联深度 ICP/虚拟扫描，因为离线结果显示它们会引入额外偏置。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
colcon build --packages-select k3_rgbd_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动

相机必须发布已经对齐到彩色相机的深度，并且 TF 树中应能查询
`base_footprint <- camera_color_optical_frame`。

```bash
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py
```

### 常用 launch 参数

默认值针对 D415 当前的 640×480、15 Hz 配置。下列常用项可直接在启动命令中覆盖，
未列出的高级调优参数继续从 `config/k3_d415.yaml` 读取。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `config_file` | 包内 `k3_d415.yaml` | 切换整套 YAML 配置 |
| `namespace` / `node_name` | 空 / `k3_rgbd_odometry` | 节点命名空间和名称 |
| `color_topic` | `/camera/color/image_raw` | 彩色图像话题 |
| `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | 对齐到彩色相机的深度话题 |
| `camera_info_topic` | `/camera/color/camera_info` | 彩色相机内参话题 |
| `odom_topic` | `/rgbd_odom/odom` | 里程计输出话题 |
| `odom_frame` | `odom_rgbd` | 里程计父坐标系 |
| `base_frame` | `base_footprint` | 查询相机外参时使用的机器人基座坐标系 |
| `child_frame` | `base_footprint_rgbd` | 里程计和 TF 的子坐标系 |
| `publish_tf` | `true` | 是否发布 TF |
| `processing_rate_hz` | `20.0` | 最大处理频率；相机 15 Hz 时实际输出约 15 Hz |
| `expected_frame_rate_hz` | `15.0` | 丢帧恢复所依据的相机帧率 |
| `sync_tolerance_ms` | `40.0` | 彩色与深度帧最大时间差 |
| `min_depth_m` / `max_depth_m` | `0.25` / `6.0` | 有效深度范围 |
| `max_features` | `420` | 最大特征数，降低该值可进一步节省 CPU |
| `loop_closure_enabled` | `true` | 是否启用轻量级回环校正 |
| `use_sim_time` | `false` | 回放 rosbag 时使用 `/clock` |

例如，直接发布标准的 `odom -> base_footprint`，并将处理上限设为 15 Hz：

```bash
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py \
  odom_frame:=odom child_frame:=base_footprint \
  processing_rate_hz:=15.0
```

查看完整参数列表：

```bash
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py --show-args
```

主要输出：

- `/rgbd_odom/odom`：`nav_msgs/msg/Odometry`
- `/k3_rgbd_odometry/tracking`：当前帧是否可靠
- `/k3_rgbd_odometry/inliers`：重投影内点数
- `/k3_rgbd_odometry/loop_closure`：本帧是否接受了回环
- `/k3_rgbd_odometry/optimized_path`：回环后重新优化的完整关键帧轨迹
- `odom_rgbd -> base_footprint_rgbd`：默认 TF，避免与参考数据冲突

真机接入导航前，可把配置中的 `odom_frame` 与 `child_frame` 改成系统所需名称。

## 离线基线

仓库提供的三圈参考数据中，cuVSLAM 相对激光里程计的平移 RMSE 约为
`0.155 / 0.247 / 0.262 m`，回到起点的平移残差约为
`0.339 / 0.238 / 0.199 m`。这些数值是后续版本必须同时达到或超过的验收线。

辅助工具：

```bash
source /opt/ros/humble/setup.bash
python3 tools/extract_bag_reference.py rgbd_odom_data_1 analysis/data_1
python3 tools/evaluate_trajectory.py analysis/data_1/tf.csv
```

## 已知边界

当前指标来自仓库中的三组 D415/K3 路线。更换相机安装外参、深度尺度、底盘或场景后，
应重新采集闭环路线验证 `forward_gain`、自动模式切换阈值和回环参数；节点不使用激光或
参考里程计作为运行输入。
