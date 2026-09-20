# ORB-SLAM3 RGB-D 里程计（D455 / ROS 2 Humble）

本包使用 Intel RealSense D455 运行无界面的 ORB-SLAM3 RGB-D 里程计，并在
`/orbslam3/odom` 发布米制里程计，在 `/orbslam3/tracking_state` 发布跟踪状态。
跟踪状态为 `2` 表示跟踪正常。里程计可执行程序名为
`orbslam3_odometry_node`；同一包还提供 `orbslam3_slam_node`，建图用法参见
[`README-SLAM.md`](README-SLAM.md)。

## 运行方法

使用经过实机验证、兼容 USB 2 的 640×480@15 配置启动 D455：

```bash
source /opt/ros/humble/setup.bash
source ~/jdbot_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640,480,15 \
  depth_module.depth_profile:=640,480,15 \
  align_depth.enable:=true enable_sync:=true
```

在另一个终端启动 ORB-SLAM3：

```bash
source /opt/ros/humble/setup.bash
source ~/orbslam3_ws/install/setup.bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py
```

所有节点参数均可在 launch 命令行中覆盖。例如启用 TF 并修改坐标系名称：

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
  publish_tf:=true \
  odom_frame:=odom \
  base_frame:=base_link \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  odom_topic:=/localization/orb_odom \
  tracking_state_topic:=/localization/orb_tracking_state
```

查看全部可配置参数及其默认值：

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py --show-args
```

## D415 640×480

`d415_rgbd.yaml` 使用设备序列号 `036422061119` 在 640×480@15 下实际发布的
彩色相机内参。D415 标定数据给出的红外双目基线为 0.0549711 m，深度单位为
0.001 m。里程计启动命令为：

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
  settings:=$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/d415_rgbd.yaml
```

建图时使用对应的 `d415_rgbd_slam.yaml`：

```bash
ros2 launch orbslam3_ros2 rgbd_slam.launch.py \
  settings:=$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/d415_rgbd_slam.yaml
```

快速检查运行状态：

```bash
ros2 topic echo /orbslam3/tracking_state --once
ros2 topic hz /orbslam3/odom
ros2 topic echo /orbslam3/odom --once
```

## 1280×720 导航图像与低分辨率 ORB 跟踪

当前 USB 2.1 连接下，实测可用的高分辨率组合为彩色 1280×720@10、深度
848×480@10；对齐后的深度图为 1280×720。导航模块可以直接订阅原始高分辨率
话题，ORB 节点则在内部把彩色图以面积插值、深度图以最近邻插值缩小到
640×360：

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=1280,720,10 \
  depth_module.depth_profile:=848,480,10 \
  align_depth.enable:=true enable_sync:=true
```

```bash
source /opt/ros/humble/setup.bash
source ~/jdbot_ws/install/setup.bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
  settings:=$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/d455_rgbd_1280x720_orb_640x360.yaml \
  resize_for_orb:=true \
  tracking_width:=640 \
  tracking_height:=360
```

不要将现有 640×480 内参直接用于 1280×720 图像。上述配置中的内参来自本机
D455 的 1280×720 `camera_info`，并按 0.5 比例缩放到 640×360。缩放只发生在
ORB 节点内部，不会改变导航模块收到的原始图像。

launch 文件中有意将 `publish_tf` 设置为 `false`，这样可以与 Cartographer
里程计进行对比，同时避免多个节点为 `base_footprint` 发布不同的 TF 父节点。

ORB-SLAM3 和 Pangolin 已安装到 `/opt/orbslam3`。动态库目录通过
`/etc/ld.so.conf.d/orbslam3.conf` 注册，因此不需要设置指向用户 home 目录的
`LD_LIBRARY_PATH`。只有使用其他依赖安装前缀时，才需要在启动前设置
`ORB_SLAM3_PREFIX`。

完整的编译和部署过程参见 [`../编译与分发说明.md`](../编译与分发说明.md)。

## 当前硬件连接情况

当前 D455 被枚举为 USB 2.1 设备。Librealsense 2.57.7 报告
`No HID info provided, IMU is disabled`，且当前连接无法提供 ORB-SLAM3
双目惯性模式所需的两路 Y8 红外图像。因此目前经过验收的是 RGB-D 模式，
本包暂不使用 IMU。
