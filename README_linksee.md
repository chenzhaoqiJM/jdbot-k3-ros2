# linksee 机器人建图导航测试

## 底盘

```bash
ros2 launch jdbot_base_bringup start_control_linglong_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.185 wheel_diameter:=0.065 min_angular_speed:=0.4
```

## realsense 相机

```bash
cd ~
wget https://archive.spacemit.com/ros2/prebuilt_libs/source_code_common/librealsense-2.57.4.tar.gz
tar xzvf librealsense-2.57.4.tar.gz
cd librealsense-2.57.4/
./scripts/setup_udev_rules.sh
cd ..
rm -rf librealsense-2.57.4 librealsense-2.57.4.tar.gz
```

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640,480,15 \
  depth_module.depth_profile:=640,480,15 \
  align_depth.enable:=true enable_sync:=true
```

若 HID 驱动已经配置, 添加以下选项开启 IMU 的数据

```bash
enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=1280,720,10 \
  depth_module.depth_profile:=848,480,10 \
  align_depth.enable:=true enable_sync:=true
```

## 雷达里程计

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

```bash
ros2 launch jdbot_base_bringup start_rplidar.launch.py serial_port:=/dev/ttyUSB1 flip_x_axis:=true frame_id:=rplidar_link
```

```bash
ros2 launch jdbot_slam cartographer_odom.launch.py
```

## orb3 (RGBD) 里程计

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
  publish_tf:=true \
  odom_frame:=odom \
  base_frame:=base_footprint \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  tracking_state_topic:=/localization/orb_tracking_state
```

odom tf 转 /odom

```bash
ros2 run jdbot_slam tf_to_odom_node
```

测试命令

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
  settings:=$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/d415_rgbd.yaml
```

```bash
ros2 run jdbot_base_bringup orbslam3_odom_to_tf
```

## rtabmap 里程计

```bash
ros2 launch jdbot_slam rtabmap_rgbd_odom.launch.py
```

use_imu:=true 参数默认订阅 imu_topic:=/camera/imu

测试用

```bash
ros2 launch jdbot_slam rtabmap_rgbd_odom.launch.py odom_topic:=/odom_rtabmap publish_tf:=false
```

```bash
ros2 run jdbot_base_bringup orbslam3_odom_to_tf \
  --ros-args \
  -p odom_topic:=/odom_rtabmap \
  -p parent_frame:=odom \
  -p child_frame:=base_footprint_rm
```

## 建图

```bash
ros2 launch jdbot_slam rtabmap_rgbd.launch.py
```

与 rtabmap 里程计配合时

```bash
ros2 launch jdbot_slam rtabmap_rgbd.launch.py launch_point_cloud_xyz:=false
```

## 导航

```bash
ros2 launch jdbot_navigation nav2_rtabmap.launch.py controller_type:=mpc # 默认rpp
```