# 里程计

要求底盘启动，静态 TF 完整

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

## 雷达说明

默认箭头标注方向为x正向，tf 的旋转全 0

```bash
ros2 launch jdbot_base_bringup start_lslidar_n10p.launch.py
```

ydlidar x3 pro，该配置为电机在后

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

使箭头标注方向为x正向，tf 的旋转全 0

```bash
ros2 launch jdbot_base_bringup start_rplidar.launch.py serial_port:=/dev/ttyUSB0 flip_x_axis:=true frame_id:=rplidar_link
```


## 雷达里程计

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

```bash
ros2 launch jdbot_base_bringup start_rplidar.launch.py serial_port:=/dev/ttyUSB1 flip_x_axis:=true frame_id:=rplidar_link
```

```bash
ros2 launch cartographer_lidar_odometry lidar_odometry.launch.py
```

## orb3 (RGBD) 里程计

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
settings:=$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/d415_rgbd.yaml \
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

或者配套

```bash
ros2 launch orbslam3_ros2 rgbd_odometry.launch.py \
    settings:=$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/d455_rgbd_1280x720_orb_640x360.yaml \
    resize_for_orb:=true \
    tracking_width:=640 \
    tracking_height:=360 \
    publish_tf:=true \
    odom_frame:=odom \
    base_frame:=base_footprint \
    color_topic:=/camera/color/image_raw \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    tracking_state_topic:=/localization/orb_tracking_state
```

```bash
ros2 run jdbot_slam tf_to_odom_node
```

测试时使用

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
ros2 run jdbot_base_bringup orbslam3_odom_to_tf --ros-args -p odom_topic:=/odom_rtabmap -p parent_frame:=odom -p child_frame:=base_footprint_rm
```

## cuvslam 里程计

```bash
ros2 launch cuvslam_rgbd cuvslam_rgbd.launch.py
```

更多信息见：cuvslam/cuvslam_rgbd/README.md
