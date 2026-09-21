## 基本命令

### 底盘

```bash
ros2 launch jdbot_base_bringup start_control_esp32_c6.launch.py wheel_base:=0.270 wheel_diameter:=0.065
```

或

```bash
ros2 launch jdbot_base_bringup start_control_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.270 wheel_diameter:=0.065
```

```bash
ros2 launch jdbot_slam cartographer_odom.launch.py
```

### 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 雷达

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

## 2d 激光建图

启动激光雷达

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

### slam_toolbox

```bash
ros2 launch jdbot_slam online_async_launch.py
```

### cartographer

```bash
ros2 launch jdbot_base_bringup start_control_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.270 wheel_diameter:=0.065
```

```bash
ros2 launch jdbot_slam cartographer.launch.py
```

### 保存

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 2d 激光导航

启动激光雷达

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

```bash
cp my_map.* ~/jdbot_ws/install/jdbot_navigation/share/jdbot_navigation/map/
```

```bash
ros2 launch jdbot_navigation lidar_nav2.launch.py
```

## rgbd 建图

realsense 相机

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/
```

底盘

```bash
ros2 launch jdbot_base_bringup start_control_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.185 wheel_diameter:=0.065 min_angular_speed:=0.4
```

雷达里程计

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

```bash
ros2 launch jdbot_slam cartographer_odom.launch.py
```

建图

```bash
ros2 launch jdbot_slam rtabmap_rgbd.launch.py
```

或

```bash
ros2 launch jdbot_slam rtabmap_rgbd_icp_odom.launch.py # 无需雷达里程计
```

导航模式

```bash
ros2 launch jdbot_slam rtabmap_rgbd.launch.py localization:=true
```

或

```bash
ros2 launch jdbot_slam rtabmap_rgbd_icp_odom.launch.py localization:=true # 无需雷达里程计
```

```bash
ros2 launch jdbot_navigation nav2_rtabmap.launch.py controller_type:=mpc # 默认rpp
```

### rtabmap icp 里程计

雷达、相机、底盘同上
