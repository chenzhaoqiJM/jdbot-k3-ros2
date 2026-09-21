## 基本命令

### 底盘

```bash
ros2 launch jdbot_base_bringup start_control_esp32_c6.launch.py wheel_base:=0.270 wheel_diameter:=0.065
```

或

```bash
ros2 launch jdbot_base_bringup start_control_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.270 wheel_diameter:=0.065
```

详见 b-里程计.md

```bash
ros2 launch cartographer_lidar_odometry lidar_odometry.launch.py
```

### 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

启动底盘 -> 启动激光雷达 -> 启动里程计

```bash
cp my_map.* ~/jdbot_ws/install/jdbot_navigation/share/jdbot_navigation/map/
```

```bash
ros2 launch jdbot_navigation lidar_nav2.launch.py
```

## rgbd 建图

启动 realsense 相机

启动底盘 -> 启动激光雷达（可选） -> 启动里程计（雷达或rgbd）

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/
```

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

## rgbd 导航

需要开 rgbd 建图节点

```bash
ros2 launch jdbot_navigation nav2_rtabmap.launch.py controller_type:=mpc # 默认rpp
```
