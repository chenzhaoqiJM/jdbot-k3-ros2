## 安装依赖

```
sudo apt install python3-zmq python3-serial python3-transforms3d \
libspdlog-dev libconsole-bridge-dev liborocos-kdl-dev nlohmann-json3-dev liblgpio-dev \
liblttng-ust-dev libgpiod-dev ros-humble-camera-info-manager ros-humble-slam-toolbox \
ros-humble-cartographer ros-humble-cartographer-ros ros-humble-nav2*
```


## 安装 YDLIDAR SDK

```
git clone https://github.com/chenzhaoqiJM/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
cmake --build . -- -j8
sudo cmake --install .
```


## 镭神激光雷达SDK参考
https://github.com/Lslidar/Lslidar_ROS2_driver

对于 M10P

```
git clone -b M10/M10-PHY_V1.0 https://github.com/Lslidar/Lslidar_ROS2_driver.git
```

修改参数文件以使用


## 3D SLAM

```
mkdir ~/hesai_jt128_ws && cd  ~/hesai_jt128_ws
git clone https://github.com/chenzhaoqiJM/HesaiLidar_ROS_2.0.git
cd HesaiLidar_ROS_2.0
git submodule update --init --recursive
```

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

### 启动激光雷达

```bash
ros2 launch jdbot_base_bringup start_ydlidar.launch.py
```

## 建图

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

## 导航

```bash
cp my_map.* ~/jdbot_ws/install/jdbot_navigation/share/jdbot_navigation/map/
```

```bash
ros2 launch jdbot_navigation lidar_nav2.launch.py
```