## 安装依赖

```
sudo apt install python3-zmq python3-serial python3-transforms3d \
libspdlog-dev libconsole-bridge-dev liborocos-kdl-dev nlohmann-json3-dev liblgpio-dev \
liblttng-ust-dev libgpiod-dev ros-humble-camera-info-manager ros-humble-slam-toolbox \
ros-humble-cartographer ros-humble-cartographer-ros ros-humble-nav2* 'ros-humble-rtabmap*' ros-humble-aruco-markers-msgs
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

## 编译说明

### RVV

环境准备

```bash
sudo apt install git cmake ninja-build build-essential patchelf \
  libeigen3-dev opencv-spacemit=4.14.0-2bb4 \
  libboost-serialization-dev libssl-dev libsuitesparse-dev \
  libglew-dev libepoxy-dev libx11-dev libwayland-dev \
  libjpeg-dev libpng-dev libtiff-dev
```

```bash
wget https://archive.spacemit.com/ros2/prebuilt_libs/bianbu26/opt/ext/orbslam3_rvv/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/orbslam3.tar.gz
sudo tar xzf orbslam3.tar.gz -C /opt
echo /opt/orbslam3/lib | sudo tee /etc/ld.so.conf.d/orbslam3.conf
sudo ldconfig
```

编译

```bash
export RVV_C_FLAGS="-O3 -DNDEBUG -march=rv64gcv_zvl256b -mrvv-vector-bits=zvl"
export RVV_CXX_FLAGS="$RVV_C_FLAGS -DEIGEN_RISCV64_USE_RVV10"
export OPENCV_PREFIX=/opt/opencv-spacemit
export OPENCV_DIR="$OPENCV_PREFIX/lib/cmake/opencv4"

source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
export CMAKE_PREFIX_PATH="/opt/orbslam3:/opt/opencv-spacemit:$CMAKE_PREFIX_PATH"
colcon build --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="$RVV_C_FLAGS" -DCMAKE_CXX_FLAGS_RELEASE="$RVV_CXX_FLAGS" -DOpenCV_DIR="$OPENCV_DIR"
```

### 非 RVV

环境准备

```bash
sudo apt install -y git cmake ninja-build build-essential patchelf \
  libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
  libsuitesparse-dev libglew-dev libepoxy-dev \
  libx11-dev libwayland-dev libjpeg-dev libpng-dev libtiff-dev
```

```bash
wget https://archive.spacemit.com/ros2/prebuilt_libs/bianbu26/opt/ext/orbslam3/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/orbslam3.tar.gz
sudo tar xzf orbslam3.tar.gz -C /opt
echo /opt/orbslam3/lib | sudo tee /etc/ld.so.conf.d/orbslam3.conf
sudo ldconfig
```

编译

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
export CMAKE_PREFIX_PATH=/opt/orbslam3:${CMAKE_PREFIX_PATH}
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

### 跳过 orb slam 的编译

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip orbslam3_ros2 orbslam3_slam_ros2
```


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
ros2 launch jdbot_base_bringup start_rplidar.launch.py serial_port:=/dev/ttyUSB1 flip_x_axis:=true
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

