## 安装依赖

```bash
sudo apt install python3-zmq python3-serial python3-transforms3d \
libspdlog-dev libconsole-bridge-dev liborocos-kdl-dev nlohmann-json3-dev liblgpio-dev \
liblttng-ust-dev libgpiod-dev ros-humble-camera-info-manager ros-humble-slam-toolbox \
ros-humble-cartographer ros-humble-cartographer-ros ros-humble-nav2* 'ros-humble-rtabmap*' \
ros-humble-aruco-markers-msgs ros-humble-realsense2-camera ros-dev-tools ros-humble-desktop \
udev git cmake ninja-build build-essential patchelf libboost-serialization-dev libssl-dev libsuitesparse-dev \
libglew-dev libepoxy-dev libx11-dev libwayland-dev libjpeg-dev libpng-dev libtiff-dev wget \
libeigen3-dev opencv-spacemit=4.14.0-2bb4
```

## 安装 YDLIDAR SDK

```bash
git clone https://github.com/chenzhaoqiJM/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
cmake --build . -- -j8
sudo cmake --install .
```

## 其他模块

```bash
git submodule update --init --recursive
```

## 编译说明

### RVV

环境准备

```bash
wget https://archive.spacemit.com/ros2/prebuilt_libs/bianbu26/opt/ext/orbslam3_rvv/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/orbslam3.tar.gz
sudo tar xzf orbslam3.tar.gz -C /opt
echo /opt/orbslam3/lib | sudo tee /etc/ld.so.conf.d/orbslam3.conf
sudo ldconfig
```

两个 ORB 节点的可执行文件会写入 `/opt/orbslam3/lib` 和
`/opt/opencv-spacemit/lib` 的 RPATH；仍需运行
`ldconfig`，以便预编译动态库的间接依赖也能被系统加载器找到。

编译

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`orbslam3_ros2` 默认启用 RVV，并自动使用 `/opt/orbslam3`、
`/opt/opencv-spacemit` 和 OpenCV 4.14 的 CMake 配置，因此无需设置
`RVV_FLAGS`、`CMAKE_PREFIX_PATH` 或 `OpenCV_DIR`。

### 非 RVV

环境准备

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
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_USE_RVV=OFF
```

### 跳过 orb slam 的编译

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip orbslam3_ros2
```
