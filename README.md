## 安装依赖

```
sudo apt install python3-zmq python3-serial python3-transforms3d \
libspdlog-dev libconsole-bridge-dev liborocos-kdl-dev nlohmann-json3-dev liblgpio-dev \
liblttng-ust-dev libgpiod-dev
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