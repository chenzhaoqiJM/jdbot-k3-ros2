# 安装依赖

```
sudo apt install python3-zmq
```


# 安装 YDLIDAR SDK

```
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
cmake --build . -- -j8
sudo cmake --install .
```


# 镭神激光雷达SDK参考
https://github.com/Lslidar/Lslidar_ROS2_driver

对于 M10P

```
git clone -b M10/M10-PHY_V1.0 https://github.com/Lslidar/Lslidar_ROS2_driver.git
```

修改参数文件以使用