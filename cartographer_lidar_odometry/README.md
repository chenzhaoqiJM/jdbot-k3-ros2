# Cartographer 2D 激光里程计

该功能包仅运行 Cartographer 的 2D 局部轨迹构建器，用于通过 2D 激光雷达
计算里程计。它不会创建 `MapBuilder`、`PoseGraph`，也不包含回环检测、全局
优化、地图服务或占据栅格地图发布功能。

## 输入

- `/scan`（`sensor_msgs/msg/LaserScan`）
- 从 `base_footprint` 到激光雷达坐标系的静态 TF；随附测试 bag 使用的激光
  坐标系为 `laser_link`

## 输出

- `/odom`（`nav_msgs/msg/Odometry`），发布频率与激光匹配频率一致
- 当 `publish_tf:=true` 时发布 `odom -> base_footprint` TF

## 编译和运行

```bash
cd ~/jdbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cartographer_lidar_odometry
source install/setup.bash
ros2 launch cartographer_lidar_odometry lidar_odometry.launch.py
```

## 使用 bag 验证

测试 bag 中已经包含原 Cartographer 发布的里程计 TF，因此回放时应关闭新节点
的 TF 发布，并将里程计输出重映射到 `/scan_odom`，避免与原数据冲突：

```bash
ros2 launch cartographer_lidar_odometry lidar_odometry.launch.py \
  use_sim_time:=false publish_tf:=false odom_topic:=/scan_odom
ros2 bag play ~/carto_odom_test_data
ros2 topic hz /scan_odom
```

## 源码和兼容性说明

功能包中内置的局部 SLAM 源码来自 Cartographer 2.0.9004 的
`bianbu-26.04` 分支，与机器人上安装的 ROS Humble 软件包版本一致。

局部轨迹构建器、2D 子图与栅格、扫描匹配器以及 Ceres 代价函数会在本功能包
中一起编译，以避免在 Bianbu GCC 15 工具链上跨越 Cartographer 不稳定的内部
C++ ABI。位姿图、回环检测、全局优化和建图服务均未包含在本功能包中。

如果机器人上安装的 Cartographer ABI 版本发生变化，需要从相同版本的源码中
同步更新 `vendor/cartographer_2.0.9004` 下的内置文件。
