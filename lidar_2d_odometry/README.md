# 轻量级 2D 激光雷达里程计

`lidar_2d_odometry` 是一个独立的 ROS 2 二维激光雷达里程计节点，采用激光帧到局部地图的匹配方式估计机器人位姿。除 ROS 2 组件外，仅依赖 Eigen，不依赖 Ceres、PCL 或 RF2O。

匹配器使用恒速模型提供初始位姿估计，通过带 Huber 鲁棒核的点到线 ICP 完成扫描匹配，并使用数量受限的关键帧维护局部地图。激光点会通过 TF 转换到配置的跟踪坐标系，因此算法实现不依赖固定的激光雷达安装位姿。

## 编译与运行

```bash
cd ~/jdbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_2d_odometry
source install/setup.bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py
```

可在启动命令中覆盖话题、坐标系等参数，例如：

```bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py \
  tracking_frame:=base_link odom_frame:=laser_odom
```

## 输入与输出

- 输入：`/scan`（`sensor_msgs/msg/LaserScan`）
- 输出：`/odom`（`nav_msgs/msg/Odometry`）
- TF：默认发布 `odom -> base_footprint`，可通过 `publish_tf:=false` 关闭

回放包含已有里程计或 TF 的数据包时，应避免话题和 TF 冲突。例如：

```bash
ros2 launch lidar_2d_odometry lidar_2d_odometry.launch.py \
  publish_tf:=false odom_topic:=/lidar_odom use_sim_time:=true
ros2 bag play <验收数据包路径> --clock
```

## 参数配置

匹配器参数位于 `config/lidar_2d_odometry.yaml`，主要包括：

- 激光有效距离：`min_range`、`max_range`
- 点云降采样：`voxel_size`
- 匹配约束：`correspondence_distance`、`huber_scale`
- 迭代控制：`max_iterations`、`min_correspondences`
- 关键帧判定：`keyframe_translation`、`keyframe_rotation`
- 局部地图大小：`max_keyframes`
- 速度滤波：`velocity_filter_alpha`

局部地图中的关键帧数量由 `max_keyframes` 限制，因此长时间运行时的内存占用和近邻查询开销均保持有界。

