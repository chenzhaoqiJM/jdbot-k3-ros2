# jdbot_base_bringup

本包使用 `ament_cmake` 构建，通过 `ament_cmake_python` 同时安装 Python 模块。
包名、现有 launch/config 路径以及原先注册的四个 Python 节点命令保持不变。

## 目录

- `src/`：C++ 节点源码。
- `include/jdbot_base_bringup/`：C++ 头文件。
- `scripts/`：Python 模块和节点；安装后的 Python 包名仍为 `jdbot_base_bringup`。
- `launch/`、`config/`：启动文件和配置。
- `CMakeLists.txt`：统一管理构建和安装。

## 构建

在工作空间根目录执行（按实际 ROS 2 发行版调整环境路径）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select jdbot_base_bringup --symlink-install
source install/setup.bash
```

如果工作空间已经构建过旧的 `ament_python` 版本，首次迁移建议先清理该包的
`build/jdbot_base_bringup` 和 `install/jdbot_base_bringup`，然后重新构建。
这里的安装目录适用于默认的隔离安装布局；使用 `--merge-install` 时应使用新的
build/install 目录，避免旧 Python 入口或包元数据残留。

## Fast LIO2 平面里程计转换

```bash
ros2 launch jdbot_base_bringup fastlio_odom.launch.py
# 或直接使用节点默认参数
ros2 run jdbot_base_bringup fastlio_odom_converter_node
```

节点订阅 `/Odometry`（`nav_msgs/msg/Odometry`，SensorData QoS），每收到一帧有效数据，
同时发布 `/odom` 和 `odom -> base_footprint` TF，两者使用完全相同的时间戳，
默认使用发布时节点时钟的 `now()`（启用 `use_sim_time` 时使用仿真时间）。
通过 launch 参数 `tf_stamp_source` 可统一切换 `/odom` 和 TF 的时间戳来源：

```bash
# 默认 now：使用发布时的最新时间
ros2 launch jdbot_base_bringup fastlio_odom.launch.py tf_stamp_source:=now
# odometry：使用输入 /Odometry 的 header.stamp
ros2 launch jdbot_base_bringup fastlio_odom.launch.py tf_stamp_source:=odometry
```

保留输入的 x、y、yaw；z 默认为 0，roll/pitch 固定为 0。不会把首帧位姿归零。
`scripts/print_odometry.py` 可继续用于查看原始数据。

其余节点参数集中在 `launch/fastlio_odom.launch.py` 的 `parameters` 字典中，
可直接修改 `input_topic`、`output_topic`、`odom_frame`、
`base_frame`、`fixed_z`、`publish_tf`、`max_dt` 和 `twist_covariance_diagonal`。
直接运行时可以通过 `--ros-args -p fixed_z:=0.0 -p publish_tf:=false` 等方式覆盖。
直接运行时也支持 `--ros-args -p tf_stamp_source:=odometry`，默认值为 `now`。

无论输出时间戳来源如何设置，平面线速度和角速度均由相邻输入位姿及输入时间戳差分得到，线速度转换至 `base_footprint`
坐标系，yaw 差分处理跨越 ±π 的情况；不直接采用输入的 twist。
首帧、时间倒退、坐标系名变化或超过 `max_dt` 的间隔会将速度置零，并将平面速度的
协方差对角线设为 `1e6`，表示该帧速度未知。相同时间戳的重复数据不再发布。
正常速度帧使用配置的协方差（经验值，需要按实际数据调整）；位姿保留输入的
x/y/yaw 协方差子矩阵，受平面约束的 z/roll/pitch 协方差清零。

此转换假设输入世界坐标系可直接作为 odom，输入 body/IMU 坐标系与底盘在平面内
对齐且原点重合；不会查询 TF 或补偿传感器安装外参。如果存在平面安装偏移或旋转，
需要先补偿外参再使用该节点。运行时确保其他底盘或 EKF 节点不同时发布相同的
`odom -> base_footprint` TF 和 `/odom`；只需话题时可设置 `publish_tf:=false`。

## 添加 C++ 节点

将源码放到 `src/example_node.cpp`，在 `CMakeLists.txt` 的 `ament_package()` 前添加：

```cmake
add_executable(example_node src/example_node.cpp)
target_include_directories(example_node PRIVATE include)
ament_target_dependencies(example_node rclcpp)
install(TARGETS example_node DESTINATION lib/${PROJECT_NAME})
```

`rclcpp` 已声明并查找；使用其他依赖（如 `geometry_msgs`）时，还需添加对应的
`find_package(... REQUIRED)` 和 `ament_target_dependencies`，并确保
`package.xml` 中有相应的依赖声明。C++ 默认使用 C++17。

构建后运行：

```bash
ros2 run jdbot_base_bringup example_node
```

## 添加 Python 节点

将节点放到 `scripts/new_node.py`，第一行使用
`#!/usr/bin/env python3`，并提供执行入口：

```python
def main():
    # 节点逻辑
    pass


if __name__ == '__main__':
    main()
```

然后在 `CMakeLists.txt` 的 `python_nodes` 列表中添加 `new_node`。
无需维护 `setup.py`；构建后可以使用 `ros2 run jdbot_base_bringup new_node`，
也可以通过 `import jdbot_base_bringup.new_node` 导入。

原有可执行节点：`ros2_ctrl_odom_node`、`orbslam3_odom_to_tf`、
`teleop_twist_keyboard_bias`、`tf_pose_diff`。其他 Python 文件仍作为模块安装，
需要作为 ROS 2 可执行节点使用时，按上述方式加入列表。
