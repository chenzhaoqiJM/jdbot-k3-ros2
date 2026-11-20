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
