# linglong 机器人建图导航测试

## 底盘

```bash
ros2 launch jdbot_base_bringup start_control_linglong_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.270 wheel_diameter:=0.065 min_angular_speed:=0.4
```

## 实机模拟

```bash
ros2 launch jdbot_base_bringup start_control_linglong_fact.launch.py
```

## 建图

```bash
ros2 launch jdbot_slam rtabmap_rgbd.launch.py
```

与 rtabmap 里程计配合时

```bash
ros2 launch jdbot_slam rtabmap_rgbd.launch.py launch_point_cloud_xyz:=false
```

## 导航

```bash
ros2 launch jdbot_navigation nav2_rtabmap.launch.py controller_type:=mpc # 默认rpp
```

## 键盘控制

```bash
ros2 run jdbot_base_bringup teleop_twist_keyboard_bias \
  --ros-args \
  -p x_bias:=0.0 \
  -p y_bias:=0.2 \
  -p yaw_bias:=0.0

```

行为示例：
- 原始 (0.5, 0, 0) → 输出 (0.6, 0.02, 0.15)
- 原始 (0, 0, -1.0) → 输出 (0.1, 0.02, -0.85)
- 原始 (0, 0, 0) → 输出仍为 (0, 0, 0)