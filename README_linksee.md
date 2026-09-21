# linksee 机器人建图导航测试

## 底盘

```bash
ros2 launch jdbot_base_bringup start_control_linksee_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.185 wheel_diameter:=0.065 min_angular_speed:=0.4
```

## 建图

启动 realsense 相机

启动底盘 -> 启动激光雷达（可选） -> 启动里程计（雷达或rgbd）

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