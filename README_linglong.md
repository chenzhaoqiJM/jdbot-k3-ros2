# linglong 机器人建图导航测试

底盘

```bash
ros2 launch jdbot_base_bringup start_control_linglong_esp32_c6.launch.py publish_tf:=false odom_topic:=odom_base wheel_base:=0.270 wheel_diameter:=0.065 min_angular_speed:=0.4
```

realsense 相机

```bash
cd ~
wget https://archive.spacemit.com/ros2/prebuilt_libs/source_code_common/librealsense-2.57.4.tar.gz
tar xzvf librealsense-2.57.4.tar.gz
cd librealsense-2.57.4/
./scripts/setup_udev_rules.sh
cd ..
rm -rf librealsense-2.57.4 librealsense-2.57.4.tar.gz
```

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/
```
