# ORB-SLAM3 RGB-D 建图节点

建图节点与里程计节点位于同一个 `orbslam3_ros2` 包中，两者仍是独立的
可执行程序，按需启动其中一个。

## 输出接口

- `/orbslam3_slam/pose`：当前底盘在 `orb_map` 下的全局位姿。
- `/orbslam3_slam/path`：本次建图轨迹。
- `/orbslam3_slam/map_points`：当前帧跟踪到的 ORB-SLAM3 稀疏地图点云，QoS
  为 transient local。
- `/orbslam3_slam/tracking_state`：跟踪状态，`2` 表示正常。
- TF：默认发布 `orb_map -> base_footprint`。
- `/orbslam3_slam/finish_mapping`：结束建图并保存 Atlas 和 TUM 轨迹。
- `/orbslam3_slam/reset_map`：清空当前活动地图并重新建图。

ORB-SLAM3 生成的是稀疏视觉地图，本节点不把稀疏点伪装成二维占据栅格。若
Nav2 需要障碍物代价地图，应继续使用 D455 深度点云或激光雷达建立 costmap。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
colcon build --packages-select orbslam3_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 640×480 建图

相机使用现有的 640×480@15 配置后运行：

```bash
ros2 launch orbslam3_ros2 rgbd_slam.launch.py
```

## 高分辨率导航图像、低分辨率视觉建图

D455 在当前 USB 2.1 链路下使用彩色 1280×720@10、深度 848×480@10，导航
使用原始 1280×720 图像，ORB 节点内部缩放到 640×360：

```bash
ros2 launch orbslam3_ros2 rgbd_slam.launch.py \
  settings:=$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/d455_rgbd_1280x720_slam.yaml \
  resize_for_orb:=true tracking_width:=640 tracking_height:=360
```

所有话题、坐标系、保存目录和缩放参数均可通过 launch 命令行覆盖，可用
`ros2 launch orbslam3_ros2 rgbd_slam.launch.py --show-args` 查看。

## 保存结果

```bash
ros2 service call /orbslam3_slam/finish_mapping std_srvs/srv/Trigger '{}'
```

默认输出目录为 `~/.ros/orbslam3_slam`：

- `orbslam3_atlas.osa`
- `CameraTrajectory.txt`
- `KeyFrameTrajectory.txt`

调用完成服务后节点仍存在，但不再处理图像；退出节点前应先调用该服务。
