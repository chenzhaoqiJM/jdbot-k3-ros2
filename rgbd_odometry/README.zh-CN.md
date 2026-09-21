# rgbd_odometry

这是一个独立实现、纯 CPU 的 ROS 2 Humble RGB-D 局部视觉里程计。在线估计只订阅彩色图、对齐深度图、CameraInfo 和 TF；不订阅轮速、ORB-SLAM、Cartographer 或其他参考轨迹。实现不使用 OpenCV、cuVSLAM、GPU、CUDA 或 OpenCL。

## 算法

深度按 CameraInfo 的 `plumb_bob` 或 `rational_polynomial` 模型反投影，并先经过不跨越深度边缘的局部中值滤波。估计器在两个降采样层级执行带 Huber 权重、法向一致性检查和尺度自适应阻尼的投影式 point-to-plane ICP，同时使用亚像素双线性灰度残差增强转动与平面切向运动的可观测性。短期视觉关键帧不参与正常运动的强制平滑，只在相邻帧跟踪失败时尝试恢复，因此不要求差速、阿克曼或其他底盘运动模型。稀疏灰度结构差仍用于识别完全静止状态，默认门限已从 14 降为 8，减少慢速运动被吞掉的概率。上述计算只使用 RGB-D 输入。随后通过固定 `rig_frame <- camera_frame` 外参把相机增量变换为 Rig 增量。默认只把累计位姿的 X、Y、yaw 输出为 REP-103 平面里程计。

当前 RVV 状态为 `off`，使用可移植标量路径；日志和周期统计会明确报告该状态。代码没有启用会在非 RVV 机器上触发非法指令的编译选项。

## 构建与测试

```bash
cd ~/jdbot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rgbd_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select rgbd_odometry
colcon test-result --verbose
```

构建依赖为 ROS 2 Humble、Eigen 3 和标准 C++17。审计示例：

```bash
rg -n 'opencv|cuvslam' src/rgbd_odometry
readelf -d install/rgbd_odometry/lib/rgbd_odometry/rgbd_odometry_node | grep NEEDED
```

第一条命令只应命中说明文档中的否定性声明；第二条不得出现 OpenCV 或 cuVSLAM。

## 启动

先启动相机（30 FPS 验收时把两个 profile 的最后一个数字改为 30）：

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ \
  enable_color:=true enable_depth:=true \
  rgb_camera.color_profile:=640,480,15 \
  depth_module.depth_profile:=640,480,15 \
  align_depth.enable:=true enable_sync:=true
```

再启动里程计：

```bash
ros2 launch rgbd_odometry rgbd_odometry.launch.py
```

所有配置都由该 launch 文件单独声明，不依赖默认 YAML。修改输入话题示例：

```bash
ros2 launch rgbd_odometry rgbd_odometry.launch.py \
  color_topic:=/camera/color/image_raw \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  planar_mode:=true publish_tf:=false
```

影响订阅、标定或估计器的参数需要重启节点后生效。默认细层迭代次数已根据 K3 峰值 CPU 验证设为 3；若在其他场景继续减少迭代次数或增大 `fine_stride`，应重新进行精度验收。

默认 `input_reliable_qos:=true`，与当前 D415 驱动和 rosbag 的 Reliable 发布端一致，可避免大图回放丢帧。若接入只提供 Best Effort 的其他相机驱动，启动时显式设为 `false`。

## 输出与故障语义

- `/rgbd_odometry/odom`：有效帧才发布，时间戳等于彩色图时间戳。
- `/rgbd_odometry/tracking`：每个已处理或明确丢弃的输入事件更新 true/false。
- TF 默认关闭；开启后发布 `rgbd_odom -> base_footprint`，数值和时间戳与 Odometry 相同。
- 缺少标定或外参、不同步、尺寸/编码错误、时间戳倒序、深度无效或 ICP 质量不合格时不伪造新位姿。
- 参数在启动时验证。订阅、标定和估计器参数均要求重启后生效。

## 离线回放与评测

在三个终端分别启动节点、记录结果和播放 bag。若 bag 的 `/tf_static` 完整，节点会自动取得相机外参。

```bash
ros2 launch rgbd_odometry rgbd_odometry.launch.py
ros2 bag record -o /tmp/rgbd_result /rgbd_odometry/odom /rgbd_odometry/tracking
ros2 bag play /path/to/input_bag
```

独立评测工具只读取结果，不向在线节点反馈。它采用首帧 SE(2) 刚体对齐、不拟合尺度，默认以 30 ms 做最近时间关联：

```bash
ros2 run rgbd_odometry evaluate_trajectory.py \
  /path/to/reference_bag /tmp/rgbd_result \
  --reference-topic /orbslam3/odom
```

工具报告 ATE RMSE、平移/旋转 RPE、路径长度比、匹配数和相对参考采样数的跟踪成功率。

参考为 TF 时，可直接选择父子坐标系：

```bash
ros2 run rgbd_odometry evaluate_trajectory.py \
  /path/to/reference_bag /tmp/rgbd_result \
  --reference-topic /tf \
  --reference-parent-frame odom \
  --reference-child-frame base_footprint
```

平面模式只对低于 1.5 mm 的单帧伪平移做与运动方向无关的小死区处理。旧版“转弯时清除 25 mm 以下平移”的底盘相关规则已经删除，真实圆弧、横移、手持和其他六自由度运动不会再因 yaw 增量被特殊清零。

## 实机验收建议

先做 60 秒静止，再依次做三次约 1 m 往返、三次低于 0.4 rad/s 的原地 360° 和三次约 1 m×1 m 方形闭环。参考 Cartographer 只由记录和离线评测读取，绝不能 remap 到本节点输入。用 `pidstat -p $(pgrep -n rgbd_odometry_node) 1` 记录 CPU，用 RSS 观察 30 分钟内存趋势。
