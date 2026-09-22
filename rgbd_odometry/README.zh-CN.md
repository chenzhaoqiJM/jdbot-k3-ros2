# rgbd_odometry

这是一个独立实现、纯 CPU 的 ROS 2 Humble RGB-D 视觉里程计。在线估计只订阅彩色图、对齐深度图、CameraInfo 和 TF；不订阅轮速、ORB-SLAM、Cartographer 或其他参考轨迹。逐帧稠密/稀疏跟踪为 clean-room 自研实现；OpenCV 仅用于低频 ORB 回环重识别。实现不使用 cuVSLAM、GPU、CUDA 或 OpenCL。

## 算法

深度按 CameraInfo 的 `plumb_bob` 或 `rational_polynomial` 模型反投影，并先经过不跨越深度边缘的局部中值滤波。估计器在两个降采样层级执行带 Huber 权重、法向一致性检查和尺度自适应阻尼的投影式 point-to-plane ICP，同时使用亚像素双线性灰度残差增强转动与平面切向运动的可观测性。粗层和细层分别要求至少 40/180 对几何对应：粗层允许在远景、背光和近距深度稀疏时继续给出收敛初值，细层仍采用严格门限验收最终增量。几何与光度法方程分别累计；归一化几何 Hessian 的最小特征值较低时，光度权重才从 0.1 平滑提高，最大为 0.18。

为补足大平面的切向退化，新增了完全独立实现的稀疏前端：Shi–Tomasi 最小特征值选点、三层迭代 Lucas–Kanade、前后向一致性、带 3D 共识门控的 RANSAC 刚体初值，以及“参考帧 3D—当前帧 2D”的鲁棒重投影细化。稀疏跟踪优先跨短期视觉关键帧，减少逐帧积分漂移；仅在稀疏结果与稠密解处于同一收敛域时，以最高 0.2 的置信度权重融合。3D–3D 只用于排除误匹配和提供初值，最终平移由不依赖当前帧深度的重投影优化细化，避免 D415 双帧深度噪声污染平移。实现依据公开论文公式重写，不复制参考实现，也不链接 OpenCV。

上述退化检测和稀疏约束都直接来自当前 RGB-D 场景，不使用速度方向或车辆运动学。除活动关键帧外，估计器默认保留最近 3 个关键帧，每 3 个运动帧尝试一次旧关键帧约束；旧帧必须通过基线范围、前后向光流、RANSAC、重投影和稠密解一致性五层门控，窗口总修正权重最高 0.12。这是固定计算预算的局部多关键帧约束，不是带边缘化的完整滑窗 BA。短期视觉关键帧仍可在相邻帧稠密跟踪失败时恢复，因此不要求差速、阿克曼或其他底盘运动模型。

长期层额外保存一个有界的 20 关键帧地图，并永久保留首帧原点锚点。每 30 帧才执行一次地图查询；先用低成本归一化灰度相关性筛选候选，再用最多 200 个 ORB 特征、Hamming 双向比值检查、位姿引导像素门控、RGB-D 3D RANSAC 和重投影细化确认重访。OpenCV 被固定为单线程。短期地图匹配只校正旋转，原点重访才以限幅、置信度加权的方式修正全局位姿，避免深度量化噪声造成平移跳变。该层计算和内存均有固定上限，但不是完整位姿图优化，不能回写已经发布的历史轨迹。

稀疏灰度结构差用于识别完全静止状态，默认门限为 8，减少慢速运动被吞掉的概率。上述计算只使用 RGB-D 输入。随后通过固定 `rig_frame <- camera_frame` 外参把相机增量变换为 Rig 增量。默认只把累计位姿的 X、Y、yaw 输出为 REP-103 平面里程计。

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

构建依赖为 ROS 2 Humble、Eigen 3、OpenCV 4 和标准 C++17。审计示例：

```bash
rg -ni 'cuvslam|cuda|opencl' src/rgbd_odometry
readelf -d install/rgbd_odometry/lib/rgbd_odometry/rgbd_odometry_node | grep NEEDED
```

第一条命令只应命中说明文档中的边界声明，不应出现 cuVSLAM/CUDA/OpenCL 实现依赖；动态依赖中允许出现 OpenCV，用于上述低频 ORB 回环模块。

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

`photometric_weight` 是正常场景的光度下限；`photometric_max_weight` 是几何退化时的上限；`photometric_adaptive_threshold` 控制开始增权的归一化可观测性阈值。周期日志中的 `obs` 和 `photo` 分别给出实际参与求解帧的平均几何可观测性和平均有效光度权重。

默认有效深度范围为 0.2–4.5 m。超过 4.5 m 的 D415 深度不进入 ICP，以减少远距离量化噪声和边缘飞点对旋转估计的杠杆效应。

`coarse_min_correspondences` 与 `fine_min_correspondences` 是两个金字塔层级各自的质量门限。不要再用单一门限同时调节两层；降低细层门限可能接受切向漂移明显的解。

`sparse_fusion_weight` 和 `sparse_translation_fusion_weight` 分别控制稀疏旋转与重投影平移的最大融合比例，默认均为 0.2；`sparse_max_*_disagreement` 是与稠密解的一致性门限。把权重调高会更依赖局部纹理与光照恒常性，修改后应同时回归 data4 和 data5。

局部窗口默认参数是 `window_max_keyframes:=3`、`window_evaluation_interval:=3`、`window_max_translation_m:=0.45`、`window_max_rotation_rad:=0.45`、`window_fusion_weight:=0.12`。将窗口扩大到 6 帧并逐帧求解只带来毫米级 ATE 变化，却显著增加时延，因此不建议直接提高这些参数；若需要明显的长期漂移改善，应实现持久化特征轨迹和真正的滑窗联合优化。

历史地图默认参数是 `map_max_keyframes:=20`、`map_query_interval:=30`、`map_min_keyframe_age:=30`、`map_min_inliers:=8` 和 `map_fusion_weight:=0.50`。当前策略刻意只把首帧作为全局平移锚点；实验中把普通、已漂移的历史关键帧当作绝对锚点会在 data5 产生近 1 m ATE，因此不要通过放宽查询距离或修正限幅来追求更频繁的“回环”。

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
