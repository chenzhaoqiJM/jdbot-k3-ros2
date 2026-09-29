# k3_rgbd_odometry

面向 K3 RISC-V 与 RealSense D415 的轻量 ROS 2 纯 RGB-D 里程计原型。核心不依赖
OpenCV、PCL、Ceres 或 TBB，仅使用 Eigen 与 ROS 2/TF2。

## 当前状态

- 可在 amd64 与 K3 riscv64 上以 Release 模式编译。
- 支持 `rgb8`、`bgr8`、`mono8` 彩色输入与 `16UC1` 对齐深度。
- 按每帧图像时间戳从 TF 获取 `base_frame <- camera_frame` 外参，支持活动头部。
- 同时发布完整 6DoF 里程计和自动稳定的 `x/y/yaw` 平面里程计，无需设置底盘类型。
- 自动融合 SE(3) 与全向 SE(2) 两种运动假设：平稳平台优先低漂移平面解，持续升降、
  俯仰或横滚时自动转向 6DoF 解；持续横移会自动解除横向抑制。
- K3 真机 640×480@15 Hz 下，实测跟踪与 odom 输出约 `14.3–14.9 Hz`；
  20 秒逐秒 CPU 峰值 `80%`、稳定区间 `61–71%`，内存约 `36–39 MB`，
  低于单核 100% 上限。
- 已加入轻量关键帧、BRIEF 描述子、RGB-D SE(2) RANSAC、位姿图优化和优化轨迹发布。
- 三份闭环数据的逐帧完整优化轨迹平移 RMSE 为 `0.091 / 0.083 / 0.176 m`，均优于
  同包 cuVSLAM 的 `0.154 / 0.248 / 0.263 m`。

## 算法结构

1. 自实现灰度转换和四层图像金字塔；
2. 分块 Shi–Tomasi 角点；
3. 粗层小窗口块匹配与亚像素 KLT；
4. 深度恢复 3D，对应点 RANSAC 刚体初值；
5. 解析雅可比 Gauss–Newton 完整 SE(3) 重投影优化；
6. 并行全向 SE(2) 假设，根据连续非平面运动证据自动选择或融合，不要求底盘类型；
7. 每帧动态 TF 外参补偿，支持相机固定安装或随机器人关节运动；
8. 独立发布连续局部 6DoF 与导航用平面 odom，并输出质量相关协方差；
9. 按位移/转角创建持久关键帧，自研 256-bit BRIEF 描述子检索；
10. 由匹配特征的 RGB-D 三维点执行 SE(2) RANSAC，验证后加入小型位姿图；
11. 回环开启时，校正后的轨迹直接作为 odom 输出；关闭时输出未经回环校正的连续里程计。

源码中保留了实验性的直接光度、深度 ICP 与虚拟二维扫描校正函数，便于继续消融；
生产路径目前没有串联深度 ICP/虚拟扫描，因为离线结果显示它们会引入额外偏置。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/jdbot_ws
colcon build --packages-select k3_rgbd_odometry --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动

相机必须发布已经对齐到彩色相机的深度，并且 TF 树中应能查询
`base_footprint <- camera_color_optical_frame`。

```bash
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py
```

### 常用 launch 参数

默认值针对 D415 当前的 640×480、15 Hz 配置。下列常用项可直接在启动命令中覆盖，
未列出的高级调优参数继续从 `config/k3_d415.yaml` 读取。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `config_file` | 包内 `k3_d415.yaml` | 切换整套 YAML 配置 |
| `namespace` / `node_name` | 空 / `k3_rgbd_odometry` | 节点命名空间和名称 |
| `color_topic` | `/camera/color/image_raw` | 彩色图像话题 |
| `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | 对齐到彩色相机的深度话题 |
| `camera_info_topic` | `/camera/color/camera_info` | 彩色相机内参话题 |
| `odom_topic` | `/k3_rgbd_odometry/odom` | 平面里程计话题 |
| `odom_6d_topic` | `/k3_rgbd_odometry/odom_6d` | 扩展的完整六自由度里程计输出 |
| `tracking_topic` | `/k3_rgbd_odometry/tracking` | 跟踪状态话题 |
| `odom_frame` | `odom_k3_rgbd` | 里程计消息的 `header.frame_id` |
| `base_frame` | `base_footprint` | 查询相机外参时使用的机器人基座坐标系 |
| `child_frame` | `base_footprint_k3_rgbd` | 里程计消息的 `child_frame_id` |
| `tf_odom_frame` | `odom_k3_rgbd` | TF 的父坐标系 |
| `tf_child_frame` | `base_footprint_k3_rgbd` | TF 的子坐标系 |
| `publish_tf` | `true` | 是否发布 TF |
| `processing_rate_hz` | `20.0` | 最大处理频率；相机 15 Hz 时实际输出约 15 Hz |
| `expected_frame_rate_hz` | `15.0` | 丢帧恢复所依据的相机帧率 |
| `sync_tolerance_ms` | `40.0` | 彩色与深度帧最大时间差 |
| `min_depth_m` / `max_depth_m` | `0.25` / `6.0` | 有效深度范围 |
| `max_features` | `420` | 最大特征数，降低该值可进一步节省 CPU |
| `enable_loop_closure` | `true` | 是否将轻量回环校正应用到 odom 输出 |
| `use_sim_time` | `false` | 回放 rosbag 时使用 `/clock` |

例如，发布标准的 `odom -> base_footprint`，并关闭回环、将处理上限设为 15 Hz：

```bash
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py \
  tf_child_frame:=base_footprint enable_loop_closure:=false \
  processing_rate_hz:=15.0
```

查看完整参数列表：

```bash
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py --show-args
```

主要输出：

- `/k3_rgbd_odometry/odom`：`nav_msgs/Odometry`，默认消息坐标系为
  `odom_k3_rgbd -> base_footprint_k3_rgbd`
- `/k3_rgbd_odometry/tracking`：`std_msgs/Bool`，当前帧是否可靠
- `odom_k3_rgbd -> base_footprint_k3_rgbd`：默认发布的 TF
- `/k3_rgbd_odometry/odom_6d`：扩展的完整 `x/y/z/roll/pitch/yaw` 原始里程计
- `/k3_rgbd_odometry/inliers`、`/k3_rgbd_odometry/loop_closure`：扩展诊断信息
- `/k3_rgbd_odometry/optimized_path`：在 `odom_k3_rgbd` 下追溯校正的完整轨迹

发布形式与 cuVSLAM 一致，但话题和 TF 均使用独立的清晰命名，因此两套里程计可以同时
运行并进行对比。默认不发布 `map -> odom`。

回环开关：

```bash
# 默认：检测到回环后，odom 话题和 TF 直接采用校正轨迹，校正瞬间可能跳变
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py enable_loop_closure:=true

# 纯局部里程计：不检测回环，输出连续但长期漂移不会被校正
ros2 launch k3_rgbd_odometry rgbd_odometry.launch.py enable_loop_closure:=false
```


真机接入导航前，可把配置中的 `odom_frame` 与 `child_frame` 改成系统所需名称。

## 离线基线

仓库提供的三圈参考数据中，cuVSLAM 相对激光里程计的平移 RMSE 约为
`0.154 / 0.248 / 0.263 m`，回到起点的平移残差约为
`0.339 / 0.238 / 0.199 m`。这些数值是后续版本必须同时达到或超过的验收线。

离线重放和评估：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select k3_rgbd_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_OFFLINE_TOOLS=ON
source install/setup.bash
ros2 run k3_rgbd_odometry offline_rgbd_evaluator \
  rgbd_odom_data_1 analysis/universal_dual_final_data_1.csv
python3 tools/evaluate_estimate.py analysis/data_1/tf.csv \
  analysis/universal_fullpath_data_1.csv.optimized.csv
```

## 已知边界

当前精度指标来自仓库中的三组 D415/K3 轮式路线；完整 6DoF 输出已在 K3 真机验证消息、
频率和姿态变化，但仓库尚无双足人形机器人实采 RGB-D 数据，因此人形步态精度仍需在目标
机器人上补充直行、横移、转弯、原地踏步和活动头部数据验收。活动头部必须提供时间同步的
关节状态与 TF，否则系统只能可靠估计相机轨迹，无法唯一恢复机器人基座轨迹。

节点运行时不读取激光或参考里程计。玻璃、镜面、严重运动模糊、长时间无纹理以及大面积
动态遮挡仍可能令纯 RGB-D 跟踪退化。
