-- 使用激光雷达和 IMU，不使用轮速里程计。
-- 默认 launch 将 Cartographer 的 imu 输入重映射到 /camera/imu。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "odom",
  -- Cartographer 要求 IMU frame 与 tracking_frame 共点。RealSense 合并后的
  -- /camera/imu 默认使用 camera_imu_optical_frame。
  tracking_frame = "camera_imu_optical_frame",
  -- 对外发布机器人基座位姿。
  published_frame = "base_footprint",
  -- map_frame 已作为外部 odom；使用独立中间帧避免 TF 自环。
  odom_frame = "odom_inter",
  -- Cartographer 发布 odom→odom_inter→base_footprint TF 链。
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  -- 不使用轮速里程计，只融合激光和 IMU。
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 乐山雷达参数（根据 lsn10 实际规格调整）
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 启用 IMU 数据融合。IMU frame 必须能通过 TF 连接到激光和机器人基座。
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- IMU 重力方向估计的时间常数，使用 Cartographer 推荐默认值。
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.
-- 无轮速里程计时启用实时相关扫描匹配。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 运动滤波器：小角度变化也要处理
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 回环检测与后端优化参数
-- 在 RISC-V 上长时间运行时，后端回环优化产生错误约束导致地图撕裂
-- 彻底禁用：关闭全局优化 + 禁用全局/非全局约束搜索
-- 前端扫描匹配已足够精确（静止漂移 <1cm/30s）
POSE_GRAPH.optimize_every_n_nodes = 0
-- 彻底禁用全局约束搜索（loop closure）
POSE_GRAPH.global_sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
-- 减少全局约束搜索范围
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
POSE_GRAPH.constraint_builder.log_matches = false

return options
