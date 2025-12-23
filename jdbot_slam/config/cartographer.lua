include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "base_footprint", 	-- "imu_link",gyro_link
    published_frame = "odom", 	-- "odom",
    odom_frame = "odom", 		-- "odom",
    provide_odom_frame = false,
    -- 仅发布2D位资
    publish_frame_projected_to_2d = true,
    use_odometry = true,
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
    odometry_sampling_ratio = 0.1,
    fixed_frame_pose_sampling_ratio = 1.,
    imu_sampling_ratio = 1.,
    landmarks_sampling_ratio = 1.,
}

-- 启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- 深度数据最小范围
TRAJECTORY_BUILDER_2D.min_range = 0.15
-- 深度数据最大范围
TRAJECTORY_BUILDER_2D.max_range = 5.0
-- 传感器数据超出有效范围最大值时，按此值来处理
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- 是否使用IMU数据
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 是否使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 运动过滤，检测运动变化，提高对运动的敏感度，避免机器人静止时插入数据
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
--  TEST 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
MAP_BUILDER.num_background_threads = 4

--  TEST
-- Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.65
-- 全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001

-- 设置0可关闭全局SLAM
POSE_GRAPH.optimize_every_n_nodes = 50


return options
