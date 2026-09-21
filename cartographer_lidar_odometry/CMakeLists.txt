cmake_minimum_required(VERSION 3.16)
project(cartographer_lidar_odometry)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cartographer REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lidar_odometry_node
  src/lidar_odometry_node.cpp
  vendor/cartographer_2.0.9004/src/common_internal_ceres_solver_options.cc
  vendor/cartographer_2.0.9004/src/local_trajectory_builder_2d.cc
  vendor/cartographer_2.0.9004/src/mapping_2d_grid_2d.cc
  vendor/cartographer_2.0.9004/src/mapping_2d_probability_grid.cc
  vendor/cartographer_2.0.9004/src/mapping_2d_probability_grid_range_data_inserter_2d.cc
  vendor/cartographer_2.0.9004/src/mapping_2d_submap_2d.cc
  vendor/cartographer_2.0.9004/src/mapping_imu_tracker.cc
  vendor/cartographer_2.0.9004/src/mapping_pose_extrapolator.cc
  vendor/cartographer_2.0.9004/src/mapping_probability_values.cc
  vendor/cartographer_2.0.9004/src/mapping_value_conversion_tables.cc
  vendor/cartographer_2.0.9004/src/mapping_internal_motion_filter.cc
  vendor/cartographer_2.0.9004/src/mapping_internal_range_data_collator.cc
  vendor/cartographer_2.0.9004/src/mapping_internal_2d_scan_matching_ceres_scan_matcher_2d.cc
  vendor/cartographer_2.0.9004/src/mapping_internal_2d_scan_matching_correlative_scan_matcher_2d.cc
  vendor/cartographer_2.0.9004/src/mapping_internal_2d_scan_matching_occupied_space_cost_function_2d.cc
  vendor/cartographer_2.0.9004/src/mapping_internal_2d_scan_matching_real_time_correlative_scan_matcher_2d.cc
  vendor/cartographer_2.0.9004/src/sensor_internal_voxel_filter.cc
  vendor/cartographer_2.0.9004/src/sensor_point_cloud.cc
  vendor/cartographer_2.0.9004/src/sensor_range_data.cc)
target_include_directories(lidar_odometry_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/vendor/cartographer_2.0.9004/include)
ament_target_dependencies(lidar_odometry_node
  cartographer_ros geometry_msgs nav_msgs rclcpp sensor_msgs tf2_ros)
target_link_libraries(lidar_odometry_node cartographer)

install(TARGETS lidar_odometry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE NOTICE README.md DESTINATION share/${PROJECT_NAME})

ament_package()
