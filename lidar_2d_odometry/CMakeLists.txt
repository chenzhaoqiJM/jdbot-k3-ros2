cmake_minimum_required(VERSION 3.16)
project(lidar_2d_odometry)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(scan_matcher src/scan_matcher.cpp)
target_include_directories(scan_matcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scan_matcher PUBLIC Eigen3::Eigen)

add_executable(lidar_odometry_node src/lidar_odometry_node.cpp)
target_link_libraries(lidar_odometry_node scan_matcher)
ament_target_dependencies(lidar_odometry_node
  geometry_msgs nav_msgs rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS scan_matcher lidar_odometry_node
  EXPORT export_scan_matcher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(scan_matcher_test test/scan_matcher_test.cpp)
  target_link_libraries(scan_matcher_test scan_matcher)
endif()

ament_export_targets(export_scan_matcher HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
