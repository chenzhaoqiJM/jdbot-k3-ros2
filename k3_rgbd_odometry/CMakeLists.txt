cmake_minimum_required(VERSION 3.16)
project(k3_rgbd_odometry)

option(BUILD_OFFLINE_TOOLS "Build rosbag2 offline evaluation tools" OFF)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(rgbd_frontend src/rgbd_frontend.cpp src/loop_closure.cpp)
target_include_directories(rgbd_frontend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rgbd_frontend Eigen3::Eigen)
target_compile_options(rgbd_frontend PRIVATE -O3 -Wall -Wextra -Wpedantic)

add_executable(rgbd_odometry_node src/rgbd_odometry_node.cpp)
target_link_libraries(rgbd_odometry_node rgbd_frontend Eigen3::Eigen)
ament_target_dependencies(rgbd_odometry_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_ros)
target_compile_options(rgbd_odometry_node PRIVATE -O3 -Wall -Wextra -Wpedantic)

install(TARGETS rgbd_frontend rgbd_odometry_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_OFFLINE_TOOLS)
  find_package(rosbag2_cpp REQUIRED)
  add_executable(offline_rgbd_evaluator tools/offline_rgbd_evaluator.cpp)
  target_link_libraries(offline_rgbd_evaluator rgbd_frontend Eigen3::Eigen)
  ament_target_dependencies(offline_rgbd_evaluator
    rclcpp sensor_msgs nav_msgs geometry_msgs rosbag2_cpp)
  target_compile_options(offline_rgbd_evaluator PRIVATE -O3 -Wall -Wextra -Wpedantic)
  install(TARGETS offline_rgbd_evaluator RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(rgbd_frontend)
ament_export_dependencies(Eigen3)
ament_package()
