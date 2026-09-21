cmake_minimum_required(VERSION 3.16)
project(rgbd_odometry LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(rgbd_odometry_core
  src/image.cpp
  src/engine.cpp)
target_include_directories(rgbd_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rgbd_odometry_core Eigen3::Eigen)
ament_target_dependencies(rgbd_odometry_core sensor_msgs)

add_executable(rgbd_odometry_node src/node.cpp)
target_link_libraries(rgbd_odometry_node rgbd_odometry_core Eigen3::Eigen)
ament_target_dependencies(rgbd_odometry_node
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs tf2 tf2_ros tf2_eigen)

install(TARGETS rgbd_odometry_core rgbd_odometry_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/evaluate_trajectory.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.zh-CN.md THIRD_PARTY_LICENSES.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_image test/test_image.cpp)
  target_link_libraries(test_image rgbd_odometry_core)
  ament_target_dependencies(test_image sensor_msgs)
  ament_add_gtest(test_engine test/test_engine.cpp)
  target_link_libraries(test_engine rgbd_odometry_core Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_libraries(rgbd_odometry_core)
ament_package()
