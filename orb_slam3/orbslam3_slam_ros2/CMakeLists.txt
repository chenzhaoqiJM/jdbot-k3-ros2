cmake_minimum_required(VERSION 3.16)
project(orbslam3_slam_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
find_package(ORB_SLAM3 CONFIG REQUIRED)

add_executable(orbslam3_slam_node src/orbslam3_slam_node.cpp)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # Eigen 3.4 emits false-positive warnings for Transform::inverse at -O3.
  target_compile_options(orbslam3_slam_node PRIVATE -Wno-maybe-uninitialized)
endif()
target_include_directories(orbslam3_slam_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS})
ament_target_dependencies(orbslam3_slam_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs cv_bridge
  message_filters tf2_ros)
target_link_libraries(orbslam3_slam_node
  ORB_SLAM3::ORB_SLAM3
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES})
set_target_properties(orbslam3_slam_node PROPERTIES
  SKIP_BUILD_RPATH TRUE
  INSTALL_RPATH ""
  INSTALL_RPATH_USE_LINK_PATH FALSE)

install(TARGETS orbslam3_slam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
