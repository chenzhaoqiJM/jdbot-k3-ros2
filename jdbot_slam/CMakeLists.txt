cmake_minimum_required(VERSION 3.8)
project(jdbot_slam)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tf_to_odom_node src/tf_to_odom_node.cpp)
target_compile_features(tf_to_odom_node PUBLIC cxx_std_17)
ament_target_dependencies(tf_to_odom_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS tf_to_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS jdbot_slam/hello_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hello_node
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
