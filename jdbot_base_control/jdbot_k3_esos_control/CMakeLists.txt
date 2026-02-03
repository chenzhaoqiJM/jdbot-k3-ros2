cmake_minimum_required(VERSION 3.8)
project(jdbot_k3_esos_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件目录
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# RPMsg 电机控制节点 (新协议)
add_executable(rpmsg_motor_node
  src/rpmsg_motor_node.cpp
  src/rpmsg_motor_main.cpp
)

ament_target_dependencies(rpmsg_motor_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# RPMsg 旧协议节点 (Linux 端计算里程计)
add_executable(rpmsg_legacy_node
  src/rpmsg_legacy_node.cpp
  src/rpmsg_legacy_main.cpp
)

ament_target_dependencies(rpmsg_legacy_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# 安装可执行文件
install(TARGETS rpmsg_motor_node rpmsg_legacy_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
