cmake_minimum_required(VERSION 3.8)
project(jdbot_muse_pi_pro_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(nlohmann_json REQUIRED)

# openloop node
add_executable(openloop_node
  src/openloop/openloop_node.cpp
  src/openloop/cmdvel_to_motor.cpp
  
  src/motor_direction_ctrl.cpp
  src/motor_model.cpp
)

target_link_libraries(openloop_node
  lgpio
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(openloop_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# encoder node
add_executable(pid_ff_node
  src/pid_control/pid_ff_node.cpp
  src/pid_control/pid_cmdvel_to_motor.cpp
  src/pid_control/pid_encoder.cpp
  src/pid_control/pid_controller.cpp

  src/motor_direction_ctrl.cpp
  src/motor_model.cpp
)

target_link_libraries(pid_ff_node
  gpiod
  lgpio
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(pid_ff_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

install(TARGETS openloop_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pid_ff_node
  DESTINATION lib/${PROJECT_NAME})

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
