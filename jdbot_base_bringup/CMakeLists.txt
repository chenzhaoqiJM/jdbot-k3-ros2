cmake_minimum_required(VERSION 3.8)
project(jdbot_base_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# Install Python modules so imports keep working alongside C++ nodes.
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR scripts)

# Keep the executable names previously provided by setup.py console_scripts.
set(python_nodes
  ros2_ctrl_odom_node
  orbslam3_odom_to_tf
  teleop_twist_keyboard_bias
  tf_pose_diff
)
foreach(node IN LISTS python_nodes)
  install(PROGRAMS scripts/${node}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${node}
  )
endforeach()

# Add C++ targets here; see README.md for a complete example.
# add_executable(example_node src/example_node.cpp)
# target_include_directories(example_node PRIVATE include)
# ament_target_dependencies(example_node rclcpp)
# install(TARGETS example_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
