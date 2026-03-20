cmake_minimum_required(VERSION 3.8)
project(jobot_websocket_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# 添加可执行文件
add_executable(jobot_websocket_cpp_node src/jobot_websocket_cpp_node.cpp)

# 链接依赖库
target_link_libraries(jobot_websocket_cpp_node
  ${OpenCV_LIBRARIES}
  Boost::thread
)

ament_target_dependencies(jobot_websocket_cpp_node
  rclcpp
  std_srvs
  sensor_msgs
)

# 安装节点
install(TARGETS jobot_websocket_cpp_node
  DESTINATION lib/${PROJECT_NAME})

# 安装资源文件
install(DIRECTORY templates
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

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
