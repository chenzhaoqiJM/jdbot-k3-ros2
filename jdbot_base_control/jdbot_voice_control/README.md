

# 编译
colcon build --packages-select jdbot_voice_control

# 启动节点
ros2 launch jdbot_voice_control voice_cmd.launch.py

# 发送指令（另一个终端）
ros2 topic pub --once /voice_cmd std_msgs/String "data: 'forward'"
ros2 topic pub --once /voice_cmd std_msgs/String "data: 'turn_left'"