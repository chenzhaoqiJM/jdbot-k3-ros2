from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'image_topic',
            default_value='/result_img',
            description='Posted image topics'),

        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='port'),

        Node(
            package='jobot_websocket_cpp',
            executable='jobot_websocket_cpp_node',
            name='jobot_websocket_cpp_node',
            output='screen',
            parameters=[
                {'image_topic': LaunchConfiguration('image_topic')},
                {'port': LaunchConfiguration('port')}
                ]
        ),
    ])
