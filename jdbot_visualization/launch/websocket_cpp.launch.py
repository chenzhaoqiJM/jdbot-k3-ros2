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

        Node(
            package='websocket_cpp',
            executable='websocket_cpp_node',
            name='websocket_cpp_node',
            output='screen',
            parameters=[
                {'image_topic': LaunchConfiguration('image_topic')}]
        ),
    ])
