from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'publish_result_img',
            default_value='false',
            description='Whether to publish the image'),

        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video20',
            description='Video device'),

        Node(
            package='jobot_mono_follow',
            executable='agv_follow_node',
            name='agv_follow_node',
            output='screen',
            parameters=[
                {'publish_result_img': LaunchConfiguration('publish_result_img')},
                {'video_device': LaunchConfiguration('video_device')},
            ],
            additional_env={'PYTHONUNBUFFERED': '1'}
        ),
    ])
