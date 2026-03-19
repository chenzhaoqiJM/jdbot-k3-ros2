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
            'sub_image_topic',
            default_value='/image_raw',
            description='Video stream'),

        DeclareLaunchArgument(
            'linear_x',
            default_value='0.4',
            description='linear_x speed'),

        DeclareLaunchArgument(
            'angular_z',
            default_value='0.37',
            description='angular_z speed'),

        Node(
            package='jobot_mono_follow',
            executable='agv_follow_node_old',
            name='agv_follow_node_old',
            output='screen',
            parameters=[
                {'publish_result_img': LaunchConfiguration('publish_result_img')},
                {'sub_image_topic': LaunchConfiguration('sub_image_topic')},
                {'linear_x': LaunchConfiguration('linear_x')},
                {'angular_z': LaunchConfiguration('angular_z')},
            ],
            additional_env={'PYTHONUNBUFFERED': '1'}
        ),
    ])
