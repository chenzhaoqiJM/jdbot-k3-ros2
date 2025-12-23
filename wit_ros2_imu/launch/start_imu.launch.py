from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='IMU serial port'
        ),

        # IMU Node
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='wit_ros2_imu',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
            ],
            additional_env={'PYTHONUNBUFFERED': '1'}
        ),

    ])
