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

        # 👉 新增：发布 base_link -> imu_link 的静态 TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                # x y z qx qy qz qw
                '0', '0', '0',      # 平移 (按你的实际情况修改)
                '0', '0', '0', '1', # 四元数 (按你的实际情况修改)
                'base_link',
                'imu_link'
            ]
        ),
    ])
