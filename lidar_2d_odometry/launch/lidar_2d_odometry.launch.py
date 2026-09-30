import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lidar_2d_odometry')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('tracking_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        Node(
            package='lidar_2d_odometry',
            executable='lidar_odometry_node',
            name='lidar_2d_odometry',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'lidar_2d_odometry.yaml'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'tracking_frame': LaunchConfiguration('tracking_frame'),
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'publish_tf': LaunchConfiguration('publish_tf'),
                },
            ],
        ),
    ])
