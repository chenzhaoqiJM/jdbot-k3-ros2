import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'cartographer_lidar_odometry')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=os.path.join(package_share, 'config')),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='lidar_odometry.lua'),
        Node(
            package='cartographer_lidar_odometry',
            executable='lidar_odometry_node',
            name='cartographer_lidar_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'configuration_directory': LaunchConfiguration(
                    'configuration_directory'),
                'configuration_basename': LaunchConfiguration(
                    'configuration_basename'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            }],
        ),
    ])
