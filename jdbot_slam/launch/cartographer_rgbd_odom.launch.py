"""Provide Cartographer odometry from an RGB-D camera's depth image.

Only the depth stream is used.  ``depthimage_to_laserscan`` projects a
horizontal band of the depth image into ``/scan`` for Cartographer 2D.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('jdbot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    depth_topic = LaunchConfiguration('depth_topic')
    depth_camera_info_topic = LaunchConfiguration(
        'depth_camera_info_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    output_frame = LaunchConfiguration('output_frame')
    scan_height = LaunchConfiguration('scan_height')
    scan_time = LaunchConfiguration('scan_time')
    range_min = LaunchConfiguration('range_min')
    range_max = LaunchConfiguration('range_max')

    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_frame': output_frame,
            'scan_height': ParameterValue(scan_height, value_type=int),
            'scan_time': ParameterValue(scan_time, value_type=float),
            'range_min': ParameterValue(range_min, value_type=float),
            'range_max': ParameterValue(range_max, value_type=float),
        }],
        remappings=[
            ('depth', depth_topic),
            ('depth_camera_info', depth_camera_info_topic),
            ('scan', scan_topic),
        ],
    )

    cartographer_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share_dir, 'launch', 'cartographer_odom.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'base_frame': base_frame,
            'scan_topic': scan_topic,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_rect_raw',
            description='Rectified depth image topic.'),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/depth/camera_info',
            description='Camera calibration topic for the depth image.'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='LaserScan topic consumed by Cartographer.'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot base frame published by Cartographer.'),
        DeclareLaunchArgument(
            'output_frame',
            default_value='base_footprint',
            description='Frame of the generated horizontal laser scan.'),
        DeclareLaunchArgument(
            'scan_height',
            default_value='10',
            description='Number of depth-image rows combined into one scan.'),
        DeclareLaunchArgument(
            'scan_time',
            default_value='0.033',
            description='Time between scans in seconds.'),
        DeclareLaunchArgument(
            'range_min',
            default_value='0.15',
            description='Minimum valid depth in metres.'),
        DeclareLaunchArgument(
            'range_max',
            default_value='3.0',
            description='Maximum valid depth in metres.'),
        depth_to_scan_node,
        cartographer_odom,
    ])
