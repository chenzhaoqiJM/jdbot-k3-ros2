"""Launch Cartographer 2D odometry with laser scan and IMU input."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('jdbot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_directory = LaunchConfiguration(
        'configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    scan_topic = LaunchConfiguration('scan_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    map_topic = LaunchConfiguration('map_topic')

    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_publish_rate = LaunchConfiguration('odom_publish_rate')
    velocity_filter_alpha = LaunchConfiguration('velocity_filter_alpha')
    max_sample_interval = LaunchConfiguration('max_sample_interval')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity')
    launch_tf_to_odom = LaunchConfiguration('launch_tf_to_odom')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('map', map_topic),
            ('scan', scan_topic),
            ('imu', imu_topic),
        ],
    )

    tf_to_odom_node = Node(
        package='jdbot_slam',
        executable='tf_to_odom_node',
        name='tf_to_odom_node',
        output='screen',
        condition=IfCondition(launch_tf_to_odom),
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': odom_frame,
            'child_frame': base_frame,
            'publish_rate': odom_publish_rate,
            'velocity_filter_alpha': velocity_filter_alpha,
            'max_sample_interval': max_sample_interval,
            'max_linear_velocity': max_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
        }],
        remappings=[('odom', odom_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=os.path.join(pkg_share_dir, 'config'),
            description='Directory containing Cartographer Lua files.'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='provider_odom_with_imu.lua',
            description='Cartographer Lua configuration filename.'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='sensor_msgs/LaserScan topic used by Cartographer.'),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/camera/imu',
            description='sensor_msgs/Imu topic used by Cartographer.'),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map_cartographer',
            description='Cartographer map output topic.'),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Parent frame sampled by tf_to_odom_node.'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot child frame sampled by tf_to_odom_node.'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic published by tf_to_odom_node.'),
        DeclareLaunchArgument(
            'odom_publish_rate',
            default_value='50.0',
            description='Odometry publication rate in Hz.'),
        DeclareLaunchArgument(
            'launch_tf_to_odom',
            default_value='true',
            choices=['true', 'false'],
            description='Launch the TF-to-Odometry converter.'),
        DeclareLaunchArgument(
            'velocity_filter_alpha',
            default_value='0.35',
            description='Low-pass filter alpha for estimated velocity.'),
        DeclareLaunchArgument(
            'max_sample_interval',
            default_value='1.0',
            description='Maximum TF interval used for velocity estimation.'),
        DeclareLaunchArgument(
            'max_linear_velocity',
            default_value='3.0',
            description='Maximum accepted linear velocity in m/s.'),
        DeclareLaunchArgument(
            'max_angular_velocity',
            default_value='6.0',
            description='Maximum accepted angular velocity in rad/s.'),
        cartographer_node,
        tf_to_odom_node,
    ])
