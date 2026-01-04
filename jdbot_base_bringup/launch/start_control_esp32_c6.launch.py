#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================== 声明 Launch 参数 ==================
    publish_tf = LaunchConfiguration('publish_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish odom -> base TF'
    )

    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom',
        description='Odometry topic name'
    )

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame id'
    )

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame id'
    )

    # ================== TF 静态变换 ==================
    tf2_node_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.0', '0.0', '0.0001', '0.0', '0.0', '0.0',
                   'base_footprint', 'base_link'],
    )

    tf2_node_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_laser',
        arguments=['0.0', '-0.03', '0.10', '0', '0.0', '0.0',
                   'base_link', 'laser_link'],
    )

    tf2_node_rgbd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_rgbd',
        arguments=['0.15', '0', '0.06', '0.0', '0.0', '0.0',
                   'base_link', 'camera_link'],
    )

    # ================== 底盘里程计节点 ==================
    drive_node = Node(
        package='jdbot_base_bringup',
        executable='ros2_ctrl_odom_node',
        name='ros2_ctrl_odom_node',
        parameters=[{
            'publish_tf': publish_tf,
            'odom_topic': odom_topic,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
        }]
    )

    return LaunchDescription([
        declare_publish_tf,
        declare_odom_topic,
        declare_odom_frame,
        declare_base_frame,

        tf2_node_base,
        tf2_node_laser,
        tf2_node_rgbd,
        drive_node,
    ])
