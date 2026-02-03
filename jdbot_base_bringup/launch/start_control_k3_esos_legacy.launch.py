#!/usr/bin/env python3

"""
K3 ESOS 旧协议电机控制启动文件

使用 rpmsg_legacy_node，在 Linux 端计算里程计
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================== Launch 参数 ==================
    publish_tf = LaunchConfiguration('publish_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_base = LaunchConfiguration('wheel_base')
    motor1_factor = LaunchConfiguration('motor1_factor')
    motor2_factor = LaunchConfiguration('motor2_factor')

    # ================== 参数声明 ==================
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

    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0335',
        description='Wheel radius in meters'
    )

    declare_wheel_base = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.28',
        description='Wheel base (distance between wheels) in meters'
    )

    declare_motor1_factor = DeclareLaunchArgument(
        'motor1_factor',
        default_value='1.0',
        description='Speed factor for motor 1'
    )

    declare_motor2_factor = DeclareLaunchArgument(
        'motor2_factor',
        default_value='1.0',
        description='Speed factor for motor 2'
    )

    # ================== TF 静态变换 ==================
    tf2_node_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=['0.0', '0.0', '0.0001', '0.0', '0.0', '0.0',
                   'base_footprint', 'base_link'],
    )

    # ydlidar
    # tf2_node_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='tf_pub_base_to_laser',
    #     arguments=['0.03', '0.0', '0.20', '3.14159', '0.0', '0.0',
    #                'base_link', 'laser_link'],
    # )

    tf2_node_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_laser',
        arguments=['0.0', '0.0', '0.20', '0.0', '0.0', '0.0',
                   'base_link', 'laser_link'],
    )

    tf2_node_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_imu',
        arguments=['0.0', '0.0', '0.01', '0', '0.0', '0.0',
                   'base_link', 'imu_link'],
    )

    tf2_node_rgbd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_rgbd',
        arguments=['0.15', '0', '0.06', '0.0', '0.0', '0.0',
                   'base_link', 'camera_link'],
    )

    # ================== 旧协议电机控制节点 ==================
    rpmsg_legacy_node = Node(
        package='jdbot_k3_esos_control',
        executable='rpmsg_legacy_node',
        name='rpmsg_legacy_node',
        output='screen',
        parameters=[{
            'publish_tf': publish_tf,
            'odom_topic': odom_topic,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'motor1_factor': motor1_factor,
            'motor2_factor': motor2_factor,
        }]
    )

    return LaunchDescription([
        # 参数声明
        declare_publish_tf,
        declare_odom_topic,
        declare_odom_frame,
        declare_base_frame,
        declare_wheel_radius,
        declare_wheel_base,
        declare_motor1_factor,
        declare_motor2_factor,

        # 静态 TF
        tf2_node_base,
        tf2_node_laser,
        tf2_node_imu,
        tf2_node_rgbd,

        # 电机控制节点
        rpmsg_legacy_node,
    ])
