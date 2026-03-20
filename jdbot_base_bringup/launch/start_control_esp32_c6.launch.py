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

    motor1_factor = LaunchConfiguration('motor1_factor')
    motor2_factor = LaunchConfiguration('motor2_factor')
    wheel_diameter = LaunchConfiguration('wheel_diameter')
    wheel_base = LaunchConfiguration('wheel_base')

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

    declare_motor1_factor = DeclareLaunchArgument(
        'motor1_factor',
        default_value='1.0',
        description='motor1_factor for speed'
    )

    declare_motor2_factor = DeclareLaunchArgument(
        'motor2_factor',
        default_value='1.0',
        description='motor2_factor for speed'
    )

    declare_wheel_diameter = DeclareLaunchArgument(
        'wheel_diameter',
        default_value='0.067',
        description='Wheel diameter in meters'
    )

    declare_wheel_base = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.183',
        description='Wheel base (distance between wheels) in meters'
    )

    # ================== TF 静态变换 ==================
    tf2_node_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.0', '0.0', '0.0001', '0.0', '0.0', '0.0',
                   'base_footprint', 'base_link'],
    )

    # hesai是Y轴向前，X轴向左，Z轴向上的坐标系，而ROS通常使用X轴向前，Y轴向左，Z轴向上的坐标系，因此需要进行坐标变换调整。这里的旋转角度 -1.57079632679（-90度）将 hesai_lidar 的坐标系从 ROS 标准坐标系旋转到正确的方向，使得激光雷达的数据能够正确地与 ROS 中的其他传感器数据进行融合和处理。
    tf2_node_3d_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_hesai_lidar',
        arguments=['0.01', '0.0', '0.25', '-1.57079632679', '0.0', '0.0',
                   'base_link', 'hesai_lidar'],
    )

    tf2_node_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_laser',
        arguments=['0.03', '0.0', '0.20', '3.14159', '0.0', '0.0',
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
            'motor1_factor':motor1_factor,
            'motor2_factor':motor2_factor,
            'wheel_diameter': wheel_diameter,
            'wheel_base': wheel_base,
        }]
    )

    return LaunchDescription([
        declare_publish_tf,
        declare_odom_topic,
        declare_odom_frame,
        declare_base_frame,
        declare_motor1_factor,
        declare_motor2_factor,
        declare_wheel_diameter,
        declare_wheel_base,

        tf2_node_base,
        tf2_node_3d_laser,
        tf2_node_laser,
        tf2_node_imu,
        tf2_node_rgbd,
        drive_node,
    ])
