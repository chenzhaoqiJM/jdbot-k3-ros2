#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    scan_frequency = LaunchConfiguration('scan_frequency', default='10.0')
    topic_name = LaunchConfiguration('topic_name', default='scan')
    flip_x_axis = LaunchConfiguration('flip_x_axis', default='false')
    auto_standby = LaunchConfiguration('auto_standby', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='雷达连接方式，C1 使用 serial'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='雷达串口设备路径'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='串口波特率，C1 默认为 460800'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='激光扫描消息的坐标系名称'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='是否反转扫描数据的角度顺序'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='是否对扫描点进行角度补偿'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='雷达扫描模式，C1 默认为 Standard'),

        DeclareLaunchArgument(
            'scan_frequency',
            default_value=scan_frequency,
            description='请求的扫描频率，单位 Hz，C1 典型值为 10.0'),

        DeclareLaunchArgument(
            'topic_name',
            default_value=topic_name,
            description='发布激光扫描数据的话题名称'),

        DeclareLaunchArgument(
            'flip_x_axis',
            default_value=flip_x_axis,
            description='是否将扫描数据旋转 180 度'),

        DeclareLaunchArgument(
            'auto_standby',
            default_value=auto_standby,
            description='没有扫描话题订阅者时是否自动停止扫描'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode,
                         'scan_frequency': scan_frequency,
                         'topic_name': topic_name,
                         'flip_x_axis': flip_x_axis,
                         'auto_standby': auto_standby}],
            output='screen'),
    ])
