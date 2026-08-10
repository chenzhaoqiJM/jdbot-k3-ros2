#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================== 声明 Launch 参数 ==================
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    send_hz = LaunchConfiguration('send_hz')
    odom_hz = LaunchConfiguration('odom_hz')
    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    publish_tf = LaunchConfiguration('publish_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    motor1_factor = LaunchConfiguration('motor1_factor')
    motor2_factor = LaunchConfiguration('motor2_factor')
    wheel_diameter = LaunchConfiguration('wheel_diameter')
    wheel_base = LaunchConfiguration('wheel_base')
    feedback_pwm_deadzone = LaunchConfiguration('feedback_pwm_deadzone')
    encoder_ppr = LaunchConfiguration('encoder_ppr')
    reduction_ratio = LaunchConfiguration('reduction_ratio')
    ff_factor = LaunchConfiguration('ff_factor')
    pid_kp = LaunchConfiguration('pid_kp')
    pid_ki = LaunchConfiguration('pid_ki')
    pid_kd = LaunchConfiguration('pid_kd')
    straight_balance_kp = LaunchConfiguration('straight_balance_kp')
    straight_balance_ki = LaunchConfiguration('straight_balance_ki')
    straight_balance_kd = LaunchConfiguration('straight_balance_kd')
    cfg_send_on_startup = LaunchConfiguration('cfg_send_on_startup')
    debug = LaunchConfiguration('debug')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for ESP32 C6'
    )

    declare_baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate for ESP32 C6'
    )

    declare_send_hz = DeclareLaunchArgument(
        'send_hz',
        default_value='20.0',
        description='Command send frequency in Hz'
    )

    declare_odom_hz = DeclareLaunchArgument(
        'odom_hz',
        default_value='50.0',
        description='Odometry publish frequency in Hz'
    )

    declare_cmd_vel_timeout = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='0.4',
        description='Timeout in seconds before stopping when cmd_vel is lost'
    )


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

    declare_feedback_pwm_deadzone = DeclareLaunchArgument(
        'feedback_pwm_deadzone',
        default_value='0',
        description='PWM deadzone threshold for wheel speed feedback; 0 keeps encoder feedback unfiltered'
    )

    declare_encoder_ppr = DeclareLaunchArgument(
        'encoder_ppr',
        default_value='1000.0',
        description='Encoder pulses per revolution'
    )

    declare_reduction_ratio = DeclareLaunchArgument(
        'reduction_ratio',
        default_value='56.0',
        description='Motor gearbox reduction ratio'
    )

    declare_ff_factor = DeclareLaunchArgument(
        'ff_factor',
        default_value='310.0',
        description='Feed-forward control factor'
    )

    declare_pid_kp = DeclareLaunchArgument(
        'pid_kp',
        default_value='10.0',
        description='PID proportional gain'
    )

    declare_pid_ki = DeclareLaunchArgument(
        'pid_ki',
        default_value='70.0',
        description='PID integral gain'
    )

    declare_pid_kd = DeclareLaunchArgument(
        'pid_kd',
        default_value='0.0',
        description='PID derivative gain'
    )

    declare_straight_balance_kp = DeclareLaunchArgument(
        'straight_balance_kp',
        default_value='25.0',
        description='Straight driving balance PID proportional gain'
    )

    declare_straight_balance_ki = DeclareLaunchArgument(
        'straight_balance_ki',
        default_value='8.0',
        description='Straight driving balance PID integral gain'
    )

    declare_straight_balance_kd = DeclareLaunchArgument(
        'straight_balance_kd',
        default_value='0.0',
        description='Reserved CFG compatibility field; current ESP32-C6 firmware ignores it'
    )

    declare_cfg_send_on_startup = DeclareLaunchArgument(
        'cfg_send_on_startup',
        default_value='true',
        description='Whether to send CFG command to ESP32-C6 on startup'
    )

    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logs for serial RX/TX and feedback parsing'
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
            'serial_port': serial_port,
            'baudrate': baudrate,
            'send_hz': send_hz,
            'odom_hz': odom_hz,
            'cmd_vel_timeout': cmd_vel_timeout,
            'publish_tf': publish_tf,
            'odom_topic': odom_topic,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'motor1_factor':motor1_factor,
            'motor2_factor':motor2_factor,
            'wheel_diameter': wheel_diameter,
            'wheel_base': wheel_base,
            'feedback_pwm_deadzone': feedback_pwm_deadzone,
            'encoder_ppr': encoder_ppr,
            'reduction_ratio': reduction_ratio,
            'ff_factor': ff_factor,
            'pid_kp': pid_kp,
            'pid_ki': pid_ki,
            'pid_kd': pid_kd,
            'straight_balance_kp': straight_balance_kp,
            'straight_balance_ki': straight_balance_ki,
            'straight_balance_kd': straight_balance_kd,
            'cfg_send_on_startup': cfg_send_on_startup,
            'debug': debug,
        }]
    )

    return LaunchDescription([
        declare_serial_port,
        declare_baudrate,
        declare_send_hz,
        declare_odom_hz,
        declare_cmd_vel_timeout,
        declare_publish_tf,
        declare_odom_topic,
        declare_odom_frame,
        declare_base_frame,
        declare_motor1_factor,
        declare_motor2_factor,
        declare_wheel_diameter,
        declare_wheel_base,
        declare_feedback_pwm_deadzone,
        declare_encoder_ppr,
        declare_reduction_ratio,
        declare_ff_factor,
        declare_pid_kp,
        declare_pid_ki,
        declare_pid_kd,
        declare_straight_balance_kp,
        declare_straight_balance_ki,
        declare_straight_balance_kd,
        declare_cfg_send_on_startup,
        declare_debug,

        tf2_node_base,
        tf2_node_3d_laser,
        tf2_node_laser,
        tf2_node_imu,
        tf2_node_rgbd,
        drive_node,
    ])
