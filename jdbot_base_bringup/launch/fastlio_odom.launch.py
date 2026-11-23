"""Convert Fast LIO2 odometry to planar odometry and TF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_stamp_source',
            default_value='now',
            choices=['now', 'odometry'],
            description='Odometry and TF timestamp: publication time or input Odometry header stamp',
        ),
        Node(
            package='jdbot_base_bringup',
            executable='fastlio_odom_converter_node',
            name='fastlio_odom_converter_node',
            parameters=[{
                'input_topic': '/Odometry',
                'output_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'fixed_z': 0.0,
                'publish_tf': True,
                'tf_stamp_source': LaunchConfiguration('tf_stamp_source'),
                # Larger gaps or backward time reset the velocity estimate.
                'max_dt': 1.0,
                # x, y, z, roll, pitch, yaw; tune for the input data and sampling rate.
                'twist_covariance_diagonal': [0.1, 0.1, 1e6, 1e6, 1e6, 0.1],
            }],
            output='screen',
        ),
    ])
