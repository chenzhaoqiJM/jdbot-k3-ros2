#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    tf2_node_base = Node(package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.0', '0.0', '0.0001', '0.0', '0.0', '0.0', '/base_footprint', '/base_link'],
        )

    tf2_node_laser = Node(package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_laser',
        # arguments=['0.07', '0.0', '0.10', '3.14159265', '0.0', '0.0', '/base_link', '/laser_link'],
        arguments=['0.0', '-0.03', '0.10', '0', '0.0', '0.0', '/base_link', '/laser_link'],
        )


    tf2_node_rgbd = Node(package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_rgbd',
        arguments=['0.15', '0', '0.06', '0.0', '0.0', '0.0', '/base_link', '/camera_link'],
        )

    drive_node = Node(package='jdbot_base_bringup',
        executable='ros2_ctrl_odom_node',
        name='ros2_ctrl_odom_node',
        )

    return LaunchDescription([
        # robot_state_pub_node,
        tf2_node_base,
        tf2_node_laser,
        tf2_node_rgbd,
        drive_node,

    ])
