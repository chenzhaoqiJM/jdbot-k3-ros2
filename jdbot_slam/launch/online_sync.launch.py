from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # remappings=[('odom','odom_my')]
    return LaunchDescription([
        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("jdbot_slam") + '/config/mapper_params_online_sync.yaml'
        	],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
        )
    ])
