from ament_index_python.packages import get_package_share_directory
import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')

    bringup_dir = get_package_share_directory('rdk_description')
    launch_dir = os.path.join(bringup_dir, 'launch')

    robot_base = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_base.launch.py')),
            launch_arguments={'carto_slam': 'true'}.items(),
    )
    robot_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_lidar.launch.py')),
    )


    return LaunchDescription([
        robot_base, robot_lidar,

        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("jdbot_slam") + '/config/mapper_params_online_async.yaml'
        	],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time':use_sim_time}]
        )
    ])
