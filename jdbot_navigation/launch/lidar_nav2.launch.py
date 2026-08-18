import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    myrobot_nav_dir = get_package_share_directory('jdbot_navigation')
    myrobot_nav_launchr = os.path.join(myrobot_nav_dir, 'launch')

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # 地图文件
    map_dir = os.path.join(myrobot_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(map_dir, 'my_map.yaml'))

    # 用于导航的配置参数
    param_dir = os.path.join(myrobot_nav_dir, 'config')
    param_file = LaunchConfiguration('params', default=os.path.join(param_dir, 'jdbot_diff_mpc.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

    ])
