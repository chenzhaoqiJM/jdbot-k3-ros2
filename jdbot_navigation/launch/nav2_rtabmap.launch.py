import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    controller_type = LaunchConfiguration('controller_type')

    custom_nav_dir = get_package_share_directory('jdbot_navigation')

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py')

    # 用于导航的配置参数
    param_dir = os.path.join(custom_nav_dir, 'config')
    rpp_param_file = os.path.join(param_dir, 'rgbd_nav2_params.yaml')
    mpc_param_file = os.path.join(param_dir, 'rgbd_nav2_params_mpc.yaml')
    selected_param_file = PythonExpression([
        "'", rpp_param_file, "' if '", controller_type,
        "' == 'rpp' else '", mpc_param_file, "'",
    ])
    param_file = LaunchConfiguration(
        'params', default=selected_param_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_type',
            default_value='rpp',
            choices=['rpp', 'mpc'],
            description='Controller parameter set: rpp or mpc'),

        DeclareLaunchArgument(
            'params',
            default_value=selected_param_file,
            description=(
                'Full path to the Nav2 parameter file. Overrides the file '
                'selected by controller_type.')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

    ])
