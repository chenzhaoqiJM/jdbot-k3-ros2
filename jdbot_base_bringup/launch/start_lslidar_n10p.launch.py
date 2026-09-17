import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    driver_config = os.path.join(get_package_share_directory('jdbot_base_bringup'),'config','lslidar_n10p.yaml')


    driver_node = LifecycleNode(package='lslidar_driver',
            executable='lslidar_driver_node',
            name='lslidar_driver_node',
            namespace='x10', # 与对应yaml文件中命名空间一致
            parameters=[driver_config, {'laserscan_topic': '/scan'}],
            output='screen'
            )

    return LaunchDescription([
        driver_node
    ])
