from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("k3_rgbd_odometry"), "config", "k3_d415.yaml"
    )
    return LaunchDescription([
        Node(
            package="k3_rgbd_odometry",
            executable="rgbd_odometry_node",
            name="k3_rgbd_odometry",
            output="screen",
            parameters=[config],
        )
    ])
