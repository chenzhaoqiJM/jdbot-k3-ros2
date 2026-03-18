import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = True

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': False,
            'approx_sync': False,
            'Reg/Strategy': '1',
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Grid/FromDepth': 'false',
            'Grid/3D': 'true',
            'Grid/RangeMax': '20.0',
            'Grid/CellSize': '0.1',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Rtabmap/DetectionRate': '10.0',
            'Optimizer/GravitySigma': '0',
            'publish_tf': True,
        }],
        remappings=[
            ('odom', '/odom'),
            ('scan_cloud', '/points'),
        ],
        arguments=['-d'],
    )

    lidar_icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='lidar_icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'lidar_odom',
            'guess_frame_id': 'odom',
            'guess_min_translation': 0.02,
            'guess_min_rotation': 0.02,
            'publish_tf': False,
            'wait_for_transform': 0.2,
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/MaxTranslation': '2.0',
            'Icp/MaxRotation': '1.0',
            'Odom/Strategy': '0',
        }],
        remappings=[
            ('scan_cloud', '/points'),
            ('odom', '/lidar_icp_odom'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_scan_cloud': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
        }],
        remappings=[
            ('odom', '/odom'),
            ('scan_cloud', '/points'),
        ],
    )

    return LaunchDescription([
        rtabmap,
    ])
