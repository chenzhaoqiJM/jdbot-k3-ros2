"""Publish ICP odometry from an RGB-D camera's depth stream.

The depth stream is converted to a point cloud, then ``icp_odometry``
publishes ``/odom`` and the ``odom -> base_footprint`` transform.  This
launch file intentionally does not start SLAM, localization, visualization,
or obstacle-detection nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    depth_topic = LaunchConfiguration('depth_topic')
    depth_camera_info_topic = LaunchConfiguration(
        'depth_camera_info_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    topic_queue_size = LaunchConfiguration('topic_queue_size')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    qos = LaunchConfiguration('qos')
    odom_reset_countdown = LaunchConfiguration('odom_reset_countdown')

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id,
            'use_sim_time': use_sim_time,
            'publish_tf': True,
            'topic_queue_size': topic_queue_size,
            'sync_queue_size': sync_queue_size,
            'qos': qos,
            'wait_for_transform': 0.2,
            'scan_voxel_size': 0.05,
            'scan_normal_k': 5,
            'Reg/Force3DoF': 'true',
            'Odom/Strategy': '0',  # Frame-to-Map (F2M)
            'Odom/ResetCountdown': ParameterValue(
                odom_reset_countdown, value_type=str),
            'Odom/ScanKeyFrameThr': '0.6',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '0',
            'Icp/VoxelSize': '0',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/CorrespondenceRatio': '0.1',
        }],
        remappings=[
            # The real robot also publishes /scan. Keep ICP odometry bound
            # exclusively to the cloud generated from the depth camera.
            ('scan', '/rtabmap/unused_scan'),
            ('scan_cloud', cloud_topic),
            ('odom', odom_topic),
        ],
    )

    point_cloud_xyz = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'decimation': 2,
            'max_depth': 3.0,
            'voxel_size': 0.02,
            'qos': qos,
        }],
        remappings=[
            ('depth/image', depth_topic),
            ('depth/camera_info', depth_camera_info_topic),
            ('cloud', cloud_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_footprint',
            description='Robot base frame used by ICP odometry.'),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Frame published by RTAB-Map ICP odometry.'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic published by icp_odometry.'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_rect_raw',
            description='Rectified/registered depth image topic.'),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/depth/camera_info',
            description='Camera calibration topic for the depth image.'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/camera/cloud',
            description='Point cloud generated from the depth image.'),
        DeclareLaunchArgument(
            'topic_queue_size',
            default_value='30',
            description='Queue size for individual camera subscriptions.'),
        DeclareLaunchArgument(
            'sync_queue_size',
            default_value='30',
            description='Queue size used by ICP odometry subscriptions.'),
        DeclareLaunchArgument(
            'qos',
            default_value='2',
            choices=['0', '1', '2'],
            description=(
                'Camera QoS: 0=system default, 1=reliable, 2=best effort.')),
        DeclareLaunchArgument(
            'odom_reset_countdown',
            default_value='5',
            description=(
                'Reset ICP odometry after this many consecutive lost '
                'frames so that tracking can recover. Zero disables reset.')),
        point_cloud_xyz,
        icp_odometry,
    ])
