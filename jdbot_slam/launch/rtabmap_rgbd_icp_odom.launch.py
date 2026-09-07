"""Launch RGB-D RTAB-Map with depth-cloud ICP odometry on the real robot.

Unlike ``rtabmap_rgbd.launch.py``, this launch file does not require an
external odometry source.  The depth stream is converted to a point cloud,
then ``icp_odometry`` publishes ``/odom`` and the ``odom -> base_footprint``
transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    depth_camera_info_topic = LaunchConfiguration(
        'depth_camera_info_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    approx_sync = LaunchConfiguration('approx_sync')
    approx_sync_max_interval = LaunchConfiguration(
        'approx_sync_max_interval')
    topic_queue_size = LaunchConfiguration('topic_queue_size')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    qos = LaunchConfiguration('qos')
    launch_viz = LaunchConfiguration('launch_viz')
    min_inliers = LaunchConfiguration('min_inliers')
    odom_reset_countdown = LaunchConfiguration('odom_reset_countdown')

    parameters = {
        'frame_id': base_frame_id,
        # Keep this empty so RTAB-Map consumes the synchronized /odom topic.
        # icp_odometry below still publishes the configured odom TF frame.
        'odom_frame_id': '',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_odom': True,
        'approx_sync': approx_sync,
        'approx_sync_max_interval': approx_sync_max_interval,
        'topic_queue_size': topic_queue_size,
        'sync_queue_size': sync_queue_size,
        'qos': qos,
        'qos_camera_info': qos,
        'use_action_for_goal': True,
        'Rtabmap/DetectionRate': '8.0',
        'Reg/Force3DoF': 'true',
        'Vis/MinInliers': ParameterValue(min_inliers, value_type=str),
        'Grid/RayTracing': 'true',
        'Grid/3D': 'false',
        'Grid/RangeMax': '3',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '0.4',
        'Optimizer/GravitySigma': '0',
    }

    rgbd_remappings = [
        ('rgb/image', rgb_topic),
        ('rgb/camera_info', camera_info_topic),
        ('depth/image', depth_topic),
    ]
    slam_remappings = rgbd_remappings + [('odom', odom_topic)]

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

    rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[parameters],
        remappings=slam_remappings,
        arguments=['-d'],
    )

    rtabmap_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[parameters, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True',
        }],
        remappings=slam_remappings,
    )

    rtabmap_viz = Node(
        condition=IfCondition(launch_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[parameters],
        remappings=slam_remappings,
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

    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('cloud', cloud_topic),
            ('obstacles', '/camera/obstacles'),
            ('ground', '/camera/ground'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch RTAB-Map in localization mode.'),
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
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic.'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_rect_raw',
            description='Rectified/registered depth image topic.'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/color/camera_info',
            description='Camera calibration topic for the RGB image.'),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/depth/camera_info',
            description='Camera calibration topic for the depth image.'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/camera/cloud',
            description='Point cloud generated from the depth image.'),
        DeclareLaunchArgument(
            'approx_sync',
            default_value='true',
            description=(
                'Use approximate RGB/depth synchronization for hardware '
                'camera streams.')),
        DeclareLaunchArgument(
            'approx_sync_max_interval',
            default_value='0.05',
            description='Maximum RGB/depth timestamp difference in seconds.'),
        DeclareLaunchArgument(
            'topic_queue_size',
            default_value='30',
            description='Queue size for individual camera subscriptions.'),
        DeclareLaunchArgument(
            'sync_queue_size',
            default_value='30',
            description='Queue size for the RGB-D synchronizer.'),
        DeclareLaunchArgument(
            'qos',
            default_value='2',
            choices=['0', '1', '2'],
            description=(
                'Camera QoS: 0=system default, 1=reliable, 2=best effort.')),
        DeclareLaunchArgument(
            'launch_viz',
            default_value='false',
            description='Launch rtabmap_viz (disabled to save robot CPU).'),
        DeclareLaunchArgument(
            'min_inliers',
            default_value='20',
            description=(
                'Minimum visual registration inliers. Increase this value '
                'to reject weak matches.')),
        DeclareLaunchArgument(
            'odom_reset_countdown',
            default_value='5',
            description=(
                'Reset ICP odometry after this many consecutive lost '
                'frames so that tracking can recover. Zero disables reset.')),
        point_cloud_xyz,
        icp_odometry,
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
        obstacles_detection,
    ])
