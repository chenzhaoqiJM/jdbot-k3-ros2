"""Launch RGB-D RTAB-Map with RTAB-Map visual odometry.

Unlike ``rtabmap_rgbd.launch.py``, this launch file does not require an
external odometry source.  ``rgbd_odometry`` publishes ``/odom`` and the
``odom -> base_footprint`` transform from the RGB-D camera stream.
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
    odom_topic = LaunchConfiguration('odom_topic')
    approx_sync = LaunchConfiguration('approx_sync')
    min_inliers = LaunchConfiguration('min_inliers')
    odom_reset_countdown = LaunchConfiguration('odom_reset_countdown')

    parameters = {
        'frame_id': base_frame_id,
        # Keep this empty so RTAB-Map consumes the synchronized /odom topic.
        # rgbd_odometry below still publishes the configured odom TF frame.
        'odom_frame_id': '',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_odom': True,
        'approx_sync': approx_sync,
        'use_action_for_goal': True,
        'Rtabmap/DetectionRate': '8.0',
        'Reg/Force3DoF': 'true',
        'Vis/MinInliers': ParameterValue(min_inliers, value_type=str),
        'Grid/RayTracing': 'true',
        'Grid/3D': 'true',
        'Grid/RangeMax': '3',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '0.6',
        'Optimizer/GravitySigma': '0',
    }

    rgbd_remappings = [
        ('rgb/image', rgb_topic),
        ('rgb/camera_info', camera_info_topic),
        ('depth/image', depth_topic),
    ]
    slam_remappings = rgbd_remappings + [('odom', odom_topic)]

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id,
            'use_sim_time': use_sim_time,
            'publish_tf': True,
            'approx_sync': approx_sync,
            'wait_for_transform': 0.2,
            'Reg/Force3DoF': 'true',
            'Vis/MinInliers': ParameterValue(min_inliers, value_type=str),
            'Odom/ResetCountdown': ParameterValue(
                odom_reset_countdown, value_type=str),
        }],
        remappings=rgbd_remappings + [('odom', odom_topic)],
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
        }],
        remappings=[
            ('depth/image', depth_topic),
            ('depth/camera_info', camera_info_topic),
            ('cloud', '/camera/cloud'),
        ],
    )

    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('cloud', '/camera/cloud'),
            ('obstacles', '/camera/obstacles'),
            ('ground', '/camera/ground'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch RTAB-Map in localization mode.'),
        DeclareLaunchArgument(
            'base_frame_id',
            default_value='base_footprint',
            description='Robot base frame used by visual odometry.'),
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom',
            description='Frame published by RTAB-Map visual odometry.'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Odometry topic published by rgbd_odometry.'),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/image_raw',
            description='RGB image topic.'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Registered depth image topic.'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera_info',
            description='Camera calibration topic for the RGB image.'),
        DeclareLaunchArgument(
            'approx_sync',
            default_value='false',
            description=(
                'Use approximate RGB/depth synchronization. Gazebo RGB-D '
                'streams should normally use false.')),
        DeclareLaunchArgument(
            'min_inliers',
            default_value='10',
            description=(
                'Minimum visual registration inliers. Lower values help in '
                'low-texture simulation scenes but reduce robustness.')),
        DeclareLaunchArgument(
            'odom_reset_countdown',
            default_value='1',
            description=(
                'Reset visual odometry after this many consecutive lost '
                'frames so that tracking can recover. Zero disables reset.')),
        rgbd_odometry,
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
        point_cloud_xyz,
        obstacles_detection,
    ])
