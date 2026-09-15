"""Publish ICP odometry from an RGB-D camera's depth stream.

The depth stream is converted to a point cloud, then ``icp_odometry``
publishes ``/odom`` and can publish the ``odom -> base_footprint`` transform.
This launch file intentionally does not start SLAM, localization,
visualization, or obstacle-detection nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    use_imu = LaunchConfiguration('use_imu')
    imu_topic = LaunchConfiguration('imu_topic')
    imu_qos = LaunchConfiguration('imu_qos')
    always_check_imu_tf = LaunchConfiguration('always_check_imu_tf')
    topic_queue_size = LaunchConfiguration('topic_queue_size')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    qos = LaunchConfiguration('qos')
    odom_reset_countdown = LaunchConfiguration('odom_reset_countdown')
    publish_tf = LaunchConfiguration('publish_tf')

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id,
            'use_sim_time': use_sim_time,
            'publish_tf': ParameterValue(publish_tf, value_type=bool),
            'wait_imu_to_init': ParameterValue(use_imu, value_type=bool),
            'always_check_imu_tf': ParameterValue(
                always_check_imu_tf, value_type=bool),
            'qos_imu': imu_qos,
            'topic_queue_size': topic_queue_size,
            'sync_queue_size': sync_queue_size,
            'qos': qos,
            'wait_for_transform': 0.2,  # 等待传感器到基座 TF 的最长时间，单位秒
            'scan_voxel_size': 0.05,  # 输入点云的体素降采样边长，单位米
            'scan_normal_k': 5,  # 用每个点的 5 个近邻估计表面法线
            'Reg/Force3DoF': 'true',  # 仅估计平面运动 x、y 和 yaw
            'Odom/Strategy': '0',  # 使用帧到局部地图（Frame-to-Map）里程计
            'Odom/ResetCountdown': ParameterValue(
                odom_reset_countdown, value_type=str),  # 连续失败指定帧数后重置
            'Odom/ScanKeyFrameThr': '0.6',  # ICP 内点率低于 60% 时创建关键帧
            'Icp/PointToPlane': 'true',  # 使用点到平面误差进行 ICP 配准
            'Icp/PointToPlaneK': '0',  # 点云已在上游计算法线，不在 ICP 内重复计算
            'Icp/VoxelSize': '0',  # 关闭 ICP 内部体素滤波，避免重复降采样
            'Icp/MaxCorrespondenceDistance': '0.1',  # 对应点最大距离为 0.1 米
            'Icp/CorrespondenceRatio': '0.1',  # 接受变换所需的最小对应点比例
        }],
        remappings=[
            # The real robot also publishes /scan. Keep ICP odometry bound
            # exclusively to the cloud generated from the depth camera.
            ('scan', '/rtabmap/unused_scan'),
            ('scan_cloud', cloud_topic),
            # Keep IMU completely disconnected unless use_imu is enabled.
            ('imu', PythonExpression([
                "'", use_imu, "' == 'true' and '", imu_topic,
                "' or '/rtabmap/unused_imu'",
            ])),
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
            'use_imu',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Use IMU orientation as an initial motion estimate for ICP.')),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/camera/imu',
            description='sensor_msgs/Imu topic used when use_imu is true.'),
        DeclareLaunchArgument(
            'imu_qos',
            default_value='2',
            choices=['0', '1', '2'],
            description=(
                'IMU QoS: 0=system default, 1=reliable, 2=best effort.')),
        DeclareLaunchArgument(
            'always_check_imu_tf',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Keep checking the transform from the IMU frame to the '
                'base frame while IMU input is enabled.')),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            choices=['true', 'false'],
            description='Publish the odom_frame_id -> base_frame_id TF.'),
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
