from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('node_name', default_value='cuvslam_rgbd'),
        DeclareLaunchArgument('color_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('output_odom_topic', default_value='/cuvslam/odom'),
        DeclareLaunchArgument('tracking_topic', default_value='/cuvslam/tracking'),
        DeclareLaunchArgument('rig_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='cuvslam_odom'),
        DeclareLaunchArgument('depth_scale_factor', default_value='1000.0'),
        DeclareLaunchArgument('sync_slop_s', default_value='0.025'),
        DeclareLaunchArgument('sync_queue_size', default_value='30'),
        DeclareLaunchArgument('qos_depth', default_value='40'),
        DeclareLaunchArgument('report_interval_s', default_value='5.0'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(
            package='cuvslam_rgbd',
            executable='rgbd_odometry',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[{
                'color_topic': LaunchConfiguration('color_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'output_odom_topic': LaunchConfiguration('output_odom_topic'),
                'tracking_topic': LaunchConfiguration('tracking_topic'),
                'rig_frame': LaunchConfiguration('rig_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'depth_scale_factor': ParameterValue(
                    LaunchConfiguration('depth_scale_factor'), value_type=float),
                'sync_slop_s': ParameterValue(
                    LaunchConfiguration('sync_slop_s'), value_type=float),
                'sync_queue_size': ParameterValue(
                    LaunchConfiguration('sync_queue_size'), value_type=int),
                'qos_depth': ParameterValue(
                    LaunchConfiguration('qos_depth'), value_type=int),
                'report_interval_s': ParameterValue(
                    LaunchConfiguration('report_interval_s'), value_type=float),
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf'), value_type=bool),
            }],
        ),
    ])
