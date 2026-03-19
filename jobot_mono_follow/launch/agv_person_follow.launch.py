from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'publish_result_img',
            default_value='false',
            description='是否发布带检测框的结果图像'),

        DeclareLaunchArgument(
            'sub_image_topic',
            default_value='/image_raw',
            description='输入相机图像话题'),

        DeclareLaunchArgument(
            'linear_x',
            default_value='0.4',
            description='小车前进线速度上限（m/s）'),

        DeclareLaunchArgument(
            'angular_z',
            default_value='0.37',
            description='小车转向角速度上限（rad/s）'),

        DeclareLaunchArgument(
            'image_center_x',
            default_value='320.0',
            description='图像中心 x 坐标，需与相机分辨率匹配'),

        DeclareLaunchArgument(
            'allowable_deviation',
            default_value='52.0',
            description='允许直行的中心偏差范围'),

        DeclareLaunchArgument(
            'angular_deadband',
            default_value='22.0',
            description='转向死区，小于该偏差不转向'),

        DeclareLaunchArgument(
            'angular_soft_zone',
            default_value='135.0',
            description='软转向区范围，区内减弱转向指令'),

        DeclareLaunchArgument(
            'angular_smooth_ratio',
            default_value='0.82',
            description='角速度平滑系数，越大越平滑'),

        DeclareLaunchArgument(
            'min_curve_speed_ratio',
            default_value='0.42',
            description='转弯时保留的最小前进速度比例'),

        DeclareLaunchArgument(
            'target_lost_timeout',
            default_value='4.0',
            description='目标丢失后停车超时时间（秒）'),

        DeclareLaunchArgument(
            'stop_y1_threshold',
            default_value='10',
            description='目标框顶部接近图像顶端时的停车阈值'),

        DeclareLaunchArgument(
            'steering_kp_scale',
            default_value='1.0',
            description='转向比例系数缩放'),

        DeclareLaunchArgument(
            'steering_kd_ratio',
            default_value='14.0',
            description='转向微分系数相对比例'),

        DeclareLaunchArgument(
            'angular_cmd_deadband_ratio',
            default_value='0.15',
            description='角速度输出死区比例'),

        DeclareLaunchArgument(
            'angular_cmd_deadband_min',
            default_value='0.025',
            description='角速度最小输出死区'),

        DeclareLaunchArgument(
            'max_linear_step_ratio',
            default_value='0.9',
            description='线速度最大变化率比例'),

        DeclareLaunchArgument(
            'max_angular_step_ratio',
            default_value='1.1',
            description='角速度最大变化率比例'),

        Node(
            package='jobot_mono_follow',
            executable='agv_follow_node',
            name='agv_follow_node',
            output='screen',
            parameters=[
                {'publish_result_img': LaunchConfiguration('publish_result_img')},
                {'sub_image_topic': LaunchConfiguration('sub_image_topic')},
                {'linear_x': LaunchConfiguration('linear_x')},
                {'angular_z': LaunchConfiguration('angular_z')},
                {'image_center_x': LaunchConfiguration('image_center_x')},
                {'allowable_deviation': LaunchConfiguration('allowable_deviation')},
                {'angular_deadband': LaunchConfiguration('angular_deadband')},
                {'angular_soft_zone': LaunchConfiguration('angular_soft_zone')},
                {'angular_smooth_ratio': LaunchConfiguration('angular_smooth_ratio')},
                {'min_curve_speed_ratio': LaunchConfiguration('min_curve_speed_ratio')},
                {'target_lost_timeout': LaunchConfiguration('target_lost_timeout')},
                {'stop_y1_threshold': LaunchConfiguration('stop_y1_threshold')},
                {'steering_kp_scale': LaunchConfiguration('steering_kp_scale')},
                {'steering_kd_ratio': LaunchConfiguration('steering_kd_ratio')},
                {'angular_cmd_deadband_ratio': LaunchConfiguration('angular_cmd_deadband_ratio')},
                {'angular_cmd_deadband_min': LaunchConfiguration('angular_cmd_deadband_min')},
                {'max_linear_step_ratio': LaunchConfiguration('max_linear_step_ratio')},
                {'max_angular_step_ratio': LaunchConfiguration('max_angular_step_ratio')},
            ],
            additional_env={'PYTHONUNBUFFERED': '1'}
        ),
    ])
