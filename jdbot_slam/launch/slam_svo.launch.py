from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define calibration file path
    calib_file = LaunchConfiguration('calib_file')
    return LaunchDescription([
        # Declare launch argument
        DeclareLaunchArgument(
            'calib_file',
            default_value=os.path.join(
                get_package_share_directory('svo_ros'),
                'param', 'calib', 'euroc_mono.yaml'
            ),
            description='Path to camera calibration file'
        ),

        # SVO node
        Node(
            package='svo_ros',
            executable='svo_node',
            name='svo',
            output='screen',
            # prefix='xterm -e gdb -ex run --args',
            parameters=[
                {'cam0_topic': '/cam0/image_raw'},
                {'cam1_topic': '/cam1/image_raw'},
                {'imu_topic': '/imu0'},
                {'calib_file': calib_file},
                {'runlc': False},
                # "--v=40",
                # "--log-level=debug",  # 确保 ROS2 日志级别足够低   
                os.path.join(
                    get_package_share_directory('svo_ros'),
                    'param', 'frontend_imu','rs_mono_ros2.yaml'
                )
            ],
        ),

        # RViz node
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='vis',
        #     arguments=[
        #         '-d', os.path.join(
        #             get_package_share_directory('svo_ros'),
        #             'rviz_config_vio.rviz'
        #         )
        #     ]
        # ),
    ])
