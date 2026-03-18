from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='default_cam',
            description='Name of the camera'),

        DeclareLaunchArgument(
            'camera_info_url',
            default_value='',
            description='Camera info URL'),

        DeclareLaunchArgument(
            'framerate',
            default_value='30.0',
            description='Camera framerate'),

        DeclareLaunchArgument(
            'frame_id',
            default_value='default_cam',
            description='Frame ID'),

        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Image height'),

        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Image width'),

        DeclareLaunchArgument(
            'io_method',
            default_value='mmap',
            description='IO method[mmap、]'),

        DeclareLaunchArgument(
            'pixel_format',
            default_value='yuyv',
            description='Pixel format[raw_mjpeg, yuyv, mjpeg2rgb]'),

        DeclareLaunchArgument(
            'av_device_format',
            default_value='YUV422P',
            description='AV device format'),

        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Video device path'),

        DeclareLaunchArgument(
            'brightness',
            default_value='50',
            description='Camera brightness'),

        DeclareLaunchArgument(
            'contrast',
            default_value='-1',
            description='Camera contrast'),

        DeclareLaunchArgument(
            'saturation',
            default_value='-1',
            description='Camera saturation'),

        DeclareLaunchArgument(
            'sharpness',
            default_value='-1',
            description='Camera sharpness'),

        DeclareLaunchArgument(
            'gain',
            default_value='-1',
            description='Camera gain'),

        DeclareLaunchArgument(
            'auto_white_balance',
            default_value='true',
            description='Whether to use auto white balance'),

        DeclareLaunchArgument(
            'white_balance',
            default_value='4000',
            description='White balance setting'),

        DeclareLaunchArgument(
            'autoexposure',
            default_value='true',
            description='Whether to use auto exposure'),

        DeclareLaunchArgument(
            'exposure',
            default_value='30',
            description='Exposure setting[40 ~ 1000]'),

        DeclareLaunchArgument(
            'autofocus',
            default_value='false',
            description='Whether to use autofocus'),

        DeclareLaunchArgument(
            'focus',
            default_value='-1',
            description='Focus setting'),

        # Node for jobot_usb_cam
        Node(
            package='jobot_usb_cam',
            executable='jobot_usb_cam_node_exe',
            name='jobot_usb_cam_node',
            output='screen',
            parameters=[
                {'camera_name': LaunchConfiguration('camera_name')},
                {'camera_info_url': LaunchConfiguration('camera_info_url')},
                {'framerate': LaunchConfiguration('framerate')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'image_height': LaunchConfiguration('image_height')},
                {'image_width': LaunchConfiguration('image_width')},
                {'io_method': LaunchConfiguration('io_method')},
                {'pixel_format': LaunchConfiguration('pixel_format')},
                {'av_device_format': LaunchConfiguration('av_device_format')},
                {'video_device': LaunchConfiguration('video_device')},
                {'brightness': LaunchConfiguration('brightness')},
                {'contrast': LaunchConfiguration('contrast')},
                {'saturation': LaunchConfiguration('saturation')},
                {'sharpness': LaunchConfiguration('sharpness')},
                {'gain': LaunchConfiguration('gain')},
                {'auto_white_balance': LaunchConfiguration('auto_white_balance')},
                {'white_balance': LaunchConfiguration('white_balance')},
                {'autoexposure': LaunchConfiguration('autoexposure')},
                {'exposure': LaunchConfiguration('exposure')},
                {'autofocus': LaunchConfiguration('autofocus')},
                {'focus': LaunchConfiguration('focus')},
            ],
            additional_env={'PYTHONUNBUFFERED': '1'}
        ),
    ])
