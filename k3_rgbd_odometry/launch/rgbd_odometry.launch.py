import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("k3_rgbd_odometry"), "config", "k3_d415.yaml"
    )

    launch_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML configuration file; launch arguments below override it",
        ),
        DeclareLaunchArgument(
            "namespace", default_value="", description="ROS namespace of the odometry node"
        ),
        DeclareLaunchArgument(
            "node_name", default_value="k3_rgbd_odometry", description="Odometry node name"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use /clock instead of wall time"
        ),
        DeclareLaunchArgument(
            "color_topic", default_value="/camera/color/image_raw", description="Color image topic"
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/aligned_depth_to_color/image_raw",
            description="Depth image aligned to color topic",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/color/camera_info",
            description="Color camera calibration topic",
        ),
        DeclareLaunchArgument(
            "odom_topic", default_value="/rgbd_odom/odom", description="Odometry output topic"
        ),
        DeclareLaunchArgument(
            "odom_frame", default_value="odom_rgbd", description="Odometry reference frame"
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_footprint",
            description="Robot base frame used to look up camera extrinsics",
        ),
        DeclareLaunchArgument(
            "child_frame",
            default_value="base_footprint_rgbd",
            description="Child frame written into odometry and TF output",
        ),
        DeclareLaunchArgument(
            "publish_tf", default_value="true", description="Publish odom-to-child TF"
        ),
        DeclareLaunchArgument(
            "processing_rate_hz",
            default_value="20.0",
            description="Maximum RGB-D processing rate",
        ),
        DeclareLaunchArgument(
            "expected_frame_rate_hz",
            default_value="15.0",
            description="Expected camera frame rate used by gap handling",
        ),
        DeclareLaunchArgument(
            "sync_tolerance_ms",
            default_value="40.0",
            description="Maximum color/depth timestamp difference",
        ),
        DeclareLaunchArgument(
            "min_depth_m", default_value="0.25", description="Nearest accepted depth"
        ),
        DeclareLaunchArgument(
            "max_depth_m", default_value="6.0", description="Farthest accepted depth"
        ),
        DeclareLaunchArgument(
            "max_features",
            default_value="420",
            description="Maximum tracked features; lower it to reduce CPU load",
        ),
        DeclareLaunchArgument(
            "loop_closure_enabled",
            default_value="true",
            description="Enable lightweight loop-closure correction",
        ),
    ]

    parameter_overrides = {
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        "color_topic": LaunchConfiguration("color_topic"),
        "depth_topic": LaunchConfiguration("depth_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        "odom_topic": LaunchConfiguration("odom_topic"),
        "odom_frame": LaunchConfiguration("odom_frame"),
        "base_frame": LaunchConfiguration("base_frame"),
        "child_frame": LaunchConfiguration("child_frame"),
        "publish_tf": ParameterValue(LaunchConfiguration("publish_tf"), value_type=bool),
        "processing_rate_hz": ParameterValue(
            LaunchConfiguration("processing_rate_hz"), value_type=float
        ),
        "expected_frame_rate_hz": ParameterValue(
            LaunchConfiguration("expected_frame_rate_hz"), value_type=float
        ),
        "sync_tolerance_ms": ParameterValue(
            LaunchConfiguration("sync_tolerance_ms"), value_type=float
        ),
        "min_depth_m": ParameterValue(LaunchConfiguration("min_depth_m"), value_type=float),
        "max_depth_m": ParameterValue(LaunchConfiguration("max_depth_m"), value_type=float),
        "max_features": ParameterValue(LaunchConfiguration("max_features"), value_type=int),
        "loop_closure.enabled": ParameterValue(
            LaunchConfiguration("loop_closure_enabled"), value_type=bool
        ),
    }

    node = Node(
        package="k3_rgbd_odometry",
        executable="rgbd_odometry_node",
        namespace=LaunchConfiguration("namespace"),
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[LaunchConfiguration("config_file"), parameter_overrides],
    )

    return LaunchDescription(launch_arguments + [node])
