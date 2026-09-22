from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        "node_name": "rgbd_odometry",
        "color_topic": "/camera/color/image_raw",
        "depth_topic": "/camera/aligned_depth_to_color/image_raw",
        "camera_info_topic": "/camera/color/camera_info",
        "output_odom_topic": "/rgbd_odometry/odom",
        "tracking_topic": "/rgbd_odometry/tracking",
        "rig_frame": "base_footprint",
        "odom_frame": "rgbd_odom",
        "depth_scale_factor": "1000.0",
        "min_depth_m": "0.2",
        "max_depth_m": "4.5",
        "sync_slop_s": "0.015",
        "sync_queue_size": "30",
        "qos_depth": "40",
        "input_reliable_qos": "true",
        "planar_mode": "true",
        "publish_tf": "false",
        "report_interval_s": "5.0",
        "coarse_stride": "8",
        "fine_stride": "4",
        "coarse_iterations": "8",
        "fine_iterations": "3",
        "max_correspondence_m": "0.08",
        "depth_filter_radius": "1",
        "depth_filter_max_difference_m": "0.05",
        "coarse_min_correspondences": "40",
        "fine_min_correspondences": "180",
        "color_static_threshold": "8.0",
        "static_keyframe_max_frames": "60",
        "photometric_weight": "0.1",
        "photometric_max_weight": "0.18",
        "photometric_adaptive_threshold": "0.1",
        "photometric_affine_compensation": "false",
        "local_keyframe_fusion_weight": "0.0",
        "local_keyframe_max_frames": "12",
        "local_keyframe_max_translation_m": "0.15",
        "local_keyframe_max_rotation_rad": "0.15",
        "local_keyframe_consistency_translation_m": "0.06",
        "local_keyframe_consistency_rotation_rad": "0.08",
        "sparse_fusion_weight": "0.20",
        "sparse_translation_fusion_weight": "0.20",
        "sparse_max_translation_disagreement_m": "0.06",
        "sparse_max_rotation_disagreement_rad": "0.08",
        "window_max_keyframes": "3",
        "window_evaluation_interval": "3",
        "window_max_translation_m": "0.45",
        "window_max_rotation_rad": "0.45",
        "window_fusion_weight": "0.12",
        "map_max_keyframes": "20",
        "map_query_interval": "30",
        "map_min_keyframe_age": "30",
        "map_min_inliers": "8",
        "map_keyframe_translation_m": "0.20",
        "map_keyframe_rotation_rad": "0.20",
        "map_query_translation_m": "0.80",
        "map_query_rotation_rad": "0.65",
        "map_max_correction_translation_m": "0.80",
        "map_max_correction_rotation_rad": "0.40",
        "map_fusion_weight": "0.50",
        "planar_translation_deadband_m": "0.0015",
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()
    ]
    bool_names = {
        "planar_mode", "publish_tf", "input_reliable_qos",
        "photometric_affine_compensation",
    }
    int_names = {
        "sync_queue_size", "qos_depth", "coarse_stride", "fine_stride",
        "coarse_iterations", "fine_iterations",
        "coarse_min_correspondences", "fine_min_correspondences",
        "static_keyframe_max_frames", "local_keyframe_max_frames",
        "window_max_keyframes", "window_evaluation_interval",
        "map_max_keyframes", "map_query_interval", "map_min_keyframe_age",
        "map_min_inliers",
        "depth_filter_radius",
    }
    parameters = {}
    for name in defaults:
        if name == "node_name":
            continue
        value = LaunchConfiguration(name)
        if name in bool_names:
            parameters[name] = ParameterValue(value, value_type=bool)
        elif name in int_names:
            parameters[name] = ParameterValue(value, value_type=int)
        elif name in {
            "depth_scale_factor", "min_depth_m", "max_depth_m", "sync_slop_s",
            "report_interval_s", "max_correspondence_m",
            "depth_filter_max_difference_m",
            "color_static_threshold",
            "photometric_weight", "photometric_max_weight",
            "photometric_adaptive_threshold",
            "local_keyframe_fusion_weight",
            "local_keyframe_max_translation_m",
            "local_keyframe_max_rotation_rad",
            "local_keyframe_consistency_translation_m",
            "local_keyframe_consistency_rotation_rad",
            "sparse_fusion_weight",
            "sparse_translation_fusion_weight",
            "sparse_max_translation_disagreement_m",
            "sparse_max_rotation_disagreement_rad",
            "window_max_translation_m",
            "window_max_rotation_rad",
            "window_fusion_weight",
            "map_keyframe_translation_m",
            "map_keyframe_rotation_rad",
            "map_query_translation_m",
            "map_query_rotation_rad",
            "map_max_correction_translation_m",
            "map_max_correction_rotation_rad",
            "map_fusion_weight",
            "planar_translation_deadband_m",
        }:
            parameters[name] = ParameterValue(value, value_type=float)
        else:
            parameters[name] = value
    node = Node(
        package="rgbd_odometry",
        executable="rgbd_odometry_node",
        name=LaunchConfiguration("node_name"),
        output="screen",
        emulate_tty=True,
        parameters=[parameters],
    )
    return LaunchDescription(declarations + [node])
