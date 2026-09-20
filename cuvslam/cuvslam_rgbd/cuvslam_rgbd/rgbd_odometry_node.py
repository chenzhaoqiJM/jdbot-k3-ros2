#!/usr/bin/env python3
"""ROS 2 wrapper for CPU/RVV cuVSLAM RGB-D odometry."""

import time

import cuvslam
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class RGBDOdometryNode(Node):
    """Track synchronized RGB-D frames and publish cuVSLAM odometry."""

    def __init__(self) -> None:
        super().__init__('cuvslam_rgbd')
        defaults = {
            'color_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'output_odom_topic': '/cuvslam/odom',
            'tracking_topic': '/cuvslam/tracking',
            'rig_frame': 'base_footprint',
            'odom_frame': 'cuvslam_odom',
            'depth_scale_factor': 1000.0,
            'sync_slop_s': 0.025,
            'sync_queue_size': 30,
            'qos_depth': 40,
            'report_interval_s': 5.0,
            'publish_tf': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.tracker = None
        self.frame_count = 0
        self.tracked_count = 0
        self.failed_count = 0
        self.processing_seconds = 0.0
        self.last_init_warning = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self) if self.param('publish_tf') else None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self.param('qos_depth')),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, str(self.param('camera_info_topic')), self.on_camera_info, sensor_qos)
        self.color_subscription = message_filters.Subscriber(
            self, Image, str(self.param('color_topic')), qos_profile=sensor_qos)
        self.depth_subscription = message_filters.Subscriber(
            self, Image, str(self.param('depth_topic')), qos_profile=sensor_qos)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_subscription, self.depth_subscription],
            queue_size=int(self.param('sync_queue_size')),
            slop=float(self.param('sync_slop_s')),
        )
        self.synchronizer.registerCallback(self.on_rgbd)

        self.odom_publisher = self.create_publisher(
            Odometry, str(self.param('output_odom_topic')), 20)
        self.tracking_publisher = self.create_publisher(
            Bool, str(self.param('tracking_topic')), 10)
        self.create_timer(float(self.param('report_interval_s')), self.report)

        self.get_logger().info(
            f'Waiting for RGB-D input: color={self.param("color_topic")}, '
            f'depth={self.param("depth_topic")}, info={self.param("camera_info_topic")}')

    def param(self, name: str):
        return self.get_parameter(name).value

    def on_camera_info(self, msg: CameraInfo) -> None:
        if self.tracker is not None:
            return

        rig_frame = str(self.param('rig_frame'))
        camera_frame = msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                rig_frame, camera_frame, Time(), timeout=Duration(seconds=0.05))
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_init_warning > 2.0:
                self.get_logger().warning(
                    f'Waiting for transform {rig_frame} <- {camera_frame}: {exc}')
                self.last_init_warning = now
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rig_from_camera = cuvslam.Pose(
            rotation=[rotation.x, rotation.y, rotation.z, rotation.w],
            translation=[translation.x, translation.y, translation.z],
        )
        camera = cuvslam.Camera(
            size=[int(msg.width), int(msg.height)],
            principal=[float(msg.k[2]), float(msg.k[5])],
            focal=[float(msg.k[0]), float(msg.k[4])],
            rig_from_camera=rig_from_camera,
        )
        if msg.distortion_model == 'plumb_bob' and len(msg.d) >= 5:
            camera.distortion = cuvslam.Distortion(
                cuvslam.Distortion.Model.Brown,
                [float(msg.d[0]), float(msg.d[1]), float(msg.d[4]),
                 float(msg.d[2]), float(msg.d[3])],
            )
        elif msg.distortion_model == 'rational_polynomial' and len(msg.d) >= 8:
            camera.distortion = cuvslam.Distortion(
                cuvslam.Distortion.Model.Polynomial,
                [float(msg.d[index]) for index in range(8)],
            )

        rgbd_settings = cuvslam.Odometry.RGBDSettings(
            depth_scale_factor=float(self.param('depth_scale_factor')),
            depth_camera_id=0,
            enable_depth_stereo_tracking=False,
        )
        config = cuvslam.Odometry.Config(
            odometry_mode=cuvslam.Odometry.OdometryMode.RGBD,
            use_gpu=False,
            async_sba=False,
            use_motion_model=True,
            enable_observations_export=False,
            enable_landmarks_export=False,
            rgbd_settings=rgbd_settings,
        )
        self.tracker = cuvslam.Tracker(cuvslam.Rig([camera]), config)
        self.get_logger().info(
            f'CPU/RVV RGB-D ready: {msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.3f}, fy={msg.k[4]:.3f}, '
            f'rig={rig_frame}, camera={camera_frame}')

    @staticmethod
    def color_array(msg: Image) -> np.ndarray:
        if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            raise ValueError(f'unsupported color encoding: {msg.encoding}')
        channels = 1 if msg.encoding == 'mono8' else 3
        required_step = int(msg.width) * channels
        if msg.step < required_step:
            raise ValueError(f'invalid color step: {msg.step} < {required_step}')
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), int(msg.step))
        if channels == 1:
            image = raw[:, :int(msg.width)]
        else:
            image = raw[:, :required_step].reshape(int(msg.height), int(msg.width), channels)
            if msg.encoding == 'bgr8':
                image = image[:, :, ::-1]
        return np.ascontiguousarray(image)

    @staticmethod
    def depth_array(msg: Image) -> np.ndarray:
        if msg.encoding not in ('16UC1', 'mono16'):
            raise ValueError(f'unsupported depth encoding: {msg.encoding}')
        required_step = int(msg.width) * 2
        if msg.step < required_step:
            raise ValueError(f'invalid depth step: {msg.step} < {required_step}')
        byte_rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(int(msg.height), int(msg.step))
        depth = byte_rows[:, :required_step].copy().view(np.uint16).reshape(
            int(msg.height), int(msg.width))
        if bool(msg.is_bigendian) == np.little_endian:
            depth.byteswap(inplace=True)
        return depth

    def on_rgbd(self, color_msg: Image, depth_msg: Image) -> None:
        if self.tracker is None:
            return
        try:
            color = self.color_array(color_msg)
            depth = self.depth_array(depth_msg)
            timestamp_ns = (
                int(color_msg.header.stamp.sec) * 1_000_000_000
                + int(color_msg.header.stamp.nanosec)
            )
            start = time.perf_counter()
            estimate, _ = self.tracker.track(timestamp_ns, images=[color], depths=[depth])
            self.processing_seconds += time.perf_counter() - start
        except Exception as exc:
            self.frame_count += 1
            self.failed_count += 1
            self.tracking_publisher.publish(Bool(data=False))
            self.get_logger().error(f'RGB-D frame failed: {exc}')
            return

        self.frame_count += 1
        if estimate.world_from_rig is None:
            self.failed_count += 1
            self.tracking_publisher.publish(Bool(data=False))
            return

        self.tracked_count += 1
        self.tracking_publisher.publish(Bool(data=True))
        pose = estimate.world_from_rig.pose
        position = np.asarray(pose.translation, dtype=np.float64)
        quaternion = np.asarray(pose.rotation, dtype=np.float64)

        output = Odometry()
        output.header.stamp = color_msg.header.stamp
        output.header.frame_id = str(self.param('odom_frame'))
        output.child_frame_id = str(self.param('rig_frame'))
        output.pose.pose.position.x = float(position[0])
        output.pose.pose.position.y = float(position[1])
        output.pose.pose.position.z = float(position[2])
        output.pose.pose.orientation.x = float(quaternion[0])
        output.pose.pose.orientation.y = float(quaternion[1])
        output.pose.pose.orientation.z = float(quaternion[2])
        output.pose.pose.orientation.w = float(quaternion[3])
        covariance = np.asarray(
            estimate.world_from_rig.covariance_xyz_rpy, dtype=np.float64).reshape(6, 6)
        output.pose.covariance = covariance.reshape(-1).tolist()
        self.odom_publisher.publish(output)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = output.child_frame_id
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def report(self) -> None:
        if self.frame_count == 0:
            return
        processing_fps = self.frame_count / max(self.processing_seconds, 1e-9)
        success_rate = 100.0 * self.tracked_count / self.frame_count
        self.get_logger().info(
            f'frames={self.frame_count}, tracked={self.tracked_count}, '
            f'failed={self.failed_count}, success={success_rate:.1f}%, '
            f'processing_fps={processing_fps:.1f}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RGBDOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
