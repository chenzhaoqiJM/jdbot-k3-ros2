#!/usr/bin/env python3
"""ROS 2 wrapper for CPU/RVV cuVSLAM RGB-D odometry."""

from collections import deque
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
from sensor_msgs.msg import CameraInfo, Image, Imu
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
            'enable_imu': False,
            'imu_topic': '/camera/imu',
            'imu_frequency': 200.0,
            'imu_queue_size': 2000,
            'imu_wait_s': 0.15,
            'gyroscope_noise_density': 0.006067337,
            'gyroscope_random_walk': 0.00003621195,
            'accelerometer_noise_density': 0.033621979,
            'accelerometer_random_walk': 0.0009825659,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.use_imu = bool(self.param('enable_imu'))
        if int(self.param('imu_queue_size')) < 2 or float(self.param('imu_wait_s')) <= 0:
            raise ValueError('imu_queue_size must be >= 2 and imu_wait_s must be positive')
        self.imu_samples = deque()
        self.pending_frames = deque()
        self.camera_info = None
        self.imu_frame = None
        self.last_imu_received_ns = -1
        self.last_tracker_timestamp_ns = -1
        self.last_image_timestamp_ns = -1
        self.imu_submitted = 0
        self.imu_dropped = 0
        self.image_dropped = 0
        self.last_imu_warning = 0.0
        self.imu_initialized = False
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
        if self.use_imu:
            imu_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=int(self.param('imu_queue_size')),
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            self.imu_subscription = self.create_subscription(
                Imu, str(self.param('imu_topic')), self.on_imu, imu_qos)
            self.create_timer(0.01, self.drain_frames)

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

    @staticmethod
    def timestamp_ns(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def imu_warning(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_imu_warning > 2.0:
            self.get_logger().warning(text)
            self.last_imu_warning = now

    def on_imu(self, msg: Imu) -> None:
        timestamp = self.timestamp_ns(msg)
        a, w = msg.linear_acceleration, msg.angular_velocity
        if (not msg.header.frame_id or not np.isfinite([a.x, a.y, a.z, w.x, w.y, w.z]).all()
                or msg.linear_acceleration_covariance[0] == -1
                or msg.angular_velocity_covariance[0] == -1):
            self.imu_dropped += 1
            self.imu_warning('Invalid IMU: need combined acceleration and gyro in a named frame')
            return
        if self.imu_frame is not None and msg.header.frame_id != self.imu_frame:
            self.imu_dropped += 1
            self.imu_warning('IMU frame changed; restart the node after changing sensor frames')
            return
        if timestamp <= max(self.last_imu_received_ns, self.last_tracker_timestamp_ns):
            self.imu_dropped += 1
            self.imu_warning('Dropping duplicate or late IMU timestamp; check sensor clock synchronization')
            return
        self.imu_frame = msg.header.frame_id
        self.last_imu_received_ns = timestamp
        if len(self.imu_samples) >= int(self.param('imu_queue_size')):
            self.imu_samples.popleft()
            self.imu_dropped += 1
            self.imu_warning('IMU buffer overflow; input is faster than processing')
        self.imu_samples.append(msg)
        if self.tracker is None and self.camera_info is not None:
            self.on_camera_info(self.camera_info)
        self.drain_frames()

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg
        if self.tracker is not None or (self.use_imu and self.imu_frame is None):
            return

        rig_frame = str(self.param('rig_frame'))
        camera_frame = msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                rig_frame, camera_frame, Time(), timeout=Duration(seconds=0.05))
            imu_transform = None
            if self.use_imu:
                imu_transform = self.tf_buffer.lookup_transform(
                    rig_frame, self.imu_frame, Time(), timeout=Duration(seconds=0.05))
        except TransformException as exc:
            now = time.monotonic()
            if now - self.last_init_warning > 2.0:
                self.get_logger().warning(
                    f'Waiting for camera/IMU transforms into {rig_frame}: {exc}')
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
        imus = []
        if self.use_imu:
            t, q = imu_transform.transform.translation, imu_transform.transform.rotation
            imus = [cuvslam.ImuCalibration(
                rig_from_imu=cuvslam.Pose(rotation=[q.x, q.y, q.z, q.w], translation=[t.x, t.y, t.z]),
                frequency=float(self.param('imu_frequency')),
                gyroscope_noise_density=float(self.param('gyroscope_noise_density')),
                gyroscope_random_walk=float(self.param('gyroscope_random_walk')),
                accelerometer_noise_density=float(self.param('accelerometer_noise_density')),
                accelerometer_random_walk=float(self.param('accelerometer_random_walk')))]
        self.tracker = cuvslam.Tracker(cuvslam.Rig(cameras=[camera], imus=imus), config)
        self.get_logger().info(
            f'CPU/RVV RGB-D ready: {msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.3f}, fy={msg.k[4]:.3f}, '
            f'rig={rig_frame}, camera={camera_frame}, IMU={self.imu_frame if self.use_imu else "disabled"}')
        if self.use_imu:
            self.get_logger().info('Keep the robot stationary for at least 1 second for IMU initialization')

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
        timestamp = self.timestamp_ns(color_msg)
        if timestamp <= self.last_image_timestamp_ns:
            self.image_dropped += 1
            return
        self.last_image_timestamp_ns = timestamp
        if not self.use_imu:
            self.process_rgbd(color_msg, depth_msg)
            return
        if len(self.pending_frames) >= int(self.param('sync_queue_size')):
            self.pending_frames.popleft()
            self.image_dropped += 1
            self.imu_warning('RGB-D buffer overflow while waiting for IMU')
        self.pending_frames.append((time.monotonic(), color_msg, depth_msg))
        self.drain_frames()

    def drain_frames(self) -> None:
        # rclpy.spin uses one thread; buffer IMU callbacks and serialize every
        # register_imu_measurement/track call here in sensor timestamp order.
        if self.tracker is None:
            return
        while self.pending_frames:
            received, color, depth = self.pending_frames[0]
            timestamp = self.timestamp_ns(color)
            if self.last_imu_received_ns < timestamp:
                if time.monotonic() - received < float(self.param('imu_wait_s')):
                    return
                self.pending_frames.popleft()
                self.image_dropped += 1
                self.tracking_publisher.publish(Bool(data=False))
                self.imu_warning('Dropping RGB-D frame: IMU timestamp has not caught up; check /camera/imu')
                continue
            self.pending_frames.popleft()
            self.process_rgbd(color, depth)

    def process_rgbd(self, color_msg: Image, depth_msg: Image) -> None:
        try:
            color = self.color_array(color_msg)
            depth = self.depth_array(depth_msg)
            timestamp_ns = (
                int(color_msg.header.stamp.sec) * 1_000_000_000
                + int(color_msg.header.stamp.nanosec)
            )
            start = time.perf_counter()
            if self.use_imu:
                while self.imu_samples and self.timestamp_ns(self.imu_samples[0]) <= timestamp_ns:
                    msg = self.imu_samples.popleft()
                    stamp = self.timestamp_ns(msg)
                    if stamp < self.last_tracker_timestamp_ns:
                        self.imu_dropped += 1
                        continue
                    a, w = msg.linear_acceleration, msg.angular_velocity
                    self.tracker.register_imu_measurement(0, cuvslam.ImuMeasurement(
                        timestamp_ns=stamp, linear_accelerations=[a.x, a.y, a.z],
                        angular_velocities=[w.x, w.y, w.z]))
                    self.last_tracker_timestamp_ns = stamp
                    self.imu_submitted += 1
            # Advance even when track rejects a frame: its internal time may have
            # advanced, so replaying older IMU into this tracker would be unsafe.
            self.last_tracker_timestamp_ns = timestamp_ns
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
        if self.use_imu and self.tracker is not None:
            ready = self.tracker.odometry.get_last_gravity() is not None
            if ready != self.imu_initialized:
                self.get_logger().info(f'IMU initialized={ready}')
            elif not ready:
                self.imu_warning('Waiting for stationary IMU initialization (at least 0.5 seconds)')
            self.imu_initialized = ready
        processing_fps = self.frame_count / max(self.processing_seconds, 1e-9)
        success_rate = 100.0 * self.tracked_count / self.frame_count
        self.get_logger().info(
            f'frames={self.frame_count}, tracked={self.tracked_count}, '
            f'failed={self.failed_count}, success={success_rate:.1f}%, '
            f'processing_fps={processing_fps:.1f}, imu_submitted={self.imu_submitted}, '
            f'imu_dropped={self.imu_dropped}, image_dropped={self.image_dropped}')


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
