#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x, y, z, w):
    """Return the planar yaw angle represented by a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def calculate_planar_velocity(previous_pose, current_pose, dt):
    """Calculate the constant body-frame twist between two SE(2) poses."""
    previous_x, previous_y, previous_yaw = previous_pose
    current_x, current_y, current_yaw = current_pose
    delta_x = current_x - previous_x
    delta_y = current_y - previous_y
    delta_yaw = normalize_angle(current_yaw - previous_yaw)

    cos_yaw = math.cos(previous_yaw)
    sin_yaw = math.sin(previous_yaw)
    translation_x = cos_yaw * delta_x + sin_yaw * delta_y
    translation_y = -sin_yaw * delta_x + cos_yaw * delta_y

    if abs(delta_yaw) < 1e-6:
        coefficient_a = 1.0 - delta_yaw * delta_yaw / 6.0
        coefficient_b = delta_yaw / 2.0
    else:
        coefficient_a = math.sin(delta_yaw) / delta_yaw
        coefficient_b = (1.0 - math.cos(delta_yaw)) / delta_yaw

    denominator = (
        coefficient_a * coefficient_a + coefficient_b * coefficient_b)
    velocity_x = (
        coefficient_a * translation_x + coefficient_b * translation_y
    ) / denominator / dt
    velocity_y = (
        -coefficient_b * translation_x + coefficient_a * translation_y
    ) / denominator / dt
    return velocity_x, velocity_y, delta_yaw / dt


class TfToOdomNode(Node):
    """Publish a planar Odometry message from a TF transform."""

    _COVARIANCE_DIAGONAL_INDICES = (0, 7, 14, 21, 28, 35)

    def __init__(self):
        super().__init__('tf_to_odom_node')

        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('velocity_filter_alpha', 0.35)
        self.declare_parameter('max_sample_interval', 1.0)
        self.declare_parameter('max_linear_velocity', 3.0)
        self.declare_parameter('max_angular_velocity', 6.0)
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.0025, 0.0025, 1e6, 1e6, 1e6, 0.0012])
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.01, 0.01, 1e6, 1e6, 1e6, 0.0076])

        self.parent_frame = self.get_parameter(
            'parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter(
            'child_frame').get_parameter_value().string_value
        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.velocity_filter_alpha = self.get_parameter(
            'velocity_filter_alpha').value
        self.max_sample_interval = self.get_parameter(
            'max_sample_interval').value
        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity').value
        if not 0.0 < self.velocity_filter_alpha <= 1.0:
            raise ValueError('velocity_filter_alpha must be in (0, 1]')
        if (not math.isfinite(self.max_sample_interval) or
                self.max_sample_interval <= 0.0):
            raise ValueError('max_sample_interval must be greater than zero')
        if (not math.isfinite(self.max_linear_velocity) or
                self.max_linear_velocity <= 0.0):
            raise ValueError('max_linear_velocity must be greater than zero')
        if (not math.isfinite(self.max_angular_velocity) or
                self.max_angular_velocity <= 0.0):
            raise ValueError('max_angular_velocity must be greater than zero')

        self.pose_covariance = self._make_covariance(
            'pose_covariance_diagonal')
        self.twist_covariance = self._make_covariance(
            'twist_covariance_diagonal')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odom)

        self.previous_stamp_ns = None
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_yaw = 0.0
        self.filtered_velocity = None
        self.last_warning_ns = None

        self.get_logger().info(
            f'Publishing /odom from TF {self.parent_frame} -> '
            f'{self.child_frame}')

    def _make_covariance(self, parameter_name):
        diagonal = self.get_parameter(parameter_name).value
        if (len(diagonal) != 6 or
                any(not math.isfinite(value) or value < 0.0
                    for value in diagonal)):
            raise ValueError(
                f'{parameter_name} must contain 6 non-negative values')

        covariance = [0.0] * 36
        for index, value in zip(
                self._COVARIANCE_DIAGONAL_INDICES, diagonal):
            covariance[index] = float(value)
        return covariance

    def _set_previous_sample(self, stamp_ns, x, y, yaw, reset_filter=False):
        self.previous_stamp_ns = stamp_ns
        self.previous_x = x
        self.previous_y = y
        self.previous_yaw = yaw
        if reset_filter:
            self.filtered_velocity = None

    def _warn_throttled(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if (self.last_warning_ns is None or
                now_ns < self.last_warning_ns or
                now_ns - self.last_warning_ns >= 5_000_000_000):
            self.get_logger().warning(message)
            self.last_warning_ns = now_ns

    def publish_odom(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, Time())
        except TransformException as error:
            self._warn_throttled(
                f'Waiting for TF {self.parent_frame} -> '
                f'{self.child_frame}: {error}')
            return

        stamp_ns = (
            transform.header.stamp.sec * 1_000_000_000 +
            transform.header.stamp.nanosec)
        # Do not emit the same TF sample repeatedly at the timer frequency.
        if (self.previous_stamp_ns is not None and
                stamp_ns == self.previous_stamp_ns):
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)

        # The first sample only establishes the differentiation baseline.
        if self.previous_stamp_ns is None:
            self._set_previous_sample(
                stamp_ns, translation.x, translation.y, yaw,
                reset_filter=True)
            return

        dt = (stamp_ns - self.previous_stamp_ns) / 1_000_000_000.0
        if dt <= 0.0 or dt > self.max_sample_interval:
            self._warn_throttled(
                f'Resetting velocity estimate after invalid TF interval: '
                f'{dt:.3f} s')
            self._set_previous_sample(
                stamp_ns, translation.x, translation.y, yaw,
                reset_filter=True)
            return

        raw_velocity = calculate_planar_velocity(
            (self.previous_x, self.previous_y, self.previous_yaw),
            (translation.x, translation.y, yaw),
            dt,
        )

        linear_speed = math.hypot(raw_velocity[0], raw_velocity[1])
        if (not all(math.isfinite(value) for value in raw_velocity) or
                linear_speed > self.max_linear_velocity or
                abs(raw_velocity[2]) > self.max_angular_velocity):
            self._warn_throttled(
                'Rejecting implausible TF velocity sample: '
                f'linear={linear_speed:.3f} m/s, '
                f'angular={raw_velocity[2]:.3f} rad/s')
            self._set_previous_sample(
                stamp_ns, translation.x, translation.y, yaw,
                reset_filter=True)
            return

        if self.filtered_velocity is None:
            self.filtered_velocity = raw_velocity
        else:
            alpha = self.velocity_filter_alpha
            self.filtered_velocity = tuple(
                alpha * current + (1.0 - alpha) * previous
                for current, previous in zip(
                    raw_velocity, self.filtered_velocity))

        odom = Odometry()
        odom.header = transform.header
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = translation.x
        odom.pose.pose.position.y = translation.y
        odom.pose.pose.position.z = translation.z
        odom.pose.pose.orientation = rotation
        odom.pose.covariance = self.pose_covariance
        odom.twist.twist.linear.x = self.filtered_velocity[0]
        odom.twist.twist.linear.y = self.filtered_velocity[1]
        odom.twist.twist.angular.z = self.filtered_velocity[2]
        odom.twist.covariance = self.twist_covariance

        self.odom_publisher.publish(odom)
        self._set_previous_sample(stamp_ns, translation.x, translation.y, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
