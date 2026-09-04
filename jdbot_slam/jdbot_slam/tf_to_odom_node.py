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


class TfToOdomNode(Node):
    """Publish a planar Odometry message from a TF transform."""

    def __init__(self):
        super().__init__('tf_to_odom_node')

        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)

        self.parent_frame = self.get_parameter(
            'parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter(
            'child_frame').get_parameter_value().string_value
        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odom)

        self.previous_stamp_ns = None
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_yaw = 0.0
        self.last_warning_ns = None

        self.get_logger().info(
            f'Publishing /odom from TF {self.parent_frame} -> '
            f'{self.child_frame}')

    def publish_odom(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, Time())
        except TransformException as error:
            now_ns = self.get_clock().now().nanoseconds
            if (self.last_warning_ns is None or
                    now_ns - self.last_warning_ns >= 5_000_000_000):
                self.get_logger().warning(
                    f'Waiting for TF {self.parent_frame} -> '
                    f'{self.child_frame}: {error}')
                self.last_warning_ns = now_ns
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

        odom = Odometry()
        odom.header = transform.header
        odom.header.frame_id = self.parent_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = translation.x
        odom.pose.pose.position.y = translation.y
        odom.pose.pose.position.z = translation.z
        odom.pose.pose.orientation = rotation

        if self.previous_stamp_ns is not None:
            dt = (stamp_ns - self.previous_stamp_ns) / 1_000_000_000.0
            if dt > 0.0:
                velocity_x_parent = (translation.x - self.previous_x) / dt
                velocity_y_parent = (translation.y - self.previous_y) / dt

                # nav_msgs/Odometry defines twist in child_frame_id.
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                odom.twist.twist.linear.x = (
                    cos_yaw * velocity_x_parent +
                    sin_yaw * velocity_y_parent)
                odom.twist.twist.linear.y = (
                    -sin_yaw * velocity_x_parent +
                    cos_yaw * velocity_y_parent)
                odom.twist.twist.linear.z = 0.0
                odom.twist.twist.angular.z = normalize_angle(
                    yaw - self.previous_yaw) / dt

        self.odom_publisher.publish(odom)
        self.previous_stamp_ns = stamp_ns
        self.previous_x = translation.x
        self.previous_y = translation.y
        self.previous_yaw = yaw


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
