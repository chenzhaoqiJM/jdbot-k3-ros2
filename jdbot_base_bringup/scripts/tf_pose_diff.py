#!/usr/bin/env python3
"""Print planar pose differences between two TF frames in a common parent."""

import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(rotation):
    """Return yaw in radians from a geometry_msgs Quaternion."""
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


class TFPoseDiff(Node):
    def __init__(self):
        super().__init__('tf_pose_diff')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('first_frame', 'base_footprint')
        self.declare_parameter('second_frame', 'base_footprint_rgbd')
        self.declare_parameter('rate_hz', 10.0)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        rate_hz = self.get_parameter('rate_hz').value
        if rate_hz <= 0:
            raise ValueError('rate_hz must be greater than zero')
        self.timer = self.create_timer(1.0 / rate_hz, self.print_diff)
        self.missing_tf = False

    def print_diff(self):
        parent = self.get_parameter('parent_frame').value
        first = self.get_parameter('first_frame').value
        second = self.get_parameter('second_frame').value
        try:
            first_tf = self.buffer.lookup_transform(
                parent, first, rclpy.time.Time()
            )
            second_tf = self.buffer.lookup_transform(
                parent, second, rclpy.time.Time()
            )
        except TransformException as exc:
            if not self.missing_tf:
                self.get_logger().warn(f'Waiting for TF: {exc}')
                self.missing_tf = True
            return

        self.missing_tf = False
        dx = (second_tf.transform.translation.x
              - first_tf.transform.translation.x)
        dy = (second_tf.transform.translation.y
              - first_tf.transform.translation.y)
        yaw_diff = (
            yaw_from_quaternion(second_tf.transform.rotation)
            - yaw_from_quaternion(first_tf.transform.rotation)
        )
        dyaw_deg = math.degrees(math.atan2(math.sin(yaw_diff),
                                           math.cos(yaw_diff)))
        self.get_logger().info(
            f'{parent}: {second} - {first} | '
            f'dx={dx:+.3f} m, dy={dy:+.3f} m, dyaw={dyaw_deg:+.2f} deg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFPoseDiff()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
