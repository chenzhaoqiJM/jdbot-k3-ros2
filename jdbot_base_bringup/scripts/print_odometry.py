#!/usr/bin/env python3
"""Print /Odometry position in metres and roll/pitch/yaw in degrees."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def quaternion_to_rpy(q):
    """Convert a normalized quaternion to roll, pitch, yaw in radians."""
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class OdometryPrinter(Node):
    def __init__(self):
        super().__init__('print_odometry')
        self.declare_parameter('topic', '/Odometry')
        topic = self.get_parameter('topic').value
        self.subscription = self.create_subscription(
            Odometry, topic, self.on_odometry, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Subscribing to {topic}: xyz in metres, roll/pitch/yaw in degrees'
        )

    def on_odometry(self, msg):
        position = msg.pose.pose.position
        try:
            roll, pitch, yaw = map(math.degrees, quaternion_to_rpy(msg.pose.pose.orientation))
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self.get_logger().info(
            f'[{msg.header.frame_id} -> {msg.child_frame_id}] '
            f'xyz(m): x={position.x:.4f} y={position.y:.4f} z={position.z:.4f} | '
            f'rpy(deg): roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPrinter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
