#!/usr/bin/env python3

"""Broadcast the pose in an Odometry message as a TF transform."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """Convert ORB-SLAM3 odometry messages to TF transforms."""

    def __init__(self):
        super().__init__('orbslam3_odom_to_tf')

        self.declare_parameter('odom_topic', '/orbslam3/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint_orb')

        odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(
            f'Converting {odom_topic} to TF '
            f'({self.parent_frame} -> {self.child_frame})'
        )

    def odom_callback(self, msg: Odometry):
        """Broadcast one transform for an incoming odometry message."""
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
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
