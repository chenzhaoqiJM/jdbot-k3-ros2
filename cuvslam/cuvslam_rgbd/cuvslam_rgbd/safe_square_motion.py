#!/usr/bin/env python3
"""Publish a conservative square motion with a guaranteed final stop."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SafeSquareMotion(Node):
    def __init__(self) -> None:
        super().__init__('cuvslam_safe_square_motion')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('angular_speed', 0.60)
        self.declare_parameter('edge_length', 0.30)
        self.declare_parameter('start_delay_s', 5.0)
        self.declare_parameter('pause_s', 0.4)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)

    def publish_for(self, linear_x: float, angular_z: float, duration: float) -> None:
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        self.stop()

    def stop(self) -> None:
        command = Twist()
        for _ in range(8):
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def run(self) -> bool:
        if not bool(self.get_parameter('auto_start').value):
            self.get_logger().error(
                'Motion is disabled. Pass --ros-args -p auto_start:=true after checking the area.')
            self.stop()
            return False

        linear_speed = min(abs(float(self.get_parameter('linear_speed').value)), 0.25)
        angular_speed = min(abs(float(self.get_parameter('angular_speed').value)), 0.60)
        edge_length = min(abs(float(self.get_parameter('edge_length').value)), 0.35)
        start_delay = max(float(self.get_parameter('start_delay_s').value), 2.0)
        pause = max(float(self.get_parameter('pause_s').value), 0.2)
        if linear_speed < 0.05 or angular_speed < 0.4 or edge_length < 0.05:
            raise ValueError('Unsafe or ineffective square-motion parameters')

        deadline = time.monotonic() + start_delay
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.publisher.get_subscription_count() == 0:
            self.get_logger().error('No /cmd_vel subscriber; refusing to move')
            self.stop()
            return False

        remaining_delay = max(0.0, deadline - time.monotonic())
        self.get_logger().warning(
            f'Square motion starts in {remaining_delay:.1f} s: edge={edge_length:.2f} m, '
            f'linear={linear_speed:.2f} m/s, angular={angular_speed:.2f} rad/s')
        while rclpy.ok() and time.monotonic() < deadline:
            self.stop()

        forward_duration = edge_length / linear_speed
        turn_duration = (math.pi / 2.0) / angular_speed
        for side in range(4):
            self.get_logger().info(f'Side {side + 1}/4: forward')
            self.publish_for(linear_speed, 0.0, forward_duration)
            time.sleep(pause)
            self.get_logger().info(f'Side {side + 1}/4: turn left')
            self.publish_for(0.0, angular_speed, turn_duration)
            time.sleep(pause)
        self.stop()
        self.get_logger().info('Square motion complete; zero velocity published')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeSquareMotion()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted; stopping the base')
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
