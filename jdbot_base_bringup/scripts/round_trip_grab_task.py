#!/usr/bin/env python3

import math
import sys
import time
from typing import Optional

import rclpy
import zmq
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation = quaternion_from_yaw(yaw)
    return pose


class RoundTripGrabTask:
    def __init__(
        self,
        point_a_x: float,
        point_a_y: float,
        point_a_yaw: float,
        point_b_x: float,
        point_b_y: float,
        point_b_yaw: float,
        zmq_bind: str,
        wait_interval_sec: float = 0.1,
    ) -> None:
        self.navigator = BasicNavigator()
        self.point_a = make_pose(self.navigator, point_a_x, point_a_y, point_a_yaw)
        self.point_b = make_pose(self.navigator, point_b_x, point_b_y, point_b_yaw)
        self.wait_interval_sec = wait_interval_sec

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PAIR)
        self.socket.bind(zmq_bind)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.navigator.get_logger().info(f'ZMQ PAIR 已绑定到 {zmq_bind}')

    def navigate_to_pose(self, target: PoseStamped, label: str) -> bool:
        self.navigator.goToPose(target)
        self.navigator.get_logger().info(f'开始导航到 {label}')

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback is not None:
                self.navigator.get_logger().debug(f'导航中: {label}')
            rclpy.spin_once(self.navigator, timeout_sec=self.wait_interval_sec)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.navigator.get_logger().info(f'已到达 {label}')
            return True

        if result == TaskResult.CANCELED:
            self.navigator.get_logger().warning(f'前往 {label} 的任务已取消')
            return False

        self.navigator.get_logger().error(f'前往 {label} 失败')
        return False

    def send_message(self, message: str) -> None:
        self.socket.send_string(message)
        self.navigator.get_logger().info(f'已发送 ZMQ 消息: {message}')

    def wait_for_message(self, expected_message: str, timeout_sec: Optional[float] = None) -> bool:
        self.navigator.get_logger().info(f'等待 ZMQ 消息: {expected_message}')
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec

        while rclpy.ok():
            if deadline is not None and time.monotonic() > deadline:
                self.navigator.get_logger().error(f'等待消息超时: {expected_message}')
                return False

            events = dict(self.poller.poll(timeout=int(self.wait_interval_sec * 1000)))
            if self.socket in events and events[self.socket] == zmq.POLLIN:
                message = self.socket.recv_string().strip()
                self.navigator.get_logger().info(f'收到 ZMQ 消息: {message}')
                if message == expected_message:
                    return True
                self.navigator.get_logger().warning(
                    f'收到非预期消息，期望 {expected_message}，实际 {message}'
                )

            rclpy.spin_once(self.navigator, timeout_sec=self.wait_interval_sec)

        return False

    def run(self) -> int:
        self.navigator.waitUntilNav2Active()
        self.navigator.get_logger().info('Nav2 已激活，开始执行 A->B->A 往返抓取任务')

        if not self.navigate_to_pose(self.point_b, 'B 点'):
            return 1

        self.send_message('start grab')
        if not self.wait_for_message('finish grab'):
            return 2

        if not self.navigate_to_pose(self.point_a, 'A 点'):
            return 3

        self.send_message('Release the claws')
        self.navigator.get_logger().info('往返抓取任务完成')
        return 0

    def shutdown(self) -> None:
        self.poller.unregister(self.socket)
        self.socket.close()
        self.context.term()
        self.navigator.destroyNode()


def main() -> int:
    rclpy.init()
    task = RoundTripGrabTask(
        point_a_x=0.0,
        point_a_y=0.0,
        point_a_yaw=0.0,
        point_b_x=1.0,
        point_b_y=0.0,
        point_b_yaw=0.0,
        zmq_bind='tcp://*:5777',
    )

    try:
        return task.run()
    except KeyboardInterrupt:
        task.navigator.cancelTask()
        task.navigator.get_logger().warning('收到中断，任务已取消')
        return 130
    finally:
        task.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())