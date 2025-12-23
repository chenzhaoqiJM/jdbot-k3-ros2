#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeKiwi Teleoperation Client via ROS2 cmd_vel
--------------------------------------------
Receives ROS2 cmd_vel messages and converts to ZMQ base command
while keeping arm joint positions constant.
"""

import json
import zmq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import logging


class CmdVelLeKiwiClient(Node):
    def __init__(self):
        super().__init__('cmdvel_teleop_client')


        self.declare_parameter('remote_ip', '10.0.91.5')

        self.remote_ip = self.get_parameter('remote_ip').get_parameter_value().string_value

        self.port_zmq_cmd = 5555
        self.port_zmq_observations = 5556

        self._is_connected = False
        self.last_remote_state = {}

        # ZMQ setup
        self.connect_zmq()

        # ROS2 cmd_vel subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused warning

    def connect_zmq(self):
        ctx = zmq.Context()
        self.ctx = ctx
        self.zmq_cmd_socket = ctx.socket(zmq.PUSH)
        self.zmq_observation_socket = ctx.socket(zmq.PULL)

        self.zmq_cmd_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_cmd}")
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)

        self.zmq_observation_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_observations}")
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)

        self._is_connected = True
        self.get_logger().info(f"[INFO] Connected to host {self.remote_ip}")

    def disconnect_zmq(self):
        self.zmq_cmd_socket.close()
        self.zmq_observation_socket.close()
        self.ctx.term()
        self._is_connected = False
        self.get_logger().info("[INFO] Disconnected ZMQ")

    def _decode_obs(self, obs_str):
        """Parse JSON observation & store arm positions"""
        try:
            data = json.loads(obs_str)
            self.last_remote_state = {k: v for k, v in data.items() if k.endswith(".pos")}
            return data
        except Exception as e:
            logging.warning(f"Failed to decode obs: {e}")
            return {}

    def get_observation(self):
        """Get last observation from ZMQ (non-blocking)"""
        try:
            msg = self.zmq_observation_socket.recv_string(flags=zmq.NOBLOCK)
            return self._decode_obs(msg)
        except zmq.Again:
            return None

    def cmd_vel_callback(self, msg: Twist):
        """Convert ROS2 cmd_vel to ZMQ base command"""
        # 更新最新机械臂状态
        obs = self.get_observation()  # 非阻塞
        if obs:
            self.last_remote_state.update(obs)

        # 生成动作
        action = {
            "x.vel": msg.linear.x,
            "y.vel": msg.linear.y,
            "theta.vel": msg.angular.z
        }

        # 保持机械臂状态
        action.update(self.last_remote_state)

        # 发送到 ZMQ
        if self._is_connected:
            self.zmq_cmd_socket.send_string(json.dumps(action))

        # 打印调试信息
        self.get_logger().info(f"[CMD_VEL] {action}")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CmdVelLeKiwiClient()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User interruption, exit program...")
        node.disconnect_zmq()
        node.destroy_node()
    finally:
        node.disconnect_zmq()

if __name__ == "__main__":
    main()

