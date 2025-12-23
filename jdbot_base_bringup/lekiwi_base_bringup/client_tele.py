#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simplified LeKiwi Teleoperation Client
--------------------------------------
This version only uses ZMQ to control the base speed (x, y, theta)
via keyboard, while keeping arm joint positions constant.

Author: chenzhaoqi (based on HuggingFace LeKiwiClient)
"""

import json
import time
import zmq
import logging
import sys
import termios
import tty
import select


class SimpleLeKiwiClient:
    def __init__(self, remote_ip="10.0.91.5", port_zmq_cmd=5555, port_zmq_observations=5556):
        self.remote_ip = remote_ip
        self.port_zmq_cmd = port_zmq_cmd
        self.port_zmq_observations = port_zmq_observations

        self._is_connected = False
        self.last_remote_state = {}

        # 键盘控制定义
        self.teleop_keys = {
            "forward": "w",
            "backward": "s",
            "left": "a",
            "right": "d",
            "rotate_left": "q",
            "rotate_right": "e",
            "speed_up": "z",
            "speed_down": "x",
            "quit": " "
        }

        # 三档速度
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},
            {"xy": 0.2, "theta": 60},
            {"xy": 0.3, "theta": 90},
        ]
        self.speed_index = 0

    def connect(self):
        ctx = zmq.Context()
        self.ctx = ctx
        self.zmq_cmd_socket = ctx.socket(zmq.PUSH)
        self.zmq_observation_socket = ctx.socket(zmq.PULL)

        self.zmq_cmd_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_cmd}")
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)

        self.zmq_observation_socket.connect(f"tcp://{self.remote_ip}:{self.port_zmq_observations}")
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)

        self._is_connected = True
        print(f"[INFO] Connected to host {self.remote_ip}")

    def disconnect(self):
        self.zmq_cmd_socket.close()
        self.zmq_observation_socket.close()
        self.ctx.term()
        self._is_connected = False
        print("[INFO] Disconnected")

    def _decode_obs(self, obs_str):
        """Parse JSON observation & store arm positions"""
        try:
            data = json.loads(obs_str)
            # 仅记录机械臂状态，不控制
            self.last_remote_state = {
                k: v for k, v in data.items() if k.endswith(".pos")
            }
            return data
        except Exception as e:
            logging.warning(f"Failed to decode obs: {e}")
            return {}

    def get_observation(self):
        """Get last observation from ZMQ"""
        try:
            msg = self.zmq_observation_socket.recv_string(flags=zmq.NOBLOCK)
            return self._decode_obs(msg)
        except zmq.Again:
            return None

    def _get_key(self):
        """非阻塞键盘读取"""
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    def _keyboard_to_action(self, pressed_key):
        """Translate key input to base velocity command"""
        if not pressed_key:
            return {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

        # 调速
        if pressed_key == self.teleop_keys["speed_up"]:
            self.speed_index = min(self.speed_index + 1, 2)
            print(f"[Speed] ↑ to level {self.speed_index}")
        elif pressed_key == self.teleop_keys["speed_down"]:
            self.speed_index = max(self.speed_index - 1, 0)
            print(f"[Speed] ↓ to level {self.speed_index}")

        speed = self.speed_levels[self.speed_index]
        xy = speed["xy"]
        theta = speed["theta"]

        x = y = th = 0.0
        if pressed_key == self.teleop_keys["forward"]:
            x = xy
        elif pressed_key == self.teleop_keys["backward"]:
            x = -xy
        elif pressed_key == self.teleop_keys["left"]:
            y = xy
        elif pressed_key == self.teleop_keys["right"]:
            y = -xy
        elif pressed_key == self.teleop_keys["rotate_left"]:
            th = theta
        elif pressed_key == self.teleop_keys["rotate_right"]:
            th = -theta

        return {"x.vel": x, "y.vel": y, "theta.vel": th}

    def teleop_loop(self, freq_hz=10):
        """Main teleoperation loop"""
        if not self._is_connected:
            raise RuntimeError("Client not connected")

        print("[INFO] Starting teleop control (press SPACE to quit)")
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            dt = 1.0 / freq_hz
            while True:
                key = self._get_key()
                if key == self.teleop_keys["quit"]:
                    print("[INFO] Quit teleop.")
                    break

                obs = self.get_observation()
                if obs:
                    sys.stdout.write(f"\r[OBS] keys: {len(obs.keys())}    ")
                    sys.stdout.flush()
                    # print(obs.keys())
                    print(f"当前速度: x:{obs['x.vel']}, y:{obs['y.vel']}, theta:{obs['theta.vel']}")

                action = self._keyboard_to_action(key)

                # 机械臂状态保持
                action.update(self.last_remote_state)

                # 发送动作
                self.zmq_cmd_socket.send_string(json.dumps(action))
                time.sleep(dt)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.disconnect()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Simple LeKiwi Teleop Client using ZMQ only")
    parser.add_argument("--ip", type=str, default="10.0.91.5", help="Host IP of LeKiwiServer")
    parser.add_argument("--cmd-port", type=int, default=5555)
    parser.add_argument("--obs-port", type=int, default=5556)
    args = parser.parse_args()

    client = SimpleLeKiwiClient(remote_ip=args.ip,
                                port_zmq_cmd=args.cmd_port,
                                port_zmq_observations=args.obs_port)
    client.connect()
    client.teleop_loop(freq_hz=10)
