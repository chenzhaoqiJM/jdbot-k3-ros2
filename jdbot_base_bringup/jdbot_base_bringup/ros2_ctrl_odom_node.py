#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import math
import serial
import threading

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# ================== 底盘默认参数 ==================
DEFAULT_WHEEL_DIAMETER = 0.067    # m
DEFAULT_WHEEL_BASE = 0.183        # m


class CmdVelToSerial(Node):

    def __init__(self):
        super().__init__('cmdvel_to_serial')

        # ---------------- 参数 ----------------
        self.declare_parameter('serial_port', '/dev/jdbot')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.4)
        self.declare_parameter('wheel_diameter', DEFAULT_WHEEL_DIAMETER)
        self.declare_parameter('wheel_base', DEFAULT_WHEEL_BASE)

        # ===== odom 参数（新增）=====
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.send_hz = self.get_parameter('send_hz').value
        self.timeout = self.get_parameter('cmd_vel_timeout').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value

        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ---------------- 串口 ----------------
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f"Serial opened: {port} @ {baud}")

        # ---------------- 状态 ----------------
        self.last_cmd_time = time.time()
        self.cur_v = 0.0
        self.cur_w = 0.0

        self.lock = threading.Lock()
        self.running = True

        # ===== odom 状态（新增）=====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = time.monotonic()

        # 轮速缓存（来自串口）
        self.v_l = 0.0
        self.v_r = 0.0

        # ---------------- ROS ----------------
        self.create_subscription(Twist, 'cmd_vel', self.cmdvel_cb, 10)

        self.odom_pub = self.create_publisher(
            Odometry, self.odom_topic, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            1.0 / self.send_hz,
            self.send_timer_cb
        )

        # ===== odom 定时器（新增）=====
        self.odom_timer = self.create_timer(
            1.0 / 50.0,
            self.odom_timer_cb
        )

        # ---------------- 串口读取线程 ----------------
        self.read_thread = threading.Thread(
            target=self.serial_read_thread,
            daemon=True
        )
        self.read_thread.start()

        self.get_logger().info("cmd_vel → serial + odom node started")

    # =====================================================
    # cmd_vel 回调
    # =====================================================
    def cmdvel_cb(self, msg: Twist):
        with self.lock:
            self.cur_v = msg.linear.x
            self.cur_w = msg.angular.z
            self.last_cmd_time = time.time()

    # =====================================================
    # 定时器：发送
    # =====================================================
    def send_timer_cb(self):
        now = time.time()

        with self.lock:
            if now - self.last_cmd_time > self.timeout:
                v = 0.0
                w = 0.0
            else:
                v = self.cur_v
                w = self.cur_w

        self.send_cmd(v, w)

    # =====================================================
    # 串口发送
    # =====================================================
    def send_cmd(self, v, w):
        v_l = v - w * self.wheel_base / 2.0
        v_r = v + w * self.wheel_base / 2.0

        dir_l, spd_l = self.wheel_speed_to_cmd(v_l)
        dir_r, spd_r = self.wheel_speed_to_cmd(v_r)

        cmd = f"{dir_l},{spd_l:.2f};{dir_r},{spd_r:.2f}\n"
        self.ser.write(cmd.encode())

    def wheel_speed_to_cmd(self, v):
        if abs(v) < 1e-3:
            return 0, 0.0

        direction = 1 if v > 0 else 2
        speed = abs(v) / (math.pi * self.wheel_diameter)
        return direction, speed

    # =====================================================
    # 串口读取线程（解析轮速 → odom）
    # =====================================================
    def serial_read_thread(self):
        buf = b""
        self.get_logger().info("Serial RX thread started")

        while rclpy.ok() and self.running:
            try:
                data = self.ser.read(64)
                if not data:
                    continue

                buf += data

                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode('utf-8', errors='ignore').strip()
                    if text:
                        self.parse_feedback(text)

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

    # =====================================================
    # 串口反馈解析（新增）
    # =====================================================
    def parse_feedback(self, text: str):
        try:
            text_split_text = text.split(';')
            left, right = text_split_text[-2], text_split_text[-1]

            v_l = self.parse_motor(left)
            v_r = self.parse_motor(right)

            with self.lock:
                self.v_l = v_l
                self.v_r = v_r

        except Exception as e:
            self.get_logger().warn(f"Parse error: {text}")

    def parse_motor(self, s: str) -> float:
        items = s.split(',')
        speed = float(items[-3])           # 转/s
        direction = int(items[-2])          # 0 / 1 / 2

        if direction == 2:
            speed = -speed
        elif direction == 0:
            speed = 0.0

        # 转/s → m/s
        return speed * math.pi * self.wheel_diameter

    # =====================================================
    # odom 定时发布（新增）
    # =====================================================
    def odom_timer_cb(self):
        now = time.monotonic()
        dt = now - self.last_odom_time
        self.last_odom_time = now
        if dt <= 0.0:
            return

        with self.lock:
            v_l = self.v_l
            v_r = self.v_r

        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.wheel_base

        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        self.publish_odom(v, w)

    def publish_odom(self, v, w):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # =====================================================
    def destroy_node(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
