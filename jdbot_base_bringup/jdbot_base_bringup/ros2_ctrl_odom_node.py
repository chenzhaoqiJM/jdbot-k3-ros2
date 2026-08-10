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
        self.declare_parameter('odom_hz', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.4)
        self.declare_parameter('wheel_diameter', DEFAULT_WHEEL_DIAMETER)
        self.declare_parameter('wheel_base', DEFAULT_WHEEL_BASE)
        self.declare_parameter('motor1_factor', 1.0)
        self.declare_parameter('motor2_factor', 1.0)
        self.declare_parameter('feedback_pwm_deadzone', 0)
        self.declare_parameter('encoder_ppr', 1000.0)
        self.declare_parameter('reduction_ratio', 56.0)
        self.declare_parameter('ff_factor', 310.0)
        self.declare_parameter('pid_kp', 10.0)
        self.declare_parameter('pid_ki', 70.0)
        self.declare_parameter('pid_kd', 0.0)
        self.declare_parameter('straight_balance_kp', 25.0)
        self.declare_parameter('straight_balance_ki', 8.0)
        self.declare_parameter('straight_balance_kd', 0.0)
        self.declare_parameter('cfg_send_on_startup', True)
        self.declare_parameter('debug', False)

        # ===== odom 参数（新增）=====
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')


        self.motor1_factor = self.get_parameter('motor1_factor').value
        self.motor2_factor = self.get_parameter('motor2_factor').value
        self.feedback_pwm_deadzone = self.get_parameter('feedback_pwm_deadzone').value
        self.encoder_ppr = float(self.get_parameter('encoder_ppr').value)
        self.reduction_ratio = float(self.get_parameter('reduction_ratio').value)
        self.ff_factor = float(self.get_parameter('ff_factor').value)
        self.pid_kp = float(self.get_parameter('pid_kp').value)
        self.pid_ki = float(self.get_parameter('pid_ki').value)
        self.pid_kd = float(self.get_parameter('pid_kd').value)
        self.straight_balance_kp = float(self.get_parameter('straight_balance_kp').value)
        self.straight_balance_ki = float(self.get_parameter('straight_balance_ki').value)
        self.straight_balance_kd = float(self.get_parameter('straight_balance_kd').value)
        self.debug = self.get_parameter('debug').value

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.send_hz = self.get_parameter('send_hz').value
        self.odom_hz = self.get_parameter('odom_hz').value
        self.timeout = self.get_parameter('cmd_vel_timeout').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.cfg_send_on_startup = self.get_parameter('cfg_send_on_startup').value

        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # ---------------- 串口 ----------------
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f"Serial opened: {port} @ {baud}")
        self.send_cfg()

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
        self.power_voltage = 0.0

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
            1.0 / self.odom_hz,
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
    # 下发参数配置
    # =====================================================
    def send_cfg(self):
        if not self.cfg_send_on_startup:
            return

        cfg = (
            f"CFG,{self.encoder_ppr:.3f},{self.reduction_ratio:.3f},"
            f"{self.ff_factor:.3f},{self.pid_kp:.3f},{self.pid_ki:.3f},{self.pid_kd:.3f},"
            f"{self.straight_balance_kp:.3f},{self.straight_balance_ki:.3f},{self.straight_balance_kd:.3f}\n"
        )
        self.ser.write(cfg.encode())
        self.get_logger().info(f"Sent CFG: {cfg.strip()}")

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

        spd_l = spd_l * self.motor1_factor
        spd_r = spd_r * self.motor2_factor

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
                    if not text:
                        continue

                    if text.startswith('CFG_OK'):
                        self.get_logger().info(f"CFG ACK: {text}")
                        continue

                    if text.startswith('CFG_ERR'):
                        self.get_logger().error(f"CFG error: {text}")
                        continue

                    self.parse_feedback(text)

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

    # =====================================================
    # 串口反馈解析（新增）
    # =====================================================
    def parse_feedback(self, text: str):
        try:
            power, left, right = text.split(';')

            v_l = self.parse_motor(left)
            v_r = self.parse_motor(right)
            power_voltage = float(power)

            # left: "pluse_counts,spd(转/s),方向,pwm值"
            if self.debug:
                items = left.split(',')
                pwm = float(items[-1])
                speed = float(items[-3])           # 转/s
                _dynamic_left_ff_factor = pwm / speed if abs(speed) > 1e-3 else 0.0

                items_r = right.split(',')
                pwm_r = float(items_r[-1])
                speed_r = float(items_r[-3])           # 转/s
                _dynamic_right_ff_factor = pwm_r / speed_r if abs(speed_r) > 1e-3 else 0.0
                self.get_logger().info(f"left:{left}, right:{right},速度差:{(speed_r-speed):.2f}, 左轮ff_factor:{_dynamic_left_ff_factor:.2f}, 右轮ff_factor:{_dynamic_right_ff_factor:.2f}")

            with self.lock:
                self.v_l = v_l
                self.v_r = v_r
                self.power_voltage = power_voltage

        except Exception as e:
            self.get_logger().warn(f"Parse error: {text}: {e}")

    def parse_motor(self, s: str) -> float:
        items = s.split(',')
        pwm = int(items[-1])
        speed = float(items[-3])           # 转/s
        direction = int(items[-2])          # 0 / 1 / 2

        if abs(pwm) < self.feedback_pwm_deadzone:
            return 0.0

        if direction == 1:
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
