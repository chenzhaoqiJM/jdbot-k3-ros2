#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class TFMonitor(Node):
    def __init__(self):
        super().__init__('tf_monitor')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # 10 Hz 定时器
        self.timer = self.create_timer(0.1, self.on_timer)
        self.last_xyz = None

    def on_timer(self):
        try:
            map_to_odom = self.buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time()
            )

            # 直接查 map → base_footprint 的 TF
            trans = self.buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            odom_x = map_to_odom.transform.translation.x
            odom_y = map_to_odom.transform.translation.y
            odom_z = map_to_odom.transform.translation.z

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            if self.last_xyz is None:
                self.last_xyz = (x, y, z)
                self.get_logger().info(f"Initial XYZ: {self.last_xyz}")
                return

            dx = x - self.last_xyz[0]
            dy = y - self.last_xyz[1]
            dz = z - self.last_xyz[2]

            self.last_xyz = (x, y, z)

            self.get_logger().info(
                f"map→odom: ({odom_x:.3f}, {odom_y:.3f}, {odom_z:.3f}) | "
                f"map→base_footprint XYZ: ({x:.3f}, {y:.3f}, {z:.3f}) | "
                f"ΔXYZ: ({dx:.3f}, {dy:.3f}, {dz:.3f})"
            )

        except Exception as e:
            # 每帧 TF 不一定都存在，忽略警告
            pass

def main():
    rclpy.init()
    node = TFMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
