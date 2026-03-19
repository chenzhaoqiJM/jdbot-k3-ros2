import cv2
import argparse
import numpy as np
import threading
import queue
import time
import copy
import os
import sys

# ros2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist


from std_msgs.msg import Int32
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool

from ament_index_python.packages import get_package_share_directory
try:
    package_share_directory = get_package_share_directory('jobot_mono_follow')
except:
    package_share_directory = ''
    print('NO INSTALL MODE')

current_file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(current_file_path)
sys.path.append(current_dir)
# ours
from jobot_mono_follow_cv import AGVDetection
from downloader import ModelDownloader
from cv_bridge import CvBridge

# 跟踪控制节点
class FollowControl(Node):
    def __init__(self):
        super().__init__('follow_control_service')
        self.srv = self.create_service(SetBool, 'toggle_follow', self.callback)
        self.ai_enabled = False  # 初始状态
        self.get_logger().info('AI 跟踪控制服务已启动, 当前状态为暂停')
        self.lock = threading.Lock() # 锁，用于线程数据同步

        # self.declare_parameter('video_device', '/dev/video20')
        # 是否发布带检测框的结果图像，便于调试观察
        self.declare_parameter('publish_result_img', False)
        # 小车前进线速度上限（m/s）
        self.declare_parameter('linear_x', 0.4)
        # 小车转向角速度上限（rad/s）
        self.declare_parameter('angular_z', 0.37)
        # 订阅的相机图像话题
        self.declare_parameter('sub_image_topic', '/image_raw')
        # 图像中心的 x 坐标，需按相机分辨率调整
        self.declare_parameter('image_center_x', 320.0)
        # 允许直行的中心偏差范围，越大越不容易触发转向
        self.declare_parameter('allowable_deviation', 52.0)
        # 转向死区，小于该偏差时不转向，避免抖动
        self.declare_parameter('angular_deadband', 22.0)
        # 软转向区，在该范围内会减弱转向指令，提升平滑性
        self.declare_parameter('angular_soft_zone', 135.0)
        # 角速度平滑系数，越大越平滑但响应越慢
        self.declare_parameter('angular_smooth_ratio', 0.82)
        # 转弯时保留的最小前进速度比例，避免急转时完全停住
        self.declare_parameter('min_curve_speed_ratio', 0.42)
        # 目标丢失超时时间，超过该时间后停车（秒）
        self.declare_parameter('target_lost_timeout', 4.0)
        # 目标框顶部接近图像顶端时的停车阈值，越小表示越靠近相机
        self.declare_parameter('stop_y1_threshold', 10)
        # 转向比例系数，数值越大转向越积极
        self.declare_parameter('steering_kp_scale', 1.0)
        # 转向微分系数相对比例，数值越大对变化更敏感
        self.declare_parameter('steering_kd_ratio', 14.0)
        # 角速度输出死区比例，过小角速度将被置零，避免轻微摆动
        self.declare_parameter('angular_cmd_deadband_ratio', 0.15)
        # 角速度最小输出死区，防止上面比例过小时死区太小
        self.declare_parameter('angular_cmd_deadband_min', 0.025)
        # 线速度最大变化率比例，越大加减速越快
        self.declare_parameter('max_linear_step_ratio', 0.9)
        # 角速度最大变化率比例，越大转向变化越快
        self.declare_parameter('max_angular_step_ratio', 1.1)

        # self.video_device = self.get_parameter('video_device').get_parameter_value().string_value
        self.publish_result_img = self.get_parameter('publish_result_img').get_parameter_value().bool_value
        self.linear_x_set = self.get_parameter('linear_x').get_parameter_value().double_value
        self.angular_z_set = self.get_parameter('angular_z').get_parameter_value().double_value
        sub_image_topic = self.get_parameter('sub_image_topic').get_parameter_value().string_value
        self.image_center_x = self.get_parameter('image_center_x').get_parameter_value().double_value
        self.allowable_deviation = self.get_parameter('allowable_deviation').get_parameter_value().double_value
        self.angular_deadband = self.get_parameter('angular_deadband').get_parameter_value().double_value
        self.angular_soft_zone = self.get_parameter('angular_soft_zone').get_parameter_value().double_value
        self.angular_smooth_ratio = self.get_parameter('angular_smooth_ratio').get_parameter_value().double_value
        self.min_curve_speed_ratio = self.get_parameter('min_curve_speed_ratio').get_parameter_value().double_value
        self.target_lost_timeout = self.get_parameter('target_lost_timeout').get_parameter_value().double_value
        self.stop_y1_threshold = self.get_parameter('stop_y1_threshold').get_parameter_value().integer_value
        self.steering_kp_scale = self.get_parameter('steering_kp_scale').get_parameter_value().double_value
        self.steering_kd_ratio = self.get_parameter('steering_kd_ratio').get_parameter_value().double_value
        self.angular_cmd_deadband_ratio = self.get_parameter('angular_cmd_deadband_ratio').get_parameter_value().double_value
        self.angular_cmd_deadband_min = self.get_parameter('angular_cmd_deadband_min').get_parameter_value().double_value
        self.max_linear_step_ratio = self.get_parameter('max_linear_step_ratio').get_parameter_value().double_value
        self.max_angular_step_ratio = self.get_parameter('max_angular_step_ratio').get_parameter_value().double_value

        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 30)

        self.publisher_img = self.create_publisher(CompressedImage, '/result_img_follow', 30)
        self.bridge = CvBridge()

        # 图像话题订阅
        sync_hz = 30
        self.subscription = self.create_subscription(
            Image,
            sub_image_topic,
            self.image_callback2,
            sync_hz)

        self.infer_queue = queue.Queue(maxsize=2) # 放置推理线程的图片
        self.img_queue = queue.Queue(maxsize=2)   # 放原始图片

    def callback(self, request, response):
        with self.lock:
            self.ai_enabled = request.data
        if self.ai_enabled:
            msg = 'AI 跟踪模块已开启'
        else:
            msg = 'AI 跟踪模块已关闭'

        self.get_logger().info(f'收到请求: data={request.data} -> {msg}')
        response.success = True
        response.message = msg
        return response

    def image_callback2(self, msg):
        try:
            # 0.007890462875366211 s
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.infer_queue.full():
                try:
                    _ = self.infer_queue.get_nowait()
                except queue.Empty:
                    pass

            if self.img_queue.full():
                try:
                    _ = self.img_queue.get_nowait()
                except queue.Empty:
                    pass

            self.infer_queue.put_nowait(cv_image)
            self.img_queue.put_nowait(cv_image)
        except queue.Full:
            self.get_logger().warn("推理队列已满，丢弃当前帧")

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.velocity_publisher.publish(msg)

    def publish_compressed_img(self, result):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        _, encoded_img = cv2.imencode('.jpg', result, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        msg.data = encoded_img.tobytes()
        self.publisher_img.publish(msg)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def approach_value(current, target, max_step):
    if current < target:
        return min(current + max_step, target)
    return max(current - max_step, target)

class MyDetectionThread(threading.Thread):
    def __init__(self, result_queue, model_path, label_path, follow_control:FollowControl, publish_result_img):
        threading.Thread.__init__(self)
        self.result_queue = result_queue
        self.detector = AGVDetection(model_path, label_path)
        self.publish_result_img = publish_result_img

        # self.cap.release()
        self.running = True
        self.follow_control = follow_control

    def run(self):
        while self.running:
            follow_me_flag = self.follow_control.ai_enabled
            if not follow_me_flag:
                time.sleep(0.03)
                continue

            try:
                frame = self.follow_control.infer_queue.get_nowait()
                ret = True
            except queue.Empty:
                # 队列为空时会走到这里
                frame = None
                ret = False
            # print(frame.shape)
            if ret:
                # t1 = time.time()
                detections = self.detector.infer(frame)
                # print(f"推理耗时: {time.time() - t1} s")
                if detections:
                    height, width, _ = frame.shape
                    center_x = width // 2
                    center_y = height // 2
                    min_distance = float('inf')
                    closest_box = None
                    for det in detections:
                        # print(det)
                        x1, y1, x2, y2 = det
                        center = ((x1 + x2) // 2, (y1 + y2) // 2)
                        distance = ((center[0] - center_x) ** 2 + (center[1] - center_y) ** 2) ** 0.5
                        if distance < min_distance:
                            min_distance = distance
                            closest_box = det

                    if closest_box:
                        self.result_queue.put(closest_box)

                        if self.publish_result_img:
                            x1, y1, x2, y2 = closest_box
                            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 5)

                if self.publish_result_img:
                    self.follow_control.publish_compressed_img(frame)
            else:
                time.sleep(0.005)
                break

    def stop(self):
        self.running = False
        # self.cap.release()

def run_executor(executor):
    executor.spin()  # 运行 ROS 2 事件循环（不会阻塞主线程）

def main():

    aa = ModelDownloader()

    rclpy.init()

    # 跟踪开关变量服务器
    follow_control = FollowControl()

    # ros2 多线程管理
    executor = MultiThreadedExecutor()
    executor.add_node(follow_control)

    executor_thread = threading.Thread(target=run_executor, args=(executor,), daemon=True)
    executor_thread.start()

    # 目标检测线程
    result_queue = queue.Queue(maxsize=2)
    model_path = os.path.expanduser('~/.brdk_models/jobot_mono_follow/yolov8n.q.onnx')
    label_path = os.path.join(package_share_directory, 'jobot_mono_follow_cv/data/label.txt')

    # 获取参数
    publish_result_img = follow_control.publish_result_img

    detection_thread = MyDetectionThread(result_queue, model_path, label_path, follow_control, publish_result_img)
    detection_thread.start()

    # 主线程，执行跟踪逻辑
    try:
        count = 0
        start_time = time.time()
        closest_box = []
        follow_me_flag = 0
        cmd_x_z = [0.0, 0.0]
        target_cmd_x_z = [0.0, 0.0]
        last_detection_time = time.time()
        last_control_time = time.time()
        last_error = 0.0

        linear_x_set = follow_control.linear_x_set
        angular_z_set = follow_control.angular_z_set

        image_center_x = follow_control.image_center_x
        allowable_deviation = follow_control.allowable_deviation
        angular_deadband = follow_control.angular_deadband
        angular_cmd_deadband = max(
            angular_z_set * follow_control.angular_cmd_deadband_ratio,
            follow_control.angular_cmd_deadband_min,
        )
        steering_kp = (
            angular_z_set / max(image_center_x, 1e-6) * follow_control.steering_kp_scale
            if angular_z_set > 0.0 else 0.0
        )
        steering_kd = steering_kp * follow_control.steering_kd_ratio
        angular_soft_zone = follow_control.angular_soft_zone
        angular_smooth_ratio = follow_control.angular_smooth_ratio
        min_curve_speed_ratio = follow_control.min_curve_speed_ratio
        max_linear_step = max(linear_x_set, 0.05) * follow_control.max_linear_step_ratio
        max_angular_step = max(angular_z_set, 0.05) * follow_control.max_angular_step_ratio

        while rclpy.ok():
            now = time.time()
            dt = max(now - last_control_time, 1e-3)
            last_control_time = now

            with follow_control.lock:
                follow_me_flag = follow_control.ai_enabled

            # 更新框
            if not result_queue.empty():
                closest_box = copy.deepcopy(result_queue.get())
                last_detection_time = time.time()  # 更新检测时间
                # print(f"最靠近摄像头中心的检测框: {closest_box}")

            # 是否开启跟随
            if not follow_me_flag:
                target_cmd_x_z = [0.0, 0.0]
                last_error = 0.0
                if abs(cmd_x_z[0]-0.0) >= 0.01 or abs(cmd_x_z[1]-0.0) >= 0.01:
                    cmd_x_z[0] = approach_value(cmd_x_z[0], target_cmd_x_z[0], max_linear_step * dt)
                    cmd_x_z[1] = approach_value(cmd_x_z[1], target_cmd_x_z[1], max_angular_step * dt)
                    for i in range(0, 3):
                        follow_control.publish_velocity(cmd_x_z[0], cmd_x_z[1])
                        time.sleep(0.01)

                time.sleep(0.02)
                continue

            # 是否超过3秒没有检测到目标
            time_since_last_detection = time.time() - last_detection_time
            if time_since_last_detection > follow_control.target_lost_timeout:
                target_cmd_x_z = [0.0, 0.0]
                last_error = 0.0
                if abs(cmd_x_z[0]-0.0) >= 0.01 or abs(cmd_x_z[1]-0.0) >= 0.01:
                    cmd_x_z[0] = approach_value(cmd_x_z[0], target_cmd_x_z[0], max_linear_step * dt)
                    cmd_x_z[1] = approach_value(cmd_x_z[1], target_cmd_x_z[1], max_angular_step * dt)
                    for i in range(0, 3):
                        follow_control.publish_velocity(cmd_x_z[0], cmd_x_z[1])
                        time.sleep(0.01)
                time.sleep(0.02)
                continue

            # 始终跟踪上一次框
            target_cmd_x_z = [0.0, 0.0]
            if len(closest_box) == 4:
                x1, y1, x2, y2 = closest_box
                center_x = (x1 + x2) / 2

                # print(f"中心位置：{center_x}, 顶部位置：{y1}")

                # 曲线跟随：持续前进 + 按偏差连续转向
                diff = center_x - image_center_x
                abs_diff = abs(diff)
                error_rate = (diff - last_error) / dt
                last_error = diff

                if abs_diff <= angular_deadband:
                    angular_z = 0.0
                else:
                    angular_cmd = -(diff * steering_kp + error_rate * steering_kd)
                    if abs_diff < angular_soft_zone:
                        angular_cmd *= abs_diff / angular_soft_zone
                    angular_z = clamp(angular_cmd, -angular_z_set, angular_z_set)
                    if abs(angular_z) < angular_cmd_deadband:
                        angular_z = 0.0

                angular_z = angular_smooth_ratio * cmd_x_z[1] + (1.0 - angular_smooth_ratio) * angular_z
                if abs(angular_z) < angular_cmd_deadband:
                    angular_z = 0.0

                turn_ratio = abs(angular_z) / max(angular_z_set, 1e-6)
                turn_ratio = clamp(turn_ratio, 0.0, 1.0)

                if y1 <= follow_control.stop_y1_threshold:
                    linear_x = 0.0
                else:
                    if abs_diff <= allowable_deviation:
                        curve_speed_ratio = 0.95
                    else:
                        curve_speed_ratio = 0.95 - 0.35 * turn_ratio
                        curve_speed_ratio = clamp(curve_speed_ratio, min_curve_speed_ratio, 1.0)

                    linear_x = linear_x_set * curve_speed_ratio

                # 更新速度
                target_cmd_x_z = [linear_x, angular_z]

            cmd_x_z[0] = approach_value(cmd_x_z[0], target_cmd_x_z[0], max_linear_step * dt)
            cmd_x_z[1] = approach_value(cmd_x_z[1], target_cmd_x_z[1], max_angular_step * dt)

            if abs(cmd_x_z[0]) < 1e-3:
                cmd_x_z[0] = 0.0
            if abs(cmd_x_z[1]) < 1e-3:
                cmd_x_z[1] = 0.0

            # 发布速度
            follow_control.publish_velocity(cmd_x_z[0], cmd_x_z[1])

            count += 1
            elapsed_time = time.time() - start_time  # 计算运行时间

            if elapsed_time >= 1.0:  # 每秒打印一次
                frequency = count / elapsed_time
                # print(f"循环频率: {frequency:.2f} 次/秒")
                print(f"cmd_vel -- linear_x:{cmd_x_z[0]}, angular_z:{cmd_x_z[1]}")
                count = 0  # 重置计数
                start_time = time.time()  # 重新计时

            time.sleep(0.003)

    except KeyboardInterrupt:
        print("停止检测线程...")
        detection_thread.stop()  # 先停止线程
        # detection_thread.join()  # 等待线程退出

        print("停止机器人...")
        follow_control.publish_velocity(0.0, 0.0)  # 停止小车
        time.sleep(0.2)

        # rclpy.shutdown()

    finally:
        follow_control.publish_velocity(0.0, 0.0)  # 停止小车
        time.sleep(0.2)
        rclpy.shutdown()



if __name__ == '__main__':
    main()