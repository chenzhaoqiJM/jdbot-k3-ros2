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
        self.declare_parameter('publish_result_img', False)
        self.declare_parameter('linear_x', 0.4)
        self.declare_parameter('angular_z', 0.37)
        self.declare_parameter('sub_image_topic', '/image_raw')

        # self.video_device = self.get_parameter('video_device').get_parameter_value().string_value
        self.publish_result_img = self.get_parameter('publish_result_img').get_parameter_value().bool_value
        self.linear_x_set = self.get_parameter('linear_x').get_parameter_value().double_value
        self.angular_z_set = self.get_parameter('angular_z').get_parameter_value().double_value
        sub_image_topic = self.get_parameter('sub_image_topic').get_parameter_value().string_value

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
        last_detection_time = time.time()

        linear_x_set = follow_control.linear_x_set
        angular_z_set = follow_control.angular_z_set

        while rclpy.ok():
            
            with follow_control.lock:
                follow_me_flag = follow_control.ai_enabled

            # 更新框
            if not result_queue.empty():
                closest_box = copy.deepcopy(result_queue.get())
                last_detection_time = time.time()  # 更新检测时间
                # print(f"最靠近摄像头中心的检测框: {closest_box}")

            # 是否开启跟随
            if not follow_me_flag:
                if abs(cmd_x_z[0]-0.0) >= 0.01 or abs(cmd_x_z[1]-0.0) >= 0.01:
                    cmd_x_z = [0.0, 0.0]
                    for i in range(0, 3):
                        follow_control.publish_velocity(0.0, 0.0)  # 停止小车
                        time.sleep(0.01)
                
                time.sleep(0.02)
                continue

            # 是否超过3秒没有检测到目标
            time_since_last_detection = time.time() - last_detection_time
            if time_since_last_detection > 4.0:
                if abs(cmd_x_z[0]-0.0) >= 0.01 or abs(cmd_x_z[1]-0.0) >= 0.01:
                    cmd_x_z = [0.0, 0.0]
                    for i in range(0, 3):
                        follow_control.publish_velocity(0.0, 0.0)
                        time.sleep(0.01)
                time.sleep(0.02)
                continue
            
            # 始终跟踪上一次框
            cmd_x_z = [0.0, 0.0]
            if len(closest_box) == 4:
                x1, y1, x2, y2 = closest_box
                center_x = (x1 + x2) / 2
                # print(f"中心位置：{center_x}, 顶部位置：{y1}")

                # 速度决策
                allowable_deviation = 90.0
                diff = center_x - 320.0
                if diff <= -allowable_deviation:
                    angular_z = angular_z_set
                    linear_x = linear_x_set * 0.1
                elif diff >= allowable_deviation:
                    angular_z = -1.0 * angular_z_set
                    linear_x = linear_x_set * 0.1
                else:
                    angular_z = 0.0

                    # 向前移动决策
                    if y1 > 10:
                        linear_x = linear_x_set
                    else:
                        linear_x = 0.0

                # 更新速度
                cmd_x_z = [linear_x, angular_z]

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