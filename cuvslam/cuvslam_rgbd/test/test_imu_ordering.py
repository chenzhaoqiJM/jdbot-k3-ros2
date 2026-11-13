"""Check image/IMU serialization without a ROS graph or native tracking work."""
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from sensor_msgs.msg import Image, Imu
from cuvslam_rgbd.rgbd_odometry_node import RGBDOdometryNode as Node


def imu(ns):
    m = Imu()
    m.header.stamp.nanosec = ns
    m.header.frame_id = 'imu_optical'
    m.linear_acceleration.z = 9.81
    return m


def image(ns, depth=False):
    m = Image()
    m.header.stamp.nanosec = ns
    m.height = m.width = 1
    m.encoding = '16UC1' if depth else 'mono8'
    m.step = 2 if depth else 1
    m.data = bytes([0xe8, 3] if depth else [80])
    return m


def fake_node():
    n = SimpleNamespace(
        tracker=Mock(), use_imu=True, imu_samples=deque(), pending_frames=deque(),
        timestamp_ns=Node.timestamp_ns, color_array=Node.color_array, depth_array=Node.depth_array,
        last_tracker_timestamp_ns=-1, last_imu_received_ns=-1, last_image_timestamp_ns=-1,
        frame_count=0, tracked_count=0, failed_count=0, processing_seconds=0.,
        imu_submitted=0, imu_dropped=0, image_dropped=0, imu_frame=None,
        camera_info=None, tracking_publisher=Mock(), imu_warning=Mock(), get_logger=Mock())
    n.param = lambda name: {'imu_queue_size': 2000, 'sync_queue_size': 30, 'imu_wait_s': .15}[name]
    n.tracker.track.return_value = (SimpleNamespace(world_from_rig=None), None)
    n.process_rgbd = lambda c,d: Node.process_rgbd(n,c,d)
    n.drain_frames = lambda: Node.drain_frames(n)
    return n


class ImuOrderingTest(unittest.TestCase):
    def test_future_sample_is_held_until_next_image(self):
        n = fake_node()
        calls = []
        n.tracker.register_imu_measurement.side_effect = lambda _, m: calls.append(('imu',m.timestamp_ns))
        n.tracker.track.side_effect = lambda ts, **_: (calls.append(('image',ts)) or
                                                       (SimpleNamespace(world_from_rig=None),None))
        Node.on_imu(n, imu(10))
        Node.on_imu(n, imu(30))
        Node.on_rgbd(n, image(20), image(20,True))
        self.assertEqual(calls, [('imu',10),('image',20)])
        Node.on_rgbd(n, image(40), image(40,True))
        self.assertEqual(len(n.pending_frames),1)
        Node.on_imu(n, imu(50))
        self.assertEqual(calls, [('imu',10),('image',20),('imu',30),('image',40)])
        self.assertEqual(Node.timestamp_ns(n.imu_samples[0]),50)

    def test_duplicate_late_and_invalid_imu_are_dropped(self):
        n = fake_node()
        Node.on_imu(n, imu(30))
        Node.on_imu(n, imu(30))
        Node.on_imu(n, imu(20))
        bad = imu(40)
        bad.angular_velocity.x = float('nan')
        Node.on_imu(n,bad)
        unavailable = imu(50)
        unavailable.linear_acceleration_covariance[0] = -1
        Node.on_imu(n,unavailable)
        self.assertEqual(n.imu_dropped,4)
        self.assertEqual(len(n.imu_samples),1)
        n.last_tracker_timestamp_ns=60
        Node.on_imu(n,imu(55))
        self.assertEqual(n.imu_dropped,5)

    def test_missing_imu_drops_image_after_bounded_wait(self):
        n = fake_node()
        n.pending_frames.append((0.,image(20),image(20,True)))
        Node.drain_frames(n)
        self.assertEqual(n.image_dropped,1)
        n.tracker.track.assert_not_called()
        n.tracking_publisher.publish.assert_called_once()
