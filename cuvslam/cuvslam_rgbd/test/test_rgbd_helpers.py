import unittest

import numpy as np
from sensor_msgs.msg import Image

from cuvslam_rgbd.rgbd_odometry_node import RGBDOdometryNode


class RGBDHelpersTest(unittest.TestCase):
    def test_ros_image_arrays_handle_row_padding(self):
        color = Image()
        color.width = 2
        color.height = 2
        color.encoding = 'rgb8'
        color.step = 8
        color.data = bytes([
            1, 2, 3, 4, 5, 6, 99, 99,
            7, 8, 9, 10, 11, 12, 99, 99,
        ])
        np.testing.assert_array_equal(
            RGBDOdometryNode.color_array(color),
            np.array([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], dtype=np.uint8),
        )

        depth = Image()
        depth.width = 2
        depth.height = 2
        depth.encoding = '16UC1'
        depth.step = 6
        depth.data = bytes([
            0xE8, 0x03, 0xD0, 0x07, 0xFF, 0xFF,
            0xB8, 0x0B, 0xA0, 0x0F, 0xFF, 0xFF,
        ])
        np.testing.assert_array_equal(
            RGBDOdometryNode.depth_array(depth),
            np.array([[1000, 2000], [3000, 4000]], dtype=np.uint16),
        )
