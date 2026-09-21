#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rgbd_odometry/image.hpp"

using rgbd_odometry::decode_depth;
using rgbd_odometry::parse_intrinsics;
using rgbd_odometry::validate_color;

TEST(Image, AcceptsRequiredColorEncodingsAndPadding) {
  for (const std::string encoding : {"rgb8", "bgr8", "mono8"}) {
    sensor_msgs::msg::Image image;
    image.width = 3; image.height = 2; image.encoding = encoding;
    const uint32_t bpp = encoding == "mono8" ? 1U : 3U;
    image.step = image.width * bpp + 5U;
    image.data.resize(image.step * image.height);
    std::string error;
    EXPECT_TRUE(validate_color(image, error));
  }
}

TEST(Image, RejectsUnsupportedColor) {
  sensor_msgs::msg::Image image;
  image.width = image.height = image.step = 1; image.encoding = "rgba8";
  image.data.resize(1);
  std::string error;
  EXPECT_FALSE(validate_color(image, error));
}

TEST(Image, ConvertsRgbBgrAndMonoToGray) {
  sensor_msgs::msg::Image image;
  image.width = 1; image.height = 1; image.step = 4; image.encoding = "rgb8";
  image.data = {255, 0, 0, 99};
  std::vector<uint8_t> gray;
  std::string error;
  ASSERT_TRUE(rgbd_odometry::decode_grayscale(image, gray, error));
  EXPECT_EQ(gray[0], 77U);
  image.encoding = "bgr8"; image.data = {0, 0, 255, 99};
  ASSERT_TRUE(rgbd_odometry::decode_grayscale(image, gray, error));
  EXPECT_EQ(gray[0], 77U);
  image.encoding = "mono8"; image.step = 2; image.data = {42, 99};
  ASSERT_TRUE(rgbd_odometry::decode_grayscale(image, gray, error));
  EXPECT_EQ(gray[0], 42U);
}

TEST(Image, DecodesLittleEndianDepthWithPaddingAndFiltering) {
  sensor_msgs::msg::Image image;
  image.width = 3; image.height = 1; image.step = 8;
  image.encoding = "16UC1"; image.is_bigendian = false;
  image.data = {0xe8, 0x03, 0x00, 0x00, 0xd0, 0x07, 99, 99};
  std::vector<float> depth;
  std::string error;
  ASSERT_TRUE(decode_depth(image, 1000.0F, 0.2F, 1.5F, depth, error));
  EXPECT_FLOAT_EQ(depth[0], 1.0F);
  EXPECT_TRUE(std::isnan(depth[1]));
  EXPECT_TRUE(std::isnan(depth[2]));
}

TEST(Image, DecodesBigEndianDepth) {
  sensor_msgs::msg::Image image;
  image.width = image.height = 1; image.step = 2;
  image.encoding = "mono16"; image.is_bigendian = true;
  image.data = {0x03, 0xe8};
  std::vector<float> depth;
  std::string error;
  ASSERT_TRUE(decode_depth(image, 1000.0F, 0.2F, 2.0F, depth, error));
  EXPECT_FLOAT_EQ(depth[0], 1.0F);
}

TEST(CameraInfo, ValidatesModelAndBuildsRays) {
  sensor_msgs::msg::CameraInfo info;
  info.width = 4; info.height = 3; info.distortion_model = "plumb_bob";
  info.k[0] = 100.0; info.k[4] = 100.0; info.k[2] = 2.0; info.k[5] = 1.0;
  info.d.assign(5, 0.0);
  rgbd_odometry::Intrinsics intrinsics;
  std::string error;
  ASSERT_TRUE(parse_intrinsics(info, intrinsics, error));
  std::vector<float> x, y;
  ASSERT_TRUE(rgbd_odometry::build_undistorted_rays(intrinsics, x, y, error));
  EXPECT_NEAR(x[1 * 4 + 2], 0.0, 1e-7);
  EXPECT_NEAR(y[1 * 4 + 2], 0.0, 1e-7);
  info.distortion_model = "equidistant";
  EXPECT_FALSE(parse_intrinsics(info, intrinsics, error));
}
