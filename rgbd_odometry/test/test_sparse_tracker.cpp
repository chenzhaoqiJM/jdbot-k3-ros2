#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "rgbd_odometry/sparse_tracker.hpp"

namespace rgbd_odometry {
namespace {

TEST(SparseTracker, RecoversIdentityFromTexturedRgbdPair) {
  Intrinsics intrinsics;
  intrinsics.width = 160;
  intrinsics.height = 120;
  intrinsics.fx = 150.0;
  intrinsics.fy = 150.0;
  intrinsics.cx = 79.5;
  intrinsics.cy = 59.5;
  const size_t count = static_cast<size_t>(intrinsics.width) * intrinsics.height;
  std::vector<uint8_t> gray(count);
  std::vector<float> depth(count, 1.5F);
  std::vector<float> ray_x(count), ray_y(count);
  for (int y = 0; y < intrinsics.height; ++y) {
    for (int x = 0; x < intrinsics.width; ++x) {
      const size_t index = static_cast<size_t>(y) * intrinsics.width + x;
      const bool checker = ((x / 8) + (y / 8)) % 2 != 0;
      gray[index] = static_cast<uint8_t>(checker ? 220 : 30);
      // Slight non-planarity prevents a degenerate three-point RANSAC sample.
      depth[index] += 0.08F * std::sin(0.07F * x) * std::cos(0.09F * y);
      ray_x[index] = static_cast<float>((x - intrinsics.cx) / intrinsics.fx);
      ray_y[index] = static_cast<float>((y - intrinsics.cy) / intrinsics.fy);
    }
  }
  SparseTracker tracker;
  const SparseTrackingResult result =
      tracker.track(gray, depth, gray, depth, intrinsics, ray_x, ray_y);
  ASSERT_TRUE(result.valid);
  EXPECT_GE(result.inliers, 18);
  EXPECT_LT(result.previous_T_current.translation().norm(), 1e-4);
  EXPECT_LT(Eigen::AngleAxisd(result.previous_T_current.linear()).angle(), 1e-4);
}

}  // namespace
}  // namespace rgbd_odometry
