#include "lidar_2d_odometry/scan_matcher.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace lidar_2d_odometry {
namespace {

std::vector<Eigen::Vector2d> RoomScan(const Pose2d& sensor) {
  std::vector<Eigen::Vector2d> points;
  const Pose2d inverse = ScanMatcher::Inverse(sensor);
  for (double x = -3.0; x <= 3.0; x += 0.04) points.push_back({x, -2.0});
  for (double y = -1.96; y <= 2.0; y += 0.04) points.push_back({3.0, y});
  for (double x = 2.96; x >= -3.0; x -= 0.04) points.push_back({x, 2.0});
  for (double y = 1.96; y > -2.0; y -= 0.04) points.push_back({-3.0, y});
  for (auto& point : points) {
    const double c = std::cos(inverse.yaw);
    const double s = std::sin(inverse.yaw);
    point = {inverse.x + c * point.x() - s * point.y(),
             inverse.y + s * point.x() + c * point.y()};
  }
  return points;
}

TEST(ScanMatcherTest, RecoversSmallMotion) {
  MatcherOptions options;
  options.keyframe_translation = 0.05;
  ScanMatcher matcher(options);
  ASSERT_TRUE(matcher.AddScan(RoomScan({})).accepted);
  const auto result = matcher.AddScan(RoomScan({0.08, -0.03, 0.02}));
  ASSERT_TRUE(result.accepted);
  EXPECT_NEAR(result.pose.x, 0.08, 0.015);
  EXPECT_NEAR(result.pose.y, -0.03, 0.015);
  EXPECT_NEAR(result.pose.yaw, 0.02, 0.01);
}

TEST(ScanMatcherTest, PoseCompositionRoundTrips) {
  const Pose2d pose{1.2, -0.7, 0.4};
  const Pose2d identity = ScanMatcher::Compose(ScanMatcher::Inverse(pose), pose);
  EXPECT_NEAR(identity.x, 0.0, 1e-12);
  EXPECT_NEAR(identity.y, 0.0, 1e-12);
  EXPECT_NEAR(identity.yaw, 0.0, 1e-12);
}

}  // namespace
}  // namespace lidar_2d_odometry
