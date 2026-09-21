#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "rgbd_odometry/engine.hpp"

TEST(Engine, Se3ExponentialIsFiniteAndCorrectAtZero) {
  Eigen::Matrix<double, 6, 1> zero = Eigen::Matrix<double, 6, 1>::Zero();
  EXPECT_TRUE(rgbd_odometry::exp_se3(zero).matrix().isApprox(Eigen::Matrix4d::Identity()));
  zero(0) = 0.1; zero(5) = 0.2;
  const auto transform = rgbd_odometry::exp_se3(zero);
  EXPECT_TRUE(transform.matrix().allFinite());
  EXPECT_NEAR(Eigen::AngleAxisd(transform.linear()).angle(), 0.2, 1e-9);
}

TEST(Engine, FirstValidFrameProducesIdentityDelta) {
  rgbd_odometry::Intrinsics k;
  k.width = 32; k.height = 24; k.fx = 30.0; k.fy = 30.0; k.cx = 16.0; k.cy = 12.0;
  std::vector<float> x(k.width * k.height), y(x.size());
  for (int v = 0; v < k.height; ++v) for (int u = 0; u < k.width; ++u) {
    x[v * k.width + u] = static_cast<float>((u - k.cx) / k.fx);
    y[v * k.width + u] = static_cast<float>((v - k.cy) / k.fy);
  }
  rgbd_odometry::Engine engine;
  ASSERT_TRUE(engine.configure(k, x, y));
  std::vector<float> depth(x.size(), 1.0F);
  std::vector<uint8_t> gray(x.size(), 80U);
  const auto result = engine.process(depth, gray);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.previous_T_current.matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST(PlanarOutputMath, QuaternionUsesRosXyzwConvention) {
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(q.x(), 0.0, 1e-9);
  EXPECT_NEAR(q.y(), 0.0, 1e-9);
  EXPECT_NEAR(q.z(), std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(q.w(), std::sqrt(0.5), 1e-9);
}
