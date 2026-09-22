#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rgbd_odometry/image.hpp"

namespace rgbd_odometry {

struct SparseTrackerConfig {
  int max_features{180};
  int pyramid_levels{3};
  int patch_radius{3};
  int iterations{7};
  float quality_fraction{0.05F};
  float minimum_distance_px{12.0F};
  float forward_backward_max_error_px{1.0F};
  float ransac_threshold_m{0.055F};
  int ransac_iterations{64};
  int minimum_inliers{18};
};

struct SparseTrackingResult {
  bool valid{false};
  Eigen::Isometry3d previous_T_current{Eigen::Isometry3d::Identity()};
  int detected{0};
  int tracked{0};
  int depth_pairs{0};
  int inliers{0};
  double rmse_m{0.0};
};

// Independent clean-room implementation of the sparse front-end described by
// Shi--Tomasi, pyramidal Lucas--Kanade, and absolute orientation.  It is kept
// separate from the dense projective solver so the two estimators do not share
// correspondence failure modes.
class SparseTracker {
 public:
  explicit SparseTracker(SparseTrackerConfig config = {});
  SparseTrackingResult track(const std::vector<uint8_t> &previous_gray,
                             const std::vector<float> &previous_depth,
                             const std::vector<uint8_t> &current_gray,
                             const std::vector<float> &current_depth,
                             const Intrinsics &intrinsics,
                             const std::vector<float> &ray_x,
                             const std::vector<float> &ray_y) const;
  SparseTrackingResult match_revisit(
      const std::vector<uint8_t> &previous_gray,
      const std::vector<float> &previous_depth,
      const std::vector<uint8_t> &current_gray,
      const std::vector<float> &current_depth,
      const Intrinsics &intrinsics,
      const std::vector<float> &ray_x,
      const std::vector<float> &ray_y,
      const Eigen::Isometry3d &expected_previous_T_current) const;

 private:
  SparseTrackerConfig config_;
};

}  // namespace rgbd_odometry
