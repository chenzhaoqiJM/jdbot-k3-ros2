#pragma once

#include "k3_rgbd_odometry/rgbd_frontend.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace k3_rgbd_odometry {

struct LoopClosureConfig {
  bool enabled = true;
  double keyframe_translation_m = 0.45;
  double keyframe_rotation_rad = 0.32;
  int min_keyframe_separation = 10;
  int max_features = 320;
  int descriptor_distance = 80;
  int min_matches = 16;
  int min_inliers = 12;
  double ransac_threshold_m = 0.15;
  int ransac_iterations = 100;
  double max_consistency_translation_m = 8.0;
  double max_consistency_rotation_rad = 3.14;
};

struct LoopClosureResult {
  Eigen::Isometry3d corrected_pose = Eigen::Isometry3d::Identity();
  bool keyframe_added = false;
  bool loop_detected = false;
  int loop_inliers = 0;
  int keyframes = 0;
};

struct OptimizedGraphPose {
  int64_t stamp_ns = 0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

class LoopClosure {
 public:
  explicit LoopClosure(LoopClosureConfig config = {});

  LoopClosureResult update(const uint8_t* color, int color_step, const std::string& encoding,
                           const uint16_t* depth, int depth_stride_bytes,
                           const CameraModel& camera,
                           const Eigen::Isometry3d& base_from_camera,
                           const Eigen::Isometry3d& raw_pose, int64_t stamp_ns,
                           double odometry_information = 1.0);
  Eigen::Isometry3d correctPose(const Eigen::Isometry3d& raw_pose) const;
  Eigen::Isometry3d correctHistoricalPose(const Eigen::Isometry3d& raw_pose,
                                          int64_t stamp_ns) const;
  std::vector<OptimizedGraphPose> optimizedPath() const;
  void reset();

 private:
  struct DescriptorFeature {
    Eigen::Vector2f pixel = Eigen::Vector2f::Zero();
    Eigen::Vector2d base_point = Eigen::Vector2d::Zero();
    std::array<uint64_t, 4> descriptor{};
  };
  struct Keyframe {
    int64_t stamp_ns = 0;
    Eigen::Vector3d raw_pose = Eigen::Vector3d::Zero();
    Eigen::Vector3d optimized_pose = Eigen::Vector3d::Zero();
    std::vector<DescriptorFeature> features;
  };
  struct Edge {
    int from = 0;
    int to = 0;
    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
    bool loop = false;
    double information_scale = 1.0;
  };

  std::vector<float> makeGray(const uint8_t* color, int step, const std::string& encoding,
                              int width, int height) const;
  std::vector<DescriptorFeature> extractFeatures(const std::vector<float>& gray,
      const uint16_t* depth, int depth_stride, const CameraModel& camera,
      const Eigen::Isometry3d& base_from_camera) const;
  bool verifyLoop(const Keyframe& candidate, const Keyframe& current,
                  Eigen::Vector3d& candidate_from_current, int& inliers);
  void optimizeGraph();

  static Eigen::Vector3d pose2d(const Eigen::Isometry3d& pose);
  static Eigen::Isometry3d pose3d(const Eigen::Vector3d& pose);
  static Eigen::Vector3d compose(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
  static Eigen::Vector3d inverse(const Eigen::Vector3d& pose);
  static Eigen::Vector3d between(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
  static double wrap(double angle);

  LoopClosureConfig config_;
  std::vector<Keyframe> keyframes_;
  std::vector<Edge> edges_;
  Eigen::Isometry3d correction_ = Eigen::Isometry3d::Identity();
  double accumulated_information_ = 1.0;
  std::mt19937 random_{19U};
};

}  // namespace k3_rgbd_odometry
