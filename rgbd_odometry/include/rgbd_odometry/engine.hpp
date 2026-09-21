#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rgbd_odometry/image.hpp"

namespace rgbd_odometry {

struct EngineConfig {
  int coarse_stride{8};
  int fine_stride{4};
  int coarse_iterations{8};
  int fine_iterations{3};
  float max_correspondence_m{0.08F};
  float max_normal_angle_deg{55.0F};
  int min_correspondences{180};
  float huber_delta_m{0.03F};
  int depth_filter_radius{1};
  float depth_filter_max_difference_m{0.05F};
  float max_translation_per_frame_m{0.25F};
  float max_rotation_per_frame_rad{0.35F};
  float color_static_threshold{8.0F};
  int static_keyframe_max_frames{60};
  float photometric_weight{0.1F};
  bool photometric_affine_compensation{false};
  float local_keyframe_fusion_weight{0.0F};
  int local_keyframe_max_frames{12};
  float local_keyframe_max_translation_m{0.15F};
  float local_keyframe_max_rotation_rad{0.15F};
  float local_keyframe_consistency_translation_m{0.06F};
  float local_keyframe_consistency_rotation_rad{0.08F};
};

struct TrackingResult {
  bool valid{false};
  Eigen::Isometry3d previous_T_current{Eigen::Isometry3d::Identity()};
  int correspondences{0};
  double rmse_m{0.0};
  Eigen::Matrix<double, 6, 6> information{
      Eigen::Matrix<double, 6, 6>::Zero()};
};

class Engine {
 public:
  explicit Engine(EngineConfig config = {});
  bool configure(const Intrinsics &intrinsics,
                 const std::vector<float> &ray_x,
                 const std::vector<float> &ray_y);
  void reset();
  TrackingResult process(const std::vector<float> &depth_m,
                         const std::vector<uint8_t> &gray);
  bool initialized() const { return initialized_; }

 private:
  struct Level {
    int width{0};
    int height{0};
    int stride{1};
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> normals;
    std::vector<float> intensity;
    std::vector<uint8_t> valid;
    float intensity_mean{0.0F};
    float intensity_stddev{1.0F};
  };

  Level make_level(const std::vector<float> &depth_m,
                   const std::vector<uint8_t> &gray, int stride) const;
  bool solve_level(const Level &previous, const Level &current,
                   const Intrinsics &scaled, int iterations,
                   Eigen::Isometry3d &previous_T_current,
                   TrackingResult &result) const;
  bool track_pair(const std::vector<float> &reference_depth,
                  const std::vector<uint8_t> &reference_gray,
                  const std::vector<float> &current_depth,
                  const std::vector<uint8_t> &current_gray,
                  Eigen::Isometry3d &reference_T_current,
                  TrackingResult &result) const;

  EngineConfig config_;
  Intrinsics intrinsics_;
  std::vector<float> ray_x_;
  std::vector<float> ray_y_;
  std::vector<float> previous_depth_;
  std::vector<uint8_t> previous_gray_;
  std::vector<float> keyframe_depth_;
  std::vector<uint8_t> keyframe_gray_;
  Eigen::Isometry3d last_delta_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d keyframe_T_previous_{Eigen::Isometry3d::Identity()};
  bool configured_{false};
  bool initialized_{false};
  int static_frames_{0};
  int frames_since_keyframe_{0};
};

Eigen::Isometry3d exp_se3(const Eigen::Matrix<double, 6, 1> &xi);

}  // namespace rgbd_odometry
