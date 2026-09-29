#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace k3_rgbd_odometry {

struct CameraModel {
  int width = 0;
  int height = 0;
  float fx = 0.0F;
  float fy = 0.0F;
  float cx = 0.0F;
  float cy = 0.0F;
};

struct FrontendConfig {
  int pyramid_levels = 4;
  int cell_size = 24;
  int max_features = 420;
  int min_features = 45;
  int patch_radius = 3;
  int klt_iterations = 8;
  float min_depth_m = 0.25F;
  float max_depth_m = 6.0F;
  float min_corner_score = 900.0F;
  float max_patch_rmse = 28.0F;
  float max_klt_step = 4.0F;
  float ransac_threshold_m = 0.075F;
  int ransac_iterations = 72;
  float reprojection_threshold_px = 3.0F;
  float depth_scale = 0.001F;
  bool photometric_refinement = false;
  bool planar_refinement = false;
};

struct TrackingResult {
  bool success = false;
  Eigen::Isometry3d current_from_previous = Eigen::Isometry3d::Identity();
  int detected = 0;
  int tracked = 0;
  int depth_pairs = 0;
  int inliers = 0;
  double reprojection_rmse = 0.0;
  std::string reason;
};

class RgbdFrontend {
 public:
  explicit RgbdFrontend(FrontendConfig config = {});

  TrackingResult process(const uint8_t* color, int color_step, const std::string& encoding,
                         const uint16_t* depth, int depth_step_bytes,
                         const CameraModel& camera);
  void setBaseFromCamera(const Eigen::Isometry3d& base_from_camera);
  void reset();

 public:
  struct GrayImage {
    int width = 0;
    int height = 0;
    std::vector<float> pixels;
    float at(int x, int y) const { return pixels[static_cast<size_t>(y * width + x)]; }
  };
  struct Feature {
    Eigen::Vector2f pixel = Eigen::Vector2f::Zero();
    float score = 0.0F;
  };
  struct Match {
    Eigen::Vector2f previous_pixel = Eigen::Vector2f::Zero();
    Eigen::Vector2f current_pixel = Eigen::Vector2f::Zero();
    Eigen::Vector3d previous_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_point = Eigen::Vector3d::Zero();
    bool has_current_depth = false;
  };

 private:

  std::vector<GrayImage> makePyramid(const uint8_t* color, int step,
                                     const std::string& encoding,
                                     const CameraModel& camera) const;
  std::vector<Feature> detectFeatures(const GrayImage& image, const uint16_t* depth,
                                      int depth_stride, const CameraModel& camera) const;
  bool trackFeature(const std::vector<GrayImage>& previous,
                    const std::vector<GrayImage>& current,
                    const Eigen::Vector2f& point, Eigen::Vector2f& tracked,
                    float& patch_rmse) const;
  bool estimateRigid(const std::vector<Match>& matches, Eigen::Isometry3d& transform,
                     std::vector<int>& inliers);
  bool refineReprojection(const std::vector<Match>& matches, Eigen::Isometry3d& transform,
                          const CameraModel& camera, std::vector<int>& inliers,
                          double& rmse) const;
  bool refinePlanar(const std::vector<Match>& matches, Eigen::Isometry3d& transform,
                    const CameraModel& camera, std::vector<int>& inliers,
                    double& rmse) const;
  bool refineDepthPlanar(const uint16_t* current_depth, int current_stride,
                         Eigen::Isometry3d& transform, const CameraModel& camera) const;
  bool refinePhotometricPlanar(const std::vector<GrayImage>& current,
                               Eigen::Isometry3d& transform,
                               const CameraModel& camera) const;
  bool refineScanPlanar(const uint16_t* current_depth, int current_stride,
                        Eigen::Isometry3d& transform, const CameraModel& camera) const;
  Eigen::Vector3d unproject(const Eigen::Vector2f& pixel, float depth,
                            const CameraModel& camera) const;
  float sampleDepth(const uint16_t* depth, int stride, int width, int height,
                    const Eigen::Vector2f& pixel) const;

  FrontendConfig config_;
  CameraModel previous_camera_;
  std::vector<GrayImage> previous_pyramid_;
  std::vector<uint16_t> previous_depth_;
  Eigen::Isometry3d base_from_camera_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d reference_to_previous_ = Eigen::Isometry3d::Identity();
  int frames_since_reference_ = 0;
  int processed_frames_ = 0;
  double cumulative_visual_distance_ = 0.0;
  bool local_window_active_ = true;
  bool have_base_from_camera_ = false;
  std::mt19937 random_{7U};
};

}  // namespace k3_rgbd_odometry
