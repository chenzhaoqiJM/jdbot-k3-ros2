#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace lidar_2d_odometry {

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct MatcherOptions {
  double voxel_size = 0.045;
  double correspondence_distance = 0.28;
  double huber_scale = 0.06;
  double keyframe_translation = 0.12;
  double keyframe_rotation = 0.06;
  double translation_prediction_weight = 2.0;
  double rotation_prediction_weight = 4.0;
  int max_iterations = 12;
  int min_correspondences = 35;
  int max_keyframes = 90;
};

struct MatchResult {
  Pose2d pose;
  int correspondences = 0;
  double mean_absolute_residual = 0.0;
  bool accepted = false;
};

class ScanMatcher {
 public:
  explicit ScanMatcher(MatcherOptions options = {});

  MatchResult AddScan(const std::vector<Eigen::Vector2d>& points);
  void Reset();
  const Pose2d& pose() const { return pose_; }
  std::size_t map_point_count() const { return map_points_.size(); }

  static Pose2d Compose(const Pose2d& a, const Pose2d& b);
  static Pose2d Inverse(const Pose2d& pose);
  static double NormalizeAngle(double angle);

 private:
  struct OrientedPoint {
    Eigen::Vector2d position;
    Eigen::Vector2d normal;
  };
  struct Keyframe {
    Pose2d pose;
    std::vector<OrientedPoint> points;
  };

  using Cell = std::int64_t;
  static Eigen::Vector2d Transform(const Pose2d& pose,
                                   const Eigen::Vector2d& point);
  std::vector<OrientedPoint> PrepareScan(
      const std::vector<Eigen::Vector2d>& points) const;
  MatchResult Match(const std::vector<OrientedPoint>& scan,
                    const Pose2d& prediction) const;
  void AddKeyframe(const std::vector<OrientedPoint>& scan, const Pose2d& pose);
  void RebuildMap();
  Cell CellKey(int x, int y) const;

  MatcherOptions options_;
  Pose2d pose_;
  Pose2d last_delta_;
  Pose2d last_keyframe_pose_;
  bool initialized_ = false;
  std::deque<Keyframe> keyframes_;
  std::vector<OrientedPoint> map_points_;
  std::unordered_map<Cell, std::vector<std::size_t>> spatial_index_;
};

}  // namespace lidar_2d_odometry
