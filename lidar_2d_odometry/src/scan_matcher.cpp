#include "lidar_2d_odometry/scan_matcher.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace lidar_2d_odometry {
namespace {

double SquaredDistance(const Pose2d& a, const Pose2d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

}  // namespace

ScanMatcher::ScanMatcher(MatcherOptions options) : options_(options) {
  if (!(options_.voxel_size > 0.0) ||
      !(options_.correspondence_distance > options_.voxel_size) ||
      !(options_.huber_scale > 0.0) || options_.max_iterations < 1 ||
      options_.min_correspondences < 3 || options_.max_keyframes < 2) {
    throw std::invalid_argument("invalid scan matcher options");
  }
}

double ScanMatcher::NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2d ScanMatcher::Compose(const Pose2d& a, const Pose2d& b) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
          NormalizeAngle(a.yaw + b.yaw)};
}

Pose2d ScanMatcher::Inverse(const Pose2d& pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {-c * pose.x - s * pose.y, s * pose.x - c * pose.y,
          NormalizeAngle(-pose.yaw)};
}

Eigen::Vector2d ScanMatcher::Transform(const Pose2d& pose,
                                       const Eigen::Vector2d& point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x() - s * point.y(),
          pose.y + s * point.x() + c * point.y()};
}

ScanMatcher::Cell ScanMatcher::CellKey(int x, int y) const {
  const auto high = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
  const auto low = static_cast<std::uint32_t>(y);
  return static_cast<Cell>((high << 32) | low);
}

std::vector<ScanMatcher::OrientedPoint> ScanMatcher::PrepareScan(
    const std::vector<Eigen::Vector2d>& points) const {
  std::vector<OrientedPoint> result;
  if (points.size() < 5) return result;
  result.reserve(points.size());
  std::unordered_set<Cell> occupied;
  for (std::size_t i = 2; i + 2 < points.size(); ++i) {
    const Eigen::Vector2d tangent = points[i + 2] - points[i - 2];
    if (tangent.squaredNorm() < 1e-5 ||
        (points[i] - points[i - 1]).norm() > 0.35 ||
        (points[i + 1] - points[i]).norm() > 0.35) {
      continue;
    }
    const int ix = static_cast<int>(std::floor(points[i].x() / options_.voxel_size));
    const int iy = static_cast<int>(std::floor(points[i].y() / options_.voxel_size));
    if (!occupied.insert(CellKey(ix, iy)).second) continue;
    Eigen::Vector2d normal(-tangent.y(), tangent.x());
    normal.normalize();
    result.push_back({points[i], normal});
  }
  return result;
}

MatchResult ScanMatcher::Match(const std::vector<OrientedPoint>& scan,
                               const Pose2d& prediction) const {
  Pose2d estimate = prediction;
  MatchResult result;
  constexpr int radius = 1;
  const double max_distance_sq = options_.correspondence_distance *
                                 options_.correspondence_distance;

  for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    int count = 0;
    double absolute_residual = 0.0;
    const double c = std::cos(estimate.yaw);
    const double s = std::sin(estimate.yaw);

    for (const auto& point : scan) {
      const Eigen::Vector2d world = Transform(estimate, point.position);
      const int cx = static_cast<int>(
          std::floor(world.x() / options_.correspondence_distance));
      const int cy = static_cast<int>(
          std::floor(world.y() / options_.correspondence_distance));
      const Eigen::Vector2d world_normal(
          c * point.normal.x() - s * point.normal.y(),
          s * point.normal.x() + c * point.normal.y());
      const OrientedPoint* nearest = nullptr;
      double nearest_sq = max_distance_sq;
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          const auto cell = spatial_index_.find(CellKey(cx + dx, cy + dy));
          if (cell == spatial_index_.end()) continue;
          for (const std::size_t index : cell->second) {
            if (std::abs(map_points_[index].normal.dot(world_normal)) < 0.6) {
              continue;
            }
            const double distance_sq =
                (map_points_[index].position - world).squaredNorm();
            if (distance_sq < nearest_sq) {
              nearest_sq = distance_sq;
              nearest = &map_points_[index];
            }
          }
        }
      }
      if (nearest == nullptr) continue;

      const double residual = nearest->normal.dot(world - nearest->position);
      const double abs_residual = std::abs(residual);
      const double weight = abs_residual <= options_.huber_scale
                                ? 1.0
                                : options_.huber_scale / abs_residual;
      const Eigen::Vector2d derivative(-s * point.position.x() -
                                           c * point.position.y(),
                                       c * point.position.x() -
                                           s * point.position.y());
      Eigen::Vector3d jacobian;
      jacobian << nearest->normal.x(), nearest->normal.y(),
          nearest->normal.dot(derivative);
      hessian.noalias() += weight * jacobian * jacobian.transpose();
      gradient.noalias() += weight * jacobian * residual;
      absolute_residual += abs_residual;
      ++count;
    }

    result.correspondences = count;
    result.mean_absolute_residual = count > 0 ? absolute_residual / count : 0.0;
    if (count < options_.min_correspondences) return result;

    // A weak constant-velocity prior resolves corridor degeneracy without
    // preventing the range data from correcting the prediction.
    const Eigen::Vector3d prior_error(
        estimate.x - prediction.x, estimate.y - prediction.y,
        NormalizeAngle(estimate.yaw - prediction.yaw));
    const Eigen::Vector3d prior_weight(
        options_.translation_prediction_weight,
        options_.translation_prediction_weight,
        options_.rotation_prediction_weight);
    hessian.diagonal() += prior_weight;
    gradient.noalias() += prior_weight.cwiseProduct(prior_error);
    hessian.diagonal().array() += 1e-6;

    const Eigen::Vector3d update = -hessian.ldlt().solve(gradient);
    if (!update.allFinite()) return result;
    estimate.x += update.x();
    estimate.y += update.y();
    estimate.yaw = NormalizeAngle(estimate.yaw + update.z());
    if (update.head<2>().norm() < 1e-4 && std::abs(update.z()) < 1e-4) break;
  }
  result.pose = estimate;
  result.accepted = true;
  return result;
}

void ScanMatcher::AddKeyframe(const std::vector<OrientedPoint>& scan,
                              const Pose2d& pose) {
  Keyframe keyframe;
  keyframe.pose = pose;
  keyframe.points.reserve(scan.size());
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  for (const auto& point : scan) {
    keyframe.points.push_back(
        {Transform(pose, point.position),
         {c * point.normal.x() - s * point.normal.y(),
          s * point.normal.x() + c * point.normal.y()}});
  }
  keyframes_.push_back(std::move(keyframe));
  while (static_cast<int>(keyframes_.size()) > options_.max_keyframes) {
    keyframes_.pop_front();
  }
  last_keyframe_pose_ = pose;
  RebuildMap();
}

void ScanMatcher::RebuildMap() {
  map_points_.clear();
  spatial_index_.clear();
  std::unordered_set<Cell> occupied;
  // Keep the newest observation in each fine voxel. This avoids overweighting
  // walls merely because the robot was stationary near them for many scans.
  for (auto keyframe = keyframes_.rbegin(); keyframe != keyframes_.rend();
       ++keyframe) {
    for (const auto& point : keyframe->points) {
      const int x = static_cast<int>(
          std::floor(point.position.x() / options_.voxel_size));
      const int y = static_cast<int>(
          std::floor(point.position.y() / options_.voxel_size));
      if (occupied.insert(CellKey(x, y)).second) map_points_.push_back(point);
    }
  }
  spatial_index_.reserve(map_points_.size());
  for (std::size_t i = 0; i < map_points_.size(); ++i) {
    const int x = static_cast<int>(std::floor(
        map_points_[i].position.x() / options_.correspondence_distance));
    const int y = static_cast<int>(std::floor(
        map_points_[i].position.y() / options_.correspondence_distance));
    spatial_index_[CellKey(x, y)].push_back(i);
  }
}

MatchResult ScanMatcher::AddScan(const std::vector<Eigen::Vector2d>& points) {
  const auto scan = PrepareScan(points);
  MatchResult result;
  if (scan.size() < static_cast<std::size_t>(options_.min_correspondences)) {
    result.pose = pose_;
    return result;
  }
  if (!initialized_) {
    initialized_ = true;
    AddKeyframe(scan, pose_);
    result.pose = pose_;
    result.correspondences = static_cast<int>(scan.size());
    result.accepted = true;
    return result;
  }

  const Pose2d previous = pose_;
  const Pose2d prediction = Compose(pose_, last_delta_);
  result = Match(scan, prediction);
  if (!result.accepted) {
    result.pose = pose_;
    last_delta_ = {};
    return result;
  }
  pose_ = result.pose;
  last_delta_ = Compose(Inverse(previous), pose_);
  if (SquaredDistance(pose_, last_keyframe_pose_) >=
          options_.keyframe_translation * options_.keyframe_translation ||
      std::abs(NormalizeAngle(pose_.yaw - last_keyframe_pose_.yaw)) >=
          options_.keyframe_rotation) {
    AddKeyframe(scan, pose_);
  }
  return result;
}

void ScanMatcher::Reset() {
  pose_ = {};
  last_delta_ = {};
  last_keyframe_pose_ = {};
  initialized_ = false;
  keyframes_.clear();
  map_points_.clear();
  spatial_index_.clear();
}

}  // namespace lidar_2d_odometry
