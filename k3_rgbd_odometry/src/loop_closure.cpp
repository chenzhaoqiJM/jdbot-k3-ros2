#include "k3_rgbd_odometry/loop_closure.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace k3_rgbd_odometry {
namespace {

int hamming(const std::array<uint64_t, 4>& a, const std::array<uint64_t, 4>& b) {
  int distance = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    distance += __builtin_popcountll(a[i] ^ b[i]);
  }
  return distance;
}

uint32_t mixBits(uint32_t value) {
  value ^= value >> 16;
  value *= 0x7feb352dU;
  value ^= value >> 15;
  value *= 0x846ca68bU;
  value ^= value >> 16;
  return value;
}

}  // namespace

LoopClosure::LoopClosure(LoopClosureConfig config) : config_(std::move(config)) {}

void LoopClosure::reset() {
  keyframes_.clear();
  edges_.clear();
  correction_ = Eigen::Isometry3d::Identity();
  accumulated_information_ = 1.0;
}

double LoopClosure::wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d LoopClosure::pose2d(const Eigen::Isometry3d& pose) {
  return Eigen::Vector3d(pose.translation().x(), pose.translation().y(),
                         std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0)));
}

Eigen::Isometry3d LoopClosure::pose3d(const Eigen::Vector3d& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.x(), pose.y(), 0.0);
  result.linear() = Eigen::AngleAxisd(pose.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

Eigen::Vector3d LoopClosure::compose(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  const double c = std::cos(a.z());
  const double s = std::sin(a.z());
  return Eigen::Vector3d(a.x() + c * b.x() - s * b.y(),
                         a.y() + s * b.x() + c * b.y(), wrap(a.z() + b.z()));
}

Eigen::Vector3d LoopClosure::inverse(const Eigen::Vector3d& pose) {
  const double c = std::cos(pose.z());
  const double s = std::sin(pose.z());
  return Eigen::Vector3d(-c * pose.x() - s * pose.y(),
                          s * pose.x() - c * pose.y(), wrap(-pose.z()));
}

Eigen::Vector3d LoopClosure::between(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return compose(inverse(a), b);
}

std::vector<float> LoopClosure::makeGray(const uint8_t* color, int step,
                                         const std::string& encoding,
                                         int width, int height) const {
  std::vector<float> gray(static_cast<size_t>(width * height));
  const bool mono = encoding == "mono8" || encoding == "8UC1";
  const bool bgr = encoding == "bgr8";
  for (int y = 0; y < height; ++y) {
    const uint8_t* row = color + static_cast<size_t>(y * step);
    for (int x = 0; x < width; ++x) {
      if (mono) {
        gray[static_cast<size_t>(y * width + x)] = row[x];
      } else {
        const uint8_t* p = row + 3 * x;
        const float r = bgr ? p[2] : p[0];
        const float g = p[1];
        const float b = bgr ? p[0] : p[2];
        gray[static_cast<size_t>(y * width + x)] = 0.299F * r + 0.587F * g + 0.114F * b;
      }
    }
  }
  return gray;
}

std::vector<LoopClosure::DescriptorFeature> LoopClosure::extractFeatures(
    const std::vector<float>& gray, const uint16_t* depth, int depth_stride,
    const CameraModel& camera, const Eigen::Isometry3d& base_from_camera) const {
  struct Candidate { int x = 0; int y = 0; float score = 0.0F; };
  std::vector<Candidate> candidates;
  const int border = 17;
  const int cell = 24;
  auto pixel = [&gray, &camera](int x, int y) {
    return gray[static_cast<size_t>(y * camera.width + x)];
  };
  for (int cy = border; cy < camera.height - border; cy += cell) {
    for (int cx = border; cx < camera.width - border; cx += cell) {
      Candidate best;
      for (int y = cy; y < std::min(cy + cell, camera.height - border); y += 2) {
        for (int x = cx; x < std::min(cx + cell, camera.width - border); x += 2) {
          double gxx = 0.0, gxy = 0.0, gyy = 0.0;
          for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
              const double gx = 0.5 * (pixel(x + dx + 1, y + dy) -
                                       pixel(x + dx - 1, y + dy));
              const double gy = 0.5 * (pixel(x + dx, y + dy + 1) -
                                       pixel(x + dx, y + dy - 1));
              gxx += gx * gx; gxy += gx * gy; gyy += gy * gy;
            }
          }
          const double trace = gxx + gyy;
          const float score = static_cast<float>(0.5 * (trace - std::sqrt(std::max(
              0.0, (gxx - gyy) * (gxx - gyy) + 4.0 * gxy * gxy))));
          if (score > best.score) best = {x, y, score};
        }
      }
      if (best.score > 650.0F) candidates.push_back(best);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score > b.score; });
  if (static_cast<int>(candidates.size()) > config_.max_features) {
    candidates.resize(static_cast<size_t>(config_.max_features));
  }

  std::vector<DescriptorFeature> features;
  features.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    std::array<uint16_t, 9> depths{};
    int count = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const uint16_t value = depth[(candidate.y + dy) * depth_stride + candidate.x + dx];
        if (value != 0) depths[static_cast<size_t>(count++)] = value;
      }
    }
    if (count < 3) continue;
    std::nth_element(depths.begin(), depths.begin() + count / 2, depths.begin() + count);
    const double z = depths[static_cast<size_t>(count / 2)] * 0.001;
    if (z < 0.25 || z > 6.0) continue;
    const Eigen::Vector3d camera_point(
        (candidate.x - camera.cx) * z / camera.fx,
        (candidate.y - camera.cy) * z / camera.fy, z);
    const Eigen::Vector3d base_point = base_from_camera * camera_point;
    DescriptorFeature feature;
    feature.pixel = Eigen::Vector2f(candidate.x, candidate.y);
    feature.base_point = base_point.head<2>();
    for (int bit = 0; bit < 256; ++bit) {
      const uint32_t first = mixBits(static_cast<uint32_t>(bit) + 0x9e3779b9U);
      const uint32_t second = mixBits(static_cast<uint32_t>(bit) + 0x85ebca6bU);
      const int x1 = static_cast<int>(first & 31U) % 25 - 12;
      const int y1 = static_cast<int>((first >> 8) & 31U) % 25 - 12;
      const int x2 = static_cast<int>(second & 31U) % 25 - 12;
      const int y2 = static_cast<int>((second >> 8) & 31U) % 25 - 12;
      if (pixel(candidate.x + x1, candidate.y + y1) <
          pixel(candidate.x + x2, candidate.y + y2)) {
        feature.descriptor[static_cast<size_t>(bit / 64)] |= uint64_t{1} << (bit % 64);
      }
    }
    features.push_back(feature);
  }
  return features;
}

bool LoopClosure::verifyLoop(const Keyframe& candidate, const Keyframe& current,
                             Eigen::Vector3d& candidate_from_current, int& inliers) {
  struct Pair { int candidate = 0; int current = 0; int distance = 0; };
  std::vector<Pair> tentative;
  for (size_t j = 0; j < current.features.size(); ++j) {
    int best = 257, second = 257, best_index = -1;
    for (size_t i = 0; i < candidate.features.size(); ++i) {
      const int distance = hamming(current.features[j].descriptor,
                                   candidate.features[i].descriptor);
      if (distance < best) { second = best; best = distance; best_index = static_cast<int>(i); }
      else if (distance < second) { second = distance; }
    }
    if (best_index >= 0 && best <= config_.descriptor_distance && best * 100 < second * 88) {
      tentative.push_back({best_index, static_cast<int>(j), best});
    }
  }
  std::sort(tentative.begin(), tentative.end(),
            [](const Pair& a, const Pair& b) { return a.distance < b.distance; });
  std::vector<Pair> matches;
  std::vector<bool> used(candidate.features.size(), false);
  for (const auto& match : tentative) {
    if (!used[static_cast<size_t>(match.candidate)]) {
      used[static_cast<size_t>(match.candidate)] = true;
      matches.push_back(match);
    }
  }
  if (static_cast<int>(matches.size()) < config_.min_matches) return false;

  std::uniform_int_distribution<size_t> distribution(0, matches.size() - 1);
  std::vector<int> best_inliers;
  Eigen::Vector3d best_transform = Eigen::Vector3d::Zero();
  for (int iteration = 0; iteration < config_.ransac_iterations; ++iteration) {
    size_t a = distribution(random_), b = distribution(random_);
    if (a == b) continue;
    const auto& ma = matches[a]; const auto& mb = matches[b];
    const Eigen::Vector2d ca = current.features[static_cast<size_t>(ma.current)].base_point;
    const Eigen::Vector2d cb = current.features[static_cast<size_t>(mb.current)].base_point;
    const Eigen::Vector2d ka = candidate.features[static_cast<size_t>(ma.candidate)].base_point;
    const Eigen::Vector2d kb = candidate.features[static_cast<size_t>(mb.candidate)].base_point;
    const Eigen::Vector2d cv = cb - ca, kv = kb - ka;
    if (cv.norm() < 0.15 || kv.norm() < 0.15) continue;
    const double angle = wrap(std::atan2(kv.y(), kv.x()) - std::atan2(cv.y(), cv.x()));
    const double c = std::cos(angle), s = std::sin(angle);
    Eigen::Matrix2d rotation; rotation << c, -s, s, c;
    const Eigen::Vector2d translation = ka - rotation * ca;
    std::vector<int> selected;
    for (size_t index = 0; index < matches.size(); ++index) {
      const auto& match = matches[index];
      const Eigen::Vector2d cp = current.features[static_cast<size_t>(match.current)].base_point;
      const Eigen::Vector2d kp = candidate.features[static_cast<size_t>(match.candidate)].base_point;
      if ((rotation * cp + translation - kp).norm() < config_.ransac_threshold_m) {
        selected.push_back(static_cast<int>(index));
      }
    }
    if (selected.size() > best_inliers.size()) {
      best_inliers = std::move(selected);
      best_transform = Eigen::Vector3d(translation.x(), translation.y(), angle);
    }
  }
  if (static_cast<int>(best_inliers.size()) < config_.min_inliers) return false;

  Eigen::Vector2d current_mean = Eigen::Vector2d::Zero();
  Eigen::Vector2d candidate_mean = Eigen::Vector2d::Zero();
  for (int index : best_inliers) {
    const auto& match = matches[static_cast<size_t>(index)];
    current_mean += current.features[static_cast<size_t>(match.current)].base_point;
    candidate_mean += candidate.features[static_cast<size_t>(match.candidate)].base_point;
  }
  current_mean /= best_inliers.size(); candidate_mean /= best_inliers.size();
  double dot = 0.0, cross = 0.0;
  for (int index : best_inliers) {
    const auto& match = matches[static_cast<size_t>(index)];
    const Eigen::Vector2d c = current.features[static_cast<size_t>(match.current)].base_point - current_mean;
    const Eigen::Vector2d k = candidate.features[static_cast<size_t>(match.candidate)].base_point - candidate_mean;
    dot += c.dot(k); cross += c.x() * k.y() - c.y() * k.x();
  }
  const double angle = std::atan2(cross, dot);
  Eigen::Matrix2d rotation;
  rotation << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle);
  const Eigen::Vector2d translation = candidate_mean - rotation * current_mean;
  candidate_from_current = Eigen::Vector3d(translation.x(), translation.y(), angle);
  inliers = static_cast<int>(best_inliers.size());
  (void)best_transform;
  return true;
}

void LoopClosure::optimizeGraph() {
  if (keyframes_.size() < 2) return;
  const int variables = static_cast<int>((keyframes_.size() - 1) * 3);
  for (int iteration = 0; iteration < 7; ++iteration) {
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(variables, variables);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variables);
    double total_update = 0.0;
    for (const auto& edge : edges_) {
      const auto residual_for = [this, &edge](const Eigen::Vector3d& from,
                                               const Eigen::Vector3d& to) {
        Eigen::Vector3d residual = between(edge.measurement, between(from, to));
        residual.z() = wrap(residual.z());
        return residual;
      };
      const Eigen::Vector3d residual = residual_for(
          keyframes_[static_cast<size_t>(edge.from)].optimized_pose,
          keyframes_[static_cast<size_t>(edge.to)].optimized_pose);
      Eigen::Matrix3d jf = Eigen::Matrix3d::Zero(), jt = Eigen::Matrix3d::Zero();
      const double epsilon = 1e-5;
      for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d from = keyframes_[static_cast<size_t>(edge.from)].optimized_pose;
        Eigen::Vector3d to = keyframes_[static_cast<size_t>(edge.to)].optimized_pose;
        from(axis) += epsilon; to(axis) += epsilon;
        jf.col(axis) = (residual_for(from,
            keyframes_[static_cast<size_t>(edge.to)].optimized_pose) - residual) / epsilon;
        jt.col(axis) = (residual_for(
            keyframes_[static_cast<size_t>(edge.from)].optimized_pose, to) - residual) / epsilon;
      }
      const Eigen::Vector3d base_weights = edge.loop ? Eigen::Vector3d(600.0, 600.0, 1000.0)
                                                      : Eigen::Vector3d(70.0, 70.0, 120.0);
      const Eigen::Vector3d weights = base_weights * edge.information_scale;
      const Eigen::Matrix3d information = weights.asDiagonal();
      auto add_block = [&hessian, &gradient, &information, &residual](
          int node, const Eigen::Matrix3d& jacobian) {
        if (node == 0) return;
        const int offset = (node - 1) * 3;
        hessian.block<3, 3>(offset, offset) += jacobian.transpose() * information * jacobian;
        gradient.segment<3>(offset) += jacobian.transpose() * information * residual;
      };
      add_block(edge.from, jf); add_block(edge.to, jt);
      if (edge.from != 0 && edge.to != 0) {
        const int a = (edge.from - 1) * 3, b = (edge.to - 1) * 3;
        const Eigen::Matrix3d cross_block = jf.transpose() * information * jt;
        hessian.block<3, 3>(a, b) += cross_block;
        hessian.block<3, 3>(b, a) += cross_block.transpose();
      }
    }
    hessian.diagonal().array() += 1e-6;
    const Eigen::VectorXd update = -hessian.ldlt().solve(gradient);
    if (!update.allFinite()) return;
    for (size_t i = 1; i < keyframes_.size(); ++i) {
      const Eigen::Vector3d delta = update.segment<3>(static_cast<int>((i - 1) * 3));
      keyframes_[i].optimized_pose += delta;
      keyframes_[i].optimized_pose.z() = wrap(keyframes_[i].optimized_pose.z());
      total_update += delta.norm();
    }
    if (total_update < 1e-5) break;
  }
}

Eigen::Isometry3d LoopClosure::correctPose(const Eigen::Isometry3d& raw_pose) const {
  return correction_ * raw_pose;
}

std::vector<OptimizedGraphPose> LoopClosure::optimizedPath() const {
  std::vector<OptimizedGraphPose> path;
  path.reserve(keyframes_.size());
  for (const auto& keyframe : keyframes_) {
    path.push_back({keyframe.stamp_ns, pose3d(keyframe.optimized_pose)});
  }
  return path;
}

LoopClosureResult LoopClosure::update(const uint8_t* color, int color_step,
                                      const std::string& encoding, const uint16_t* depth,
                                      int depth_stride_bytes, const CameraModel& camera,
                                      const Eigen::Isometry3d& base_from_camera,
                                      const Eigen::Isometry3d& raw_pose, int64_t stamp_ns,
                                      double odometry_information) {
  LoopClosureResult result;
  result.corrected_pose = correctPose(raw_pose);
  if (!config_.enabled) return result;
  accumulated_information_ = std::min(
      accumulated_information_, std::clamp(odometry_information, 0.01, 1.0));
  const Eigen::Vector3d raw = pose2d(raw_pose);
  if (!keyframes_.empty()) {
    const Eigen::Vector3d motion = between(keyframes_.back().raw_pose, raw);
    if (motion.head<2>().norm() < config_.keyframe_translation_m &&
        std::abs(motion.z()) < config_.keyframe_rotation_rad) {
      result.keyframes = static_cast<int>(keyframes_.size());
      return result;
    }
  }
  const auto gray = makeGray(color, color_step, encoding, camera.width, camera.height);
  Keyframe current;
  current.stamp_ns = stamp_ns;
  current.raw_pose = raw;
  current.optimized_pose = pose2d(correctPose(raw_pose));
  current.features = extractFeatures(gray, depth, depth_stride_bytes / 2, camera,
                                     base_from_camera);
  if (static_cast<int>(current.features.size()) < config_.min_matches) {
    result.keyframes = static_cast<int>(keyframes_.size());
    return result;
  }
  if (!keyframes_.empty()) {
    edges_.push_back({static_cast<int>(keyframes_.size() - 1),
                      static_cast<int>(keyframes_.size()),
                      between(keyframes_.back().raw_pose, raw), false,
                      accumulated_information_});
  }
  keyframes_.push_back(std::move(current));
  result.keyframe_added = true;
  accumulated_information_ = 1.0;
  const int current_index = static_cast<int>(keyframes_.size() - 1);

  int best_candidate = -1, best_inliers = 0;
  Eigen::Vector3d best_measurement = Eigen::Vector3d::Zero();
  for (int index = 0; index <= current_index - config_.min_keyframe_separation; ++index) {
    Eigen::Vector3d measurement;
    int inliers = 0;
    if (!verifyLoop(keyframes_[static_cast<size_t>(index)], keyframes_.back(),
                    measurement, inliers)) continue;
    const Eigen::Vector3d predicted = between(
        keyframes_[static_cast<size_t>(index)].optimized_pose,
        keyframes_.back().optimized_pose);
    const Eigen::Vector3d consistency = between(measurement, predicted);
    if (consistency.head<2>().norm() > config_.max_consistency_translation_m ||
        std::abs(consistency.z()) > config_.max_consistency_rotation_rad) continue;
    if (inliers > best_inliers) {
      best_candidate = index; best_inliers = inliers; best_measurement = measurement;
    }
  }
  if (best_candidate >= 0) {
    edges_.push_back({best_candidate, current_index, best_measurement, true, 1.0});
    optimizeGraph();
    const Eigen::Isometry3d optimized = pose3d(keyframes_.back().optimized_pose);
    correction_ = optimized * raw_pose.inverse();
    result.corrected_pose = optimized;
    result.loop_detected = true;
    result.loop_inliers = best_inliers;
  }
  result.keyframes = static_cast<int>(keyframes_.size());
  return result;
}

}  // namespace k3_rgbd_odometry
