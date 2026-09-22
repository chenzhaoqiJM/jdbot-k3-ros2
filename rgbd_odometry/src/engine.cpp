#include "rgbd_odometry/engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

namespace rgbd_odometry {
namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

bool project(const Eigen::Vector3d &p, const Intrinsics &k, double &u, double &v) {
  if (!(p.z() > 0.05) || !p.allFinite()) return false;
  double x = p.x() / p.z();
  double y = p.y() / p.z();
  if (!k.d.empty()) {
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    double radial = 1.0 + k.d[0] * r2 + k.d[1] * r4 + k.d[4] * r6;
    if (k.distortion_model == "rational_polynomial") {
      const double denominator = 1.0 + k.d[5] * r2 + k.d[6] * r4 + k.d[7] * r6;
      if (std::abs(denominator) < 1e-12) return false;
      radial /= denominator;
    }
    const double xd = x * radial + 2.0 * k.d[2] * x * y + k.d[3] * (r2 + 2.0 * x * x);
    const double yd = y * radial + k.d[2] * (r2 + 2.0 * y * y) + 2.0 * k.d[3] * x * y;
    x = xd;
    y = yd;
  }
  u = k.fx * x + k.cx;
  v = k.fy * y + k.cy;
  return std::isfinite(u) && std::isfinite(v);
}

bool bilinear(const std::vector<float> &image, int width, int height,
              double u, double v, double &value) {
  const int x = static_cast<int>(std::floor(u));
  const int y = static_cast<int>(std::floor(v));
  if (x < 0 || y < 0 || x + 1 >= width || y + 1 >= height) return false;
  const double ax = u - x;
  const double ay = v - y;
  const size_t i = static_cast<size_t>(y) * width + x;
  value = (1.0 - ay) * ((1.0 - ax) * image[i] + ax * image[i + 1U]) +
          ay * ((1.0 - ax) * image[i + static_cast<size_t>(width)] +
                ax * image[i + static_cast<size_t>(width) + 1U]);
  return std::isfinite(value);
}

Eigen::Isometry3d blend_transform(const Eigen::Isometry3d &a,
                                  const Eigen::Isometry3d &b,
                                  double b_weight) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = (1.0 - b_weight) * a.translation() +
                         b_weight * b.translation();
  Eigen::Quaterniond qa(a.linear());
  Eigen::Quaterniond qb(b.linear());
  result.linear() = qa.slerp(b_weight, qb).normalized().toRotationMatrix();
  return result;
}

Eigen::Isometry3d inverse_isometry(const Eigen::Isometry3d &transform) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = transform.linear().transpose();
  result.translation() = -result.linear() * transform.translation();
  return result;
}

double normalized_image_correlation(const std::vector<uint8_t> &a,
                                    const std::vector<uint8_t> &b) {
  if (a.size() != b.size() || a.empty()) return -1.0;
  constexpr size_t stride = 32;
  double mean_a = 0.0;
  double mean_b = 0.0;
  size_t count = 0;
  for (size_t index = 0; index < a.size(); index += stride) {
    mean_a += a[index];
    mean_b += b[index];
    ++count;
  }
  mean_a /= count;
  mean_b /= count;
  double numerator = 0.0;
  double norm_a = 0.0;
  double norm_b = 0.0;
  for (size_t index = 0; index < a.size(); index += stride) {
    const double centered_a = a[index] - mean_a;
    const double centered_b = b[index] - mean_b;
    numerator += centered_a * centered_b;
    norm_a += centered_a * centered_a;
    norm_b += centered_b * centered_b;
  }
  return numerator / std::sqrt(std::max(norm_a * norm_b, 1e-12));
}

}  // namespace

Eigen::Isometry3d exp_se3(const Eigen::Matrix<double, 6, 1> &xi) {
  const Eigen::Vector3d translation = xi.head<3>();
  const Eigen::Vector3d omega = xi.tail<3>();
  const double theta = omega.norm();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d v_matrix = Eigen::Matrix3d::Identity();
  if (theta < 1e-8) {
    const Eigen::Matrix3d w = skew(omega);
    rotation += w + 0.5 * w * w;
    v_matrix += 0.5 * w + (1.0 / 6.0) * w * w;
  } else {
    const Eigen::Matrix3d w = skew(omega);
    rotation = Eigen::AngleAxisd(theta, omega / theta).toRotationMatrix();
    v_matrix += (1.0 - std::cos(theta)) / (theta * theta) * w +
                (theta - std::sin(theta)) / (theta * theta * theta) * w * w;
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation;
  result.translation() = v_matrix * translation;
  return result;
}

Engine::Engine(EngineConfig config) : config_(config), sparse_tracker_() {}

bool Engine::configure(const Intrinsics &intrinsics,
                       const std::vector<float> &ray_x,
                       const std::vector<float> &ray_y) {
  const size_t expected = static_cast<size_t>(intrinsics.width) * intrinsics.height;
  if (intrinsics.width <= 0 || intrinsics.height <= 0 ||
      ray_x.size() != expected || ray_y.size() != expected) return false;
  intrinsics_ = intrinsics;
  ray_x_ = ray_x;
  ray_y_ = ray_y;
  configured_ = true;
  reset();
  return true;
}

void Engine::reset() {
  previous_depth_.clear();
  previous_gray_.clear();
  keyframe_depth_.clear();
  keyframe_gray_.clear();
  last_delta_.setIdentity();
  keyframe_T_previous_.setIdentity();
  origin_T_previous_.setIdentity();
  window_keyframes_.clear();
  map_keyframes_.clear();
  initialized_ = false;
  static_frames_ = 0;
  frames_since_keyframe_ = 0;
  window_evaluation_counter_ = 0;
  map_frame_counter_ = 0;
  map_query_counter_ = 0;
}

void Engine::reset_window(const std::vector<float> &depth_m,
                          const std::vector<uint8_t> &gray) {
  window_keyframes_.clear();
  append_window_keyframe(depth_m, gray);
  window_evaluation_counter_ = 0;
}

void Engine::append_window_keyframe(const std::vector<float> &depth_m,
                                    const std::vector<uint8_t> &gray) {
  WindowKeyframe keyframe;
  keyframe.depth = depth_m;
  keyframe.gray = gray;
  window_keyframes_.push_back(std::move(keyframe));
  while (static_cast<int>(window_keyframes_.size()) >
         config_.window_max_keyframes) {
    window_keyframes_.pop_front();
  }
}

void Engine::append_map_keyframe(const std::vector<float> &depth_m,
                                 const std::vector<uint8_t> &gray) {
  if (!map_keyframes_.empty()) {
    const Eigen::Isometry3d baseline =
        inverse_isometry(map_keyframes_.back().origin_T_keyframe) *
        origin_T_previous_;
    const double angle = Eigen::AngleAxisd(baseline.linear()).angle();
    if (baseline.translation().norm() < config_.map_keyframe_translation_m &&
        std::abs(angle) < config_.map_keyframe_rotation_rad) {
      return;
    }
  }
  MapKeyframe keyframe;
  keyframe.depth = depth_m;
  keyframe.gray = gray;
  keyframe.origin_T_keyframe = origin_T_previous_;
  keyframe.created_frame = map_frame_counter_;
  if (static_cast<int>(map_keyframes_.size()) >= config_.map_max_keyframes) {
    // Preserve the first keyframe as a stable loop anchor and age out the
    // second-oldest entry. The map remains strictly bounded.
    if (map_keyframes_.size() > 1U) {
      map_keyframes_.erase(map_keyframes_.begin() + 1);
    } else {
      map_keyframes_.pop_front();
    }
  }
  map_keyframes_.push_back(std::move(keyframe));
}

Engine::Level Engine::make_level(const std::vector<float> &depth_m,
                                 const std::vector<uint8_t> &gray, int stride) const {
  Level level;
  level.stride = stride;
  level.width = (intrinsics_.width + stride - 1) / stride;
  level.height = (intrinsics_.height + stride - 1) / stride;
  const size_t count = static_cast<size_t>(level.width) * level.height;
  level.points.resize(count, Eigen::Vector3f::Zero());
  level.normals.resize(count, Eigen::Vector3f::Zero());
  level.intensity.resize(count, 0.0F);
  level.valid.resize(count, 0U);
  for (int y = 0; y < level.height; ++y) {
    const int source_y = std::min(y * stride + stride / 2, intrinsics_.height - 1);
    for (int x = 0; x < level.width; ++x) {
      const int source_x = std::min(x * stride + stride / 2, intrinsics_.width - 1);
      const size_t source = static_cast<size_t>(source_y) * intrinsics_.width + source_x;
      const size_t index = static_cast<size_t>(y) * level.width + x;
      level.intensity[index] = static_cast<float>(gray[source]) / 255.0F;
      float z = depth_m[source];
      if (!std::isfinite(z)) continue;
      if (config_.depth_filter_radius > 0) {
        std::array<float, 25> neighborhood{};
        size_t neighborhood_size = 0;
        const int radius = config_.depth_filter_radius;
        for (int dy = -radius; dy <= radius; ++dy) {
          const int sy = source_y + dy;
          if (sy < 0 || sy >= intrinsics_.height) continue;
          for (int dx = -radius; dx <= radius; ++dx) {
            const int sx = source_x + dx;
            if (sx < 0 || sx >= intrinsics_.width) continue;
            const float candidate =
                depth_m[static_cast<size_t>(sy) * intrinsics_.width + sx];
            if (std::isfinite(candidate) &&
                std::abs(candidate - z) <= config_.depth_filter_max_difference_m) {
              neighborhood[neighborhood_size++] = candidate;
            }
          }
        }
        if (neighborhood_size >= 3U) {
          const auto middle = neighborhood.begin() + neighborhood_size / 2U;
          std::nth_element(neighborhood.begin(), middle,
                           neighborhood.begin() + neighborhood_size);
          z = *middle;
        }
      }
      level.points[index] = Eigen::Vector3f(ray_x_[source] * z, ray_y_[source] * z, z);
      level.valid[index] = 1U;
    }
  }
  double intensity_sum = 0.0;
  double intensity_squared_sum = 0.0;
  for (const float value : level.intensity) {
    intensity_sum += value;
    intensity_squared_sum += static_cast<double>(value) * value;
  }
  level.intensity_mean = static_cast<float>(intensity_sum / count);
  const double variance = std::max(0.0, intensity_squared_sum / count -
                                        intensity_sum * intensity_sum /
                                            (static_cast<double>(count) * count));
  level.intensity_stddev = static_cast<float>(std::sqrt(variance));
  constexpr float max_edge_m = 0.20F;
  for (int y = 1; y + 1 < level.height; ++y) {
    for (int x = 1; x + 1 < level.width; ++x) {
      const size_t i = static_cast<size_t>(y) * level.width + x;
      const size_t il = i - 1U, ir = i + 1U;
      const size_t iu = i - static_cast<size_t>(level.width);
      const size_t id = i + static_cast<size_t>(level.width);
      if (!level.valid[il] || !level.valid[ir] || !level.valid[iu] || !level.valid[id]) continue;
      const Eigen::Vector3f dx = level.points[ir] - level.points[il];
      const Eigen::Vector3f dy = level.points[id] - level.points[iu];
      if (dx.norm() > max_edge_m || dy.norm() > max_edge_m) continue;
      Eigen::Vector3f normal = dx.cross(dy);
      const float norm = normal.norm();
      if (norm < 1e-7F) continue;
      normal /= norm;
      if (normal.z() > 0.0F) normal = -normal;
      level.normals[i] = normal;
    }
  }
  return level;
}

bool Engine::solve_level(const Level &previous, const Level &current,
                         const Intrinsics &scaled, int iterations,
                         int min_correspondences,
                         Eigen::Isometry3d &transform,
                         TrackingResult &result) const {
  const double cosine_threshold = std::cos(config_.max_normal_angle_deg * M_PI / 180.0);
  const double brightness_gain = config_.photometric_affine_compensation
      ? std::clamp(static_cast<double>(previous.intensity_stddev) /
                       std::max(static_cast<double>(current.intensity_stddev), 0.03),
                   0.7, 1.4)
      : 1.0;
  const double brightness_offset = config_.photometric_affine_compensation
      ? previous.intensity_mean - brightness_gain * current.intensity_mean
      : 0.0;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    Eigen::Matrix<double, 6, 6> geometric_hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> geometric_gradient =
        Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> photometric_hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> photometric_gradient =
        Eigen::Matrix<double, 6, 1>::Zero();
    double squared_error = 0.0;
    int count = 0;
    const auto accumulate_photo = [&](const Eigen::Vector3d &point,
                                      double projected_u, double projected_v,
                                      float current_intensity,
                                      bool translation_observable,
                                      Eigen::Matrix<double, 6, 6> &photo_hessian,
                                      Eigen::Matrix<double, 6, 1> &photo_gradient) {
      double target_intensity = 0.0;
      double intensity_left = 0.0, intensity_right = 0.0;
      double intensity_up = 0.0, intensity_down = 0.0;
      const bool has_samples =
          bilinear(previous.intensity, previous.width, previous.height,
                   projected_u, projected_v, target_intensity) &&
          bilinear(previous.intensity, previous.width, previous.height,
                   projected_u - 1.0, projected_v, intensity_left) &&
          bilinear(previous.intensity, previous.width, previous.height,
                   projected_u + 1.0, projected_v, intensity_right) &&
          bilinear(previous.intensity, previous.width, previous.height,
                   projected_u, projected_v - 1.0, intensity_up) &&
          bilinear(previous.intensity, previous.width, previous.height,
                   projected_u, projected_v + 1.0, intensity_down);
      if (!has_samples) return false;
      const double photo_residual = target_intensity -
          (brightness_gain * current_intensity + brightness_offset);
      const double grad_x = 0.5 * (intensity_right - intensity_left);
      const double grad_y = 0.5 * (intensity_down - intensity_up);
      if (grad_x * grad_x + grad_y * grad_y <= 1e-5 ||
          std::abs(photo_residual) >= 0.25) return false;
      Eigen::Matrix<double, 2, 3> projection_jacobian;
      const double inverse_z = 1.0 / point.z();
      projection_jacobian << scaled.fx * inverse_z, 0.0,
          -scaled.fx * point.x() * inverse_z * inverse_z,
          0.0, scaled.fy * inverse_z,
          -scaled.fy * point.y() * inverse_z * inverse_z;
      Eigen::Matrix<double, 3, 6> motion_jacobian;
      if (translation_observable) {
        motion_jacobian.leftCols<3>().setIdentity();
      } else {
        motion_jacobian.leftCols<3>().setZero();
      }
      motion_jacobian.rightCols<3>() = -skew(point);
      Eigen::Matrix<double, 1, 2> image_gradient;
      image_gradient << grad_x, grad_y;
      const Eigen::Matrix<double, 1, 6> photo_jacobian =
          image_gradient * projection_jacobian * motion_jacobian;
      const double photo_huber = std::abs(photo_residual) <= 0.08
          ? 1.0 : 0.08 / std::abs(photo_residual);
      photo_hessian.noalias() +=
          photo_huber * photo_jacobian.transpose() * photo_jacobian;
      photo_gradient.noalias() +=
          photo_huber * photo_jacobian.transpose() * photo_residual;
      return true;
    };
    for (int y = 1; y + 1 < current.height; ++y) {
      for (int x = 1; x + 1 < current.width; ++x) {
        const size_t ci = static_cast<size_t>(y) * current.width + x;
        if (!current.valid[ci]) continue;
        const Eigen::Vector3d transformed =
            transform * current.points[ci].cast<double>();
        double projected_u = 0.0, projected_v = 0.0;
        if (!project(transformed, scaled, projected_u, projected_v)) continue;
        const int px = static_cast<int>(std::lround(projected_u));
        const int py = static_cast<int>(std::lround(projected_v));
        if (px < 1 || py < 1 || px + 1 >= previous.width || py + 1 >= previous.height) continue;
        const size_t pi = static_cast<size_t>(py) * previous.width + px;
        if (!previous.valid[pi] ||
            previous.normals[pi].squaredNorm() < 0.5F) continue;
        const Eigen::Vector3d target = previous.points[pi].cast<double>();
        const Eigen::Vector3d normal = previous.normals[pi].cast<double>();
        const Eigen::Vector3d difference = transformed - target;
        if (std::abs(difference.z()) > config_.max_correspondence_m ||
            difference.norm() > config_.max_correspondence_m * 1.5) continue;
        if (current.normals[ci].squaredNorm() > 0.5F) {
          const Eigen::Vector3d current_normal =
              transform.linear() * current.normals[ci].cast<double>();
          if (std::abs(current_normal.dot(normal)) < cosine_threshold) continue;
        }
        const double residual = normal.dot(difference);
        const double abs_residual = std::abs(residual);
        const double weight = abs_residual <= config_.huber_delta_m
            ? 1.0 : config_.huber_delta_m / abs_residual;
        Eigen::Matrix<double, 1, 6> jacobian;
        jacobian << normal.x(), normal.y(), normal.z(),
                    transformed.cross(normal).x(), transformed.cross(normal).y(),
                    transformed.cross(normal).z();
        geometric_hessian.noalias() += weight * jacobian.transpose() * jacobian;
        geometric_gradient.noalias() += weight * jacobian.transpose() * residual;
        accumulate_photo(transformed, projected_u, projected_v,
                         current.intensity[ci], true,
                         photometric_hessian, photometric_gradient);
        squared_error += residual * residual;
        ++count;
      }
    }
    if (count < min_correspondences) return false;
    // Normalize out the different translation/rotation units before measuring
    // the weakest geometric direction. A low minimum eigenvalue means that
    // surface normals alone cannot distinguish one or more SE(3) motions.
    // In that case, increase the independent image-gradient constraint. This
    // is scene-driven and does not encode a wheeled or planar motion model.
    Eigen::Matrix<double, 6, 1> inverse_sqrt_diagonal;
    for (int i = 0; i < 6; ++i) {
      inverse_sqrt_diagonal(i) =
          1.0 / std::sqrt(std::max(geometric_hessian(i, i), 1e-12));
    }
    const Eigen::Matrix<double, 6, 6> normalized_geometric_hessian =
        inverse_sqrt_diagonal.asDiagonal() * geometric_hessian *
        inverse_sqrt_diagonal.asDiagonal();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(
        normalized_geometric_hessian, Eigen::EigenvaluesOnly);
    if (eigensolver.info() != Eigen::Success) return false;
    const double largest_eigenvalue =
        std::max(eigensolver.eigenvalues().maxCoeff(), 1e-12);
    const double observability = std::clamp(
        eigensolver.eigenvalues().minCoeff() / largest_eigenvalue, 0.0, 1.0);
    double adaptive_fraction = 0.0;
    if (config_.photometric_adaptive_threshold > 0.0F) {
      adaptive_fraction = std::clamp(
          (config_.photometric_adaptive_threshold - observability) /
              config_.photometric_adaptive_threshold,
          0.0, 1.0);
    }
    const double photometric_weight = config_.photometric_weight +
        adaptive_fraction *
            (config_.photometric_max_weight - config_.photometric_weight);
    Eigen::Matrix<double, 6, 6> hessian = geometric_hessian +
        photometric_weight * photometric_hessian;
    const Eigen::Matrix<double, 6, 1> gradient = geometric_gradient +
        photometric_weight * photometric_gradient;
    // Scale-aware Levenberg damping keeps weakly observed directions finite
    // in planar or low-texture scenes without imposing a vehicle model.
    const double damping = std::max(1e-7, hessian.diagonal().maxCoeff() * 1e-6);
    hessian.diagonal().array() += damping;
    const Eigen::Matrix<double, 6, 1> increment = -hessian.ldlt().solve(gradient);
    if (!increment.allFinite()) return false;
    transform = exp_se3(increment) * transform;
    result.correspondences = count;
    result.rmse_m = std::sqrt(squared_error / static_cast<double>(count));
    result.information = hessian / std::max(result.rmse_m * result.rmse_m, 1e-8);
    result.observability = observability;
    result.effective_photometric_weight = photometric_weight;
    if (increment.norm() < 1e-5) break;
  }
  return true;
}

bool Engine::track_pair(const std::vector<float> &reference_depth,
                        const std::vector<uint8_t> &reference_gray,
                        const std::vector<float> &current_depth,
                        const std::vector<uint8_t> &current_gray,
                        Eigen::Isometry3d &reference_T_current,
                        TrackingResult &result) const {
  const int strides[2] = {config_.coarse_stride, config_.fine_stride};
  const int iteration_counts[2] = {config_.coarse_iterations,
                                   config_.fine_iterations};
  const int minimum_correspondences[2] = {
      config_.coarse_min_correspondences,
      config_.fine_min_correspondences};
  for (int level_index = 0; level_index < 2; ++level_index) {
    const int stride = strides[level_index];
    const Level reference = make_level(reference_depth, reference_gray, stride);
    const Level current = make_level(current_depth, current_gray, stride);
    Intrinsics scaled = intrinsics_;
    scaled.width = reference.width;
    scaled.height = reference.height;
    scaled.fx /= stride;
    scaled.fy /= stride;
    scaled.cx = (scaled.cx - stride / 2.0) / stride;
    scaled.cy = (scaled.cy - stride / 2.0) / stride;
    if (!solve_level(reference, current, scaled, iteration_counts[level_index],
                     minimum_correspondences[level_index],
                     reference_T_current, result)) return false;
  }
  return true;
}

TrackingResult Engine::process(const std::vector<float> &depth_m,
                               const std::vector<uint8_t> &gray) {
  TrackingResult result;
  const size_t expected = static_cast<size_t>(intrinsics_.width) * intrinsics_.height;
  if (!configured_ || depth_m.size() != expected || gray.size() != expected) return result;
  if (!initialized_) {
    previous_depth_ = depth_m;
    previous_gray_ = gray;
    keyframe_depth_ = depth_m;
    keyframe_gray_ = gray;
    keyframe_T_previous_.setIdentity();
    frames_since_keyframe_ = 0;
    static_frames_ = 0;
    reset_window(depth_m, gray);
    append_map_keyframe(depth_m, gray);
    initialized_ = true;
    result.valid = true;
    result.correspondences = static_cast<int>(std::count_if(
        depth_m.begin(), depth_m.end(), [](float z) { return std::isfinite(z); }));
    result.information.setIdentity();
    result.information *= 1e4;
    return result;
  }

  // The D415's per-pixel depth can vary by centimetres while the rig is
  // stationary.  Color is substantially more stable, so use a sparse robust
  // appearance check to prevent that depth noise becoming coherent drift.
  // This check observes only the camera input and never a reference pose.
  double signed_color_difference_sum = 0.0;
  size_t color_samples = 0;
  constexpr size_t color_sample_stride = 16;
  for (size_t i = 0; i < gray.size(); i += color_sample_stride) {
    signed_color_difference_sum += static_cast<int>(gray[i]) -
                                   static_cast<int>(previous_gray_[i]);
    ++color_samples;
  }
  const double brightness_offset = signed_color_difference_sum / color_samples;
  double centered_difference_sum = 0.0;
  for (size_t i = 0; i < gray.size(); i += color_sample_stride) {
    const double difference = static_cast<int>(gray[i]) -
                              static_cast<int>(previous_gray_[i]);
    centered_difference_sum += std::abs(difference - brightness_offset);
  }
  const double mean_color_difference = centered_difference_sum / color_samples;
  bool run_map_while_static = false;
  if (mean_color_difference <= config_.color_static_threshold) {
    // Keep the last geometric/appearance keyframe while stationary.  If the
    // rig starts moving very slowly, appearance change then accumulates
    // against this anchor until it is observable, at which point ICP recovers
    // the full keyframe-to-current increment instead of losing each subpixel
    // step to the deadband.
    last_delta_.setIdentity();
    ++static_frames_;
    if (static_frames_ >= config_.static_keyframe_max_frames) {
      previous_depth_ = depth_m;
      previous_gray_ = gray;
      keyframe_depth_ = depth_m;
      keyframe_gray_ = gray;
      keyframe_T_previous_.setIdentity();
      frames_since_keyframe_ = 0;
      static_frames_ = 0;
      reset_window(depth_m, gray);
    }
    result.valid = true;
    result.correspondences = static_cast<int>(color_samples);
    result.rmse_m = 0.001;
    result.information.setIdentity();
    result.information *= 1e5;
    if (config_.map_fusion_weight > 0.0F &&
        map_frame_counter_ >= config_.map_min_keyframe_age) {
      ++map_query_counter_;
      if (map_query_counter_ >= config_.map_query_interval) {
        // The regular map block increments once. Leave it one count short so
        // a stationary revisit can still converge without running ORB on
        // every unchanged frame.
        map_query_counter_ = config_.map_query_interval - 1;
        run_map_while_static = true;
      }
    }
    if (!run_map_while_static) return result;
  }

  Eigen::Isometry3d estimate = last_delta_;
  bool tracked = track_pair(previous_depth_, previous_gray_, depth_m, gray,
                            estimate, result);
  if (!tracked && frames_since_keyframe_ > 0 && !keyframe_depth_.empty()) {
    // Adjacent frames can fail after a short occlusion or depth dropout even
    // though the current view still overlaps the local keyframe. Recover from
    // that independent visual reference before declaring tracking lost.
    Eigen::Isometry3d keyframe_estimate = keyframe_T_previous_ * last_delta_;
    TrackingResult keyframe_result;
    if (track_pair(keyframe_depth_, keyframe_gray_, depth_m, gray,
                   keyframe_estimate, keyframe_result)) {
      const Eigen::Isometry3d recovered =
          inverse_isometry(keyframe_T_previous_) * keyframe_estimate;
      const double recovered_angle =
          Eigen::AngleAxisd(recovered.linear()).angle();
      if (recovered.matrix().allFinite() &&
          recovered.translation().norm() <= config_.max_translation_per_frame_m &&
          std::abs(recovered_angle) <= config_.max_rotation_per_frame_rad &&
          keyframe_result.rmse_m <= config_.max_correspondence_m * 0.7) {
        estimate = recovered;
        result = keyframe_result;
        tracked = true;
      }
    }
  }
  if (!tracked) {
    // Rebase after a lost frame. Keeping an old reference lets a later local
    // minimum create a large, plausible-looking jump.
    previous_depth_ = depth_m;
    previous_gray_ = gray;
    keyframe_depth_ = depth_m;
    keyframe_gray_ = gray;
    keyframe_T_previous_.setIdentity();
    frames_since_keyframe_ = 0;
    static_frames_ = 0;
    last_delta_.setIdentity();
    reset_window(depth_m, gray);
    return TrackingResult{};
  }

  // Dense point-to-plane alignment is weak for translation tangent to a large
  // plane.  Fuse an independently selected and tracked sparse estimate only
  // when its RANSAC consensus agrees with the dense basin.  This adds no
  // vehicle-motion assumption and remains valid for general SE(3) motion.
  if (config_.sparse_fusion_weight > 0.0F) {
    const bool use_sparse_keyframe = frames_since_keyframe_ > 0 &&
                                     !keyframe_depth_.empty();
    const SparseTrackingResult sparse = sparse_tracker_.track(
        use_sparse_keyframe ? keyframe_gray_ : previous_gray_,
        use_sparse_keyframe ? keyframe_depth_ : previous_depth_,
        gray, depth_m, intrinsics_, ray_x_, ray_y_);
    if (sparse.valid) {
      const Eigen::Isometry3d sparse_increment = use_sparse_keyframe
          ? inverse_isometry(keyframe_T_previous_) * sparse.previous_T_current
          : sparse.previous_T_current;
      const Eigen::Isometry3d disagreement =
          inverse_isometry(estimate) * sparse_increment;
      const double disagreement_angle =
          Eigen::AngleAxisd(disagreement.linear()).angle();
      if (disagreement.translation().norm() <=
              config_.sparse_max_translation_disagreement_m &&
          std::abs(disagreement_angle) <=
              config_.sparse_max_rotation_disagreement_rad) {
        const double confidence = std::clamp(
            static_cast<double>(sparse.inliers) / 80.0, 0.35, 1.0);
        const double rotation_weight = config_.sparse_fusion_weight * confidence;
        const Eigen::Vector3d dense_translation = estimate.translation();
        estimate = blend_transform(estimate, sparse_increment, rotation_weight);
        // The sparse translation has already been refined as a one-depth
        // 3D--2D reprojection estimate; retain a conservative independent
        // fusion weight so range noise cannot dominate the dense solution.
        estimate.translation() = dense_translation +
            config_.sparse_translation_fusion_weight * confidence *
                (sparse_increment.translation() - dense_translation);
      }
    }
  }

  // Periodically look farther back than the active keyframe.  A small local
  // window increases parallax and prevents every correction from sharing the
  // same short-baseline drift.  Old-frame constraints remain optional: each
  // must pass its own FB/RANSAC/reprojection checks and agree with the dense
  // basin before it enters a robust consensus.
  ++window_evaluation_counter_;
  if (config_.window_fusion_weight > 0.0F &&
      config_.window_max_keyframes > 1 &&
      window_evaluation_counter_ >= config_.window_evaluation_interval &&
      window_keyframes_.size() > 1U) {
    window_evaluation_counter_ = 0;
    std::vector<std::pair<Eigen::Isometry3d, double>> candidates;
    candidates.reserve(window_keyframes_.size() - 1U);
    for (auto iterator = window_keyframes_.begin();
         iterator + 1 != window_keyframes_.end(); ++iterator) {
      const double baseline_angle =
          Eigen::AngleAxisd(iterator->keyframe_T_previous.linear()).angle();
      if (iterator->keyframe_T_previous.translation().norm() >
              config_.window_max_translation_m ||
          std::abs(baseline_angle) > config_.window_max_rotation_rad) {
        continue;
      }
      const SparseTrackingResult sparse = sparse_tracker_.track(
          iterator->gray, iterator->depth, gray, depth_m,
          intrinsics_, ray_x_, ray_y_);
      if (!sparse.valid) continue;
      const Eigen::Isometry3d candidate =
          inverse_isometry(iterator->keyframe_T_previous) *
          sparse.previous_T_current;
      const Eigen::Isometry3d disagreement =
          inverse_isometry(estimate) * candidate;
      const double disagreement_angle =
          Eigen::AngleAxisd(disagreement.linear()).angle();
      if (disagreement.translation().norm() <=
              config_.sparse_max_translation_disagreement_m &&
          std::abs(disagreement_angle) <=
              config_.sparse_max_rotation_disagreement_rad) {
        candidates.emplace_back(
            candidate,
            std::clamp(static_cast<double>(sparse.inliers) / 80.0, 0.35, 1.0));
      }
    }
    if (!candidates.empty()) {
      Eigen::Isometry3d consensus = candidates.front().first;
      double accumulated_weight = candidates.front().second;
      for (size_t index = 1; index < candidates.size(); ++index) {
        const double next_weight = candidates[index].second;
        consensus = blend_transform(consensus, candidates[index].first,
                                    next_weight /
                                        (accumulated_weight + next_weight));
        accumulated_weight += next_weight;
      }
      const double confidence = std::clamp(accumulated_weight / 1.5, 0.35, 1.0);
      estimate = blend_transform(
          estimate, consensus, config_.window_fusion_weight * confidence);
    }
  }

  // Bounded historical-map relocalization. Candidate selection uses only the
  // current visual estimate; acceptance requires an independent sparse RGB-D
  // match. This is a clean-room, CPU-budgeted analogue of landmark revisit,
  // not a dependency on an external SLAM pose or reference trajectory.
  ++map_query_counter_;
  if (config_.map_fusion_weight > 0.0F &&
      map_query_counter_ >= config_.map_query_interval &&
      !map_keyframes_.empty()) {
    map_query_counter_ = 0;
    result.map_query_attempted = true;
    const bool likely_origin_revisit = !map_keyframes_.empty() &&
        normalized_image_correlation(map_keyframes_.front().gray, gray) >= 0.60;
    if (!likely_origin_revisit && frames_since_keyframe_ > 0 &&
        !keyframe_depth_.empty()) {
      const Eigen::Isometry3d expected_keyframe_T_current =
          keyframe_T_previous_ * estimate;
      const SparseTrackingResult local_orb = sparse_tracker_.match_revisit(
          keyframe_gray_, keyframe_depth_, gray, depth_m,
          intrinsics_, ray_x_, ray_y_, expected_keyframe_T_current);
      if (local_orb.valid && local_orb.inliers >= 12) {
        const Eigen::Isometry3d local_increment =
            inverse_isometry(keyframe_T_previous_) *
            local_orb.previous_T_current;
        const Eigen::Isometry3d disagreement =
            inverse_isometry(estimate) * local_increment;
        const double disagreement_angle =
            Eigen::AngleAxisd(disagreement.linear()).angle();
        if (disagreement.translation().norm() <=
                config_.sparse_max_translation_disagreement_m &&
            std::abs(disagreement_angle) <=
                config_.sparse_max_rotation_disagreement_rad) {
          const Eigen::Vector3d dense_translation = estimate.translation();
          estimate = blend_transform(estimate, local_increment, 0.50);
          // Short-baseline RGB-D translation carries correlated range noise;
          // use ORB here only for the better-observed rotation component.
          estimate.translation() = dense_translation;
        }
      }
    }
    const Eigen::Isometry3d origin_T_current = origin_T_previous_ * estimate;
    std::vector<std::pair<double, size_t>> nearby;
    nearby.reserve(map_keyframes_.size());
    for (size_t index = 0; index < map_keyframes_.size(); ++index) {
      const MapKeyframe &keyframe = map_keyframes_[index];
      if (map_frame_counter_ - keyframe.created_frame <
          config_.map_min_keyframe_age) continue;
      const Eigen::Isometry3d relative =
          inverse_isometry(keyframe.origin_T_keyframe) * origin_T_current;
      const double angle = Eigen::AngleAxisd(relative.linear()).angle();
      const double distance = relative.translation().norm();
      if (distance <= config_.map_query_translation_m &&
          std::abs(angle) <= config_.map_query_rotation_rad) {
        const double age_preference =
            1e-4 * static_cast<double>(keyframe.created_frame);
        nearby.emplace_back(distance + 0.25 * std::abs(angle) + age_preference,
                            index);
      }
    }
    std::sort(nearby.begin(), nearby.end());
    // When the oldest anchor is geometrically plausible, always test it. A
    // drifted pose can make a recent (and itself drifted) keyframe look closer
    // than the true loop origin, which would otherwise starve loop closure.
    std::vector<size_t> candidate_indices;
    candidate_indices.reserve(2U);
    const auto oldest = std::find_if(
        nearby.begin(), nearby.end(),
        [](const std::pair<double, size_t> &candidate) {
          return candidate.second == 0U;
        });
    if (oldest != nearby.end()) candidate_indices.push_back(0U);
    for (const size_t candidate_index : candidate_indices) {
      const MapKeyframe &keyframe = map_keyframes_[candidate_index];
      if (candidate_index == 0U &&
          normalized_image_correlation(keyframe.gray, gray) < 0.60) {
        continue;
      }
      const Eigen::Isometry3d expected_keyframe_T_current =
          inverse_isometry(keyframe.origin_T_keyframe) * origin_T_current;
      const SparseTrackingResult sparse = sparse_tracker_.match_revisit(
          keyframe.gray, keyframe.depth, gray, depth_m,
          intrinsics_, ray_x_, ray_y_, expected_keyframe_T_current);
      result.map_descriptor_matches =
          std::max(result.map_descriptor_matches, sparse.tracked);
      result.map_match_inliers =
          std::max(result.map_match_inliers, sparse.inliers);
      if (!sparse.valid || sparse.inliers < config_.map_min_inliers) continue;
      const Eigen::Isometry3d mapped_origin_T_current =
          keyframe.origin_T_keyframe * sparse.previous_T_current;
      const Eigen::Isometry3d correction =
          inverse_isometry(origin_T_current) * mapped_origin_T_current;
      const double correction_angle =
          Eigen::AngleAxisd(correction.linear()).angle();
      const double correction_translation = correction.translation().norm();
      result.map_correction_m = correction_translation;
      result.map_correction_rad = std::abs(correction_angle);
      if (correction_translation > config_.map_max_correction_translation_m ||
          std::abs(correction_angle) > config_.map_max_correction_rotation_rad) {
        continue;
      }
      const double confidence = std::clamp(
          static_cast<double>(sparse.inliers - config_.map_min_inliers) / 40.0,
          0.1, 1.0);
      double weight = config_.map_fusion_weight * confidence;
      if (correction_translation > 1e-6) {
        weight = std::min(weight, 0.20 / correction_translation);
      }
      if (std::abs(correction_angle) > 1e-6) {
        weight = std::min(weight, 0.10 / std::abs(correction_angle));
      }
      const Eigen::Isometry3d corrected_origin_T_current = blend_transform(
          origin_T_current, mapped_origin_T_current,
          std::clamp(weight, 0.0, static_cast<double>(config_.map_fusion_weight)));
      estimate = inverse_isometry(origin_T_previous_) *
                 corrected_origin_T_current;
      result.map_relocalized = true;
      break;
    }
  }
  const Eigen::AngleAxisd rotation(estimate.linear());
  if (!estimate.matrix().allFinite() ||
      estimate.translation().norm() > config_.max_translation_per_frame_m ||
      std::abs(rotation.angle()) > config_.max_rotation_per_frame_rad ||
      result.rmse_m > config_.max_correspondence_m * 0.7) {
    previous_depth_ = depth_m;
    previous_gray_ = gray;
    keyframe_depth_ = depth_m;
    keyframe_gray_ = gray;
    keyframe_T_previous_.setIdentity();
    frames_since_keyframe_ = 0;
    static_frames_ = 0;
    last_delta_.setIdentity();
    reset_window(depth_m, gray);
    return TrackingResult{};
  }

  // Use a short-lived visual keyframe as a second, independent estimate.
  // This limits frame-to-frame drift without assuming wheeled, aerial, or
  // hand-held motion.  The direct estimate is fused only when it agrees with
  // the adjacent-frame estimate, so a stale or occluded keyframe cannot pull
  // the trajectory into an unrelated local minimum.
  if (config_.local_keyframe_fusion_weight > 0.0F &&
      frames_since_keyframe_ > 0 && !keyframe_depth_.empty()) {
    Eigen::Isometry3d keyframe_estimate = keyframe_T_previous_ * estimate;
    TrackingResult keyframe_result;
    if (track_pair(keyframe_depth_, keyframe_gray_, depth_m, gray,
                   keyframe_estimate, keyframe_result)) {
      const Eigen::Isometry3d from_keyframe =
          inverse_isometry(keyframe_T_previous_) * keyframe_estimate;
      const Eigen::Isometry3d disagreement =
          inverse_isometry(estimate) * from_keyframe;
      const double disagreement_angle =
          Eigen::AngleAxisd(disagreement.linear()).angle();
      if (disagreement.translation().norm() <=
              config_.local_keyframe_consistency_translation_m &&
          std::abs(disagreement_angle) <=
              config_.local_keyframe_consistency_rotation_rad &&
          keyframe_result.rmse_m <= config_.max_correspondence_m * 0.7) {
        estimate = blend_transform(estimate, from_keyframe,
                                   config_.local_keyframe_fusion_weight);
        result.rmse_m = std::max(result.rmse_m, keyframe_result.rmse_m);
        result.correspondences = std::min(result.correspondences,
                                          keyframe_result.correspondences);
        result.information = 0.5 * (result.information +
                                    keyframe_result.information);
      }
    }
  }

  previous_depth_ = depth_m;
  previous_gray_ = gray;
  static_frames_ = 0;
  last_delta_ = estimate;
  origin_T_previous_ = origin_T_previous_ * estimate;
  ++map_frame_counter_;
  keyframe_T_previous_ = keyframe_T_previous_ * estimate;
  for (WindowKeyframe &keyframe : window_keyframes_) {
    keyframe.keyframe_T_previous = keyframe.keyframe_T_previous * estimate;
  }
  ++frames_since_keyframe_;
  const double keyframe_angle =
      Eigen::AngleAxisd(keyframe_T_previous_.linear()).angle();
  if (frames_since_keyframe_ >= config_.local_keyframe_max_frames ||
      keyframe_T_previous_.translation().norm() >=
          config_.local_keyframe_max_translation_m ||
      std::abs(keyframe_angle) >= config_.local_keyframe_max_rotation_rad) {
    keyframe_depth_ = depth_m;
    keyframe_gray_ = gray;
    keyframe_T_previous_.setIdentity();
    frames_since_keyframe_ = 0;
    append_window_keyframe(depth_m, gray);
    append_map_keyframe(depth_m, gray);
  }
  result.valid = true;
  result.previous_T_current = estimate;
  return result;
}

}  // namespace rgbd_odometry
