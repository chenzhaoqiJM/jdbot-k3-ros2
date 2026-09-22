#include "rgbd_odometry/sparse_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace rgbd_odometry {
namespace {

struct PyramidLevel {
  int width{0};
  int height{0};
  std::vector<float> pixels;
};

struct Feature {
  Eigen::Vector2f point{Eigen::Vector2f::Zero()};
  float score{0.0F};
};

float sample(const PyramidLevel &image, float x, float y) {
  const int ix = static_cast<int>(std::floor(x));
  const int iy = static_cast<int>(std::floor(y));
  if (ix < 0 || iy < 0 || ix + 1 >= image.width || iy + 1 >= image.height) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const float ax = x - ix;
  const float ay = y - iy;
  const size_t i = static_cast<size_t>(iy) * image.width + ix;
  return (1.0F - ay) * ((1.0F - ax) * image.pixels[i] +
                        ax * image.pixels[i + 1U]) +
         ay * ((1.0F - ax) * image.pixels[i + image.width] +
               ax * image.pixels[i + image.width + 1U]);
}

std::vector<PyramidLevel> make_pyramid(const std::vector<uint8_t> &gray,
                                       int width, int height, int levels) {
  std::vector<PyramidLevel> pyramid;
  pyramid.reserve(levels);
  PyramidLevel base;
  base.width = width;
  base.height = height;
  base.pixels.resize(gray.size());
  std::transform(gray.begin(), gray.end(), base.pixels.begin(),
                 [](uint8_t value) { return static_cast<float>(value); });
  pyramid.push_back(std::move(base));
  for (int level = 1; level < levels; ++level) {
    const PyramidLevel &source = pyramid.back();
    PyramidLevel destination;
    destination.width = source.width / 2;
    destination.height = source.height / 2;
    destination.pixels.resize(
        static_cast<size_t>(destination.width) * destination.height);
    for (int y = 0; y < destination.height; ++y) {
      for (int x = 0; x < destination.width; ++x) {
        // Separable [1 2 1] low-pass, evaluated at the decimated grid.
        float sum = 0.0F;
        float weight_sum = 0.0F;
        for (int dy = -1; dy <= 1; ++dy) {
          const int sy = std::clamp(2 * y + dy, 0, source.height - 1);
          const float wy = dy == 0 ? 2.0F : 1.0F;
          for (int dx = -1; dx <= 1; ++dx) {
            const int sx = std::clamp(2 * x + dx, 0, source.width - 1);
            const float weight = wy * (dx == 0 ? 2.0F : 1.0F);
            sum += weight * source.pixels[static_cast<size_t>(sy) *
                                          source.width + sx];
            weight_sum += weight;
          }
        }
        destination.pixels[static_cast<size_t>(y) * destination.width + x] =
            sum / weight_sum;
      }
    }
    pyramid.push_back(std::move(destination));
  }
  return pyramid;
}

std::vector<Feature> detect_features(const PyramidLevel &image,
                                     const std::vector<float> &depth,
                                     const SparseTrackerConfig &config) {
  std::vector<Feature> candidates;
  float maximum_score = 0.0F;
  constexpr int border = 10;
  for (int y = border; y + border < image.height; y += 2) {
    for (int x = border; x + border < image.width; x += 2) {
      const size_t center = static_cast<size_t>(y) * image.width + x;
      if (!std::isfinite(depth[center])) continue;
      double xx = 0.0, xy = 0.0, yy = 0.0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const size_t i = static_cast<size_t>(y + dy) * image.width + x + dx;
          const double gx = 0.5 * (image.pixels[i + 1U] - image.pixels[i - 1U]);
          const double gy = 0.5 *
              (image.pixels[i + image.width] - image.pixels[i - image.width]);
          xx += gx * gx;
          xy += gx * gy;
          yy += gy * gy;
        }
      }
      const double trace = xx + yy;
      const double discriminant = std::sqrt(std::max(
          0.0, (xx - yy) * (xx - yy) + 4.0 * xy * xy));
      const float score = static_cast<float>(0.5 * (trace - discriminant));
      if (score <= 1.0F) continue;
      candidates.push_back({Eigen::Vector2f(x, y), score});
      maximum_score = std::max(maximum_score, score);
    }
  }
  const float threshold = config.quality_fraction * maximum_score;
  candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                  [threshold](const Feature &feature) {
                                    return feature.score < threshold;
                                  }),
                   candidates.end());
  std::sort(candidates.begin(), candidates.end(),
            [](const Feature &a, const Feature &b) { return a.score > b.score; });
  std::vector<Feature> selected;
  selected.reserve(config.max_features);
  const float minimum_squared = config.minimum_distance_px *
                                config.minimum_distance_px;
  for (const Feature &candidate : candidates) {
    bool separated = true;
    for (const Feature &kept : selected) {
      if ((candidate.point - kept.point).squaredNorm() < minimum_squared) {
        separated = false;
        break;
      }
    }
    if (separated) selected.push_back(candidate);
    if (static_cast<int>(selected.size()) >= config.max_features) break;
  }
  return selected;
}

bool track_point(const std::vector<PyramidLevel> &reference,
                 const std::vector<PyramidLevel> &target,
                 const Eigen::Vector2f &point, int radius, int iterations,
                 Eigen::Vector2f &tracked) {
  Eigen::Vector2f displacement = Eigen::Vector2f::Zero();
  for (int level = static_cast<int>(reference.size()) - 1; level >= 0; --level) {
    if (level != static_cast<int>(reference.size()) - 1) displacement *= 2.0F;
    const float scale = 1.0F / static_cast<float>(1 << level);
    const Eigen::Vector2f p = point * scale;
    const PyramidLevel &a = reference[level];
    const PyramidLevel &b = target[level];
    for (int iteration = 0; iteration < iterations; ++iteration) {
      const Eigen::Vector2f q = p + displacement;
      if (p.x() < radius + 2 || p.y() < radius + 2 ||
          p.x() + radius + 2 >= a.width || p.y() + radius + 2 >= a.height ||
          q.x() < radius + 2 || q.y() < radius + 2 ||
          q.x() + radius + 2 >= b.width || q.y() + radius + 2 >= b.height) {
        return false;
      }
      double mean_difference = 0.0;
      int samples = 0;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          mean_difference += sample(a, p.x() + dx, p.y() + dy) -
                             sample(b, q.x() + dx, q.y() + dy);
          ++samples;
        }
      }
      mean_difference /= samples;
      Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
      Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
      double squared_error = 0.0;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const float ax = p.x() + dx;
          const float ay = p.y() + dy;
          const float bx = q.x() + dx;
          const float by = q.y() + dy;
          const double gx = 0.5 * (sample(b, bx + 1.0F, by) -
                                    sample(b, bx - 1.0F, by));
          const double gy = 0.5 * (sample(b, bx, by + 1.0F) -
                                    sample(b, bx, by - 1.0F));
          const double residual = sample(a, ax, ay) - sample(b, bx, by) -
                                  mean_difference;
          const Eigen::Vector2d image_gradient(gx, gy);
          hessian.noalias() += image_gradient * image_gradient.transpose();
          gradient.noalias() += image_gradient * residual;
          squared_error += residual * residual;
        }
      }
      if (hessian.determinant() < 1e-3) return false;
      const Eigen::Vector2d update = hessian.ldlt().solve(gradient);
      if (!update.allFinite() || update.norm() > 3.0) return false;
      displacement += update.cast<float>();
      if (update.norm() < 0.03) {
        if (std::sqrt(squared_error / samples) > 28.0) return false;
        break;
      }
    }
  }
  tracked = point + displacement;
  return tracked.allFinite();
}

bool point_from_depth(const std::vector<float> &depth, int width, int height,
                      const std::vector<float> &ray_x,
                      const std::vector<float> &ray_y,
                      const Eigen::Vector2f &pixel,
                      Eigen::Vector3d &point) {
  const int x = static_cast<int>(std::lround(pixel.x()));
  const int y = static_cast<int>(std::lround(pixel.y()));
  if (x < 1 || y < 1 || x + 1 >= width || y + 1 >= height) return false;
  float values[9];
  int count = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const float value = depth[static_cast<size_t>(y + dy) * width + x + dx];
      if (std::isfinite(value)) values[count++] = value;
    }
  }
  if (count < 3) return false;
  std::nth_element(values, values + count / 2, values + count);
  const double z = values[count / 2];
  const int x0 = static_cast<int>(std::floor(pixel.x()));
  const int y0 = static_cast<int>(std::floor(pixel.y()));
  const float ax = pixel.x() - x0;
  const float ay = pixel.y() - y0;
  const size_t i = static_cast<size_t>(y0) * width + x0;
  const auto interpolate = [&](const std::vector<float> &rays) {
    return (1.0F - ay) * ((1.0F - ax) * rays[i] + ax * rays[i + 1U]) +
           ay * ((1.0F - ax) * rays[i + width] +
                 ax * rays[i + width + 1U]);
  };
  point = Eigen::Vector3d(interpolate(ray_x) * z, interpolate(ray_y) * z, z);
  return point.allFinite();
}

bool rigid_fit(const std::vector<Eigen::Vector3d> &source,
               const std::vector<Eigen::Vector3d> &target,
               const std::vector<int> &indices, Eigen::Isometry3d &transform) {
  if (indices.size() < 3U) return false;
  Eigen::Vector3d source_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_mean = Eigen::Vector3d::Zero();
  for (int index : indices) {
    source_mean += source[index];
    target_mean += target[index];
  }
  source_mean /= static_cast<double>(indices.size());
  target_mean /= static_cast<double>(indices.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (int index : indices) {
    covariance.noalias() += (target[index] - target_mean) *
                            (source[index] - source_mean).transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0) {
    correction(2, 2) = -1.0;
  }
  const Eigen::Matrix3d rotation =
      svd.matrixU() * correction * svd.matrixV().transpose();
  if (!rotation.allFinite()) return false;
  transform.setIdentity();
  transform.linear() = rotation;
  transform.translation() = target_mean - rotation * source_mean;
  return transform.matrix().allFinite();
}

uint32_t next_random(uint32_t &state) {
  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}

Eigen::Matrix3d skew3(const Eigen::Vector3d &v) {
  Eigen::Matrix3d matrix;
  matrix << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return matrix;
}

Eigen::Isometry3d small_exp(const Eigen::Matrix<double, 6, 1> &increment) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d rotation = increment.tail<3>();
  const double angle = rotation.norm();
  if (angle > 1e-12) {
    transform.linear() = Eigen::AngleAxisd(angle, rotation / angle).toRotationMatrix();
  }
  transform.translation() = increment.head<3>();
  return transform;
}

bool project_distorted(const Eigen::Vector3d &point, const Intrinsics &intrinsics,
                       Eigen::Vector2d &pixel) {
  if (point.z() <= 0.05 || !point.allFinite()) return false;
  double x = point.x() / point.z();
  double y = point.y() / point.z();
  if (!intrinsics.d.empty()) {
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    double radial = 1.0 + intrinsics.d[0] * r2 + intrinsics.d[1] * r4 +
                    intrinsics.d[4] * r6;
    if (intrinsics.distortion_model == "rational_polynomial") {
      const double denominator = 1.0 + intrinsics.d[5] * r2 +
                                 intrinsics.d[6] * r4 + intrinsics.d[7] * r6;
      if (std::abs(denominator) < 1e-12) return false;
      radial /= denominator;
    }
    const double distorted_x = x * radial + 2.0 * intrinsics.d[2] * x * y +
                               intrinsics.d[3] * (r2 + 2.0 * x * x);
    const double distorted_y = y * radial + intrinsics.d[2] *
                               (r2 + 2.0 * y * y) +
                               2.0 * intrinsics.d[3] * x * y;
    x = distorted_x;
    y = distorted_y;
  }
  pixel = Eigen::Vector2d(intrinsics.fx * x + intrinsics.cx,
                          intrinsics.fy * y + intrinsics.cy);
  return pixel.allFinite();
}

bool refine_reprojection(const std::vector<Eigen::Vector3d> &previous,
                         const std::vector<Eigen::Vector2f> &current_pixels,
                         const std::vector<int> &indices,
                         const Intrinsics &intrinsics,
                         Eigen::Isometry3d &previous_T_current,
                         int minimum_points = 12) {
  Eigen::Isometry3d current_T_previous = previous_T_current.inverse();
  for (int iteration = 0; iteration < 6; ++iteration) {
    Eigen::Matrix<double, 6, 6> hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> gradient =
        Eigen::Matrix<double, 6, 1>::Zero();
    int count = 0;
    for (int index : indices) {
      const Eigen::Vector3d point = current_T_previous * previous[index];
      Eigen::Vector2d predicted;
      if (!project_distorted(point, intrinsics, predicted)) continue;
      const Eigen::Vector2d residual =
          predicted - current_pixels[index].cast<double>();
      const double error = residual.norm();
      if (error > 8.0) continue;
      const double inverse_z = 1.0 / point.z();
      Eigen::Matrix<double, 2, 3> projection;
      projection << intrinsics.fx * inverse_z, 0.0,
          -intrinsics.fx * point.x() * inverse_z * inverse_z,
          0.0, intrinsics.fy * inverse_z,
          -intrinsics.fy * point.y() * inverse_z * inverse_z;
      Eigen::Matrix<double, 3, 6> motion;
      motion.leftCols<3>().setIdentity();
      motion.rightCols<3>() = -skew3(point);
      const Eigen::Matrix<double, 2, 6> jacobian = projection * motion;
      const double weight = error <= 2.0 ? 1.0 : 2.0 / error;
      hessian.noalias() += weight * jacobian.transpose() * jacobian;
      gradient.noalias() += weight * jacobian.transpose() * residual;
      ++count;
    }
    if (count < minimum_points) return false;
    hessian.diagonal().array() += hessian.diagonal().maxCoeff() * 1e-7;
    const Eigen::Matrix<double, 6, 1> update = -hessian.ldlt().solve(gradient);
    if (!update.allFinite() || update.head<3>().norm() > 0.08 ||
        update.tail<3>().norm() > 0.08) return false;
    current_T_previous = small_exp(update) * current_T_previous;
    if (update.norm() < 1e-5) break;
  }
  previous_T_current = current_T_previous.inverse();
  return previous_T_current.matrix().allFinite();
}

SparseTrackingResult ransac_fit(const std::vector<Eigen::Vector3d> &current,
                                const std::vector<Eigen::Vector3d> &previous,
                                const std::vector<Eigen::Vector2f> &current_pixels,
                                const Intrinsics &intrinsics,
                                const SparseTrackerConfig &config,
                                int minimum_inliers = -1,
                                int minimum_ratio_percent = 45) {
  SparseTrackingResult result;
  result.depth_pairs = static_cast<int>(current.size());
  if (minimum_inliers < 0) minimum_inliers = config.minimum_inliers;
  if (result.depth_pairs < minimum_inliers) return result;
  std::vector<int> best_inliers;
  uint32_t random_state = 0x9e3779b9U ^ static_cast<uint32_t>(current.size());
  for (int iteration = 0; iteration < config.ransac_iterations; ++iteration) {
    std::vector<int> sample_indices;
    while (sample_indices.size() < 3U) {
      const int index = static_cast<int>(next_random(random_state) % current.size());
      if (std::find(sample_indices.begin(), sample_indices.end(), index) ==
          sample_indices.end()) sample_indices.push_back(index);
    }
    const Eigen::Vector3d a = current[sample_indices[1]] - current[sample_indices[0]];
    const Eigen::Vector3d b = current[sample_indices[2]] - current[sample_indices[0]];
    if (a.cross(b).norm() < 0.002) continue;
    Eigen::Isometry3d candidate;
    if (!rigid_fit(current, previous, sample_indices, candidate)) continue;
    std::vector<int> inliers;
    for (size_t i = 0; i < current.size(); ++i) {
      if ((candidate * current[i] - previous[i]).norm() <
          config.ransac_threshold_m) {
        inliers.push_back(static_cast<int>(i));
      }
    }
    if (inliers.size() > best_inliers.size()) best_inliers = std::move(inliers);
  }
  if (static_cast<int>(best_inliers.size()) < minimum_inliers ||
      best_inliers.size() * 100U <
          current.size() * static_cast<size_t>(minimum_ratio_percent)) return result;
  if (!rigid_fit(current, previous, best_inliers, result.previous_T_current)) {
    return result;
  }
  // The two-depth fit is an excellent outlier filter and rotation seed, but
  // its translation inherits range noise from both frames.  Refine the same
  // consensus as previous-frame 3D to current-frame 2D reprojection, so the
  // final translation does not depend on current depth.
  if (!refine_reprojection(previous, current_pixels, best_inliers, intrinsics,
                           result.previous_T_current,
                           std::min(12, minimum_inliers))) return result;
  double squared_error = 0.0;
  for (int index : best_inliers) {
    squared_error +=
        (result.previous_T_current * current[index] - previous[index]).squaredNorm();
  }
  result.inliers = static_cast<int>(best_inliers.size());
  result.rmse_m = std::sqrt(squared_error / best_inliers.size());
  result.valid = result.rmse_m < config.ransac_threshold_m * 0.65;
  return result;
}

}  // namespace

SparseTracker::SparseTracker(SparseTrackerConfig config) : config_(config) {
  // Keep the low-rate ORB recovery path inside the same one-core CPU budget as
  // the scalar odometry pipeline. OpenCV otherwise may create worker threads,
  // producing short multi-core utilization spikes on K3.
  cv::setNumThreads(1);
}

SparseTrackingResult SparseTracker::track(
    const std::vector<uint8_t> &previous_gray,
    const std::vector<float> &previous_depth,
    const std::vector<uint8_t> &current_gray,
    const std::vector<float> &current_depth,
    const Intrinsics &intrinsics, const std::vector<float> &ray_x,
    const std::vector<float> &ray_y) const {
  SparseTrackingResult result;
  const size_t expected = static_cast<size_t>(intrinsics.width) * intrinsics.height;
  if (previous_gray.size() != expected || current_gray.size() != expected ||
      previous_depth.size() != expected || current_depth.size() != expected ||
      ray_x.size() != expected || ray_y.size() != expected) {
    return result;
  }
  const auto previous_pyramid = make_pyramid(
      previous_gray, intrinsics.width, intrinsics.height, config_.pyramid_levels);
  const auto current_pyramid = make_pyramid(
      current_gray, intrinsics.width, intrinsics.height, config_.pyramid_levels);
  const std::vector<Feature> features =
      detect_features(previous_pyramid.front(), previous_depth, config_);
  result.detected = static_cast<int>(features.size());
  std::vector<Eigen::Vector3d> previous_points;
  std::vector<Eigen::Vector3d> current_points;
  std::vector<Eigen::Vector2f> current_pixels;
  previous_points.reserve(features.size());
  current_points.reserve(features.size());
  current_pixels.reserve(features.size());
  for (const Feature &feature : features) {
    Eigen::Vector2f current_pixel;
    if (!track_point(previous_pyramid, current_pyramid, feature.point,
                     config_.patch_radius, config_.iterations, current_pixel)) continue;
    Eigen::Vector2f backward_pixel;
    if (!track_point(current_pyramid, previous_pyramid, current_pixel,
                     config_.patch_radius, config_.iterations, backward_pixel) ||
        (backward_pixel - feature.point).norm() >
            config_.forward_backward_max_error_px) continue;
    ++result.tracked;
    Eigen::Vector3d previous_point, current_point;
    if (!point_from_depth(previous_depth, intrinsics.width, intrinsics.height,
                          ray_x, ray_y, feature.point, previous_point) ||
        !point_from_depth(current_depth, intrinsics.width, intrinsics.height,
                          ray_x, ray_y, current_pixel, current_point)) continue;
    previous_points.push_back(previous_point);
    current_points.push_back(current_point);
    current_pixels.push_back(current_pixel);
  }
  SparseTrackingResult fitted = ransac_fit(
      current_points, previous_points, current_pixels, intrinsics, config_);
  fitted.detected = result.detected;
  fitted.tracked = result.tracked;
  return fitted;
}

SparseTrackingResult SparseTracker::match_revisit(
    const std::vector<uint8_t> &previous_gray,
    const std::vector<float> &previous_depth,
    const std::vector<uint8_t> &current_gray,
    const std::vector<float> &current_depth,
    const Intrinsics &intrinsics, const std::vector<float> &ray_x,
    const std::vector<float> &ray_y,
    const Eigen::Isometry3d &expected_previous_T_current) const {
  SparseTrackingResult result;
  const size_t expected = static_cast<size_t>(intrinsics.width) * intrinsics.height;
  if (previous_gray.size() != expected || current_gray.size() != expected ||
      previous_depth.size() != expected || current_depth.size() != expected ||
      ray_x.size() != expected || ray_y.size() != expected) {
    return result;
  }
  cv::Mat previous_image(intrinsics.height, intrinsics.width, CV_8UC1,
                         const_cast<uint8_t *>(previous_gray.data()));
  cv::Mat current_image(intrinsics.height, intrinsics.width, CV_8UC1,
                        const_cast<uint8_t *>(current_gray.data()));
  cv::Mat previous_mask(intrinsics.height, intrinsics.width, CV_8UC1,
                        cv::Scalar(0));
  cv::Mat current_mask(intrinsics.height, intrinsics.width, CV_8UC1,
                       cv::Scalar(0));
  for (int y = 0; y < intrinsics.height; ++y) {
    uint8_t *previous_row = previous_mask.ptr<uint8_t>(y);
    uint8_t *current_row = current_mask.ptr<uint8_t>(y);
    for (int x = 0; x < intrinsics.width; ++x) {
      const size_t index = static_cast<size_t>(y) * intrinsics.width + x;
      previous_row[x] = std::isfinite(previous_depth[index]) ? 255U : 0U;
      current_row[x] = std::isfinite(current_depth[index]) ? 255U : 0U;
    }
  }
  const cv::Ptr<cv::ORB> orb = cv::ORB::create(200);
  std::vector<cv::KeyPoint> previous_features;
  std::vector<cv::KeyPoint> current_features;
  cv::Mat previous_descriptors;
  cv::Mat current_descriptors;
  orb->detectAndCompute(previous_image, previous_mask, previous_features,
                        previous_descriptors);
  orb->detectAndCompute(current_image, current_mask, current_features,
                        current_descriptors);
  result.detected = static_cast<int>(previous_features.size());
  if (previous_descriptors.empty() || current_descriptors.empty()) return result;
  cv::BFMatcher matcher(cv::NORM_HAMMING, true);
  std::vector<cv::DMatch> matches;
  matcher.match(previous_descriptors, current_descriptors, matches);

  std::vector<Eigen::Vector3d> previous_points;
  std::vector<Eigen::Vector3d> current_points;
  std::vector<Eigen::Vector2f> current_pixels;
  previous_points.reserve(matches.size());
  current_points.reserve(matches.size());
  current_pixels.reserve(matches.size());
  for (const cv::DMatch &match : matches) {
    if (match.distance > 55.0F) continue;
    const cv::Point2f previous_pixel_cv =
        previous_features[static_cast<size_t>(match.queryIdx)].pt;
    const cv::Point2f current_pixel_cv =
        current_features[static_cast<size_t>(match.trainIdx)].pt;
    const Eigen::Vector2f previous_pixel(previous_pixel_cv.x, previous_pixel_cv.y);
    const Eigen::Vector2f current_pixel(current_pixel_cv.x, current_pixel_cv.y);
    Eigen::Vector3d previous_point;
    Eigen::Vector3d current_point;
    if (!point_from_depth(previous_depth, intrinsics.width, intrinsics.height,
                          ray_x, ray_y, previous_pixel,
                          previous_point) ||
        !point_from_depth(current_depth, intrinsics.width, intrinsics.height,
                          ray_x, ray_y, current_pixel,
                          current_point)) {
      continue;
    }
    Eigen::Vector2d expected_pixel;
    if (!project_distorted(expected_previous_T_current.inverse() * previous_point,
                           intrinsics, expected_pixel) ||
        (current_pixel.cast<double>() - expected_pixel).norm() > 220.0) {
      continue;
    }
    ++result.tracked;
    previous_points.push_back(previous_point);
    current_points.push_back(current_point);
    current_pixels.push_back(current_pixel);
  }
  SparseTrackingResult fitted = ransac_fit(
      current_points, previous_points, current_pixels, intrinsics, config_, 8, 25);
  fitted.detected = result.detected;
  fitted.tracked = result.tracked;
  return fitted;
}

}  // namespace rgbd_odometry
