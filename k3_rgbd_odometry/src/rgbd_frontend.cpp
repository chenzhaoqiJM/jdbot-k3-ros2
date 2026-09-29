#include "k3_rgbd_odometry/rgbd_frontend.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace k3_rgbd_odometry {
namespace {

float bilinear(const RgbdFrontend::GrayImage& image, float x, float y) {
  const int ix = static_cast<int>(std::floor(x));
  const int iy = static_cast<int>(std::floor(y));
  if (ix < 0 || iy < 0 || ix + 1 >= image.width || iy + 1 >= image.height) {
    return 0.0F;
  }
  const float ax = x - static_cast<float>(ix);
  const float ay = y - static_cast<float>(iy);
  const float v00 = image.at(ix, iy);
  const float v10 = image.at(ix + 1, iy);
  const float v01 = image.at(ix, iy + 1);
  const float v11 = image.at(ix + 1, iy + 1);
  return (1.0F - ay) * ((1.0F - ax) * v00 + ax * v10) +
         ay * ((1.0F - ax) * v01 + ax * v11);
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Isometry3d se3Exp(const Eigen::Matrix<double, 6, 1>& xi) {
  const Eigen::Vector3d translation = xi.head<3>();
  const Eigen::Vector3d rotation = xi.tail<3>();
  const double angle = rotation.norm();
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d v = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d w = skew(rotation);
  if (angle < 1e-8) {
    r += w;
    v += 0.5 * w;
  } else {
    const double a = std::sin(angle) / angle;
    const double b = (1.0 - std::cos(angle)) / (angle * angle);
    const double c = (angle - std::sin(angle)) / (angle * angle * angle);
    r += a * w + b * w * w;
    v += b * w + c * w * w;
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = r;
  result.translation() = v * translation;
  return result;
}

bool rigidFit(const std::vector<RgbdFrontend::Match>& matches, const std::vector<int>& indices,
              Eigen::Isometry3d& transform) {
  if (indices.size() < 3) return false;
  Eigen::Vector3d previous_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_mean = Eigen::Vector3d::Zero();
  for (int index : indices) {
    previous_mean += matches[static_cast<size_t>(index)].previous_point;
    current_mean += matches[static_cast<size_t>(index)].current_point;
  }
  previous_mean /= static_cast<double>(indices.size());
  current_mean /= static_cast<double>(indices.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (int index : indices) {
    const auto& match = matches[static_cast<size_t>(index)];
    covariance += (match.current_point - current_mean) *
                  (match.previous_point - previous_mean).transpose();
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance,
                                               Eigen::ComputeFullU | Eigen::ComputeFullV);
  if (svd.singularValues()(1) < 1e-8) return false;
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  const Eigen::Matrix3d rotation = svd.matrixU() * correction * svd.matrixV().transpose();
  transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation;
  transform.translation() = current_mean - rotation * previous_mean;
  return transform.matrix().allFinite();
}

}  // namespace

RgbdFrontend::RgbdFrontend(FrontendConfig config) : config_(std::move(config)) {}

void RgbdFrontend::setBaseFromCamera(const Eigen::Isometry3d& base_from_camera) {
  base_from_camera_ = base_from_camera;
  have_base_from_camera_ = true;
}

void RgbdFrontend::reset() {
  previous_pyramid_.clear();
  previous_depth_.clear();
  previous_camera_ = {};
  reference_to_previous_ = Eigen::Isometry3d::Identity();
  frames_since_reference_ = 0;
  processed_frames_ = 0;
  cumulative_visual_distance_ = 0.0;
  local_window_active_ = true;
}

std::vector<RgbdFrontend::GrayImage> RgbdFrontend::makePyramid(
    const uint8_t* color, int step, const std::string& encoding,
    const CameraModel& camera) const {
  std::vector<GrayImage> pyramid;
  pyramid.reserve(static_cast<size_t>(config_.pyramid_levels));
  GrayImage base;
  base.width = camera.width;
  base.height = camera.height;
  base.pixels.resize(static_cast<size_t>(base.width * base.height));
  const bool bgr = encoding == "bgr8";
  const bool mono = encoding == "mono8" || encoding == "8UC1";
  for (int y = 0; y < base.height; ++y) {
    const uint8_t* row = color + static_cast<size_t>(y * step);
    for (int x = 0; x < base.width; ++x) {
      float gray = 0.0F;
      if (mono) {
        gray = row[x];
      } else {
        const uint8_t* p = row + 3 * x;
        const float red = bgr ? p[2] : p[0];
        const float green = p[1];
        const float blue = bgr ? p[0] : p[2];
        gray = 0.299F * red + 0.587F * green + 0.114F * blue;
      }
      base.pixels[static_cast<size_t>(y * base.width + x)] = gray;
    }
  }
  pyramid.push_back(std::move(base));
  for (int level = 1; level < config_.pyramid_levels; ++level) {
    const GrayImage& source = pyramid.back();
    if (source.width < 40 || source.height < 30) break;
    GrayImage down;
    down.width = source.width / 2;
    down.height = source.height / 2;
    down.pixels.resize(static_cast<size_t>(down.width * down.height));
    for (int y = 0; y < down.height; ++y) {
      for (int x = 0; x < down.width; ++x) {
        down.pixels[static_cast<size_t>(y * down.width + x)] =
            0.25F * (source.at(2 * x, 2 * y) + source.at(2 * x + 1, 2 * y) +
                     source.at(2 * x, 2 * y + 1) + source.at(2 * x + 1, 2 * y + 1));
      }
    }
    pyramid.push_back(std::move(down));
  }
  return pyramid;
}

float RgbdFrontend::sampleDepth(const uint16_t* depth, int stride, int width, int height,
                                const Eigen::Vector2f& pixel) const {
  const int x = static_cast<int>(std::lround(pixel.x()));
  const int y = static_cast<int>(std::lround(pixel.y()));
  if (x < 1 || y < 1 || x + 1 >= width || y + 1 >= height) return 0.0F;
  std::array<uint16_t, 9> values{};
  int count = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const uint16_t raw = depth[(y + dy) * stride + x + dx];
      if (raw != 0) values[static_cast<size_t>(count++)] = raw;
    }
  }
  if (count < 3) return 0.0F;
  std::nth_element(values.begin(), values.begin() + count / 2, values.begin() + count);
  const float meters = static_cast<float>(values[static_cast<size_t>(count / 2)]) *
                       config_.depth_scale;
  return meters >= config_.min_depth_m && meters <= config_.max_depth_m ? meters : 0.0F;
}

std::vector<RgbdFrontend::Feature> RgbdFrontend::detectFeatures(
    const GrayImage& image, const uint16_t* depth, int depth_stride,
    const CameraModel& camera) const {
  std::vector<Feature> result;
  const int border = 8;
  for (int cell_y = border; cell_y < image.height - border; cell_y += config_.cell_size) {
    for (int cell_x = border; cell_x < image.width - border; cell_x += config_.cell_size) {
      Feature best;
      for (int y = cell_y; y < std::min(cell_y + config_.cell_size, image.height - border);
           y += 2) {
        for (int x = cell_x; x < std::min(cell_x + config_.cell_size, image.width - border);
             x += 2) {
          float gxx = 0.0F, gxy = 0.0F, gyy = 0.0F;
          for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
              const float gx = 0.5F * (image.at(x + dx + 1, y + dy) -
                                       image.at(x + dx - 1, y + dy));
              const float gy = 0.5F * (image.at(x + dx, y + dy + 1) -
                                       image.at(x + dx, y + dy - 1));
              gxx += gx * gx;
              gxy += gx * gy;
              gyy += gy * gy;
            }
          }
          const float trace = gxx + gyy;
          const float discriminant = std::sqrt(std::max(0.0F,
              (gxx - gyy) * (gxx - gyy) + 4.0F * gxy * gxy));
          const float score = 0.5F * (trace - discriminant);
          if (score > best.score &&
              sampleDepth(depth, depth_stride, camera.width, camera.height,
                          Eigen::Vector2f(static_cast<float>(x), static_cast<float>(y))) > 0.0F) {
            best.pixel = Eigen::Vector2f(static_cast<float>(x), static_cast<float>(y));
            best.score = score;
          }
        }
      }
      if (best.score >= config_.min_corner_score) result.push_back(best);
    }
  }
  if (static_cast<int>(result.size()) > config_.max_features) {
    std::nth_element(result.begin(), result.begin() + config_.max_features, result.end(),
                     [](const Feature& a, const Feature& b) { return a.score > b.score; });
    result.resize(static_cast<size_t>(config_.max_features));
  }
  return result;
}

bool RgbdFrontend::trackFeature(const std::vector<GrayImage>& previous,
                                const std::vector<GrayImage>& current,
                                const Eigen::Vector2f& point, Eigen::Vector2f& tracked,
                                float& patch_rmse) const {
  const int top = static_cast<int>(std::min(previous.size(), current.size())) - 1;
  Eigen::Vector2f q = point / static_cast<float>(1 << top);
  for (int level = top; level >= 0; --level) {
    const auto& reference = previous[static_cast<size_t>(level)];
    const auto& target = current[static_cast<size_t>(level)];
    const float scale = static_cast<float>(1 << level);
    const Eigen::Vector2f p = point / scale;
    if (level != top) q *= 2.0F;
    const int margin = config_.patch_radius + 2;
    if (level == top) {
      float best_error = std::numeric_limits<float>::max();
      Eigen::Vector2f best = q;
      for (int sy = -5; sy <= 5; ++sy) {
        for (int sx = -5; sx <= 5; ++sx) {
          const Eigen::Vector2f candidate = p + Eigen::Vector2f(sx, sy);
          if (candidate.x() < margin || candidate.y() < margin ||
              candidate.x() >= target.width - margin || candidate.y() >= target.height - margin ||
              p.x() < margin || p.y() < margin || p.x() >= reference.width - margin ||
              p.y() >= reference.height - margin) continue;
          float error = 0.0F;
          for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
              const float difference = bilinear(target, candidate.x() + dx, candidate.y() + dy) -
                                       bilinear(reference, p.x() + dx, p.y() + dy);
              error += difference * difference;
            }
          }
          if (error < best_error) {
            best_error = error;
            best = candidate;
          }
        }
      }
      q = best;
    }
    for (int iteration = 0; iteration < config_.klt_iterations; ++iteration) {
      if (p.x() < margin || p.y() < margin || p.x() >= reference.width - margin ||
          p.y() >= reference.height - margin || q.x() < margin || q.y() < margin ||
          q.x() >= target.width - margin || q.y() >= target.height - margin) return false;
      Eigen::Matrix2f hessian = Eigen::Matrix2f::Zero();
      Eigen::Vector2f gradient = Eigen::Vector2f::Zero();
      for (int dy = -config_.patch_radius; dy <= config_.patch_radius; ++dy) {
        for (int dx = -config_.patch_radius; dx <= config_.patch_radius; ++dx) {
          const float tx = q.x() + dx;
          const float ty = q.y() + dy;
          const float residual = bilinear(target, tx, ty) -
                                 bilinear(reference, p.x() + dx, p.y() + dy);
          const Eigen::Vector2f jacobian(0.5F * (bilinear(target, tx + 1.0F, ty) -
                                                 bilinear(target, tx - 1.0F, ty)),
                                          0.5F * (bilinear(target, tx, ty + 1.0F) -
                                                 bilinear(target, tx, ty - 1.0F)));
          hessian.noalias() += jacobian * jacobian.transpose();
          gradient.noalias() += jacobian * residual;
        }
      }
      if (hessian.determinant() < 1e-3F) return false;
      const Eigen::Vector2f update = -hessian.ldlt().solve(gradient);
      if (!update.allFinite() || update.norm() > config_.max_klt_step) return false;
      q += update;
      if (update.squaredNorm() < 1e-4F) break;
    }
  }
  if (q.x() < 8.0F || q.y() < 8.0F || q.x() >= current[0].width - 8.0F ||
      q.y() >= current[0].height - 8.0F) return false;
  double squared_error = 0.0;
  int count = 0;
  for (int dy = -config_.patch_radius; dy <= config_.patch_radius; ++dy) {
    for (int dx = -config_.patch_radius; dx <= config_.patch_radius; ++dx) {
      const float error = bilinear(current[0], q.x() + dx, q.y() + dy) -
                          bilinear(previous[0], point.x() + dx, point.y() + dy);
      squared_error += error * error;
      ++count;
    }
  }
  patch_rmse = static_cast<float>(std::sqrt(squared_error / count));
  tracked = q;
  return patch_rmse <= config_.max_patch_rmse;
}

Eigen::Vector3d RgbdFrontend::unproject(const Eigen::Vector2f& pixel, float depth,
                                        const CameraModel& camera) const {
  return Eigen::Vector3d((pixel.x() - camera.cx) * depth / camera.fx,
                         (pixel.y() - camera.cy) * depth / camera.fy, depth);
}

bool RgbdFrontend::estimateRigid(const std::vector<Match>& matches,
                                 Eigen::Isometry3d& transform,
                                 std::vector<int>& inliers) {
  std::vector<int> depth_indices;
  for (size_t i = 0; i < matches.size(); ++i) {
    if (matches[i].has_current_depth) depth_indices.push_back(static_cast<int>(i));
  }
  if (depth_indices.size() < static_cast<size_t>(config_.min_features)) return false;
  std::uniform_int_distribution<size_t> distribution(0, depth_indices.size() - 1);
  std::vector<int> best;
  Eigen::Isometry3d best_transform = Eigen::Isometry3d::Identity();
  for (int iteration = 0; iteration < config_.ransac_iterations; ++iteration) {
    std::vector<int> sample;
    while (sample.size() < 3) {
      const int index = depth_indices[distribution(random_)];
      if (std::find(sample.begin(), sample.end(), index) == sample.end()) sample.push_back(index);
    }
    Eigen::Isometry3d candidate;
    if (!rigidFit(matches, sample, candidate)) continue;
    std::vector<int> candidate_inliers;
    for (int index : depth_indices) {
      const auto& match = matches[static_cast<size_t>(index)];
      const Eigen::Vector3d error = candidate * match.previous_point - match.current_point;
      const double threshold = config_.ransac_threshold_m + 0.008 * match.previous_point.z();
      if (error.norm() < threshold) candidate_inliers.push_back(index);
    }
    if (candidate_inliers.size() > best.size()) {
      best = std::move(candidate_inliers);
      best_transform = candidate;
      if (best.size() > depth_indices.size() * 4 / 5) break;
    }
  }
  if (best.size() < static_cast<size_t>(config_.min_features)) return false;
  if (!rigidFit(matches, best, transform)) transform = best_transform;
  inliers = std::move(best);
  return true;
}

bool RgbdFrontend::refineReprojection(const std::vector<Match>& matches,
                                      Eigen::Isometry3d& transform,
                                      const CameraModel& camera,
                                      std::vector<int>& inliers, double& rmse) const {
  for (int iteration = 0; iteration < 7; ++iteration) {
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
    int used = 0;
    for (const auto& match : matches) {
      const Eigen::Vector3d point = transform * match.previous_point;
      if (point.z() < config_.min_depth_m) continue;
      const Eigen::Vector2d projection(camera.fx * point.x() / point.z() + camera.cx,
                                       camera.fy * point.y() / point.z() + camera.cy);
      const Eigen::Vector2d residual = projection - match.current_pixel.cast<double>();
      const double norm = residual.norm();
      if (norm > 12.0) continue;
      const double weight = norm <= 2.5 ? 1.0 : 2.5 / norm;
      Eigen::Matrix<double, 2, 3> projection_jacobian;
      const double inverse_z = 1.0 / point.z();
      projection_jacobian << camera.fx * inverse_z, 0.0,
          -camera.fx * point.x() * inverse_z * inverse_z,
          0.0, camera.fy * inverse_z,
          -camera.fy * point.y() * inverse_z * inverse_z;
      Eigen::Matrix<double, 3, 6> motion_jacobian;
      motion_jacobian.leftCols<3>().setIdentity();
      motion_jacobian.rightCols<3>() = -skew(point);
      const Eigen::Matrix<double, 2, 6> jacobian = projection_jacobian * motion_jacobian;
      hessian.noalias() += weight * jacobian.transpose() * jacobian;
      gradient.noalias() += weight * jacobian.transpose() * residual;
      ++used;
    }
    if (used < config_.min_features || hessian.ldlt().info() != Eigen::Success) return false;
    const Eigen::Matrix<double, 6, 1> update = -hessian.ldlt().solve(gradient);
    if (!update.allFinite() || update.norm() > 0.5) return false;
    transform = se3Exp(update) * transform;
    if (update.norm() < 1e-6) break;
  }
  inliers.clear();
  double squared_error = 0.0;
  for (size_t i = 0; i < matches.size(); ++i) {
    const Eigen::Vector3d point = transform * matches[i].previous_point;
    if (point.z() <= 0.0) continue;
    const Eigen::Vector2d projection(camera.fx * point.x() / point.z() + camera.cx,
                                     camera.fy * point.y() / point.z() + camera.cy);
    const double error = (projection - matches[i].current_pixel.cast<double>()).norm();
    if (error <= config_.reprojection_threshold_px) {
      inliers.push_back(static_cast<int>(i));
      squared_error += error * error;
    }
  }
  if (inliers.size() < static_cast<size_t>(config_.min_features)) return false;
  rmse = std::sqrt(squared_error / static_cast<double>(inliers.size()));
  return true;
}

bool RgbdFrontend::refinePlanar(const std::vector<Match>& matches,
                                Eigen::Isometry3d& transform,
                                const CameraModel& camera,
                                std::vector<int>& inliers, double& rmse) const {
  const Eigen::Isometry3d initial_delta =
      base_from_camera_ * transform.inverse() * base_from_camera_.inverse();
  Eigen::Vector3d state(initial_delta.translation().x(), initial_delta.translation().y(),
                        std::atan2(initial_delta.rotation()(1, 0),
                                   initial_delta.rotation()(0, 0)));
  auto camera_transform = [this](const Eigen::Vector3d& value) {
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.translation() = Eigen::Vector3d(value.x(), value.y(), 0.0);
    delta.linear() = Eigen::AngleAxisd(value.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return base_from_camera_.inverse() * delta.inverse() * base_from_camera_;
  };
  auto project = [&camera](const Eigen::Isometry3d& motion, const Eigen::Vector3d& point) {
    const Eigen::Vector3d p = motion * point;
    return Eigen::Vector2d(camera.fx * p.x() / p.z() + camera.cx,
                           camera.fy * p.y() / p.z() + camera.cy);
  };
  for (int iteration = 0; iteration < 8; ++iteration) {
    const Eigen::Isometry3d motion = camera_transform(state);
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    int used = 0;
    for (const auto& match : matches) {
      const Eigen::Vector3d point = motion * match.previous_point;
      if (point.z() <= config_.min_depth_m) continue;
      const Eigen::Vector2d residual = project(motion, match.previous_point) -
                                       match.current_pixel.cast<double>();
      const double norm = residual.norm();
      if (norm > 10.0) continue;
      Eigen::Matrix<double, 2, 3> jacobian;
      for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d perturbed = state;
        const double epsilon = 1e-5;
        perturbed(axis) += epsilon;
        jacobian.col(axis) =
            (project(camera_transform(perturbed), match.previous_point) -
             project(motion, match.previous_point)) / epsilon;
      }
      const double weight = norm <= 2.0 ? 1.0 : 2.0 / norm;
      hessian.noalias() += weight * jacobian.transpose() * jacobian;
      gradient.noalias() += weight * jacobian.transpose() * residual;
      ++used;
    }
    if (used < config_.min_features || std::abs(hessian.determinant()) < 1e-9) return false;
    const Eigen::Vector3d update = -hessian.ldlt().solve(gradient);
    if (!update.allFinite() || update.head<2>().norm() > 0.25 || std::abs(update.z()) > 0.25) {
      return false;
    }
    state += update;
    if (update.norm() < 1e-7) break;
  }
  transform = camera_transform(state);
  inliers.clear();
  double squared_error = 0.0;
  for (size_t i = 0; i < matches.size(); ++i) {
    const Eigen::Vector3d point = transform * matches[i].previous_point;
    if (point.z() <= 0.0) continue;
    const double error = (project(transform, matches[i].previous_point) -
                          matches[i].current_pixel.cast<double>()).norm();
    if (error <= config_.reprojection_threshold_px) {
      inliers.push_back(static_cast<int>(i));
      squared_error += error * error;
    }
  }
  if (inliers.size() < static_cast<size_t>(config_.min_features)) return false;
  rmse = std::sqrt(squared_error / static_cast<double>(inliers.size()));
  return true;
}

bool RgbdFrontend::refineDepthPlanar(const uint16_t* current_depth, int current_stride,
                                     Eigen::Isometry3d& transform,
                                     const CameraModel& camera) const {
  const Eigen::Isometry3d initial_delta =
      base_from_camera_ * transform.inverse() * base_from_camera_.inverse();
  Eigen::Vector3d state(initial_delta.translation().x(), initial_delta.translation().y(),
                        std::atan2(initial_delta.rotation()(1, 0),
                                   initial_delta.rotation()(0, 0)));
  auto camera_transform = [this](const Eigen::Vector3d& value) {
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.translation() = Eigen::Vector3d(value.x(), value.y(), 0.0);
    delta.linear() = Eigen::AngleAxisd(value.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return base_from_camera_.inverse() * delta.inverse() * base_from_camera_;
  };
  auto observed_point = [this, current_depth, current_stride, &camera](float x, float y) {
    const Eigen::Vector2f pixel(x, y);
    const float z = sampleDepth(current_depth, current_stride, camera.width, camera.height, pixel);
    return z > 0.0F ? unproject(pixel, z, camera) : Eigen::Vector3d::Zero();
  };
  int final_used = 0;
  for (int iteration = 0; iteration < 7; ++iteration) {
    const Eigen::Isometry3d motion = camera_transform(state);
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    int used = 0;
    for (int y = 10; y < camera.height - 10; y += 7) {
      for (int x = 10; x < camera.width - 10; x += 7) {
        const Eigen::Vector2f previous_pixel(static_cast<float>(x), static_cast<float>(y));
        const float previous_z = sampleDepth(previous_depth_.data(), previous_camera_.width,
                                             previous_camera_.width, previous_camera_.height,
                                             previous_pixel);
        if (previous_z <= 0.0F) continue;
        const Eigen::Vector3d previous_point = unproject(previous_pixel, previous_z, camera);
        const Eigen::Vector3d predicted = motion * previous_point;
        if (predicted.z() <= config_.min_depth_m) continue;
        const float u = camera.fx * static_cast<float>(predicted.x() / predicted.z()) + camera.cx;
        const float v = camera.fy * static_cast<float>(predicted.y() / predicted.z()) + camera.cy;
        if (u < 5.0F || v < 5.0F || u >= camera.width - 5.0F || v >= camera.height - 5.0F) {
          continue;
        }
        const Eigen::Vector3d observed = observed_point(u, v);
        const Eigen::Vector3d left = observed_point(u - 2.0F, v);
        const Eigen::Vector3d right = observed_point(u + 2.0F, v);
        const Eigen::Vector3d up = observed_point(u, v - 2.0F);
        const Eigen::Vector3d down = observed_point(u, v + 2.0F);
        if (observed.z() <= 0.0 || left.z() <= 0.0 || right.z() <= 0.0 ||
            up.z() <= 0.0 || down.z() <= 0.0) continue;
        if (std::abs(predicted.z() - observed.z()) > 0.18 ||
            std::abs(left.z() - right.z()) > 0.12 || std::abs(up.z() - down.z()) > 0.12) {
          continue;
        }
        const Eigen::Vector3d residual = predicted - observed;
        const double residual_norm = residual.norm();
        if (residual_norm > 0.16) continue;
        Eigen::Matrix3d jacobian;
        for (int axis = 0; axis < 3; ++axis) {
          Eigen::Vector3d perturbed = state;
          const double epsilon = 1e-5;
          perturbed(axis) += epsilon;
          const Eigen::Vector3d changed = camera_transform(perturbed) * previous_point;
          jacobian.col(axis) = (changed - predicted) / epsilon;
        }
        const double weight = residual_norm <= 0.04 ? 1.0 : 0.04 / residual_norm;
        hessian.noalias() += weight * jacobian.transpose() * jacobian;
        gradient.noalias() += weight * jacobian.transpose() * residual;
        ++used;
      }
    }
    final_used = used;
    if (used < 180 || std::abs(hessian.determinant()) < 1e-10) return false;
    const Eigen::Vector3d update = -hessian.ldlt().solve(gradient);
    if (!update.allFinite() || update.head<2>().norm() > 0.15 || std::abs(update.z()) > 0.15) {
      return false;
    }
    state += update;
    if (update.norm() < 2e-6) break;
  }
  if (final_used < 180) return false;
  transform = camera_transform(state);
  return true;
}

bool RgbdFrontend::refinePhotometricPlanar(const std::vector<GrayImage>& current,
                                           Eigen::Isometry3d& transform,
                                           const CameraModel& camera) const {
  const Eigen::Isometry3d initial_delta =
      base_from_camera_ * transform.inverse() * base_from_camera_.inverse();
  Eigen::Vector3d state(initial_delta.translation().x(), initial_delta.translation().y(),
                        std::atan2(initial_delta.rotation()(1, 0),
                                   initial_delta.rotation()(0, 0)));
  auto camera_transform = [this](const Eigen::Vector3d& value) {
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.translation() = Eigen::Vector3d(value.x(), value.y(), 0.0);
    delta.linear() = Eigen::AngleAxisd(value.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return base_from_camera_.inverse() * delta.inverse() * base_from_camera_;
  };
  const int top = static_cast<int>(std::min(previous_pyramid_.size(), current.size())) - 1;
  for (int level = top; level >= 0; --level) {
    const GrayImage& reference = previous_pyramid_[static_cast<size_t>(level)];
    const GrayImage& target = current[static_cast<size_t>(level)];
    const float scale = static_cast<float>(1 << level);
    const int sampling = level >= 2 ? 2 : (level == 1 ? 3 : 5);
    for (int iteration = 0; iteration < 6; ++iteration) {
      const Eigen::Isometry3d motion = camera_transform(state);
      Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      int used = 0;
      for (int y = 5; y < reference.height - 5; y += sampling) {
        for (int x = 5; x < reference.width - 5; x += sampling) {
          const float gx_ref = 0.5F * (reference.at(x + 1, y) - reference.at(x - 1, y));
          const float gy_ref = 0.5F * (reference.at(x, y + 1) - reference.at(x, y - 1));
          if (gx_ref * gx_ref + gy_ref * gy_ref < 36.0F) continue;
          const Eigen::Vector2f full_pixel((x + 0.5F) * scale - 0.5F,
                                           (y + 0.5F) * scale - 0.5F);
          const float z = sampleDepth(previous_depth_.data(), previous_camera_.width,
                                      previous_camera_.width, previous_camera_.height,
                                      full_pixel);
          if (z <= 0.0F) continue;
          const Eigen::Vector3d point = unproject(full_pixel, z, camera);
          const Eigen::Vector3d changed = motion * point;
          if (changed.z() <= config_.min_depth_m) continue;
          const Eigen::Vector2d projected_full(
              camera.fx * changed.x() / changed.z() + camera.cx,
              camera.fy * changed.y() / changed.z() + camera.cy);
          const Eigen::Vector2f projected(
              static_cast<float>((projected_full.x() + 0.5) / scale - 0.5),
              static_cast<float>((projected_full.y() + 0.5) / scale - 0.5));
          if (projected.x() < 3.0F || projected.y() < 3.0F ||
              projected.x() >= target.width - 3.0F || projected.y() >= target.height - 3.0F) {
            continue;
          }
          const double residual = bilinear(target, projected.x(), projected.y()) -
                                  reference.at(x, y);
          if (std::abs(residual) > 55.0) continue;
          const Eigen::Vector2d image_gradient(
              0.5 * (bilinear(target, projected.x() + 1.0F, projected.y()) -
                     bilinear(target, projected.x() - 1.0F, projected.y())),
              0.5 * (bilinear(target, projected.x(), projected.y() + 1.0F) -
                     bilinear(target, projected.x(), projected.y() - 1.0F)));
          Eigen::Vector3d jacobian;
          for (int axis = 0; axis < 3; ++axis) {
            Eigen::Vector3d perturbed = state;
            const double epsilon = 1e-5;
            perturbed(axis) += epsilon;
            const Eigen::Vector3d p = camera_transform(perturbed) * point;
            if (p.z() <= 0.0) { jacobian(axis) = 0.0; continue; }
            const Eigen::Vector2d uv(camera.fx * p.x() / p.z() + camera.cx,
                                     camera.fy * p.y() / p.z() + camera.cy);
            jacobian(axis) = image_gradient.dot((uv - projected_full) / (epsilon * scale));
          }
          const double absolute = std::abs(residual);
          const double weight = absolute <= 12.0 ? 1.0 : 12.0 / absolute;
          hessian.noalias() += weight * jacobian * jacobian.transpose();
          gradient.noalias() += weight * jacobian * residual;
          ++used;
        }
      }
      if (used < 100 || std::abs(hessian.determinant()) < 1e-9) return false;
      const Eigen::Vector3d update = -hessian.ldlt().solve(gradient);
      if (!update.allFinite() || update.head<2>().norm() > 0.15 ||
          std::abs(update.z()) > 0.15) return false;
      state += update;
      if (update.norm() < 1e-6) break;
    }
  }
  transform = camera_transform(state);
  return true;
}

bool RgbdFrontend::refineScanPlanar(const uint16_t* current_depth, int current_stride,
                                    Eigen::Isometry3d& transform,
                                    const CameraModel& camera) const {
  struct ScanPoint {
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    Eigen::Vector2d normal = Eigen::Vector2d::Zero();
    int column = 0;
  };
  auto make_scan = [this, &camera](const uint16_t* depth, int stride) {
    std::vector<ScanPoint> scan;
    for (int x = 4; x < camera.width - 4; x += 3) {
      double best_range = std::numeric_limits<double>::max();
      Eigen::Vector2d best = Eigen::Vector2d::Zero();
      for (int y = 35; y < camera.height - 5; y += 4) {
        const Eigen::Vector2f pixel(static_cast<float>(x), static_cast<float>(y));
        const float z = sampleDepth(depth, stride, camera.width, camera.height, pixel);
        if (z <= 0.0F) continue;
        const Eigen::Vector3d camera_point = unproject(pixel, z, camera);
        const Eigen::Vector3d base_point = base_from_camera_ * camera_point;
        const double range = std::hypot(base_point.x(), base_point.y());
        if (base_point.z() < 0.04 || base_point.z() > 1.35 || range < 0.3 || range > 5.5) {
          continue;
        }
        if (range < best_range) {
          best_range = range;
          best = base_point.head<2>();
        }
      }
      if (best_range < std::numeric_limits<double>::max()) {
        ScanPoint point;
        point.point = best;
        point.column = x;
        scan.push_back(point);
      }
    }
    for (size_t i = 1; i + 1 < scan.size(); ++i) {
      if (scan[i + 1].column - scan[i - 1].column > 12) continue;
      const Eigen::Vector2d tangent = scan[i + 1].point - scan[i - 1].point;
      if (tangent.norm() < 0.015 || tangent.norm() > 0.45) continue;
      scan[i].normal = Eigen::Vector2d(-tangent.y(), tangent.x()).normalized();
    }
    return scan;
  };
  const auto previous_scan = make_scan(previous_depth_.data(), previous_camera_.width);
  const auto current_scan = make_scan(current_depth, current_stride);
  if (previous_scan.size() < 45 || current_scan.size() < 45) return false;

  const Eigen::Isometry3d initial_delta =
      base_from_camera_ * transform.inverse() * base_from_camera_.inverse();
  Eigen::Vector3d state(initial_delta.translation().x(), initial_delta.translation().y(),
                        std::atan2(initial_delta.rotation()(1, 0),
                                   initial_delta.rotation()(0, 0)));
  auto current_from_previous = [](const Eigen::Vector3d& value,
                                  const Eigen::Vector2d& point) {
    const double c = std::cos(value.z());
    const double s = std::sin(value.z());
    Eigen::Matrix2d rotation = Eigen::Matrix2d::Zero();
    rotation << c, -s, s, c;
    return rotation.transpose() * (point - value.head<2>());
  };
  int final_pairs = 0;
  for (int iteration = 0; iteration < 10; ++iteration) {
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    int pairs = 0;
    for (const auto& source : previous_scan) {
      const Eigen::Vector2d predicted = current_from_previous(state, source.point);
      const ScanPoint* nearest = nullptr;
      double nearest_squared = 0.20 * 0.20;
      for (const auto& candidate : current_scan) {
        if (candidate.normal.squaredNorm() < 0.5) continue;
        const double squared = (candidate.point - predicted).squaredNorm();
        if (squared < nearest_squared) {
          nearest_squared = squared;
          nearest = &candidate;
        }
      }
      if (!nearest) continue;
      const double residual = nearest->normal.dot(predicted - nearest->point);
      if (std::abs(residual) > 0.08) continue;
      Eigen::Vector3d jacobian;
      for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d perturbed = state;
        const double epsilon = 1e-5;
        perturbed(axis) += epsilon;
        const Eigen::Vector2d changed = current_from_previous(perturbed, source.point);
        jacobian(axis) = nearest->normal.dot(changed - predicted) / epsilon;
      }
      const double weight = std::abs(residual) <= 0.025 ? 1.0 : 0.025 / std::abs(residual);
      hessian.noalias() += weight * jacobian * jacobian.transpose();
      gradient.noalias() += weight * jacobian * residual;
      ++pairs;
    }
    final_pairs = pairs;
    if (pairs < 35 || std::abs(hessian.determinant()) < 1e-10) return false;
    const Eigen::Vector3d update = -hessian.ldlt().solve(gradient);
    if (!update.allFinite() || update.head<2>().norm() > 0.12 || std::abs(update.z()) > 0.12) {
      return false;
    }
    state += update;
    if (update.norm() < 1e-6) break;
  }
  if (final_pairs < 35) return false;
  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  delta.translation() = Eigen::Vector3d(state.x(), state.y(), 0.0);
  delta.linear() = Eigen::AngleAxisd(state.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform = base_from_camera_.inverse() * delta.inverse() * base_from_camera_;
  return true;
}

TrackingResult RgbdFrontend::process(const uint8_t* color, int color_step,
                                     const std::string& encoding, const uint16_t* depth,
                                     int depth_step_bytes, const CameraModel& camera) {
  TrackingResult result;
  if (!color || !depth || camera.width <= 0 || camera.height <= 0 || camera.fx <= 0.0F ||
      (encoding != "rgb8" && encoding != "bgr8" && encoding != "mono8" &&
       encoding != "8UC1")) {
    result.reason = "unsupported image input or invalid calibration";
    return result;
  }
  const int depth_stride = depth_step_bytes / static_cast<int>(sizeof(uint16_t));
  auto current_pyramid = makePyramid(color, color_step, encoding, camera);
  if (previous_pyramid_.empty() || previous_camera_.width != camera.width ||
      previous_camera_.height != camera.height) {
    previous_pyramid_ = std::move(current_pyramid);
    previous_depth_.resize(static_cast<size_t>(camera.width * camera.height));
    for (int y = 0; y < camera.height; ++y) {
      std::copy_n(depth + y * depth_stride, camera.width,
                  previous_depth_.begin() + static_cast<size_t>(y * camera.width));
    }
    previous_camera_ = camera;
    reference_to_previous_ = Eigen::Isometry3d::Identity();
    frames_since_reference_ = 0;
    result.reason = "initialized";
    return result;
  }

  const auto features = detectFeatures(previous_pyramid_[0], previous_depth_.data(),
                                       previous_camera_.width, previous_camera_);
  result.detected = static_cast<int>(features.size());
  std::vector<Match> matches;
  matches.reserve(features.size());
  for (const auto& feature : features) {
    Eigen::Vector2f tracked;
    float patch_rmse = 0.0F;
    if (!trackFeature(previous_pyramid_, current_pyramid, feature.pixel, tracked, patch_rmse)) {
      continue;
    }
    const float previous_z = sampleDepth(previous_depth_.data(), previous_camera_.width,
                                         previous_camera_.width, previous_camera_.height,
                                         feature.pixel);
    if (previous_z <= 0.0F) continue;
    Match match;
    match.previous_pixel = feature.pixel;
    match.current_pixel = tracked;
    match.previous_point = unproject(feature.pixel, previous_z, previous_camera_);
    const float current_z = sampleDepth(depth, depth_stride, camera.width, camera.height, tracked);
    if (current_z > 0.0F) {
      match.current_point = unproject(tracked, current_z, camera);
      match.has_current_depth = true;
      ++result.depth_pairs;
    }
    matches.push_back(match);
  }
  result.tracked = static_cast<int>(matches.size());

  Eigen::Isometry3d transform = reference_to_previous_;
  std::vector<int> inliers;
  const bool have_rigid_seed = estimateRigid(matches, transform, inliers);
  if (!have_rigid_seed && matches.size() < static_cast<size_t>(config_.min_features)) {
    result.reason = "insufficient tracked features";
  } else if (config_.planar_refinement && have_base_from_camera_ &&
             !refinePlanar(matches, transform, camera, inliers, result.reprojection_rmse)) {
    result.reason = "planar reprojection refinement failed";
  } else if ((!config_.planar_refinement || !have_base_from_camera_) &&
             !refineReprojection(matches, transform, camera, inliers,
                                 result.reprojection_rmse)) {
    result.reason = "reprojection refinement failed";
  } else {
    result.success = true;
    result.current_from_previous = transform * reference_to_previous_.inverse();
    result.inliers = static_cast<int>(inliers.size());
    result.reason = "tracking";
  }

  ++frames_since_reference_;
  ++processed_frames_;
  bool replace_reference = false;
  if (result.success) {
    const Eigen::Isometry3d incremental_base = base_from_camera_ *
        result.current_from_previous.inverse() * base_from_camera_.inverse();
    cumulative_visual_distance_ += incremental_base.translation().head<2>().norm();
  }
  if (local_window_active_ && processed_frames_ >= 150 &&
      cumulative_visual_distance_ < 0.98) {
    local_window_active_ = false;
    replace_reference = true;
  }
  if (!local_window_active_) {
    replace_reference = true;
  }
  if (result.success) {
    reference_to_previous_ = transform;
    const Eigen::Isometry3d reference_base_delta =
        base_from_camera_ * transform.inverse() * base_from_camera_.inverse();
    const double reference_yaw = std::atan2(reference_base_delta.rotation()(1, 0),
                                             reference_base_delta.rotation()(0, 0));
    if (local_window_active_) {
      replace_reference = reference_base_delta.translation().head<2>().norm() > 0.10 ||
                          std::abs(reference_yaw) > 0.10 || frames_since_reference_ >= 3;
    }
  } else {
    // Re-anchor after a failed local solve so a single blurred frame cannot
    // strand the tracker on an increasingly distant reference.
    replace_reference = true;
  }
  if (replace_reference) {
    previous_pyramid_ = std::move(current_pyramid);
    previous_depth_.resize(static_cast<size_t>(camera.width * camera.height));
    for (int y = 0; y < camera.height; ++y) {
      std::copy_n(depth + y * depth_stride, camera.width,
                  previous_depth_.begin() + static_cast<size_t>(y * camera.width));
    }
    previous_camera_ = camera;
    reference_to_previous_ = Eigen::Isometry3d::Identity();
    frames_since_reference_ = 0;
  }
  return result;
}

}  // namespace k3_rgbd_odometry
