#include "rgbd_odometry/image.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rgbd_odometry {

bool validate_color(const sensor_msgs::msg::Image &image, std::string &error) {
  int bytes_per_pixel = 0;
  if (image.encoding == "rgb8" || image.encoding == "bgr8") {
    bytes_per_pixel = 3;
  } else if (image.encoding == "mono8") {
    bytes_per_pixel = 1;
  } else {
    error = "unsupported color encoding: " + image.encoding;
    return false;
  }
  if (image.width == 0 || image.height == 0 ||
      image.step < image.width * static_cast<uint32_t>(bytes_per_pixel) ||
      image.data.size() < static_cast<size_t>(image.step) * image.height) {
    error = "invalid color dimensions, step, or data length";
    return false;
  }
  return true;
}

bool decode_grayscale(const sensor_msgs::msg::Image &image,
                      std::vector<uint8_t> &gray, std::string &error) {
  if (!validate_color(image, error)) return false;
  const bool mono = image.encoding == "mono8";
  const bool rgb = image.encoding == "rgb8";
  gray.resize(static_cast<size_t>(image.width) * image.height);
  for (uint32_t y = 0; y < image.height; ++y) {
    const uint8_t *row = image.data.data() + static_cast<size_t>(y) * image.step;
    for (uint32_t x = 0; x < image.width; ++x) {
      if (mono) {
        gray[static_cast<size_t>(y) * image.width + x] = row[x];
      } else {
        const uint8_t *pixel = row + 3U * x;
        const uint8_t r = pixel[rgb ? 0 : 2];
        const uint8_t g = pixel[1];
        const uint8_t b = pixel[rgb ? 2 : 0];
        gray[static_cast<size_t>(y) * image.width + x] = static_cast<uint8_t>(
            (77U * r + 150U * g + 29U * b + 128U) >> 8U);
      }
    }
  }
  return true;
}

bool decode_depth(const sensor_msgs::msg::Image &image, float scale,
                  float min_depth, float max_depth,
                  std::vector<float> &depth_m, std::string &error) {
  if (image.encoding != "16UC1" && image.encoding != "mono16") {
    error = "unsupported depth encoding: " + image.encoding;
    return false;
  }
  if (image.width == 0 || image.height == 0 || image.step < image.width * 2U ||
      image.data.size() < static_cast<size_t>(image.step) * image.height) {
    error = "invalid depth dimensions, step, or data length";
    return false;
  }
  if (!(scale > 0.0F) || !(min_depth >= 0.0F) || !(max_depth > min_depth)) {
    error = "invalid depth scale or range";
    return false;
  }
  depth_m.resize(static_cast<size_t>(image.width) * image.height);
  for (uint32_t y = 0; y < image.height; ++y) {
    const uint8_t *row = image.data.data() + static_cast<size_t>(y) * image.step;
    for (uint32_t x = 0; x < image.width; ++x) {
      const uint8_t a = row[2U * x];
      const uint8_t b = row[2U * x + 1U];
      const uint16_t raw = image.is_bigendian
          ? static_cast<uint16_t>((static_cast<uint16_t>(a) << 8U) | b)
          : static_cast<uint16_t>((static_cast<uint16_t>(b) << 8U) | a);
      const float z = static_cast<float>(raw) / scale;
      depth_m[static_cast<size_t>(y) * image.width + x] =
          (raw != 0U && z >= min_depth && z <= max_depth)
              ? z : std::numeric_limits<float>::quiet_NaN();
    }
  }
  return true;
}

bool parse_intrinsics(const sensor_msgs::msg::CameraInfo &info,
                      Intrinsics &out, std::string &error) {
  if (info.width == 0 || info.height == 0 || !std::isfinite(info.k[0]) ||
      !std::isfinite(info.k[4]) || info.k[0] <= 0.0 || info.k[4] <= 0.0 ||
      !std::isfinite(info.k[2]) || !std::isfinite(info.k[5])) {
    error = "invalid image dimensions or pinhole intrinsics";
    return false;
  }
  if (info.distortion_model != "plumb_bob" &&
      info.distortion_model != "rational_polynomial" &&
      !info.distortion_model.empty()) {
    error = "unsupported distortion model: " + info.distortion_model;
    return false;
  }
  const size_t required = info.distortion_model == "rational_polynomial" ? 8U :
                          info.distortion_model == "plumb_bob" ? 5U : 0U;
  if (info.d.size() < required ||
      !std::all_of(info.d.begin(), info.d.end(),
                   [](double v) { return std::isfinite(v); })) {
    error = "invalid distortion coefficient vector";
    return false;
  }
  out.width = static_cast<int>(info.width);
  out.height = static_cast<int>(info.height);
  out.fx = info.k[0];
  out.fy = info.k[4];
  out.cx = info.k[2];
  out.cy = info.k[5];
  out.distortion_model = info.distortion_model;
  out.d = info.d;
  return true;
}

bool build_undistorted_rays(const Intrinsics &in,
                            std::vector<float> &ray_x,
                            std::vector<float> &ray_y,
                            std::string &error) {
  if (in.width <= 0 || in.height <= 0 || in.fx <= 0.0 || in.fy <= 0.0) {
    error = "invalid intrinsics";
    return false;
  }
  const size_t count = static_cast<size_t>(in.width) * in.height;
  ray_x.resize(count);
  ray_y.resize(count);
  for (int v = 0; v < in.height; ++v) {
    for (int u = 0; u < in.width; ++u) {
      const double xd = (static_cast<double>(u) - in.cx) / in.fx;
      const double yd = (static_cast<double>(v) - in.cy) / in.fy;
      double x = xd;
      double y = yd;
      if (!in.d.empty()) {
        for (int iteration = 0; iteration < 8; ++iteration) {
          const double r2 = x * x + y * y;
          const double r4 = r2 * r2;
          const double r6 = r4 * r2;
          const double numerator = 1.0 + in.d[0] * r2 + in.d[1] * r4 + in.d[4] * r6;
          double denominator = 1.0;
          if (in.distortion_model == "rational_polynomial") {
            denominator += in.d[5] * r2 + in.d[6] * r4 + in.d[7] * r6;
          }
          if (std::abs(denominator) < 1e-12) {
            error = "singular distortion model";
            return false;
          }
          const double radial = numerator / denominator;
          const double dx = 2.0 * in.d[2] * x * y + in.d[3] * (r2 + 2.0 * x * x);
          const double dy = in.d[2] * (r2 + 2.0 * y * y) + 2.0 * in.d[3] * x * y;
          if (std::abs(radial) < 1e-12) {
            error = "singular distortion model";
            return false;
          }
          x = (xd - dx) / radial;
          y = (yd - dy) / radial;
        }
      }
      const size_t index = static_cast<size_t>(v) * in.width + u;
      ray_x[index] = static_cast<float>(x);
      ray_y[index] = static_cast<float>(y);
    }
  }
  return true;
}

}  // namespace rgbd_odometry
