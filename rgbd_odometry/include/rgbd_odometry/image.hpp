#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rgbd_odometry {

struct Intrinsics {
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string distortion_model;
  std::vector<double> d;
};

bool validate_color(const sensor_msgs::msg::Image &image, std::string &error);
bool decode_grayscale(const sensor_msgs::msg::Image &image,
                      std::vector<uint8_t> &gray, std::string &error);
bool decode_depth(const sensor_msgs::msg::Image &image, float scale,
                  float min_depth, float max_depth,
                  std::vector<float> &depth_m, std::string &error);
bool parse_intrinsics(const sensor_msgs::msg::CameraInfo &info,
                      Intrinsics &out, std::string &error);
bool build_undistorted_rays(const Intrinsics &in,
                            std::vector<float> &ray_x,
                            std::vector<float> &ray_y,
                            std::string &error);

}  // namespace rgbd_odometry
