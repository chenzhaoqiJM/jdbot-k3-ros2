// Copyright 2026 chenzhaoqi
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class FastlioOdomConverter : public rclcpp::Node
{
public:
  FastlioOdomConverter()
  : Node("fastlio_odom_converter_node")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/Odometry");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    fixed_z_ = declare_parameter<double>("fixed_z", 0.0);
    max_dt_ = declare_parameter<double>("max_dt", 1.0);
    const auto publish_tf = declare_parameter<bool>("publish_tf", true);
    const auto twist_diagonal = declare_parameter<std::vector<double>>(
      "twist_covariance_diagonal", {0.1, 0.1, 1e6, 1e6, 1e6, 0.1});

    if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_ ||
      odom_frame_.front() == '/' || base_frame_.front() == '/')
    {
      throw std::invalid_argument(
              "odom_frame/base_frame must be distinct, nonempty TF names without a leading slash");
    }
    if (!std::isfinite(fixed_z_) || !std::isfinite(max_dt_) || max_dt_ <= 0.0) {
      throw std::invalid_argument("fixed_z must be finite and max_dt must be positive and finite");
    }
    if (twist_diagonal.size() != 6) {
      throw std::invalid_argument("twist_covariance_diagonal must contain 6 values");
    }
    for (std::size_t i = 0; i < twist_diagonal.size(); ++i) {
      if (!std::isfinite(twist_diagonal[i]) || twist_diagonal[i] < 0.0) {
        throw std::invalid_argument(
                "twist_covariance_diagonal values must be finite and nonnegative");
      }
      twist_covariance_[i * 7] = twist_diagonal[i];
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
    if (publish_tf) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&FastlioOdomConverter::on_odometry, this, std::placeholders::_1));
    if (std::string(subscription_->get_topic_name()) == odom_publisher_->get_topic_name()) {
      throw std::invalid_argument("input_topic and output_topic must resolve to different topics");
    }
    RCLCPP_INFO(
      get_logger(), "Converting %s to %s, %s -> %s, z=%.3f, TF=%s",
      subscription_->get_topic_name(), odom_publisher_->get_topic_name(),
      odom_frame_.c_str(), base_frame_.c_str(), fixed_z_, publish_tf ? "on" : "off");
  }

private:
  void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr input)
  {
    const auto & position = input->pose.pose.position;
    const auto & q = input->pose.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(norm) || norm < 1e-12)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Skipping invalid position or orientation");
      return;
    }
    const double qx = q.x / norm;
    const double qy = q.y / norm;
    const double qz = q.z / norm;
    const double qw = q.w / norm;
    const double yaw = std::atan2(
      2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    const std::int64_t stamp_ns = rclcpp::Time(input->header.stamp).nanoseconds();
    const bool same_frames = input->header.frame_id == previous_parent_frame_ &&
      input->child_frame_id == previous_child_frame_;
    if (has_previous_ && same_frames && stamp_ns == previous_stamp_ns_) {
      return;
    }

    nav_msgs::msg::Odometry output;
    output.header.stamp = input->header.stamp;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = position.x;
    output.pose.pose.position.y = position.y;
    output.pose.pose.position.z = fixed_z_;
    output.pose.pose.orientation.z = std::sin(yaw / 2.0);
    output.pose.pose.orientation.w = std::cos(yaw / 2.0);

    // Retain the x/y/yaw covariance submatrix; z/roll/pitch are constrained.
    constexpr std::array<std::size_t, 3> planar_axes{0, 1, 5};
    for (const auto row : planar_axes) {
      for (const auto column : planar_axes) {
        output.pose.covariance[row * 6 + column] = input->pose.covariance[row * 6 + column];
      }
    }
    output.twist.covariance = twist_covariance_;
    const double dt = static_cast<double>(stamp_ns - previous_stamp_ns_) * 1e-9;
    if (has_previous_ && same_frames && dt > 0.0 && dt <= max_dt_) {
      const double world_vx = (position.x - previous_x_) / dt;
      const double world_vy = (position.y - previous_y_) / dt;
      // Odometry twist must be expressed in the child (base_footprint) frame.
      output.twist.twist.linear.x = std::cos(yaw) * world_vx + std::sin(yaw) * world_vy;
      output.twist.twist.linear.y = -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy;
      output.twist.twist.angular.z =
        std::atan2(std::sin(yaw - previous_yaw_), std::cos(yaw - previous_yaw_)) / dt;
    } else {
      // No valid velocity on the first sample, after a clock reset, or a long gap.
      for (const auto axis : planar_axes) {
        output.twist.covariance[axis * 7] = 1e6;
      }
    }

    odom_publisher_->publish(output);
    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    has_previous_ = true;
    previous_stamp_ns_ = stamp_ns;
    previous_x_ = position.x;
    previous_y_ = position.y;
    previous_yaw_ = yaw;
    previous_parent_frame_ = input->header.frame_id;
    previous_child_frame_ = input->child_frame_id;
  }

  std::string odom_frame_;
  std::string base_frame_;
  double fixed_z_{0.0};
  double max_dt_{1.0};
  std::array<double, 36> twist_covariance_{};
  bool has_previous_{false};
  std::int64_t previous_stamp_ns_{0};
  double previous_x_{0.0};
  double previous_y_{0.0};
  double previous_yaw_{0.0};
  std::string previous_parent_frame_;
  std::string previous_child_frame_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<FastlioOdomConverter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("fastlio_odom_converter_node"), "%s", error.what());
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
