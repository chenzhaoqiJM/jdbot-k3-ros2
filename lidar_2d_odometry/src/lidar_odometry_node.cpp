#include "lidar_2d_odometry/scan_matcher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace lidar_2d_odometry {
namespace {

std::array<double, 36> Covariance(const std::vector<double>& diagonal) {
  if (diagonal.size() != 6) {
    throw std::invalid_argument("covariance diagonal must contain 6 values");
  }
  std::array<double, 36> covariance{};
  constexpr std::array<std::size_t, 6> indices{0, 7, 14, 21, 28, 35};
  for (std::size_t i = 0; i < diagonal.size(); ++i) {
    if (!std::isfinite(diagonal[i]) || diagonal[i] < 0.0) {
      throw std::invalid_argument("covariance values must be finite and >= 0");
    }
    covariance[indices[i]] = diagonal[i];
  }
  return covariance;
}

double Yaw(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

class LidarOdometryNode final : public rclcpp::Node {
 public:
  LidarOdometryNode()
      : Node("lidar_2d_odometry"), matcher_(ReadMatcherOptions()) {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    tracking_frame_ =
        declare_parameter<std::string>("tracking_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    min_range_ = declare_parameter<double>("min_range", 0.15);
    max_range_ = declare_parameter<double>("max_range", 10.0);
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.2);
    velocity_filter_alpha_ =
        declare_parameter<double>("velocity_filter_alpha", 0.35);
    pose_covariance_ = Covariance(declare_parameter<std::vector<double>>(
        "pose_covariance_diagonal",
        {0.0025, 0.0025, 1e6, 1e6, 1e6, 0.0012}));
    twist_covariance_ = Covariance(declare_parameter<std::vector<double>>(
        "twist_covariance_diagonal",
        {0.01, 0.01, 1e6, 1e6, 1e6, 0.0076}));
    if (!(min_range_ >= 0.0 && max_range_ > min_range_) ||
        !(velocity_filter_alpha_ > 0.0 && velocity_filter_alpha_ <= 1.0)) {
      throw std::invalid_argument("invalid range or velocity filter parameter");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, true);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&LidarOdometryNode::HandleScan, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
                "Eigen scan-to-map odometry: %s -> %s, frame %s -> %s",
                scan_topic_.c_str(), odom_topic_.c_str(), odom_frame_.c_str(),
                tracking_frame_.c_str());
  }

 private:
  MatcherOptions ReadMatcherOptions() {
    MatcherOptions options;
    options.voxel_size = declare_parameter<double>("voxel_size", 0.045);
    options.correspondence_distance =
        declare_parameter<double>("correspondence_distance", 0.28);
    options.huber_scale = declare_parameter<double>("huber_scale", 0.06);
    options.keyframe_translation =
        declare_parameter<double>("keyframe_translation", 0.12);
    options.keyframe_rotation =
        declare_parameter<double>("keyframe_rotation", 0.06);
    options.translation_prediction_weight =
        declare_parameter<double>("translation_prediction_weight", 2.0);
    options.rotation_prediction_weight =
        declare_parameter<double>("rotation_prediction_weight", 4.0);
    options.max_iterations = declare_parameter<int>("max_iterations", 12);
    options.min_correspondences =
        declare_parameter<int>("min_correspondences", 35);
    options.max_keyframes = declare_parameter<int>("max_keyframes", 90);
    return options;
  }

  void HandleScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr message) {
    std::string sensor_frame = message->header.frame_id;
    if (!sensor_frame.empty() && sensor_frame.front() == '/') {
      sensor_frame.erase(sensor_frame.begin());
    }
    std::size_t last_valid_index = 0;
    for (std::size_t i = 0; i < message->ranges.size(); ++i) {
      const float range = message->ranges[i];
      if (std::isfinite(range) && range >= message->range_min &&
          range <= message->range_max) {
        last_valid_index = i;
      }
    }
    const double scan_duration =
        static_cast<double>(last_valid_index) * message->time_increment;
    const rclcpp::Time scan_end_time =
        rclcpp::Time(message->header.stamp) +
        rclcpp::Duration::from_seconds(scan_duration);

    geometry_msgs::msg::TransformStamped sensor_to_tracking;
    try {
      sensor_to_tracking = tf_buffer_->lookupTransform(
          tracking_frame_, sensor_frame, scan_end_time,
          tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException&) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for transform %s <- %s",
                           tracking_frame_.c_str(), sensor_frame.c_str());
      return;
    }

    const auto& translation = sensor_to_tracking.transform.translation;
    const auto& rotation = sensor_to_tracking.transform.rotation;
    const double sensor_yaw =
        Yaw(rotation.x, rotation.y, rotation.z, rotation.w);
    const double c = std::cos(sensor_yaw);
    const double s = std::sin(sensor_yaw);
    const double effective_min = std::max(min_range_,
                                          static_cast<double>(message->range_min));
    const double effective_max = std::min(max_range_,
                                          static_cast<double>(message->range_max));
    std::vector<Eigen::Vector2d> points;
    points.reserve(message->ranges.size());
    double angle = message->angle_min;
    for (std::size_t i = 0; i < message->ranges.size(); ++i) {
      const float range = message->ranges[i];
      if (std::isfinite(range) && range >= effective_min &&
          range <= effective_max) {
        const double lx = range * std::cos(angle);
        const double ly = range * std::sin(angle);
        points.emplace_back(translation.x + c * lx - s * ly,
                            translation.y + s * lx + c * ly);
      }
      angle += message->angle_increment;
    }

    const MatchResult result = matcher_.AddScan(points);
    if (!result.accepted) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Scan match rejected (%d correspondences)",
                           result.correspondences);
      return;
    }
    Publish(static_cast<builtin_interfaces::msg::Time>(scan_end_time),
            result.pose);
  }

  void Publish(const builtin_interfaces::msg::Time& stamp, const Pose2d& pose) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = tracking_frame_;
    odometry.pose.pose.position.x = pose.x;
    odometry.pose.pose.position.y = pose.y;
    odometry.pose.pose.orientation.z = std::sin(0.5 * pose.yaw);
    odometry.pose.pose.orientation.w = std::cos(0.5 * pose.yaw);
    std::copy(pose_covariance_.begin(), pose_covariance_.end(),
              odometry.pose.covariance.begin());

    const rclcpp::Time time(stamp);
    if (have_previous_pose_) {
      const double dt = (time - previous_time_).seconds();
      if (dt > 0.0 && dt < 1.0) {
        const Pose2d delta = ScanMatcher::Compose(
            ScanMatcher::Inverse(previous_pose_), pose);
        const std::array<double, 3> raw{delta.x / dt, delta.y / dt,
                                        delta.yaw / dt};
        if (!have_velocity_) filtered_velocity_ = raw;
        for (std::size_t i = 0; i < raw.size(); ++i) {
          filtered_velocity_[i] =
              velocity_filter_alpha_ * raw[i] +
              (1.0 - velocity_filter_alpha_) * filtered_velocity_[i];
        }
        have_velocity_ = true;
      }
    }
    if (have_velocity_) {
      odometry.twist.twist.linear.x = filtered_velocity_[0];
      odometry.twist.twist.linear.y = filtered_velocity_[1];
      odometry.twist.twist.angular.z = filtered_velocity_[2];
    }
    std::copy(twist_covariance_.begin(), twist_covariance_.end(),
              odometry.twist.covariance.begin());
    odom_publisher_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = tracking_frame_;
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
    previous_pose_ = pose;
    previous_time_ = time;
    have_previous_pose_ = true;
  }

  ScanMatcher matcher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string tracking_frame_;
  bool publish_tf_ = true;
  double min_range_ = 0.15;
  double max_range_ = 10.0;
  double transform_timeout_ = 0.2;
  double velocity_filter_alpha_ = 0.35;
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};
  bool have_previous_pose_ = false;
  bool have_velocity_ = false;
  Pose2d previous_pose_;
  rclcpp::Time previous_time_{0, 0, RCL_ROS_TIME};
  std::array<double, 3> filtered_velocity_{};
};

}  // namespace lidar_2d_odometry

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lidar_2d_odometry::LidarOdometryNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_2d_odometry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
