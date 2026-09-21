#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer_ros/node_options.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace cartographer_lidar_odometry {

using cartographer::mapping::LocalTrajectoryBuilder2D;
using cartographer::transform::Rigid3d;

double NormalizeAngle(const double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

rclcpp::Time ToRosTime(const cartographer::common::Time time) {
  const int64_t universal_time = cartographer::common::ToUniversal(time);
  const int64_t nanoseconds =
      (universal_time -
       cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
           10000000LL) *
      100LL;
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

cartographer::common::Time FromRosTime(const rclcpp::Time& time) {
  return cartographer::common::FromUniversal(
      cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
          10000000LL +
      (time.nanoseconds() + 50) / 100);
}

std::tuple<cartographer::sensor::PointCloudWithIntensities,
           cartographer::common::Time>
ConvertLaserScan(const sensor_msgs::msg::LaserScan& msg) {
  cartographer::sensor::PointCloudWithIntensities point_cloud;
  point_cloud.points.reserve(msg.ranges.size());
  point_cloud.intensities.reserve(msg.ranges.size());
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const float range = msg.ranges[i];
    if (std::isfinite(range) && range >= msg.range_min &&
        range <= msg.range_max) {
      point_cloud.points.push_back({
          Eigen::Vector3f(range * std::cos(angle), range * std::sin(angle), 0.f),
          static_cast<float>(i) * msg.time_increment});
      point_cloud.intensities.push_back(
          i < msg.intensities.size() ? msg.intensities[i] : 0.f);
    }
    angle += msg.angle_increment;
  }
  auto timestamp = FromRosTime(rclcpp::Time(msg.header.stamp));
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return {std::move(point_cloud), timestamp};
}

Rigid3d ToRigid3d(const geometry_msgs::msg::TransformStamped& transform) {
  const auto& translation = transform.transform.translation;
  const auto& rotation = transform.transform.rotation;
  return Rigid3d(Eigen::Vector3d(translation.x, translation.y, translation.z),
                 Eigen::Quaterniond(rotation.w, rotation.x, rotation.y,
                                    rotation.z));
}

std::array<double, 36> MakeCovariance(const std::vector<double>& diagonal) {
  if (diagonal.size() != 6U) {
    throw std::invalid_argument("covariance diagonal must have 6 elements");
  }
  std::array<double, 36> result{};
  constexpr std::array<size_t, 6> indices{0, 7, 14, 21, 28, 35};
  for (size_t i = 0; i < diagonal.size(); ++i) {
    if (!std::isfinite(diagonal[i]) || diagonal[i] < 0.) {
      throw std::invalid_argument(
          "covariance diagonal values must be finite and non-negative");
    }
    result[indices[i]] = diagonal[i];
  }
  return result;
}

class LidarOdometryNode final : public rclcpp::Node {
 public:
  LidarOdometryNode() : Node("cartographer_lidar_odometry") {
    const auto configuration_directory = declare_parameter<std::string>(
        "configuration_directory", "");
    const auto configuration_basename = declare_parameter<std::string>(
        "configuration_basename", "lidar_odometry.lua");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const auto odom_topic =
        declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    velocity_filter_alpha_ =
        declare_parameter<double>("velocity_filter_alpha", 0.35);
    max_sample_interval_ =
        declare_parameter<double>("max_sample_interval", 1.0);
    pose_covariance_ = MakeCovariance(declare_parameter<std::vector<double>>(
        "pose_covariance_diagonal",
        {0.0025, 0.0025, 1e6, 1e6, 1e6, 0.0012}));
    twist_covariance_ = MakeCovariance(declare_parameter<std::vector<double>>(
        "twist_covariance_diagonal",
        {0.01, 0.01, 1e6, 1e6, 1e6, 0.0076}));

    if (configuration_directory.empty()) {
      throw std::invalid_argument("configuration_directory must not be empty");
    }
    if (!(velocity_filter_alpha_ > 0. && velocity_filter_alpha_ <= 1.)) {
      throw std::invalid_argument("velocity_filter_alpha must be in (0, 1]");
    }
    if (!(max_sample_interval_ > 0.) || !std::isfinite(max_sample_interval_)) {
      throw std::invalid_argument("max_sample_interval must be finite and > 0");
    }

    cartographer_ros::NodeOptions node_options;
    std::tie(node_options, trajectory_options_) = cartographer_ros::LoadOptions(
        configuration_directory, configuration_basename);
    if (!trajectory_options_.trajectory_builder_options
             .has_trajectory_builder_2d_options()) {
      throw std::invalid_argument("configuration does not enable 2D SLAM");
    }
    if (trajectory_options_.trajectory_builder_options
            .trajectory_builder_2d_options()
            .use_imu_data()) {
      throw std::invalid_argument(
          "this node is LiDAR-only; set TRAJECTORY_BUILDER_2D.use_imu_data=false");
    }
    if (trajectory_options_.num_laser_scans != 1) {
      throw std::invalid_argument("this node requires exactly one LaserScan");
    }

    child_frame_ = trajectory_options_.published_frame;
    const auto& local_options = trajectory_options_.trajectory_builder_options
                                    .trajectory_builder_2d_options();
    local_trajectory_builder_ = std::make_unique<LocalTrajectoryBuilder2D>(
        local_options, std::vector<std::string>{"scan"});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    // TfBridge performs bounded lookups from the scan callback. Use the
    // listener's dedicated executor so TF can continue filling the buffer
    // while that callback waits.
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, true);
    lookup_transform_timeout_sec_ = node_options.lookup_transform_timeout_sec;
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(this);

    odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&LidarOdometryNode::HandleScan, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "LiDAR-only Cartographer front end: %s -> %s, %s -> %s",
                scan_topic.c_str(), odom_topic.c_str(), odom_frame_.c_str(),
                child_frame_.c_str());
  }

 private:
  void HandleScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    cartographer::sensor::PointCloudWithIntensities points;
    cartographer::common::Time scan_end_time;
    std::tie(points, scan_end_time) =
        ConvertLaserScan(*msg);
    if (points.points.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring an empty LaserScan");
      return;
    }

    const int subdivisions =
        std::max(1, trajectory_options_.num_subdivisions_per_laser_scan);
    for (int i = 0; i < subdivisions; ++i) {
      const size_t begin = points.points.size() * i / subdivisions;
      const size_t end = points.points.size() * (i + 1) / subdivisions;
      if (begin == end) {
        continue;
      }
      cartographer::sensor::TimedPointCloud ranges(points.points.begin() + begin,
                                                   points.points.begin() + end);
      const double relative_end_time = ranges.back().time;
      const auto subdivision_time =
          scan_end_time + cartographer::common::FromSeconds(relative_end_time);
      if (have_last_subdivision_time_ && subdivision_time <= last_subdivision_time_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Ignoring non-monotonic LaserScan timestamp");
        continue;
      }
      last_subdivision_time_ = subdivision_time;
      have_last_subdivision_time_ = true;
      for (auto& point : ranges) {
        point.time -= relative_end_time;
      }

      std::string frame_id = msg->header.frame_id;
      if (!frame_id.empty() && frame_id.front() == '/') {
        frame_id.erase(frame_id.begin());
      }
      std::unique_ptr<Rigid3d> sensor_to_tracking;
      try {
        sensor_to_tracking = std::make_unique<Rigid3d>(ToRigid3d(
            tf_buffer_->lookupTransform(
                trajectory_options_.tracking_frame, frame_id,
                ToRosTime(subdivision_time),
                tf2::durationFromSec(lookup_transform_timeout_sec_))));
      } catch (const tf2::TransformException&) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Waiting for transform %s <- %s", 
            trajectory_options_.tracking_frame.c_str(), frame_id.c_str());
        continue;
      }

      const auto transformed_ranges = cartographer::sensor::TransformTimedPointCloud(
          ranges, sensor_to_tracking->cast<float>());
      auto result = local_trajectory_builder_->AddRangeData(
          "scan", cartographer::sensor::TimedPointCloudData{
                      subdivision_time,
                      sensor_to_tracking->translation().cast<float>(),
                      transformed_ranges, {}});
      if (result) {
        PublishOdometry(result->time, result->local_pose);
      }
    }
  }

  void PublishOdometry(const cartographer::common::Time time,
                       const Rigid3d& pose) {
    const auto stamp = ToRosTime(time);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose.position.x = pose.translation().x();
    odom.pose.pose.position.y = pose.translation().y();
    odom.pose.pose.position.z = pose.translation().z();
    odom.pose.pose.orientation.w = pose.rotation().w();
    odom.pose.pose.orientation.x = pose.rotation().x();
    odom.pose.pose.orientation.y = pose.rotation().y();
    odom.pose.pose.orientation.z = pose.rotation().z();
    std::copy(pose_covariance_.begin(), pose_covariance_.end(),
              odom.pose.covariance.begin());

    const double yaw = cartographer::transform::GetYaw(pose);
    if (have_previous_pose_) {
      const double dt = cartographer::common::ToSeconds(time - previous_time_);
      if (dt > 0. && dt <= max_sample_interval_) {
        const double dx = pose.translation().x() - previous_pose_.translation().x();
        const double dy = pose.translation().y() - previous_pose_.translation().y();
        const double previous_yaw =
            cartographer::transform::GetYaw(previous_pose_);
        const double delta_yaw = NormalizeAngle(yaw - previous_yaw);
        const double c = std::cos(previous_yaw);
        const double s = std::sin(previous_yaw);
        const double body_dx = c * dx + s * dy;
        const double body_dy = -s * dx + c * dy;
        const double a = std::abs(delta_yaw) < 1e-6
                             ? 1. - delta_yaw * delta_yaw / 6.
                             : std::sin(delta_yaw) / delta_yaw;
        const double b = std::abs(delta_yaw) < 1e-6
                             ? delta_yaw / 2.
                             : (1. - std::cos(delta_yaw)) / delta_yaw;
        const double denominator = a * a + b * b;
        std::array<double, 3> raw{
            (a * body_dx + b * body_dy) / denominator / dt,
            (-b * body_dx + a * body_dy) / denominator / dt,
            delta_yaw / dt};
        if (!have_filtered_velocity_) {
          filtered_velocity_ = raw;
          have_filtered_velocity_ = true;
        } else {
          for (size_t i = 0; i < raw.size(); ++i) {
            filtered_velocity_[i] = velocity_filter_alpha_ * raw[i] +
                                    (1. - velocity_filter_alpha_) *
                                        filtered_velocity_[i];
          }
        }
      } else {
        have_filtered_velocity_ = false;
      }
    }
    if (have_filtered_velocity_) {
      odom.twist.twist.linear.x = filtered_velocity_[0];
      odom.twist.twist.linear.y = filtered_velocity_[1];
      odom.twist.twist.angular.z = filtered_velocity_[2];
    }
    std::copy(twist_covariance_.begin(), twist_covariance_.end(),
              odom.twist.covariance.begin());
    odom_publisher_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odom.header;
      transform.child_frame_id = child_frame_;
      transform.transform.translation.x = pose.translation().x();
      transform.transform.translation.y = pose.translation().y();
      transform.transform.translation.z = pose.translation().z();
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
    previous_pose_ = pose;
    previous_time_ = time;
    have_previous_pose_ = true;
    ++published_count_;
    if (published_count_ == 1U) {
      RCLCPP_INFO(get_logger(), "First scan-matched odometry pose published");
    }
  }

  cartographer_ros::TrajectoryOptions trajectory_options_;
  std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  std::string odom_frame_;
  std::string child_frame_;
  bool publish_tf_ = true;
  double velocity_filter_alpha_ = 0.35;
  double max_sample_interval_ = 1.;
  double lookup_transform_timeout_sec_ = 0.2;
  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};
  bool have_last_subdivision_time_ = false;
  cartographer::common::Time last_subdivision_time_;
  bool have_previous_pose_ = false;
  Rigid3d previous_pose_ = Rigid3d::Identity();
  cartographer::common::Time previous_time_;
  bool have_filtered_velocity_ = false;
  std::array<double, 3> filtered_velocity_{};
  size_t published_count_ = 0;
};

}  // namespace cartographer_lidar_odometry

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<cartographer_lidar_odometry::LidarOdometryNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("cartographer_lidar_odometry"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
