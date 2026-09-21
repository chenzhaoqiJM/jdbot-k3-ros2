#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "rgbd_odometry/engine.hpp"
#include "rgbd_odometry/image.hpp"

namespace rgbd_odometry {
namespace {

int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

geometry_msgs::msg::Quaternion quaternion_message(const Eigen::Matrix3d &rotation) {
  Eigen::Quaterniond q(rotation);
  q.normalize();
  geometry_msgs::msg::Quaternion message;
  message.x = q.x(); message.y = q.y(); message.z = q.z(); message.w = q.w();
  return message;
}

}  // namespace

class RgbdOdometryNode final : public rclcpp::Node {
 public:
  RgbdOdometryNode() : Node("rgbd_odometry"), tf_buffer_(get_clock()),
      tf_listener_(tf_buffer_) {
    declare_parameters();
    if (!read_and_validate_parameters()) {
      RCLCPP_FATAL(get_logger(), "Invalid parameters; tracking is disabled");
      parameters_valid_ = false;
      return;
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
    tracking_publisher_ = create_publisher<std_msgs::msg::Bool>(tracking_topic_, 10);
    if (publish_tf_) tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)));
    if (input_reliable_qos_) {
      qos.reliable().durability_volatile();
    } else {
      qos.best_effort().durability_volatile();
    }
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          ++received_color_; color_queue_.push_back(std::move(message)); trim_queues(); match();
        });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, qos, [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          ++received_depth_; depth_queue_.push_back(std::move(message)); trim_queues(); match();
        });
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) { receive_camera_info(*message); });
    report_timer_ = create_wall_timer(
        std::chrono::duration<double>(report_interval_s_), [this]() { report(); });
    RCLCPP_INFO(get_logger(),
                "ready: color=%s depth=%s output=%s planar=%s RVV=off (portable scalar path)",
                color_topic_.c_str(), depth_topic_.c_str(), output_topic_.c_str(),
                planar_mode_ ? "true" : "false");
  }

 private:
  void declare_parameters() {
    declare_parameter<std::string>("node_name", "rgbd_odometry");
    declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
    declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    declare_parameter<std::string>("output_odom_topic", "/rgbd_odometry/odom");
    declare_parameter<std::string>("tracking_topic", "/rgbd_odometry/tracking");
    declare_parameter<std::string>("rig_frame", "base_footprint");
    declare_parameter<std::string>("odom_frame", "rgbd_odom");
    declare_parameter<double>("depth_scale_factor", 1000.0);
    declare_parameter<double>("min_depth_m", 0.2);
    declare_parameter<double>("max_depth_m", 8.0);
    declare_parameter<double>("sync_slop_s", 0.015);
    declare_parameter<int>("sync_queue_size", 30);
    declare_parameter<int>("qos_depth", 40);
    declare_parameter<bool>("input_reliable_qos", true);
    declare_parameter<bool>("planar_mode", true);
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<double>("report_interval_s", 5.0);
    declare_parameter<int>("coarse_stride", 8);
    declare_parameter<int>("fine_stride", 4);
    declare_parameter<int>("coarse_iterations", 8);
    declare_parameter<int>("fine_iterations", 3);
    declare_parameter<double>("max_correspondence_m", 0.08);
    declare_parameter<int>("depth_filter_radius", 1);
    declare_parameter<double>("depth_filter_max_difference_m", 0.05);
    declare_parameter<int>("min_correspondences", 180);
    declare_parameter<double>("color_static_threshold", 8.0);
    declare_parameter<int>("static_keyframe_max_frames", 60);
    declare_parameter<double>("photometric_weight", 0.1);
    declare_parameter<bool>("photometric_affine_compensation", false);
    declare_parameter<double>("local_keyframe_fusion_weight", 0.0);
    declare_parameter<int>("local_keyframe_max_frames", 12);
    declare_parameter<double>("local_keyframe_max_translation_m", 0.15);
    declare_parameter<double>("local_keyframe_max_rotation_rad", 0.15);
    declare_parameter<double>("local_keyframe_consistency_translation_m", 0.06);
    declare_parameter<double>("local_keyframe_consistency_rotation_rad", 0.08);
    declare_parameter<double>("planar_translation_deadband_m", 0.0015);
  }

  bool read_and_validate_parameters() {
    color_topic_ = get_parameter("color_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    output_topic_ = get_parameter("output_odom_topic").as_string();
    tracking_topic_ = get_parameter("tracking_topic").as_string();
    rig_frame_ = get_parameter("rig_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    depth_scale_ = get_parameter("depth_scale_factor").as_double();
    min_depth_ = get_parameter("min_depth_m").as_double();
    max_depth_ = get_parameter("max_depth_m").as_double();
    sync_slop_ns_ = static_cast<int64_t>(get_parameter("sync_slop_s").as_double() * 1e9);
    sync_queue_size_ = get_parameter("sync_queue_size").as_int();
    qos_depth_ = get_parameter("qos_depth").as_int();
    input_reliable_qos_ = get_parameter("input_reliable_qos").as_bool();
    planar_mode_ = get_parameter("planar_mode").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    report_interval_s_ = get_parameter("report_interval_s").as_double();
    EngineConfig config;
    config.coarse_stride = get_parameter("coarse_stride").as_int();
    config.fine_stride = get_parameter("fine_stride").as_int();
    config.coarse_iterations = get_parameter("coarse_iterations").as_int();
    config.fine_iterations = get_parameter("fine_iterations").as_int();
    config.max_correspondence_m = get_parameter("max_correspondence_m").as_double();
    config.depth_filter_radius = get_parameter("depth_filter_radius").as_int();
    config.depth_filter_max_difference_m =
        get_parameter("depth_filter_max_difference_m").as_double();
    config.min_correspondences = get_parameter("min_correspondences").as_int();
    config.color_static_threshold = get_parameter("color_static_threshold").as_double();
    config.static_keyframe_max_frames = get_parameter("static_keyframe_max_frames").as_int();
    config.photometric_weight = get_parameter("photometric_weight").as_double();
    config.photometric_affine_compensation =
        get_parameter("photometric_affine_compensation").as_bool();
    config.local_keyframe_fusion_weight =
        get_parameter("local_keyframe_fusion_weight").as_double();
    config.local_keyframe_max_frames = get_parameter("local_keyframe_max_frames").as_int();
    config.local_keyframe_max_translation_m =
        get_parameter("local_keyframe_max_translation_m").as_double();
    config.local_keyframe_max_rotation_rad =
        get_parameter("local_keyframe_max_rotation_rad").as_double();
    config.local_keyframe_consistency_translation_m =
        get_parameter("local_keyframe_consistency_translation_m").as_double();
    config.local_keyframe_consistency_rotation_rad =
        get_parameter("local_keyframe_consistency_rotation_rad").as_double();
    planar_translation_deadband_m_ =
        get_parameter("planar_translation_deadband_m").as_double();
    engine_ = Engine(config);
    const auto nonempty = [](const std::string &s) { return !s.empty(); };
    const bool valid = nonempty(color_topic_) && nonempty(depth_topic_) &&
        nonempty(camera_info_topic_) && nonempty(output_topic_) &&
        nonempty(tracking_topic_) && nonempty(rig_frame_) && nonempty(odom_frame_) &&
        depth_scale_ > 0.0 && min_depth_ >= 0.0 && max_depth_ > min_depth_ &&
        sync_slop_ns_ >= 0 && sync_slop_ns_ <= 1000000000LL &&
        sync_queue_size_ >= 2 && sync_queue_size_ <= 1000 &&
        qos_depth_ >= 1 && qos_depth_ <= 1000 && report_interval_s_ > 0.0 &&
        config.coarse_stride >= 2 && config.coarse_stride <= 32 &&
        config.fine_stride >= 1 && config.fine_stride <= config.coarse_stride &&
        config.coarse_iterations >= 1 && config.coarse_iterations <= 30 &&
        config.fine_iterations >= 1 && config.fine_iterations <= 30 &&
        config.max_correspondence_m > 0.01F && config.max_correspondence_m <= 1.0F &&
        config.depth_filter_radius >= 0 && config.depth_filter_radius <= 2 &&
        config.depth_filter_max_difference_m >= 0.005F &&
        config.depth_filter_max_difference_m <= 0.2F &&
        config.min_correspondences >= 20 && config.color_static_threshold >= 0.0F &&
        config.color_static_threshold <= 30.0F &&
        config.static_keyframe_max_frames >= 1 && config.static_keyframe_max_frames <= 1000 &&
        config.local_keyframe_max_frames >= 2 && config.local_keyframe_max_frames <= 60 &&
        config.local_keyframe_max_translation_m >= 0.03F &&
        config.local_keyframe_max_translation_m <= 1.0F &&
        config.local_keyframe_max_rotation_rad >= 0.03F &&
        config.local_keyframe_max_rotation_rad <= 1.0F &&
        config.local_keyframe_consistency_translation_m >= 0.005F &&
        config.local_keyframe_consistency_translation_m <= 0.5F &&
        config.local_keyframe_consistency_rotation_rad >= 0.005F &&
        config.local_keyframe_consistency_rotation_rad <= 0.5F;
    if (config.photometric_weight < 0.0F || config.photometric_weight > 1.0F ||
        config.local_keyframe_fusion_weight < 0.0F ||
        config.local_keyframe_fusion_weight > 1.0F ||
        planar_translation_deadband_m_ < 0.0 || planar_translation_deadband_m_ > 0.05) {
      RCLCPP_ERROR(get_logger(), "fusion/photometric weight or planar deadband outside range");
      return false;
    }
    if (!valid) RCLCPP_ERROR(get_logger(), "parameter outside documented range");
    return valid;
  }

  void receive_camera_info(const sensor_msgs::msg::CameraInfo &message) {
    if (calibration_ready_) return;
    std::string error;
    Intrinsics parsed;
    if (!parse_intrinsics(message, parsed, error) ||
        !build_undistorted_rays(parsed, ray_x_, ray_y_, error)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "camera calibration rejected: %s", error.c_str());
      publish_tracking(false);
      return;
    }
    intrinsics_ = std::move(parsed);
    camera_frame_ = message.header.frame_id;
    if (camera_frame_.empty()) {
      RCLCPP_ERROR(get_logger(), "CameraInfo.header.frame_id is empty");
      return;
    }
    calibration_ready_ = true;
    RCLCPP_INFO(get_logger(), "accepted calibration %dx%d fx=%.3f fy=%.3f model=%s",
                intrinsics_.width, intrinsics_.height, intrinsics_.fx, intrinsics_.fy,
                intrinsics_.distortion_model.empty() ? "none" : intrinsics_.distortion_model.c_str());
  }

  void trim_queues() {
    while (color_queue_.size() > static_cast<size_t>(sync_queue_size_)) {
      color_queue_.pop_front(); ++dropped_; publish_tracking(false);
    }
    while (depth_queue_.size() > static_cast<size_t>(sync_queue_size_)) {
      depth_queue_.pop_front(); ++dropped_; publish_tracking(false);
    }
  }

  void match() {
    if (!parameters_valid_) return;
    while (!color_queue_.empty() && !depth_queue_.empty()) {
      const int64_t color_stamp = stamp_ns(color_queue_.front()->header.stamp);
      auto best = depth_queue_.begin();
      int64_t best_delta = std::llabs(stamp_ns((*best)->header.stamp) - color_stamp);
      for (auto it = std::next(depth_queue_.begin()); it != depth_queue_.end(); ++it) {
        const int64_t delta = std::llabs(stamp_ns((*it)->header.stamp) - color_stamp);
        if (delta >= best_delta) break;
        best = it; best_delta = delta;
      }
      if (best_delta <= sync_slop_ns_) {
        auto color = color_queue_.front();
        auto depth = *best;
        color_queue_.pop_front();
        dropped_ += static_cast<uint64_t>(std::distance(depth_queue_.begin(), best));
        depth_queue_.erase(depth_queue_.begin(), std::next(best));
        process_pair(*color, *depth);
      } else if (stamp_ns(depth_queue_.front()->header.stamp) < color_stamp) {
        depth_queue_.pop_front(); ++dropped_; publish_tracking(false);
      } else {
        color_queue_.pop_front(); ++dropped_; publish_tracking(false);
      }
    }
  }

  bool initialize_tracker() {
    if (!calibration_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for valid CameraInfo");
      return false;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
          rig_frame_, camera_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      rig_T_camera_ = tf2::transformToEigen(transform);
    } catch (const tf2::TransformException &exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "waiting for TF %s <- %s: %s", rig_frame_.c_str(),
                           camera_frame_.c_str(), exception.what());
      return false;
    }
    if (!engine_.configure(intrinsics_, ray_x_, ray_y_)) {
      RCLCPP_ERROR(get_logger(), "failed to configure odometry engine");
      return false;
    }
    tracker_ready_ = true;
    world_T_rig_.setIdentity();
    RCLCPP_INFO(get_logger(), "tracker initialized with fixed TF %s <- %s",
                rig_frame_.c_str(), camera_frame_.c_str());
    return true;
  }

  void process_pair(const sensor_msgs::msg::Image &color,
                    const sensor_msgs::msg::Image &depth) {
    const auto started = std::chrono::steady_clock::now();
    ++paired_;
    const int64_t timestamp = stamp_ns(color.header.stamp);
    if (timestamp <= last_input_stamp_ns_) {
      ++failed_; publish_tracking(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "rejected duplicate or decreasing timestamp");
      if (timestamp < last_input_stamp_ns_) reset_tracking("timestamp moved backwards");
      return;
    }
    last_input_stamp_ns_ = timestamp;
    std::string error;
    if (!validate_color(color, error) || color.width != depth.width ||
        color.height != depth.height) {
      ++failed_; publish_tracking(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "input rejected: %s or RGB/depth size mismatch", error.c_str());
      return;
    }
    if (!calibration_ready_ || color.width != static_cast<uint32_t>(intrinsics_.width) ||
        color.height != static_cast<uint32_t>(intrinsics_.height)) {
      ++failed_; publish_tracking(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "input dimensions do not match current CameraInfo");
      return;
    }
    if (!tracker_ready_ && !initialize_tracker()) {
      ++failed_; publish_tracking(false); return;
    }
    std::vector<float> depth_m;
    if (!decode_depth(depth, static_cast<float>(depth_scale_),
                      static_cast<float>(min_depth_), static_cast<float>(max_depth_),
                      depth_m, error)) {
      ++failed_; publish_tracking(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth rejected: %s", error.c_str());
      return;
    }
    std::vector<uint8_t> gray;
    if (!decode_grayscale(color, gray, error)) {
      ++failed_; publish_tracking(false); return;
    }
    TrackingResult result = engine_.process(depth_m, gray);
    if (!result.valid) {
      ++failed_; publish_tracking(false);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RGB-D tracking failed quality checks");
      return;
    }
    if (successful_ > 0) {
      Eigen::Isometry3d rig_delta =
          rig_T_camera_ * result.previous_T_current * rig_T_camera_.inverse();
      if (planar_mode_) {
        const double yaw = std::atan2(rig_delta.linear()(1, 0), rig_delta.linear()(0, 0));
        Eigen::Vector2d translation = rig_delta.translation().head<2>();
        if (translation.norm() < planar_translation_deadband_m_) translation.setZero();
        rig_delta.setIdentity();
        rig_delta.translation().head<2>() = translation;
        rig_delta.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      }
      world_T_rig_ = world_T_rig_ * rig_delta;
    }
    ++successful_;
    publish_result(color.header.stamp, result);
    publish_tracking(true);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    total_processing_ms_ += elapsed_ms;
    max_processing_ms_ = std::max(max_processing_ms_, elapsed_ms);
  }

  Eigen::Isometry3d output_pose() const {
    if (!planar_mode_) return world_T_rig_;
    Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
    planar.translation().x() = world_T_rig_.translation().x();
    planar.translation().y() = world_T_rig_.translation().y();
    const double yaw = std::atan2(world_T_rig_.linear()(1, 0), world_T_rig_.linear()(0, 0));
    planar.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return planar;
  }

  void publish_result(const builtin_interfaces::msg::Time &stamp,
                      const TrackingResult &result) {
    const Eigen::Isometry3d pose = output_pose();
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = rig_frame_;
    message.pose.pose.position.x = pose.translation().x();
    message.pose.pose.position.y = pose.translation().y();
    message.pose.pose.position.z = pose.translation().z();
    message.pose.pose.orientation = quaternion_message(pose.linear());
    const double position_variance = std::clamp(
        result.rmse_m * result.rmse_m * 4.0 + 1e-5, 1e-5, 0.25);
    const double angle_variance = std::clamp(
        position_variance / 0.25, 4e-5, 1.0);
    message.pose.covariance.fill(0.0);
    message.pose.covariance[0] = position_variance;
    message.pose.covariance[7] = position_variance;
    message.pose.covariance[14] = planar_mode_ ? 1e-6 : position_variance * 2.0;
    message.pose.covariance[21] = planar_mode_ ? 1e-6 : angle_variance;
    message.pose.covariance[28] = planar_mode_ ? 1e-6 : angle_variance;
    message.pose.covariance[35] = angle_variance;
    message.twist.covariance.fill(0.0);
    message.twist.covariance[0] = message.twist.covariance[7] = position_variance * 10.0;
    message.twist.covariance[14] = position_variance * 20.0;
    message.twist.covariance[21] = message.twist.covariance[28] = angle_variance * 10.0;
    message.twist.covariance[35] = angle_variance * 10.0;
    if (has_last_output_) {
      const double dt = static_cast<double>(stamp_ns(stamp) - last_output_stamp_ns_) * 1e-9;
      if (dt > 1e-5) {
        const Eigen::Isometry3d delta = last_output_pose_.inverse() * pose;
        message.twist.twist.linear.x = delta.translation().x() / dt;
        message.twist.twist.linear.y = delta.translation().y() / dt;
        message.twist.twist.linear.z = delta.translation().z() / dt;
        Eigen::AngleAxisd aa(delta.linear());
        const Eigen::Vector3d angular = aa.axis() * aa.angle() / dt;
        message.twist.twist.angular.x = angular.x();
        message.twist.twist.angular.y = angular.y();
        message.twist.twist.angular.z = angular.z();
      }
    }
    last_output_pose_ = pose;
    last_output_stamp_ns_ = stamp_ns(stamp);
    has_last_output_ = true;
    odom_publisher_->publish(message);
    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_message;
      tf_message.header = message.header;
      tf_message.child_frame_id = rig_frame_;
      tf_message.transform.translation.x = pose.translation().x();
      tf_message.transform.translation.y = pose.translation().y();
      tf_message.transform.translation.z = pose.translation().z();
      tf_message.transform.rotation = message.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_message);
    }
  }

  void publish_tracking(bool tracking) {
    if (!tracking_publisher_) return;
    std_msgs::msg::Bool message; message.data = tracking;
    tracking_publisher_->publish(message);
  }

  void reset_tracking(const char *reason) {
    engine_.reset();
    tracker_ready_ = false;
    world_T_rig_.setIdentity();
    has_last_output_ = false;
    RCLCPP_WARN(get_logger(), "tracking reset: %s", reason);
  }

  void report() {
    const double average_ms = successful_ == 0 ? 0.0 : total_processing_ms_ / successful_;
    RCLCPP_INFO(get_logger(),
                "stats color=%lu depth=%lu paired=%lu ok=%lu failed=%lu dropped=%lu avg=%.2fms max=%.2fms RVV=off",
                received_color_, received_depth_, paired_, successful_, failed_, dropped_,
                average_ms, max_processing_ms_);
  }

  bool parameters_valid_{true};
  bool calibration_ready_{false};
  bool tracker_ready_{false};
  bool planar_mode_{true};
  bool publish_tf_{false};
  bool input_reliable_qos_{true};
  int sync_queue_size_{30};
  int qos_depth_{40};
  int64_t sync_slop_ns_{15000000};
  int64_t last_input_stamp_ns_{-1};
  int64_t last_output_stamp_ns_{0};
  double depth_scale_{1000.0};
  double min_depth_{0.2};
  double max_depth_{8.0};
  double report_interval_s_{5.0};
  double planar_translation_deadband_m_{0.0015};
  std::string color_topic_, depth_topic_, camera_info_topic_, output_topic_;
  std::string tracking_topic_, rig_frame_, odom_frame_, camera_frame_;
  Intrinsics intrinsics_;
  std::vector<float> ray_x_, ray_y_;
  Engine engine_;
  Eigen::Isometry3d rig_T_camera_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_T_rig_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d last_output_pose_{Eigen::Isometry3d::Identity()};
  bool has_last_output_{false};
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> color_queue_, depth_queue_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_, depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_publisher_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  uint64_t received_color_{0}, received_depth_{0}, paired_{0}, successful_{0};
  uint64_t failed_{0}, dropped_{0};
  double total_processing_ms_{0.0}, max_processing_ms_{0.0};
};

}  // namespace rgbd_odometry

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rgbd_odometry::RgbdOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
