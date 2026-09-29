#include "k3_rgbd_odometry/rgbd_frontend.hpp"
#include "k3_rgbd_odometry/loop_closure.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace k3_rgbd_odometry {
namespace {

Eigen::Isometry3d fromMessage(const geometry_msgs::msg::Transform& message) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(message.translation.x, message.translation.y,
                                             message.translation.z);
  Eigen::Quaterniond quaternion(message.rotation.w, message.rotation.x,
                                 message.rotation.y, message.rotation.z);
  transform.linear() = quaternion.normalized().toRotationMatrix();
  return transform;
}

geometry_msgs::msg::Quaternion toMessage(const Eigen::Matrix3d& rotation) {
  const Eigen::Quaterniond q(rotation);
  geometry_msgs::msg::Quaternion message;
  message.w = q.w();
  message.x = q.x();
  message.y = q.y();
  message.z = q.z();
  return message;
}

double yawOf(const Eigen::Matrix3d& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

}  // namespace

class RgbdOdometryNode : public rclcpp::Node {
 public:
  RgbdOdometryNode() : Node("k3_rgbd_odometry"), tf_buffer_(get_clock()),
                       tf_listener_(tf_buffer_), tf_broadcaster_(*this) {
    color_topic_ = declare_parameter("color_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    camera_info_topic_ = declare_parameter("camera_info_topic", "/camera/color/camera_info");
    odom_topic_ = declare_parameter("odom_topic", "/rgbd_odom/odom");
    odom_frame_ = declare_parameter("odom_frame", "odom_rgbd");
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    child_frame_ = declare_parameter("child_frame", "base_footprint_rgbd");
    publish_tf_ = declare_parameter("publish_tf", true);
    processing_rate_hz_ = declare_parameter("processing_rate_hz", 15.0);
    expected_frame_rate_hz_ = declare_parameter("expected_frame_rate_hz", 15.0);
    // D415 color and aligned-depth streams can be phase shifted even when both
    // are configured at 15 Hz.  Half a frame keeps the nearest pair while not
    // requiring message_filters (and its extra queues/copies) on the K3.
    sync_tolerance_ms_ = declare_parameter("sync_tolerance_ms", 40.0);
    max_translation_per_frame_ = declare_parameter("max_translation_per_frame", 0.35);
    max_rotation_per_frame_ = declare_parameter("max_rotation_per_frame", 0.35);
    yaw_gain_ = declare_parameter("yaw_gain", 1.0);
    lateral_gain_ = declare_parameter("lateral_gain", 0.0);
    forward_gain_ = declare_parameter("forward_gain", 0.95);
    gap_recovery_gain_ = declare_parameter("gap_recovery_gain", 0.25);

    FrontendConfig config;
    config.pyramid_levels = declare_parameter("pyramid_levels", 4);
    config.cell_size = declare_parameter("cell_size", 24);
    config.max_features = declare_parameter("max_features", 420);
    config.min_features = declare_parameter("min_features", 45);
    config.patch_radius = declare_parameter("patch_radius", 3);
    config.klt_iterations = declare_parameter("klt_iterations", 8);
    config.min_depth_m = static_cast<float>(declare_parameter("min_depth_m", 0.25));
    config.max_depth_m = static_cast<float>(declare_parameter("max_depth_m", 6.0));
    config.min_corner_score = static_cast<float>(declare_parameter("min_corner_score", 900.0));
    config.max_patch_rmse = static_cast<float>(declare_parameter("max_patch_rmse", 28.0));
    config.ransac_threshold_m =
        static_cast<float>(declare_parameter("ransac_threshold_m", 0.075));
    config.ransac_iterations = declare_parameter("ransac_iterations", 72);
    config.reprojection_threshold_px =
        static_cast<float>(declare_parameter("reprojection_threshold_px", 3.0));
    config.depth_scale = static_cast<float>(declare_parameter("depth_scale", 0.001));
    config.photometric_refinement = declare_parameter("photometric_refinement", false);
    frontend_ = std::make_unique<RgbdFrontend>(config);
    LoopClosureConfig loop_config;
    loop_config.enabled = declare_parameter("loop_closure.enabled", true);
    loop_config.keyframe_translation_m =
        declare_parameter("loop_closure.keyframe_translation_m", 0.45);
    loop_config.keyframe_rotation_rad =
        declare_parameter("loop_closure.keyframe_rotation_rad", 0.32);
    loop_config.min_keyframe_separation =
        declare_parameter("loop_closure.min_keyframe_separation", 10);
    loop_config.max_features = declare_parameter("loop_closure.max_features", 320);
    loop_config.descriptor_distance =
        declare_parameter("loop_closure.descriptor_distance", 80);
    loop_config.min_matches = declare_parameter("loop_closure.min_matches", 16);
    loop_config.min_inliers = declare_parameter("loop_closure.min_inliers", 12);
    loop_config.ransac_threshold_m =
        declare_parameter("loop_closure.ransac_threshold_m", 0.15);
    loop_closure_ = std::make_unique<LoopClosure>(loop_config);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(3);
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic_, sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          latest_color_ = std::move(message);
          tryProcess();
        });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          latest_depth_ = std::move(message);
          tryProcess();
        });
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, sensor_qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
          camera_.width = static_cast<int>(message->width);
          camera_.height = static_cast<int>(message->height);
          camera_.fx = static_cast<float>(message->k[0]);
          camera_.fy = static_cast<float>(message->k[4]);
          camera_.cx = static_cast<float>(message->k[2]);
          camera_.cy = static_cast<float>(message->k[5]);
          calibrated_ = camera_.fx > 0.0F && camera_.fy > 0.0F;
        });

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tracking_publisher_ = create_publisher<std_msgs::msg::Bool>("~/tracking", 10);
    inliers_publisher_ = create_publisher<std_msgs::msg::Int32>("~/inliers", 10);
    loop_publisher_ = create_publisher<std_msgs::msg::Bool>("~/loop_closure", 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("~/optimized_path", 1);
    RCLCPP_INFO(get_logger(),
                "OpenCV-free RGB-D odometry: %s + %s -> %s (%g Hz max)",
                color_topic_.c_str(), depth_topic_.c_str(), odom_topic_.c_str(),
                processing_rate_hz_);
  }

 private:
  void tryProcess() {
    if (!latest_color_ || !latest_depth_ || !calibrated_) return;
    const rclcpp::Time color_time(latest_color_->header.stamp);
    const rclcpp::Time depth_time(latest_depth_->header.stamp);
    const double difference_ms = std::abs((color_time - depth_time).seconds()) * 1000.0;
    if (difference_ms > sync_tolerance_ms_) return;
    if (color_time.nanoseconds() == last_input_stamp_ns_) return;
    if (last_processed_stamp_ns_ != 0 && processing_rate_hz_ > 0.0 &&
        (color_time.nanoseconds() - last_processed_stamp_ns_) * 1e-9 <
            1.0 / processing_rate_hz_) return;
    if (latest_depth_->encoding != "16UC1" && latest_depth_->encoding != "mono16") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Depth encoding must be 16UC1/mono16, got %s",
                            latest_depth_->encoding.c_str());
      return;
    }
    if (latest_color_->width != latest_depth_->width ||
        latest_color_->height != latest_depth_->height ||
        static_cast<int>(latest_color_->width) != camera_.width || latest_depth_->is_bigendian) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Color/depth dimensions, calibration, or byte order are incompatible");
      return;
    }
    if (!have_extrinsic_) {
      try {
        const auto stamped = tf_buffer_.lookupTransform(
            base_frame_, latest_color_->header.frame_id, tf2::TimePointZero);
        base_from_camera_ = fromMessage(stamped.transform);
        frontend_->setBaseFromCamera(base_from_camera_);
        have_extrinsic_ = true;
        RCLCPP_INFO(get_logger(), "Using TF extrinsic %s <- %s", base_frame_.c_str(),
                    latest_color_->header.frame_id.c_str());
      } catch (const tf2::TransformException& exception) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Waiting for camera extrinsic: %s", exception.what());
        return;
      }
    }

    const auto* depth = reinterpret_cast<const uint16_t*>(latest_depth_->data.data());
    TrackingResult result = frontend_->process(
        latest_color_->data.data(), static_cast<int>(latest_color_->step), latest_color_->encoding,
        depth, static_cast<int>(latest_depth_->step), camera_);
    last_input_stamp_ns_ = color_time.nanoseconds();
    last_processed_stamp_ns_ = color_time.nanoseconds();

    std_msgs::msg::Bool tracking_message;
    tracking_message.data = result.success;
    tracking_publisher_->publish(tracking_message);
    std_msgs::msg::Int32 inliers_message;
    inliers_message.data = result.inliers;
    inliers_publisher_->publish(inliers_message);

    if (!result.success) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                           "Tracking unavailable: %s (corners=%d tracks=%d depth=%d)",
                           result.reason.c_str(), result.detected, result.tracked,
                           result.depth_pairs);
      previous_pose_stamp_ns_ = 0;
      return;
    }

    const Eigen::Isometry3d previous_from_current_camera =
        result.current_from_previous.inverse();
    const Eigen::Isometry3d base_delta_6d =
        base_from_camera_ * previous_from_current_camera * base_from_camera_.inverse();
    double interval_scale = 1.0;
    if (last_successful_tracking_stamp_ns_ != 0 && expected_frame_rate_hz_ > 0.0) {
      const double frame_intervals = std::clamp(
          (color_time.nanoseconds() - last_successful_tracking_stamp_ns_) * 1e-9 *
              expected_frame_rate_hz_,
          1.0, 35.0);
      interval_scale += gap_recovery_gain_ * (frame_intervals - 1.0);
    }
    const double dx = interval_scale * forward_gain_ * base_delta_6d.translation().x();
    // Suppress visually induced lateral slip while retaining a tunable amount
    // for calibration of a non-ideal differential-drive base.
    const double dy = interval_scale * lateral_gain_ * base_delta_6d.translation().y();
    const double delta_yaw = interval_scale * yaw_gain_ * yawOf(base_delta_6d.rotation());
    if (std::hypot(dx, dy) > max_translation_per_frame_ ||
        std::abs(delta_yaw) > max_rotation_per_frame_) {
      RCLCPP_WARN(get_logger(), "Rejected implausible increment: %.3f m %.2f deg",
                  std::hypot(dx, dy), delta_yaw * 180.0 / M_PI);
      previous_pose_stamp_ns_ = 0;
      return;
    }
    Eigen::Isometry3d planar_delta = Eigen::Isometry3d::Identity();
    planar_delta.translation() = Eigen::Vector3d(dx, dy, 0.0);
    planar_delta.linear() =
        Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    raw_world_from_base_ = raw_world_from_base_ * planar_delta;
    last_successful_tracking_stamp_ns_ = color_time.nanoseconds();
    const double odometry_information = 1.0;
    const LoopClosureResult loop = loop_closure_->update(
        latest_color_->data.data(), static_cast<int>(latest_color_->step), latest_color_->encoding,
        depth, static_cast<int>(latest_depth_->step), camera_, base_from_camera_,
        raw_world_from_base_, color_time.nanoseconds(), odometry_information);
    world_from_base_ = loop.corrected_pose;
    std_msgs::msg::Bool loop_message;
    loop_message.data = loop.loop_detected;
    loop_publisher_->publish(loop_message);
    if (loop.loop_detected) {
      RCLCPP_INFO(get_logger(), "Loop closure accepted: %d inliers, %d keyframes",
                  loop.loop_inliers, loop.keyframes);
      nav_msgs::msg::Path path;
      path.header.stamp = latest_color_->header.stamp;
      path.header.frame_id = odom_frame_;
      for (const auto& graph_pose : loop_closure_->optimizedPath()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = odom_frame_;
        pose.header.stamp.sec = static_cast<int32_t>(graph_pose.stamp_ns / 1000000000LL);
        pose.header.stamp.nanosec = static_cast<uint32_t>(graph_pose.stamp_ns % 1000000000LL);
        pose.pose.position.x = graph_pose.pose.translation().x();
        pose.pose.position.y = graph_pose.pose.translation().y();
        pose.pose.orientation = toMessage(graph_pose.pose.rotation());
        path.poses.push_back(std::move(pose));
      }
      path_publisher_->publish(path);
    }

    double dt = 0.0;
    if (previous_pose_stamp_ns_ != 0) {
      dt = (color_time.nanoseconds() - previous_pose_stamp_ns_) * 1e-9;
    }
    previous_pose_stamp_ns_ = color_time.nanoseconds();
    publishPose(latest_color_->header.stamp, planar_delta, dt, result);
  }

  void publishPose(const builtin_interfaces::msg::Time& stamp,
                   const Eigen::Isometry3d& delta, double dt,
                   const TrackingResult& tracking) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = child_frame_;
    odometry.pose.pose.position.x = world_from_base_.translation().x();
    odometry.pose.pose.position.y = world_from_base_.translation().y();
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = toMessage(world_from_base_.rotation());
    const double position_variance = std::clamp(
        tracking.reprojection_rmse * tracking.reprojection_rmse * 2e-4, 2e-4, 0.04);
    odometry.pose.covariance[0] = position_variance;
    odometry.pose.covariance[7] = position_variance;
    odometry.pose.covariance[14] = 1e3;
    odometry.pose.covariance[21] = 1e3;
    odometry.pose.covariance[28] = 1e3;
    odometry.pose.covariance[35] = std::max(5e-4, position_variance * 2.0);
    if (dt > 1e-3) {
      odometry.twist.twist.linear.x = delta.translation().x() / dt;
      odometry.twist.twist.linear.y = delta.translation().y() / dt;
      odometry.twist.twist.angular.z = yawOf(delta.rotation()) / dt;
    }
    odometry_publisher_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = child_frame_;
      transform.transform.translation.x = world_from_base_.translation().x();
      transform.transform.translation.y = world_from_base_.translation().y();
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }
  }

  std::string color_topic_, depth_topic_, camera_info_topic_, odom_topic_;
  std::string odom_frame_, base_frame_, child_frame_;
  bool publish_tf_ = true;
  double processing_rate_hz_ = 15.0;
  double expected_frame_rate_hz_ = 15.0;
  double sync_tolerance_ms_ = 40.0;
  double max_translation_per_frame_ = 0.35;
  double max_rotation_per_frame_ = 0.35;
  double yaw_gain_ = 1.0;
  double lateral_gain_ = 0.0;
  double forward_gain_ = 0.95;
  double gap_recovery_gain_ = 0.25;

  CameraModel camera_;
  bool calibrated_ = false;
  bool have_extrinsic_ = false;
  Eigen::Isometry3d base_from_camera_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d raw_world_from_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d world_from_base_ = Eigen::Isometry3d::Identity();
  int64_t last_input_stamp_ns_ = 0;
  int64_t last_processed_stamp_ns_ = 0;
  int64_t previous_pose_stamp_ns_ = 0;
  int64_t last_successful_tracking_stamp_ns_ = 0;

  std::unique_ptr<RgbdFrontend> frontend_;
  std::unique_ptr<LoopClosure> loop_closure_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr inliers_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loop_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace k3_rgbd_odometry

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<k3_rgbd_odometry::RgbdOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
