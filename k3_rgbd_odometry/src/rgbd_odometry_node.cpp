#include "k3_rgbd_odometry/loop_closure.hpp"
#include "k3_rgbd_odometry/rgbd_frontend.hpp"

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
#include <vector>

namespace k3_rgbd_odometry {
namespace {

Eigen::Isometry3d fromMessage(const geometry_msgs::msg::Transform& message) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(message.translation.x, message.translation.y,
                                             message.translation.z);
  const Eigen::Quaterniond quaternion(message.rotation.w, message.rotation.x,
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

Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1e-10) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.axis() * angle_axis.angle();
}

}  // namespace

class RgbdOdometryNode : public rclcpp::Node {
 public:
  RgbdOdometryNode() : Node("k3_rgbd_odometry"), tf_buffer_(get_clock()),
                       tf_listener_(tf_buffer_), tf_broadcaster_(*this) {
    color_topic_ = declare_parameter("color_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    camera_info_topic_ = declare_parameter("camera_info_topic", "/camera/color/camera_info");
    odom_topic_ = declare_parameter("odom_topic", "/k3_rgbd_odometry/odom");
    odom_6d_topic_ = declare_parameter("odom_6d_topic", "/k3_rgbd_odometry/odom_6d");
    tracking_topic_ = declare_parameter("tracking_topic", "/k3_rgbd_odometry/tracking");
    inliers_topic_ = declare_parameter("inliers_topic", "/k3_rgbd_odometry/inliers");
    loop_topic_ = declare_parameter("loop_topic", "/k3_rgbd_odometry/loop_closure");
    optimized_path_topic_ =
        declare_parameter("optimized_path_topic", "/k3_rgbd_odometry/optimized_path");
    odom_frame_ = declare_parameter("odom_frame", "odom_k3_rgbd");
    base_frame_ = declare_parameter("base_frame", "base_footprint");
    child_frame_ = declare_parameter("child_frame", "base_footprint_k3_rgbd");
    tf_odom_frame_ = declare_parameter("tf_odom_frame", "odom_k3_rgbd");
    tf_child_frame_ = declare_parameter("tf_child_frame", "base_footprint_k3_rgbd");
    full_child_frame_ = declare_parameter("full_child_frame", "");
    if (full_child_frame_.empty()) full_child_frame_ = base_frame_;
    publish_tf_ = declare_parameter("publish_tf", true);
    processing_rate_hz_ = declare_parameter("processing_rate_hz", 15.0);
    expected_frame_rate_hz_ = declare_parameter("expected_frame_rate_hz", 15.0);
    sync_tolerance_ms_ = declare_parameter("sync_tolerance_ms", 40.0);
    tf_timeout_ms_ = declare_parameter("tf_timeout_ms", 20.0);
    max_translation_per_frame_ = declare_parameter("max_translation_per_frame", 0.50);
    max_rotation_per_frame_ = declare_parameter("max_rotation_per_frame", 0.60);
    planar_translation_scale_ = declare_parameter("planar.translation_scale", 0.95);
    planar_gap_recovery_strength_ =
        declare_parameter("planar.gap_recovery_strength", 0.25);
    planar_lateral_min_gain_ = declare_parameter("planar.lateral_min_gain", 0.0);

    FrontendConfig config;
    config.pyramid_levels = declare_parameter("pyramid_levels", 4);
    config.cell_size = declare_parameter("cell_size", 24);
    config.max_features = declare_parameter("max_features", 420);
    config.min_features = declare_parameter("min_features", 8);
    config.patch_radius = declare_parameter("patch_radius", 3);
    config.klt_iterations = declare_parameter("klt_iterations", 8);
    config.min_depth_m = static_cast<float>(declare_parameter("min_depth_m", 0.25));
    config.max_depth_m = static_cast<float>(declare_parameter("max_depth_m", 6.0));
    config.min_corner_score = static_cast<float>(declare_parameter("min_corner_score", 600.0));
    config.max_patch_rmse = static_cast<float>(declare_parameter("max_patch_rmse", 28.0));
    config.ransac_threshold_m =
        static_cast<float>(declare_parameter("ransac_threshold_m", 0.075));
    config.ransac_iterations = declare_parameter("ransac_iterations", 72);
    config.reprojection_threshold_px =
        static_cast<float>(declare_parameter("reprojection_threshold_px", 3.0));
    config.depth_scale = static_cast<float>(declare_parameter("depth_scale", 0.001));
    frontend_ = std::make_unique<RgbdFrontend>(config);
    config.planar_refinement = true;
    planar_frontend_ = std::make_unique<RgbdFrontend>(config);

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
    odometry_6d_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_6d_topic_, 10);
    tracking_publisher_ = create_publisher<std_msgs::msg::Bool>(tracking_topic_, 10);
    inliers_publisher_ = create_publisher<std_msgs::msg::Int32>(inliers_topic_, 10);
    loop_publisher_ = create_publisher<std_msgs::msg::Bool>(loop_topic_, 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(optimized_path_topic_, 1);
    RCLCPP_INFO(get_logger(),
                "Generic 6DoF RGB-D odometry: %s + %s -> %s and %s (%g Hz max)",
                color_topic_.c_str(), depth_topic_.c_str(), odom_topic_.c_str(),
                odom_6d_topic_.c_str(), processing_rate_hz_);
  }

 private:
  bool lookupBaseFromCamera(const rclcpp::Time& stamp, const std::string& camera_frame,
                            Eigen::Isometry3d& base_from_camera) {
    try {
      const auto transform = tf_buffer_.lookupTransform(
          base_frame_, camera_frame, stamp,
          rclcpp::Duration::from_seconds(std::max(0.0, tf_timeout_ms_) * 1e-3));
      base_from_camera = fromMessage(transform.transform);
      return true;
    } catch (const tf2::TransformException& exact_exception) {
      try {
        const auto latest =
            tf_buffer_.lookupTransform(base_frame_, camera_frame, tf2::TimePointZero);
        base_from_camera = fromMessage(latest.transform);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "No timestamped camera TF; using latest %s <- %s: %s", base_frame_.c_str(),
            camera_frame.c_str(), exact_exception.what());
        return true;
      } catch (const tf2::TransformException& latest_exception) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Waiting for camera TF %s <- %s: %s", base_frame_.c_str(),
                             camera_frame.c_str(), latest_exception.what());
        return false;
      }
    }
  }

  void tryProcess() {
    if (!latest_color_ || !latest_depth_ || !calibrated_) return;
    const rclcpp::Time color_time(latest_color_->header.stamp);
    const rclcpp::Time depth_time(latest_depth_->header.stamp);
    const double difference_ms = std::abs((color_time - depth_time).seconds()) * 1000.0;
    if (difference_ms > sync_tolerance_ms_) return;
    if (color_time.nanoseconds() == last_input_stamp_ns_) return;
    if (last_processed_stamp_ns_ != 0 && processing_rate_hz_ > 0.0 &&
        (color_time.nanoseconds() - last_processed_stamp_ns_) * 1e-9 <
            1.0 / processing_rate_hz_) {
      return;
    }
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

    Eigen::Isometry3d current_base_from_camera = Eigen::Isometry3d::Identity();
    if (!lookupBaseFromCamera(color_time, latest_color_->header.frame_id,
                              current_base_from_camera)) {
      return;
    }
    frontend_->setBaseFromCamera(current_base_from_camera);
    planar_frontend_->setBaseFromCamera(current_base_from_camera);

    const auto* depth = reinterpret_cast<const uint16_t*>(latest_depth_->data.data());
    TrackingResult result = frontend_->process(
        latest_color_->data.data(), static_cast<int>(latest_color_->step), latest_color_->encoding,
        depth, static_cast<int>(latest_depth_->step), camera_);
    TrackingResult planar_result = planar_frontend_->process(
        latest_color_->data.data(), static_cast<int>(latest_color_->step), latest_color_->encoding,
        depth, static_cast<int>(latest_depth_->step), camera_);
    last_input_stamp_ns_ = color_time.nanoseconds();
    last_processed_stamp_ns_ = color_time.nanoseconds();

    const bool tracking_success = result.success || planar_result.success;
    const TrackingResult& quality = result.success ? result : planar_result;
    std_msgs::msg::Bool tracking_message;
    tracking_message.data = tracking_success;
    tracking_publisher_->publish(tracking_message);
    std_msgs::msg::Int32 inliers_message;
    inliers_message.data = quality.inliers;
    inliers_publisher_->publish(inliers_message);

    if (!tracking_success || !have_previous_extrinsic_) {
      previous_base_from_camera_ = current_base_from_camera;
      have_previous_extrinsic_ = true;
      previous_pose_stamp_ns_ = 0;
      if (!tracking_success && result.reason != "initialized") {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                             "Tracking unavailable: 6d=%s planar=%s",
                             result.reason.c_str(), planar_result.reason.c_str());
      }
      return;
    }

    Eigen::Isometry3d base_delta_6d = Eigen::Isometry3d::Identity();
    if (result.success) {
      const Eigen::Isometry3d previous_from_current_camera =
          result.current_from_previous.inverse();
      base_delta_6d = previous_base_from_camera_ * previous_from_current_camera *
          current_base_from_camera.inverse();
    }
    Eigen::Isometry3d planar_measurement = base_delta_6d;
    if (planar_result.success) {
      const Eigen::Isometry3d planar_camera_delta =
          planar_result.current_from_previous.inverse();
      const Eigen::Isometry3d planar_base_delta =
          previous_base_from_camera_ * planar_camera_delta * current_base_from_camera.inverse();
      if (result.success) {
        planar_measurement = selectPlanarMeasurement(base_delta_6d, planar_base_delta, true);
      } else {
        base_delta_6d = planar_base_delta;
        planar_measurement = planar_base_delta;
        nonplanar_confidence_ = 0.0;
      }
    } else {
      planar_measurement = selectPlanarMeasurement(
          base_delta_6d, Eigen::Isometry3d::Identity(), false);
    }
    previous_base_from_camera_ = current_base_from_camera;

    const double translation = base_delta_6d.translation().norm();
    const double rotation = rotationVector(base_delta_6d.rotation()).norm();
    if (!base_delta_6d.matrix().allFinite() || translation > max_translation_per_frame_ ||
        rotation > max_rotation_per_frame_) {
      RCLCPP_WARN(get_logger(), "Rejected implausible 6DoF increment: %.3f m %.2f deg",
                  translation, rotation * 180.0 / M_PI);
      previous_pose_stamp_ns_ = 0;
      return;
    }

    odom_from_base_6d_ = odom_from_base_6d_ * base_delta_6d;
    const Eigen::Isometry3d planar_delta = stabilizePlanarDelta(
        planar_measurement, color_time.nanoseconds());
    odom_from_base_planar_ = odom_from_base_planar_ * planar_delta;
    const LoopClosureResult loop = loop_closure_->update(
        latest_color_->data.data(), static_cast<int>(latest_color_->step), latest_color_->encoding,
        depth, static_cast<int>(latest_depth_->step), camera_, current_base_from_camera,
        odom_from_base_planar_, color_time.nanoseconds(), 1.0);
    raw_path_.push_back({color_time.nanoseconds(), odom_from_base_planar_});
    std_msgs::msg::Bool loop_message;
    loop_message.data = loop.loop_detected;
    loop_publisher_->publish(loop_message);
    if (loop.loop_detected) {
      RCLCPP_INFO(get_logger(), "Loop closure accepted: %d inliers, %d keyframes",
                  loop.loop_inliers, loop.keyframes);
      publishOptimizedPath(latest_color_->header.stamp);
    }

    double dt = 0.0;
    if (previous_pose_stamp_ns_ != 0) {
      dt = (color_time.nanoseconds() - previous_pose_stamp_ns_) * 1e-9;
    }
    previous_pose_stamp_ns_ = color_time.nanoseconds();
    publishOdometry(latest_color_->header.stamp, loop.corrected_pose, planar_delta,
                    base_delta_6d, dt, quality);
    publishTransforms(latest_color_->header.stamp, loop.corrected_pose);
  }

  Eigen::Isometry3d stabilizePlanarDelta(const Eigen::Isometry3d& delta_6d,
                                         int64_t stamp_ns) {
    double interval_scale = 1.0;
    if (last_successful_tracking_stamp_ns_ != 0 && expected_frame_rate_hz_ > 0.0) {
      const double frame_intervals = std::clamp(
          (stamp_ns - last_successful_tracking_stamp_ns_) * 1e-9 * expected_frame_rate_hz_,
          1.0, 35.0);
      interval_scale += planar_gap_recovery_strength_ * (frame_intervals - 1.0);
    }
    last_successful_tracking_stamp_ns_ = stamp_ns;

    const double observed_lateral = delta_6d.translation().y();
    const double bounded_lateral = std::clamp(observed_lateral, -0.06, 0.06);
    constexpr double smoothing = 0.90;
    lateral_trend_ = smoothing * lateral_trend_ + (1.0 - smoothing) * bounded_lateral;
    lateral_activity_ =
        smoothing * lateral_activity_ + (1.0 - smoothing) * std::abs(bounded_lateral);
    const double coherence = std::abs(lateral_trend_) / std::max(1e-5, lateral_activity_);
    const double magnitude_score = std::clamp((lateral_activity_ - 0.003) / 0.009, 0.0, 1.0);
    const double coherence_score = std::clamp((coherence - 0.55) / 0.35, 0.0, 1.0);
    const double target_confidence = magnitude_score * coherence_score;
    const double confidence_rate = target_confidence > lateral_confidence_ ? 0.12 : 0.03;
    lateral_confidence_ += confidence_rate * (target_confidence - lateral_confidence_);
    const double lateral_gain = planar_lateral_min_gain_ +
        (1.0 - planar_lateral_min_gain_) * lateral_confidence_;

    Eigen::Isometry3d planar = Eigen::Isometry3d::Identity();
    planar.translation().x() = interval_scale * planar_translation_scale_ *
        delta_6d.translation().x();
    planar.translation().y() = interval_scale * planar_translation_scale_ *
        lateral_gain * observed_lateral;
    const double yaw = interval_scale * yawOf(delta_6d.rotation());
    planar.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return planar;
  }

  Eigen::Isometry3d selectPlanarMeasurement(const Eigen::Isometry3d& delta_6d,
                                             const Eigen::Isometry3d& planar_delta,
                                             bool planar_valid) {
    if (!planar_valid) {
      nonplanar_confidence_ = 1.0;
      Eigen::Isometry3d selected = Eigen::Isometry3d::Identity();
      selected.translation().head<2>() = delta_6d.translation().head<2>();
      selected.linear() = Eigen::AngleAxisd(
          yawOf(delta_6d.rotation()), Eigen::Vector3d::UnitZ()).toRotationMatrix();
      return selected;
    }
    const Eigen::Vector3d rotation = rotationVector(delta_6d.rotation());
    const double tilt_score = std::clamp(
        (rotation.head<2>().norm() - 0.012) / 0.035, 0.0, 1.0);
    const double vertical_score = std::clamp(
        (std::abs(delta_6d.translation().z()) - 0.006) / 0.025, 0.0, 1.0);
    const double instant_evidence = std::max(tilt_score, vertical_score);
    nonplanar_evidence_ = 0.92 * nonplanar_evidence_ + 0.08 * instant_evidence;
    const double target = std::clamp((nonplanar_evidence_ - 0.25) / 0.50, 0.0, 1.0);
    const double rate = target > nonplanar_confidence_ ? 0.18 : 0.035;
    nonplanar_confidence_ += rate * (target - nonplanar_confidence_);

    const double weight_6d = nonplanar_confidence_;
    const double weight_planar = 1.0 - weight_6d;
    Eigen::Isometry3d selected = Eigen::Isometry3d::Identity();
    selected.translation().x() = weight_planar * planar_delta.translation().x() +
        weight_6d * delta_6d.translation().x();
    selected.translation().y() = weight_planar * planar_delta.translation().y() +
        weight_6d * delta_6d.translation().y();
    const double selected_yaw = weight_planar * yawOf(planar_delta.rotation()) +
        weight_6d * yawOf(delta_6d.rotation());
    selected.linear() =
        Eigen::AngleAxisd(selected_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return selected;
  }

  static double positionVariance(const TrackingResult& tracking) {
    const double inlier_penalty = std::max(1.0, 30.0 / std::max(1, tracking.inliers));
    return std::clamp(tracking.reprojection_rmse * tracking.reprojection_rmse * 2e-4 *
                          inlier_penalty,
                      2e-4, 0.08);
  }

  void publishOdometry(const builtin_interfaces::msg::Time& stamp,
                       const Eigen::Isometry3d& planar_pose,
                       const Eigen::Isometry3d& planar_delta,
                       const Eigen::Isometry3d& delta_6d, double dt,
                       const TrackingResult& tracking) {
    const double position_variance = positionVariance(tracking);

    nav_msgs::msg::Odometry planar;
    planar.header.stamp = stamp;
    planar.header.frame_id = odom_frame_;
    planar.child_frame_id = child_frame_;
    planar.pose.pose.position.x = planar_pose.translation().x();
    planar.pose.pose.position.y = planar_pose.translation().y();
    planar.pose.pose.orientation = toMessage(planar_pose.rotation());
    planar.pose.covariance[0] = position_variance;
    planar.pose.covariance[7] = position_variance;
    planar.pose.covariance[14] = 1e3;
    planar.pose.covariance[21] = 1e3;
    planar.pose.covariance[28] = 1e3;
    planar.pose.covariance[35] = std::max(5e-4, position_variance * 2.0);
    if (dt > 1e-3) {
      planar.twist.twist.linear.x = planar_delta.translation().x() / dt;
      planar.twist.twist.linear.y = planar_delta.translation().y() / dt;
      planar.twist.twist.angular.z = yawOf(planar_delta.rotation()) / dt;
    }
    odometry_publisher_->publish(planar);

    nav_msgs::msg::Odometry full;
    full.header = planar.header;
    full.child_frame_id = full_child_frame_;
    full.pose.pose.position.x = odom_from_base_6d_.translation().x();
    full.pose.pose.position.y = odom_from_base_6d_.translation().y();
    full.pose.pose.position.z = odom_from_base_6d_.translation().z();
    full.pose.pose.orientation = toMessage(odom_from_base_6d_.rotation());
    full.pose.covariance[0] = position_variance;
    full.pose.covariance[7] = position_variance;
    full.pose.covariance[14] = position_variance * 1.5;
    full.pose.covariance[21] = position_variance * 2.0;
    full.pose.covariance[28] = position_variance * 2.0;
    full.pose.covariance[35] = position_variance * 2.0;
    if (dt > 1e-3) {
      const Eigen::Vector3d angular = rotationVector(delta_6d.rotation()) / dt;
      full.twist.twist.linear.x = delta_6d.translation().x() / dt;
      full.twist.twist.linear.y = delta_6d.translation().y() / dt;
      full.twist.twist.linear.z = delta_6d.translation().z() / dt;
      full.twist.twist.angular.x = angular.x();
      full.twist.twist.angular.y = angular.y();
      full.twist.twist.angular.z = angular.z();
    }
    odometry_6d_publisher_->publish(full);
  }

  void publishTransforms(const builtin_interfaces::msg::Time& stamp,
                         const Eigen::Isometry3d& planar_pose) {
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = tf_odom_frame_;
      transform.child_frame_id = tf_child_frame_;
      transform.transform.translation.x = planar_pose.translation().x();
      transform.transform.translation.y = planar_pose.translation().y();
      transform.transform.rotation = toMessage(planar_pose.rotation());
      tf_broadcaster_.sendTransform(transform);
    }
  }

  void publishOptimizedPath(const builtin_interfaces::msg::Time& stamp) {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = odom_frame_;
    for (const auto& raw_pose : raw_path_) {
      const Eigen::Isometry3d corrected =
          loop_closure_->correctHistoricalPose(raw_pose.pose, raw_pose.stamp_ns);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = odom_frame_;
      pose.header.stamp.sec = static_cast<int32_t>(raw_pose.stamp_ns / 1000000000LL);
      pose.header.stamp.nanosec =
          static_cast<uint32_t>(raw_pose.stamp_ns % 1000000000LL);
      pose.pose.position.x = corrected.translation().x();
      pose.pose.position.y = corrected.translation().y();
      pose.pose.orientation = toMessage(corrected.rotation());
      path.poses.push_back(std::move(pose));
    }
    path_publisher_->publish(path);
  }

  std::string color_topic_, depth_topic_, camera_info_topic_;
  std::string odom_topic_, odom_6d_topic_;
  std::string tracking_topic_, inliers_topic_, loop_topic_, optimized_path_topic_;
  std::string odom_frame_, base_frame_, child_frame_, full_child_frame_;
  std::string tf_odom_frame_, tf_child_frame_;
  bool publish_tf_ = true;
  double processing_rate_hz_ = 15.0;
  double expected_frame_rate_hz_ = 15.0;
  double sync_tolerance_ms_ = 40.0;
  double tf_timeout_ms_ = 20.0;
  double max_translation_per_frame_ = 0.50;
  double max_rotation_per_frame_ = 0.60;
  double planar_translation_scale_ = 0.95;
  double planar_gap_recovery_strength_ = 0.25;
  double planar_lateral_min_gain_ = 0.0;
  double lateral_trend_ = 0.0;
  double lateral_activity_ = 0.0;
  double lateral_confidence_ = 0.0;
  double nonplanar_confidence_ = 0.0;
  double nonplanar_evidence_ = 0.0;

  CameraModel camera_;
  bool calibrated_ = false;
  bool have_previous_extrinsic_ = false;
  Eigen::Isometry3d previous_base_from_camera_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d odom_from_base_6d_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d odom_from_base_planar_ = Eigen::Isometry3d::Identity();
  std::vector<OptimizedGraphPose> raw_path_;
  int64_t last_input_stamp_ns_ = 0;
  int64_t last_processed_stamp_ns_ = 0;
  int64_t previous_pose_stamp_ns_ = 0;
  int64_t last_successful_tracking_stamp_ns_ = 0;

  std::unique_ptr<RgbdFrontend> frontend_;
  std::unique_ptr<RgbdFrontend> planar_frontend_;
  std::unique_ptr<LoopClosure> loop_closure_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_6d_publisher_;
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
