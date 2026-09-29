#include "k3_rgbd_odometry/loop_closure.hpp"
#include "k3_rgbd_odometry/rgbd_frontend.hpp"

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <string>

namespace {

template <typename Message>
Message deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message) {
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  Message message;
  rclcpp::Serialization<Message> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

int64_t stampNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

Eigen::Isometry3d transform(const Eigen::Vector3d& translation,
                            const Eigen::Quaterniond& quaternion) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = translation;
  value.linear() = quaternion.normalized().toRotationMatrix();
  return value;
}

double yaw(const Eigen::Isometry3d& pose) {
  return std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: offline_rgbd_evaluator BAG OUTPUT.csv\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  rosbag2_cpp::Reader reader;
  reader.open(argv[1]);

  const Eigen::Isometry3d base_from_link = transform(
      Eigen::Vector3d(0.065, 0.02, 0.285), Eigen::Quaterniond::Identity());
  const Eigen::Isometry3d link_from_color = transform(
      Eigen::Vector3d(6.963906344026327e-05, 0.014911666512489319,
                      -5.126022006152198e-05),
      Eigen::Quaterniond(0.9999925494194031, -0.0009792504133656619,
                         -0.0010582504328340292, -0.003579868469387293));
  const Eigen::Isometry3d color_from_optical = transform(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5));
  const Eigen::Isometry3d base_from_camera =
      base_from_link * link_from_color * color_from_optical;

  k3_rgbd_odometry::FrontendConfig frontend_config;
  frontend_config.min_features = 8;
  frontend_config.min_corner_score = 600.0F;
  k3_rgbd_odometry::RgbdFrontend frontend(frontend_config);
  frontend.setBaseFromCamera(base_from_camera);
  frontend_config.planar_refinement = true;
  k3_rgbd_odometry::RgbdFrontend planar_frontend(frontend_config);
  planar_frontend.setBaseFromCamera(base_from_camera);
  k3_rgbd_odometry::LoopClosure loop;

  sensor_msgs::msg::Image color, depth;
  bool have_color = false, have_depth = false, have_camera = false;
  k3_rgbd_odometry::CameraModel camera;
  int64_t last_processed = 0;
  Eigen::Isometry3d full_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d planar_pose = Eigen::Isometry3d::Identity();
  std::map<std::string, int> tracking_outcomes;
  int64_t last_success_stamp = 0;
  double lateral_trend = 0.0;
  double lateral_activity = 0.0;
  double lateral_confidence = 0.0;
  double nonplanar_confidence = 0.0;
  double nonplanar_evidence = 0.0;
  std::vector<k3_rgbd_odometry::OptimizedGraphPose> raw_path;
  std::ofstream output(argv[2]);
  output << "time_ns,x,y,yaw,raw_x,raw_y,raw_yaw,full_x,full_y,full_z,loop,keyframes,loop_inliers,tracking_inliers\n";

  auto process = [&]() {
    if (!have_color || !have_depth || !have_camera) return;
    const int64_t color_stamp = stampNs(color.header.stamp);
    const int64_t depth_stamp = stampNs(depth.header.stamp);
    if (std::llabs(color_stamp - depth_stamp) > 18000000LL ||
        color_stamp == last_processed ||
        (last_processed != 0 && color_stamp - last_processed < 66000000LL)) return;
    last_processed = color_stamp;
    const auto* depth_data = reinterpret_cast<const uint16_t*>(depth.data.data());
    auto tracking = frontend.process(color.data.data(), static_cast<int>(color.step),
                                     color.encoding, depth_data, static_cast<int>(depth.step),
                                     camera);
    auto planar_tracking = planar_frontend.process(
        color.data.data(), static_cast<int>(color.step), color.encoding, depth_data,
        static_cast<int>(depth.step), camera);
    ++tracking_outcomes[tracking.reason];
    if (!tracking.success && !planar_tracking.success) return;
    Eigen::Isometry3d base_delta_6d = Eigen::Isometry3d::Identity();
    if (tracking.success) {
      const Eigen::Isometry3d camera_delta = tracking.current_from_previous.inverse();
      base_delta_6d = base_from_camera * camera_delta * base_from_camera.inverse();
    }
    Eigen::Isometry3d selected_delta = base_delta_6d;
    if (planar_tracking.success) {
      const Eigen::Isometry3d planar_camera_delta =
          planar_tracking.current_from_previous.inverse();
      const Eigen::Isometry3d planar_base_delta =
          base_from_camera * planar_camera_delta * base_from_camera.inverse();
      if (tracking.success) {
        const Eigen::AngleAxisd angle_axis(base_delta_6d.rotation());
        const Eigen::Vector3d rotation = angle_axis.axis() * angle_axis.angle();
        const double tilt_score = std::clamp(
            (rotation.head<2>().norm() - 0.012) / 0.035, 0.0, 1.0);
        const double vertical_score = std::clamp(
            (std::abs(base_delta_6d.translation().z()) - 0.006) / 0.025, 0.0, 1.0);
        const double instant_evidence = std::max(tilt_score, vertical_score);
        nonplanar_evidence = 0.92 * nonplanar_evidence + 0.08 * instant_evidence;
        const double target = std::clamp(
            (nonplanar_evidence - 0.25) / 0.50, 0.0, 1.0);
        const double rate = target > nonplanar_confidence ? 0.18 : 0.035;
        nonplanar_confidence += rate * (target - nonplanar_confidence);
        selected_delta.translation().x() =
            (1.0 - nonplanar_confidence) * planar_base_delta.translation().x() +
            nonplanar_confidence * base_delta_6d.translation().x();
        selected_delta.translation().y() =
            (1.0 - nonplanar_confidence) * planar_base_delta.translation().y() +
            nonplanar_confidence * base_delta_6d.translation().y();
        selected_delta.translation().z() = 0.0;
        const double selected_yaw =
            (1.0 - nonplanar_confidence) * yaw(planar_base_delta) +
            nonplanar_confidence * yaw(base_delta_6d);
        selected_delta.linear() = Eigen::AngleAxisd(
            selected_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      } else {
        base_delta_6d = planar_base_delta;
        selected_delta = planar_base_delta;
        nonplanar_confidence = 0.0;
      }
    } else {
      nonplanar_confidence = 1.0;
    }
    double interval_scale = 1.0;
    if (last_success_stamp != 0) {
      const double frame_intervals = std::clamp(
          static_cast<double>(color_stamp - last_success_stamp) / 67190000.0, 1.0, 35.0);
      interval_scale += 0.25 * (frame_intervals - 1.0);
    }
    last_success_stamp = color_stamp;
    const double delta_yaw = interval_scale * yaw(selected_delta);
    if (std::hypot(base_delta_6d.translation().x(), base_delta_6d.translation().y()) > 0.35 ||
        std::abs(delta_yaw) > 0.35) return;
    full_pose = full_pose * base_delta_6d;

    const double observed_lateral = selected_delta.translation().y();
    const double bounded_lateral = std::clamp(observed_lateral, -0.06, 0.06);
    constexpr double smoothing = 0.90;
    lateral_trend = smoothing * lateral_trend + (1.0 - smoothing) * bounded_lateral;
    lateral_activity =
        smoothing * lateral_activity + (1.0 - smoothing) * std::abs(bounded_lateral);
    const double coherence = std::abs(lateral_trend) / std::max(1e-5, lateral_activity);
    const double magnitude_score = std::clamp((lateral_activity - 0.003) / 0.009, 0.0, 1.0);
    const double coherence_score = std::clamp((coherence - 0.55) / 0.35, 0.0, 1.0);
    const double target_confidence = magnitude_score * coherence_score;
    const double confidence_rate = target_confidence > lateral_confidence ? 0.12 : 0.03;
    lateral_confidence += confidence_rate * (target_confidence - lateral_confidence);
    const double lateral_gain = lateral_confidence;

    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.translation() = Eigen::Vector3d(
        interval_scale * 0.95 * selected_delta.translation().x(),
        interval_scale * 0.95 * lateral_gain * observed_lateral, 0.0);
    delta.linear() = Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    planar_pose = planar_pose * delta;
    raw_path.push_back({color_stamp, planar_pose});
    const double odometry_information = 1.0;
    const auto closure = loop.update(color.data.data(), static_cast<int>(color.step),
                                     color.encoding, depth_data, static_cast<int>(depth.step),
                                     camera, base_from_camera, planar_pose, color_stamp,
                                     odometry_information);
    const Eigen::Isometry3d pose = closure.corrected_pose;
    output << color_stamp << ',' << pose.translation().x() << ',' << pose.translation().y()
           << ',' << yaw(pose) << ',' << planar_pose.translation().x() << ','
           << planar_pose.translation().y() << ',' << yaw(planar_pose) << ','
           << full_pose.translation().x() << ',' << full_pose.translation().y() << ','
           << full_pose.translation().z() << ','
           << (closure.loop_detected ? 1 : 0) << ','
           << closure.keyframes << ',' << closure.loop_inliers << ','
           << (tracking.success ? tracking.inliers : planar_tracking.inliers) << '\n';
  };

  while (reader.has_next()) {
    auto message = reader.read_next();
    if (message->topic_name == "/camera/color/image_raw") {
      color = deserialize<sensor_msgs::msg::Image>(message);
      have_color = true;
      process();
    } else if (message->topic_name == "/camera/aligned_depth_to_color/image_raw") {
      depth = deserialize<sensor_msgs::msg::Image>(message);
      have_depth = true;
      process();
    } else if (message->topic_name == "/camera/color/camera_info") {
      const auto info = deserialize<sensor_msgs::msg::CameraInfo>(message);
      camera.width = static_cast<int>(info.width);
      camera.height = static_cast<int>(info.height);
      camera.fx = static_cast<float>(info.k[0]); camera.fy = static_cast<float>(info.k[4]);
      camera.cx = static_cast<float>(info.k[2]); camera.cy = static_cast<float>(info.k[5]);
      have_camera = true;
    }
  }
  std::ofstream graph_output(std::string(argv[2]) + ".graph.csv");
  graph_output << "time_ns,x,y,yaw\n";
  for (const auto& graph_pose : loop.optimizedPath()) {
    graph_output << graph_pose.stamp_ns << ',' << graph_pose.pose.translation().x() << ','
                 << graph_pose.pose.translation().y() << ',' << yaw(graph_pose.pose) << '\n';
  }
  std::ofstream optimized_output(std::string(argv[2]) + ".optimized.csv");
  optimized_output << "time_ns,x,y,yaw\n";
  for (const auto& raw : raw_path) {
    const Eigen::Isometry3d corrected = loop.correctHistoricalPose(raw.pose, raw.stamp_ns);
    optimized_output << raw.stamp_ns << ',' << corrected.translation().x() << ','
                     << corrected.translation().y() << ',' << yaw(corrected) << '\n';
  }
  for (const auto& outcome : tracking_outcomes) {
    std::cerr << outcome.first << '=' << outcome.second << '\n';
  }
  rclcpp::shutdown();
  return 0;
}
