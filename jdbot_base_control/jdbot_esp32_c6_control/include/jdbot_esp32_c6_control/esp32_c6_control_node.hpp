/**
 * ESP32-C6 底盘控制节点 (C++ 实现)
 *
 * 完全按照 jdbot_base_bringup/jdbot_base_bringup/ros2_ctrl_odom_node.py
 * 的逻辑改写为 C++ 版本。
 *
 * - 订阅 cmd_vel，换算成左右轮指令下发到串口
 * - 读取串口轮速反馈，在 Linux 端计算并发布 odom 与 odom->base TF
 * - 通过 termios 直连串口，无额外依赖
 */

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace jdbot_esp32_c6_control {

class Esp32C6ControlNode : public rclcpp::Node {
 public:
  Esp32C6ControlNode();
  ~Esp32C6ControlNode() override;

 private:
  // ---- 串口 ----
  bool serial_open(const std::string& port, int baudrate);
  void serial_close();
  void serial_read_thread();
  void serial_write(const std::string& data);

  // ---- 下发参数配置 ----
  void send_cfg();

  // ---- cmd_vel 回调 ----
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // ---- 定时器 ----
  void send_timer_callback();
  void odom_timer_callback();

  // ---- 辅助 ----
  double apply_cmd_vel_deadzone(double value, double min_abs_value);
  void send_cmd(double v, double w);
  std::pair<int, double> wheel_speed_to_cmd(double v);
  void parse_feedback(const std::string& text);
  double parse_motor(const std::string& s);
  void publish_odom(double v, double w);

  // ---- 通用参数 ----
  std::string serial_port_{"/dev/jdbot"};
  int baudrate_{115200};
  double send_hz_{20.0};
  double odom_hz_{50.0};
  double cmd_vel_timeout_{0.4};
  double cmd_vel_deadzone_threshold_{0.001};
  double min_linear_speed_{0.05};
  double min_angular_speed_{0.26};
  double wheel_diameter_{0.067};
  double wheel_base_{0.183};
  double motor1_factor_{1.0};
  double motor2_factor_{1.0};
  int feedback_pwm_deadzone_{0};
  double encoder_ppr_{1000.0};
  double reduction_ratio_{56.0};
  double ff_factor_{310.0};
  double pid_kp_{10.0};
  double pid_ki_{70.0};
  double pid_kd_{0.0};
  double straight_balance_kp_{25.0};
  double straight_balance_ki_{8.0};
  double straight_balance_kd_{0.0};
  bool cfg_send_on_startup_{true};
  bool debug_{false};

  // ---- odom 参数 ----
  bool publish_tf_{true};
  std::string odom_topic_{"odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  // ---- ROS 接口 ----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  // ---- 串口 ----
  int serial_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread read_thread_;

  // ---- cmd_vel 状态 ----
  std::mutex cmd_mutex_;
  double cur_v_{0.0};
  double cur_w_{0.0};
  std::chrono::steady_clock::time_point last_cmd_time_;

  // ---- 反馈状态（来自串口）----
  std::mutex feedback_mutex_;
  double v_l_{0.0};
  double v_r_{0.0};
  double power_voltage_{0.0};

  // ---- odom 状态 ----
  std::mutex odom_mutex_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  std::chrono::steady_clock::time_point last_odom_time_;
};

}  // namespace jdbot_esp32_c6_control
