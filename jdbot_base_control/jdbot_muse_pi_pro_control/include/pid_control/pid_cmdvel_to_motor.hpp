#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <mutex>
#include <string>

#include "motor_direction_ctrl.hpp"
#include "encoder/encoder_speed_meter.hpp"
#include "pid_control/pid_controller.hpp"

class PidCmdVelToMotor : public rclcpp::Node {
public:
  PidCmdVelToMotor();
  ~PidCmdVelToMotor();

private:
  /* ---------- ROS callbacks ---------- */
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void control_timer_callback();
  void odom_timer_callback();

  /* ---------- internal ---------- */
  void send_cmd(double v, double w);
  void publish_odom(double v, double w);
  std::pair<int, double> cmd_to_wheel(double v);
  double to_wheel_linear(int direction, double speed_angular);

  /* ---------- parameters ---------- */
  double control_hz_;
  double timeout_;
  bool publish_tf_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  /* ---------- ROS ---------- */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  /* ---------- state ---------- */
  std::mutex mutex_;
  double cur_v_{0.0};
  double cur_w_{0.0};
  double v_l_{0.0};
  double v_r_{0.0};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  rclcpp::Time last_cmd_time_;
  std::chrono::steady_clock::time_point last_odom_time_;

  /* ---------- hardware ---------- */
  int sock_;
  MotorDirectionCtrl dir_ctrl_;

  /* ---------- encoder ---------- */
  std::unique_ptr<EncoderSpeedMeter> encoder_motor1_; // GPIO 73
  std::unique_ptr<EncoderSpeedMeter> encoder_motor2_; // GPIO 72

  /* PID INIT*/
  PID_Controller motor1_pid_;
  PID_Controller motor2_pid_;
};
