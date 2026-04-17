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

namespace jdbot_k3_esos_control {

/**
 * @brief 旧协议电机控制节点
 *
 * 通过 RPMsg 使用旧协议与 RCPU 通信，在 Linux 端计算里程计
 *
 * 发送格式: "dir1,speed1;dir2,speed2"
 *   - dir: 0=停止, 1=正转, 2=反转
 *   - speed: 轮速 (转/秒)
 *
 * 接收格式: "dir1,speed1_mrs;dir2,speed2_mrs"
 *   - speed_mrs: 实际轮速 (毫弧度/秒)
 */
class RpmsgLegacyNode : public rclcpp::Node {
public:
  RpmsgLegacyNode();
  ~RpmsgLegacyNode() override;

private:
  /* ---------- RPMsg 操作 ---------- */
  bool rpmsg_init();
  void rpmsg_cleanup();
  void recv_thread_func();

  /* ---------- 命令发送 ---------- */
  bool send_motor_command(int dir1, double speed1, int dir2, double speed2);

  /* ---------- ROS 回调 ---------- */
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void send_timer_callback();
  void odom_timer_callback();

  /* ---------- 辅助函数 ---------- */
  std::pair<int, double> velocity_to_motor(double v);
  void publish_odom(double v, double w);

  /* ---------- 参数 ---------- */
  double send_hz_{20.0};
  double odom_hz_{50.0};
  double cmd_vel_timeout_{0.4};
  bool publish_tf_{true};
  std::string odom_topic_{"odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  // 机器人参数
  double wheel_radius_;
  double wheel_base_;
  double motor1_factor_{1.0};
  double motor2_factor_{1.0};

  /* ---------- ROS 接口 ---------- */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  /* ---------- 速度状态 ---------- */
  std::mutex cmd_mutex_;
  double cur_v_{0.0};
  double cur_w_{0.0};
  rclcpp::Time last_cmd_time_;

  /* ---------- 反馈状态 ---------- */
  std::mutex feedback_mutex_;
  double v_l_{0.0}; // 左轮线速度 (m/s)
  double v_r_{0.0}; // 右轮线速度 (m/s)

  /* ---------- 里程计状态 ---------- */
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  std::chrono::steady_clock::time_point last_odom_time_;

  /* ---------- RPMsg ---------- */
  int rpmsg_fd_{-1};
  int rpmsg_ctrl_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread recv_thread_;
};

} // namespace jdbot_k3_esos_control
