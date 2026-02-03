#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace jdbot_k3_esos_control {

/**
 * @brief 里程计状态结构
 */
struct OdometryState {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double v{0.0};
  double w{0.0};
  uint32_t timestamp_ms{0};
};

/**
 * @brief RPMsg 电机控制节点
 *
 * 通过 RPMsg 与 RCPU 通信，实现速度控制和里程计发布
 *
 * 发送协议:
 * - CFG:wheel_radius=R;wheel_base=L;gear_ratio=G;ppr=P
 * - VEL:v,w
 * - RST:
 *
 * 接收协议:
 * - ODM:x,y,theta,v,w,timestamp_ms
 */
class RpmsgMotorNode : public rclcpp::Node {
public:
  RpmsgMotorNode();
  ~RpmsgMotorNode() override;

private:
  /* ---------- RPMsg 操作 ---------- */
  bool rpmsg_init();
  void rpmsg_cleanup();
  void recv_thread_func();

  /* ---------- 命令发送 ---------- */
  bool send_cfg_command();
  bool send_vel_command(double v, double w);
  bool send_rst_command();

  /* ---------- ROS 回调 ---------- */
  void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void send_timer_callback();
  void odom_timer_callback();

  /* ---------- 里程计发布 ---------- */
  void publish_odom();

  /* ---------- 参数 ---------- */
  double send_hz_{50.0};
  double odom_hz_{50.0};
  double cmd_vel_timeout_{0.4};
  bool publish_tf_{true};
  std::string odom_topic_{"odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};

  // 机器人参数 (可通过 ROS 参数覆盖)
  double wheel_radius_;
  double wheel_base_;
  double gear_ratio_;
  double encoder_ppr_;

  /* ---------- ROS 接口 ---------- */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  /* ---------- 状态 ---------- */
  std::mutex cmd_mutex_;
  double cur_v_{0.0};
  double cur_w_{0.0};
  rclcpp::Time last_cmd_time_;

  std::mutex odom_mutex_;
  OdometryState odom_state_;

  /* ---------- RPMsg ---------- */
  int rpmsg_fd_{-1};
  int rpmsg_ctrl_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread recv_thread_;
};

} // namespace jdbot_k3_esos_control
