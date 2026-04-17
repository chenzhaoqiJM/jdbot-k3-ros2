/**
 * 旧协议电机控制节点实现
 *
 * 使用旧协议格式与 RCPU 通信:
 * - 发送: "dir1,speed1;dir2,speed2"
 * - 接收: "dir1,speed1_mrs;dir2,speed2_mrs"
 *
 * 在 Linux 端根据反馈计算里程计
 */

#include "jdbot_k3_esos_control/rpmsg_node.hpp"
#include "jdbot_k3_esos_control/robot_config.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <cstring>

namespace jdbot_k3_esos_control {

using namespace robot_config;

/* ================= RPMsg ioctl 定义 ================= */

struct rpmsg_endpoint_info {
  char name[32];
  uint32_t src;
  uint32_t dst;
};

#define RPMSG_CREATE_EPT_IOCTL _IOW(0xb5, 0x1, struct rpmsg_endpoint_info)
#define RPMSG_DESTROY_EPT_IOCTL _IO(0xb5, 0x2)

/* ================= 构造/析构 ================= */

RpmsgLegacyNode::RpmsgLegacyNode() : Node("rpmsg_legacy_node") {
  // 声明参数
  declare_parameter("send_hz", 20.0);
  declare_parameter("odom_hz", 50.0);
  declare_parameter("cmd_vel_timeout", 0.2);
  declare_parameter("publish_tf", true);
  declare_parameter("odom_topic", "odom");
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_footprint");

  // 机器人参数
  declare_parameter("wheel_radius", WHEEL_RADIUS);
  declare_parameter("wheel_base", WHEEL_BASE);
  declare_parameter("motor1_factor", 1.0);
  declare_parameter("motor2_factor", 1.0);
  declare_parameter("reduction_ratio", 56.0);
  declare_parameter("ff_factor", 0.3);
  declare_parameter("pid_kp", 0.05);
  declare_parameter("pid_ki", 0.2);
  declare_parameter("pid_kd", 0.01);
  declare_parameter("cfg_send_on_startup", true);

  // 获取参数
  send_hz_ = get_parameter("send_hz").as_double();
  odom_hz_ = get_parameter("odom_hz").as_double();
  cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  odom_topic_ = get_parameter("odom_topic").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  wheel_radius_ = get_parameter("wheel_radius").as_double();
  wheel_base_ = get_parameter("wheel_base").as_double();
  motor1_factor_ = get_parameter("motor1_factor").as_double();
  motor2_factor_ = get_parameter("motor2_factor").as_double();
  reduction_ratio_ = get_parameter("reduction_ratio").as_double();
  ff_factor_ = get_parameter("ff_factor").as_double();
  pid_kp_ = get_parameter("pid_kp").as_double();
  pid_ki_ = get_parameter("pid_ki").as_double();
  pid_kd_ = get_parameter("pid_kd").as_double();
  cfg_send_on_startup_ = get_parameter("cfg_send_on_startup").as_bool();

  RCLCPP_INFO(get_logger(), "RPMsg Legacy Node starting...");
  RCLCPP_INFO(get_logger(),
              "Parameters: wheel_radius=%.4f, wheel_base=%.4f, "
              "motor1_factor=%.2f, motor2_factor=%.2f",
              wheel_radius_, wheel_base_, motor1_factor_, motor2_factor_);
  RCLCPP_INFO(get_logger(),
              "CFG: send_on_startup=%s ratio=%.3f ff=%.3f kp=%.3f ki=%.3f "
              "kd=%.3f",
              cfg_send_on_startup_ ? "true" : "false", reduction_ratio_,
              ff_factor_, pid_kp_, pid_ki_, pid_kd_);

  // 初始化 RPMsg
  if (!rpmsg_init()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize RPMsg");
    throw std::runtime_error("RPMsg initialization failed");
  }

  // ROS 订阅和发布
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&RpmsgLegacyNode::cmdvel_callback, this,
                std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 定时器
  send_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / send_hz_),
                        std::bind(&RpmsgLegacyNode::send_timer_callback, this));

  odom_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / odom_hz_),
                        std::bind(&RpmsgLegacyNode::odom_timer_callback, this));

  last_cmd_time_ = now();
  last_odom_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(), "RPMsg Legacy Node started successfully");
}

RpmsgLegacyNode::~RpmsgLegacyNode() {
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  rpmsg_cleanup();
  RCLCPP_INFO(get_logger(), "RPMsg Legacy Node stopped");
}

/* ================= RPMsg 初始化/清理 ================= */

bool RpmsgLegacyNode::rpmsg_init() {
  RCLCPP_INFO(get_logger(), "Opening RPMsg control device: %s", RPMSG_CTRL_DEV);

  // 打开 RPMsg 控制设备
  rpmsg_ctrl_fd_ = open(RPMSG_CTRL_DEV, O_RDWR);
  if (rpmsg_ctrl_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", RPMSG_CTRL_DEV,
                 strerror(errno));
    return false;
  }

  // 配置端点信息
  rpmsg_endpoint_info epinfo{};
  strncpy(epinfo.name, RPMSG_SERVICE_NAME, sizeof(epinfo.name) - 1);
  epinfo.src = RPMSG_LOCAL_ADDR;
  epinfo.dst = RPMSG_REMOTE_ADDR;

  RCLCPP_INFO(get_logger(), "Creating endpoint: %s (src=%u, dst=%u)",
              RPMSG_SERVICE_NAME, RPMSG_LOCAL_ADDR, RPMSG_REMOTE_ADDR);

  // 创建端点
  if (ioctl(rpmsg_ctrl_fd_, RPMSG_CREATE_EPT_IOCTL, &epinfo) < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to create endpoint: %s",
                 strerror(errno));
    close(rpmsg_ctrl_fd_);
    rpmsg_ctrl_fd_ = -1;
    return false;
  }

  // 打开数据通道
  rpmsg_fd_ = open(RPMSG_DATA_DEV, O_RDWR);
  if (rpmsg_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", RPMSG_DATA_DEV,
                 strerror(errno));
    close(rpmsg_ctrl_fd_);
    rpmsg_ctrl_fd_ = -1;
    return false;
  }

  // 启动接收线程
  running_ = true;
  recv_thread_ = std::thread(&RpmsgLegacyNode::recv_thread_func, this);

  RCLCPP_INFO(get_logger(), "RPMsg initialized successfully");
  return true;
}

void RpmsgLegacyNode::rpmsg_cleanup() {
  if (rpmsg_fd_ >= 0) {
    close(rpmsg_fd_);
    rpmsg_fd_ = -1;
  }
  if (rpmsg_ctrl_fd_ >= 0) {
    close(rpmsg_ctrl_fd_);
    rpmsg_ctrl_fd_ = -1;
  }
  RCLCPP_INFO(get_logger(), "RPMsg cleaned up");
}

/* ================= 接收线程 ================= */

void RpmsgLegacyNode::recv_thread_func() {
  RCLCPP_INFO(get_logger(), "Receive thread started");

  char recv_buf[256];
  pollfd pfd{};
  pfd.fd = rpmsg_fd_;
  pfd.events = POLLIN;

  while (running_) {
    int ret = poll(&pfd, 1, 100); // 100ms 超时
    if (ret < 0) {
      if (errno == EINTR)
        continue;
      RCLCPP_ERROR(get_logger(), "Poll error: %s", strerror(errno));
      break;
    }

    if (ret == 0)
      continue;

    if (pfd.revents & POLLIN) {
      memset(recv_buf, 0, sizeof(recv_buf));
      ssize_t n = read(rpmsg_fd_, recv_buf, sizeof(recv_buf) - 1);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
          continue;
        RCLCPP_ERROR(get_logger(), "Read error: %s", strerror(errno));
        break;
      }

      if (n > 0) {
        // 解析旧协议反馈: "dir1,speed1_mrs;dir2,speed2_mrs"
        int dir1 = 0, dir2 = 0;
        int speed1_mrs = 0, speed2_mrs = 0;

        if (sscanf(recv_buf, "%d,%d;%d,%d", &dir1, &speed1_mrs, &dir2,
                   &speed2_mrs) == 4) {
          // 转换 毫转/秒 -> m/s
          // speed_mrs 是轮子的转速 (毫转/秒, milli-revolutions per second)
          // 转/秒 = 毫转/秒 / 1000
          // 线速度 = 转/秒 * 2 * pi * 轮半径
          double rps1 = speed1_mrs / 1000.0; // 转/秒
          double rps2 = speed2_mrs / 1000.0;

          double v1 = rps1 * 2.0 * M_PI * wheel_radius_;
          double v2 = rps2 * 2.0 * M_PI * wheel_radius_;

          // 根据方向调整符号
          if (dir1 == 2)
            v1 = -v1;
          else if (dir1 == 0)
            v1 = 0.0;

          if (dir2 == 2)
            v2 = -v2;
          else if (dir2 == 0)
            v2 = 0.0;

          std::lock_guard<std::mutex> lock(feedback_mutex_);
          v_l_ = v1;
          v_r_ = v2;
        }
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Receive thread exiting");
}

/* ================= 命令发送 ================= */

bool RpmsgLegacyNode::send_motor_command(int dir1, double speed1, int dir2,
                                         double speed2) {
  if (rpmsg_fd_ < 0)
    return false;

  char cmd[64];
  snprintf(cmd, sizeof(cmd), "%d,%.3f;%d,%.3f", dir1, speed1, dir2, speed2);

  ssize_t ret = write(rpmsg_fd_, cmd, strlen(cmd) + 1);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "Write failed: %s", strerror(errno));
    return false;
  }
  return true;
}

bool RpmsgLegacyNode::send_cfg_command(double ratio, double ff, double kp,
                                       double ki, double kd) {
  if (rpmsg_fd_ < 0)
    return false;

  char cmd[96];
  snprintf(cmd, sizeof(cmd), "CFG,%.3f,%.3f,%.3f,%.3f,%.3f", ratio, ff, kp,
           ki, kd);

  ssize_t ret = write(rpmsg_fd_, cmd, strlen(cmd) + 1);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "Write CFG failed: %s", strerror(errno));
    return false;
  }

  RCLCPP_INFO(get_logger(), "Sent CFG command: %s", cmd);
  return true;
}

/* ================= 辅助函数 ================= */

std::pair<int, double> RpmsgLegacyNode::velocity_to_motor(double v) {
  if (std::abs(v) < 1e-3) {
    return {0, 0.0};
  }

  int dir = v > 0 ? 1 : 2;
  // 线速度 -> 转/秒: speed = |v| / (2 * pi * wheel_radius)
  double speed = std::abs(v) / (2.0 * M_PI * wheel_radius_);
  return {dir, speed};
}

/* ================= ROS 回调 ================= */

void RpmsgLegacyNode::cmdvel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  cur_v_ = msg->linear.x;
  cur_w_ = msg->angular.z;
  last_cmd_time_ = now();
}

void RpmsgLegacyNode::send_timer_callback() {
  if (cfg_send_on_startup_ && !cfg_sent_) {
    if (send_cfg_command(reduction_ratio_, ff_factor_, pid_kp_, pid_ki_,
                         pid_kd_)) {
      cfg_sent_ = true;
    }
  }

  double v, w;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if ((now() - last_cmd_time_).seconds() > cmd_vel_timeout_) {
      v = 0.0;
      w = 0.0;
    } else {
      v = cur_v_;
      w = cur_w_;
    }
  }

  // 差速运动学: v_l = v - w * L/2, v_r = v + w * L/2
  double v_l = v - w * wheel_base_ / 2.0;
  double v_r = v + w * wheel_base_ / 2.0;

  auto [dir1, speed1] = velocity_to_motor(v_l);
  auto [dir2, speed2] = velocity_to_motor(v_r);

  // 应用电机因子
  speed1 *= motor1_factor_;
  speed2 *= motor2_factor_;

  send_motor_command(dir1, speed1, dir2, speed2);
}

void RpmsgLegacyNode::odom_timer_callback() {
  auto now_tp = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now_tp - last_odom_time_).count();
  last_odom_time_ = now_tp;

  if (dt <= 0.0)
    return;

  double v_l, v_r;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    v_l = v_l_;
    v_r = v_r_;
  }

  // 差速运动学逆解
  double v = (v_l + v_r) / 2.0;
  double w = (v_r - v_l) / wheel_base_;

  // 航迹推算
  yaw_ += w * dt;
  x_ += v * std::cos(yaw_) * dt;
  y_ += v * std::sin(yaw_) * dt;

  publish_odom(v, w);
}

/* ================= 里程计发布 ================= */

void RpmsgLegacyNode::publish_odom(double v, double w) {
  auto stamp = now();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.z = std::sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(yaw_ / 2.0);

  odom.twist.twist.linear.x = v;
  odom.twist.twist.angular.z = w;

  odom_pub_->publish(odom);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }
}

} // namespace jdbot_k3_esos_control
