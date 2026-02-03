/**
 * RPMsg 电机控制节点实现
 *
 * 通过 RPMsg 与 RCPU 通信实现:
 * - 接收 /cmd_vel 并发送 VEL 命令
 * - 接收 ODM 里程计数据并发布
 */

#include "jdbot_k3_esos_control/rpmsg_motor_node.hpp"
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

RpmsgMotorNode::RpmsgMotorNode() : Node("rpmsg_motor_node") {
  // 声明参数
  declare_parameter("send_hz", 50.0);
  declare_parameter("odom_hz", 50.0);
  declare_parameter("cmd_vel_timeout", 0.4);
  declare_parameter("publish_tf", true);
  declare_parameter("odom_topic", "odom");
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_footprint");

  // 机器人参数
  declare_parameter("wheel_radius", WHEEL_RADIUS);
  declare_parameter("wheel_base", WHEEL_BASE);
  declare_parameter("gear_ratio", GEAR_RATIO);
  declare_parameter("encoder_ppr", ENCODER_PPR);

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
  gear_ratio_ = get_parameter("gear_ratio").as_double();
  encoder_ppr_ = get_parameter("encoder_ppr").as_double();

  RCLCPP_INFO(get_logger(), "RPMsg Motor Node starting...");
  RCLCPP_INFO(get_logger(),
              "Parameters: wheel_radius=%.4f, wheel_base=%.4f, "
              "gear_ratio=%.1f, ppr=%.0f",
              wheel_radius_, wheel_base_, gear_ratio_, encoder_ppr_);

  // 初始化 RPMsg
  if (!rpmsg_init()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize RPMsg");
    throw std::runtime_error("RPMsg initialization failed");
  }

  // 发送配置命令
  send_cfg_command();

  // ROS 订阅和发布
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&RpmsgMotorNode::cmdvel_callback, this, std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 定时器
  send_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / send_hz_),
                        std::bind(&RpmsgMotorNode::send_timer_callback, this));

  odom_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / odom_hz_),
                        std::bind(&RpmsgMotorNode::odom_timer_callback, this));

  last_cmd_time_ = now();

  RCLCPP_INFO(get_logger(), "RPMsg Motor Node started successfully");
}

RpmsgMotorNode::~RpmsgMotorNode() {
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  rpmsg_cleanup();
  RCLCPP_INFO(get_logger(), "RPMsg Motor Node stopped");
}

/* ================= RPMsg 初始化/清理 ================= */

bool RpmsgMotorNode::rpmsg_init() {
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

  // 注意: 不设置 O_NONBLOCK，使用 poll() 处理读取超时
  // 阻塞写入可以避免 EAGAIN 错误

  // 启动接收线程
  running_ = true;
  recv_thread_ = std::thread(&RpmsgMotorNode::recv_thread_func, this);

  RCLCPP_INFO(get_logger(), "RPMsg initialized successfully");
  return true;
}

void RpmsgMotorNode::rpmsg_cleanup() {
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

void RpmsgMotorNode::recv_thread_func() {
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
        // 解析 ODM 数据
        if (strncmp(recv_buf, "ODM:", 4) == 0) {
          OdometryState odom;
          if (sscanf(recv_buf + 4, "%lf,%lf,%lf,%lf,%lf,%u", &odom.x, &odom.y,
                     &odom.theta, &odom.v, &odom.w, &odom.timestamp_ms) == 6) {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            odom_state_ = odom;
          }
        }
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Receive thread exiting");
}

/* ================= 命令发送 ================= */

bool RpmsgMotorNode::send_cfg_command() {
  if (rpmsg_fd_ < 0)
    return false;

  char cmd[128];
  snprintf(cmd, sizeof(cmd),
           "CFG:wheel_radius=%.4f;wheel_base=%.4f;gear_ratio=%.1f;ppr=%.0f",
           wheel_radius_, wheel_base_, gear_ratio_, encoder_ppr_);

  RCLCPP_INFO(get_logger(), "Sending config: %s", cmd);

  ssize_t ret = write(rpmsg_fd_, cmd, strlen(cmd) + 1);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "Write CFG failed: %s", strerror(errno));
    return false;
  }
  return true;
}

bool RpmsgMotorNode::send_vel_command(double v, double w) {
  if (rpmsg_fd_ < 0)
    return false;

  char cmd[64];
  snprintf(cmd, sizeof(cmd), "VEL:%.3f,%.3f", v, w);

  ssize_t ret = write(rpmsg_fd_, cmd, strlen(cmd) + 1);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "Write VEL failed: %s", strerror(errno));
    return false;
  }
  return true;
}

bool RpmsgMotorNode::send_rst_command() {
  if (rpmsg_fd_ < 0)
    return false;

  const char *cmd = "RST:";
  RCLCPP_INFO(get_logger(), "Sending reset command");

  ssize_t ret = write(rpmsg_fd_, cmd, strlen(cmd) + 1);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "Write RST failed: %s", strerror(errno));
    return false;
  }
  return true;
}

/* ================= ROS 回调 ================= */

void RpmsgMotorNode::cmdvel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  cur_v_ = msg->linear.x;
  cur_w_ = msg->angular.z;
  last_cmd_time_ = now();
}

void RpmsgMotorNode::send_timer_callback() {
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
  send_vel_command(v, w);
}

void RpmsgMotorNode::odom_timer_callback() { publish_odom(); }

/* ================= 里程计发布 ================= */

void RpmsgMotorNode::publish_odom() {
  OdometryState odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom = odom_state_;
  }

  auto stamp = now();

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = odom.x;
  odom_msg.pose.pose.position.y = odom.y;
  odom_msg.pose.pose.orientation.z = std::sin(odom.theta / 2.0);
  odom_msg.pose.pose.orientation.w = std::cos(odom.theta / 2.0);

  odom_msg.twist.twist.linear.x = odom.v;
  odom_msg.twist.twist.angular.z = odom.w;

  odom_pub_->publish(odom_msg);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = odom.x;
    tf.transform.translation.y = odom.y;
    tf.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }
}

} // namespace jdbot_k3_esos_control
