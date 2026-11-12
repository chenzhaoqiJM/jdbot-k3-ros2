/**
 * ESP32-C6 底盘控制节点实现 (C++ 版本)
 *
 * 完全按照 jdbot_base_bringup/jdbot_base_bringup/ros2_ctrl_odom_node.py
 * 的逻辑改写，行为保持一致。
 */

#include "jdbot_esp32_c6_control/esp32_c6_control_node.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace jdbot_esp32_c6_control {

namespace {

// 将整数波特率映射为 termios 常量
speed_t baudrate_to_speed(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

}  // namespace

Esp32C6ControlNode::Esp32C6ControlNode() : Node("esp32_c6_control_node") {
  // ---------------- 参数 ----------------
  declare_parameter("serial_port", std::string("/dev/jdbot"));
  declare_parameter("baudrate", 115200);
  declare_parameter("send_hz", 20.0);
  declare_parameter("odom_hz", 50.0);
  declare_parameter("cmd_vel_timeout", 0.4);
  declare_parameter("cmd_vel_deadzone_threshold", 0.001);
  declare_parameter("min_linear_speed", 0.05);
  declare_parameter("min_angular_speed", 0.26);
  declare_parameter("wheel_diameter", 0.067);
  declare_parameter("wheel_base", 0.183);
  declare_parameter("motor1_factor", 1.0);
  declare_parameter("motor2_factor", 1.0);
  declare_parameter("feedback_pwm_deadzone", 0);
  declare_parameter("encoder_ppr", 1000.0);
  declare_parameter("reduction_ratio", 56.0);
  declare_parameter("ff_factor", 310.0);
  declare_parameter("pid_kp", 10.0);
  declare_parameter("pid_ki", 70.0);
  declare_parameter("pid_kd", 0.0);
  declare_parameter("straight_balance_kp", 25.0);
  declare_parameter("straight_balance_ki", 8.0);
  declare_parameter("straight_balance_kd", 0.0);
  declare_parameter("cfg_send_on_startup", true);
  declare_parameter("debug", false);

  // ---- odom 参数 ----
  declare_parameter("publish_tf", true);
  declare_parameter("odom_topic", std::string("odom"));
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_footprint"));

  motor1_factor_ = get_parameter("motor1_factor").as_double();
  motor2_factor_ = get_parameter("motor2_factor").as_double();
  feedback_pwm_deadzone_ = get_parameter("feedback_pwm_deadzone").as_int();
  encoder_ppr_ = get_parameter("encoder_ppr").as_double();
  reduction_ratio_ = get_parameter("reduction_ratio").as_double();
  ff_factor_ = get_parameter("ff_factor").as_double();
  pid_kp_ = get_parameter("pid_kp").as_double();
  pid_ki_ = get_parameter("pid_ki").as_double();
  pid_kd_ = get_parameter("pid_kd").as_double();
  straight_balance_kp_ = get_parameter("straight_balance_kp").as_double();
  straight_balance_ki_ = get_parameter("straight_balance_ki").as_double();
  straight_balance_kd_ = get_parameter("straight_balance_kd").as_double();
  debug_ = get_parameter("debug").as_bool();

  serial_port_ = get_parameter("serial_port").as_string();
  baudrate_ = get_parameter("baudrate").as_int();
  send_hz_ = get_parameter("send_hz").as_double();
  odom_hz_ = get_parameter("odom_hz").as_double();
  cmd_vel_timeout_ = get_parameter("cmd_vel_timeout").as_double();
  cmd_vel_deadzone_threshold_ =
      get_parameter("cmd_vel_deadzone_threshold").as_double();
  min_linear_speed_ = get_parameter("min_linear_speed").as_double();
  min_angular_speed_ = get_parameter("min_angular_speed").as_double();
  wheel_diameter_ = get_parameter("wheel_diameter").as_double();
  wheel_base_ = get_parameter("wheel_base").as_double();
  cfg_send_on_startup_ = get_parameter("cfg_send_on_startup").as_bool();

  publish_tf_ = get_parameter("publish_tf").as_bool();
  odom_topic_ = get_parameter("odom_topic").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  // ---------------- 串口 ----------------
  if (!serial_open(serial_port_, baudrate_)) {
    throw std::runtime_error("Failed to open serial port: " + serial_port_);
  }
  RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", serial_port_.c_str(),
              baudrate_);
  send_cfg();

  // ---------------- 状态 ----------------
  last_cmd_time_ = std::chrono::steady_clock::now();
  last_odom_time_ = std::chrono::steady_clock::now();
  running_ = true;

  // ---------------- ROS ----------------
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&Esp32C6ControlNode::cmdvel_callback, this,
                std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  send_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / send_hz_),
                        std::bind(&Esp32C6ControlNode::send_timer_callback,
                                  this));

  odom_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / odom_hz_),
                        std::bind(&Esp32C6ControlNode::odom_timer_callback,
                                  this));

  // ---------------- 串口读取线程 ----------------
  read_thread_ = std::thread(&Esp32C6ControlNode::serial_read_thread, this);

  RCLCPP_INFO(get_logger(), "cmd_vel -> serial + odom node started");
}

Esp32C6ControlNode::~Esp32C6ControlNode() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  serial_close();
}

// =====================================================
// 下发参数配置
// =====================================================
void Esp32C6ControlNode::send_cfg() {
  if (!cfg_send_on_startup_) {
    return;
  }

  char buf[256];
  snprintf(buf, sizeof(buf),
           "CFG,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
           encoder_ppr_, reduction_ratio_, ff_factor_, pid_kp_, pid_ki_,
           pid_kd_, straight_balance_kp_, straight_balance_ki_,
           straight_balance_kd_);
  std::string cfg(buf);
  serial_write(cfg);

  // 去掉末尾换行打印
  std::string trimmed = cfg;
  if (!trimmed.empty() && trimmed.back() == '\n') {
    trimmed.pop_back();
  }
  RCLCPP_INFO(get_logger(), "Sent CFG: %s", trimmed.c_str());
}

// =====================================================
// cmd_vel 回调
// =====================================================
void Esp32C6ControlNode::cmdvel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  cur_v_ = msg->linear.x;
  cur_w_ = msg->angular.z;
  last_cmd_time_ = std::chrono::steady_clock::now();
}

// =====================================================
// 定时器：发送
// =====================================================
void Esp32C6ControlNode::send_timer_callback() {
  double v, w;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    auto elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_cmd_time_)
            .count();
    if (elapsed > cmd_vel_timeout_) {
      v = 0.0;
      w = 0.0;
    } else {
      v = cur_v_;
      w = cur_w_;
    }
  }

  v = apply_cmd_vel_deadzone(v, min_linear_speed_);
  w = apply_cmd_vel_deadzone(w, min_angular_speed_);

  send_cmd(v, w);
}

// =====================================================
// 串口发送
// =====================================================
double Esp32C6ControlNode::apply_cmd_vel_deadzone(double value,
                                                  double min_abs_value) {
  double abs_value = std::abs(value);
  if (cmd_vel_deadzone_threshold_ < abs_value &&
      abs_value < min_abs_value) {
    return std::copysign(min_abs_value, value);
  }
  return value;
}

void Esp32C6ControlNode::send_cmd(double v, double w) {
  double v_l = v - w * wheel_base_ / 2.0;
  double v_r = v + w * wheel_base_ / 2.0;

  auto [dir_l, spd_l] = wheel_speed_to_cmd(v_l);
  auto [dir_r, spd_r] = wheel_speed_to_cmd(v_r);

  spd_l *= motor1_factor_;
  spd_r *= motor2_factor_;

  char buf[64];
  snprintf(buf, sizeof(buf), "%d,%.2f;%d,%.2f\n", dir_l, spd_l, dir_r, spd_r);
  serial_write(buf);
}

std::pair<int, double> Esp32C6ControlNode::wheel_speed_to_cmd(double v) {
  if (std::abs(v) < 1e-3) {
    return {0, 0.0};
  }

  int direction = v > 0 ? 1 : 2;
  double speed = std::abs(v) / (M_PI * wheel_diameter_);
  return {direction, speed};
}

// =====================================================
// 串口读取线程（解析轮速 -> odom）
// =====================================================
void Esp32C6ControlNode::serial_read_thread() {
  RCLCPP_INFO(get_logger(), "Serial RX thread started");

  std::string buf;
  char chunk[64];

  while (rclcpp::ok() && running_) {
    pollfd pfd{};
    pfd.fd = serial_fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 50);  // 50ms 超时，对应 pyserial timeout=0.05
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_ERROR(get_logger(), "Poll error: %s", strerror(errno));
      break;
    }
    if (ret == 0) {
      continue;
    }

    if (pfd.revents & POLLIN) {
      ssize_t n = read(serial_fd_, chunk, sizeof(chunk));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "Serial read error: %s", strerror(errno));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      if (n == 0) {
        continue;
      }

      buf.append(chunk, static_cast<size_t>(n));

      size_t pos;
      while ((pos = buf.find('\n')) != std::string::npos) {
        std::string line = buf.substr(0, pos);
        buf.erase(0, pos + 1);

        // 去除行尾空白
        while (!line.empty() &&
               (line.back() == '\r' || line.back() == ' ' ||
                line.back() == '\t')) {
          line.pop_back();
        }
        if (line.empty()) {
          continue;
        }

        if (line.rfind("CFG_OK", 0) == 0) {
          RCLCPP_INFO(get_logger(), "CFG ACK: %s", line.c_str());
          continue;
        }
        if (line.rfind("CFG_ERR", 0) == 0) {
          RCLCPP_ERROR(get_logger(), "CFG error: %s", line.c_str());
          continue;
        }

        parse_feedback(line);
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Serial RX thread stopped");
}

// =====================================================
// 串口反馈解析
// =====================================================
void Esp32C6ControlNode::parse_feedback(const std::string& text) {
  size_t semi = text.find(';');
  if (semi == std::string::npos) {
    RCLCPP_WARN(get_logger(), "Parse error: %s", text.c_str());
    return;
  }

  std::string power_str = text.substr(0, semi);
  std::string rest = text.substr(semi + 1);

  size_t semi2 = rest.find(';');
  if (semi2 == std::string::npos) {
    RCLCPP_WARN(get_logger(), "Parse error: %s", text.c_str());
    return;
  }

  std::string left = rest.substr(0, semi2);
  std::string right = rest.substr(semi2 + 1);

  try {
    double v_l = parse_motor(left);
    double v_r = parse_motor(right);
    double power_voltage = std::stod(power_str);

    if (debug_) {
      // 转/s 位于倒数第 3 个字段
      auto field = [](const std::string& s, int from_back) {
        std::string tmp = s;
        std::string out;
        for (int i = 0; i < from_back; ++i) {
          size_t p = tmp.find_last_of(',');
          if (p == std::string::npos) {
            return std::string("0");
          }
          out = tmp.substr(p + 1);
          tmp = tmp.substr(0, p);
        }
        return out;
      };

      double speed = std::atof(field(left, 3).c_str());   // 转/s
      double speed_r = std::atof(field(right, 3).c_str());  // 转/s
      double pwm = std::atof(field(left, 1).c_str());
      double pwm_r = std::atof(field(right, 1).c_str());

      double dynamic_left_ff =
          std::abs(speed) > 1e-3 ? pwm / speed : 0.0;
      double dynamic_right_ff =
          std::abs(speed_r) > 1e-3 ? pwm_r / speed_r : 0.0;

      RCLCPP_INFO(get_logger(),
                  "left:%s, right:%s,速度差:%.2f, 左轮ff_factor:%.2f, "
                  "右轮ff_factor:%.2f",
                  left.c_str(), right.c_str(), speed_r - speed,
                  dynamic_left_ff, dynamic_right_ff);
    }

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    v_l_ = v_l;
    v_r_ = v_r;
    power_voltage_ = power_voltage;
  } catch (const std::exception& e) {
    RCLCPP_WARN(get_logger(), "Parse error: %s: %s", text.c_str(), e.what());
  }
}

double Esp32C6ControlNode::parse_motor(const std::string& s) {
  // 字段顺序: pluse_counts,spd(转/s),方向,pwm值
  auto field = [](const std::string& str, int from_back) {
    std::string tmp = str;
    std::string out;
    for (int i = 0; i < from_back; ++i) {
      size_t p = tmp.find_last_of(',');
      if (p == std::string::npos) {
        return std::string("0");
      }
      out = tmp.substr(p + 1);
      tmp = tmp.substr(0, p);
    }
    return out;
  };

  int pwm = std::atoi(field(s, 1).c_str());       // items[-1]
  double speed = std::atof(field(s, 3).c_str());  // items[-3] 转/s
  int direction = std::atoi(field(s, 2).c_str()); // items[-2]

  if (std::abs(pwm) < feedback_pwm_deadzone_) {
    return 0.0;
  }

  if (direction == 1) {
    speed = -speed;
  } else if (direction == 0) {
    speed = 0.0;
  }

  // 转/s -> m/s
  return speed * M_PI * wheel_diameter_;
}

// =====================================================
// odom 定时发布
// =====================================================
void Esp32C6ControlNode::odom_timer_callback() {
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_odom_time_).count();
  last_odom_time_ = now;
  if (dt <= 0.0) {
    return;
  }

  double v_l, v_r;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    v_l = v_l_;
    v_r = v_r_;
  }

  double v = (v_r + v_l) / 2.0;
  double w = (v_r - v_l) / wheel_base_;

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    yaw_ += w * dt;
    x_ += v * std::cos(yaw_) * dt;
    y_ += v * std::sin(yaw_) * dt;
  }

  publish_odom(v, w);
}

void Esp32C6ControlNode::publish_odom(double v, double w) {
  double x, y, yaw;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    x = x_;
    y = y_;
    yaw = yaw_;
  }

  auto stamp = now();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;

  double qz = std::sin(yaw / 2.0);
  double qw = std::cos(yaw / 2.0);
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  odom.twist.twist.linear.x = v;
  odom.twist.twist.angular.z = w;

  odom_pub_->publish(odom);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(t);
  }
}

// =====================================================
// 串口打开/关闭/写
// =====================================================
bool Esp32C6ControlNode::serial_open(const std::string& port, int baudrate) {
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port.c_str(),
                 strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
    close(fd);
    return false;
  }

  // 8N1, 无流控
  cfmakeraw(&tty);
  tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tty.c_cflag |= CS8 | CREAD | CLOCAL;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag = 0;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  // 读取超时: VMIN=0, VTIME=1 -> 0.1s 超时
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  speed_t speed = baudrate_to_speed(baudrate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
    close(fd);
    return false;
  }

  serial_fd_ = fd;
  return true;
}

void Esp32C6ControlNode::serial_close() {
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

void Esp32C6ControlNode::serial_write(const std::string& data) {
  if (serial_fd_ < 0) {
    return;
  }
  ssize_t ret = write(serial_fd_, data.data(), data.size());
  (void)ret;
}

}  // namespace jdbot_esp32_c6_control
