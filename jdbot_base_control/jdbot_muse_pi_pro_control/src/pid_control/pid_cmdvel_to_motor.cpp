
#include "motor_model.hpp"
#include "robot_config.hpp"

#include "pid_control/pid_cmdvel_to_motor.hpp"

#include "encoder/encoder_speed_meter.hpp"


#include <cmath>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace robot_config;

PidCmdVelToMotor::PidCmdVelToMotor() : Node("cmdvel_to_motor"), dir_ctrl_() {
  declare_parameter("control_hz", 40.0);
  declare_parameter("cmd_vel_timeout", 0.4);
  declare_parameter("publish_tf", true);
  declare_parameter("odom_topic", "odom");
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_footprint");

  control_hz_ = get_parameter("control_hz").as_double();
  timeout_ = get_parameter("cmd_vel_timeout").as_double();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  odom_topic_ = get_parameter("odom_topic").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  /* ---------- socket ---------- */
  sock_ = socket(AF_UNIX, SOCK_STREAM, 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  strcpy(addr.sun_path, "/run/pwm_control_uds.sock");
  connect(sock_, (sockaddr *)&addr, sizeof(addr));

  /* ---------- ROS ---------- */
  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&PidCmdVelToMotor::cmdvel_callback, this,
                std::placeholders::_1));

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_hz_),
      std::bind(&PidCmdVelToMotor::control_timer_callback, this));

  odom_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&PidCmdVelToMotor::odom_timer_callback, this));

  last_cmd_time_ = now();
  last_odom_time_ = std::chrono::steady_clock::now();

  /* ---------- encoder speed meters ---------- */
  double sample_period = 0.033; // 33ms 30Hz

  encoder_motor1_ = std::make_unique<EncoderSpeedMeter>(
      73, 0, sample_period, ENCODER_PPR, ENCODER_EDGES, ENCODER_GEAR_RATIO, ENCODER_ALPHA ,ENCODER_QUEUE_SIZE); // motor1
  encoder_motor2_ = std::make_unique<EncoderSpeedMeter>(
      72, 0, sample_period, ENCODER_PPR, ENCODER_EDGES, ENCODER_GEAR_RATIO, ENCODER_ALPHA ,ENCODER_QUEUE_SIZE); // motor2

  encoder_motor1_->start();
  encoder_motor2_->start();

  // if (!encoder_motor1_->start()) {
  //   RCLCPP_ERROR(get_logger(), "Failed to start encoder for motor1 (GPIO 73)");
  // }
  // if (!encoder_motor2_->start()) {
  //   RCLCPP_ERROR(get_logger(), "Failed to start encoder for motor2 (GPIO 72)");
  // }

  /* PID Init*/
  PID_Controller_Init(&motor1_pid_, 0.01, 0.5, 0.01, 1.0 / control_hz_, 1.0, 1.0); // *PID_Controller、kp、ki、kd、dt、i_limit、out_limit
  PID_Controller_Init(&motor2_pid_, 0.01, 0.5, 0.01, 1.0 / control_hz_, 1.0, 1.0); // *PID_Controller、kp、ki、kd、dt、i_limit、out_limit
}

PidCmdVelToMotor::~PidCmdVelToMotor() { 
  encoder_motor1_->stop();
  encoder_motor2_->stop();
  close(sock_); 
  }

void PidCmdVelToMotor::cmdvel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    cur_v_ = msg->linear.x;
    cur_w_ = msg->angular.z;
    last_cmd_time_ = now();
}

void PidCmdVelToMotor::control_timer_callback() {
  double v, w;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((now() - last_cmd_time_).seconds() > timeout_) {
      v = 0.0;
      w = 0.0;
    } else {
      v = cur_v_;
      w = cur_w_;
    }
  }
  send_cmd(v, w);
}

void PidCmdVelToMotor::send_cmd(double v, double w) {

  // 计算目标轮速和方向
  double v_l = v - w * WHEEL_BASE / 2.0;
  double v_r = v + w * WHEEL_BASE / 2.0;

  auto [dir_l, target_spd_l] = cmd_to_wheel(v_l);
  auto [dir_r, target_spd_r] = cmd_to_wheel(v_r);

  dir_ctrl_.motor1_direction(dir_l); // 设置电机方向
  dir_ctrl_.motor2_direction(dir_r);

  //
  motor1_pid_.setpoint = target_spd_l;
  motor2_pid_.setpoint = target_spd_r;

  // 计算前馈值
  double motor1_pwm_ff = motor_model::motor1_model(dir_l, target_spd_l);
  double motor2_pwm_ff = motor_model::motor2_model(dir_r, target_spd_r);

  // 获取实际轮速
  double motor1_speed_feedback = encoder_motor1_->get_rps();
  double motor2_speed_feedback = encoder_motor2_->get_rps();
  
  // 计算 PID
  double motor1_pwm_output = PID_FF_Update(&motor1_pid_, motor1_speed_feedback, motor1_pwm_ff); // 带前馈的PID
  double motor2_pwm_output = PID_FF_Update(&motor2_pid_, motor2_speed_feedback, motor2_pwm_ff); // 带前馈的PID
  
  printf("motor1_target: %.3f motor1: %.3f 转/s, motor2_target: %.3f motor2: "
         "%.3f 转/s\n",
         target_spd_l, motor1_speed_feedback, target_spd_r, motor2_speed_feedback);

  json j{{"duty_motor1", motor1_pwm_output},
         {"duty_motor2", motor2_pwm_output}};

  auto s = j.dump();
  send(sock_, s.c_str(), s.size(), 0);

  char buf[64];
  recv(sock_, buf, sizeof(buf), 0);

  v_l_ = to_wheel_linear(dir_l, motor1_speed_feedback);
  v_r_ = to_wheel_linear(dir_r, motor2_speed_feedback);
}

std::pair<int, double> PidCmdVelToMotor::cmd_to_wheel(double v) {
  if (std::abs(v) < 1e-3)
    return {0, 0.0};

  int dir = v > 0 ? 1 : 2;
  double speed = std::abs(v) / (M_PI * WHEEL_DIAMETER);
  return {dir, speed};
}

double PidCmdVelToMotor::to_wheel_linear(int direction, double speed_angular) {

  double linear_speed = 0;
  if (std::abs(speed_angular) < 1e-3 || direction == 0){
    return 0;
  }
  
  if(direction == 1){
    linear_speed = speed_angular * (M_PI * WHEEL_DIAMETER);
  }
  else{
    linear_speed = -1.0 * speed_angular * (M_PI * WHEEL_DIAMETER);
  }

  return linear_speed;
}

void PidCmdVelToMotor::odom_timer_callback() {
  auto now_tp = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now_tp - last_odom_time_).count();
  last_odom_time_ = now_tp;

  double v = (v_l_ + v_r_) / 2.0;
  double w = (v_r_ - v_l_) / WHEEL_BASE;

  yaw_ += w * dt;
  x_ += v * std::cos(yaw_) * dt;
  y_ += v * std::sin(yaw_) * dt;

  publish_odom(v, w);
}

void PidCmdVelToMotor::publish_odom(double v, double w) {
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
