#include "jdbot_muse_pi_pro_control/cmdvel_to_motor.hpp"
#include "jdbot_muse_pi_pro_control/robot_config.hpp"
#include "jdbot_muse_pi_pro_control/motor_model.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cmath>

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace robot_config;


CmdVelToMotor::CmdVelToMotor()
: Node("cmdvel_to_motor"),
  dir_ctrl_()
{
    declare_parameter("send_hz", 40.0);
    declare_parameter("cmd_vel_timeout", 0.4);
    declare_parameter("publish_tf", true);
    declare_parameter("odom_topic", "odom");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_footprint");

    send_hz_ = get_parameter("send_hz").as_double();
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
    connect(sock_, (sockaddr*)&addr, sizeof(addr));

    /* ---------- ROS ---------- */
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&CmdVelToMotor::cmdvel_callback, this, std::placeholders::_1)
    );

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    send_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / send_hz_),
        std::bind(&CmdVelToMotor::send_timer_callback, this)
    );

    odom_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CmdVelToMotor::odom_timer_callback, this)
    );

    last_cmd_time_ = now();
    last_odom_time_ = std::chrono::steady_clock::now();
}

CmdVelToMotor::~CmdVelToMotor()
{
    close(sock_);
}

void CmdVelToMotor::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cur_v_ = msg->linear.x;
    cur_w_ = msg->angular.z;
    last_cmd_time_ = now();
}

void CmdVelToMotor::send_timer_callback()
{
    double v, w;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if ((now() - last_cmd_time_).seconds() > timeout_)
        {
            v = 0.0;
            w = 0.0;
        }
        else
        {
            v = cur_v_;
            w = cur_w_;
        }
    }
    send_cmd(v, w);
}

void CmdVelToMotor::send_cmd(double v, double w)
{
    double v_l = v - w * WHEEL_BASE / 2.0;
    double v_r = v + w * WHEEL_BASE / 2.0;

    auto [dir_l, spd_l] = cmd_to_wheel(v_l);
    auto [dir_r, spd_r] = cmd_to_wheel(v_r);

    dir_ctrl_.motor1_direction(dir_l);
    dir_ctrl_.motor2_direction(dir_r);

    json j{
        {"duty_motor1", motor_model::motor1_model(dir_l, spd_l)},
        {"duty_motor2", motor_model::motor2_model(dir_r, spd_r)}
    };

    auto s = j.dump();
    send(sock_, s.c_str(), s.size(), 0);

    char buf[64];
    recv(sock_, buf, sizeof(buf), 0);

    v_l_ = v_l;
    v_r_ = v_r;
}

std::pair<int, double> CmdVelToMotor::cmd_to_wheel(double v)
{
    if (std::abs(v) < 1e-3)
        return {0, 0.0};

    int dir = v > 0 ? 1 : 2;
    double speed = std::abs(v) / (M_PI * WHEEL_DIAMETER);
    return {dir, speed};
}

void CmdVelToMotor::odom_timer_callback()
{
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

void CmdVelToMotor::publish_odom(double v, double w)
{
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

    if (publish_tf_)
    {
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
