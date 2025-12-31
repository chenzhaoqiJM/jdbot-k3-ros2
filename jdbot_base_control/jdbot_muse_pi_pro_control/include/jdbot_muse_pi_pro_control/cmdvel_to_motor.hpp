#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <string>

#include "jdbot_muse_pi_pro_control/motor_direction_ctrl.hpp"

class CmdVelToMotor : public rclcpp::Node
{
public:
    CmdVelToMotor();
    ~CmdVelToMotor();

private:
    /* ---------- ROS callbacks ---------- */
    void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void send_timer_callback();
    void odom_timer_callback();

    /* ---------- internal ---------- */
    void send_cmd(double v, double w);
    void publish_odom(double v, double w);
    std::pair<int, double> cmd_to_wheel(double v);

    /* ---------- parameters ---------- */
    double send_hz_;
    double timeout_;
    bool publish_tf_;
    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;

    /* ---------- ROS ---------- */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr send_timer_;
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
};
