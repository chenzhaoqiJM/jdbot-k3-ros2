#include "pid_control/pid_cmdvel_to_motor.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidCmdVelToMotor>());
  rclcpp::shutdown();
  return 0;
}