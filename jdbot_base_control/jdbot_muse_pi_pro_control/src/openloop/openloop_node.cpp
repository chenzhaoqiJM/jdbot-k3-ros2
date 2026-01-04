#include "openloop/cmdvel_to_motor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToMotor>());
    rclcpp::shutdown();
    return 0;
}
