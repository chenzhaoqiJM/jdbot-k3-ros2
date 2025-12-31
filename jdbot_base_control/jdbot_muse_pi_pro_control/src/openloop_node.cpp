#include "jdbot_muse_pi_pro_control/cmdvel_to_motor.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToMotor>());
    rclcpp::shutdown();
    return 0;
}
