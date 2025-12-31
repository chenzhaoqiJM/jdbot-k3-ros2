#include "jdbot_muse_pi_pro_control/motor_direction_ctrl.hpp"
#include "jdbot_muse_pi_pro_control/robot_config.hpp"
#include <stdexcept>

using namespace robot_config;

MotorDirectionCtrl::MotorDirectionCtrl()
{
    chip_ = lgGpiochipOpen(0); // /dev/gpiochip0
    if (chip_ < 0)
        throw std::runtime_error("Failed to open /dev/gpiochip0");

    // motor1
    claim_output(MOTOR1_IN1);
    claim_output(MOTOR1_IN2);

    // motor2
    claim_output(MOTOR2_IN1);
    claim_output(MOTOR2_IN2);
}

MotorDirectionCtrl::~MotorDirectionCtrl()
{
    lgGpiochipClose(chip_);
}

void MotorDirectionCtrl::claim_output(int gpio)
{
    lgGpioClaimOutput(chip_, 0, gpio, 0);
}

void MotorDirectionCtrl::set_pair(int in1, int in2, int direction)
{
    if (direction == 0)
    {
        lgGpioWrite(chip_, in1, 0);
        lgGpioWrite(chip_, in2, 0);
    }
    else if (direction == 1)
    {
        lgGpioWrite(chip_, in1, 1);
        lgGpioWrite(chip_, in2, 0);
    }
    else if (direction == 2)
    {
        lgGpioWrite(chip_, in1, 0);
        lgGpioWrite(chip_, in2, 1);
    }
}

void MotorDirectionCtrl::motor1_direction(int direction)
{
    set_pair(MOTOR1_IN1, MOTOR1_IN2, direction);
}

void MotorDirectionCtrl::motor2_direction(int direction)
{
    set_pair(MOTOR2_IN1, MOTOR2_IN2, direction);
}
