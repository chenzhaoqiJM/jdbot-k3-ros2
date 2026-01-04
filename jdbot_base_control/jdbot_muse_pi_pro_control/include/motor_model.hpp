#pragma once

namespace motor_model
{
    // direction: 0 stop, 1 forward, 2 backward
    double motor1_model(int direction, double wheel_speed_rps);
    double motor2_model(int direction, double wheel_speed_rps);
}
