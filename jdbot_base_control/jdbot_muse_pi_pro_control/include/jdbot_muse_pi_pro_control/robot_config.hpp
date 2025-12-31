#pragma once

namespace robot_config
{
    /* ================== GPIO 引脚 ================== */

    // motor1
    constexpr int MOTOR1_IN1 = 49;
    constexpr int MOTOR1_IN2 = 50;

    // motor2
    constexpr int MOTOR2_IN1 = 91;
    constexpr int MOTOR2_IN2 = 92;

    /* ================== 运动学参数 ================== */

    // 轮径（m）
    constexpr double WHEEL_DIAMETER = 0.067;

    // 轮距（m）
    constexpr double WHEEL_BASE = 0.275;
}
