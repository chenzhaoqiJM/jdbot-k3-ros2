#pragma once
#include <cstddef>
#include <cstdint>

namespace robot_config {
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

// encoder
constexpr double ENCODER_PPR = 11;
constexpr double ENCODER_EDGES = 1;
constexpr double ENCODER_GEAR_RATIO = 56.0;
constexpr double ENCODER_ALPHA = 0.1;
constexpr size_t ENCODER_QUEUE_SIZE = 2;
} // namespace robot_config
