#pragma once

#include <cstdint>

namespace robot_config {

/* ================== RPMsg 配置 ================== */

// RPMsg 设备路径
constexpr const char* RPMSG_CTRL_DEV = "/dev/rpmsg_ctrl0";
constexpr const char* RPMSG_DATA_DEV = "/dev/rpmsg0";

// RPMsg 服务配置 - 必须与 RCPU 端匹配
constexpr const char* RPMSG_SERVICE_NAME = "rpmsg:motor_ctrl";
constexpr uint32_t RPMSG_LOCAL_ADDR = 1003;   // Linux 端地址
constexpr uint32_t RPMSG_REMOTE_ADDR = 1002;  // RCPU 端地址

}  // namespace robot_config
