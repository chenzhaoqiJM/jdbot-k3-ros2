/**
 * ESP32-C6 底盘控制节点入口
 */

#include <rclcpp/rclcpp.hpp>

#include "jdbot_esp32_c6_control/esp32_c6_control_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<jdbot_esp32_c6_control::Esp32C6ControlNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("esp32_c6_control_node"), "Exception: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
