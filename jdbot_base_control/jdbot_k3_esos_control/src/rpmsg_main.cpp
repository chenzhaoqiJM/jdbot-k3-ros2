/**
 * 旧协议电机控制节点入口
 */

#include <rclcpp/rclcpp.hpp>

#include "jdbot_k3_esos_control/rpmsg_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<jdbot_k3_esos_control::RpmsgLegacyNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rpmsg_legacy_node"), "Exception: %s",
                 e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
