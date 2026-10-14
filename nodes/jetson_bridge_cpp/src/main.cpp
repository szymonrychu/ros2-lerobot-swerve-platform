// ros2 run jetson_bridge_cpp jetson_bridge_node
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "jetson_bridge_cpp/bridge_node.hpp"
#include "jetson_bridge_cpp/config.hpp"

int main(int argc, char ** argv)
{
  const jetson_bridge_cpp::BridgeConfig config = jetson_bridge_cpp::load_config_or_exit();
  rclcpp::init(argc, argv);
  auto node = std::make_shared<jetson_bridge_cpp::JetsonBridgeNode>(config);
  if (const auto port = ros2_common_cpp::resolve_metrics_port(config.metrics_port)) {
    node->metrics().serve(*port);
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
