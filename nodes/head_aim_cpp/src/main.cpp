#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "head_aim_cpp/config.hpp"
#include "head_aim_cpp/head_aim_node.hpp"

int main(int argc, char ** argv)
{
  const head_aim_cpp::Config config = head_aim_cpp::load_config_or_exit();
  rclcpp::init(argc, argv);
  // Single-threaded: the input callbacks and the aim timer never run concurrently (the TF listener has its own thread).
  rclcpp::executors::SingleThreadedExecutor executor;
  const auto node = std::make_shared<head_aim_cpp::HeadAimNode>(config);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
