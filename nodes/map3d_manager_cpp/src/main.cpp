// map3d_manager_node: the standalone executable (config from MAP3D_MANAGER_CONFIG, exit 78 on a bad config).
#include <memory>

#include "map3d_manager_cpp/map3d_manager_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  const map3d_manager_cpp::Config config = map3d_manager_cpp::load_config_or_exit();
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<map3d_manager_cpp::Map3dManagerNode>(config);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
