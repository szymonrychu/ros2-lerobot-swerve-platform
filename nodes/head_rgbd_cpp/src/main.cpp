#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "head_rgbd_cpp/config.hpp"
#include "head_rgbd_cpp/head_rgbd_node.hpp"

int main(int argc, char ** argv)
{
  const head_rgbd_cpp::Config config = head_rgbd_cpp::load_config_or_exit();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<head_rgbd_cpp::HeadRgbdNode>(config));
  rclcpp::shutdown();
  return 0;
}
