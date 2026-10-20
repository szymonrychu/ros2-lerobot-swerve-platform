#include <pcl/console/print.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ground_estimator_cpp/config.hpp"
#include "ground_estimator_cpp/ground_node.hpp"

int main(int argc, char ** argv)
{
  const ground_estimator_cpp::Config config = ground_estimator_cpp::load_config_or_exit();
  rclcpp::init(argc, argv);
  // A rejected fit is counted in ground_fits_total{result}; PCL's own "no solution found" lines would log it at 10 Hz.
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  // Single-threaded: the cloud, IMU and RGB-D callbacks and the state timer never run concurrently.
  rclcpp::executors::SingleThreadedExecutor executor;
  const auto node = std::make_shared<ground_estimator_cpp::GroundNode>(config);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
