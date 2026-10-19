// Configuration of the 3D map manager. Loaded from MAP3D_MANAGER_CONFIG (default
// /etc/ros2/map3d_manager/config.yaml) through ros2_common_cpp::load_config_or_exit: unknown keys, wrong types and
// out-of-range values are errors and exit 78 (ros2_common_cpp::EXIT_CONFIG). The topic defaults equal the map3d topic
// contract.
#ifndef MAP3D_MANAGER_CPP__CONFIG_HPP_
#define MAP3D_MANAGER_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"

namespace map3d_manager_cpp
{

constexpr const char * ENV_CONFIG_PATH_KEY = "MAP3D_MANAGER_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/map3d_manager/config.yaml";

// Every setting of the manager.
struct Config
{
  std::string command_topic = "/uplink/map3d/command";
  std::string map_reset_topic = "/uplink/map_guard/map_reset";
  std::string result_topic = "/map3d/result_server";
  std::string status_topic = "/map3d/status_server";
  std::string rtabmap_namespace = "/rtabmap";
  std::string database_dir = "/var/lib/ros2/rtabmap";
  double status_period_s = 1.0;                                      // > 0
  double service_timeout_s = 10.0;                                   // > 0
  double input_stale_s = 3.0;                                        // > 0
  double correction_warn_m = 0.05;                                   // >= 0
  double correction_warn_deg = 2.0;                                  // >= 0
  std::string rgb_info_topic = "/uplink/head_rgbd/rgb/camera_info";  // head_rgbd_cpp pair
  std::string depth_info_topic = "/uplink/head_rgbd/depth/camera_info";
  std::string rtabmap_unit = "ros2-rtabmap_mapping.service";  // systemd unit whose cgroup memory is reported
};

// Every field name of Config (the keys allowed in the YAML file).
const std::vector<std::string> & config_keys();

// Read a YAML mapping into a Config (defaults for missing keys), recording every error as "<key>: <problem>".
Config parse_config(const YAML::Node & node, ros2_common_cpp::Errors & errors);

// Errors of a YAML mapping (empty when valid): the ros2_common_cpp::Validator of this node.
ros2_common_cpp::Errors validate_config(const YAML::Node & node);

// Load MAP3D_MANAGER_CONFIG (default DEFAULT_CONFIG_PATH); on a missing file or any error log to stderr and exit 78.
Config load_config_or_exit();

}  // namespace map3d_manager_cpp

#endif  // MAP3D_MANAGER_CPP__CONFIG_HPP_
