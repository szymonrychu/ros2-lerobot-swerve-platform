#include "map3d_manager_cpp/config.hpp"

#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace map3d_manager_cpp
{

const std::vector<std::string> & config_keys()
{
  static const std::vector<std::string> keys = {
    "command_topic",       "map_reset_topic", "result_topic",      "status_topic",  "rtabmap_namespace",
    "database_dir",        "status_period_s", "service_timeout_s", "input_stale_s", "correction_warn_m",
    "correction_warn_deg", "rgb_info_topic",  "depth_info_topic",  "rtabmap_unit"};
  return keys;
}

Config parse_config(const YAML::Node & node, ros2_common_cpp::Errors & errors)
{
  Config c;
  if (!node.IsMap() && !node.IsNull()) {
    errors.push_back("config: expected a mapping");
    return c;
  }
  ros2_common_cpp::forbid_unknown(node, config_keys(), errors);
  for (auto [key, field] : std::initializer_list<std::pair<const char *, std::string *>>{
         {"command_topic", &c.command_topic},
         {"map_reset_topic", &c.map_reset_topic},
         {"result_topic", &c.result_topic},
         {"status_topic", &c.status_topic},
         {"rtabmap_namespace", &c.rtabmap_namespace},
         {"database_dir", &c.database_dir},
         {"rgb_info_topic", &c.rgb_info_topic},
         {"depth_info_topic", &c.depth_info_topic},
         {"rtabmap_unit", &c.rtabmap_unit}}) {
    *field = ros2_common_cpp::get_or<std::string>(node, key, *field, errors);
  }
  for (auto [key, field] : std::initializer_list<std::pair<const char *, double *>>{
         {"status_period_s", &c.status_period_s},
         {"service_timeout_s", &c.service_timeout_s},
         {"input_stale_s", &c.input_stale_s}}) {
    *field = ros2_common_cpp::get_or<double>(node, key, *field, errors);
    ros2_common_cpp::gt(key, *field, 0.0, errors);
  }
  for (auto [key, field] : std::initializer_list<std::pair<const char *, double *>>{
         {"correction_warn_m", &c.correction_warn_m}, {"correction_warn_deg", &c.correction_warn_deg}}) {
    *field = ros2_common_cpp::get_or<double>(node, key, *field, errors);
    ros2_common_cpp::ge(key, *field, 0.0, errors);
  }
  return c;
}

ros2_common_cpp::Errors validate_config(const YAML::Node & node)
{
  ros2_common_cpp::Errors errors;
  parse_config(node, errors);
  return errors;
}

Config load_config_or_exit()
{
  const YAML::Node node =
    ros2_common_cpp::load_config_or_exit(ENV_CONFIG_PATH_KEY, DEFAULT_CONFIG_PATH, validate_config);
  ros2_common_cpp::Errors errors;
  return parse_config(node, errors);
}

}  // namespace map3d_manager_cpp
