#include "jetson_bridge_cpp/config.hpp"

#include <string>
#include <vector>

namespace jetson_bridge_cpp
{

namespace
{

using ros2_common_cpp::Coercion;
using ros2_common_cpp::Errors;

constexpr const char * WHITESPACE = " \t\r\n\f\v";
constexpr const char * ENDPOINT_SCHEME = "tcp://";

std::string strip(const std::string & value)
{
  const auto first = value.find_first_not_of(WHITESPACE);
  if (first == std::string::npos) {
    return "";
  }
  return value.substr(first, value.find_last_not_of(WHITESPACE) - first + 1);
}

// A string field, stripped; "<key>: must not be empty" when it is empty after stripping.
std::string non_empty(const YAML::Node & node, const std::string & key, const std::string & fallback, Errors & errors)
{
  if (!node[key].IsDefined()) {
    return fallback;
  }
  const auto value = ros2_common_cpp::convert<std::string>(node[key], key, errors, Coercion::LAX);
  if (!value) {
    return fallback;
  }
  const std::string out = strip(*value);
  if (out.empty()) {
    errors.push_back(key + ": must not be empty");
  }
  return out;
}

int64_t positive_ms(const YAML::Node & node, const std::string & key, int64_t fallback, Errors & errors)
{
  const auto value =
    ros2_common_cpp::get_or<long>(node, key, static_cast<long>(fallback), errors, Coercion::LAX);  // NOLINT
  ros2_common_cpp::gt<long>(key, value, 0, errors);  // NOLINT(runtime/int)
  return value;
}

BridgeConfig parse(const YAML::Node & node, Errors & errors)
{
  const BridgeConfig d;
  BridgeConfig c;
  c.endpoint = non_empty(node, "endpoint", d.endpoint, errors);
  if (!c.endpoint.empty() && c.endpoint.rfind(ENDPOINT_SCHEME, 0) != 0) {
    errors.push_back(std::string("endpoint: must start with ") + ENDPOINT_SCHEME);
  }
  c.depth_topic = non_empty(node, "depth_topic", d.depth_topic, errors);
  c.camera_info_topic = non_empty(node, "camera_info_topic", d.camera_info_topic, errors);
  c.fused_topic = non_empty(node, "fused_topic", d.fused_topic, errors);
  c.rgb_topic = non_empty(node, "rgb_topic", d.rgb_topic, errors);
  c.rgb_info_topic = non_empty(node, "rgb_info_topic", d.rgb_info_topic, errors);
  c.max_age_ms = positive_ms(node, "max_age_ms", d.max_age_ms, errors);
  c.absent_after_ms = positive_ms(node, "absent_after_ms", d.absent_after_ms, errors);

  const YAML::Node port = node["metrics_port"];
  if (port.IsDefined() && ros2_common_cpp::is_yaml_null(port)) {
    c.metrics_port = std::nullopt;
  } else if (port.IsDefined()) {
    if (const auto value = ros2_common_cpp::convert<int>(port, "metrics_port", errors, Coercion::LAX)) {
      ros2_common_cpp::ge("metrics_port", *value, 1, errors);
      ros2_common_cpp::le("metrics_port", *value, MAX_PORT, errors);
      c.metrics_port = *value;
    }
  }
  return c;
}

}  // namespace

const std::vector<std::string> & config_keys()
{
  static const std::vector<std::string> keys{"endpoint",    "depth_topic",     "camera_info_topic",
                                             "fused_topic", "rgb_topic",       "rgb_info_topic",
                                             "max_age_ms",  "absent_after_ms", "metrics_port"};
  return keys;
}

Errors validate(const YAML::Node & node)
{
  Errors errors;
  ros2_common_cpp::forbid_unknown(node, config_keys(), errors);
  if (node.IsMap()) {
    parse(node, errors);
  }
  return errors;
}

BridgeConfig from_yaml(const YAML::Node & node)
{
  Errors ignored;
  return parse(node, ignored);
}

ConfigLoad load_config(const std::string & path)
{
  const auto result = ros2_common_cpp::load_config(path, validate);
  if (!result.errors.empty()) {
    return {BridgeConfig{}, result.errors};
  }
  return {from_yaml(result.node), {}};
}

BridgeConfig load_config_or_exit()
{
  return from_yaml(ros2_common_cpp::load_config_or_exit(CONFIG_ENV_VAR, DEFAULT_CONFIG_PATH, validate));
}

}  // namespace jetson_bridge_cpp
