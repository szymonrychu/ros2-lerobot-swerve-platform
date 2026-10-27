#include "head_rgbd_cpp/config.hpp"

#include <string>
#include <vector>

namespace head_rgbd_cpp
{

namespace
{

constexpr long MAX_PORT = 65535;  // NOLINT(runtime/int)
constexpr int MAX_DIVISOR = 2;
constexpr double MAX_OUTPUT_RATE_HZ = 15.0;
constexpr int MAX_JPEG_QUALITY = 100;
constexpr double MAX_CLOUD_RATE_HZ = 10.0;

// The key as written in errors: "<prefix><key>".
std::string full_key(const std::string & prefix, const char * key) { return prefix + key; }

// The value under node[key] as T (fallback when missing or invalid) with the lax pydantic-style coercions; errors
// name prefix + key.
template <typename T>
T read(const YAML::Node & node, const std::string & prefix, const char * key, const T & fallback, Errors & errors)
{
  const YAML::Node value = node[key];
  if (!value.IsDefined()) {
    return fallback;
  }
  return ros2_common_cpp::convert<T>(value, full_key(prefix, key), errors, ros2_common_cpp::Coercion::LAX)
    .value_or(fallback);
}

// A section mapping: the node under key (undefined when absent), after rejecting unknown keys inside it.
YAML::Node section(const YAML::Node & root, const char * key, const std::vector<std::string> & allowed, Errors & errors)
{
  const YAML::Node node = root[key];
  if (!node.IsDefined()) {
    return node;
  }
  ros2_common_cpp::forbid_unknown(node, allowed, errors, key);
  return node.IsMap() ? node : YAML::Node();
}

Config parse(const YAML::Node & node, Errors & errors)
{
  Config c;
  ros2_common_cpp::forbid_unknown(
    node,
    {"depth_topic", "rgb_image_topic", "rgb_info_topic", "output_namespace", "output", "depth", "cloud", "lazy",
     "metrics_port"},
    errors);
  if (!node.IsMap()) {
    return c;
  }
  const std::string none;
  c.depth_topic = read<std::string>(node, none, "depth_topic", c.depth_topic, errors);
  c.rgb_image_topic = read<std::string>(node, none, "rgb_image_topic", c.rgb_image_topic, errors);
  c.rgb_info_topic = read<std::string>(node, none, "rgb_info_topic", c.rgb_info_topic, errors);
  c.output_namespace = read<std::string>(node, none, "output_namespace", c.output_namespace, errors);

  if (const YAML::Node s = section(node, "output", {"divisor", "rate_hz", "jpeg_quality"}, errors); s.IsDefined()) {
    const std::string p = "output.";
    c.output.divisor = read<int>(s, p, "divisor", c.output.divisor, errors);
    ros2_common_cpp::ge<int>(p + "divisor", c.output.divisor, 1, errors);
    ros2_common_cpp::le<int>(p + "divisor", c.output.divisor, MAX_DIVISOR, errors);
    c.output.rate_hz = read<double>(s, p, "rate_hz", c.output.rate_hz, errors);
    ros2_common_cpp::gt(p + "rate_hz", c.output.rate_hz, 0.0, errors);
    ros2_common_cpp::le(p + "rate_hz", c.output.rate_hz, MAX_OUTPUT_RATE_HZ, errors);
    c.output.jpeg_quality = read<int>(s, p, "jpeg_quality", c.output.jpeg_quality, errors);
    ros2_common_cpp::ge<int>(p + "jpeg_quality", c.output.jpeg_quality, 1, errors);
    ros2_common_cpp::le<int>(p + "jpeg_quality", c.output.jpeg_quality, MAX_JPEG_QUALITY, errors);
  }

  if (const YAML::Node s = section(node, "depth", {"min_m", "max_m"}, errors); s.IsDefined()) {
    c.depth.min_m = read<double>(s, "depth.", "min_m", c.depth.min_m, errors);
    ros2_common_cpp::gt("depth.min_m", c.depth.min_m, 0.0, errors);
    c.depth.max_m = read<double>(s, "depth.", "max_m", c.depth.max_m, errors);
  }
  if (!(c.depth.max_m > c.depth.min_m)) {
    errors.push_back(
      "depth.max_m: (" + std::to_string(c.depth.max_m) + ") must be above depth.min_m (" +
      std::to_string(c.depth.min_m) + ")");
  }

  if (const YAML::Node s = section(node, "cloud", {"enabled", "rate_hz"}, errors); s.IsDefined()) {
    c.cloud.enabled = read<bool>(s, "cloud.", "enabled", c.cloud.enabled, errors);
    c.cloud.rate_hz = read<double>(s, "cloud.", "rate_hz", c.cloud.rate_hz, errors);
    ros2_common_cpp::gt("cloud.rate_hz", c.cloud.rate_hz, 0.0, errors);
    ros2_common_cpp::le("cloud.rate_hz", c.cloud.rate_hz, MAX_CLOUD_RATE_HZ, errors);
  }

  if (const YAML::Node s = section(node, "lazy", {"idle_unsubscribe_s"}, errors); s.IsDefined()) {
    c.lazy.idle_unsubscribe_s = read<double>(s, "lazy.", "idle_unsubscribe_s", c.lazy.idle_unsubscribe_s, errors);
    ros2_common_cpp::ge("lazy.idle_unsubscribe_s", c.lazy.idle_unsubscribe_s, 0.0, errors);
  }

  if (const YAML::Node port = node["metrics_port"]; port.IsDefined()) {
    if (ros2_common_cpp::is_yaml_null(port)) {
      c.metrics_port.reset();
    } else if (const auto value = ros2_common_cpp::lax_int(port)) {
      c.metrics_port = *value;
      ros2_common_cpp::ge<long>("metrics_port", *value, 1, errors);         // NOLINT(runtime/int)
      ros2_common_cpp::le<long>("metrics_port", *value, MAX_PORT, errors);  // NOLINT(runtime/int)
    } else {
      errors.push_back("metrics_port: expected int or null");
    }
  }
  return c;
}

}  // namespace

Errors validate(const YAML::Node & node)
{
  Errors errors;
  parse(node, errors);
  return errors;
}

Config from_yaml(const YAML::Node & node)
{
  Errors errors;
  return parse(node, errors);
}

Config load_config_or_exit()
{
  return from_yaml(ros2_common_cpp::load_config_or_exit(ENV_CONFIG_PATH_KEY, DEFAULT_CONFIG_PATH, validate));
}

}  // namespace head_rgbd_cpp
