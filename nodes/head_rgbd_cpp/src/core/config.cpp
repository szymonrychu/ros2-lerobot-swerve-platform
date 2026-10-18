#include "head_rgbd_cpp/config.hpp"

#include <string>
#include <vector>

namespace head_rgbd_cpp
{

namespace
{

constexpr long MAX_PORT = 65535;  // NOLINT(runtime/int)
constexpr int MAX_DIVISOR = 2;
constexpr double MAX_PAIR_DT_S = 0.2;
constexpr int MIN_QUEUE_SIZE = 2;
constexpr int MAX_NEIGHBOURS = 8;
constexpr int MAX_RADIUS = 16;
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
    {"tof_depth_topic", "rgb_image_topic", "rgb_info_topic", "output_namespace", "tof_calibration_file", "output",
     "sync", "motion", "filter", "depth", "upsampling", "cloud", "lazy", "metrics_port"},
    errors);
  if (!node.IsMap()) {
    return c;
  }
  const std::string none;
  c.tof_depth_topic = read<std::string>(node, none, "tof_depth_topic", c.tof_depth_topic, errors);
  c.rgb_image_topic = read<std::string>(node, none, "rgb_image_topic", c.rgb_image_topic, errors);
  c.rgb_info_topic = read<std::string>(node, none, "rgb_info_topic", c.rgb_info_topic, errors);
  c.output_namespace = read<std::string>(node, none, "output_namespace", c.output_namespace, errors);
  c.tof_calibration_file = read<std::string>(node, none, "tof_calibration_file", c.tof_calibration_file, errors);

  if (const YAML::Node s = section(node, "output", {"divisor"}, errors); s.IsDefined()) {
    c.output.divisor = read<int>(s, "output.", "divisor", c.output.divisor, errors);
    ros2_common_cpp::ge<int>("output.divisor", c.output.divisor, 1, errors);
    ros2_common_cpp::le<int>("output.divisor", c.output.divisor, MAX_DIVISOR, errors);
  }

  if (const YAML::Node s = section(node, "sync", {"max_pair_dt_s", "queue_size"}, errors); s.IsDefined()) {
    c.sync.max_pair_dt_s = read<double>(s, "sync.", "max_pair_dt_s", c.sync.max_pair_dt_s, errors);
    ros2_common_cpp::gt("sync.max_pair_dt_s", c.sync.max_pair_dt_s, 0.0, errors);
    ros2_common_cpp::le("sync.max_pair_dt_s", c.sync.max_pair_dt_s, MAX_PAIR_DT_S, errors);
    c.sync.queue_size = read<int>(s, "sync.", "queue_size", c.sync.queue_size, errors);
    ros2_common_cpp::ge<int>("sync.queue_size", c.sync.queue_size, MIN_QUEUE_SIZE, errors);
  }

  if (const YAML::Node s = section(
        node, "motion",
        {"compensate", "fixed_frame", "max_angular_rate_deg_s", "max_linear_rate_m_s", "rate_window_s", "tf_timeout_s"},
        errors);
      s.IsDefined()) {
    const std::string p = "motion.";
    c.motion.compensate = read<bool>(s, p, "compensate", c.motion.compensate, errors);
    c.motion.fixed_frame = read<std::string>(s, p, "fixed_frame", c.motion.fixed_frame, errors);
    c.motion.max_angular_rate_deg_s =
      read<double>(s, p, "max_angular_rate_deg_s", c.motion.max_angular_rate_deg_s, errors);
    ros2_common_cpp::gt(p + "max_angular_rate_deg_s", c.motion.max_angular_rate_deg_s, 0.0, errors);
    c.motion.max_linear_rate_m_s = read<double>(s, p, "max_linear_rate_m_s", c.motion.max_linear_rate_m_s, errors);
    ros2_common_cpp::gt(p + "max_linear_rate_m_s", c.motion.max_linear_rate_m_s, 0.0, errors);
    c.motion.rate_window_s = read<double>(s, p, "rate_window_s", c.motion.rate_window_s, errors);
    ros2_common_cpp::gt(p + "rate_window_s", c.motion.rate_window_s, 0.0, errors);
    c.motion.tf_timeout_s = read<double>(s, p, "tf_timeout_s", c.motion.tf_timeout_s, errors);
    ros2_common_cpp::ge(p + "tf_timeout_s", c.motion.tf_timeout_s, 0.0, errors);
  }

  if (const YAML::Node s = section(node, "filter", {"jump_rel", "jump_abs_mm", "min_valid_neighbours"}, errors);
      s.IsDefined()) {
    const std::string p = "filter.";
    c.filter.jump_rel = read<double>(s, p, "jump_rel", c.filter.jump_rel, errors);
    ros2_common_cpp::gt(p + "jump_rel", c.filter.jump_rel, 0.0, errors);
    if (!(c.filter.jump_rel < 1.0)) {
      errors.push_back(p + "jump_rel: must be < 1 (got " + std::to_string(c.filter.jump_rel) + ")");
    }
    c.filter.jump_abs_mm = read<double>(s, p, "jump_abs_mm", c.filter.jump_abs_mm, errors);
    ros2_common_cpp::ge(p + "jump_abs_mm", c.filter.jump_abs_mm, 0.0, errors);
    c.filter.min_valid_neighbours = read<int>(s, p, "min_valid_neighbours", c.filter.min_valid_neighbours, errors);
    ros2_common_cpp::ge<int>(p + "min_valid_neighbours", c.filter.min_valid_neighbours, 0, errors);
    ros2_common_cpp::le<int>(p + "min_valid_neighbours", c.filter.min_valid_neighbours, MAX_NEIGHBOURS, errors);
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

  if (const YAML::Node s = section(node, "upsampling", {"enabled", "radius", "eps", "min_support"}, errors);
      s.IsDefined()) {
    const std::string p = "upsampling.";
    c.upsampling.enabled = read<bool>(s, p, "enabled", c.upsampling.enabled, errors);
    c.upsampling.radius = read<int>(s, p, "radius", c.upsampling.radius, errors);
    ros2_common_cpp::ge<int>(p + "radius", c.upsampling.radius, 1, errors);
    ros2_common_cpp::le<int>(p + "radius", c.upsampling.radius, MAX_RADIUS, errors);
    c.upsampling.eps = read<double>(s, p, "eps", c.upsampling.eps, errors);
    ros2_common_cpp::gt(p + "eps", c.upsampling.eps, 0.0, errors);
    c.upsampling.min_support = read<double>(s, p, "min_support", c.upsampling.min_support, errors);
    ros2_common_cpp::gt(p + "min_support", c.upsampling.min_support, 0.0, errors);
    ros2_common_cpp::le(p + "min_support", c.upsampling.min_support, 1.0, errors);
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
