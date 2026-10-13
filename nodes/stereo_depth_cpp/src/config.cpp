#include "stereo_depth_cpp/config.hpp"

#include <string>
#include <vector>

namespace stereo_depth_cpp
{

namespace
{

using ros2_common_cpp::Errors;

constexpr const char * WHITESPACE = " \t\r\n\f\v";

std::string strip(const std::string & value)
{
  const auto first = value.find_first_not_of(WHITESPACE);
  if (first == std::string::npos) {
    return "";
  }
  return value.substr(first, value.find_last_not_of(WHITESPACE) - first + 1);
}

// A string field, stripped; "<key>: <empty_message>" when it is empty after stripping (field_validator).
std::string stripped_non_empty(
  const YAML::Node & node, const std::string & key, const std::string & fallback, const std::string & empty_message,
  Errors & errors)
{
  if (!node[key].IsDefined()) {
    return fallback;
  }
  const auto value = ros2_common_cpp::convert<std::string>(node[key], key, errors);
  if (!value) {
    return fallback;
  }
  const std::string out = strip(*value);
  if (out.empty()) {
    errors.push_back(key + ": " + empty_message);
  }
  return out;
}

int int_field(const YAML::Node & node, const std::string & key, int fallback, Errors & errors)
{
  return ros2_common_cpp::get_or<int>(node, key, fallback, errors);
}

double float_field(const YAML::Node & node, const std::string & key, double fallback, Errors & errors)
{
  return ros2_common_cpp::get_or<double>(node, key, fallback, errors);
}

// Parse every field with the pydantic bounds; field errors land in errors and the relations (p1 < p2,
// min_depth_m < max_depth_m) are checked only when every field is valid, as the model_validator runs after them.
DepthConfig parse(const YAML::Node & node, Errors & errors)
{
  using ros2_common_cpp::ge;
  using ros2_common_cpp::gt;
  using ros2_common_cpp::le;
  const DepthConfig d;
  const std::size_t before = errors.size();
  DepthConfig c;
  const std::string topic_empty = "topic must not be empty";
  c.left_image_topic = stripped_non_empty(node, "left_image_topic", d.left_image_topic, topic_empty, errors);
  c.right_image_topic = stripped_non_empty(node, "right_image_topic", d.right_image_topic, topic_empty, errors);
  c.depth_topic = stripped_non_empty(node, "depth_topic", d.depth_topic, topic_empty, errors);
  c.camera_info_topic = stripped_non_empty(node, "camera_info_topic", d.camera_info_topic, topic_empty, errors);
  c.publish_disparity = ros2_common_cpp::get_or<bool>(node, "publish_disparity", d.publish_disparity, errors);
  c.disparity_topic = stripped_non_empty(node, "disparity_topic", d.disparity_topic, topic_empty, errors);
  c.calibration_dir = ros2_common_cpp::get_or<std::string>(node, "calibration_dir", d.calibration_dir, errors);

  c.calibration_check_s = float_field(node, "calibration_check_s", d.calibration_check_s, errors);
  gt("calibration_check_s", c.calibration_check_s, 0.0, errors);

  c.num_disparities = int_field(node, "num_disparities", d.num_disparities, errors);
  gt("num_disparities", c.num_disparities, 0, errors);
  if (c.num_disparities % DISPARITY_MULTIPLE != 0) {
    errors.push_back("num_disparities: must be a multiple of " + std::to_string(DISPARITY_MULTIPLE));
  }
  c.block_size = int_field(node, "block_size", d.block_size, errors);
  ge("block_size", c.block_size, 1, errors);
  le("block_size", c.block_size, MAX_BLOCK_SIZE, errors);
  if (c.block_size % 2 == 0) {
    errors.push_back("block_size: must be odd");
  }
  c.p1 = int_field(node, "p1", d.p1, errors);
  gt("p1", c.p1, 0, errors);
  c.p2 = int_field(node, "p2", d.p2, errors);
  gt("p2", c.p2, 0, errors);
  c.uniqueness_ratio = int_field(node, "uniqueness_ratio", d.uniqueness_ratio, errors);
  ge("uniqueness_ratio", c.uniqueness_ratio, 0, errors);
  c.speckle_window_size = int_field(node, "speckle_window_size", d.speckle_window_size, errors);
  ge("speckle_window_size", c.speckle_window_size, 0, errors);
  c.speckle_range = int_field(node, "speckle_range", d.speckle_range, errors);
  ge("speckle_range", c.speckle_range, 0, errors);
  c.disp12_max_diff = int_field(node, "disp12_max_diff", d.disp12_max_diff, errors);
  ge("disp12_max_diff", c.disp12_max_diff, -1, errors);
  c.pre_filter_cap = int_field(node, "pre_filter_cap", d.pre_filter_cap, errors);
  ge("pre_filter_cap", c.pre_filter_cap, 1, errors);
  le("pre_filter_cap", c.pre_filter_cap, MAX_PRE_FILTER_CAP, errors);

  c.min_depth_m = float_field(node, "min_depth_m", d.min_depth_m, errors);
  gt("min_depth_m", c.min_depth_m, 0.0, errors);
  c.max_depth_m = float_field(node, "max_depth_m", d.max_depth_m, errors);
  gt("max_depth_m", c.max_depth_m, 0.0, errors);
  c.sync_slop_s = float_field(node, "sync_slop_s", d.sync_slop_s, errors);
  gt("sync_slop_s", c.sync_slop_s, 0.0, errors);
  le("sync_slop_s", c.sync_slop_s, MAX_SYNC_SLOP_S, errors);
  c.max_rate_hz = float_field(node, "max_rate_hz", d.max_rate_hz, errors);
  gt("max_rate_hz", c.max_rate_hz, 0.0, errors);
  le("max_rate_hz", c.max_rate_hz, MAX_RATE_HZ, errors);
  c.matcher_threads = int_field(node, "matcher_threads", d.matcher_threads, errors);
  ge("matcher_threads", c.matcher_threads, 1, errors);
  le("matcher_threads", c.matcher_threads, MAX_MATCHER_THREADS, errors);

  c.frame_id = stripped_non_empty(node, "frame_id", d.frame_id, "must not be empty", errors);

  const YAML::Node port = node["metrics_port"];
  if (port.IsDefined() && port.IsNull()) {
    c.metrics_port = std::nullopt;
  } else if (port.IsDefined()) {
    const auto value = ros2_common_cpp::convert<int>(port, "metrics_port", errors);
    if (value) {
      ge("metrics_port", *value, 1, errors);
      le("metrics_port", *value, MAX_PORT, errors);
      c.metrics_port = *value;
    }
  }

  if (errors.size() == before) {
    if (c.p1 >= c.p2) {
      errors.push_back("p1 must be below p2");
    }
    if (c.min_depth_m >= c.max_depth_m) {
      errors.push_back("min_depth_m must be below max_depth_m");
    }
  }
  return c;
}

}  // namespace

const std::vector<std::string> & config_keys()
{
  static const std::vector<std::string> keys{
    "left_image_topic",
    "right_image_topic",
    "depth_topic",
    "camera_info_topic",
    "publish_disparity",
    "disparity_topic",
    "calibration_dir",
    "calibration_check_s",
    "num_disparities",
    "block_size",
    "p1",
    "p2",
    "uniqueness_ratio",
    "speckle_window_size",
    "speckle_range",
    "disp12_max_diff",
    "pre_filter_cap",
    "min_depth_m",
    "max_depth_m",
    "sync_slop_s",
    "max_rate_hz",
    "matcher_threads",
    "frame_id",
    "metrics_port"};
  return keys;
}

ros2_common_cpp::Errors validate(const YAML::Node & node)
{
  Errors errors;
  ros2_common_cpp::forbid_unknown(node, config_keys(), errors);
  if (!node.IsMap()) {
    return errors;
  }
  parse(node, errors);
  return errors;
}

DepthConfig from_yaml(const YAML::Node & node)
{
  Errors ignored;
  return parse(node, ignored);
}

ConfigLoad load_config(const std::string & path)
{
  const auto result = ros2_common_cpp::load_config(path, validate);
  if (!result.errors.empty()) {
    return {DepthConfig{}, result.errors};
  }
  return {from_yaml(result.node), {}};
}

DepthConfig load_config_or_exit()
{
  return from_yaml(ros2_common_cpp::load_config_or_exit(CONFIG_ENV_VAR, DEFAULT_CONFIG_PATH, validate));
}

}  // namespace stereo_depth_cpp
