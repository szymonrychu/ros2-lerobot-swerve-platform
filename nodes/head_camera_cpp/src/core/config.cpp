#include "head_camera_cpp/config.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace head_camera_cpp
{

namespace
{

constexpr int MAX_JPEG_QUALITY = 100;
constexpr double MAX_MAPPING_RATE_HZ = 10.0;
constexpr int MAX_PORT = 65535;
constexpr int MAX_ZOOM_PX_LIMIT = 1024;
constexpr int ASPECT_TOLERANCE_PX = 1;
const std::vector<std::string> AF_MODES = {"continuous", "auto", "manual"};
const std::vector<int> ALLOWED_FACTORS = {2, 4, 8};
const auto LAX = ros2_common_cpp::Coercion::LAX;

// get_or for a key of a nested mapping: errors are recorded as "<prefix><key>: ...".
template <typename T>
T field(
  const YAML::Node & node, const std::string & prefix, const std::string & key, const T & fallback, Errors & errors)
{
  Errors local;
  const T value = ros2_common_cpp::get_or<T>(node, key, fallback, local, LAX);
  for (const auto & line : local) {
    errors.push_back(prefix + line);
  }
  return value;
}

// Read a two-element list of positive ints (width, height) from node[key].
std::array<int, 2> size_pair(
  const YAML::Node & node, const std::string & prefix, const std::string & key, const std::array<int, 2> & fallback,
  Errors & errors)
{
  const YAML::Node value = node[key];
  if (!value.IsDefined()) {
    return fallback;
  }
  const std::string name = prefix + key;
  if (!value.IsSequence() || value.size() != 2) {
    errors.push_back(name + ": expected list of 2 ints");
    return fallback;
  }
  const auto first = ros2_common_cpp::lax_int(value[0]);
  const auto second = ros2_common_cpp::lax_int(value[1]);
  if (!first || !second) {
    errors.push_back(name + ": expected list of 2 ints");
    return fallback;
  }
  if (*first <= 0 || *second <= 0) {
    errors.push_back(name + ": must be > 0");
    return fallback;
  }
  return {static_cast<int>(*first), static_cast<int>(*second)};
}

// An int in [low, high].
int bounded_int(
  const YAML::Node & node, const std::string & prefix, const std::string & key, int fallback, int low, int high,
  Errors & errors)
{
  const int value = field<int>(node, prefix, key, fallback, errors);
  ros2_common_cpp::ge(prefix + key, value, low, errors);
  ros2_common_cpp::le(prefix + key, value, high, errors);
  return value;
}

// A float > 0.
double positive(
  const YAML::Node & node, const std::string & prefix, const std::string & key, double fallback, Errors & errors)
{
  const double value = field<double>(node, prefix, key, fallback, errors);
  ros2_common_cpp::gt(prefix + key, value, 0.0, errors);
  return value;
}

std::string number_text(double value)
{
  std::ostringstream out;
  out << value;
  return out.str();
}

ZoomConfig parse_zoom(const YAML::Node & node, Errors & errors)
{
  ZoomConfig z;
  const std::string p = "zoom.";
  ros2_common_cpp::forbid_unknown(
    node,
    {"enabled", "still_size", "factors", "default_max_px", "min_max_px", "max_max_px", "jpeg_quality", "af_timeout_s",
     "switch_timeout_s", "settle_frames", "still_buffers", "watchdog_s", "restore_retry_delay_s", "af_window_fraction"},
    errors, "zoom");
  if (!node.IsMap()) {
    return z;
  }
  z.enabled = field<bool>(node, p, "enabled", z.enabled, errors);
  z.still_size = size_pair(node, p, "still_size", z.still_size, errors);

  if (const YAML::Node factors = node["factors"]; factors.IsDefined()) {
    std::vector<int> parsed;
    bool ok = factors.IsSequence() && factors.size() > 0;
    if (!ok) {
      errors.push_back(p + "factors: expected non-empty list of 2, 4, 8");
    } else {
      for (const auto & item : factors) {
        const auto value = ros2_common_cpp::lax_int(item);
        if (!value || std::find(ALLOWED_FACTORS.begin(), ALLOWED_FACTORS.end(), *value) == ALLOWED_FACTORS.end()) {
          errors.push_back(p + "factors: every factor must be one of 2, 4, 8");
          ok = false;
          break;
        }
        parsed.push_back(static_cast<int>(*value));
      }
      if (ok) {
        z.factors = parsed;
      }
    }
  }

  z.default_max_px = bounded_int(node, p, "default_max_px", z.default_max_px, 1, MAX_ZOOM_PX_LIMIT, errors);
  z.min_max_px = bounded_int(node, p, "min_max_px", z.min_max_px, 1, MAX_ZOOM_PX_LIMIT, errors);
  z.max_max_px = bounded_int(node, p, "max_max_px", z.max_max_px, 1, MAX_ZOOM_PX_LIMIT, errors);
  if (z.min_max_px > z.default_max_px) {
    errors.push_back(
      p + "min_max_px: (" + std::to_string(z.min_max_px) + ") must not exceed default_max_px (" +
      std::to_string(z.default_max_px) + ")");
  }
  if (z.default_max_px > z.max_max_px) {
    errors.push_back(
      p + "default_max_px: (" + std::to_string(z.default_max_px) + ") must not exceed max_max_px (" +
      std::to_string(z.max_max_px) + ")");
  }
  z.jpeg_quality = bounded_int(node, p, "jpeg_quality", z.jpeg_quality, 1, MAX_JPEG_QUALITY, errors);
  z.af_timeout_s = positive(node, p, "af_timeout_s", z.af_timeout_s, errors);
  z.switch_timeout_s = positive(node, p, "switch_timeout_s", z.switch_timeout_s, errors);
  z.settle_frames = field<int>(node, p, "settle_frames", z.settle_frames, errors);
  ros2_common_cpp::ge(p + "settle_frames", z.settle_frames, 0, errors);
  z.still_buffers = field<int>(node, p, "still_buffers", z.still_buffers, errors);
  ros2_common_cpp::ge(p + "still_buffers", z.still_buffers, 1, errors);
  z.watchdog_s = field<double>(node, p, "watchdog_s", z.watchdog_s, errors);
  if (!(z.watchdog_s > z.af_timeout_s + z.switch_timeout_s)) {
    errors.push_back(
      p + "watchdog_s: (" + number_text(z.watchdog_s) + ") must exceed af_timeout_s + switch_timeout_s (" +
      number_text(z.af_timeout_s + z.switch_timeout_s) + ")");
  }
  z.restore_retry_delay_s = field<double>(node, p, "restore_retry_delay_s", z.restore_retry_delay_s, errors);
  ros2_common_cpp::ge(p + "restore_retry_delay_s", z.restore_retry_delay_s, 0.0, errors);
  z.af_window_fraction = field<double>(node, p, "af_window_fraction", z.af_window_fraction, errors);
  ros2_common_cpp::gt(p + "af_window_fraction", z.af_window_fraction, 0.0, errors);
  ros2_common_cpp::le(p + "af_window_fraction", z.af_window_fraction, 1.0, errors);
  return z;
}

MappingConfig parse_mapping(const YAML::Node & node, const Config & c, Errors & errors)
{
  MappingConfig m;
  const std::string p = "mapping.";
  ros2_common_cpp::forbid_unknown(
    node, {"enabled", "lock_focus", "rate_hz", "width", "height", "jpeg_quality"}, errors, "mapping");
  if (!node.IsMap()) {
    return m;
  }
  m.enabled = field<bool>(node, p, "enabled", m.enabled, errors);
  m.lock_focus = field<bool>(node, p, "lock_focus", m.lock_focus, errors);
  m.rate_hz = positive(node, p, "rate_hz", m.rate_hz, errors);
  ros2_common_cpp::le(p + "rate_hz", m.rate_hz, MAX_MAPPING_RATE_HZ, errors);
  m.width = field<int>(node, p, "width", m.width, errors);
  m.height = field<int>(node, p, "height", m.height, errors);
  for (const auto & [key, value, limit] :
       {std::tuple<const char *, int, int>{"width", m.width, c.width}, {"height", m.height, c.height}}) {
    const std::string name = p + key;
    ros2_common_cpp::gt<int>(name, value, 0, errors);
    if (value > 0 && value % 2 != 0) {
      errors.push_back(name + ": (" + std::to_string(value) + ") must be even");
    }
    if (value > limit) {
      errors.push_back(
        name + ": (" + std::to_string(value) + ") must not exceed the stream " + key + " " + std::to_string(limit));
    }
  }
  if (m.width > 0 && m.height > 0 && c.width > 0) {
    const double expected = static_cast<double>(m.width) * c.height / c.width;
    if (std::abs(expected - m.height) > ASPECT_TOLERANCE_PX) {
      errors.push_back(
        p + "height: (" + std::to_string(m.height) + ") must match the stream aspect, about " +
        number_text(std::round(expected)) + " for width " + std::to_string(m.width));
    }
  }
  m.jpeg_quality = bounded_int(node, p, "jpeg_quality", m.jpeg_quality, 1, MAX_JPEG_QUALITY, errors);
  return m;
}

Config parse(const YAML::Node & node, Errors & errors)
{
  Config c;
  ros2_common_cpp::forbid_unknown(
    node,
    {"camera_id", "frame_id", "topic_namespace", "sensor_mode", "width", "height", "frame_duration_us", "af_mode",
     "lens_position", "jpeg_quality", "camera_info_file", "metrics_port", "zoom", "mapping"},
    errors);
  if (!node.IsMap()) {
    return c;
  }
  const std::string none;
  c.camera_id = field<std::string>(node, none, "camera_id", c.camera_id, errors);
  c.frame_id = field<std::string>(node, none, "frame_id", c.frame_id, errors);
  c.topic_namespace = field<std::string>(node, none, "topic_namespace", c.topic_namespace, errors);
  c.sensor_mode = size_pair(node, none, "sensor_mode", c.sensor_mode, errors);

  c.width = field<int>(node, none, "width", c.width, errors);
  c.height = field<int>(node, none, "height", c.height, errors);
  for (const auto & [key, value] : {std::pair<const char *, int>{"width", c.width}, {"height", c.height}}) {
    ros2_common_cpp::gt<int>(key, value, 0, errors);
    if (value > 0 && value % 2 != 0) {
      errors.push_back(std::string(key) + ": (" + std::to_string(value) + ") must be even");
    }
  }
  if (c.width > 0 && c.height > 0) {
    const double expected = static_cast<double>(c.width) * c.sensor_mode[1] / c.sensor_mode[0];
    if (std::abs(expected - c.height) > ASPECT_TOLERANCE_PX) {
      errors.push_back(
        "height: (" + std::to_string(c.height) + ") must match the sensor_mode aspect, about " +
        number_text(std::round(expected)) + " for width " + std::to_string(c.width));
    }
  }

  c.frame_duration_us =
    field<long>(node, none, "frame_duration_us", c.frame_duration_us, errors);     // NOLINT(runtime/int)
  ros2_common_cpp::gt<long>("frame_duration_us", c.frame_duration_us, 0, errors);  // NOLINT(runtime/int)
  c.af_mode = field<std::string>(node, none, "af_mode", c.af_mode, errors);
  if (std::find(AF_MODES.begin(), AF_MODES.end(), c.af_mode) == AF_MODES.end()) {
    errors.push_back("af_mode: '" + c.af_mode + "' is not one of continuous, auto, manual");
  }
  c.lens_position = field<double>(node, none, "lens_position", c.lens_position, errors);
  ros2_common_cpp::ge(std::string("lens_position"), c.lens_position, 0.0, errors);
  c.jpeg_quality = bounded_int(node, none, "jpeg_quality", c.jpeg_quality, 1, MAX_JPEG_QUALITY, errors);

  if (const YAML::Node file = node["camera_info_file"]; file.IsDefined()) {
    if (ros2_common_cpp::is_yaml_null(file)) {
      c.camera_info_file.reset();
    } else if (const auto path = ros2_common_cpp::lax_string(file)) {
      c.camera_info_file = *path;
    } else {
      errors.push_back("camera_info_file: expected string or null");
    }
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

  if (const YAML::Node zoom = node["zoom"]; zoom.IsDefined()) {
    c.zoom = parse_zoom(zoom, errors);
  }
  if (const YAML::Node mapping = node["mapping"]; mapping.IsDefined()) {
    c.mapping = parse_mapping(mapping, c, errors);
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

}  // namespace head_camera_cpp
