#include "head_aim_cpp/config.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace head_aim_cpp
{

namespace
{

namespace rc = ros2_common_cpp;

constexpr double MAX_RATE_HZ = 50.0;
constexpr double MAX_TF_TIMEOUT_S = 0.5;

const std::vector<std::string> KEYS = {
  "plan_topic",      "cmd_vel_topic",   "goal_status_topic", "head_command_topic",  "camera_info_topic",
  "base_frame",      "head_frame",      "source_tag",        "yield_sources",       "yield_s",
  "footprint_min_x", "footprint_max_x", "footprint_min_y",   "footprint_max_y",     "lookahead_m",
  "floor_margin_m",  "pan_min_rad",     "pan_max_rad",       "tilt_min_rad",        "tilt_max_rad",
  "home_pan_rad",    "home_tilt_rad",   "reaim_pan_deg",     "reaim_tilt_deg",      "min_reaim_interval_s",
  "rate_hz",         "tf_timeout_s",    "input_timeout_s",   "min_travel_speed_mps"};

// The value under node[key] as T (fallback when missing or invalid) with the lax pydantic-style coercions.
template <typename T>
T read(const YAML::Node & node, const char * key, const T & fallback, Errors & errors)
{
  const YAML::Node value = node[key];
  if (!value.IsDefined()) {
    return fallback;
  }
  return rc::convert<T>(value, key, errors, rc::Coercion::LAX).value_or(fallback);
}

// A non-empty string under node[key].
std::string read_name(const YAML::Node & node, const char * key, const std::string & fallback, Errors & errors)
{
  const std::string value = read<std::string>(node, key, fallback, errors);
  if (value.empty()) {
    errors.push_back(std::string(key) + ": must not be empty");
  }
  return value;
}

// Record "<key>: <problem>" when ok is false.
void require_that(bool ok, const std::string & key, const std::string & problem, Errors & errors)
{
  if (!ok) {
    errors.push_back(key + ": " + problem);
  }
}

// An increasing [low, high] range inside [limit_low, limit_high]; every error names exactly one key.
// @return true when the range is valid.
bool check_range(
  const char * low_key, double low, const char * high_key, double high, double limit_low, double limit_high,
  Errors & errors)
{
  const std::size_t before = errors.size();
  rc::ge(low_key, low, limit_low, errors);
  rc::le(high_key, high, limit_high, errors);
  if (errors.size() == before) {
    require_that(
      low < high, low_key,
      "must be below " + std::string(high_key) + " (" + std::to_string(high) + ", got " + std::to_string(low) + ")",
      errors);
  }
  return errors.size() == before;
}

// value inside [low, high], reported under key.
void check_within(const char * key, double value, double low, double high, Errors & errors)
{
  require_that(
    value >= low && value <= high, key,
    "must lie within " + std::to_string(low) + ".." + std::to_string(high) + " (got " + std::to_string(value) + ")",
    errors);
}

void parse_names(const YAML::Node & node, Config & c, Errors & errors)
{
  c.plan_topic = read_name(node, "plan_topic", c.plan_topic, errors);
  c.cmd_vel_topic = read_name(node, "cmd_vel_topic", c.cmd_vel_topic, errors);
  c.goal_status_topic = read_name(node, "goal_status_topic", c.goal_status_topic, errors);
  c.head_command_topic = read_name(node, "head_command_topic", c.head_command_topic, errors);
  c.camera_info_topic = read_name(node, "camera_info_topic", c.camera_info_topic, errors);
  c.base_frame = read_name(node, "base_frame", c.base_frame, errors);
  c.head_frame = read_name(node, "head_frame", c.head_frame, errors);
  c.source_tag = read_name(node, "source_tag", c.source_tag, errors);
  c.yield_sources = rc::get_or<std::vector<std::string>>(node, "yield_sources", c.yield_sources, errors);
  require_that(
    std::find(c.yield_sources.begin(), c.yield_sources.end(), c.source_tag) == c.yield_sources.end(), "yield_sources",
    "must not contain source_tag (" + c.source_tag + ")", errors);
}

void parse_geometry(const YAML::Node & node, Config & c, Errors & errors)
{
  FootprintBox & f = c.footprint;
  f.min_x = read<double>(node, "footprint_min_x", f.min_x, errors);
  f.max_x = read<double>(node, "footprint_max_x", f.max_x, errors);
  f.min_y = read<double>(node, "footprint_min_y", f.min_y, errors);
  f.max_y = read<double>(node, "footprint_max_y", f.max_y, errors);
  require_that(
    f.min_x < f.max_x, "footprint_min_x", "must be below footprint_max_x (" + std::to_string(f.max_x) + ")", errors);
  require_that(
    f.min_y < f.max_y, "footprint_min_y", "must be below footprint_max_y (" + std::to_string(f.max_y) + ")", errors);
  c.lookahead_m = read<double>(node, "lookahead_m", c.lookahead_m, errors);
  rc::gt("lookahead_m", c.lookahead_m, 0.0, errors);
  c.floor_margin_m = read<double>(node, "floor_margin_m", c.floor_margin_m, errors);
  rc::ge("floor_margin_m", c.floor_margin_m, 0.0, errors);
}

void parse_head(const YAML::Node & node, Config & c, Errors & errors)
{
  c.pan_min_rad = read<double>(node, "pan_min_rad", c.pan_min_rad, errors);
  c.pan_max_rad = read<double>(node, "pan_max_rad", c.pan_max_rad, errors);
  const bool pan_ok = check_range(
    "pan_min_rad", c.pan_min_rad, "pan_max_rad", c.pan_max_rad, SERVO_PAN_MIN_RAD, SERVO_PAN_MAX_RAD, errors);
  c.tilt_min_rad = read<double>(node, "tilt_min_rad", c.tilt_min_rad, errors);
  c.tilt_max_rad = read<double>(node, "tilt_max_rad", c.tilt_max_rad, errors);
  const bool tilt_ok = check_range(
    "tilt_min_rad", c.tilt_min_rad, "tilt_max_rad", c.tilt_max_rad, SERVO_TILT_MIN_RAD, SERVO_TILT_MAX_RAD, errors);
  c.home_pan_rad = read<double>(node, "home_pan_rad", c.home_pan_rad, errors);
  if (pan_ok) {  // an invalid range is reported once, not again through the home pose
    check_within("home_pan_rad", c.home_pan_rad, c.pan_min_rad, c.pan_max_rad, errors);
  }
  c.home_tilt_rad = read<double>(node, "home_tilt_rad", c.home_tilt_rad, errors);
  if (tilt_ok) {
    check_within("home_tilt_rad", c.home_tilt_rad, c.tilt_min_rad, c.tilt_max_rad, errors);
  }
}

void parse_timing(const YAML::Node & node, Config & c, Errors & errors)
{
  c.yield_s = read<double>(node, "yield_s", c.yield_s, errors);
  rc::ge("yield_s", c.yield_s, 0.0, errors);
  c.reaim_pan_deg = read<double>(node, "reaim_pan_deg", c.reaim_pan_deg, errors);
  rc::ge("reaim_pan_deg", c.reaim_pan_deg, 0.0, errors);
  c.reaim_tilt_deg = read<double>(node, "reaim_tilt_deg", c.reaim_tilt_deg, errors);
  rc::ge("reaim_tilt_deg", c.reaim_tilt_deg, 0.0, errors);
  c.min_reaim_interval_s = read<double>(node, "min_reaim_interval_s", c.min_reaim_interval_s, errors);
  rc::ge("min_reaim_interval_s", c.min_reaim_interval_s, 0.0, errors);
  c.rate_hz = read<double>(node, "rate_hz", c.rate_hz, errors);
  require_that(
    c.rate_hz > 0.0 && c.rate_hz <= MAX_RATE_HZ, "rate_hz",
    "must lie in (0, " + std::to_string(MAX_RATE_HZ) + "] (got " + std::to_string(c.rate_hz) + ")", errors);
  c.tf_timeout_s = read<double>(node, "tf_timeout_s", c.tf_timeout_s, errors);
  check_within("tf_timeout_s", c.tf_timeout_s, 0.0, MAX_TF_TIMEOUT_S, errors);
  c.input_timeout_s = read<double>(node, "input_timeout_s", c.input_timeout_s, errors);
  rc::gt("input_timeout_s", c.input_timeout_s, 0.0, errors);
  c.min_travel_speed_mps = read<double>(node, "min_travel_speed_mps", c.min_travel_speed_mps, errors);
  rc::gt("min_travel_speed_mps", c.min_travel_speed_mps, 0.0, errors);
}

Config parse(const YAML::Node & node, Errors & errors)
{
  Config c;
  rc::forbid_unknown(node, KEYS, errors);
  if (!node.IsMap()) {
    return c;
  }
  parse_names(node, c, errors);
  parse_geometry(node, c, errors);
  parse_head(node, c, errors);
  parse_timing(node, c, errors);
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
  return from_yaml(rc::load_config_or_exit(ENV_CONFIG_PATH_KEY, DEFAULT_CONFIG_PATH, validate));
}

}  // namespace head_aim_cpp
