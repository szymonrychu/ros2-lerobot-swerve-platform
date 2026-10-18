#include "map_guard_cpp/config.hpp"

#include <algorithm>
#include <initializer_list>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace map_guard_cpp
{

namespace
{

std::string number_text(double value)
{
  std::ostringstream out;
  out << value;
  return out.str();
}

}  // namespace

const std::vector<std::string> & all_reasons()
{
  static const std::vector<std::string> reasons = {"mismatch", "lifted", "carried", "rotated"};
  return reasons;
}

const std::vector<std::string> & config_keys()
{
  static const std::vector<std::string> keys = {
    "dry_run",
    "reasons",
    "scan_topic",
    "map_topic",
    "imu_topic",
    "wheel_odom_topic",
    "lidar_odom_topic",
    "cmd_vel_topic",
    "map_frame",
    "reset_topic",
    "events_topic",
    "localised_topic",
    "command_topic",
    "result_topic",
    "pause_service",
    "reset_service",
    "serialize_service",
    "parameters_service",
    "service_timeout_s",
    "agent_state_url",
    "agent_poll_s",
    "agent_timeout_s",
    "metrics_port",
    "tick_hz",
    "max_range_m",
    "hit_radius_cells",
    "occupied_threshold",
    "min_points",
    "agreement_hz",
    "median_window",
    "mismatch_ratio",
    "mismatch_s",
    "healthy_ratio",
    "min_map_cells_known",
    "lift_tilt_deg",
    "lift_s",
    "level_window_s",
    "level_tolerance_deg",
    "level_rebaseline_s",
    "lift_accel_z",
    "lift_accel_s",
    "lift_confirm_s",
    "lift_confirm_tilt_deg",
    "gravity_band",
    "carried_speed_mps",
    "wheel_still_mps",
    "cmd_timeout_s",
    "cmd_eps",
    "carried_s",
    "input_timeout_s",
    "rotate_rate_rad_s",
    "wheel_still_rad_s",
    "rotate_s",
    "settle_s",
    "evaluate_s",
    "cooldown_s",
    "relocalise_timeout_s",
    "backup_dir",
    "backup_prefix",
    "keep_backups"};
  return keys;
}

Config parse_config(const YAML::Node & node, ros2_common_cpp::Errors & errors)
{
  Config c;
  ros2_common_cpp::forbid_unknown(node, config_keys(), errors);
  if (!node.IsMap()) {
    return c;
  }
  const auto lax = ros2_common_cpp::Coercion::LAX;
  c.dry_run = ros2_common_cpp::get_or<bool>(node, "dry_run", c.dry_run, errors, lax);

  // reasons: a list of the literal reason names.
  if (const YAML::Node reasons = node["reasons"]; reasons.IsDefined()) {
    if (!reasons.IsSequence()) {
      errors.push_back("reasons: expected list");
    } else {
      std::vector<std::string> accepted;
      for (const auto & item : reasons) {
        const auto name = ros2_common_cpp::lax_string(item);
        if (!name || std::find(all_reasons().begin(), all_reasons().end(), *name) == all_reasons().end()) {
          errors.push_back(
            "reasons: " + (item.IsScalar() ? "'" + item.Scalar() + "'" : std::string("item")) +
            " is not one of mismatch, lifted, carried, rotated");
        } else {
          accepted.push_back(*name);
        }
      }
      if (accepted.size() == reasons.size()) {
        c.reasons = accepted;
      }
    }
  }

  for (auto [key, field] : std::initializer_list<std::pair<const char *, std::string *>>{
         {"scan_topic", &c.scan_topic},
         {"map_topic", &c.map_topic},
         {"imu_topic", &c.imu_topic},
         {"wheel_odom_topic", &c.wheel_odom_topic},
         {"lidar_odom_topic", &c.lidar_odom_topic},
         {"cmd_vel_topic", &c.cmd_vel_topic},
         {"map_frame", &c.map_frame},
         {"reset_topic", &c.reset_topic},
         {"events_topic", &c.events_topic},
         {"localised_topic", &c.localised_topic},
         {"command_topic", &c.command_topic},
         {"result_topic", &c.result_topic},
         {"pause_service", &c.pause_service},
         {"reset_service", &c.reset_service},
         {"serialize_service", &c.serialize_service},
         {"parameters_service", &c.parameters_service},
         {"agent_state_url", &c.agent_state_url},
         {"backup_dir", &c.backup_dir},
         {"backup_prefix", &c.backup_prefix}}) {
    *field = ros2_common_cpp::get_or<std::string>(node, key, *field, errors, lax);
  }

  // Positive floats (pydantic gt=0; NaN fails every bound).
  for (auto [key, field] : std::initializer_list<std::pair<const char *, double *>>{
         {"service_timeout_s", &c.service_timeout_s},
         {"agent_poll_s", &c.agent_poll_s},
         {"agent_timeout_s", &c.agent_timeout_s},
         {"tick_hz", &c.tick_hz},
         {"max_range_m", &c.max_range_m},
         {"agreement_hz", &c.agreement_hz},
         {"mismatch_s", &c.mismatch_s},
         {"lift_tilt_deg", &c.lift_tilt_deg},
         {"lift_s", &c.lift_s},
         {"level_window_s", &c.level_window_s},
         {"level_tolerance_deg", &c.level_tolerance_deg},
         {"level_rebaseline_s", &c.level_rebaseline_s},
         {"lift_accel_z", &c.lift_accel_z},
         {"lift_accel_s", &c.lift_accel_s},
         {"lift_confirm_s", &c.lift_confirm_s},
         {"lift_confirm_tilt_deg", &c.lift_confirm_tilt_deg},
         {"gravity_band", &c.gravity_band},
         {"carried_speed_mps", &c.carried_speed_mps},
         {"wheel_still_mps", &c.wheel_still_mps},
         {"cmd_timeout_s", &c.cmd_timeout_s},
         {"cmd_eps", &c.cmd_eps},
         {"carried_s", &c.carried_s},
         {"input_timeout_s", &c.input_timeout_s},
         {"rotate_rate_rad_s", &c.rotate_rate_rad_s},
         {"wheel_still_rad_s", &c.wheel_still_rad_s},
         {"rotate_s", &c.rotate_s},
         {"settle_s", &c.settle_s},
         {"evaluate_s", &c.evaluate_s},
         {"cooldown_s", &c.cooldown_s},
         {"relocalise_timeout_s", &c.relocalise_timeout_s}}) {
    *field = ros2_common_cpp::get_or<double>(node, key, *field, errors, lax);
    ros2_common_cpp::gt(key, *field, 0.0, errors);
  }
  for (auto [key, field] : std::initializer_list<std::pair<const char *, double *>>{
         {"mismatch_ratio", &c.mismatch_ratio}, {"healthy_ratio", &c.healthy_ratio}}) {
    *field = ros2_common_cpp::get_or<double>(node, key, *field, errors, lax);
    ros2_common_cpp::ge(key, *field, 0.0, errors);
    ros2_common_cpp::le(key, *field, 1.0, errors);
  }

  if (const YAML::Node port = node["metrics_port"]; port.IsDefined()) {
    if (ros2_common_cpp::is_yaml_null(port)) {
      c.metrics_port.reset();
    } else if (const auto value = ros2_common_cpp::lax_int(port)) {
      c.metrics_port = *value;
    } else {
      errors.push_back("metrics_port: expected int or null");
    }
  }
  // Integers with a lower bound (pydantic ge); occupied_threshold is also capped at 100.
  for (auto [key, field, low] :
       std::initializer_list<std::tuple<const char *, long *, long>>{// NOLINT(runtime/int)
                                                                     {"hit_radius_cells", &c.hit_radius_cells, 0},
                                                                     {"occupied_threshold", &c.occupied_threshold, 1},
                                                                     {"min_points", &c.min_points, 1},
                                                                     {"median_window", &c.median_window, 1},
                                                                     {"min_map_cells_known", &c.min_map_cells_known, 0},
                                                                     {"keep_backups", &c.keep_backups, 1}}) {
    *field = ros2_common_cpp::get_or<long>(node, key, *field, errors, lax);  // NOLINT(runtime/int)
    ros2_common_cpp::ge(key, *field, low, errors);
  }
  ros2_common_cpp::le<long>("occupied_threshold", c.occupied_threshold, 100, errors);  // NOLINT(runtime/int)

  // The model validator runs only when every field is valid, as in pydantic.
  if (errors.empty() && c.healthy_ratio <= c.mismatch_ratio) {
    errors.push_back(
      "healthy_ratio: (" + number_text(c.healthy_ratio) + ") must be above mismatch_ratio (" +
      number_text(c.mismatch_ratio) + ")");
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

}  // namespace map_guard_cpp
