// Configuration of the map guard: the C++ counterpart of the pydantic MapGuardConfig of
// nodes/map_guard/map_guard/config.py. Same file (/etc/ros2/map_guard_cpp/config.yaml via MAP_GUARD_CONFIG), same
// fields, defaults and rejections (extra keys forbidden, gt/ge/le, healthy_ratio > mismatch_ratio), and the same
// pydantic lax-mode coercions of YAML scalars (bool from 0/1 and words, numbers from numeric strings, ints from
// integral floats). Any error exits 78 (ros2_common_cpp::EXIT_CONFIG). See nodes/map_guard/README.md for what each
// setting does.
#ifndef MAP_GUARD_CPP__CONFIG_HPP_
#define MAP_GUARD_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"

namespace map_guard_cpp
{

constexpr const char * ENV_CONFIG_PATH_KEY = "MAP_GUARD_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/map_guard/config.yaml";
constexpr int DEFAULT_METRICS_PORT = 19110;

// Triggers that can be acted on (Reason literal of the Python model), in their default order.
const std::vector<std::string> & all_reasons();

// Every setting of the map guard (field names, types and defaults of MapGuardConfig).
struct Config
{
  bool dry_run = true;
  std::vector<std::string> reasons = all_reasons();

  std::string scan_topic = "/scan_filtered";
  std::string map_topic = "/map";
  std::string imu_topic = "/imu/data";
  std::string wheel_odom_topic = "/odom";
  std::string lidar_odom_topic = "/odom_rf2o";
  std::string cmd_vel_topic = "/cmd_vel";
  std::string map_frame = "map";
  std::string reset_topic = "/map_guard/map_reset";
  std::string events_topic = "/robot_events";
  // std_msgs/Bool from map_manager: false while a loaded map is not yet localised (the guard then holds). C++-only.
  std::string localised_topic = "/map_manager/localised";
  // std_msgs/String JSON: the relocalise command goes to command_topic, the answer comes on result_topic.
  std::string command_topic = "/map_manager/command";
  std::string result_topic = "/map_manager/result";

  std::string pause_service = "/slam_toolbox/pause_new_measurements";
  std::string reset_service = "/slam_toolbox/reset";
  std::string serialize_service = "/slam_toolbox/serialize_map";
  std::string parameters_service = "/slam_toolbox/get_parameters";
  double service_timeout_s = 5.0;

  std::string agent_state_url = "http://127.0.0.1:18300/api/state";
  double agent_poll_s = 2.0;
  double agent_timeout_s = 0.5;
  std::optional<long> metrics_port = DEFAULT_METRICS_PORT;  // NOLINT(runtime/int)
  double tick_hz = 10.0;

  double max_range_m = 8.0;
  long hit_radius_cells = 1;     // NOLINT(runtime/int)
  long occupied_threshold = 65;  // NOLINT(runtime/int)
  long min_points = 40;          // NOLINT(runtime/int)
  double agreement_hz = 2.0;
  long median_window = 5;  // NOLINT(runtime/int)
  double mismatch_ratio = 0.3;
  double mismatch_s = 5.0;
  double healthy_ratio = 0.6;
  long min_map_cells_known = 2000;  // NOLINT(runtime/int)

  double lift_tilt_deg = 12.0;
  double lift_s = 1.0;
  double level_window_s = 3.0;
  double level_tolerance_deg = 2.0;
  double level_rebaseline_s = 30.0;
  double lift_accel_z = 1.5;
  double lift_accel_s = 0.4;
  double lift_confirm_s = 2.0;
  double lift_confirm_tilt_deg = 4.0;
  double gravity_band = 2.0;

  double carried_speed_mps = 0.08;
  double wheel_still_mps = 0.02;
  double cmd_timeout_s = 0.5;
  double cmd_eps = 0.01;
  double carried_s = 1.0;
  double input_timeout_s = 0.5;

  double rotate_rate_rad_s = 0.3;
  double wheel_still_rad_s = 0.05;
  double rotate_s = 0.8;

  double settle_s = 3.0;
  double evaluate_s = 4.0;
  double cooldown_s = 30.0;
  // A relocalise request without an answer for this long counts as unrecoverable, s.
  double relocalise_timeout_s = 90.0;
  std::string backup_dir = "/var/lib/ros2/maps";
  std::string backup_prefix = "map_guard_backup";
  long keep_backups = 5;  // NOLINT(runtime/int)
};

// Every field name of Config (the keys allowed in the YAML file).
const std::vector<std::string> & config_keys();

// Read every field of a YAML mapping into a Config (defaults for missing keys) and record every error the pydantic
// model would raise as "<key>: <problem>" lines.
Config parse_config(const YAML::Node & node, ros2_common_cpp::Errors & errors);

// Errors of a YAML mapping (empty when valid): the ros2_common_cpp::Validator of this node.
ros2_common_cpp::Errors validate_config(const YAML::Node & node);

// Load MAP_GUARD_CONFIG (default DEFAULT_CONFIG_PATH); on a missing file or any error, log every error to stderr and
// exit 78.
Config load_config_or_exit();

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__CONFIG_HPP_
