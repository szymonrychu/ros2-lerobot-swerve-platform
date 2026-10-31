// Configuration of the head aim node: /etc/ros2/head_aim_cpp/config.yaml (HEAD_AIM_CONFIG). Unknown keys are
// rejected and any error exits 78 (ros2_common_cpp::EXIT_CONFIG). config/head_aim.yaml documents every key with its
// meaning and unit.
#ifndef HEAD_AIM_CPP__CONFIG_HPP_
#define HEAD_AIM_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"

namespace head_aim_cpp
{

using ros2_common_cpp::Errors;

constexpr const char * ENV_CONFIG_PATH_KEY = "HEAD_AIM_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/head_aim_cpp/config.yaml";

// Head servo limits (nodes/web_ui/urdf/head.urdf, the head group of the feetech bridge): pan +-90 deg (+ = left),
// tilt -30..+45 deg (+ = down). The configured ranges must stay inside them.
constexpr double SERVO_PAN_MIN_RAD = -1.5708;
constexpr double SERVO_PAN_MAX_RAD = 1.5708;
constexpr double SERVO_TILT_MIN_RAD = -0.5236;
constexpr double SERVO_TILT_MAX_RAD = 0.7854;

// Axis-aligned bounding box of the robot footprint in base_link, metres; it contains the wheel envelopes.
struct FootprintBox
{
  double min_x = -0.215;
  double max_x = 0.3225;
  double min_y = -0.196;
  double max_y = 0.196;
};

// Every setting of the node; the defaults equal config/head_aim.yaml.
struct Config
{
  std::string plan_topic = "/transformed_global_plan";
  std::string cmd_vel_topic = "/cmd_vel_smoothed";
  std::string goal_status_topic = "/navigate_to_pose/_action/status";
  std::string head_command_topic = "/head/joint_commands";
  std::string camera_info_topic = "/head/d435/depth_filtered/camera_info";
  std::string base_frame = "base_link";
  std::string head_frame = "head_link";
  std::string source_tag = "head_aim";
  std::vector<std::string> yield_sources = {"web_ui", "autonomy"};
  double yield_s = 10.0;               // >= 0
  FootprintBox footprint;              // min < max on both axes
  double lookahead_m = 0.4;            // > 0
  double floor_margin_m = 0.05;        // >= 0
  double pan_min_rad = -1.5708;        // servo range, < pan_max_rad
  double pan_max_rad = 1.5708;         // servo range
  double tilt_min_rad = -0.5236;       // servo range, < tilt_max_rad
  double tilt_max_rad = 0.7854;        // servo range
  double home_pan_rad = 0.0;           // within the pan range
  double home_tilt_rad = 0.5236;       // within the tilt range
  double reaim_pan_deg = 10.0;         // >= 0
  double reaim_tilt_deg = 5.0;         // >= 0
  double min_reaim_interval_s = 1.0;   // >= 0
  double rate_hz = 5.0;                // 0 < v <= 50
  double tf_timeout_s = 0.1;           // 0..0.5
  double input_timeout_s = 1.0;        // > 0
  double min_travel_speed_mps = 0.02;  // > 0
};

// Every error of a YAML mapping as "<key>: <problem>" lines; empty when valid.
// @param node the parsed config root.
// @return the errors (Errors), empty when valid.
Errors validate(const YAML::Node & node);

// The Config of a valid YAML mapping (missing keys keep their defaults).
// @param node a config root that validate accepts.
// @return the parsed Config.
Config from_yaml(const YAML::Node & node);

// Load HEAD_AIM_CONFIG (default DEFAULT_CONFIG_PATH); on a missing file or any error log every error to stderr and
// exit 78.
// @return the validated Config.
Config load_config_or_exit();

}  // namespace head_aim_cpp

#endif  // HEAD_AIM_CPP__CONFIG_HPP_
