// Configuration of the ground estimator: /etc/ros2/ground_estimator_cpp/config.yaml (GROUND_ESTIMATOR_CONFIG). Extra
// keys are forbidden at every level and any error exits 78 (ros2_common_cpp::EXIT_CONFIG). config/ground_estimator.yaml
// documents every key with its meaning and unit.
#ifndef GROUND_ESTIMATOR_CPP__CONFIG_HPP_
#define GROUND_ESTIMATOR_CPP__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>

#include "ros2_common_cpp/config.hpp"

namespace ground_estimator_cpp
{

using ros2_common_cpp::Errors;

constexpr const char * ENV_CONFIG_PATH_KEY = "GROUND_ESTIMATOR_CONFIG";
constexpr const char * DEFAULT_CONFIG_PATH = "/etc/ros2/ground_estimator_cpp/config.yaml";
constexpr long DEFAULT_METRICS_PORT = 19119;  // NOLINT(runtime/int)

// IMU prior of the floor normal.
struct ImuConfig
{
  double timeout_s = 0.5;                  // > 0; older IMU samples are not used
  double level_reference_roll_deg = 0.7;   // |v| <= 30; IMU reading on level ground (roll)
  double level_reference_pitch_deg = 1.3;  // |v| <= 30; IMU reading on level ground (pitch)
};

// RANSAC plane fit.
struct FitConfig
{
  double distance_m = 0.02;            // 0 < v <= 0.1
  double max_normal_angle_deg = 10.0;  // 0 < v <= 30
  double max_offset_m = 0.08;          // 0 < v <= 0.3
  int min_inliers = 150;               // >= 10
  double min_inlier_ratio = 0.2;       // 0 < v <= 1
  int max_iterations = 100;            // >= 10
  double candidate_band_m = 0.15;      // > distance_m
  double max_range_m = 2.5;            // > 0
  double hold_s = 1.0;                 // >= 0
  double smoothing = 0.5;              // 0 < v <= 1, 1 = none
};

// Plane-relative height bands.
struct BandsConfig
{
  double obstacle_max_m = 0.6;  // > max_step_m
  double drop_max_m = 1.0;      // > max_step_m
  double max_range_m = 2.0;     // > 0
};

// Obstacle versus warning rule.
struct TraversabilityConfig
{
  double wheel_radius_m = 0.06;       // > 0
  double max_step_fraction = 0.3333;  // 0 < v < 1 of the wheel diameter
  double max_slope_deg = 8.0;         // 0 < v <= min(slope.roll_error_deg, slope.pitch_error_deg)
  double edge_band_m = 0.05;          // > 0
  int lethal_min_points = 2;          // 1..50; fewest unclustered lethal points within edges.cluster_tolerance_m
};

// Largest rideable step: max_step_fraction of the wheel diameter. The single definition of the threshold.
// @param cfg the rule settings.
// @return metres (0.04 for the 0.06 m wheel radius).
inline double max_step_m(const TraversabilityConfig & cfg) { return 2.0 * cfg.wheel_radius_m * cfg.max_step_fraction; }

// Published warning areas.
struct WarningsConfig
{
  std::string frame = "map";
};

// Step, threshold, drop and ramp regions.
struct EdgesConfig
{
  double cluster_tolerance_m = 0.1;  // > 0
  int min_cluster_points = 10;       // >= 3
  int max_regions = 8;               // 1..16
  double step_max_height_m = 0.2;    // > max_step_m, <= bands.obstacle_max_m
  double step_flatness_m = 0.03;     // > 0
  double slope_bin_m = 0.10;         // 0.02..0.5; xy bin edge a cluster is sampled in for its slope fit
  int slope_min_cells = 3;           // 2..50; fewest slope bins for a slope estimate, below it the slope is unknown
};

// Tilt limits of the tilt_guard.
struct SlopeConfig
{
  double roll_warn_deg = 8.0;
  double roll_error_deg = 15.0;
  double pitch_warn_deg = 12.0;
  double pitch_error_deg = 20.0;
};

// Floor colour model.
struct ColourConfig
{
  bool enabled = true;
  double max_mahalanobis = 3.0;  // > 0
  double forgetting = 0.1;       // 0 < v <= 1
  int min_floor_samples = 200;   // >= 10
};

// State topic.
struct StateConfig
{
  double rate_hz = 2.0;  // 0 < v <= 10
};

// Per-cell temporal median of the plane-relative heights, taken in the fixed frame before segmentation.
struct TemporalConfig
{
  bool enabled = true;
  double cell_size_m = 0.10;    // > 0; edge of a fixed-frame cell
  int window_frames = 8;        // >= 1; per-frame values a cell keeps
  int min_frames = 4;           // 1..window_frames; values a cell needs before its points are output
  double max_age_s = 1.5;       // > 0; older per-frame values are dropped
  double max_spread_m = 0.025;  // 0 < v <= 0.2; largest robust spread of a cell's per-frame medians, above it unknown
};

// Per-cell class hysteresis: a class needs a strict majority of the window to enter and few frames to leave.
struct HysteresisConfig
{
  bool enabled = true;
  int window_frames = 5;  // 1..30
  int enter_frames = 3;   // exit_frames..window_frames, enter_frames * 2 > window_frames
  int exit_frames = 1;    // 1..enter_frames
};

// Every setting of the node.
struct Config
{
  std::string cloud_topic = "/tof/points_filtered";
  std::string imu_topic = "/imu/data";
  std::string rgbd_cloud_topic = "/head_rgbd/points";
  std::string output_namespace = "/ground";
  std::string base_frame = "base_link";
  std::string fixed_frame = "odom";
  double tf_timeout_s = 0.1;  // 0..0.5
  ImuConfig imu;
  FitConfig fit;
  BandsConfig bands;
  TraversabilityConfig traversability;
  WarningsConfig warnings;
  EdgesConfig edges;
  SlopeConfig slope;
  ColourConfig colour;
  StateConfig state;
  TemporalConfig temporal;
  HysteresisConfig hysteresis;
  std::optional<long> metrics_port = DEFAULT_METRICS_PORT;  // NOLINT(runtime/int); nullopt disables metrics

  // Largest step the base rides over: a third of the wheel diameter.
  double max_step_m() const { return ground_estimator_cpp::max_step_m(traversability); }

  // Output topics derived from output_namespace.
  std::string obstacle_points() const { return output_namespace + "/obstacle_points"; }
  std::string drop_points() const { return output_namespace + "/drop_points"; }
  std::string labelled_points() const { return output_namespace + "/labelled_points"; }
  std::string state_topic() const { return output_namespace + "/state"; }
  std::string warning_areas() const { return output_namespace + "/warning_areas"; }
};

// Every error of a YAML mapping as "<key>: <problem>" lines (nested keys as "fit.distance_m"); empty when valid.
// @param node the parsed config root.
// @return the error lines.
Errors validate(const YAML::Node & node);

// The Config of a YAML mapping, defaults for missing keys. Invalid values keep their defaults; call validate first.
// @param node the parsed config root.
// @return the config.
Config from_yaml(const YAML::Node & node);

// Load GROUND_ESTIMATOR_CONFIG (default DEFAULT_CONFIG_PATH); on a missing file or any error log every error to stderr
// and exit 78.
// @return the validated config.
Config load_config_or_exit();

}  // namespace ground_estimator_cpp

#endif  // GROUND_ESTIMATOR_CPP__CONFIG_HPP_
