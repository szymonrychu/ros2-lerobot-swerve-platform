#include "ground_estimator_cpp/config.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ground_estimator_cpp
{

namespace
{

namespace rc = ros2_common_cpp;

constexpr long MAX_PORT = 65535;  // NOLINT(runtime/int)
constexpr double MAX_TF_TIMEOUT_S = 0.5;
constexpr double MAX_LEVEL_REFERENCE_DEG = 30.0;
constexpr double MAX_FIT_DISTANCE_M = 0.1;
constexpr double MAX_NORMAL_ANGLE_DEG = 30.0;
constexpr double MAX_OFFSET_M = 0.3;
constexpr int MIN_INLIERS = 10;
constexpr int MIN_ITERATIONS = 10;
constexpr int MIN_CLUSTER_POINTS = 3;
constexpr int MAX_REGIONS = 16;
constexpr double MAX_STATE_RATE_HZ = 10.0;
constexpr int MIN_FLOOR_SAMPLES = 10;
constexpr double MIN_CELL_SIZE_M = 0.02;
constexpr double MAX_CELL_SIZE_M = 0.5;
constexpr int MAX_WINDOW_FRAMES = 30;
constexpr double MAX_AGE_S = 5.0;
constexpr double MAX_SPREAD_M = 0.2;
constexpr int MAX_LETHAL_MIN_POINTS = 50;
constexpr int MIN_SLOPE_MIN_CELLS = 2;
constexpr int MAX_SLOPE_MIN_CELLS = 50;

// The value under node[key] as T (fallback when missing or invalid) with the lax pydantic-style coercions; errors
// name prefix + key.
template <typename T>
T read(const YAML::Node & node, const std::string & prefix, const char * key, const T & fallback, Errors & errors)
{
  const YAML::Node value = node[key];
  if (!value.IsDefined()) {
    return fallback;
  }
  return rc::convert<T>(value, prefix + key, errors, rc::Coercion::LAX).value_or(fallback);
}

// A section mapping: the node under key (undefined when absent), after rejecting unknown keys inside it.
YAML::Node section(const YAML::Node & root, const char * key, const std::vector<std::string> & allowed, Errors & errors)
{
  const YAML::Node node = root[key];
  if (!node.IsDefined()) {
    return node;
  }
  rc::forbid_unknown(node, allowed, errors, key);
  return node.IsMap() ? node : YAML::Node();
}

// Record "<key>: <problem>" when ok is false.
void require_that(bool ok, const std::string & key, const std::string & problem, Errors & errors)
{
  if (!ok) {
    errors.push_back(key + ": " + problem);
  }
}

// (0, bound] check.
void in_open_closed(const std::string & key, double value, double bound, Errors & errors)
{
  rc::gt(key, value, 0.0, errors);
  rc::le(key, value, bound, errors);
}

void parse_imu(const YAML::Node & root, Config & c, Errors & errors)
{
  const YAML::Node s =
    section(root, "imu", {"timeout_s", "level_reference_roll_deg", "level_reference_pitch_deg"}, errors);
  if (!s.IsDefined()) {
    return;
  }
  const std::string p = "imu.";
  c.imu.timeout_s = read<double>(s, p, "timeout_s", c.imu.timeout_s, errors);
  rc::gt(p + "timeout_s", c.imu.timeout_s, 0.0, errors);
  c.imu.level_reference_roll_deg =
    read<double>(s, p, "level_reference_roll_deg", c.imu.level_reference_roll_deg, errors);
  require_that(
    std::abs(c.imu.level_reference_roll_deg) <= MAX_LEVEL_REFERENCE_DEG, p + "level_reference_roll_deg",
    "must be within +-30 (got " + std::to_string(c.imu.level_reference_roll_deg) + ")", errors);
  c.imu.level_reference_pitch_deg =
    read<double>(s, p, "level_reference_pitch_deg", c.imu.level_reference_pitch_deg, errors);
  require_that(
    std::abs(c.imu.level_reference_pitch_deg) <= MAX_LEVEL_REFERENCE_DEG, p + "level_reference_pitch_deg",
    "must be within +-30 (got " + std::to_string(c.imu.level_reference_pitch_deg) + ")", errors);
}

void parse_fit(const YAML::Node & root, Config & c, Errors & errors)
{
  const YAML::Node s = section(
    root, "fit",
    {"distance_m", "max_normal_angle_deg", "max_offset_m", "min_inliers", "min_inlier_ratio", "max_iterations",
     "candidate_band_m", "max_range_m", "hold_s", "smoothing"},
    errors);
  if (s.IsDefined()) {
    const std::string p = "fit.";
    FitConfig & f = c.fit;
    f.distance_m = read<double>(s, p, "distance_m", f.distance_m, errors);
    in_open_closed(p + "distance_m", f.distance_m, MAX_FIT_DISTANCE_M, errors);
    f.max_normal_angle_deg = read<double>(s, p, "max_normal_angle_deg", f.max_normal_angle_deg, errors);
    in_open_closed(p + "max_normal_angle_deg", f.max_normal_angle_deg, MAX_NORMAL_ANGLE_DEG, errors);
    f.max_offset_m = read<double>(s, p, "max_offset_m", f.max_offset_m, errors);
    in_open_closed(p + "max_offset_m", f.max_offset_m, MAX_OFFSET_M, errors);
    f.min_inliers = read<int>(s, p, "min_inliers", f.min_inliers, errors);
    rc::ge<int>(p + "min_inliers", f.min_inliers, MIN_INLIERS, errors);
    f.min_inlier_ratio = read<double>(s, p, "min_inlier_ratio", f.min_inlier_ratio, errors);
    in_open_closed(p + "min_inlier_ratio", f.min_inlier_ratio, 1.0, errors);
    f.max_iterations = read<int>(s, p, "max_iterations", f.max_iterations, errors);
    rc::ge<int>(p + "max_iterations", f.max_iterations, MIN_ITERATIONS, errors);
    f.candidate_band_m = read<double>(s, p, "candidate_band_m", f.candidate_band_m, errors);
    f.max_range_m = read<double>(s, p, "max_range_m", f.max_range_m, errors);
    rc::gt(p + "max_range_m", f.max_range_m, 0.0, errors);
    f.hold_s = read<double>(s, p, "hold_s", f.hold_s, errors);
    rc::ge(p + "hold_s", f.hold_s, 0.0, errors);
    f.smoothing = read<double>(s, p, "smoothing", f.smoothing, errors);
    in_open_closed(p + "smoothing", f.smoothing, 1.0, errors);
  }
  require_that(
    c.fit.candidate_band_m > c.fit.distance_m, "fit.candidate_band_m",
    "must be above fit.distance_m (" + std::to_string(c.fit.distance_m) + ", got " +
      std::to_string(c.fit.candidate_band_m) + ")",
    errors);
}

void parse_bands(const YAML::Node & root, Config & c, Errors & errors)
{
  const YAML::Node s = section(root, "bands", {"obstacle_max_m", "drop_max_m", "max_range_m"}, errors);
  if (!s.IsDefined()) {
    return;
  }
  const std::string p = "bands.";
  c.bands.obstacle_max_m = read<double>(s, p, "obstacle_max_m", c.bands.obstacle_max_m, errors);
  c.bands.drop_max_m = read<double>(s, p, "drop_max_m", c.bands.drop_max_m, errors);
  c.bands.max_range_m = read<double>(s, p, "max_range_m", c.bands.max_range_m, errors);
  rc::gt(p + "max_range_m", c.bands.max_range_m, 0.0, errors);
}

void parse_traversability(const YAML::Node & root, Config & c, Errors & errors)
{
  const std::vector<std::string> keys = {
    "wheel_radius_m", "max_step_fraction", "max_slope_deg", "edge_band_m", "lethal_min_points"};
  const YAML::Node s = section(root, "traversability", keys, errors);
  if (!s.IsDefined()) {
    return;
  }
  const std::string p = "traversability.";
  TraversabilityConfig & t = c.traversability;
  t.wheel_radius_m = read<double>(s, p, "wheel_radius_m", t.wheel_radius_m, errors);
  rc::gt(p + "wheel_radius_m", t.wheel_radius_m, 0.0, errors);
  t.max_step_fraction = read<double>(s, p, "max_step_fraction", t.max_step_fraction, errors);
  rc::gt(p + "max_step_fraction", t.max_step_fraction, 0.0, errors);
  require_that(
    t.max_step_fraction < 1.0, p + "max_step_fraction", "must be < 1 (got " + std::to_string(t.max_step_fraction) + ")",
    errors);
  t.max_slope_deg = read<double>(s, p, "max_slope_deg", t.max_slope_deg, errors);
  rc::gt(p + "max_slope_deg", t.max_slope_deg, 0.0, errors);
  t.edge_band_m = read<double>(s, p, "edge_band_m", t.edge_band_m, errors);
  rc::gt(p + "edge_band_m", t.edge_band_m, 0.0, errors);
  t.lethal_min_points = read<int>(s, p, "lethal_min_points", t.lethal_min_points, errors);
  rc::ge<int>(p + "lethal_min_points", t.lethal_min_points, 1, errors);
  rc::le<int>(p + "lethal_min_points", t.lethal_min_points, MAX_LETHAL_MIN_POINTS, errors);
}

void parse_edges(const YAML::Node & root, Config & c, Errors & errors)
{
  const YAML::Node s = section(
    root, "edges",
    {"cluster_tolerance_m", "min_cluster_points", "max_regions", "step_max_height_m", "step_flatness_m", "slope_bin_m",
     "slope_min_cells"},
    errors);
  if (!s.IsDefined()) {
    return;
  }
  const std::string p = "edges.";
  EdgesConfig & e = c.edges;
  e.cluster_tolerance_m = read<double>(s, p, "cluster_tolerance_m", e.cluster_tolerance_m, errors);
  rc::gt(p + "cluster_tolerance_m", e.cluster_tolerance_m, 0.0, errors);
  e.min_cluster_points = read<int>(s, p, "min_cluster_points", e.min_cluster_points, errors);
  rc::ge<int>(p + "min_cluster_points", e.min_cluster_points, MIN_CLUSTER_POINTS, errors);
  e.max_regions = read<int>(s, p, "max_regions", e.max_regions, errors);
  rc::ge<int>(p + "max_regions", e.max_regions, 1, errors);
  rc::le<int>(p + "max_regions", e.max_regions, MAX_REGIONS, errors);
  e.step_max_height_m = read<double>(s, p, "step_max_height_m", e.step_max_height_m, errors);
  e.step_flatness_m = read<double>(s, p, "step_flatness_m", e.step_flatness_m, errors);
  rc::gt(p + "step_flatness_m", e.step_flatness_m, 0.0, errors);
  e.slope_bin_m = read<double>(s, p, "slope_bin_m", e.slope_bin_m, errors);
  rc::ge(p + "slope_bin_m", e.slope_bin_m, MIN_CELL_SIZE_M, errors);
  rc::le(p + "slope_bin_m", e.slope_bin_m, MAX_CELL_SIZE_M, errors);
  e.slope_min_cells = read<int>(s, p, "slope_min_cells", e.slope_min_cells, errors);
  rc::ge<int>(p + "slope_min_cells", e.slope_min_cells, MIN_SLOPE_MIN_CELLS, errors);
  rc::le<int>(p + "slope_min_cells", e.slope_min_cells, MAX_SLOPE_MIN_CELLS, errors);
}

void parse_slope(const YAML::Node & root, Config & c, Errors & errors)
{
  const YAML::Node s =
    section(root, "slope", {"roll_warn_deg", "roll_error_deg", "pitch_warn_deg", "pitch_error_deg"}, errors);
  if (s.IsDefined()) {
    const std::string p = "slope.";
    SlopeConfig & l = c.slope;
    l.roll_warn_deg = read<double>(s, p, "roll_warn_deg", l.roll_warn_deg, errors);
    rc::gt(p + "roll_warn_deg", l.roll_warn_deg, 0.0, errors);
    l.roll_error_deg = read<double>(s, p, "roll_error_deg", l.roll_error_deg, errors);
    l.pitch_warn_deg = read<double>(s, p, "pitch_warn_deg", l.pitch_warn_deg, errors);
    rc::gt(p + "pitch_warn_deg", l.pitch_warn_deg, 0.0, errors);
    l.pitch_error_deg = read<double>(s, p, "pitch_error_deg", l.pitch_error_deg, errors);
  }
  require_that(
    c.slope.roll_error_deg > c.slope.roll_warn_deg, "slope.roll_error_deg",
    "must be above slope.roll_warn_deg (" + std::to_string(c.slope.roll_warn_deg) + ", got " +
      std::to_string(c.slope.roll_error_deg) + ")",
    errors);
  require_that(
    c.slope.pitch_error_deg > c.slope.pitch_warn_deg, "slope.pitch_error_deg",
    "must be above slope.pitch_warn_deg (" + std::to_string(c.slope.pitch_warn_deg) + ", got " +
      std::to_string(c.slope.pitch_error_deg) + ")",
    errors);
}

void parse_colour_and_state(const YAML::Node & root, Config & c, Errors & errors)
{
  if (const YAML::Node s =
        section(root, "colour", {"enabled", "max_mahalanobis", "forgetting", "min_floor_samples"}, errors);
      s.IsDefined()) {
    const std::string p = "colour.";
    c.colour.enabled = read<bool>(s, p, "enabled", c.colour.enabled, errors);
    c.colour.max_mahalanobis = read<double>(s, p, "max_mahalanobis", c.colour.max_mahalanobis, errors);
    rc::gt(p + "max_mahalanobis", c.colour.max_mahalanobis, 0.0, errors);
    c.colour.forgetting = read<double>(s, p, "forgetting", c.colour.forgetting, errors);
    in_open_closed(p + "forgetting", c.colour.forgetting, 1.0, errors);
    c.colour.min_floor_samples = read<int>(s, p, "min_floor_samples", c.colour.min_floor_samples, errors);
    rc::ge<int>(p + "min_floor_samples", c.colour.min_floor_samples, MIN_FLOOR_SAMPLES, errors);
  }
  if (const YAML::Node s = section(root, "state", {"rate_hz"}, errors); s.IsDefined()) {
    c.state.rate_hz = read<double>(s, "state.", "rate_hz", c.state.rate_hz, errors);
    in_open_closed("state.rate_hz", c.state.rate_hz, MAX_STATE_RATE_HZ, errors);
  }
}

void parse_temporal(const YAML::Node & root, Config & c, Errors & errors)
{
  const YAML::Node s = section(
    root, "temporal", {"enabled", "cell_size_m", "window_frames", "min_frames", "max_age_s", "max_spread_m"}, errors);
  if (!s.IsDefined()) {
    return;
  }
  const std::string p = "temporal.";
  TemporalConfig & t = c.temporal;
  t.enabled = read<bool>(s, p, "enabled", t.enabled, errors);
  t.cell_size_m = read<double>(s, p, "cell_size_m", t.cell_size_m, errors);
  rc::ge(p + "cell_size_m", t.cell_size_m, MIN_CELL_SIZE_M, errors);
  rc::le(p + "cell_size_m", t.cell_size_m, MAX_CELL_SIZE_M, errors);
  t.window_frames = read<int>(s, p, "window_frames", t.window_frames, errors);
  rc::ge<int>(p + "window_frames", t.window_frames, 1, errors);
  rc::le<int>(p + "window_frames", t.window_frames, MAX_WINDOW_FRAMES, errors);
  t.min_frames = read<int>(s, p, "min_frames", t.min_frames, errors);
  rc::ge<int>(p + "min_frames", t.min_frames, 1, errors);
  t.max_age_s = read<double>(s, p, "max_age_s", t.max_age_s, errors);
  in_open_closed(p + "max_age_s", t.max_age_s, MAX_AGE_S, errors);
  t.max_spread_m = read<double>(s, p, "max_spread_m", t.max_spread_m, errors);
  in_open_closed(p + "max_spread_m", t.max_spread_m, MAX_SPREAD_M, errors);
}

void parse_hysteresis(const YAML::Node & root, Config & c, Errors & errors)
{
  const YAML::Node s = section(root, "hysteresis", {"enabled", "window_frames", "enter_frames", "exit_frames"}, errors);
  if (s.IsDefined()) {
    const std::string p = "hysteresis.";
    HysteresisConfig & h = c.hysteresis;
    h.enabled = read<bool>(s, p, "enabled", h.enabled, errors);
    h.window_frames = read<int>(s, p, "window_frames", h.window_frames, errors);
    rc::ge<int>(p + "window_frames", h.window_frames, 1, errors);
    rc::le<int>(p + "window_frames", h.window_frames, MAX_WINDOW_FRAMES, errors);
    h.enter_frames = read<int>(s, p, "enter_frames", h.enter_frames, errors);
    h.exit_frames = read<int>(s, p, "exit_frames", h.exit_frames, errors);
    rc::ge<int>(p + "exit_frames", h.exit_frames, 1, errors);
  }
  const HysteresisConfig & h = c.hysteresis;
  require_that(
    h.enter_frames >= h.exit_frames, "hysteresis.exit_frames",
    "must be at most hysteresis.enter_frames (" + std::to_string(h.enter_frames) + ", got " +
      std::to_string(h.exit_frames) + ")",
    errors);
  require_that(
    h.enter_frames <= h.window_frames, "hysteresis.enter_frames",
    "must be at most hysteresis.window_frames (" + std::to_string(h.window_frames) + ", got " +
      std::to_string(h.enter_frames) + ")",
    errors);
  require_that(
    h.enter_frames * 2 > h.window_frames, "hysteresis.enter_frames",
    "must be a strict majority of hysteresis.window_frames (" + std::to_string(h.window_frames) + ", got " +
      std::to_string(h.enter_frames) + ")",
    errors);
}

// Rules that span sections, checked on the parsed values (defaults included).
void check_cross_rules(const Config & c, Errors & errors)
{
  require_that(
    c.temporal.min_frames <= c.temporal.window_frames, "temporal.min_frames",
    "must be at most temporal.window_frames (" + std::to_string(c.temporal.window_frames) + ", got " +
      std::to_string(c.temporal.min_frames) + ")",
    errors);
  const double step = c.max_step_m();
  require_that(
    step > c.fit.distance_m, "traversability.max_step_fraction",
    "max step 2 * wheel_radius_m * max_step_fraction (" + std::to_string(step) + ") must exceed fit.distance_m (" +
      std::to_string(c.fit.distance_m) + ")",
    errors);
  require_that(
    c.bands.obstacle_max_m > step, "bands.obstacle_max_m",
    "must be above the max step (" + std::to_string(step) + ", got " + std::to_string(c.bands.obstacle_max_m) + ")",
    errors);
  require_that(
    c.bands.drop_max_m > step, "bands.drop_max_m",
    "must be above the max step (" + std::to_string(step) + ", got " + std::to_string(c.bands.drop_max_m) + ")",
    errors);
  require_that(
    c.edges.step_max_height_m > step && c.edges.step_max_height_m <= c.bands.obstacle_max_m, "edges.step_max_height_m",
    "must be above the max step (" + std::to_string(step) + ") and at most bands.obstacle_max_m (" +
      std::to_string(c.bands.obstacle_max_m) + "), got " + std::to_string(c.edges.step_max_height_m),
    errors);
  const double slope_limit = std::min(c.slope.roll_error_deg, c.slope.pitch_error_deg);
  require_that(
    c.traversability.max_slope_deg <= slope_limit, "traversability.max_slope_deg",
    "must be <= the tilt_guard error limits slope.roll_error_deg / slope.pitch_error_deg (" +
      std::to_string(slope_limit) + ", got " + std::to_string(c.traversability.max_slope_deg) + ")",
    errors);
}

Config parse(const YAML::Node & node, Errors & errors)
{
  Config c;
  rc::forbid_unknown(
    node,
    {"cloud_topic", "imu_topic", "rgbd_cloud_topic", "output_namespace", "base_frame", "fixed_frame", "tf_timeout_s",
     "imu", "fit", "bands", "traversability", "warnings", "edges", "slope", "colour", "state", "temporal", "hysteresis",
     "metrics_port"},
    errors);
  if (!node.IsMap()) {
    return c;
  }
  const std::string none;
  c.cloud_topic = read<std::string>(node, none, "cloud_topic", c.cloud_topic, errors);
  c.imu_topic = read<std::string>(node, none, "imu_topic", c.imu_topic, errors);
  c.rgbd_cloud_topic = read<std::string>(node, none, "rgbd_cloud_topic", c.rgbd_cloud_topic, errors);
  c.output_namespace = read<std::string>(node, none, "output_namespace", c.output_namespace, errors);
  c.base_frame = read<std::string>(node, none, "base_frame", c.base_frame, errors);
  c.fixed_frame = read<std::string>(node, none, "fixed_frame", c.fixed_frame, errors);
  c.tf_timeout_s = read<double>(node, none, "tf_timeout_s", c.tf_timeout_s, errors);
  rc::ge("tf_timeout_s", c.tf_timeout_s, 0.0, errors);
  rc::le("tf_timeout_s", c.tf_timeout_s, MAX_TF_TIMEOUT_S, errors);

  parse_imu(node, c, errors);
  parse_fit(node, c, errors);
  parse_bands(node, c, errors);
  parse_traversability(node, c, errors);
  if (const YAML::Node s = section(node, "warnings", {"frame"}, errors); s.IsDefined()) {
    c.warnings.frame = read<std::string>(s, "warnings.", "frame", c.warnings.frame, errors);
  }
  parse_edges(node, c, errors);
  parse_slope(node, c, errors);
  parse_colour_and_state(node, c, errors);
  parse_temporal(node, c, errors);
  parse_hysteresis(node, c, errors);
  check_cross_rules(c, errors);

  if (const YAML::Node port = node["metrics_port"]; port.IsDefined()) {
    if (rc::is_yaml_null(port)) {
      c.metrics_port.reset();
    } else if (const auto value = rc::lax_int(port)) {
      c.metrics_port = *value;
      rc::ge<long>("metrics_port", *value, 1, errors);         // NOLINT(runtime/int)
      rc::le<long>("metrics_port", *value, MAX_PORT, errors);  // NOLINT(runtime/int)
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
  return from_yaml(rc::load_config_or_exit(ENV_CONFIG_PATH_KEY, DEFAULT_CONFIG_PATH, validate));
}

}  // namespace ground_estimator_cpp
