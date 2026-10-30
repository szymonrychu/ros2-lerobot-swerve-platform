#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "ground_estimator_cpp/config.hpp"

namespace
{

using ground_estimator_cpp::Config;

// Errors of a YAML document.
ros2_common_cpp::Errors errors_of(const std::string & text) { return ground_estimator_cpp::validate(YAML::Load(text)); }

// True when any error line starts with key + ":".
bool names_key(const ros2_common_cpp::Errors & errors, const std::string & key)
{
  for (const auto & line : errors) {
    if (line.rfind(key + ":", 0) == 0) {
      return true;
    }
  }
  return false;
}

// Assert the document is rejected with an error line naming key.
void expect_rejected(const std::string & text, const std::string & key)
{
  const auto errors = errors_of(text);
  EXPECT_FALSE(errors.empty()) << text;
  EXPECT_TRUE(names_key(errors, key)) << text << " -> " << (errors.empty() ? "no errors" : errors.front());
}

// Assert the document is accepted.
void expect_accepted(const std::string & text)
{
  const auto errors = errors_of(text);
  EXPECT_TRUE(errors.empty()) << text << " -> " << (errors.empty() ? "" : errors.front());
}

std::string shipped_yaml_path() { return std::string(GROUND_CONFIG_DIR) + "/ground_estimator.yaml"; }

}  // namespace

TEST(Config, DefaultsMatchTheSpecification)
{
  EXPECT_TRUE(errors_of("{}").empty());
  const Config c = ground_estimator_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_EQ(c.cloud_topic, "/head/d435/points_filtered");
  EXPECT_EQ(c.imu_topic, "/imu/data");
  EXPECT_EQ(c.rgbd_cloud_topic, "/head_rgbd/points");
  EXPECT_EQ(c.output_namespace, "/ground");
  EXPECT_EQ(c.base_frame, "base_link");
  EXPECT_EQ(c.fixed_frame, "odom");
  EXPECT_DOUBLE_EQ(c.tf_timeout_s, 0.1);
  EXPECT_DOUBLE_EQ(c.imu.timeout_s, 0.5);
  EXPECT_DOUBLE_EQ(c.imu.level_reference_roll_deg, 0.7);
  EXPECT_DOUBLE_EQ(c.imu.level_reference_pitch_deg, 1.3);
  EXPECT_DOUBLE_EQ(c.fit.distance_m, 0.02);
  EXPECT_DOUBLE_EQ(c.fit.max_normal_angle_deg, 10.0);
  EXPECT_DOUBLE_EQ(c.fit.max_offset_m, 0.08);
  EXPECT_EQ(c.fit.min_inliers, 150);
  EXPECT_DOUBLE_EQ(c.fit.min_inlier_ratio, 0.2);
  EXPECT_EQ(c.fit.max_iterations, 100);
  EXPECT_DOUBLE_EQ(c.fit.candidate_band_m, 0.15);
  EXPECT_DOUBLE_EQ(c.fit.max_range_m, 2.5);
  EXPECT_DOUBLE_EQ(c.fit.hold_s, 1.0);
  EXPECT_DOUBLE_EQ(c.fit.smoothing, 0.2);
  EXPECT_DOUBLE_EQ(c.bands.obstacle_max_m, 0.6);
  EXPECT_DOUBLE_EQ(c.bands.drop_max_m, 1.0);
  EXPECT_DOUBLE_EQ(c.bands.max_range_m, 2.0);
  EXPECT_DOUBLE_EQ(c.bands.min_range_m, 0.42);
  EXPECT_DOUBLE_EQ(c.traversability.wheel_radius_m, 0.06);
  EXPECT_DOUBLE_EQ(c.traversability.max_step_up_fraction, 0.3333);
  EXPECT_DOUBLE_EQ(c.traversability.max_step_down_fraction, 0.3333);
  EXPECT_DOUBLE_EQ(c.traversability.chassis_clearance_m, 0.06);
  EXPECT_DOUBLE_EQ(c.traversability.wheelbase_m, 0.305);
  EXPECT_DOUBLE_EQ(c.traversability.robot_height_m, 0.6);
  EXPECT_DOUBLE_EQ(c.bands.floor_band_m, 0.02);
  EXPECT_DOUBLE_EQ(c.traversability.max_slope_deg, 15.0);
  EXPECT_DOUBLE_EQ(c.traversability.edge_band_m, 0.05);
  EXPECT_EQ(c.warnings.frame, "map");
  EXPECT_DOUBLE_EQ(c.edges.cluster_tolerance_m, 0.1);
  EXPECT_EQ(c.edges.min_cluster_points, 10);
  EXPECT_EQ(c.edges.max_regions, 8);
  EXPECT_DOUBLE_EQ(c.edges.step_max_height_m, 0.2);
  EXPECT_DOUBLE_EQ(c.edges.step_flatness_m, 0.03);
  EXPECT_DOUBLE_EQ(c.slope.roll_warn_deg, 8.0);
  EXPECT_DOUBLE_EQ(c.slope.roll_error_deg, 15.0);
  EXPECT_DOUBLE_EQ(c.slope.pitch_warn_deg, 12.0);
  EXPECT_DOUBLE_EQ(c.slope.pitch_error_deg, 20.0);
  EXPECT_TRUE(c.colour.enabled);
  EXPECT_DOUBLE_EQ(c.colour.max_mahalanobis, 3.0);
  EXPECT_DOUBLE_EQ(c.colour.forgetting, 0.1);
  EXPECT_EQ(c.colour.min_floor_samples, 200);
  EXPECT_DOUBLE_EQ(c.state.rate_hz, 2.0);
  EXPECT_TRUE(c.temporal.enabled);
  EXPECT_DOUBLE_EQ(c.temporal.cell_size_m, 0.10);
  EXPECT_EQ(c.temporal.window_frames, 12);
  EXPECT_EQ(c.temporal.min_frames, 6);
  EXPECT_DOUBLE_EQ(c.temporal.max_age_s, 1.5);
  EXPECT_DOUBLE_EQ(c.temporal.max_spread_m, 0.025);
  EXPECT_TRUE(c.hysteresis.enabled);
  EXPECT_EQ(c.hysteresis.window_frames, 8);
  EXPECT_EQ(c.hysteresis.enter_frames, 5);
  EXPECT_EQ(c.hysteresis.exit_frames, 2);
  EXPECT_EQ(c.hysteresis.area_enter_frames, 5);
  EXPECT_EQ(c.hysteresis.area_min_cells, 3);
  EXPECT_EQ(c.hysteresis.area_join_frames, 12);
  EXPECT_EQ(c.hysteresis.area_kind_frames, 15);
  EXPECT_EQ(c.hysteresis.area_history_frames, 30);
  EXPECT_DOUBLE_EQ(c.hysteresis.area_enter_density, 0.75);
  EXPECT_DOUBLE_EQ(c.hysteresis.area_exit_density, 0.25);
  EXPECT_EQ(c.traversability.lethal_min_points, 2);
  EXPECT_DOUBLE_EQ(c.edges.slope_bin_m, 0.10);
  EXPECT_EQ(c.edges.slope_min_cells, 3);
  EXPECT_TRUE(c.occlusion.enabled);
  EXPECT_DOUBLE_EQ(c.occlusion.azimuth_bin_deg, 4.0);
  EXPECT_DOUBLE_EQ(c.occlusion.elevation_margin_deg, 2.0);
  EXPECT_DOUBLE_EQ(c.occlusion.min_occluder_height_m, 0.03);
  EXPECT_DOUBLE_EQ(c.occlusion.min_height_drop_m, 0.005);
  EXPECT_DOUBLE_EQ(c.edges.step_segment_m, 0.10);
  EXPECT_EQ(c.edges.step_segment_min_points, 3);
  EXPECT_DOUBLE_EQ(c.edges.floor_reference_m, 0.10);
  EXPECT_EQ(c.edges.floor_reference_min_points, 4);
  EXPECT_TRUE(c.motion.enabled);
  EXPECT_DOUBLE_EQ(c.motion.max_head_rate_deg_s, 3.0);
  EXPECT_DOUBLE_EQ(c.motion.window_s, 0.1);
  ASSERT_TRUE(c.metrics_port.has_value());
  EXPECT_EQ(*c.metrics_port, 19119);
}

TEST(Config, DerivedTopicsFollowTheNamespace)
{
  const Config c = ground_estimator_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_EQ(c.obstacle_points(), "/ground/obstacle_points");
  EXPECT_EQ(c.drop_points(), "/ground/drop_points");
  EXPECT_EQ(c.labelled_points(), "/ground/labelled_points");
  EXPECT_EQ(c.state_topic(), "/ground/state");
  EXPECT_EQ(c.warning_areas(), "/ground/warning_areas");
  const Config d = ground_estimator_cpp::from_yaml(YAML::Load("output_namespace: /floor"));
  EXPECT_EQ(d.obstacle_points(), "/floor/obstacle_points");
  EXPECT_EQ(d.warning_areas(), "/floor/warning_areas");
}

TEST(Config, MaxStepIsAThirdOfTheWheelDiameterUpAndDown)
{
  // User decision 2026-10-29: the climbable step is 0.04 m, the same limit up and down (a drop deeper than it is no-go).
  Config c = ground_estimator_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_NEAR(c.max_step_up_m(), 0.04, 1e-4);
  EXPECT_NEAR(c.max_step_down_m(), 0.04, 1e-4);
  c = ground_estimator_cpp::from_yaml(YAML::Load("traversability: {wheel_radius_m: 0.09}"));
  EXPECT_NEAR(c.max_step_up_m(), 0.06, 1e-4);
  EXPECT_NEAR(c.max_step_down_m(), 0.06, 1e-4);
  // Task T3: the step up and the step down are separate keys.
  c = ground_estimator_cpp::from_yaml(YAML::Load("traversability: {max_step_up_fraction: 0.5}"));
  EXPECT_NEAR(c.max_step_up_m(), 0.06, 1e-4);
  EXPECT_NEAR(c.max_step_down_m(), 0.04, 1e-4);
  c = ground_estimator_cpp::from_yaml(YAML::Load("traversability: {max_step_down_fraction: 0.25}"));
  EXPECT_NEAR(c.max_step_up_m(), 0.04, 1e-4);
  EXPECT_NEAR(c.max_step_down_m(), 0.03, 1e-4);
}

TEST(Config, ShippedYamlDocumentsEveryKey)
{
  std::ifstream in(shipped_yaml_path());
  ASSERT_TRUE(in.good()) << shipped_yaml_path();
  const std::regex key_line(R"(^\s*[a-z_]+:)");
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) {
    lines.push_back(line);
  }
  int keys = 0;
  for (size_t i = 0; i < lines.size(); ++i) {
    if (!std::regex_search(lines[i], key_line)) {
      continue;
    }
    ++keys;
    bool commented = lines[i].find('#') != std::string::npos;
    if (!commented && i > 0) {
      const auto first = lines[i - 1].find_first_not_of(" \t");
      commented = first != std::string::npos && lines[i - 1][first] == '#';
    }
    EXPECT_TRUE(commented) << "undocumented key line " << i + 1 << ": " << lines[i];
  }
  EXPECT_GT(keys, 40);
}

TEST(Config, ShippedYamlLoadsWithTheDefaults)
{
  const YAML::Node node = YAML::LoadFile(shipped_yaml_path());
  const auto errors = ground_estimator_cpp::validate(node);
  EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors.front());
  const Config shipped = ground_estimator_cpp::from_yaml(node);
  const Config defaults = ground_estimator_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_EQ(shipped.cloud_topic, defaults.cloud_topic);
  EXPECT_EQ(shipped.guidance.depth_topic, defaults.guidance.depth_topic);
  EXPECT_EQ(shipped.guidance.info_topic, defaults.guidance.info_topic);
  EXPECT_EQ(shipped.guidance.depth_info_topic, defaults.guidance.depth_info_topic);
  EXPECT_EQ(shipped.guidance.no_return.margin_px, defaults.guidance.no_return.margin_px);
  EXPECT_EQ(shipped.temporal.window_frames, defaults.temporal.window_frames);
  EXPECT_EQ(shipped.temporal.min_frames, defaults.temporal.min_frames);
  EXPECT_EQ(shipped.hysteresis.window_frames, defaults.hysteresis.window_frames);
  EXPECT_EQ(shipped.hysteresis.enter_frames, defaults.hysteresis.enter_frames);
  EXPECT_EQ(shipped.hysteresis.exit_frames, defaults.hysteresis.exit_frames);
  EXPECT_EQ(shipped.hysteresis.area_enter_frames, defaults.hysteresis.area_enter_frames);
  EXPECT_EQ(shipped.hysteresis.area_join_frames, defaults.hysteresis.area_join_frames);
  EXPECT_EQ(shipped.hysteresis.area_kind_frames, defaults.hysteresis.area_kind_frames);
  EXPECT_EQ(shipped.hysteresis.area_history_frames, defaults.hysteresis.area_history_frames);
  EXPECT_EQ(shipped.guidance.no_return.window_frames, defaults.guidance.no_return.window_frames);
  EXPECT_EQ(shipped.guidance.no_return.enter_frames, defaults.guidance.no_return.enter_frames);
  EXPECT_DOUBLE_EQ(shipped.fit.distance_m, defaults.fit.distance_m);
  EXPECT_DOUBLE_EQ(shipped.traversability.max_step_up_fraction, defaults.traversability.max_step_up_fraction);
  EXPECT_DOUBLE_EQ(shipped.traversability.max_step_down_fraction, defaults.traversability.max_step_down_fraction);
  EXPECT_DOUBLE_EQ(shipped.traversability.chassis_clearance_m, defaults.traversability.chassis_clearance_m);
  EXPECT_DOUBLE_EQ(shipped.traversability.wheelbase_m, defaults.traversability.wheelbase_m);
  EXPECT_DOUBLE_EQ(shipped.traversability.robot_height_m, defaults.traversability.robot_height_m);
  EXPECT_DOUBLE_EQ(shipped.slope.pitch_error_deg, defaults.slope.pitch_error_deg);
  EXPECT_EQ(shipped.metrics_port, defaults.metrics_port);
}

TEST(Config, RejectsUnknownKeysAtEveryLevel)
{
  expect_rejected("bogus: 1", "bogus");
  expect_rejected("fit: {bogus: 1}", "fit.bogus");
  expect_rejected("imu: {bogus: 1}", "imu.bogus");
  expect_rejected("traversability: {bogus: 1}", "traversability.bogus");
  expect_rejected("colour: {bogus: 1}", "colour.bogus");
  expect_rejected("state: {bogus: 1}", "state.bogus");
  expect_rejected("temporal: {bogus: 1}", "temporal.bogus");
  expect_rejected("occlusion: {bogus: 1}", "occlusion.bogus");
  expect_rejected("edges: {bogus: 1}", "edges.bogus");
}

TEST(Config, RejectsTopLevelRanges)
{
  expect_rejected("tf_timeout_s: -0.1", "tf_timeout_s");
  expect_rejected("tf_timeout_s: 0.6", "tf_timeout_s");
  expect_accepted("tf_timeout_s: 0");
  expect_rejected("metrics_port: 0", "metrics_port");
  expect_rejected("metrics_port: 70000", "metrics_port");
  expect_rejected("metrics_port: abc", "metrics_port");
  expect_accepted("metrics_port: null");
  expect_rejected("cloud_topic: [a]", "cloud_topic");
}

TEST(Config, RejectsImuRanges)
{
  expect_rejected("imu: {timeout_s: 0}", "imu.timeout_s");
  expect_rejected("imu: {level_reference_roll_deg: 31}", "imu.level_reference_roll_deg");
  expect_rejected("imu: {level_reference_pitch_deg: -31}", "imu.level_reference_pitch_deg");
  expect_accepted("imu: {level_reference_pitch_deg: -30}");
}

TEST(Config, RejectsFitRangesAndCrossKeyRules)
{
  expect_rejected("fit: {distance_m: 0}", "fit.distance_m");
  expect_rejected("fit: {distance_m: 0.11}", "fit.distance_m");
  expect_rejected("fit: {max_normal_angle_deg: 0}", "fit.max_normal_angle_deg");
  expect_rejected("fit: {max_normal_angle_deg: 31}", "fit.max_normal_angle_deg");
  expect_rejected("fit: {max_offset_m: 0}", "fit.max_offset_m");
  expect_rejected("fit: {max_offset_m: 0.31}", "fit.max_offset_m");
  expect_rejected("fit: {min_inliers: 9}", "fit.min_inliers");
  expect_rejected("fit: {min_inlier_ratio: 0}", "fit.min_inlier_ratio");
  expect_rejected("fit: {min_inlier_ratio: 1.1}", "fit.min_inlier_ratio");
  expect_rejected("fit: {max_iterations: 9}", "fit.max_iterations");
  expect_rejected("fit: {candidate_band_m: 0.02}", "fit.candidate_band_m");
  expect_rejected("fit: {max_range_m: 0}", "fit.max_range_m");
  expect_rejected("fit: {hold_s: -1}", "fit.hold_s");
  expect_rejected("fit: {smoothing: 0}", "fit.smoothing");
  expect_rejected("fit: {smoothing: 1.5}", "fit.smoothing");
  expect_accepted("fit: {smoothing: 1, hold_s: 0}");
}

TEST(Config, RejectsBandRanges)
{
  expect_rejected("bands: {max_range_m: 0}", "bands.max_range_m");
  expect_accepted("bands: {min_range_m: 0}");
  expect_rejected("bands: {min_range_m: -0.1}", "bands.min_range_m");
  expect_rejected("bands: {min_range_m: 2.0}", "bands.min_range_m");
  expect_rejected("bands: {obstacle_max_m: 0.03}", "bands.obstacle_max_m");
  expect_rejected("bands: {drop_max_m: 0.03}", "bands.drop_max_m");
}

TEST(Config, RejectsFloorBandOutsideTheFitDistanceAndTheMaxStep)
{
  expect_rejected("bands: {floor_band_m: 0}", "bands.floor_band_m");
  expect_rejected("bands: {floor_band_m: 0.01}", "bands.floor_band_m");  // below fit.distance_m 0.02
  expect_rejected("bands: {floor_band_m: 0.05}", "bands.floor_band_m");  // above the 0.04 max step
  expect_accepted("bands: {floor_band_m: 0.02}");
  expect_accepted("bands: {floor_band_m: 0.04}");
  expect_accepted(
    "traversability: {max_step_up_fraction: 0.5, max_step_down_fraction: 0.5}\nbands: {floor_band_m: 0.06}");  // 0.06
}

TEST(Config, RejectsTraversabilityRules)
{
  expect_rejected("traversability: {wheel_radius_m: 0}", "traversability.wheel_radius_m");
  for (const std::string key : {"max_step_up_fraction", "max_step_down_fraction"}) {
    expect_rejected("traversability: {" + key + ": 0}", "traversability." + key);
    expect_rejected("traversability: {" + key + ": 1}", "traversability." + key);
    // max step 0.01 m is not above fit.distance_m 0.02.
    expect_rejected("traversability: {" + key + ": 0.0834}", "traversability." + key);
  }
  expect_rejected("traversability: {max_slope_deg: 0}", "traversability.max_slope_deg");
  expect_rejected("traversability: {edge_band_m: 0}", "traversability.edge_band_m");
  // The step limit must stay below the band limits.
  expect_rejected("traversability: {wheel_radius_m: 1.0}", "bands.obstacle_max_m");
}

// Task T3: the chassis (crossbar underside 0.06 m, wheelbase 0.305 m) and the robot height (head envelope 0.6 m) bound
// the rule: a climbable step must pass under the chassis, the steepest ramp must not high-centre it on its crest (the
// break-over angle 2 atan(2 clearance / wheelbase), 42.9 deg here), and the obstacle band must end at the robot height.
TEST(Config, RejectsChassisAndRobotHeightViolations)
{
  expect_accepted("traversability: {chassis_clearance_m: 0.04}");  // the 0.04 m step just passes under
  expect_rejected("traversability: {chassis_clearance_m: 0.039}", "traversability.chassis_clearance_m");
  expect_rejected("traversability: {chassis_clearance_m: 0}", "traversability.chassis_clearance_m");
  expect_rejected("traversability: {wheelbase_m: 0}", "traversability.wheelbase_m");
  expect_rejected("traversability: {wheelbase_m: 1.0}", "traversability.max_slope_deg");  // break-over 13.7 deg
  expect_accepted("traversability: {wheelbase_m: 0.89}");                                 // break-over 15.4 deg
  expect_rejected("traversability: {robot_height_m: 0}", "traversability.robot_height_m");
  expect_rejected("traversability: {robot_height_m: 0.5}", "bands.obstacle_max_m");
  expect_accepted("traversability: {robot_height_m: 0.8}");
}

TEST(Config, MaxSlopeAboveTheTiltGuardErrorLimitsIsRejected)
{
  expect_accepted("traversability: {max_slope_deg: 15}");
  expect_rejected("traversability: {max_slope_deg: 15.5}", "traversability.max_slope_deg");
  expect_rejected(
    "traversability: {max_slope_deg: 12}\nslope: {roll_error_deg: 11, roll_warn_deg: 8}",
    "traversability.max_slope_deg");
  expect_rejected(
    "traversability: {max_slope_deg: 16}\nslope: {pitch_error_deg: 15.5, pitch_warn_deg: 12}",
    "traversability.max_slope_deg");
}

TEST(Config, RejectsRobustnessKeys)
{
  expect_accepted("temporal: {max_spread_m: 0.2}");
  expect_rejected("temporal: {max_spread_m: 0}", "temporal.max_spread_m");
  expect_rejected("temporal: {max_spread_m: 0.21}", "temporal.max_spread_m");
  expect_accepted("hysteresis: {window_frames: 5, enter_frames: 3}");
  expect_accepted("hysteresis: {enabled: false}");
  expect_accepted("hysteresis: {window_frames: 30, enter_frames: 16, exit_frames: 16}");
  expect_rejected("hysteresis: {window_frames: 6, enter_frames: 3}", "hysteresis.enter_frames");
  expect_rejected("hysteresis: {exit_frames: 6}", "hysteresis.exit_frames");
  expect_rejected("hysteresis: {exit_frames: 0}", "hysteresis.exit_frames");
  expect_rejected("hysteresis: {window_frames: 31}", "hysteresis.window_frames");
  expect_rejected("hysteresis: {window_frames: 2, enter_frames: 3}", "hysteresis.enter_frames");
  expect_rejected("hysteresis: {enabled: maybe}", "hysteresis.enabled");
  expect_rejected("hysteresis: {foo: 1}", "hysteresis.foo");
  expect_accepted("hysteresis: {area_enter_frames: 1}");
  expect_accepted("hysteresis: {area_enter_frames: 30, area_join_frames: 30}");
  expect_rejected("hysteresis: {area_enter_frames: 13}", "hysteresis.area_join_frames");
  expect_rejected("hysteresis: {area_enter_frames: 0}", "hysteresis.area_enter_frames");
  expect_rejected("hysteresis: {area_enter_frames: 31}", "hysteresis.area_enter_frames");
  expect_accepted("hysteresis: {area_min_cells: 1}");
  expect_accepted("hysteresis: {area_min_cells: 100}");
  expect_rejected("hysteresis: {area_min_cells: 0}", "hysteresis.area_min_cells");
  expect_rejected("hysteresis: {area_min_cells: 101}", "hysteresis.area_min_cells");
  expect_accepted("hysteresis: {area_join_frames: 5}");
  expect_accepted("hysteresis: {area_join_frames: 30}");
  expect_rejected("hysteresis: {area_join_frames: 4}", "hysteresis.area_join_frames");
  expect_rejected("hysteresis: {area_join_frames: 31}", "hysteresis.area_join_frames");
  expect_accepted("hysteresis: {area_kind_frames: 1}");
  expect_accepted("hysteresis: {area_kind_frames: 30}");
  expect_rejected("hysteresis: {area_kind_frames: 0}", "hysteresis.area_kind_frames");
  expect_rejected("hysteresis: {area_kind_frames: 31}", "hysteresis.area_kind_frames");
  expect_accepted("hysteresis: {area_history_frames: 1}");
  expect_accepted("hysteresis: {area_history_frames: 120}");
  expect_rejected("hysteresis: {area_history_frames: 0}", "hysteresis.area_history_frames");
  expect_rejected("hysteresis: {area_history_frames: 121}", "hysteresis.area_history_frames");
  expect_accepted("hysteresis: {area_enter_density: 1.0}");
  expect_accepted("hysteresis: {area_enter_density: 0.01, area_exit_density: 0.0}");
  expect_rejected("hysteresis: {area_enter_density: 0}", "hysteresis.area_enter_density");
  expect_rejected("hysteresis: {area_enter_density: 1.01}", "hysteresis.area_enter_density");
  expect_accepted("hysteresis: {area_exit_density: 0.75}");
  expect_rejected("hysteresis: {area_exit_density: -0.01}", "hysteresis.area_exit_density");
  expect_rejected("hysteresis: {area_exit_density: 0.76}", "hysteresis.area_exit_density");
  expect_accepted("traversability: {lethal_min_points: 1}");
  expect_accepted("traversability: {lethal_min_points: 50}");
  expect_rejected("traversability: {lethal_min_points: 0}", "traversability.lethal_min_points");
  expect_rejected("traversability: {lethal_min_points: 51}", "traversability.lethal_min_points");
  expect_accepted("edges: {slope_bin_m: 0.02}");
  expect_accepted("edges: {slope_bin_m: 0.5}");
  expect_rejected("edges: {slope_bin_m: 0.01}", "edges.slope_bin_m");
  expect_rejected("edges: {slope_bin_m: 0.51}", "edges.slope_bin_m");
  expect_accepted("edges: {slope_min_cells: 2}");
  expect_accepted("edges: {slope_min_cells: 50}");
  expect_rejected("edges: {slope_min_cells: 1}", "edges.slope_min_cells");
  expect_rejected("edges: {slope_min_cells: 51}", "edges.slope_min_cells");
}

TEST(Config, RejectsEdgeRules)
{
  expect_rejected("edges: {cluster_tolerance_m: 0}", "edges.cluster_tolerance_m");
  expect_rejected("edges: {min_cluster_points: 2}", "edges.min_cluster_points");
  expect_rejected("edges: {max_regions: 0}", "edges.max_regions");
  expect_rejected("edges: {max_regions: 17}", "edges.max_regions");
  expect_rejected("edges: {step_max_height_m: 0.03}", "edges.step_max_height_m");
  expect_rejected("edges: {step_max_height_m: 0.7}", "edges.step_max_height_m");
  expect_rejected("edges: {step_flatness_m: 0}", "edges.step_flatness_m");
}

TEST(Config, RejectsOcclusionRanges)
{
  expect_accepted("occlusion: {enabled: false}");
  expect_accepted("occlusion: {azimuth_bin_deg: 0.1}");
  expect_accepted("occlusion: {azimuth_bin_deg: 10}");
  expect_rejected("occlusion: {azimuth_bin_deg: 0.09}", "occlusion.azimuth_bin_deg");
  expect_rejected("occlusion: {azimuth_bin_deg: 10.1}", "occlusion.azimuth_bin_deg");
  expect_accepted("occlusion: {elevation_margin_deg: 0}");
  expect_accepted("occlusion: {elevation_margin_deg: 5}");
  expect_rejected("occlusion: {elevation_margin_deg: -0.1}", "occlusion.elevation_margin_deg");
  expect_rejected("occlusion: {elevation_margin_deg: 5.1}", "occlusion.elevation_margin_deg");
  expect_rejected("occlusion: {min_occluder_height_m: 0}", "occlusion.min_occluder_height_m");
  expect_rejected("occlusion: {min_occluder_height_m: -0.01}", "occlusion.min_occluder_height_m");
  expect_accepted("occlusion: {min_occluder_height_m: 0.59}");
  expect_rejected("occlusion: {min_occluder_height_m: 0.6}", "occlusion.min_occluder_height_m");
  expect_rejected("occlusion: {min_occluder_height_m: 0.7}", "occlusion.min_occluder_height_m");
  expect_rejected(
    "bands: {obstacle_max_m: 0.5}\nocclusion: {min_occluder_height_m: 0.5}", "occlusion.min_occluder_height_m");
  expect_rejected("occlusion: {min_height_drop_m: 0}", "occlusion.min_height_drop_m");
  expect_rejected("occlusion: {min_height_drop_m: -0.01}", "occlusion.min_height_drop_m");
  expect_accepted("occlusion: {min_height_drop_m: 0.001}");
}

TEST(Config, RejectsSegmentStepRanges)
{
  expect_accepted("edges: {step_segment_m: 0.02}");
  expect_accepted("edges: {step_segment_m: 0.5}");
  expect_rejected("edges: {step_segment_m: 0.01}", "edges.step_segment_m");
  expect_rejected("edges: {step_segment_m: 0.51}", "edges.step_segment_m");
  expect_accepted("edges: {step_segment_min_points: 1}");
  expect_accepted("edges: {step_segment_min_points: 50}");
  expect_rejected("edges: {step_segment_min_points: 0}", "edges.step_segment_min_points");
  expect_rejected("edges: {step_segment_min_points: 51}", "edges.step_segment_min_points");
  expect_accepted("edges: {floor_reference_m: 0}");
  expect_accepted("edges: {floor_reference_m: 0.5}");
  expect_rejected("edges: {floor_reference_m: -0.01}", "edges.floor_reference_m");
  expect_rejected("edges: {floor_reference_m: 0.51}", "edges.floor_reference_m");
  expect_accepted("edges: {floor_reference_min_points: 1}");
  expect_accepted("edges: {floor_reference_min_points: 50}");
  expect_rejected("edges: {floor_reference_min_points: 0}", "edges.floor_reference_min_points");
  expect_rejected("edges: {floor_reference_min_points: 51}", "edges.floor_reference_min_points");
}

TEST(Config, RejectsSlopeRules)
{
  expect_rejected("slope: {roll_warn_deg: 15}", "slope.roll_error_deg");
  expect_rejected("slope: {pitch_warn_deg: 20}", "slope.pitch_error_deg");
  expect_rejected("slope: {roll_error_deg: 7}", "slope.roll_error_deg");
}

TEST(Config, RejectsColourAndStateRanges)
{
  expect_rejected("colour: {max_mahalanobis: 0}", "colour.max_mahalanobis");
  expect_rejected("colour: {forgetting: 0}", "colour.forgetting");
  expect_rejected("colour: {forgetting: 1.5}", "colour.forgetting");
  expect_rejected("colour: {min_floor_samples: 9}", "colour.min_floor_samples");
  expect_rejected("colour: {enabled: maybe}", "colour.enabled");
  expect_rejected("state: {rate_hz: 0}", "state.rate_hz");
  expect_rejected("state: {rate_hz: 11}", "state.rate_hz");
}

TEST(Config, RejectsTemporalRanges)
{
  expect_accepted("temporal: {enabled: false}");
  expect_accepted("temporal: {cell_size_m: 0.02}");
  expect_accepted("temporal: {cell_size_m: 0.5}");
  expect_rejected("temporal: {cell_size_m: 0.019}", "temporal.cell_size_m");
  expect_rejected("temporal: {cell_size_m: 0.51}", "temporal.cell_size_m");
  expect_accepted("temporal: {window_frames: 1, min_frames: 1}");
  expect_accepted("temporal: {window_frames: 30}");
  expect_rejected("temporal: {window_frames: 0, min_frames: 1}", "temporal.window_frames");
  expect_rejected("temporal: {window_frames: 31}", "temporal.window_frames");
  expect_rejected("temporal: {min_frames: 0}", "temporal.min_frames");
  expect_rejected("temporal: {window_frames: 5, min_frames: 6}", "temporal.min_frames");
  expect_accepted("temporal: {window_frames: 5, min_frames: 5}");
  expect_accepted("temporal: {max_age_s: 5}");
  expect_rejected("temporal: {max_age_s: 0}", "temporal.max_age_s");
  expect_rejected("temporal: {max_age_s: 5.1}", "temporal.max_age_s");
  expect_rejected("temporal: {enabled: maybe}", "temporal.enabled");
}

TEST(Config, GuidanceDefaults)
{
  const Config c = ground_estimator_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_TRUE(c.guidance.enabled);
  EXPECT_EQ(c.guidance.depth_topic, "/head_rgbd/depth/image_raw");
  EXPECT_EQ(c.guidance.info_topic, "/head_rgbd/depth/camera_info");
  EXPECT_EQ(c.guidance.rgb_topic, "/head_rgbd/rgb/image_raw/compressed");
  EXPECT_EQ(c.guidance.depth_info_topic, "/head/d435/depth/camera_info");
  EXPECT_DOUBLE_EQ(c.guidance.max_age_s, 1.25);  // 2.5 periods of the 2 Hz head_rgbd sets
  EXPECT_EQ(c.guidance.sample_step_px, 2);
  EXPECT_DOUBLE_EQ(c.guidance.max_range_m, 2.0);
  EXPECT_TRUE(c.guidance.edge.enabled);
  EXPECT_DOUBLE_EQ(c.guidance.edge.max_shift_m, 0.06);
  EXPECT_DOUBLE_EQ(c.guidance.edge.min_height_m, 0.02);
  EXPECT_DOUBLE_EQ(c.guidance.edge.voxel_m, 0.02);
  EXPECT_TRUE(c.guidance.no_return.enabled);
  EXPECT_DOUBLE_EQ(c.guidance.no_return.min_range_m, 0.3);
  EXPECT_EQ(c.guidance.no_return.cell_min_samples, 6);
  EXPECT_DOUBLE_EQ(c.guidance.no_return.cell_min_share, 0.8);
  EXPECT_EQ(c.guidance.no_return.window_frames, 6);
  EXPECT_EQ(c.guidance.no_return.enter_frames, 5);
  EXPECT_EQ(c.guidance.no_return.min_cells, 2);
  EXPECT_EQ(c.guidance.no_return.clearance_cells, 1);
  EXPECT_EQ(c.guidance.no_return.margin_px, 4);
  expect_accepted("{}");
}

TEST(Config, RejectsGuidanceRanges)
{
  expect_accepted("guidance: {enabled: false}");
  expect_rejected("guidance: {enabled: maybe}", "guidance.enabled");
  expect_accepted("guidance: {max_age_s: 2}");
  expect_accepted("guidance: {max_age_s: 0.01}");
  expect_rejected("guidance: {max_age_s: 0}", "guidance.max_age_s");
  expect_rejected("guidance: {max_age_s: 2.1}", "guidance.max_age_s");
  expect_accepted("guidance: {sample_step_px: 1}");
  expect_accepted("guidance: {sample_step_px: 8}");
  expect_rejected("guidance: {sample_step_px: 0}", "guidance.sample_step_px");
  expect_rejected("guidance: {sample_step_px: 9}", "guidance.sample_step_px");
  expect_accepted("guidance: {max_range_m: 0.31}");
  expect_rejected("guidance: {max_range_m: 0}", "guidance.max_range_m");
  expect_rejected("guidance: {max_range_m: -1}", "guidance.max_range_m");
  expect_accepted("guidance: {depth_topic: /a, info_topic: /b, rgb_topic: /c, depth_info_topic: /d}");
  expect_rejected("guidance: {depth_topic: [1]}", "guidance.depth_topic");

  expect_accepted("guidance: {edge: {max_shift_m: 0.15}}");
  expect_rejected("guidance: {edge: {max_shift_m: 0}}", "guidance.edge.max_shift_m");
  expect_rejected("guidance: {edge: {max_shift_m: 0.16}}", "guidance.edge.max_shift_m");
  expect_accepted("guidance: {edge: {min_height_m: 0.04}}");
  expect_rejected("guidance: {edge: {min_height_m: 0}}", "guidance.edge.min_height_m");
  expect_rejected("guidance: {edge: {min_height_m: 0.061}}", "guidance.edge.min_height_m");
  expect_accepted("guidance: {edge: {voxel_m: 0.005}}");
  expect_accepted("guidance: {edge: {voxel_m: 0.05}}");
  expect_rejected("guidance: {edge: {voxel_m: 0.004}}", "guidance.edge.voxel_m");
  expect_rejected("guidance: {edge: {voxel_m: 0.051}}", "guidance.edge.voxel_m");
  expect_rejected("guidance: {edge: {enabled: maybe}}", "guidance.edge.enabled");

  expect_accepted("guidance: {no_return: {min_range_m: 0}}");
  expect_rejected("guidance: {no_return: {min_range_m: -0.1}}", "guidance.no_return.min_range_m");
  expect_accepted("guidance: {no_return: {cell_min_samples: 1}}");
  expect_rejected("guidance: {no_return: {cell_min_samples: 0}}", "guidance.no_return.cell_min_samples");
  expect_accepted("guidance: {no_return: {cell_min_share: 1.0}}");
  expect_rejected("guidance: {no_return: {cell_min_share: 0}}", "guidance.no_return.cell_min_share");
  expect_rejected("guidance: {no_return: {cell_min_share: 1.01}}", "guidance.no_return.cell_min_share");
  expect_accepted("guidance: {no_return: {window_frames: 1, enter_frames: 1}}");
  expect_accepted("guidance: {no_return: {window_frames: 20, enter_frames: 20}}");
  expect_rejected("guidance: {no_return: {window_frames: 0, enter_frames: 1}}", "guidance.no_return.window_frames");
  expect_rejected("guidance: {no_return: {window_frames: 21}}", "guidance.no_return.window_frames");
  expect_rejected("guidance: {no_return: {enter_frames: 0}}", "guidance.no_return.enter_frames");
  expect_accepted("guidance: {no_return: {min_cells: 1}}");
  expect_rejected("guidance: {no_return: {min_cells: 0}}", "guidance.no_return.min_cells");
  expect_accepted("guidance: {no_return: {clearance_cells: 0}}");
  expect_accepted("guidance: {no_return: {clearance_cells: 3}}");
  expect_rejected("guidance: {no_return: {clearance_cells: -1}}", "guidance.no_return.clearance_cells");
  expect_rejected("guidance: {no_return: {clearance_cells: 4}}", "guidance.no_return.clearance_cells");
  expect_accepted("guidance: {no_return: {margin_px: 0}}");
  expect_accepted("guidance: {no_return: {margin_px: 20}}");
  expect_rejected("guidance: {no_return: {margin_px: -1}}", "guidance.no_return.margin_px");
  expect_rejected("guidance: {no_return: {margin_px: 21}}", "guidance.no_return.margin_px");
  expect_rejected("guidance: {no_return: {enabled: maybe}}", "guidance.no_return.enabled");
}

TEST(Config, RejectsGuidanceCrossKeyViolations)
{
  expect_accepted("bands: {max_range_m: 3}\nguidance: {max_range_m: 3}");
  expect_rejected("guidance: {max_range_m: 2.5}", "guidance.max_range_m");
  expect_rejected("bands: {max_range_m: 1.5}", "guidance.max_range_m");
  expect_accepted(
    "traversability: {wheel_radius_m: 0.09, max_step_up_fraction: 0.5, max_step_down_fraction: 0.5, "
    "chassis_clearance_m: 0.1}\nguidance: {edge: {min_height_m: 0.089}}");
  expect_rejected("traversability: {wheel_radius_m: 0.02}\nfit: {distance_m: 0.01}", "guidance.edge.min_height_m");
  expect_rejected("guidance: {no_return: {enter_frames: 7}}", "guidance.no_return.enter_frames");
  expect_accepted("guidance: {no_return: {window_frames: 5, enter_frames: 5}}");
  expect_rejected("guidance: {no_return: {min_range_m: 2.0}}", "guidance.no_return.min_range_m");
  expect_rejected("guidance: {max_range_m: 0.3}", "guidance.no_return.min_range_m");
  expect_accepted("guidance: {max_range_m: 0.31}");
}

TEST(Config, MotionGateRanges)
{
  expect_accepted("motion: {enabled: false}");
  expect_rejected("motion: {enabled: maybe}", "motion.enabled");
  expect_accepted("motion: {max_head_rate_deg_s: 90}");
  expect_rejected("motion: {max_head_rate_deg_s: 0}", "motion.max_head_rate_deg_s");
  expect_rejected("motion: {max_head_rate_deg_s: 90.1}", "motion.max_head_rate_deg_s");
  expect_accepted("motion: {window_s: 0.02}");
  expect_accepted("motion: {window_s: 0.5}");
  expect_rejected("motion: {window_s: 0.01}", "motion.window_s");
  expect_rejected("motion: {window_s: 0.51}", "motion.window_s");
  expect_rejected("motion: {foo: 1}", "motion.foo");
}

TEST(Config, RejectsUnknownGuidanceKeys)
{
  expect_rejected("guidance: {foo: 1}", "guidance.foo");
  expect_rejected("guidance: {edge: {foo: 1}}", "guidance.edge.foo");
  expect_rejected("guidance: {no_return: {foo: 1}}", "guidance.no_return.foo");
}

TEST(Config, TheToFKeysAreRejectedAsUnknown)
{
  // Renamed for the D435 (design D7), with no backward compatibility: the old names are unknown keys.
  expect_rejected("guidance: {tof_info_topic: /tof/camera_info}", "guidance.tof_info_topic");
  expect_rejected("guidance: {no_return: {tof_margin_px: 4}}", "guidance.no_return.tof_margin_px");
  expect_accepted("guidance: {depth_info_topic: /head/d435/depth/camera_info, no_return: {margin_px: 4}}");
}

TEST(Config, FrameWindowsKeepTheTenHertzSpansAtFifteenHertz)
{
  // Every frame-count window default is its 10 Hz value times 1.5, rounded up (design D7): the same time span at the
  // 15 Hz of the D435 cloud.
  const Config c = ground_estimator_cpp::from_yaml(YAML::Load("{}"));
  const auto at_15_hz = [](int frames_at_10_hz) { return static_cast<int>(std::ceil(frames_at_10_hz * 1.5)); };
  EXPECT_EQ(c.temporal.window_frames, at_15_hz(8));
  EXPECT_EQ(c.temporal.min_frames, at_15_hz(4));
  EXPECT_EQ(c.hysteresis.window_frames, at_15_hz(5));
  EXPECT_EQ(c.hysteresis.enter_frames, at_15_hz(3));
  EXPECT_EQ(c.hysteresis.exit_frames, at_15_hz(1));
  EXPECT_EQ(c.hysteresis.area_enter_frames, at_15_hz(3));
  EXPECT_EQ(c.hysteresis.area_join_frames, at_15_hz(8));
  EXPECT_EQ(c.hysteresis.area_kind_frames, at_15_hz(10));
  EXPECT_EQ(c.hysteresis.area_history_frames, at_15_hz(20));
  EXPECT_EQ(c.guidance.no_return.window_frames, at_15_hz(4));
  EXPECT_EQ(c.guidance.no_return.enter_frames, at_15_hz(3));
}
