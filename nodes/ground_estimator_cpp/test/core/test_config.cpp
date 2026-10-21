#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

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
  EXPECT_EQ(c.cloud_topic, "/tof/points_filtered");
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
  EXPECT_DOUBLE_EQ(c.fit.smoothing, 0.5);
  EXPECT_DOUBLE_EQ(c.bands.obstacle_max_m, 0.6);
  EXPECT_DOUBLE_EQ(c.bands.drop_max_m, 1.0);
  EXPECT_DOUBLE_EQ(c.bands.max_range_m, 2.0);
  EXPECT_DOUBLE_EQ(c.traversability.wheel_radius_m, 0.06);
  EXPECT_DOUBLE_EQ(c.traversability.max_step_fraction, 0.3333);
  EXPECT_DOUBLE_EQ(c.traversability.max_slope_deg, 8.0);
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
  EXPECT_EQ(c.temporal.window_frames, 8);
  EXPECT_EQ(c.temporal.min_frames, 4);
  EXPECT_DOUBLE_EQ(c.temporal.max_age_s, 1.5);
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

TEST(Config, MaxStepIsOneThirdOfWheelDiameter)
{
  Config c = ground_estimator_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_NEAR(c.max_step_m(), 0.04, 1e-4);
  c = ground_estimator_cpp::from_yaml(YAML::Load("traversability: {wheel_radius_m: 0.09}"));
  EXPECT_NEAR(c.max_step_m(), 0.06, 1e-4);
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
  EXPECT_DOUBLE_EQ(shipped.fit.distance_m, defaults.fit.distance_m);
  EXPECT_DOUBLE_EQ(shipped.traversability.max_step_fraction, defaults.traversability.max_step_fraction);
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
  expect_rejected("bands: {obstacle_max_m: 0.03}", "bands.obstacle_max_m");
  expect_rejected("bands: {drop_max_m: 0.03}", "bands.drop_max_m");
}

TEST(Config, RejectsTraversabilityRules)
{
  expect_rejected("traversability: {wheel_radius_m: 0}", "traversability.wheel_radius_m");
  expect_rejected("traversability: {max_step_fraction: 0}", "traversability.max_step_fraction");
  expect_rejected("traversability: {max_step_fraction: 1}", "traversability.max_step_fraction");
  // max step 0.01 m is not above fit.distance_m 0.02.
  expect_rejected("traversability: {max_step_fraction: 0.0834}", "traversability.max_step_fraction");
  expect_rejected("traversability: {max_slope_deg: 0}", "traversability.max_slope_deg");
  expect_rejected("traversability: {edge_band_m: 0}", "traversability.edge_band_m");
  // The step limit must stay below the band limits.
  expect_rejected("traversability: {wheel_radius_m: 1.0}", "bands.obstacle_max_m");
}

TEST(Config, MaxSlopeAboveTheTiltGuardWarnLimitsIsRejected)
{
  expect_accepted("traversability: {max_slope_deg: 8}");
  expect_rejected("traversability: {max_slope_deg: 8.5}", "traversability.max_slope_deg");
  expect_rejected(
    "traversability: {max_slope_deg: 7}\nslope: {roll_warn_deg: 6, roll_error_deg: 15}",
    "traversability.max_slope_deg");
  expect_rejected(
    "traversability: {max_slope_deg: 8}\nslope: {pitch_warn_deg: 7, pitch_error_deg: 20}",
    "traversability.max_slope_deg");
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
