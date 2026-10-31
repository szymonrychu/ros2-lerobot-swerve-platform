#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#include "head_aim_cpp/config.hpp"

namespace
{

using head_aim_cpp::Config;

std::string shipped_yaml_path() { return std::string(HEAD_AIM_CONFIG_DIR) + "/head_aim.yaml"; }

// Errors of a YAML document.
ros2_common_cpp::Errors errors_of(const std::string & text) { return head_aim_cpp::validate(YAML::Load(text)); }

// Number of error lines that start with key + ":".
int count_key(const ros2_common_cpp::Errors & errors, const std::string & key)
{
  int n = 0;
  for (const auto & line : errors) {
    if (line.rfind(key + ":", 0) == 0) {
      ++n;
    }
  }
  return n;
}

// Assert the document gives exactly one error, and that it names key.
void expect_one_error(const std::string & text, const std::string & key)
{
  const auto errors = errors_of(text);
  ASSERT_EQ(errors.size(), 1u) << text << " -> " << (errors.empty() ? "no errors" : errors.front());
  EXPECT_EQ(count_key(errors, key), 1) << text << " -> " << errors.front();
}

}  // namespace

TEST(Config, ShippedYamlLoadsWithoutErrors)
{
  const ros2_common_cpp::LoadResult result = ros2_common_cpp::load_config(shipped_yaml_path(), head_aim_cpp::validate);
  EXPECT_TRUE(result.errors.empty()) << (result.errors.empty() ? "" : result.errors.front());
  const Config c = head_aim_cpp::from_yaml(result.node);
  EXPECT_EQ(c.plan_topic, "/transformed_global_plan");
  EXPECT_EQ(c.cmd_vel_topic, "/cmd_vel_smoothed");
  EXPECT_EQ(c.goal_status_topic, "/navigate_to_pose/_action/status");
  EXPECT_EQ(c.head_command_topic, "/head/joint_commands");
  EXPECT_EQ(c.camera_info_topic, "/head/d435/depth_filtered/camera_info");
  EXPECT_EQ(c.base_frame, "base_link");
  EXPECT_EQ(c.head_frame, "head_link");
  EXPECT_EQ(c.source_tag, "head_aim");
  EXPECT_EQ(c.yield_sources, (std::vector<std::string>{"web_ui", "autonomy"}));
  EXPECT_DOUBLE_EQ(c.yield_s, 10.0);
  EXPECT_DOUBLE_EQ(c.footprint.min_x, -0.215);
  EXPECT_DOUBLE_EQ(c.footprint.max_x, 0.3225);
  EXPECT_DOUBLE_EQ(c.footprint.min_y, -0.196);
  EXPECT_DOUBLE_EQ(c.footprint.max_y, 0.196);
  EXPECT_DOUBLE_EQ(c.lookahead_m, 0.4);
  EXPECT_DOUBLE_EQ(c.floor_margin_m, 0.05);
  EXPECT_DOUBLE_EQ(c.pan_min_rad, -1.5708);
  EXPECT_DOUBLE_EQ(c.pan_max_rad, 1.5708);
  EXPECT_DOUBLE_EQ(c.tilt_min_rad, -0.5236);
  EXPECT_DOUBLE_EQ(c.tilt_max_rad, 0.7854);
  EXPECT_DOUBLE_EQ(c.home_pan_rad, 0.0);
  EXPECT_DOUBLE_EQ(c.home_tilt_rad, 0.5236);
  EXPECT_DOUBLE_EQ(c.reaim_pan_deg, 10.0);
  EXPECT_DOUBLE_EQ(c.reaim_tilt_deg, 5.0);
  EXPECT_DOUBLE_EQ(c.min_reaim_interval_s, 1.0);
  EXPECT_DOUBLE_EQ(c.rate_hz, 5.0);
  EXPECT_DOUBLE_EQ(c.tf_timeout_s, 0.1);
  EXPECT_DOUBLE_EQ(c.input_timeout_s, 1.0);
  EXPECT_DOUBLE_EQ(c.min_travel_speed_mps, 0.02);
}

TEST(Config, EmptyDocumentGivesTheShippedDefaults)
{
  EXPECT_TRUE(errors_of("{}").empty());
  const Config defaults = head_aim_cpp::from_yaml(YAML::Load("{}"));
  const Config shipped = head_aim_cpp::from_yaml(YAML::LoadFile(shipped_yaml_path()));
  EXPECT_EQ(defaults.plan_topic, shipped.plan_topic);
  EXPECT_EQ(defaults.yield_sources, shipped.yield_sources);
  EXPECT_DOUBLE_EQ(defaults.footprint.max_x, shipped.footprint.max_x);
  EXPECT_DOUBLE_EQ(defaults.home_tilt_rad, shipped.home_tilt_rad);
  EXPECT_DOUBLE_EQ(defaults.min_travel_speed_mps, shipped.min_travel_speed_mps);
}

TEST(Config, UnknownKeyIsRejected) { expect_one_error("head_state_topic: /head/joint_states", "head_state_topic"); }

TEST(Config, NegativeYieldIsRejected) { expect_one_error("yield_s: -1.0", "yield_s"); }

TEST(Config, PanRangeMustBeIncreasing)
{
  expect_one_error("{pan_min_rad: 0.5, pan_max_rad: 0.5}", "pan_min_rad");
  expect_one_error("{pan_min_rad: 1.0, pan_max_rad: 0.5}", "pan_min_rad");
}

TEST(Config, PanRangeStaysInsideTheServoLimits)
{
  expect_one_error("pan_min_rad: -1.7", "pan_min_rad");
  expect_one_error("pan_max_rad: 1.7", "pan_max_rad");
}

TEST(Config, TiltRangeStaysInsideTheServoLimits)
{
  expect_one_error("tilt_min_rad: -0.6", "tilt_min_rad");
  expect_one_error("tilt_max_rad: 0.9", "tilt_max_rad");
}

TEST(Config, TiltRangeMustBeIncreasing) { expect_one_error("{tilt_min_rad: 0.3, tilt_max_rad: 0.2}", "tilt_min_rad"); }

TEST(Config, FootprintMustBeIncreasing)
{
  expect_one_error("{footprint_min_x: 0.4}", "footprint_min_x");
  expect_one_error("{footprint_min_y: 0.2}", "footprint_min_y");
}

TEST(Config, HomeMustLieInsideTheRanges)
{
  expect_one_error("{home_pan_rad: 0.5, pan_max_rad: 0.4}", "home_pan_rad");
  expect_one_error("home_tilt_rad: -0.6", "home_tilt_rad");
}

TEST(Config, OwnSourceTagMayNotBeAYieldSource)
{
  expect_one_error("yield_sources: [web_ui, head_aim]", "yield_sources");
}

TEST(Config, ScalarRules)
{
  expect_one_error("lookahead_m: 0.0", "lookahead_m");
  expect_one_error("floor_margin_m: -0.01", "floor_margin_m");
  expect_one_error("reaim_pan_deg: -1.0", "reaim_pan_deg");
  expect_one_error("reaim_tilt_deg: -1.0", "reaim_tilt_deg");
  expect_one_error("min_reaim_interval_s: -0.5", "min_reaim_interval_s");
  expect_one_error("rate_hz: 0.0", "rate_hz");
  expect_one_error("rate_hz: 60.0", "rate_hz");
  expect_one_error("tf_timeout_s: 0.6", "tf_timeout_s");
  expect_one_error("input_timeout_s: 0.0", "input_timeout_s");
  expect_one_error("min_travel_speed_mps: 0.0", "min_travel_speed_mps");
  expect_one_error("source_tag: ''", "source_tag");
  expect_one_error("plan_topic: ''", "plan_topic");
  expect_one_error("yield_s: soon", "yield_s");
}
