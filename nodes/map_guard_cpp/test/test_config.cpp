// Port of nodes/map_guard/tests/test_config.py (defaults, YAML loading, validation, exit 78) plus the config.json
// golden fixture: for every YAML document the C++ validator accepts exactly what the pydantic model accepts, with the
// same parsed values, and rejects the same fields.
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "fixture_util.hpp"
#include "map_guard_cpp/config.hpp"

namespace mg = map_guard_cpp;
namespace fx = map_guard_cpp::test;
using nlohmann::json;

namespace
{

std::string write_temp(const std::string & text)
{
  char path[] = "/tmp/map_guard_cpp_config_XXXXXX";
  const int fd = mkstemp(path);
  EXPECT_GE(fd, 0);
  close(fd);
  std::ofstream(path) << text;
  return path;
}

mg::Config parse(const std::string & yaml, ros2_common_cpp::Errors & errors)
{
  return mg::parse_config(YAML::Load(yaml), errors);
}

bool valid(const std::string & yaml) { return mg::validate_config(YAML::Load(yaml)).empty(); }

// Every field as JSON (the Python model_dump()), non-finite numbers as "inf" / "-inf" / "nan".
json dump(const mg::Config & c)
{
  auto num = [](double v) -> json {
    if (std::isnan(v)) {
      return "nan";
    }
    if (std::isinf(v)) {
      return v > 0 ? "inf" : "-inf";
    }
    return v;
  };
  return {
    {"dry_run", c.dry_run},
    {"reasons", c.reasons},
    {"scan_topic", c.scan_topic},
    {"map_topic", c.map_topic},
    {"imu_topic", c.imu_topic},
    {"wheel_odom_topic", c.wheel_odom_topic},
    {"lidar_odom_topic", c.lidar_odom_topic},
    {"cmd_vel_topic", c.cmd_vel_topic},
    {"map_frame", c.map_frame},
    {"reset_topic", c.reset_topic},
    {"events_topic", c.events_topic},
    {"pause_service", c.pause_service},
    {"reset_service", c.reset_service},
    {"serialize_service", c.serialize_service},
    {"parameters_service", c.parameters_service},
    {"service_timeout_s", num(c.service_timeout_s)},
    {"agent_state_url", c.agent_state_url},
    {"agent_poll_s", num(c.agent_poll_s)},
    {"agent_timeout_s", num(c.agent_timeout_s)},
    {"metrics_port", c.metrics_port ? json(*c.metrics_port) : json(nullptr)},
    {"tick_hz", num(c.tick_hz)},
    {"max_range_m", num(c.max_range_m)},
    {"hit_radius_cells", c.hit_radius_cells},
    {"occupied_threshold", c.occupied_threshold},
    {"min_points", c.min_points},
    {"agreement_hz", num(c.agreement_hz)},
    {"median_window", c.median_window},
    {"mismatch_ratio", num(c.mismatch_ratio)},
    {"mismatch_s", num(c.mismatch_s)},
    {"healthy_ratio", num(c.healthy_ratio)},
    {"min_map_cells_known", c.min_map_cells_known},
    {"lift_tilt_deg", num(c.lift_tilt_deg)},
    {"lift_s", num(c.lift_s)},
    {"level_window_s", num(c.level_window_s)},
    {"level_tolerance_deg", num(c.level_tolerance_deg)},
    {"level_rebaseline_s", num(c.level_rebaseline_s)},
    {"lift_accel_z", num(c.lift_accel_z)},
    {"lift_accel_s", num(c.lift_accel_s)},
    {"lift_confirm_s", num(c.lift_confirm_s)},
    {"lift_confirm_tilt_deg", num(c.lift_confirm_tilt_deg)},
    {"gravity_band", num(c.gravity_band)},
    {"carried_speed_mps", num(c.carried_speed_mps)},
    {"wheel_still_mps", num(c.wheel_still_mps)},
    {"cmd_timeout_s", num(c.cmd_timeout_s)},
    {"cmd_eps", num(c.cmd_eps)},
    {"carried_s", num(c.carried_s)},
    {"input_timeout_s", num(c.input_timeout_s)},
    {"rotate_rate_rad_s", num(c.rotate_rate_rad_s)},
    {"wheel_still_rad_s", num(c.wheel_still_rad_s)},
    {"rotate_s", num(c.rotate_s)},
    {"settle_s", num(c.settle_s)},
    {"evaluate_s", num(c.evaluate_s)},
    {"cooldown_s", num(c.cooldown_s)},
    {"backup_dir", c.backup_dir},
    {"backup_prefix", c.backup_prefix},
    {"keep_backups", c.keep_backups},
  };
}

}  // namespace

TEST(Config, DefaultsMatchTheDocumentedValues)
{
  const mg::Config config;
  EXPECT_TRUE(config.dry_run);
  EXPECT_EQ(config.reasons, (std::vector<std::string>{"mismatch", "lifted", "carried", "rotated"}));
  EXPECT_EQ(config.metrics_port, 19110);
  EXPECT_EQ(config.agent_state_url, "http://127.0.0.1:18300/api/state");
  EXPECT_EQ(config.hit_radius_cells, 1);
  EXPECT_EQ(config.occupied_threshold, 65);
  EXPECT_EQ(config.min_points, 40);
  EXPECT_EQ(config.agreement_hz, 2.0);
  EXPECT_EQ(config.mismatch_ratio, 0.3);
  EXPECT_EQ(config.mismatch_s, 5.0);
  EXPECT_EQ(config.healthy_ratio, 0.6);
  EXPECT_EQ(config.lift_tilt_deg, 12.0);
  EXPECT_EQ(config.lift_s, 1.0);
  EXPECT_EQ(config.lift_accel_z, 1.5);
  EXPECT_EQ(config.lift_accel_s, 0.4);
  EXPECT_EQ(config.carried_speed_mps, 0.08);
  EXPECT_EQ(config.wheel_still_mps, 0.02);
  EXPECT_EQ(config.cmd_timeout_s, 0.5);
  EXPECT_EQ(config.carried_s, 1.0);
  EXPECT_EQ(config.rotate_rate_rad_s, 0.3);
  EXPECT_EQ(config.rotate_s, 0.8);
  EXPECT_EQ(config.settle_s, 3.0);
  EXPECT_EQ(config.evaluate_s, 4.0);
  EXPECT_EQ(config.cooldown_s, 30.0);
  EXPECT_EQ(config.keep_backups, 5);
  EXPECT_EQ(config.backup_dir, "/var/lib/ros2/maps");
  EXPECT_EQ(config.backup_prefix, "map_guard_backup");
  EXPECT_EQ(config.pause_service, "/slam_toolbox/pause_new_measurements");
  EXPECT_EQ(config.reset_service, "/slam_toolbox/reset");
  EXPECT_EQ(config.serialize_service, "/slam_toolbox/serialize_map");
  EXPECT_EQ(config.reset_topic, "/map_guard/map_reset");
  EXPECT_EQ(config.events_topic, "/robot_events");
}

TEST(Config, InvalidValuesAreRejected)
{
  for (const std::string yaml :
       {"lift_s: 0.0", "settle_s: -1.0", "agreement_hz: 0.0", "mismatch_ratio: 1.5", "healthy_ratio: -0.1",
        "{healthy_ratio: 0.3, mismatch_ratio: 0.3}", "{healthy_ratio: 0.2, mismatch_ratio: 0.3}", "reasons: [bumped]",
        "keep_backups: 0", "hit_radius_cells: -1", "occupied_threshold: 101", "no_such_key: 1"}) {
    EXPECT_FALSE(valid(yaml)) << yaml;
  }
}

TEST(Config, ParseReadsYaml)
{
  ros2_common_cpp::Errors errors;
  const auto config = parse("dry_run: false\nreasons: [lifted]\nmetrics_port: 19999\nsettle_s: 2.5\n", errors);
  EXPECT_TRUE(errors.empty());
  EXPECT_FALSE(config.dry_run);
  EXPECT_EQ(config.reasons, std::vector<std::string>{"lifted"});
  EXPECT_EQ(config.metrics_port, 19999);
  EXPECT_EQ(config.settle_s, 2.5);
}

TEST(Config, ErrorsNameTheField)
{
  ros2_common_cpp::Errors errors;
  parse("no_such_key: 1\nlift_s: 0\n", errors);
  ASSERT_EQ(errors.size(), 2u);
  EXPECT_EQ(errors[0], "no_such_key: extra field not permitted");
  EXPECT_EQ(errors[1].rfind("lift_s: ", 0), 0u);
}

TEST(Config, LoadFromEnv)
{
  const auto path = write_temp("cooldown_s: 12\n");
  setenv(mg::ENV_CONFIG_PATH_KEY, path.c_str(), 1);
  const auto config = mg::load_config_or_exit();
  EXPECT_EQ(config.cooldown_s, 12.0);
  unsetenv(mg::ENV_CONFIG_PATH_KEY);
  std::remove(path.c_str());
}

TEST(ConfigDeathTest, InvalidConfigExits78)
{
  const auto path = write_temp("lift_s: 0\nno_such_key: 1\n");
  setenv(mg::ENV_CONFIG_PATH_KEY, path.c_str(), 1);
  EXPECT_EXIT(mg::load_config_or_exit(), ::testing::ExitedWithCode(78), "no_such_key: extra field not permitted");
  unsetenv(mg::ENV_CONFIG_PATH_KEY);
  std::remove(path.c_str());
}

TEST(ConfigDeathTest, MissingOrNonMappingConfigExits78)
{
  setenv(mg::ENV_CONFIG_PATH_KEY, "/nonexistent/map_guard/config.yaml", 1);
  EXPECT_EXIT(mg::load_config_or_exit(), ::testing::ExitedWithCode(78), "cannot read config file");
  const auto path = write_temp("- 1\n- 2\n");
  setenv(mg::ENV_CONFIG_PATH_KEY, path.c_str(), 1);
  EXPECT_EXIT(mg::load_config_or_exit(), ::testing::ExitedWithCode(78), "mapping");
  unsetenv(mg::ENV_CONFIG_PATH_KEY);
  std::remove(path.c_str());
}

// --- Golden fixture (pydantic verdicts) -----------------------------------------------------------------------------

TEST(ConfigFixture, DefaultsMatchPython)
{
  const auto f = fx::load_fixture("config.json");
  EXPECT_EQ(dump(mg::Config()), f["defaults"]);
  std::vector<std::string> keys;
  for (const auto & [key, value] : f["defaults"].items()) {
    keys.push_back(key);
  }
  std::vector<std::string> ours = mg::config_keys();
  std::sort(ours.begin(), ours.end());
  EXPECT_EQ(ours, keys);
}

TEST(ConfigFixture, VerdictsAndValuesMatchPydantic)
{
  const auto f = fx::load_fixture("config.json");
  for (const auto & c : f["cases"]) {
    const std::string yaml = c["yaml"];
    ros2_common_cpp::Errors errors;
    const auto config = parse(yaml, errors);
    std::string all_errors;
    for (const auto & e : errors) {
      all_errors += e + "; ";
    }
    if (c["valid"].get<bool>()) {
      EXPECT_TRUE(errors.empty()) << yaml << " -> " << all_errors;
      EXPECT_EQ(dump(config), c["values"]) << yaml;
      continue;
    }
    EXPECT_FALSE(errors.empty()) << yaml;
    for (const auto & field : c["error_fields"]) {
      const std::string prefix = field.get<std::string>() + ": ";
      bool found = false;
      for (const auto & e : errors) {
        found = found || e.rfind(prefix, 0) == 0;
      }
      EXPECT_TRUE(found) << yaml << " should reject " << field << ", got: " << all_errors;
    }
  }
}
