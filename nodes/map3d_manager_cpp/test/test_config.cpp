// Defaults, every bound and the unknown-key rejection of the map3d manager config.
#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "map3d_manager_cpp/config.hpp"

namespace m3 = map3d_manager_cpp;

namespace
{

ros2_common_cpp::Errors errors_of(const std::string & yaml) { return m3::validate_config(YAML::Load(yaml)); }

m3::Config parse(const std::string & yaml)
{
  ros2_common_cpp::Errors errors;
  return m3::parse_config(YAML::Load(yaml), errors);
}

bool mentions(const ros2_common_cpp::Errors & errors, const std::string & key)
{
  return std::any_of(
    errors.begin(), errors.end(), [&](const std::string & e) { return e.find(key) != std::string::npos; });
}

}  // namespace

TEST(Map3dConfig, DefaultsOfEmptyMap)
{
  const m3::Config c = parse("{}");
  EXPECT_EQ(c.command_topic, "/uplink/map3d/command");
  EXPECT_EQ(c.map_reset_topic, "/uplink/map_guard/map_reset");
  EXPECT_EQ(c.result_topic, "/map3d/result_server");
  EXPECT_EQ(c.status_topic, "/map3d/status_server");
  EXPECT_EQ(c.rtabmap_namespace, "/rtabmap");
  EXPECT_EQ(c.database_dir, "/var/lib/ros2/rtabmap");
  EXPECT_DOUBLE_EQ(c.status_period_s, 1.0);
  EXPECT_DOUBLE_EQ(c.service_timeout_s, 10.0);
  EXPECT_DOUBLE_EQ(c.input_stale_s, 3.0);
  EXPECT_DOUBLE_EQ(c.correction_warn_m, 0.05);
  EXPECT_DOUBLE_EQ(c.correction_warn_deg, 2.0);
  EXPECT_EQ(c.rgb_info_topic, "/uplink/head_rgbd/rgb/camera_info");
  EXPECT_EQ(c.depth_info_topic, "/uplink/head_rgbd/depth/camera_info");
  EXPECT_EQ(c.rtabmap_unit, "ros2-rtabmap_mapping.service");
  EXPECT_TRUE(errors_of("{}").empty());
}

TEST(Map3dConfig, OverridesAreRead)
{
  const m3::Config c = parse("{command_topic: /x, status_period_s: 2.5, correction_warn_m: 0, database_dir: /d}");
  EXPECT_EQ(c.command_topic, "/x");
  EXPECT_DOUBLE_EQ(c.status_period_s, 2.5);
  EXPECT_DOUBLE_EQ(c.correction_warn_m, 0.0);
  EXPECT_EQ(c.database_dir, "/d");
}

TEST(Map3dConfig, PositiveBounds)
{
  for (const char * key : {"status_period_s", "service_timeout_s", "input_stale_s"}) {
    EXPECT_TRUE(mentions(errors_of(std::string("{") + key + ": 0}"), key)) << key;
    EXPECT_TRUE(mentions(errors_of(std::string("{") + key + ": -1}"), key)) << key;
    EXPECT_TRUE(errors_of(std::string("{") + key + ": 0.1}").empty()) << key;
  }
}

TEST(Map3dConfig, NonNegativeBounds)
{
  for (const char * key : {"correction_warn_m", "correction_warn_deg"}) {
    EXPECT_TRUE(mentions(errors_of(std::string("{") + key + ": -0.1}"), key)) << key;
    EXPECT_TRUE(errors_of(std::string("{") + key + ": 0}").empty()) << key;
  }
}

TEST(Map3dConfig, WrongTypeIsError)
{
  EXPECT_TRUE(mentions(errors_of("{status_period_s: fast}"), "status_period_s"));
  EXPECT_TRUE(mentions(errors_of("{command_topic: [a, b]}"), "command_topic"));
}

TEST(Map3dConfig, UnknownKeyIsError)
{
  const auto errors = errors_of("{bogus: 1}");
  ASSERT_FALSE(errors.empty());
  EXPECT_TRUE(mentions(errors, "bogus"));
}

TEST(Map3dConfig, ConfigKeysListEveryField) { EXPECT_EQ(m3::config_keys().size(), 14u); }
