// jetson_bridge_cpp config: defaults, overrides, lax coercion and rejections.
#include <gtest/gtest.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

#include "jetson_bridge_cpp/config.hpp"

namespace
{
using jetson_bridge_cpp::BridgeConfig;

bool has_error(const ros2_common_cpp::Errors & errors, const std::string & needle)
{
  return std::any_of(
    errors.begin(), errors.end(), [&](const std::string & e) { return e.find(needle) != std::string::npos; });
}

ros2_common_cpp::Errors errors_of(const std::string & yaml) { return jetson_bridge_cpp::validate(YAML::Load(yaml)); }

std::string write_temp(const std::string & text)
{
  char path[] = "/tmp/jetson_bridge_cfg_XXXXXX";
  const int fd = mkstemp(path);
  close(fd);
  std::ofstream(path) << text;
  return path;
}
}  // namespace

TEST(Config, DefaultsAreTheTopicContract)
{
  const BridgeConfig c = jetson_bridge_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_EQ(c.endpoint, "tcp://10.77.0.2:5550");
  EXPECT_EQ(c.depth_topic, "/stereo/depth/image_raw");
  EXPECT_EQ(c.camera_info_topic, "/stereo/camera_info");
  EXPECT_EQ(c.fused_topic, "/stereo/depth/fused");
  EXPECT_EQ(c.rgb_topic, "/overview_camera/image_raw/compressed");
  EXPECT_EQ(c.rgb_info_topic, "/overview_camera/camera_info");
  EXPECT_EQ(c.max_age_ms, 500);
  EXPECT_EQ(c.absent_after_ms, 2000);
  ASSERT_TRUE(c.metrics_port.has_value());
  EXPECT_EQ(*c.metrics_port, 19114);
  EXPECT_TRUE(errors_of("{}").empty());
}

TEST(Config, OverridesEveryKey)
{
  const std::string yaml =
    "endpoint: tcp://127.0.0.1:6000\n"
    "depth_topic: /a/depth\ncamera_info_topic: /a/info\nfused_topic: /a/fused\n"
    "rgb_topic: /a/rgb\nrgb_info_topic: /a/rgb_info\n"
    "max_age_ms: 250\nabsent_after_ms: 900\nmetrics_port: 20001\n";
  EXPECT_TRUE(errors_of(yaml).empty());
  const BridgeConfig c = jetson_bridge_cpp::from_yaml(YAML::Load(yaml));
  EXPECT_EQ(c.endpoint, "tcp://127.0.0.1:6000");
  EXPECT_EQ(c.depth_topic, "/a/depth");
  EXPECT_EQ(c.camera_info_topic, "/a/info");
  EXPECT_EQ(c.fused_topic, "/a/fused");
  EXPECT_EQ(c.rgb_topic, "/a/rgb");
  EXPECT_EQ(c.rgb_info_topic, "/a/rgb_info");
  EXPECT_EQ(c.max_age_ms, 250);
  EXPECT_EQ(c.absent_after_ms, 900);
  EXPECT_EQ(*c.metrics_port, 20001);
}

TEST(Config, LaxCoercionAndNullPortDisablesMetrics)
{
  const BridgeConfig c = jetson_bridge_cpp::from_yaml(YAML::Load("{max_age_ms: '300', metrics_port: ~}"));
  EXPECT_EQ(c.max_age_ms, 300);
  EXPECT_FALSE(c.metrics_port.has_value());
}

TEST(Config, RejectsUnknownKeys)
{
  EXPECT_TRUE(has_error(errors_of("{bogus: 1}"), "bogus: extra field not permitted"));
}

TEST(Config, RejectsBadValues)
{
  EXPECT_TRUE(has_error(errors_of("{max_age_ms: 0}"), "max_age_ms: must be > 0"));
  EXPECT_TRUE(has_error(errors_of("{absent_after_ms: -5}"), "absent_after_ms: must be > 0"));
  EXPECT_TRUE(has_error(errors_of("{metrics_port: 70000}"), "metrics_port: must be <= 65535"));
  EXPECT_TRUE(has_error(errors_of("{metrics_port: 0}"), "metrics_port: must be >= 1"));
  EXPECT_TRUE(has_error(errors_of("{max_age_ms: fast}"), "max_age_ms: expected int"));
  EXPECT_TRUE(has_error(errors_of("{endpoint: '  '}"), "endpoint: must not be empty"));
  EXPECT_TRUE(has_error(errors_of("{endpoint: 'http://x'}"), "endpoint: must start with tcp://"));
  EXPECT_TRUE(has_error(errors_of("{depth_topic: ''}"), "depth_topic: must not be empty"));
  EXPECT_TRUE(has_error(errors_of("{rgb_info_topic: ' '}"), "rgb_info_topic: must not be empty"));
  EXPECT_TRUE(has_error(errors_of("[1, 2]"), "expected mapping"));
}

TEST(Config, LoadConfigReadsFileAndReportsErrors)
{
  const auto ok = jetson_bridge_cpp::load_config(write_temp("max_age_ms: 400\n"));
  EXPECT_TRUE(ok.errors.empty());
  EXPECT_EQ(ok.config.max_age_ms, 400);
  const auto bad = jetson_bridge_cpp::load_config(write_temp("bogus: 1\n"));
  EXPECT_FALSE(bad.errors.empty());
  const auto missing = jetson_bridge_cpp::load_config("/nonexistent/jetson_bridge.yaml");
  EXPECT_FALSE(missing.errors.empty());
}

TEST(ConfigDeathTest, InvalidConfigExits78)
{
  const std::string path = write_temp("bogus: 1\n");
  setenv(jetson_bridge_cpp::CONFIG_ENV_VAR, path.c_str(), 1);
  EXPECT_EXIT(jetson_bridge_cpp::load_config_or_exit(), ::testing::ExitedWithCode(78), "bogus");
  unsetenv(jetson_bridge_cpp::CONFIG_ENV_VAR);
}
