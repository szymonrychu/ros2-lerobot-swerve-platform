// stereo_depth_cpp config: the defaults, every key, the accepted bounds and every rejection of the Python DepthConfig
// (nodes/stereo_depth/stereo_depth/config.py), from test/fixtures/config.yaml written by gen_fixtures.py; plus the
// file-level errors and the exit code 78.
#include <gtest/gtest.h>
#include <stdlib.h>

#include <algorithm>
#include <string>
#include <vector>

#include "fixtures.hpp"
#include "stereo_depth_cpp/config.hpp"

namespace
{

using stereo_depth_cpp::DepthConfig;

void expect_config_equals(const DepthConfig & c, const YAML::Node & v, const std::string & what)
{
  SCOPED_TRACE(what);
  EXPECT_EQ(c.left_image_topic, v["left_image_topic"].as<std::string>());
  EXPECT_EQ(c.right_image_topic, v["right_image_topic"].as<std::string>());
  EXPECT_EQ(c.depth_topic, v["depth_topic"].as<std::string>());
  EXPECT_EQ(c.camera_info_topic, v["camera_info_topic"].as<std::string>());
  EXPECT_EQ(c.publish_disparity, v["publish_disparity"].as<bool>());
  EXPECT_EQ(c.disparity_topic, v["disparity_topic"].as<std::string>());
  EXPECT_EQ(c.calibration_dir, v["calibration_dir"].as<std::string>());
  EXPECT_DOUBLE_EQ(c.calibration_check_s, v["calibration_check_s"].as<double>());
  EXPECT_EQ(c.num_disparities, v["num_disparities"].as<int>());
  EXPECT_EQ(c.block_size, v["block_size"].as<int>());
  EXPECT_EQ(c.p1, v["p1"].as<int>());
  EXPECT_EQ(c.p2, v["p2"].as<int>());
  EXPECT_EQ(c.uniqueness_ratio, v["uniqueness_ratio"].as<int>());
  EXPECT_EQ(c.speckle_window_size, v["speckle_window_size"].as<int>());
  EXPECT_EQ(c.speckle_range, v["speckle_range"].as<int>());
  EXPECT_EQ(c.disp12_max_diff, v["disp12_max_diff"].as<int>());
  EXPECT_EQ(c.pre_filter_cap, v["pre_filter_cap"].as<int>());
  EXPECT_DOUBLE_EQ(c.min_depth_m, v["min_depth_m"].as<double>());
  EXPECT_DOUBLE_EQ(c.max_depth_m, v["max_depth_m"].as<double>());
  EXPECT_DOUBLE_EQ(c.sync_slop_s, v["sync_slop_s"].as<double>());
  EXPECT_DOUBLE_EQ(c.max_rate_hz, v["max_rate_hz"].as<double>());
  EXPECT_EQ(c.matcher_threads, v["matcher_threads"].as<int>());
  EXPECT_EQ(c.frame_id, v["frame_id"].as<std::string>());
  if (v["metrics_port"].IsNull()) {
    EXPECT_FALSE(c.metrics_port.has_value());
  } else {
    ASSERT_TRUE(c.metrics_port.has_value());
    EXPECT_EQ(*c.metrics_port, v["metrics_port"].as<int>());
  }
}

stereo_depth_cpp::ConfigLoad load_text(const std::string & text)
{
  fixtures::TempDir tmp;
  return stereo_depth_cpp::load_config(tmp.write("config.yaml", text));
}

}  // namespace

TEST(Config, KeysAreThePythonFields)
{
  std::vector<std::string> python_keys;
  for (const auto & entry : fixtures::yaml("config.yaml")["defaults"]) {
    python_keys.push_back(entry.first.as<std::string>());
  }
  EXPECT_EQ(stereo_depth_cpp::config_keys(), python_keys);
}

TEST(Config, DefaultsAreThePythonDefaults)
{
  expect_config_equals(DepthConfig{}, fixtures::yaml("config.yaml")["defaults"], "struct defaults");
  const auto loaded = load_text("");
  EXPECT_TRUE(loaded.errors.empty());
  expect_config_equals(loaded.config, fixtures::yaml("config.yaml")["defaults"], "empty file");
}

TEST(Config, EveryAcceptedTextGivesThePythonValues)
{
  const YAML::Node accepted = fixtures::yaml("config.yaml")["accepted"];
  ASSERT_GT(accepted.size(), 10u);
  for (const auto & item : accepted) {
    const std::string text = item["text"].as<std::string>();
    const auto loaded = load_text(text);
    EXPECT_TRUE(loaded.errors.empty()) << text << ": " << (loaded.errors.empty() ? "" : loaded.errors.front());
    expect_config_equals(loaded.config, item["value"], text);
  }
}

TEST(Config, EveryPythonRejectionIsRejected)
{
  const YAML::Node rejected = fixtures::yaml("config.yaml")["rejected"];
  ASSERT_GT(rejected.size(), 40u);
  for (const auto & item : rejected) {
    const std::string text = item.as<std::string>();
    EXPECT_FALSE(load_text(text).errors.empty()) << "accepted although Python rejects it: " << text;
  }
}

TEST(Config, ErrorsNameTheField)
{
  const auto has = [](const stereo_depth_cpp::ConfigLoad & loaded, const std::string & needle) {
    return std::any_of(loaded.errors.begin(), loaded.errors.end(), [&](const std::string & e) {
      return e.find(needle) != std::string::npos;
    });
  };
  EXPECT_TRUE(has(load_text("unknown_key: 1\n"), "unknown_key: extra field not permitted"));
  EXPECT_TRUE(has(load_text("num_disparities: 40\n"), "num_disparities: must be a multiple of 16"));
  EXPECT_TRUE(has(load_text("block_size: 4\n"), "block_size: must be odd"));
  EXPECT_TRUE(has(load_text("p1: 800\np2: 200\n"), "p1 must be below p2"));
  EXPECT_TRUE(has(load_text("min_depth_m: 2.0\nmax_depth_m: 1.0\n"), "min_depth_m must be below max_depth_m"));
  EXPECT_TRUE(has(load_text("frame_id: '  '\n"), "frame_id: must not be empty"));
  EXPECT_TRUE(has(load_text("depth_topic: ''\n"), "depth_topic: topic must not be empty"));
  EXPECT_TRUE(has(load_text("sync_slop_s: 0.5\n"), "sync_slop_s: must be <= 0.1"));
  EXPECT_TRUE(has(load_text("num_disparities: abc\n"), "num_disparities: expected int"));
}

TEST(Config, EveryErrorIsReportedAtOnce)
{
  const auto loaded = load_text("unknown_key: 1\nblock_size: 4\nmatcher_threads: 0\n");
  EXPECT_EQ(loaded.errors.size(), 3u);
}

TEST(Config, MissingFileIsAnError)
{
  fixtures::TempDir tmp;
  EXPECT_FALSE(stereo_depth_cpp::load_config(tmp.path() + "/nope.yaml").errors.empty());
}

TEST(Config, InvalidConfigExits78)
{
  fixtures::TempDir tmp;
  const std::string bad = tmp.write("bad.yaml", "num_disparities: 40\n");
  EXPECT_EXIT(
    {
      setenv(stereo_depth_cpp::CONFIG_ENV_VAR, bad.c_str(), 1);
      stereo_depth_cpp::load_config_or_exit();
    },
    ::testing::ExitedWithCode(78), "num_disparities");
}

TEST(Config, MissingConfigFileExits78)
{
  fixtures::TempDir tmp;
  const std::string missing = tmp.path() + "/missing.yaml";
  EXPECT_EXIT(
    {
      setenv(stereo_depth_cpp::CONFIG_ENV_VAR, missing.c_str(), 1);
      stereo_depth_cpp::load_config_or_exit();
    },
    ::testing::ExitedWithCode(78), "config");
}

TEST(Config, ValidConfigFromEnvLoads)
{
  fixtures::TempDir tmp;
  const std::string good = tmp.write("good.yaml", "max_rate_hz: 2.0\nframe_id: ' cam '\n");
  setenv(stereo_depth_cpp::CONFIG_ENV_VAR, good.c_str(), 1);
  const DepthConfig config = stereo_depth_cpp::load_config_or_exit();
  unsetenv(stereo_depth_cpp::CONFIG_ENV_VAR);
  EXPECT_DOUBLE_EQ(config.max_rate_hz, 2.0);
  EXPECT_EQ(config.frame_id, "cam");
}
