// ros2_common_cpp config helpers: unknown key, wrong type, out of range, missing required, exit 78.
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "ros2_common_cpp/config.hpp"

namespace rc = ros2_common_cpp;

namespace
{

// The validator of a small example node: rate_hz required and > 0, queue in 1..100, topic optional string.
rc::Errors validate_example(const YAML::Node & cfg)
{
  rc::Errors errors;
  rc::forbid_unknown(cfg, {"rate_hz", "queue", "topic", "camera"}, errors);
  if (auto rate = rc::require<double>(cfg, "rate_hz", errors)) {
    rc::gt("rate_hz", *rate, 0.0, errors);
  }
  const int queue = rc::get_or<int>(cfg, "queue", 10, errors);
  rc::ge("queue", queue, 1, errors);
  rc::le("queue", queue, 100, errors);
  rc::get_or<std::string>(cfg, "topic", "/scan", errors);
  if (cfg["camera"]) {
    rc::forbid_unknown(cfg["camera"], {"width"}, errors, "camera");
  }
  return errors;
}

bool contains(const rc::Errors & errors, const std::string & expected)
{
  for (const auto & e : errors) {
    if (e == expected) {
      return true;
    }
  }
  return false;
}

std::string write_temp(const std::string & text)
{
  char path[] = "/tmp/ros2_common_cpp_test_XXXXXX";
  const int fd = mkstemp(path);
  EXPECT_GE(fd, 0);
  close(fd);
  std::ofstream(path) << text;
  return path;
}

}  // namespace

TEST(Config, ValidConfigHasNoErrors)
{
  EXPECT_TRUE(validate_example(YAML::Load("{rate_hz: 5.0, queue: 3, topic: /x}")).empty());
  EXPECT_TRUE(validate_example(YAML::Load("{rate_hz: 1}")).empty());
}

TEST(Config, UnknownKeyIsRejected)
{
  const auto errors = validate_example(YAML::Load("{rate_hz: 5.0, rat_hz: 1}"));
  EXPECT_EQ(errors, rc::Errors{"rat_hz: extra field not permitted"});
}

TEST(Config, UnknownNestedKeyCarriesItsPath)
{
  const auto errors = validate_example(YAML::Load("{rate_hz: 5.0, camera: {width: 640, hight: 480}}"));
  EXPECT_EQ(errors, rc::Errors{"camera.hight: extra field not permitted"});
}

TEST(Config, NonMappingSectionIsRejected)
{
  const auto errors = validate_example(YAML::Load("{rate_hz: 5.0, camera: [1, 2]}"));
  EXPECT_EQ(errors, rc::Errors{"camera: expected mapping"});
}

TEST(Config, WrongTypeIsRejected)
{
  EXPECT_EQ(validate_example(YAML::Load("{rate_hz: fast}")), rc::Errors{"rate_hz: expected float"});
  EXPECT_EQ(validate_example(YAML::Load("{rate_hz: 1, queue: 2.5}")), rc::Errors{"queue: expected int"});
  EXPECT_EQ(validate_example(YAML::Load("{rate_hz: 1, topic: [a]}")), rc::Errors{"topic: expected string"});
  EXPECT_EQ(validate_example(YAML::Load("{rate_hz: ~}")), rc::Errors{"rate_hz: expected float"});
  rc::Errors errors;
  EXPECT_FALSE(rc::require<bool>(YAML::Load("{on: maybe}"), "on", errors).has_value());
  EXPECT_EQ(errors, rc::Errors{"on: expected bool"});
}

TEST(Config, OutOfRangeIsRejected)
{
  EXPECT_EQ(validate_example(YAML::Load("{rate_hz: 0}")), rc::Errors{"rate_hz: must be > 0 (got 0)"});
  EXPECT_EQ(validate_example(YAML::Load("{rate_hz: 1, queue: 0}")), rc::Errors{"queue: must be >= 1 (got 0)"});
  EXPECT_EQ(validate_example(YAML::Load("{rate_hz: 1, queue: 101}")), rc::Errors{"queue: must be <= 100 (got 101)"});
}

TEST(Config, RangeBoundsAreInclusiveWhereStated)
{
  rc::Errors errors;
  rc::ge("a", 1.0, 1.0, errors);
  rc::le("b", 2, 2, errors);
  EXPECT_TRUE(errors.empty());
  rc::gt("c", 1.0, 1.0, errors);
  EXPECT_EQ(errors.size(), 1u);
}

TEST(Config, MissingRequiredIsRejected)
{
  EXPECT_EQ(validate_example(YAML::Load("{queue: 3}")), rc::Errors{"rate_hz: field required"});
}

TEST(Config, EveryErrorIsReported)
{
  const auto errors = validate_example(YAML::Load("{queue: 0, extra: 1}"));
  EXPECT_EQ(errors.size(), 3u);
  EXPECT_TRUE(contains(errors, "extra: extra field not permitted"));
  EXPECT_TRUE(contains(errors, "rate_hz: field required"));
  EXPECT_TRUE(contains(errors, "queue: must be >= 1 (got 0)"));
}

TEST(Config, PathFromEnvWithDefault)
{
  unsetenv("ROS2_COMMON_CPP_TEST_CONFIG");
  EXPECT_EQ(
    rc::config_path_from_env("ROS2_COMMON_CPP_TEST_CONFIG", "/etc/ros2/x/config.yaml"), "/etc/ros2/x/config.yaml");
  setenv("ROS2_COMMON_CPP_TEST_CONFIG", "", 1);
  EXPECT_EQ(
    rc::config_path_from_env("ROS2_COMMON_CPP_TEST_CONFIG", "/etc/ros2/x/config.yaml"), "/etc/ros2/x/config.yaml");
  setenv("ROS2_COMMON_CPP_TEST_CONFIG", "/tmp/y.yaml", 1);
  EXPECT_EQ(rc::config_path_from_env("ROS2_COMMON_CPP_TEST_CONFIG", "/etc/ros2/x/config.yaml"), "/tmp/y.yaml");
  unsetenv("ROS2_COMMON_CPP_TEST_CONFIG");
}

TEST(Config, LoadConfigCollectsFileErrors)
{
  EXPECT_EQ(
    rc::load_config("/nonexistent/config.yaml", validate_example).errors,
    rc::Errors{"cannot read config file /nonexistent/config.yaml"});
  const auto bad_yaml = write_temp("rate_hz: [1,\n");
  const auto parsed = rc::load_config(bad_yaml, validate_example);
  ASSERT_EQ(parsed.errors.size(), 1u);
  EXPECT_EQ(parsed.errors[0].rfind("invalid YAML: ", 0), 0u);
  const auto list_root = write_temp("- 1\n");
  EXPECT_EQ(rc::load_config(list_root, validate_example).errors, rc::Errors{"config root must be a mapping"});
  std::remove(bad_yaml.c_str());
  std::remove(list_root.c_str());
}

TEST(Config, LoadConfigEmptyFileIsAnEmptyMapping)
{
  const auto empty = write_temp("");
  const auto result = rc::load_config(empty, validate_example);
  EXPECT_TRUE(result.node.IsMap());
  EXPECT_EQ(result.errors, rc::Errors{"rate_hz: field required"});
  std::remove(empty.c_str());
}

TEST(Config, LoadConfigOrExitReturnsTheValidRoot)
{
  const auto good = write_temp("rate_hz: 2.5\n");
  setenv("ROS2_COMMON_CPP_TEST_CONFIG", good.c_str(), 1);
  const YAML::Node cfg = rc::load_config_or_exit("ROS2_COMMON_CPP_TEST_CONFIG", "/nonexistent.yaml", validate_example);
  EXPECT_DOUBLE_EQ(cfg["rate_hz"].as<double>(), 2.5);
  unsetenv("ROS2_COMMON_CPP_TEST_CONFIG");
  std::remove(good.c_str());
}

TEST(ConfigDeathTest, InvalidConfigLogsEveryErrorAndExits78)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  const auto bad = write_temp("queue: 0\nbogus: 1\n");
  setenv("ROS2_COMMON_CPP_TEST_CONFIG", bad.c_str(), 1);
  EXPECT_EXIT(
    rc::load_config_or_exit("ROS2_COMMON_CPP_TEST_CONFIG", "/nonexistent.yaml", validate_example),
    ::testing::ExitedWithCode(rc::EXIT_CONFIG),
    "bogus: extra field not permitted(.|\n)*rate_hz: field required(.|\n)*queue: must be >= 1");
  unsetenv("ROS2_COMMON_CPP_TEST_CONFIG");
  std::remove(bad.c_str());
}

TEST(ConfigDeathTest, MissingFileFromDefaultPathExits78)
{
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  unsetenv("ROS2_COMMON_CPP_TEST_CONFIG");
  EXPECT_EXIT(
    rc::load_config_or_exit("ROS2_COMMON_CPP_TEST_CONFIG", "/nonexistent/default.yaml", validate_example),
    ::testing::ExitedWithCode(78), "cannot read config file /nonexistent/default.yaml");
}
