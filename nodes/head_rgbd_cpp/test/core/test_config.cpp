#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>

#include "head_rgbd_cpp/config.hpp"

namespace
{

using head_rgbd_cpp::Config;

// Errors of a YAML document.
ros2_common_cpp::Errors errors_of(const std::string & text) { return head_rgbd_cpp::validate(YAML::Load(text)); }

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

}  // namespace

TEST(Config, DefaultsMatchTheSpecification)
{
  EXPECT_TRUE(errors_of("{}").empty());
  const Config c = head_rgbd_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_EQ(c.tof_depth_topic, "/tof/depth/image_raw");
  EXPECT_EQ(c.rgb_image_topic, "/head_camera/mapping/image_raw/compressed");
  EXPECT_EQ(c.rgb_info_topic, "/head_camera/mapping/camera_info");
  EXPECT_EQ(c.output_namespace, "/head_rgbd");
  EXPECT_EQ(c.tof_calibration_file, "/etc/ros2/head_rgbd_cpp/tof_calibration.yaml");
  EXPECT_EQ(c.output.divisor, 2);
  EXPECT_DOUBLE_EQ(c.sync.max_pair_dt_s, 0.06);
  EXPECT_EQ(c.sync.queue_size, 10);
  EXPECT_TRUE(c.motion.compensate);
  EXPECT_EQ(c.motion.fixed_frame, "odom");
  EXPECT_DOUBLE_EQ(c.motion.max_angular_rate_deg_s, 10.0);
  EXPECT_DOUBLE_EQ(c.motion.max_linear_rate_m_s, 0.3);
  EXPECT_DOUBLE_EQ(c.motion.rate_window_s, 0.05);
  EXPECT_DOUBLE_EQ(c.motion.tf_timeout_s, 0.05);
  EXPECT_DOUBLE_EQ(c.filter.jump_rel, 0.05);
  EXPECT_DOUBLE_EQ(c.filter.jump_abs_mm, 30.0);
  EXPECT_EQ(c.filter.min_valid_neighbours, 3);
  EXPECT_DOUBLE_EQ(c.depth.min_m, 0.1);
  EXPECT_DOUBLE_EQ(c.depth.max_m, 4.0);
  EXPECT_FALSE(c.upsampling.enabled);
  EXPECT_EQ(c.upsampling.radius, 4);
  EXPECT_DOUBLE_EQ(c.upsampling.eps, 0.001);
  EXPECT_DOUBLE_EQ(c.upsampling.min_support, 0.5);
  EXPECT_TRUE(c.cloud.enabled);
  EXPECT_DOUBLE_EQ(c.cloud.rate_hz, 1.0);
  EXPECT_DOUBLE_EQ(c.lazy.idle_unsubscribe_s, 5.0);
  ASSERT_TRUE(c.metrics_port.has_value());
  EXPECT_EQ(*c.metrics_port, head_rgbd_cpp::DEFAULT_METRICS_PORT);
}

TEST(Config, ShippedYamlLoadsAndEqualsTheDefaults)
{
  const YAML::Node node = YAML::LoadFile(std::string(HEAD_RGBD_CONFIG_DIR) + "/head_rgbd.yaml");
  const auto errors = head_rgbd_cpp::validate(node);
  EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors.front());
  const Config shipped = head_rgbd_cpp::from_yaml(node);
  const Config defaults;
  EXPECT_EQ(shipped.tof_depth_topic, defaults.tof_depth_topic);
  EXPECT_EQ(shipped.rgb_image_topic, defaults.rgb_image_topic);
  EXPECT_EQ(shipped.rgb_info_topic, defaults.rgb_info_topic);
  EXPECT_EQ(shipped.output_namespace, defaults.output_namespace);
  EXPECT_EQ(shipped.tof_calibration_file, defaults.tof_calibration_file);
  EXPECT_EQ(shipped.output.divisor, defaults.output.divisor);
  EXPECT_DOUBLE_EQ(shipped.sync.max_pair_dt_s, defaults.sync.max_pair_dt_s);
  EXPECT_EQ(shipped.sync.queue_size, defaults.sync.queue_size);
  EXPECT_EQ(shipped.motion.compensate, defaults.motion.compensate);
  EXPECT_EQ(shipped.motion.fixed_frame, defaults.motion.fixed_frame);
  EXPECT_DOUBLE_EQ(shipped.motion.max_angular_rate_deg_s, defaults.motion.max_angular_rate_deg_s);
  EXPECT_DOUBLE_EQ(shipped.motion.max_linear_rate_m_s, defaults.motion.max_linear_rate_m_s);
  EXPECT_DOUBLE_EQ(shipped.motion.rate_window_s, defaults.motion.rate_window_s);
  EXPECT_DOUBLE_EQ(shipped.motion.tf_timeout_s, defaults.motion.tf_timeout_s);
  EXPECT_DOUBLE_EQ(shipped.filter.jump_rel, defaults.filter.jump_rel);
  EXPECT_DOUBLE_EQ(shipped.filter.jump_abs_mm, defaults.filter.jump_abs_mm);
  EXPECT_EQ(shipped.filter.min_valid_neighbours, defaults.filter.min_valid_neighbours);
  EXPECT_DOUBLE_EQ(shipped.depth.min_m, defaults.depth.min_m);
  EXPECT_DOUBLE_EQ(shipped.depth.max_m, defaults.depth.max_m);
  EXPECT_EQ(shipped.upsampling.enabled, defaults.upsampling.enabled);
  EXPECT_EQ(shipped.upsampling.radius, defaults.upsampling.radius);
  EXPECT_DOUBLE_EQ(shipped.upsampling.eps, defaults.upsampling.eps);
  EXPECT_DOUBLE_EQ(shipped.upsampling.min_support, defaults.upsampling.min_support);
  EXPECT_EQ(shipped.cloud.enabled, defaults.cloud.enabled);
  EXPECT_DOUBLE_EQ(shipped.cloud.rate_hz, defaults.cloud.rate_hz);
  EXPECT_DOUBLE_EQ(shipped.lazy.idle_unsubscribe_s, defaults.lazy.idle_unsubscribe_s);
  EXPECT_EQ(shipped.metrics_port, defaults.metrics_port);
}

TEST(Config, ValidOverridesAreRead)
{
  const Config c =
    head_rgbd_cpp::from_yaml(YAML::Load("output: {divisor: 1}\n"
                                        "cloud: {enabled: false, rate_hz: 10}\n"
                                        "upsampling: {enabled: true, radius: 16, min_support: 1}\n"
                                        "filter: {min_valid_neighbours: 0}\n"
                                        "lazy: {idle_unsubscribe_s: 0}\n"
                                        "motion: {tf_timeout_s: 0}\n"
                                        "metrics_port: null\n"));
  EXPECT_EQ(c.output.divisor, 1);
  EXPECT_FALSE(c.cloud.enabled);
  EXPECT_DOUBLE_EQ(c.cloud.rate_hz, 10.0);
  EXPECT_TRUE(c.upsampling.enabled);
  EXPECT_EQ(c.upsampling.radius, 16);
  EXPECT_EQ(c.filter.min_valid_neighbours, 0);
  EXPECT_FALSE(c.metrics_port.has_value());
  EXPECT_TRUE(
    errors_of("{output: {divisor: 1}, cloud: {rate_hz: 10}, upsampling: {radius: 16, min_support: 1}}").empty());
}

TEST(Config, EveryRangeIsRejectedWithTheKeyNamed)
{
  expect_rejected("output: {divisor: 3}", "output.divisor");
  expect_rejected("output: {divisor: 0}", "output.divisor");
  expect_rejected("sync: {max_pair_dt_s: 0}", "sync.max_pair_dt_s");
  expect_rejected("sync: {max_pair_dt_s: 0.21}", "sync.max_pair_dt_s");
  expect_rejected("sync: {queue_size: 1}", "sync.queue_size");
  expect_rejected("motion: {max_angular_rate_deg_s: 0}", "motion.max_angular_rate_deg_s");
  expect_rejected("motion: {max_linear_rate_m_s: -1}", "motion.max_linear_rate_m_s");
  expect_rejected("motion: {rate_window_s: 0}", "motion.rate_window_s");
  expect_rejected("motion: {tf_timeout_s: -0.1}", "motion.tf_timeout_s");
  expect_rejected("motion: {compensate: maybe}", "motion.compensate");
  expect_rejected("filter: {jump_rel: 0}", "filter.jump_rel");
  expect_rejected("filter: {jump_rel: 1}", "filter.jump_rel");
  expect_rejected("filter: {jump_abs_mm: -1}", "filter.jump_abs_mm");
  expect_rejected("filter: {min_valid_neighbours: -1}", "filter.min_valid_neighbours");
  expect_rejected("filter: {min_valid_neighbours: 9}", "filter.min_valid_neighbours");
  expect_rejected("upsampling: {radius: 0}", "upsampling.radius");
  expect_rejected("upsampling: {radius: 17}", "upsampling.radius");
  expect_rejected("upsampling: {eps: 0}", "upsampling.eps");
  expect_rejected("upsampling: {min_support: 0}", "upsampling.min_support");
  expect_rejected("upsampling: {min_support: 1.1}", "upsampling.min_support");
  expect_rejected("cloud: {rate_hz: 0}", "cloud.rate_hz");
  expect_rejected("cloud: {rate_hz: 10.5}", "cloud.rate_hz");
  expect_rejected("lazy: {idle_unsubscribe_s: -1}", "lazy.idle_unsubscribe_s");
  expect_rejected("metrics_port: 0", "metrics_port");
  expect_rejected("metrics_port: 65536", "metrics_port");
  expect_rejected("metrics_port: abc", "metrics_port");
  expect_rejected("depth: {min_m: 0}", "depth.min_m");
  expect_rejected("tof_depth_topic: [a]", "tof_depth_topic");
}

TEST(Config, MinDepthAtOrAboveMaxDepthIsRejected)
{
  expect_rejected("depth: {min_m: 4.0, max_m: 4.0}", "depth.max_m");
  expect_rejected("depth: {min_m: 5.0}", "depth.max_m");
  expect_rejected("depth: {max_m: 0.05}", "depth.max_m");
  EXPECT_TRUE(errors_of("depth: {min_m: 0.5, max_m: 0.6}").empty());
}

TEST(Config, UnknownKeysAreRejectedAtEveryLevel)
{
  expect_rejected("bogus: 1", "bogus");
  expect_rejected("output: {divisor: 2, bogus: 1}", "output.bogus");
  expect_rejected("sync: {bogus: 1}", "sync.bogus");
  expect_rejected("motion: {bogus: 1}", "motion.bogus");
  expect_rejected("filter: {bogus: 1}", "filter.bogus");
  expect_rejected("depth: {bogus: 1}", "depth.bogus");
  expect_rejected("upsampling: {bogus: 1}", "upsampling.bogus");
  expect_rejected("cloud: {bogus: 1}", "cloud.bogus");
  expect_rejected("lazy: {bogus: 1}", "lazy.bogus");
  expect_rejected("cloud: 3", "cloud");
}

TEST(Config, DerivedTopicsFollowTheNamespace)
{
  Config c;
  EXPECT_EQ(c.depth_image(), "/head_rgbd/depth/image_raw");
  EXPECT_EQ(c.depth_info(), "/head_rgbd/depth/camera_info");
  EXPECT_EQ(c.rgb_image(), "/head_rgbd/rgb/image_raw/compressed");
  EXPECT_EQ(c.rgb_camera_info(), "/head_rgbd/rgb/camera_info");
  EXPECT_EQ(c.points(), "/head_rgbd/points");
  EXPECT_EQ(c.depth_upsampled_image(), "/head_rgbd/depth_upsampled/image_raw");
  EXPECT_EQ(c.depth_upsampled_info(), "/head_rgbd/depth_upsampled/camera_info");
  c.output_namespace = "/other";
  EXPECT_EQ(c.points(), "/other/points");
}
