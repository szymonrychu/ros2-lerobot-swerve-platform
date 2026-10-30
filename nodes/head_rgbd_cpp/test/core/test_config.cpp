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
  EXPECT_EQ(c.depth_topic, "/head/d435/aligned_depth_to_color/image_raw");
  EXPECT_EQ(c.rgb_image_topic, "/head/d435/color/image_raw");
  EXPECT_EQ(c.rgb_info_topic, "/head/d435/color/camera_info");
  EXPECT_EQ(c.output_namespace, "/head_rgbd");
  EXPECT_EQ(c.output.divisor, 2);
  EXPECT_DOUBLE_EQ(c.output.rate_hz, 2.0);
  EXPECT_EQ(c.output.jpeg_quality, 80);
  EXPECT_DOUBLE_EQ(c.depth.min_m, 0.15);
  EXPECT_DOUBLE_EQ(c.depth.max_m, 4.0);
  EXPECT_TRUE(c.cloud.enabled);
  EXPECT_DOUBLE_EQ(c.cloud.rate_hz, 1.0);
  EXPECT_DOUBLE_EQ(c.lazy.idle_unsubscribe_s, 5.0);
  ASSERT_TRUE(c.metrics_port.has_value());
  EXPECT_EQ(*c.metrics_port, head_rgbd_cpp::DEFAULT_METRICS_PORT);
  EXPECT_TRUE(c.edge_mask.enabled);
  EXPECT_EQ(c.edge_mask.window_px, 3);
  EXPECT_DOUBLE_EQ(c.edge_mask.abs_m, 0.01);
  EXPECT_DOUBLE_EQ(c.edge_mask.rel, 0.025);
  EXPECT_EQ(c.edge_mask.grow_px, 0);
  EXPECT_TRUE(c.filtered_depth.enabled);
  EXPECT_EQ(c.filtered_depth.depth_topic, "/head/d435/depth/image_rect_raw");
  EXPECT_EQ(c.filtered_depth.info_topic, "/head/d435/depth/camera_info");
  EXPECT_EQ(c.filtered_depth.output_image_topic, "/head/d435/depth_filtered/image_raw");
  EXPECT_EQ(c.filtered_depth.output_info_topic, "/head/d435/depth_filtered/camera_info");
  EXPECT_EQ(c.pairing.policy, head_rgbd_cpp::PairingPolicy::EXACT);
  EXPECT_DOUBLE_EQ(c.pairing.max_interval_s, 0.02);
}

TEST(Config, ShippedYamlLoadsAndEqualsTheDefaults)
{
  const YAML::Node node = YAML::LoadFile(std::string(HEAD_RGBD_CONFIG_DIR) + "/head_rgbd.yaml");
  const auto errors = head_rgbd_cpp::validate(node);
  EXPECT_TRUE(errors.empty()) << (errors.empty() ? "" : errors.front());
  const Config shipped = head_rgbd_cpp::from_yaml(node);
  const Config defaults;
  EXPECT_EQ(shipped.depth_topic, defaults.depth_topic);
  EXPECT_EQ(shipped.rgb_image_topic, defaults.rgb_image_topic);
  EXPECT_EQ(shipped.rgb_info_topic, defaults.rgb_info_topic);
  EXPECT_EQ(shipped.output_namespace, defaults.output_namespace);
  EXPECT_EQ(shipped.output.divisor, defaults.output.divisor);
  EXPECT_DOUBLE_EQ(shipped.output.rate_hz, defaults.output.rate_hz);
  EXPECT_EQ(shipped.output.jpeg_quality, defaults.output.jpeg_quality);
  EXPECT_DOUBLE_EQ(shipped.depth.min_m, defaults.depth.min_m);
  EXPECT_DOUBLE_EQ(shipped.depth.max_m, defaults.depth.max_m);
  EXPECT_EQ(shipped.cloud.enabled, defaults.cloud.enabled);
  EXPECT_DOUBLE_EQ(shipped.cloud.rate_hz, defaults.cloud.rate_hz);
  EXPECT_DOUBLE_EQ(shipped.lazy.idle_unsubscribe_s, defaults.lazy.idle_unsubscribe_s);
  EXPECT_EQ(shipped.metrics_port, defaults.metrics_port);
  EXPECT_EQ(shipped.edge_mask.enabled, defaults.edge_mask.enabled);
  EXPECT_EQ(shipped.edge_mask.window_px, defaults.edge_mask.window_px);
  EXPECT_DOUBLE_EQ(shipped.edge_mask.abs_m, defaults.edge_mask.abs_m);
  EXPECT_DOUBLE_EQ(shipped.edge_mask.rel, defaults.edge_mask.rel);
  EXPECT_EQ(shipped.edge_mask.grow_px, defaults.edge_mask.grow_px);
  EXPECT_EQ(shipped.filtered_depth.enabled, defaults.filtered_depth.enabled);
  EXPECT_EQ(shipped.filtered_depth.depth_topic, defaults.filtered_depth.depth_topic);
  EXPECT_EQ(shipped.filtered_depth.info_topic, defaults.filtered_depth.info_topic);
  EXPECT_EQ(shipped.filtered_depth.output_image_topic, defaults.filtered_depth.output_image_topic);
  EXPECT_EQ(shipped.filtered_depth.output_info_topic, defaults.filtered_depth.output_info_topic);
  EXPECT_EQ(shipped.pairing.policy, defaults.pairing.policy);
  EXPECT_DOUBLE_EQ(shipped.pairing.max_interval_s, defaults.pairing.max_interval_s);
}

TEST(Config, ValidOverridesAreRead)
{
  const char * text =
    "depth_topic: /d\n"
    "rgb_image_topic: /c\n"
    "rgb_info_topic: /i\n"
    "output_namespace: /o\n"
    "output: {divisor: 1, rate_hz: 15, jpeg_quality: 100}\n"
    "depth: {min_m: 0.3, max_m: 2.5}\n"
    "cloud: {enabled: false, rate_hz: 10}\n"
    "lazy: {idle_unsubscribe_s: 0}\n"
    "edge_mask: {enabled: false, window_px: 5, abs_m: 0, rel: 0.05, grow_px: 1}\n"
    "filtered_depth: {enabled: false, depth_topic: /fd, info_topic: /fi, output_image_topic: /oi, "
    "output_info_topic: /oc}\n"
    "pairing: {policy: approximate, max_interval_s: 0.01}\n"
    "metrics_port: null\n";
  EXPECT_TRUE(errors_of(text).empty());
  const Config c = head_rgbd_cpp::from_yaml(YAML::Load(text));
  EXPECT_EQ(c.depth_topic, "/d");
  EXPECT_EQ(c.rgb_image_topic, "/c");
  EXPECT_EQ(c.rgb_info_topic, "/i");
  EXPECT_EQ(c.output_namespace, "/o");
  EXPECT_EQ(c.output.divisor, 1);
  EXPECT_DOUBLE_EQ(c.output.rate_hz, 15.0);
  EXPECT_EQ(c.output.jpeg_quality, 100);
  EXPECT_DOUBLE_EQ(c.depth.min_m, 0.3);
  EXPECT_DOUBLE_EQ(c.depth.max_m, 2.5);
  EXPECT_FALSE(c.cloud.enabled);
  EXPECT_DOUBLE_EQ(c.cloud.rate_hz, 10.0);
  EXPECT_DOUBLE_EQ(c.lazy.idle_unsubscribe_s, 0.0);
  EXPECT_FALSE(c.metrics_port.has_value());
  EXPECT_FALSE(c.edge_mask.enabled);
  EXPECT_EQ(c.edge_mask.window_px, 5);
  EXPECT_DOUBLE_EQ(c.edge_mask.abs_m, 0.0);
  EXPECT_DOUBLE_EQ(c.edge_mask.rel, 0.05);
  EXPECT_EQ(c.edge_mask.grow_px, 1);
  EXPECT_FALSE(c.filtered_depth.enabled);
  EXPECT_EQ(c.filtered_depth.depth_topic, "/fd");
  EXPECT_EQ(c.filtered_depth.info_topic, "/fi");
  EXPECT_EQ(c.filtered_depth.output_image_topic, "/oi");
  EXPECT_EQ(c.filtered_depth.output_info_topic, "/oc");
  EXPECT_EQ(c.pairing.policy, head_rgbd_cpp::PairingPolicy::APPROXIMATE);
  EXPECT_DOUBLE_EQ(c.pairing.max_interval_s, 0.01);
  EXPECT_TRUE(errors_of("output: {jpeg_quality: 1}").empty());
  EXPECT_TRUE(errors_of("pairing: {policy: exact, max_interval_s: 0.0333}").empty());
}

TEST(Config, EveryRangeIsRejectedWithTheKeyNamed)
{
  expect_rejected("output: {divisor: 3}", "output.divisor");
  expect_rejected("output: {divisor: 0}", "output.divisor");
  expect_rejected("output: {rate_hz: 0}", "output.rate_hz");
  expect_rejected("output: {rate_hz: -1}", "output.rate_hz");
  expect_rejected("output: {rate_hz: 15.5}", "output.rate_hz");
  expect_rejected("output: {jpeg_quality: 0}", "output.jpeg_quality");
  expect_rejected("output: {jpeg_quality: 101}", "output.jpeg_quality");
  expect_rejected("cloud: {rate_hz: 0}", "cloud.rate_hz");
  expect_rejected("cloud: {rate_hz: 10.5}", "cloud.rate_hz");
  expect_rejected("cloud: {enabled: maybe}", "cloud.enabled");
  expect_rejected("lazy: {idle_unsubscribe_s: -1}", "lazy.idle_unsubscribe_s");
  expect_rejected("metrics_port: 0", "metrics_port");
  expect_rejected("metrics_port: 65536", "metrics_port");
  expect_rejected("metrics_port: abc", "metrics_port");
  expect_rejected("depth: {min_m: 0}", "depth.min_m");
  expect_rejected("depth_topic: [a]", "depth_topic");
  expect_rejected("rgb_image_topic: [a]", "rgb_image_topic");
  expect_rejected("rgb_info_topic: [a]", "rgb_info_topic");
  expect_rejected("edge_mask: {enabled: maybe}", "edge_mask.enabled");
  expect_rejected("edge_mask: {window_px: 4}", "edge_mask.window_px");
  expect_rejected("edge_mask: {window_px: 1}", "edge_mask.window_px");
  expect_rejected("edge_mask: {window_px: 11}", "edge_mask.window_px");
  expect_rejected("edge_mask: {abs_m: -0.01}", "edge_mask.abs_m");
  expect_rejected("edge_mask: {rel: -0.01}", "edge_mask.rel");
  expect_rejected("edge_mask: {rel: 1.5}", "edge_mask.rel");
  expect_rejected("edge_mask: {abs_m: 0, rel: 0}", "edge_mask.rel");
  expect_rejected("edge_mask: {grow_px: -1}", "edge_mask.grow_px");
  expect_rejected("edge_mask: {grow_px: 3}", "edge_mask.grow_px");
  expect_rejected("filtered_depth: {enabled: maybe}", "filtered_depth.enabled");
  expect_rejected("filtered_depth: {depth_topic: [a]}", "filtered_depth.depth_topic");
  expect_rejected("filtered_depth: {info_topic: [a]}", "filtered_depth.info_topic");
  expect_rejected("filtered_depth: {output_image_topic: [a]}", "filtered_depth.output_image_topic");
  expect_rejected("filtered_depth: {output_info_topic: [a]}", "filtered_depth.output_info_topic");
  expect_rejected("filtered_depth: {output_namespace: /x}", "filtered_depth.output_namespace");
  expect_rejected("pairing: {policy: nearest}", "pairing.policy");
  expect_rejected("pairing: {max_interval_s: 0}", "pairing.max_interval_s");
  // Half a frame period of the 15 Hz D435 at most: a wider window could pair a colour frame with the next depth.
  expect_rejected("pairing: {max_interval_s: 0.034}", "pairing.max_interval_s");
}

TEST(Config, MinDepthAtOrAboveMaxDepthIsRejected)
{
  expect_rejected("depth: {min_m: 4.0, max_m: 4.0}", "depth.max_m");
  expect_rejected("depth: {min_m: 5.0}", "depth.max_m");
  expect_rejected("depth: {max_m: 0.1}", "depth.max_m");
  EXPECT_TRUE(errors_of("depth: {min_m: 0.5, max_m: 0.6}").empty());
}

TEST(Config, UnknownKeysAreRejectedAtEveryLevel)
{
  expect_rejected("bogus: 1", "bogus");
  expect_rejected("output: {divisor: 2, bogus: 1}", "output.bogus");
  expect_rejected("depth: {bogus: 1}", "depth.bogus");
  expect_rejected("cloud: {bogus: 1}", "cloud.bogus");
  expect_rejected("lazy: {bogus: 1}", "lazy.bogus");
  expect_rejected("cloud: 3", "cloud");
  expect_rejected("edge_mask: {bogus: 1}", "edge_mask.bogus");
  expect_rejected("filtered_depth: {bogus: 1}", "filtered_depth.bogus");
  expect_rejected("pairing: {bogus: 1}", "pairing.bogus");
}

TEST(Config, RemovedTofKeysAreRejected)
{
  expect_rejected("tof_depth_topic: /tof/depth/image_raw", "tof_depth_topic");
  expect_rejected("tof_calibration_file: /etc/ros2/head_rgbd_cpp/tof_calibration.yaml", "tof_calibration_file");
  expect_rejected("sync: {max_pair_dt_s: 0.06, queue_size: 10}", "sync");
  expect_rejected("motion: {compensate: true}", "motion");
  expect_rejected("filter: {jump_rel: 0.05}", "filter");
  expect_rejected("upsampling: {enabled: true}", "upsampling");
}

TEST(Config, DerivedTopicsFollowTheNamespace)
{
  Config c;
  EXPECT_EQ(c.depth_image(), "/head_rgbd/depth/image_raw");
  EXPECT_EQ(c.depth_info(), "/head_rgbd/depth/camera_info");
  EXPECT_EQ(c.rgb_image(), "/head_rgbd/rgb/image_raw/compressed");
  EXPECT_EQ(c.rgb_camera_info(), "/head_rgbd/rgb/camera_info");
  EXPECT_EQ(c.points(), "/head_rgbd/points");
  c.output_namespace = "/other";
  EXPECT_EQ(c.points(), "/other/points");
}
