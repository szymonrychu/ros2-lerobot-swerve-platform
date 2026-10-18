#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>

#include "head_camera_cpp/config.hpp"

namespace
{

using head_camera_cpp::Config;

// Errors of a YAML document.
ros2_common_cpp::Errors errors_of(const std::string & text) { return head_camera_cpp::validate(YAML::Load(text)); }

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
  const Config c = head_camera_cpp::from_yaml(YAML::Load("{}"));
  EXPECT_EQ(c.camera_id, "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a");
  EXPECT_EQ(c.frame_id, "head_camera_optical_frame");
  EXPECT_EQ(c.topic_namespace, "/head_camera");
  EXPECT_EQ(c.sensor_mode[0], 2304);
  EXPECT_EQ(c.sensor_mode[1], 1296);
  EXPECT_EQ(c.width, 1152);
  EXPECT_EQ(c.height, 648);
  EXPECT_EQ(c.frame_duration_us, 33333);
  EXPECT_EQ(c.af_mode, "continuous");
  EXPECT_EQ(c.jpeg_quality, 70);
  EXPECT_FALSE(c.camera_info_file.has_value());
  ASSERT_TRUE(c.metrics_port.has_value());
  EXPECT_EQ(*c.metrics_port, 19113);
  EXPECT_EQ(c.zoom.still_size[0], 4608);
  EXPECT_EQ(c.zoom.still_size[1], 2592);
  EXPECT_EQ(c.zoom.factors, (std::vector<int>{2, 4, 8}));
  EXPECT_EQ(c.zoom.default_max_px, 1024);
  EXPECT_EQ(c.zoom.min_max_px, 256);
  EXPECT_EQ(c.zoom.max_max_px, 1024);
  EXPECT_EQ(c.zoom.jpeg_quality, 85);
  EXPECT_DOUBLE_EQ(c.zoom.af_timeout_s, 2.5);
  EXPECT_DOUBLE_EQ(c.zoom.switch_timeout_s, 1.5);
  EXPECT_EQ(c.zoom.settle_frames, 2);
  EXPECT_EQ(c.zoom.still_buffers, 2);
  EXPECT_DOUBLE_EQ(c.zoom.watchdog_s, 6.0);
  EXPECT_DOUBLE_EQ(c.zoom.restore_retry_delay_s, 0.5);
  EXPECT_DOUBLE_EQ(c.zoom.af_window_fraction, 0.125);
}

TEST(Config, TopicsAreDerivedFromTheNamespace)
{
  Config c;
  EXPECT_EQ(c.image_raw(), "/head_camera/image_raw");
  EXPECT_EQ(c.image_raw_compressed(), "/head_camera/image_raw/compressed");
  EXPECT_EQ(c.camera_info(), "/head_camera/camera_info");
  EXPECT_EQ(c.zoom_command(), "/head_camera/zoom/command");
  EXPECT_EQ(c.zoom_image(), "/head_camera/zoom/image");
  EXPECT_EQ(c.zoom_result(), "/head_camera/zoom/result");
  c.topic_namespace = "/other";
  EXPECT_EQ(c.image_raw(), "/other/image_raw");
}

TEST(Config, CompleteValidYamlRoundTrips)
{
  const std::string text = R"(
camera_id: /some/camera
frame_id: cam_frame
topic_namespace: /cam
sensor_mode: [4608, 2592]
width: 1536
height: 864
frame_duration_us: 66666
af_mode: auto
jpeg_quality: 55
camera_info_file: /etc/ros2/head_camera_cpp/info.yaml
metrics_port: 19200
zoom:
  still_size: [4608, 2592]
  factors: [2, 8]
  default_max_px: 800
  min_max_px: 300
  max_max_px: 900
  jpeg_quality: 90
  af_timeout_s: 1.0
  switch_timeout_s: 2.0
  settle_frames: 0
  still_buffers: 3
  watchdog_s: 10.0
  restore_retry_delay_s: 0.0
  af_window_fraction: 1.0
)";
  EXPECT_TRUE(errors_of(text).empty());
  const Config c = head_camera_cpp::from_yaml(YAML::Load(text));
  EXPECT_EQ(c.camera_id, "/some/camera");
  EXPECT_EQ(c.frame_id, "cam_frame");
  EXPECT_EQ(c.topic_namespace, "/cam");
  EXPECT_EQ(c.sensor_mode[0], 4608);
  EXPECT_EQ(c.width, 1536);
  EXPECT_EQ(c.height, 864);
  EXPECT_EQ(c.frame_duration_us, 66666);
  EXPECT_EQ(c.af_mode, "auto");
  EXPECT_EQ(c.jpeg_quality, 55);
  EXPECT_EQ(c.camera_info_file.value_or(""), "/etc/ros2/head_camera_cpp/info.yaml");
  EXPECT_EQ(c.metrics_port.value_or(0), 19200);
  EXPECT_EQ(c.zoom.factors, (std::vector<int>{2, 8}));
  EXPECT_EQ(c.zoom.default_max_px, 800);
  EXPECT_EQ(c.zoom.min_max_px, 300);
  EXPECT_EQ(c.zoom.max_max_px, 900);
  EXPECT_EQ(c.zoom.jpeg_quality, 90);
  EXPECT_DOUBLE_EQ(c.zoom.af_timeout_s, 1.0);
  EXPECT_DOUBLE_EQ(c.zoom.switch_timeout_s, 2.0);
  EXPECT_EQ(c.zoom.settle_frames, 0);
  EXPECT_EQ(c.zoom.still_buffers, 3);
  EXPECT_DOUBLE_EQ(c.zoom.watchdog_s, 10.0);
  EXPECT_DOUBLE_EQ(c.zoom.restore_retry_delay_s, 0.0);
  EXPECT_DOUBLE_EQ(c.zoom.af_window_fraction, 1.0);
}

TEST(Config, NullMetricsPortDisablesMetrics)
{
  EXPECT_TRUE(errors_of("metrics_port: null").empty());
  EXPECT_FALSE(head_camera_cpp::from_yaml(YAML::Load("metrics_port: null")).metrics_port.has_value());
}

TEST(Config, RejectsUnknownKeys)
{
  expect_rejected("bogus: 1", "bogus");
  expect_rejected("zoom: {bogus: 1}", "zoom.bogus");
}

TEST(Config, RejectsBadJpegQuality)
{
  expect_rejected("jpeg_quality: 0", "jpeg_quality");
  expect_rejected("jpeg_quality: 101", "jpeg_quality");
  expect_rejected("zoom: {jpeg_quality: 0}", "zoom.jpeg_quality");
  expect_rejected("zoom: {jpeg_quality: 101}", "zoom.jpeg_quality");
}

TEST(Config, RejectsBadGeometry)
{
  expect_rejected("width: 1151", "width");
  expect_rejected("height: 647", "height");
  expect_rejected("width: 0", "width");
  expect_rejected("width: 1280\nheight: 648", "height");  // aspect mismatch
  expect_rejected("sensor_mode: [2304]", "sensor_mode");
  expect_rejected("sensor_mode: [0, 1296]", "sensor_mode");
  expect_rejected("frame_duration_us: 0", "frame_duration_us");
  expect_rejected("af_mode: sometimes", "af_mode");
  expect_rejected("metrics_port: 0", "metrics_port");
  expect_rejected("metrics_port: 65536", "metrics_port");
  expect_rejected("zoom: {still_size: [4608]}", "zoom.still_size");
}

TEST(Config, AspectWithinOnePixelIsAccepted)
{
  EXPECT_TRUE(errors_of("width: 1000\nheight: 562").empty());  // 562.5 expected
  EXPECT_TRUE(errors_of("width: 1280\nheight: 720").empty());
}

TEST(Config, RejectsBadZoomFactors)
{
  expect_rejected("zoom: {factors: [3]}", "zoom.factors");
  expect_rejected("zoom: {factors: [2, 4, 16]}", "zoom.factors");
  expect_rejected("zoom: {factors: []}", "zoom.factors");
}

TEST(Config, RejectsBadMaxPx)
{
  expect_rejected("zoom: {default_max_px: 1025, max_max_px: 1024}", "zoom.default_max_px");
  expect_rejected("zoom: {max_max_px: 2048}", "zoom.max_max_px");
  expect_rejected("zoom: {min_max_px: 512, default_max_px: 256}", "zoom.min_max_px");
}

TEST(Config, RejectsBadTimeoutsAndWatchdog)
{
  expect_rejected("zoom: {af_timeout_s: 0}", "zoom.af_timeout_s");
  expect_rejected("zoom: {af_timeout_s: -1}", "zoom.af_timeout_s");
  expect_rejected("zoom: {switch_timeout_s: 0}", "zoom.switch_timeout_s");
  expect_rejected("zoom: {switch_timeout_s: -2.0}", "zoom.switch_timeout_s");
  expect_rejected("zoom: {watchdog_s: 4.0}", "zoom.watchdog_s");  // equal to 2.5 + 1.5
  expect_rejected("zoom: {watchdog_s: 3.0}", "zoom.watchdog_s");
  expect_rejected("zoom: {settle_frames: -1}", "zoom.settle_frames");
  expect_rejected("zoom: {still_buffers: 0}", "zoom.still_buffers");
  expect_rejected("zoom: {restore_retry_delay_s: -0.1}", "zoom.restore_retry_delay_s");
}

TEST(Config, RejectsBadAfWindowFraction)
{
  expect_rejected("zoom: {af_window_fraction: 0}", "zoom.af_window_fraction");
  expect_rejected("zoom: {af_window_fraction: 1.5}", "zoom.af_window_fraction");
}

TEST(Config, RejectsWrongTypesAndNonMappings)
{
  expect_rejected("width: wide", "width");
  expect_rejected("zoom: 3", "zoom");
  expect_rejected("[1, 2]", "config root");
}

TEST(Config, EveryErrorLineNamesItsKey)
{
  const auto errors = errors_of("jpeg_quality: 0\nwidth: 1151\nzoom: {factors: [3], af_window_fraction: 0, nope: 1}");
  ASSERT_GE(errors.size(), 4U);
  for (const auto & line : errors) {
    const auto colon = line.find(": ");
    ASSERT_NE(colon, std::string::npos) << line;
    EXPECT_GT(colon, 0U) << line;
    EXPECT_EQ(line.substr(0, colon).find(' '), std::string::npos) << line;  // the key, not prose
  }
}

TEST(ConfigDeathTest, LoadExitsWith78OnMissingFile)
{
  setenv("HEAD_CAMERA_CONFIG", "/nonexistent/head_camera_config.yaml", 1);
  EXPECT_EXIT(head_camera_cpp::load_config_or_exit(), ::testing::ExitedWithCode(78), "");
}
