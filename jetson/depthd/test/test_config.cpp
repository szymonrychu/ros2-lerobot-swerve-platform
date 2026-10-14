#include <gtest/gtest.h>

#include <string>

#include "depthd/config.hpp"

using depthd::Config;

namespace
{
const std::string kDefaultFile = std::string(DEPTHD_SOURCE_DIR) + "/config/depthd.yaml";

// Parses `yaml` and expects a rejection whose message contains `key`.
void expect_rejected(const std::string & yaml, const std::string & key)
{
  const auto r = depthd::parse_config(yaml);
  ASSERT_FALSE(r.ok()) << "accepted: " << yaml;
  EXPECT_NE(r.error.find(key), std::string::npos) << "error '" << r.error << "' does not name " << key;
}
}  // namespace

TEST(Config, DefaultFileLoadsAndEqualsStructDefaults)
{
  const auto r = depthd::load_config(kDefaultFile);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.config, Config{});
}

TEST(Config, StructDefaultsMatchTheSpec)
{
  const Config c;
  EXPECT_EQ(c.capture.left_sensor_id, 0);
  EXPECT_EQ(c.capture.right_sensor_id, 1);
  EXPECT_EQ(c.capture.sensor_mode.width, 1640);
  EXPECT_EQ(c.capture.sensor_mode.height, 1232);
  EXPECT_EQ(c.capture.sensor_mode.fps, 30);
  EXPECT_EQ(c.capture.settle_frames, 30);
  EXPECT_TRUE(c.capture.lock_exposure_after_settle);
  EXPECT_DOUBLE_EQ(c.capture.pair_window_ms, 8.0);
  EXPECT_TRUE(c.capture.zero_copy);
  EXPECT_EQ(c.processing.width, 640);
  EXPECT_EQ(c.processing.height, 480);
  EXPECT_EQ(c.processing.matcher, "cuda_sgm");
  EXPECT_EQ(c.processing.num_disparities, 128);
  EXPECT_EQ(c.processing.p1, 10);
  EXPECT_EQ(c.processing.p2, 120);
  EXPECT_EQ(c.processing.uniqueness_ratio, 5);
  EXPECT_DOUBLE_EQ(c.processing.min_depth_m, 0.2);
  EXPECT_DOUBLE_EQ(c.processing.max_depth_m, 6.0);
  EXPECT_TRUE(c.postprocess.lr_check.enabled);
  EXPECT_EQ(c.postprocess.lr_check.max_diff, 1);
  EXPECT_TRUE(c.postprocess.confidence.enabled);
  EXPECT_TRUE(c.postprocess.speckle.enabled);
  EXPECT_EQ(c.postprocess.speckle.max_size, 200);
  EXPECT_EQ(c.postprocess.speckle.max_diff, 1);
  EXPECT_TRUE(c.postprocess.bilateral.enabled);
  EXPECT_TRUE(c.postprocess.hole_fill.enabled);
  EXPECT_EQ(c.postprocess.hole_fill.max_hole_px, 64);
  EXPECT_TRUE(c.postprocess.temporal.enabled);
  EXPECT_DOUBLE_EQ(c.postprocess.temporal.alpha, 0.4);
  EXPECT_EQ(c.postprocess.temporal.motion_threshold, 12);
  EXPECT_FALSE(c.postprocess.wls.enabled);
  EXPECT_TRUE(c.mono.enabled);
  EXPECT_EQ(c.mono.model, "dav2_small");
  EXPECT_EQ(c.mono.engine_path, "/var/lib/depthd/models/dav2_small_266.engine");
  EXPECT_DOUBLE_EQ(c.mono.rate_hz, 2.0);
  EXPECT_TRUE(c.rgb.enabled);
  EXPECT_EQ(c.rgb.width, 640);
  EXPECT_EQ(c.rgb.height, 480);
  EXPECT_EQ(c.rgb.jpeg_quality, 80);
  EXPECT_DOUBLE_EQ(c.rgb.fps, 10.0);
  EXPECT_EQ(c.transport.bind_address, "10.77.0.2");
  EXPECT_EQ(c.transport.pub_port, 5550);
  EXPECT_EQ(c.transport.http_port, 8090);
  EXPECT_EQ(c.transport.zstd_level, 1);
  EXPECT_EQ(c.transport.send_hwm, 2);
  EXPECT_EQ(c.calibration.path, "/var/lib/depthd/stereo.yaml");
  EXPECT_DOUBLE_EQ(c.calibration.max_rms_px, 0.5);
  EXPECT_EQ(c.frame_ids.depth, "stereo_left_optical_frame");
  EXPECT_EQ(c.frame_ids.rgb, "overview_camera_optical_frame");
}

TEST(Config, EmptyDocumentGivesDefaults)
{
  const auto r = depthd::parse_config("");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.config, Config{});
}

TEST(Config, PartialOverrideKeepsOtherDefaults)
{
  const auto r = depthd::parse_config("processing:\n  matcher: libsgm\n  num_disparities: 64\n");
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.config.processing.matcher, "libsgm");
  EXPECT_EQ(r.config.processing.num_disparities, 64);
  EXPECT_EQ(r.config.processing.width, 640);
}

TEST(Config, MissingFileIsRejectedWithPath)
{
  const auto r = depthd::load_config("/nonexistent/depthd.yaml");
  ASSERT_FALSE(r.ok());
  EXPECT_NE(r.error.find("/nonexistent/depthd.yaml"), std::string::npos);
}

TEST(Config, RejectsUnknownKeys)
{
  expect_rejected("bogus: 1\n", "bogus");
  expect_rejected("processing:\n  bogus_key: 1\n", "processing.bogus_key");
  expect_rejected("postprocess:\n  temporal:\n    gamma: 1\n", "postprocess.temporal.gamma");
}

TEST(Config, RejectsWrongTypes)
{
  expect_rejected("processing:\n  width: wide\n", "processing.width");
  expect_rejected("mono:\n  enabled: 3\n", "mono.enabled");
}

TEST(Config, RejectsBadNumDisparities)
{
  expect_rejected("processing:\n  num_disparities: 100\n", "processing.num_disparities");
  expect_rejected("processing:\n  num_disparities: 32\n", "processing.num_disparities");
}

TEST(Config, AcceptsAllNumDisparities)
{
  for (const int n : {64, 128, 256}) {
    const auto r = depthd::parse_config("processing:\n  num_disparities: " + std::to_string(n) + "\n");
    EXPECT_TRUE(r.ok()) << n << ": " << r.error;
  }
}

TEST(Config, RejectsDimensionsNotDivisibleByEight)
{
  expect_rejected("processing:\n  width: 641\n", "processing.width");
  expect_rejected("processing:\n  height: 481\n", "processing.height");
  expect_rejected("rgb:\n  width: 642\n", "rgb.width");
  expect_rejected("rgb:\n  height: 478\n", "rgb.height");
  expect_rejected("processing:\n  width: 0\n", "processing.width");
}

TEST(Config, RejectsNonPositiveRates)
{
  expect_rejected("mono:\n  rate_hz: 0\n", "mono.rate_hz");
  expect_rejected("mono:\n  rate_hz: -1\n", "mono.rate_hz");
  expect_rejected("rgb:\n  fps: 0\n", "rgb.fps");
  expect_rejected("capture:\n  sensor_mode:\n    fps: 0\n", "capture.sensor_mode.fps");
}

TEST(Config, RejectsNonPositivePairWindow)
{
  expect_rejected("capture:\n  pair_window_ms: 0\n", "capture.pair_window_ms");
  expect_rejected("capture:\n  pair_window_ms: -2.5\n", "capture.pair_window_ms");
}

TEST(Config, RejectsUnknownMatcher)
{
  expect_rejected("processing:\n  matcher: bm\n", "processing.matcher");
  for (const char * m : {"cuda_sgm", "vpi", "libsgm"}) {
    EXPECT_TRUE(depthd::parse_config(std::string("processing:\n  matcher: ") + m + "\n").ok()) << m;
  }
}

TEST(Config, RejectsUnknownMonoModel)
{
  expect_rejected("mono:\n  model: resnet\n", "mono.model");
  for (const char * m : {"dav2_small", "midas_small"}) {
    EXPECT_TRUE(depthd::parse_config(std::string("mono:\n  model: ") + m + "\n").ok()) << m;
  }
}

TEST(Config, RejectsPortsOutsideRange)
{
  expect_rejected("transport:\n  pub_port: 1023\n", "transport.pub_port");
  expect_rejected("transport:\n  pub_port: 65536\n", "transport.pub_port");
  expect_rejected("transport:\n  http_port: 80\n", "transport.http_port");
  expect_rejected("transport:\n  http_port: 70000\n", "transport.http_port");
  EXPECT_TRUE(depthd::parse_config("transport:\n  pub_port: 1024\n  http_port: 65535\n").ok());
}

TEST(Config, RejectsOtherInvalidValues)
{
  expect_rejected("processing:\n  min_depth_m: 7\n", "processing.min_depth_m");
  expect_rejected("postprocess:\n  temporal:\n    alpha: 1.5\n", "postprocess.temporal.alpha");
  expect_rejected("rgb:\n  jpeg_quality: 101\n", "rgb.jpeg_quality");
  expect_rejected("transport:\n  send_hwm: 0\n", "transport.send_hwm");
  expect_rejected("capture:\n  settle_frames: -1\n", "capture.settle_frames");
  expect_rejected("calibration:\n  max_rms_px: 0\n", "calibration.max_rms_px");
  expect_rejected("frame_ids:\n  depth: ''\n", "frame_ids.depth");
  expect_rejected("transport:\n  bind_address: ''\n", "transport.bind_address");
  expect_rejected("capture:\n  left_sensor_id: 0\n  right_sensor_id: 0\n", "capture.right_sensor_id");
}

TEST(Config, ValidateRejectsProgrammaticValues)
{
  Config c;
  EXPECT_EQ(depthd::validate(c), "");
  c.processing.num_disparities = 99;
  EXPECT_NE(depthd::validate(c).find("processing.num_disparities"), std::string::npos);
}

TEST(Config, JsonRoundTrip)
{
  Config c;
  c.processing.matcher = "vpi";
  c.processing.num_disparities = 256;
  c.postprocess.wls.enabled = true;
  c.postprocess.temporal.alpha = 0.25;
  c.mono.rate_hz = 1.5;
  c.rgb.jpeg_quality = 55;
  c.transport.http_port = 9000;
  const auto j = depthd::to_json(c);
  EXPECT_EQ(j.at("processing").at("matcher"), "vpi");
  EXPECT_EQ(j.at("capture").at("sensor_mode").at("width"), 1640);
  EXPECT_EQ(j.at("frame_ids").at("depth"), "stereo_left_optical_frame");
  const auto r = depthd::from_json(j);
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.config, c);
  EXPECT_EQ(depthd::to_json(r.config), j);
}

TEST(Config, FromJsonRejectsUnknownKeyAndInvalidValue)
{
  auto j = depthd::to_json(Config{});
  j["processing"]["nope"] = 1;
  auto r = depthd::from_json(j);
  ASSERT_FALSE(r.ok());
  EXPECT_NE(r.error.find("processing.nope"), std::string::npos);
  j = depthd::to_json(Config{});
  j["transport"]["pub_port"] = 5;
  r = depthd::from_json(j);
  ASSERT_FALSE(r.ok());
  EXPECT_NE(r.error.find("transport.pub_port"), std::string::npos);
}

TEST(Config, RuntimeMutableSet)
{
  for (const char * k :
       {"postprocess.lr_check.enabled", "postprocess.lr_check.max_diff", "postprocess.speckle.max_size",
        "postprocess.temporal.alpha", "postprocess.wls.enabled", "postprocess.hole_fill.max_hole_px", "mono.enabled",
        "mono.rate_hz", "rgb.jpeg_quality"}) {
    EXPECT_TRUE(depthd::runtime_mutable(k)) << k;
  }
  for (const char * k :
       {"mono.model", "mono.engine_path", "rgb.fps", "rgb.width", "processing.matcher", "processing.width",
        "processing.num_disparities", "capture.pair_window_ms", "transport.pub_port", "calibration.path",
        "frame_ids.depth", "postprocess", "postprocessx.y", "nonsense", ""}) {
    EXPECT_FALSE(depthd::runtime_mutable(k)) << k;
  }
}
