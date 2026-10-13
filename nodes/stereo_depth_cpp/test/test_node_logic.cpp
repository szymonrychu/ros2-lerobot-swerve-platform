// The node's per-pair logic without ROS (StereoDepthCore): image decoding, the rate cap with RATE_JITTER_FRACTION,
// every drop reason in the Python node's order, the calibration (re)load, the metric names and labels, and parity of
// the whole path (bgr8 Image -> grey -> rectify -> SGBM -> depth mm) with the Python node's depth fixtures.
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "fixtures.hpp"
#include "stereo_depth_cpp/pipeline.hpp"

namespace
{

using stereo_depth_cpp::DepthConfig;
using stereo_depth_cpp::StereoDepthCore;

constexpr std::int64_t STAMP_NS = 10'000'000'000;  // conftest image_msg sec=10

// The value of one series in the Prometheus text ("name" or "name{labels}"), or -1 when it is absent.
double metric(const std::string & text, const std::string & series)
{
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    for (const std::string & prefix : {series + " ", series + "{} "}) {
      if (line.rfind(prefix, 0) == 0) {
        return std::stod(line.substr(prefix.size()));
      }
    }
  }
  return -1.0;
}

double dropped(const ros2_common_cpp::Metrics & metrics, const std::string & reason)
{
  return metric(metrics.render(), "stereo_depth_frames_dropped_total{reason=\"" + reason + "\"}");
}

DepthConfig config_for(const std::string & calibration_dir)
{
  DepthConfig config;
  config.calibration_dir = calibration_dir;
  return config;
}

// A loaded core on a fixture calibration case.
struct Loaded
{
  explicit Loaded(const std::string & case_name, DepthConfig config = DepthConfig{})
  : metrics(stereo_depth_cpp::NODE_NAME)
  {
    fixtures::copy_calibration(case_name, tmp.path());
    config.calibration_dir = tmp.path();
    core = std::make_unique<StereoDepthCore>(config, metrics);
    core->check_calibration();
  }
  fixtures::TempDir tmp;
  ros2_common_cpp::Metrics metrics;
  std::unique_ptr<StereoDepthCore> core;
};

void expect_depth_parity(const cv::Mat & depth, const cv::Mat & expected)
{
  ASSERT_EQ(depth.type(), CV_16UC1);
  ASSERT_EQ(depth.size(), expected.size());
  const cv::Mat valid = depth > 0;
  const cv::Mat expected_valid = expected > 0;
  EXPECT_EQ(cv::countNonZero(valid != expected_valid), 0) << "invalid masks differ";
  cv::Mat diff;
  cv::absdiff(depth, expected, diff);
  double max_diff = 0.0;
  cv::minMaxLoc(diff, nullptr, &max_diff, nullptr, nullptr, valid & expected_valid);
  EXPECT_LE(max_diff, 1.0) << "depth differs by more than 1 mm on valid pixels";
}

}  // namespace

TEST(ImageToGrey, DecodesEveryEncoding)
{
  const cv::Mat grey = fixtures::png("left.png");
  for (const char * encoding : {"mono8", "bgr8", "rgb8", "bgra8", "rgba8"}) {
    SCOPED_TRACE(encoding);
    const auto buffer = fixtures::image_buffer(grey, encoding, 0);
    const auto decoded = stereo_depth_cpp::image_to_grey(buffer.view);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->type(), CV_8UC1);
    EXPECT_EQ(cv::countNonZero(*decoded != grey), 0);
  }
}

TEST(ImageToGrey, RgbAndBgrSwapChannels)
{
  const std::uint8_t pixel[3] = {255, 0, 0};
  stereo_depth_cpp::ImageView view{"bgr8", 1, 1, 3, pixel, 3, 0};
  const int as_bgr = stereo_depth_cpp::image_to_grey(view)->at<std::uint8_t>(0, 0);
  view.encoding = "rgb8";
  const int as_rgb = stereo_depth_cpp::image_to_grey(view)->at<std::uint8_t>(0, 0);
  EXPECT_EQ(as_bgr, 29);  // blue weight 0.114
  EXPECT_EQ(as_rgb, 76);  // red weight 0.299
}

TEST(ImageToGrey, PaddedRowsAreCropped)
{
  const std::uint8_t data[] = {1, 2, 99, 99, 3, 4, 99, 99};
  const stereo_depth_cpp::ImageView view{"mono8", 2, 2, 4, data, sizeof(data), 0};
  const auto decoded = stereo_depth_cpp::image_to_grey(view);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->at<std::uint8_t>(0, 1), 2);
  EXPECT_EQ(decoded->at<std::uint8_t>(1, 0), 3);
}

TEST(ImageToGrey, RejectsUnsupportedEmptyAndTruncated)
{
  const std::uint8_t data[12] = {};
  EXPECT_FALSE(stereo_depth_cpp::image_to_grey({"16UC1", 2, 2, 4, data, 8, 0}).has_value());
  EXPECT_FALSE(stereo_depth_cpp::image_to_grey({"mono8", 0, 2, 0, data, 0, 0}).has_value());
  EXPECT_FALSE(stereo_depth_cpp::image_to_grey({"mono8", 2, 0, 2, data, 0, 0}).has_value());
  EXPECT_FALSE(stereo_depth_cpp::image_to_grey({"bgr8", 2, 2, 5, data, 12, 0}).has_value());  // step < width * 3
  EXPECT_FALSE(stereo_depth_cpp::image_to_grey({"bgr8", 2, 2, 6, data, 11, 0}).has_value());  // truncated
  EXPECT_FALSE(stereo_depth_cpp::image_to_grey({"mono8", 2, 2, 2, nullptr, 4, 0}).has_value());
}

TEST(RateCap, AdmitsOnePairPerPeriodWithJitter)
{
  stereo_depth_cpp::RateCap cap(5.0);  // period 0.2 s, jitter 0.02 s
  EXPECT_TRUE(cap.admit(0.0));
  EXPECT_FALSE(cap.admit(0.1));
  EXPECT_FALSE(cap.admit(0.179));
  EXPECT_TRUE(cap.admit(0.181));  // 0.019 s early: within RATE_JITTER_FRACTION of a period
  EXPECT_FALSE(cap.admit(0.25));  // next slot 0.4
  EXPECT_TRUE(cap.admit(1.0));    // late: the slot moves to 1.2
  EXPECT_FALSE(cap.admit(1.1));
  EXPECT_TRUE(cap.admit(1.19));
}

TEST(Core, NoCalibrationPublishesNothingAndCounts)
{
  fixtures::TempDir tmp;
  ros2_common_cpp::Metrics metrics(stereo_depth_cpp::NODE_NAME);
  StereoDepthCore core(config_for(tmp.path()), metrics);
  const auto check = core.check_calibration();
  EXPECT_FALSE(check.loaded);
  EXPECT_FALSE(core.calibration().has_value());
  const cv::Mat left = fixtures::png("left.png");
  const auto l = fixtures::image_buffer(left, "bgr8", STAMP_NS);
  const auto r = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS);
  for (int i = 0; i < 3; ++i) {
    const auto result = core.process(l.view, r.view, 100.0 + i);
    EXPECT_EQ(result.reason, "no_calibration");
    EXPECT_TRUE(result.depth_mm.empty());
  }
  EXPECT_EQ(dropped(metrics, "no_calibration"), 3.0);
  EXPECT_EQ(metric(metrics.render(), "stereo_depth_calibration_loaded"), 0.0);
  EXPECT_EQ(metric(metrics.render(), "stereo_depth_frames_published_total"), 0.0);
}

TEST(Core, ExcessiveSkewIsDroppedAndCounted)
{
  Loaded loaded("ideal");
  const auto l = fixtures::image_buffer(fixtures::png("left.png"), "bgr8", STAMP_NS);
  const auto r = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS + 3'000'000);  // 3 ms > 2 ms
  const auto result = loaded.core->process(l.view, r.view, 0.0);
  EXPECT_EQ(result.reason, "skew");
  EXPECT_TRUE(result.depth_mm.empty());
  EXPECT_EQ(dropped(loaded.metrics, "skew"), 1.0);
  const std::string text = loaded.metrics.render();
  EXPECT_EQ(metric(text, "stereo_depth_pair_skew_seconds_count"), 1.0);
  EXPECT_NEAR(metric(text, "stereo_depth_pair_skew_seconds_sum"), 0.003, 1e-9);
  // The skew drop does not take a rate slot: a good pair right after it publishes.
  const auto good = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS + 1'000'000);
  EXPECT_EQ(loaded.core->process(l.view, good.view, 0.0).reason, "");
}

TEST(Core, SkewIsCheckedBeforeCalibration)
{
  fixtures::TempDir tmp;
  ros2_common_cpp::Metrics metrics(stereo_depth_cpp::NODE_NAME);
  StereoDepthCore core(config_for(tmp.path()), metrics);
  core.check_calibration();
  const auto l = fixtures::image_buffer(fixtures::png("left.png"), "bgr8", STAMP_NS + 5'000'000);
  const auto r = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS);
  EXPECT_EQ(core.process(l.view, r.view, 0.0).reason, "skew");
}

TEST(Core, SizeMismatchIsDroppedAndCounted)
{
  Loaded loaded("ideal");
  const cv::Mat left = fixtures::png("left.png");
  const cv::Mat small = left(cv::Rect(0, 0, 320, 240)).clone();
  const auto l = fixtures::image_buffer(left, "bgr8", STAMP_NS);
  const auto r_small = fixtures::image_buffer(small, "bgr8", STAMP_NS);
  auto result = loaded.core->process(l.view, r_small.view, 0.0);
  EXPECT_EQ(result.reason, "size_mismatch");
  EXPECT_TRUE(result.depth_mm.empty());
  // Both the same size, but not the calibrated one.
  const auto l_small = fixtures::image_buffer(small, "bgr8", STAMP_NS);
  result = loaded.core->process(l_small.view, r_small.view, 10.0);
  EXPECT_EQ(result.reason, "size_mismatch");
  EXPECT_EQ(dropped(loaded.metrics, "size_mismatch"), 2.0);
  EXPECT_EQ(metric(loaded.metrics.render(), "stereo_depth_frames_published_total"), 0.0);
}

TEST(Core, BadImageIsDroppedAfterTheRateSlot)
{
  Loaded loaded("ideal");
  const std::uint8_t data[8] = {};
  const stereo_depth_cpp::ImageView bad{"16UC1", 2, 2, 4, data, 8, STAMP_NS};
  EXPECT_EQ(loaded.core->process(bad, bad, 0.0).reason, "bad_image");
  EXPECT_EQ(loaded.core->process(bad, bad, 0.01).reason, "rate_limit");  // the bad pair took the slot
  EXPECT_EQ(dropped(loaded.metrics, "bad_image"), 1.0);
  EXPECT_EQ(dropped(loaded.metrics, "rate_limit"), 1.0);
}

TEST(Core, RateCapDropsExtraPairsBeforeMatching)
{
  Loaded loaded("ideal");
  const auto l = fixtures::image_buffer(fixtures::png("left.png"), "bgr8", STAMP_NS);
  const auto r = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS);
  EXPECT_EQ(loaded.core->process(l.view, r.view, 0.0).reason, "");
  EXPECT_EQ(loaded.core->process(l.view, r.view, 0.05).reason, "rate_limit");
  EXPECT_EQ(loaded.core->process(l.view, r.view, 0.1).reason, "rate_limit");
  EXPECT_EQ(loaded.core->process(l.view, r.view, 0.19).reason, "");  // within the jitter of the 0.2 s slot
  EXPECT_EQ(dropped(loaded.metrics, "rate_limit"), 2.0);
  const std::string text = loaded.metrics.render();
  EXPECT_EQ(metric(text, "stereo_depth_frames_published_total"), 2.0);
  EXPECT_EQ(metric(text, "stereo_depth_matcher_seconds_count"), 2.0);  // rate-limited pairs are never matched
}

TEST(Core, NoValidPixelsPublishesNothing)
{
  Loaded loaded("ideal");
  const cv::Mat flat(360, 480, CV_8UC1, cv::Scalar(128));
  const auto l = fixtures::image_buffer(flat, "mono8", STAMP_NS);
  const auto r = fixtures::image_buffer(flat, "mono8", STAMP_NS);
  const auto result = loaded.core->process(l.view, r.view, 0.0);
  EXPECT_EQ(result.reason, "no_valid_pixels");
  EXPECT_TRUE(result.depth_mm.empty());
  EXPECT_EQ(dropped(loaded.metrics, "no_valid_pixels"), 1.0);
}

TEST(CoreParity, DepthMatchesThePythonNodeWithinOneMillimetre)
{
  for (const char * name : {"ideal", "distorted"}) {
    SCOPED_TRACE(name);
    Loaded loaded(name);
    ASSERT_TRUE(loaded.core->calibration().has_value());
    const auto l = fixtures::image_buffer(fixtures::png("left.png"), "bgr8", STAMP_NS);
    const auto r = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS);
    const auto result = loaded.core->process(l.view, r.view, 0.0);
    ASSERT_EQ(result.reason, "") << result.message;
    EXPECT_EQ(result.disparity.type(), CV_32FC1);
    expect_depth_parity(result.depth_mm, fixtures::png(std::string(name) + "/depth_mm.png"));
  }
}

TEST(CoreParity, EveryEncodingGivesTheSameDepth)
{
  const cv::Mat expected = fixtures::png("ideal/depth_mm.png");
  double now_s = 0.0;
  Loaded loaded("ideal");
  for (const char * encoding : {"mono8", "rgb8", "bgra8", "rgba8"}) {
    SCOPED_TRACE(encoding);
    const auto l = fixtures::image_buffer(fixtures::png("left.png"), encoding, STAMP_NS);
    const auto r = fixtures::image_buffer(fixtures::png("right.png"), encoding, STAMP_NS);
    const auto result = loaded.core->process(l.view, r.view, now_s);
    now_s += 1.0;
    ASSERT_EQ(result.reason, "");
    expect_depth_parity(result.depth_mm, expected);
  }
}

TEST(Core, CalibrationIsReloadedWhenTheFilesChange)
{
  fixtures::TempDir tmp;
  ros2_common_cpp::Metrics metrics(stereo_depth_cpp::NODE_NAME);
  StereoDepthCore core(config_for(tmp.path()), metrics);
  EXPECT_TRUE(core.check_calibration().changed);
  EXPECT_FALSE(core.check_calibration().changed);  // unchanged files: no reload
  fixtures::copy_calibration("ideal", tmp.path());
  const auto loaded = core.check_calibration();
  EXPECT_TRUE(loaded.changed);
  EXPECT_TRUE(loaded.loaded);
  EXPECT_TRUE(core.calibration().has_value());
  EXPECT_EQ(metric(metrics.render(), "stereo_depth_calibration_loaded"), 1.0);
  tmp.write("stereo.yaml", "image_width: 480\n");  // now invalid: dropped, nothing published
  const auto invalid = core.check_calibration();
  EXPECT_TRUE(invalid.changed);
  EXPECT_FALSE(invalid.loaded);
  EXPECT_NE(invalid.message.find("stereo.yaml"), std::string::npos) << invalid.message;
  EXPECT_FALSE(core.calibration().has_value());
  EXPECT_EQ(metric(metrics.render(), "stereo_depth_calibration_loaded"), 0.0);
  const auto l = fixtures::image_buffer(fixtures::png("left.png"), "bgr8", STAMP_NS);
  const auto r = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS);
  EXPECT_EQ(core.process(l.view, r.view, 0.0).reason, "no_calibration");
}

TEST(Core, MetricNamesAndLabelsAreThePythonOnes)
{
  Loaded loaded("ideal");
  const auto l = fixtures::image_buffer(fixtures::png("left.png"), "bgr8", STAMP_NS);
  const auto r = fixtures::image_buffer(fixtures::png("right.png"), "bgr8", STAMP_NS + 1'000'000);
  ASSERT_EQ(loaded.core->process(l.view, r.view, 0.0).reason, "");
  const std::string text = loaded.metrics.render();
  EXPECT_EQ(metric(text, "robot_node_info{node=\"stereo_depth\"}"), 1.0) << text;
  EXPECT_EQ(metric(text, "stereo_depth_frames_published_total"), 1.0) << text;
  EXPECT_EQ(metric(text, "stereo_depth_calibration_loaded"), 1.0) << text;
  EXPECT_EQ(metric(text, "stereo_depth_matcher_seconds_count"), 1.0) << text;
  EXPECT_EQ(metric(text, "stereo_depth_matcher_seconds_bucket{le=\"0.01\"}") >= 0.0, true) << text;
  EXPECT_EQ(metric(text, "stereo_depth_matcher_seconds_bucket{le=\"2\"}") >= 0.0, true) << text;
  EXPECT_EQ(metric(text, "stereo_depth_pair_skew_seconds_bucket{le=\"0.002\"}"), 1.0) << text;
  EXPECT_EQ(metric(text, "stereo_depth_pair_skew_seconds_bucket{le=\"0.0001\"}"), 0.0) << text;
  EXPECT_EQ(
    stereo_depth_cpp::matcher_buckets(),
    (std::vector<double>{0.01, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0, 2.0}));
  EXPECT_EQ(
    stereo_depth_cpp::skew_buckets(),
    (std::vector<double>{0.0001, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1}));
}
