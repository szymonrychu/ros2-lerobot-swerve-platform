// Continuous AE policies (exposure_control.hpp) driven by fake sensor readbacks and synthetic grey images: when the
// right camera is re-pinned to the left, when pairs are withheld across a mismatch, and when the left's exposure
// compensation steps against saturation.
#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

#include "depthd/config.hpp"
#include "depthd/exposure_control.hpp"

using depthd::ExposureFollower;
using depthd::ExposureFollowerOptions;
using depthd::SaturationCompensator;
using depthd::SaturationOptions;
using depthd::SensorExposure;

namespace
{
ExposureFollowerOptions follower_options()
{
  ExposureFollowerOptions o;
  o.threshold = 0.05;
  o.settle_frames = 2;
  return o;
}

SaturationOptions saturation_options()
{
  SaturationOptions o;
  o.high = 0.02;
  o.low = 0.005;
  o.step = 0.25;
  o.min = -1.0;
  o.hold_frames = 10;
  return o;
}

// Pins the follower to `e` and brings the right camera there: readback matched, settle frames passed.
void settle(ExposureFollower & f, const SensorExposure & e)
{
  f.on_right(e);
  f.on_right_frame();
  f.on_right_frame();
}
}  // namespace

TEST(ExposureFollower, FirstLeftReadbackPinsTheRightAndPairsWaitForIt)
{
  ExposureFollower f(follower_options());
  EXPECT_FALSE(f.matched()) << "nothing pinned yet: the right runs its own AE";
  const auto pin = f.on_left({12000, 53});
  ASSERT_TRUE(pin);
  EXPECT_EQ(*pin, (SensorExposure{12000, 53}));
  EXPECT_EQ(f.pins(), 1u);
  EXPECT_FALSE(f.matched());
  f.on_right_frame();  // frames before the readback matched do not count
  f.on_right_frame();
  EXPECT_FALSE(f.matched());
  f.on_right({20000, 80});  // still the right's own AE values
  f.on_right_frame();
  f.on_right_frame();
  EXPECT_FALSE(f.matched());
  f.on_right({12000, 53});
  f.on_right_frame();
  EXPECT_FALSE(f.matched()) << "one settle frame of two";
  f.on_right_frame();
  EXPECT_TRUE(f.matched());
}

TEST(ExposureFollower, SmallLeftChangesDoNotRepin)
{
  ExposureFollower f(follower_options());
  ASSERT_TRUE(f.on_left({12000, 64}));
  settle(f, {12000, 64});
  ASSERT_TRUE(f.matched());
  EXPECT_FALSE(f.on_left({12400, 64}));  // +3.3 %
  EXPECT_FALSE(f.on_left({11600, 66}));  // -3.3 %, +3.1 %
  EXPECT_TRUE(f.matched());
  EXPECT_EQ(f.pins(), 1u);
}

TEST(ExposureFollower, LeftChangeBeyondThresholdRepinsAndWithholdsPairsUntilTheRightFollows)
{
  ExposureFollower f(follower_options());
  ASSERT_TRUE(f.on_left({12000, 64}));
  settle(f, {12000, 64});
  ASSERT_TRUE(f.matched());
  // Lights on: the left's AE cuts the exposure by half.
  const auto pin = f.on_left({6000, 64});
  ASSERT_TRUE(pin);
  EXPECT_EQ(*pin, (SensorExposure{6000, 64}));
  EXPECT_FALSE(f.matched()) << "the right still runs at the old exposure";
  f.on_right({12000, 64});
  f.on_right_frame();
  f.on_right_frame();
  EXPECT_FALSE(f.matched());
  settle(f, {6000, 64});
  EXPECT_TRUE(f.matched());
  EXPECT_EQ(f.pins(), 2u);
  // A gain-only change beyond the threshold re-pins too.
  ASSERT_TRUE(f.on_left({6000, 80}));
  EXPECT_FALSE(f.matched());
}

TEST(ExposureFollower, ReadbackWithinSensorQuantisationMatches)
{
  ExposureFollower f(follower_options());
  ASSERT_TRUE(f.on_left({33329, 101}));
  settle(f, {33330, 100});  // a line of exposure and one gain step apart: the sensors' own quantisation
  EXPECT_TRUE(f.matched());
}

TEST(ExposureFollower, ResetForgetsThePinAfterARightRestart)
{
  ExposureFollower f(follower_options());
  ASSERT_TRUE(f.on_left({12000, 64}));
  settle(f, {12000, 64});
  ASSERT_TRUE(f.matched());
  f.reset();
  EXPECT_FALSE(f.matched());
  EXPECT_FALSE(f.pin());
  const auto pin = f.on_left({12000, 64});  // same values: pinned again all the same
  ASSERT_TRUE(pin);
  EXPECT_EQ(f.pins(), 2u);
}

TEST(ExposureFollower, OptionsFromConfig)
{
  depthd::ExposureConfig e;
  e.follow_threshold = 0.1;
  e.follow_settle_frames = 4;
  const auto o = ExposureFollowerOptions::from_config(e);
  EXPECT_DOUBLE_EQ(o.threshold, 0.1);
  EXPECT_EQ(o.settle_frames, 4);
}

TEST(PinnedRangeProperties, FormatsNanosecondsAndGainAndClampsToArgusLimits)
{
  auto p = depthd::pinned_range_properties({12000, 53});
  EXPECT_EQ(p.first, "12000000 12000000");
  EXPECT_EQ(p.second, "3.3125 3.3125");
  p = depthd::pinned_range_properties({20, 8});  // below the plugin's 34 us and 1x
  EXPECT_EQ(p.first, "34000 34000");
  EXPECT_EQ(p.second, "1 1");
  p = depthd::pinned_range_properties({20000, 400});  // above 10.625x
  EXPECT_EQ(p.second, "10.625 10.625");
}

TEST(SaturationCompensator, StepsDownWhileSaturatedNoFasterThanTheHoldAndStopsAtMin)
{
  SaturationCompensator c(saturation_options());
  auto v = c.on_measurement(0.10, 0);
  ASSERT_TRUE(v);
  EXPECT_DOUBLE_EQ(*v, -0.25);
  EXPECT_FALSE(c.on_measurement(0.10, 5)) << "within hold_frames of the last step";
  v = c.on_measurement(0.10, 10);
  ASSERT_TRUE(v);
  EXPECT_DOUBLE_EQ(*v, -0.5);
  c.on_measurement(0.10, 20);
  v = c.on_measurement(0.10, 30);
  ASSERT_TRUE(v);
  EXPECT_DOUBLE_EQ(*v, -1.0);
  EXPECT_FALSE(c.on_measurement(0.10, 40)) << "at min";
  EXPECT_DOUBLE_EQ(c.compensation(), -1.0);
  EXPECT_EQ(c.adjustments(), 4u);
}

TEST(SaturationCompensator, HoldsBetweenTheThresholdsAndRecoversBelowLow)
{
  SaturationCompensator c(saturation_options());
  ASSERT_TRUE(c.on_measurement(0.05, 0));  // -0.25
  ASSERT_TRUE(c.on_measurement(0.05, 10));  // -0.5
  // Between low and high: hysteresis, no change either way.
  EXPECT_FALSE(c.on_measurement(0.01, 20));
  EXPECT_FALSE(c.on_measurement(0.019, 40));
  EXPECT_DOUBLE_EQ(c.compensation(), -0.5);
  // Below low: back toward 0, one step per hold, never above 0.
  auto v = c.on_measurement(0.001, 50);
  ASSERT_TRUE(v);
  EXPECT_DOUBLE_EQ(*v, -0.25);
  EXPECT_FALSE(c.on_measurement(0.001, 55));
  v = c.on_measurement(0.001, 60);
  ASSERT_TRUE(v);
  EXPECT_DOUBLE_EQ(*v, 0.0);
  EXPECT_FALSE(c.on_measurement(0.0, 100)) << "already at 0";
  EXPECT_EQ(c.adjustments(), 4u);
}

TEST(SaturationCompensator, DarkSceneAtZeroNeverMoves)
{
  SaturationCompensator c(saturation_options());
  for (int64_t f = 0; f < 300; f += 2) ASSERT_FALSE(c.on_measurement(0.0, f));
  EXPECT_EQ(c.adjustments(), 0u);
}

TEST(SaturationCompensator, OptionsFromConfig)
{
  depthd::ExposureConfig e;
  e.saturated_high = 0.03;
  e.saturated_low = 0.01;
  e.compensation_step = 0.5;
  e.compensation_min = -1.5;
  e.compensation_hold_frames = 7;
  const auto o = SaturationOptions::from_config(e);
  EXPECT_DOUBLE_EQ(o.high, 0.03);
  EXPECT_DOUBLE_EQ(o.low, 0.01);
  EXPECT_DOUBLE_EQ(o.step, 0.5);
  EXPECT_DOUBLE_EQ(o.min, -1.5);
  EXPECT_EQ(o.hold_frames, 7);
}

TEST(SaturatedFraction, CountsPixelsAtOrAboveTheLevelOnTheGpu)
{
  cv::Mat img(480, 640, CV_8UC1, cv::Scalar(100));
  img(cv::Rect(0, 0, 64, 48)).setTo(250);   // 1 % exactly at the level
  img(cv::Rect(64, 0, 64, 48)).setTo(255);  // 1 % above it
  img(cv::Rect(128, 0, 64, 48)).setTo(249);  // 1 % just below
  cv::cuda::GpuMat grey(img);
  cv::cuda::GpuMat scratch;
  EXPECT_NEAR(depthd::saturated_fraction(grey, 250, scratch), 0.02, 1e-9);
  EXPECT_NEAR(depthd::saturated_fraction(grey, 249, scratch), 0.03, 1e-9);
  EXPECT_DOUBLE_EQ(depthd::saturated_fraction(cv::cuda::GpuMat(), 250, scratch), 0.0);
}
