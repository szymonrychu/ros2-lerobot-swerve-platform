// The pure libcamera helpers of the adapter: control lists per phase, AfState mapping and Rect conversion. No camera is
// opened; real-camera behaviour is checked on the robot (final-suite-and-runbook).

#include <gtest/gtest.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include <cstdint>
#include <optional>
#include <set>

#include "head_camera_cpp/libcamera_backend.hpp"

namespace
{

using head_camera_cpp::af_controls;
using head_camera_cpp::AfRequest;
using head_camera_cpp::AfState;
using head_camera_cpp::Config;
using head_camera_cpp::focus_controls;
using head_camera_cpp::from_rectangle;
using head_camera_cpp::Rect;
using head_camera_cpp::still_controls;
using head_camera_cpp::to_af_state;
using head_camera_cpp::to_rectangle;
using head_camera_cpp::video_controls;
namespace controls = libcamera::controls;

/// @return The ids of every control in the list.
std::set<unsigned int> ids(const libcamera::ControlList & list)
{
  std::set<unsigned int> out;
  for (const auto & entry : list) {
    out.insert(entry.first);
  }
  return out;
}

void expect_frame_duration(const libcamera::ControlList & list, int64_t expected_us)
{
  const auto limits = list.get(controls::FrameDurationLimits);
  ASSERT_TRUE(limits.has_value());
  ASSERT_EQ(limits->size(), 2u);
  EXPECT_EQ((*limits)[0], expected_us);
  EXPECT_EQ((*limits)[1], expected_us);
}

}  // namespace

TEST(VideoControls, DefaultConfigHoldsExactlyFrameDurationAndContinuousAf)
{
  const libcamera::ControlList list = video_controls(Config{});

  EXPECT_EQ(ids(list), (std::set<unsigned int>{controls::FRAME_DURATION_LIMITS, controls::AF_MODE}));
  expect_frame_duration(list, 33333);
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeContinuous));
}

TEST(VideoControls, FrameDurationFollowsConfig)
{
  Config config;
  config.frame_duration_us = 50000;

  expect_frame_duration(video_controls(config), 50000);
}

TEST(VideoControls, AfModeFollowsConfig)
{
  Config config;
  config.af_mode = "auto";
  EXPECT_EQ(video_controls(config).get(controls::AfMode), std::optional<int32_t>(controls::AfModeAuto));
  config.af_mode = "manual";
  EXPECT_EQ(video_controls(config).get(controls::AfMode), std::optional<int32_t>(controls::AfModeManual));
  config.af_mode = "continuous";
  EXPECT_EQ(video_controls(config).get(controls::AfMode), std::optional<int32_t>(controls::AfModeContinuous));
}

TEST(VideoControls, FocusLockCarriesManualAfModeAndLensPosition)
{
  Config config;
  config.lens_position = 1.25;
  const libcamera::ControlList list = video_controls(config, true);

  EXPECT_EQ(
    ids(list), (std::set<unsigned int>{controls::FRAME_DURATION_LIMITS, controls::AF_MODE, controls::LENS_POSITION}));
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeManual));
  EXPECT_EQ(list.get(controls::LensPosition), std::optional<float>(1.25F));
  expect_frame_duration(list, 33333);
}

TEST(VideoControls, UnlockedKeepsConfiguredAfModeWithoutLensPosition)
{
  const libcamera::ControlList list = video_controls(Config{}, false);

  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeContinuous));
  EXPECT_FALSE(list.get(controls::LensPosition).has_value());
}

TEST(FocusControls, LockedIsManualAtLensPosition)
{
  Config config;
  config.lens_position = 0.5;
  const libcamera::ControlList list = focus_controls(config, true);

  EXPECT_EQ(ids(list), (std::set<unsigned int>{controls::AF_MODE, controls::LENS_POSITION}));
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeManual));
  EXPECT_EQ(list.get(controls::LensPosition), std::optional<float>(0.5F));
}

TEST(FocusControls, UnlockedRestoresConfiguredAfMode)
{
  Config config;
  config.af_mode = "continuous";
  const libcamera::ControlList list = focus_controls(config, false);

  EXPECT_EQ(ids(list), (std::set<unsigned int>{controls::AF_MODE}));
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeContinuous));
}

TEST(AfControls, WindowedTriggerHoldsExactlyAutoWindowsAndStart)
{
  const Rect window{1200, 640, 576, 324};
  const libcamera::ControlList list = af_controls(AfRequest{true, window, true});

  EXPECT_EQ(
    ids(list),
    (std::set<unsigned int>{controls::AF_MODE, controls::AF_METERING, controls::AF_WINDOWS, controls::AF_TRIGGER}));
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeAuto));
  EXPECT_EQ(list.get(controls::AfMetering), std::optional<int32_t>(controls::AfMeteringWindows));
  EXPECT_EQ(list.get(controls::AfTrigger), std::optional<int32_t>(controls::AfTriggerStart));
  const auto windows = list.get(controls::AfWindows);
  ASSERT_TRUE(windows.has_value());
  ASSERT_EQ(windows->size(), 1u);
  EXPECT_EQ((*windows)[0], libcamera::Rectangle(1200, 640, 576, 324));
}

TEST(AfControls, WindowedWithoutTriggerHasNoTrigger)
{
  const libcamera::ControlList list = af_controls(AfRequest{true, Rect{0, 0, 100, 50}, false});

  EXPECT_EQ(ids(list), (std::set<unsigned int>{controls::AF_MODE, controls::AF_METERING, controls::AF_WINDOWS}));
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeAuto));
  EXPECT_EQ(list.get(controls::AfMetering), std::optional<int32_t>(controls::AfMeteringWindows));
}

TEST(AfControls, WholeFrameTriggerMetersAutoWithoutWindows)
{
  const libcamera::ControlList list = af_controls(AfRequest{false, Rect{10, 20, 30, 40}, true});

  EXPECT_EQ(ids(list), (std::set<unsigned int>{controls::AF_MODE, controls::AF_METERING, controls::AF_TRIGGER}));
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeAuto));
  EXPECT_EQ(list.get(controls::AfMetering), std::optional<int32_t>(controls::AfMeteringAuto));
  EXPECT_EQ(list.get(controls::AfTrigger), std::optional<int32_t>(controls::AfTriggerStart));
}

TEST(StillControls, HoldsExactlyAutoAfWithoutTrigger)
{
  const libcamera::ControlList list = still_controls();

  EXPECT_EQ(ids(list), (std::set<unsigned int>{controls::AF_MODE}));
  EXPECT_EQ(list.get(controls::AfMode), std::optional<int32_t>(controls::AfModeAuto));
}

TEST(AfStateMapping, EveryLibcameraValue)
{
  EXPECT_EQ(to_af_state(controls::AfStateIdle), AfState::Idle);
  EXPECT_EQ(to_af_state(controls::AfStateScanning), AfState::Scanning);
  EXPECT_EQ(to_af_state(controls::AfStateFocused), AfState::Focused);
  EXPECT_EQ(to_af_state(controls::AfStateFailed), AfState::Failed);
}

TEST(AfStateMapping, UnknownValueIsIdle)
{
  EXPECT_EQ(to_af_state(-1), AfState::Idle);
  EXPECT_EQ(to_af_state(42), AfState::Idle);
}

TEST(RectConversion, ToRectangleKeepsEveryField)
{
  EXPECT_EQ(to_rectangle(Rect{16, 8, 4608, 2592}), libcamera::Rectangle(16, 8, 4608, 2592));
}

TEST(RectConversion, FromRectangleKeepsEveryField)
{
  const Rect rect = from_rectangle(libcamera::Rectangle(-4, 6, 1152, 648));
  EXPECT_EQ(rect.x, -4);
  EXPECT_EQ(rect.y, 6);
  EXPECT_EQ(rect.width, 1152);
  EXPECT_EQ(rect.height, 648);
}

TEST(RectConversion, RoundTrip)
{
  const Rect original{2, 1300, 2304, 1296};
  const Rect back = from_rectangle(to_rectangle(original));
  EXPECT_EQ(back.x, original.x);
  EXPECT_EQ(back.y, original.y);
  EXPECT_EQ(back.width, original.width);
  EXPECT_EQ(back.height, original.height);

  const libcamera::Rectangle sensor(0, 0, 4608, 2592);
  EXPECT_EQ(to_rectangle(from_rectangle(sensor)), sensor);
}
