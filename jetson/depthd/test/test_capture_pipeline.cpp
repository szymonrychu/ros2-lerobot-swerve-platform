#include <gst/gst.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "depthd/capture.hpp"

using depthd::CameraSide;
using depthd::Config;

namespace
{
const std::string kCameraCaps = "video/x-raw(memory:NVMM),width=1640,height=1232,format=NV12,framerate=30/1";
const std::string kSink = " max-buffers=1 drop=true sync=false";

std::string source(const std::string & side, int id)
{
  return "nvarguscamerasrc name=" + side + "_src sensor-id=" + std::to_string(id) +
         " sensor-mode=3 ee-mode=0 tnr-mode=1 aelock=false awblock=false exposuretimerange=\"34000 20000000\" "
         "gainrange=\"1 8\" ispdigitalgainrange=\"1 1\" ! " +
         kCameraCaps;
}

const std::string kJpegBranch =
  "left_tee. ! queue name=rgb_queue max-size-buffers=1 leaky=downstream ! nvvidconv ! "
  "video/x-raw(memory:NVMM),width=640,height=480,format=I420 ! nvjpegenc name=rgb_jpeg quality=80 ! "
  "appsink name=rgb_sink max-buffers=2 drop=true sync=false";

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

int64_t now_ns(clockid_t clock)
{
  timespec ts{};
  clock_gettime(clock, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}
}  // namespace

TEST(CapturePipeline, SensorModeTable)
{
  EXPECT_EQ(depthd::imx219_sensor_mode({1640, 1232, 30}), 3);
  EXPECT_EQ(depthd::imx219_sensor_mode({3264, 2464, 21}), 0);
  EXPECT_EQ(depthd::imx219_sensor_mode({1920, 1080, 30}), 2);
  EXPECT_EQ(depthd::imx219_sensor_mode({1280, 720, 60}), 4);
  EXPECT_EQ(depthd::imx219_sensor_mode({1280, 720, 120}), 5);
  EXPECT_EQ(depthd::imx219_sensor_mode({1640, 1232, 60}), -1);
}

TEST(CapturePipeline, ZeroCopyLeftPinned)
{
  Config c;
  c.capture.zero_copy = true;
  EXPECT_EQ(
    depthd::build_pipeline_description(c, CameraSide::left),
    source("left", 0) +
      " ! tee name=left_tee left_tee. ! queue max-size-buffers=2 leaky=downstream ! nvvidconv ! "
      "video/x-raw(memory:NVMM),width=640,height=480,format=GRAY8 ! appsink name=left_sink" +
      kSink + " " + kJpegBranch);
}

TEST(CapturePipeline, ZeroCopyRightPinned)
{
  Config c;
  c.capture.zero_copy = true;
  EXPECT_EQ(
    depthd::build_pipeline_description(c, CameraSide::right),
    source("right", 1) +
      " ! nvvidconv ! video/x-raw(memory:NVMM),width=640,height=480,format=GRAY8 ! appsink name=right_sink" + kSink);
}

TEST(CapturePipeline, SystemMemoryLeftPinned)
{
  Config c;
  c.capture.zero_copy = false;
  EXPECT_EQ(
    depthd::build_pipeline_description(c, CameraSide::left),
    source("left", 0) +
      " ! tee name=left_tee left_tee. ! queue max-size-buffers=2 leaky=downstream ! nvvidconv ! "
      "video/x-raw,width=640,height=480,format=GRAY8 ! appsink name=left_sink" +
      kSink + " " + kJpegBranch);
}

TEST(CapturePipeline, SystemMemoryRightPinned)
{
  Config c;
  c.capture.zero_copy = false;
  EXPECT_EQ(
    depthd::build_pipeline_description(c, CameraSide::right),
    source("right", 1) + " ! nvvidconv ! video/x-raw,width=640,height=480,format=GRAY8 ! appsink name=right_sink" +
      kSink);
}

TEST(CapturePipeline, FollowsConfig)
{
  Config c;
  c.capture.left_sensor_id = 1;
  c.capture.right_sensor_id = 0;
  c.capture.sensor_mode = {1280, 720, 60};
  c.processing.width = 320;
  c.processing.height = 240;
  c.rgb.width = 800;
  c.rgb.height = 600;
  c.rgb.jpeg_quality = 55;
  c.capture.exposure.max_exposure_us = 15000;
  c.capture.exposure.max_gain = 10.625;
  const auto left = depthd::build_pipeline_description(c, CameraSide::left);
  const auto right = depthd::build_pipeline_description(c, CameraSide::right);
  EXPECT_TRUE(contains(left, "name=left_src sensor-id=1 sensor-mode=4 ee-mode=0 tnr-mode=1"));
  EXPECT_TRUE(contains(right, "name=right_src sensor-id=0 sensor-mode=4 ee-mode=0 tnr-mode=1"));
  EXPECT_TRUE(contains(left, "width=1280,height=720,format=NV12,framerate=60/1"));
  EXPECT_TRUE(contains(left, "width=320,height=240,format=GRAY8"));
  EXPECT_TRUE(contains(right, "width=320,height=240,format=GRAY8"));
  EXPECT_TRUE(contains(left, "width=800,height=600,format=I420 ! nvjpegenc name=rgb_jpeg quality=55"));
  EXPECT_FALSE(contains(right, "nvjpegenc"));
  EXPECT_FALSE(contains(left, "videorate"));  // drops the sensor timestamp metadata
  for (const auto & d : {left, right}) {
    EXPECT_TRUE(contains(d, "exposuretimerange=\"34000 15000000\" gainrange=\"1 10.625\" ispdigitalgainrange=\"1 1\""))
      << d;
  }
}

TEST(CapturePipeline, RgbDisabledHasNoJpegBranch)
{
  Config c;
  c.rgb.enabled = false;
  const auto left = depthd::build_pipeline_description(c, CameraSide::left);
  EXPECT_EQ(
    left, source("left", 0) +
            " ! nvvidconv ! video/x-raw(memory:NVMM),width=640,height=480,format=GRAY8 ! appsink name=left_sink" +
            kSink);
}

TEST(CapturePipeline, UnknownSensorModeIsRejectedNamingTheKey)
{
  Config c;
  c.capture.sensor_mode = {1000, 1000, 30};
  try {
    depthd::build_pipeline_description(c, CameraSide::left);
    FAIL() << "accepted an unknown sensor mode";
  } catch (const std::invalid_argument & e) {
    EXPECT_TRUE(contains(e.what(), "capture.sensor_mode")) << e.what();
  }
}

// The descriptions parse with the installed L4T plugins and expose the named elements the capture code looks up.
TEST(CapturePipeline, DescriptionsParseWithTheInstalledPlugins)
{
  gst_init(nullptr, nullptr);
  for (bool zero_copy : {true, false}) {
    Config c;
    c.capture.zero_copy = zero_copy;
    for (auto side : {CameraSide::left, CameraSide::right}) {
      const auto desc = depthd::build_pipeline_description(c, side);
      GError * err = nullptr;
      GstElement * pipeline = gst_parse_launch(desc.c_str(), &err);
      ASSERT_NE(pipeline, nullptr) << desc;
      ASSERT_EQ(err, nullptr) << err->message << " in " << desc;
      const std::string p = side == CameraSide::left ? "left" : "right";
      std::vector<std::string> names{p + "_src", p + "_sink"};
      if (side == CameraSide::left) names.insert(names.end(), {"rgb_queue", "rgb_jpeg", "rgb_sink"});
      for (const auto & name : names) {
        GstElement * e = gst_bin_get_by_name(GST_BIN(pipeline), name.c_str());
        EXPECT_NE(e, nullptr) << name << " missing in " << desc;
        if (e) gst_object_unref(e);
      }
      gst_object_unref(pipeline);
    }
  }
}

TEST(CaptureTimestamps, SensorTimestampWinsOverPts)
{
  EXPECT_EQ(depthd::capture_monotonic_ns(int64_t{5000}, 100, 1000), 5000);
  EXPECT_EQ(depthd::capture_monotonic_ns(std::nullopt, 100, 1000), 1100);
  EXPECT_EQ(depthd::capture_monotonic_ns(int64_t{0}, 100, 1000), 1100);  // a zero sensor stamp is absent
  EXPECT_EQ(depthd::capture_monotonic_ns(std::nullopt, GST_CLOCK_TIME_NONE, 1000), std::nullopt);
  EXPECT_EQ(depthd::capture_monotonic_ns(std::nullopt, 100, GST_CLOCK_TIME_NONE), std::nullopt);
}

TEST(CaptureTimestamps, ClockOffsetMapsMonotonicToRealtimeAndRefreshesEverySecond)
{
  std::atomic<int64_t> mono{10'000'000'000};
  std::atomic<int64_t> real{1'700'000'000'000'000'000};
  depthd::ClockOffset offset([&] { return mono.load(); }, [&] { return real.load(); });
  EXPECT_EQ(offset.to_realtime(9'990'000'000), 1'699'999'999'990'000'000);
  // Realtime is slewed by +3 ms (chrony). Within the second the old offset holds; after it the new one applies.
  real += 3'000'000;
  mono += 500'000'000;
  real += 500'000'000;
  EXPECT_EQ(offset.to_realtime(10'000'000'000), 1'700'000'000'000'000'000);
  mono += 600'000'000;
  real += 600'000'000;
  EXPECT_EQ(offset.to_realtime(10'000'000'000), 1'700'000'000'003'000'000);
}

TEST(CaptureTimestamps, SystemClockOffsetMatchesTheClocks)
{
  depthd::ClockOffset offset;
  const int64_t mono = now_ns(CLOCK_MONOTONIC);
  const int64_t real = now_ns(CLOCK_REALTIME);
  EXPECT_NEAR(static_cast<double>(offset.to_realtime(mono)), static_cast<double>(real), 1e6);
}

TEST(ExposureLock, LocksOnceAfterBothCamerasSettle)
{
  depthd::ExposureLockPolicy policy(3, true);
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(policy.on_frame(CameraSide::left));
  EXPECT_FALSE(policy.on_frame(CameraSide::right));
  EXPECT_FALSE(policy.on_frame(CameraSide::right));
  EXPECT_TRUE(policy.on_frame(CameraSide::right));  // right reaches 3: both settled
  EXPECT_TRUE(policy.locked());
  EXPECT_FALSE(policy.on_frame(CameraSide::right));
  EXPECT_FALSE(policy.on_frame(CameraSide::left));
  policy.reset();
  EXPECT_FALSE(policy.locked());
  for (int i = 0; i < 3; ++i) policy.on_frame(CameraSide::left);
  for (int i = 0; i < 2; ++i) EXPECT_FALSE(policy.on_frame(CameraSide::right));
  EXPECT_TRUE(policy.on_frame(CameraSide::right));
}

TEST(ExposureLock, DisabledNeverLocks)
{
  depthd::ExposureLockPolicy policy(0, false);
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(policy.on_frame(CameraSide::left));
    EXPECT_FALSE(policy.on_frame(CameraSide::right));
  }
}

TEST(ExposureLock, ZeroSettleFramesLocksOnFirstPair)
{
  depthd::ExposureLockPolicy policy(0, true);
  EXPECT_FALSE(policy.on_frame(CameraSide::left));
  EXPECT_TRUE(policy.on_frame(CameraSide::right));
}

// 30 fps sensor (33.3 ms frames) throttled to 10 fps: every third frame.
static uint64_t frame_pts(int i, uint64_t base = 0) { return base + static_cast<uint64_t>(i) * 33333333ULL; }

TEST(RgbThrottleTest, KeepsOneBufferPerRgbInterval)
{
  depthd::RgbThrottle throttle(10.0, 30.0);
  std::vector<int> admitted;
  for (int i = 0; i < 12; ++i) {
    if (throttle.admit(frame_pts(i))) admitted.push_back(i);
  }
  EXPECT_EQ(admitted, (std::vector<int>{0, 3, 6, 9}));
}

TEST(RgbThrottleTest, PtsRestartingNearZeroResyncsInsteadOfStarving)
{
  // The left pipeline ran for 60 s, then restarted (NULL -> PLAYING): running time, and so PTS, starts again near 0.
  depthd::RgbThrottle throttle(10.0, 30.0);
  for (int i = 0; i < 1800; ++i) throttle.admit(frame_pts(i));
  std::vector<int> admitted;
  for (int i = 0; i < 12; ++i) {
    if (throttle.admit(frame_pts(i, 5000000))) admitted.push_back(i);
  }
  EXPECT_EQ(admitted, (std::vector<int>{0, 3, 6, 9}));
}

TEST(RgbThrottleTest, ResetAdmitsTheNextBuffer)
{
  depthd::RgbThrottle throttle(10.0, 30.0);
  EXPECT_TRUE(throttle.admit(frame_pts(100)));
  EXPECT_FALSE(throttle.admit(frame_pts(101)));
  throttle.reset();
  EXPECT_TRUE(throttle.admit(frame_pts(101)));
  EXPECT_FALSE(throttle.admit(frame_pts(102)));
  EXPECT_TRUE(throttle.admit(frame_pts(104)));
}

TEST(CaptureStatusTest, StatesHaveHealthNames)
{
  EXPECT_STREQ(depthd::to_string(depthd::CaptureState::starting), "starting");
  EXPECT_STREQ(depthd::to_string(depthd::CaptureState::ok), "ok");
  EXPECT_STREQ(depthd::to_string(depthd::CaptureState::camera_error), "camera_error");
}

TEST(CaptureStatusTest, CameraTimesOutAfterTwoSecondsWithoutAFrame)
{
  constexpr int64_t s = 1'000'000'000;
  // No frame yet: measured from the pipeline start.
  EXPECT_FALSE(depthd::camera_timed_out(100 * s + 2 * s, 0, 100 * s));
  EXPECT_TRUE(depthd::camera_timed_out(100 * s + 2 * s + 1, 0, 100 * s));
  // Frames flowing: measured from the last frame.
  EXPECT_FALSE(depthd::camera_timed_out(110 * s, 109 * s, 100 * s));
  EXPECT_TRUE(depthd::camera_timed_out(112 * s, 109 * s, 100 * s));
  EXPECT_EQ(depthd::StereoCapture::kCameraTimeoutNs, 2 * s);
}

TEST(CaptureStatusTest, BadSensorModeFailsStartWithReason)
{
  Config c;
  c.capture.sensor_mode = {1000, 1000, 30};
  depthd::StereoCapture capture(c, [](const std::string &) {});
  std::string error;
  EXPECT_FALSE(capture.start(&error));
  EXPECT_TRUE(contains(error, "capture.sensor_mode")) << error;
  EXPECT_EQ(capture.status().state, depthd::CaptureState::camera_error);
}

// Sensors that do not exist deliver no frame: the capture reports camera_error naming them and never emits a pair.
// Uses sensor-ids 5 and 6, so it does not touch the real cameras.
TEST(CaptureStatusTest, MissingCamerasRaiseCameraErrorAndEmitNoPair)
{
  Config c;
  c.capture.left_sensor_id = 5;
  c.capture.right_sensor_id = 6;
  std::atomic<int> pairs{0};
  std::atomic<int> jpegs{0};
  depthd::StereoCapture capture(c, [](const std::string & line) { std::cerr << "[capture] " << line << "\n"; });
  capture.set_pair_callback([&](const depthd::StereoPair &) { ++pairs; });
  capture.set_rgb_callback([&](const depthd::RgbJpeg &) { ++jpegs; });
  std::string error;
  ASSERT_TRUE(capture.start(&error)) << error;
  EXPECT_EQ(capture.status().state, depthd::CaptureState::starting);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  auto both_failed = [&] {
    const auto st = capture.status();
    return st.state == depthd::CaptureState::camera_error && contains(st.reason, "sensor-id 5") &&
           contains(st.reason, "sensor-id 6");
  };
  while (std::chrono::steady_clock::now() < deadline && !both_failed()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  const auto status = capture.status();
  capture.stop();
  EXPECT_EQ(status.state, depthd::CaptureState::camera_error);
  EXPECT_TRUE(contains(status.reason, "left camera (sensor-id 5)")) << status.reason;
  EXPECT_TRUE(contains(status.reason, "right camera (sensor-id 6)")) << status.reason;
  EXPECT_EQ(pairs.load(), 0);
  EXPECT_EQ(jpegs.load(), 0);
  EXPECT_EQ(capture.pairing_stats().pairs_emitted, 0u);
}

// Live capture on the two IMX219s. Opt-in (DEPTHD_HW_TEST=1): the cameras are exclusive to one Argus client, so the
// default ctest run must not take them from a running depthd service.
class LiveCapture : public ::testing::TestWithParam<bool>
{
};

TEST_P(LiveCapture, PairsLockedFramesAndJpegs)
{
  const char * hw = std::getenv("DEPTHD_HW_TEST");
  if (!hw || std::string(hw) != "1") GTEST_SKIP() << "set DEPTHD_HW_TEST=1 to run on the cameras";
  Config c;
  c.capture.zero_copy = GetParam();
  c.capture.settle_frames = 20;
  c.capture.exposure.mode = "lock_after_settle";
  std::vector<std::string> logs;
  std::mutex mutex;
  // Only metadata is kept: holding the GpuMats would starve the capture's fixed frame pool.
  struct PairInfo
  {
    cv::Size left_size, right_size;
    int left_type = 0;
    int64_t stamp = 0;
    int64_t skew = 0;
  };
  std::vector<PairInfo> pairs;
  cv::Mat last_left;
  std::vector<int64_t> jpeg_stamps;
  std::vector<uint8_t> first_jpeg;
  depthd::StereoCapture capture(c, [&](const std::string & line) {
    std::lock_guard<std::mutex> lock(mutex);
    logs.push_back(line);
    std::cerr << "[capture] " << line << "\n";
  });
  capture.set_pair_callback([&](const depthd::StereoPair & pair) {
    std::lock_guard<std::mutex> lock(mutex);
    pairs.push_back(
      {pair.left.size(), pair.right.size(), pair.left.type(), pair.capture_realtime_ns, pair.right_minus_left_ns});
    if (pairs.size() % 30 == 0) pair.left.download(last_left);
  });
  capture.set_rgb_callback([&](const depthd::RgbJpeg & jpeg) {
    std::lock_guard<std::mutex> lock(mutex);
    if (first_jpeg.empty()) first_jpeg.assign(jpeg.data, jpeg.data + jpeg.size);
    jpeg_stamps.push_back(jpeg.capture_realtime_ns);
  });
  std::string error;
  ASSERT_TRUE(capture.start(&error)) << error;
  // Start-up: phase alignment (right-camera restarts) and AE/AWB settling, bounded.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(45);
  while (std::chrono::steady_clock::now() < deadline &&
         (capture.status().state != depthd::CaptureState::ok || !capture.status().exposure_locked)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    pairs.clear();
    jpeg_stamps.clear();
    last_left.release();
  }
  std::this_thread::sleep_for(std::chrono::seconds(5));
  const auto status = capture.status();
  const auto stats = capture.pairing_stats();
  capture.stop();

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(status.state, depthd::CaptureState::ok) << status.reason;
  EXPECT_TRUE(status.exposure_locked);
  // The caps hold on both cameras, also after the right-camera restarts of the phase alignment.
  ASSERT_TRUE(stats.exposure.left.valid && stats.exposure.right.valid);
  EXPECT_LE(stats.exposure.left.exposure_us, 20000 + 100);
  EXPECT_LE(stats.exposure.right.exposure_us, 20000 + 100);
  EXPECT_LE(stats.exposure.right.gain, 8.0 + 1e-9);
  bool logged_lock = false;
  for (const auto & l : logs) logged_lock |= contains(l, "exposure locked");
  EXPECT_TRUE(logged_lock);
  std::cerr << "pairs=" << stats.pairs_emitted << " skew_drops=" << stats.dropped_skew
            << " orphans=" << stats.dropped_orphan << " skew_hist_cum=";
  for (auto v : stats.skew.cumulative()) std::cerr << v << ",";
  std::cerr << " mean_skew_ms=" << (stats.skew.count ? stats.skew.sum_ms / stats.skew.count : 0.0)
            << " jpegs=" << jpeg_stamps.size() << "\n";
  ASSERT_GT(pairs.size(), 120u);  // 30 fps for 5 s once running
  const int64_t now = now_ns(CLOCK_REALTIME);
  for (const auto & pair : pairs) {
    EXPECT_EQ(pair.left_size, cv::Size(640, 480));
    EXPECT_EQ(pair.right_size, cv::Size(640, 480));
    EXPECT_EQ(pair.left_type, CV_8UC1);
    EXPECT_LE(std::abs(pair.skew), 8000000);
    EXPECT_LT(now - pair.stamp, 6'000'000'000);
    EXPECT_GT(now - pair.stamp, 0);
  }
  ASSERT_FALSE(last_left.empty());
  cv::Scalar mean, stddev;
  cv::meanStdDev(last_left, mean, stddev);
  EXPECT_GT(stddev[0], 1.0) << "flat image: not a real scene";
  // RGB JPEG at ~rgb.fps (10) for 5 s, real JPEG bytes.
  EXPECT_GE(jpeg_stamps.size(), 45u);
  EXPECT_LE(jpeg_stamps.size(), 55u);
  ASSERT_GT(first_jpeg.size(), 1000u);
  EXPECT_EQ(first_jpeg[0], 0xFF);
  EXPECT_EQ(first_jpeg[1], 0xD8);
}

INSTANTIATE_TEST_SUITE_P(ZeroCopy, LiveCapture, ::testing::Values(true, false));

// Continuous AE on the cameras (exposure.mode follow): the right is pinned to the left's exposure and gain, pairs flow
// once they match, both stay within the caps, and the saturation measurement runs. Prints the readbacks every second so
// a scene change during the run (lights on/off) shows the right following.
TEST(LiveExposureFollow, RightFollowsLeftAndPairsFlow)
{
  const char * hw = std::getenv("DEPTHD_HW_TEST");
  if (!hw || std::string(hw) != "1") GTEST_SKIP() << "set DEPTHD_HW_TEST=1 to run on the cameras";
  const char * secs_env = std::getenv("DEPTHD_HW_FOLLOW_S");
  const int run_s = secs_env ? std::max(5, std::atoi(secs_env)) : 5;
  Config c;
  // DEPTHD_HW_SATURATION_LEVEL forces the overexposure path in a dark room: a low level counts most pixels as
  // saturated, so the compensation steps down and the right must follow the left's falling gain.
  if (const char * level = std::getenv("DEPTHD_HW_SATURATION_LEVEL")) c.capture.exposure.saturation_level = std::atoi(level);
  std::vector<std::string> logs;
  std::mutex mutex;
  std::atomic<int> pairs{0};
  depthd::StereoCapture capture(c, [&](const std::string & line) {
    std::lock_guard<std::mutex> lock(mutex);
    logs.push_back(line);
    std::cerr << "[capture] " << line << "\n";
  });
  capture.set_pair_callback([&](const depthd::StereoPair &) { ++pairs; });
  std::string error;
  ASSERT_TRUE(capture.start(&error)) << error;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(45);
  while (std::chrono::steady_clock::now() < deadline && capture.status().state != depthd::CaptureState::ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_EQ(capture.status().state, depthd::CaptureState::ok) << capture.status().reason;
  pairs = 0;
  for (int i = 0; i < run_s; ++i) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto e = capture.pairing_stats().exposure;
    std::cerr << "t=" << i + 1 << "s left " << e.left.exposure_us << " us " << e.left.gain << "x, right "
              << e.right.exposure_us << " us " << e.right.gain << "x, saturated " << e.saturated_fraction
              << ", compensation " << e.compensation << ", pins " << e.follow_pins << ", mismatch drops "
              << e.mismatch_drops << ", pairs " << pairs.load() << "\n";
  }
  const auto stats = capture.pairing_stats();
  const auto status = capture.status();
  capture.stop();
  EXPECT_EQ(status.state, depthd::CaptureState::ok) << status.reason;
  EXPECT_FALSE(status.exposure_locked);
  const auto & e = stats.exposure;
  ASSERT_TRUE(e.left.valid);
  ASSERT_TRUE(e.right.valid);
  EXPECT_GE(e.follow_pins, 1u);
  EXPECT_LE(e.left.exposure_us, 20000 + 100);
  EXPECT_LE(e.left.gain, 8.0 + 1e-9);
  EXPECT_NEAR(static_cast<double>(e.right.exposure_us), static_cast<double>(e.left.exposure_us), 0.06 * e.left.exposure_us + 50);
  EXPECT_NEAR(e.right.gain, e.left.gain, 0.06 * e.left.gain + 0.07);
  EXPECT_TRUE(e.saturation_valid);
  EXPECT_GE(e.saturated_fraction, 0.0);
  EXPECT_LE(e.saturated_fraction, 1.0);
  EXPECT_GT(pairs.load(), 20 * run_s);  // ~30 fps once matched, minus re-pin transitions
  std::lock_guard<std::mutex> lock(mutex);
  bool logged_follow = false;
  for (const auto & l : logs) logged_follow |= contains(l, "right camera follows the left");
  EXPECT_TRUE(logged_follow);
}

// A capture that cannot start reports both cameras failing, so the pipeline publishes nothing of either kind.
TEST(StereoCaptureStatus, StartFailureMarksBothCamerasFailing)
{
  Config c;
  c.capture.sensor_mode = {1000, 1000, 30};
  depthd::StereoCapture cap(c);
  std::string err;
  ASSERT_FALSE(cap.start(&err));
  const auto st = cap.status();
  EXPECT_EQ(st.state, depthd::CaptureState::camera_error);
  EXPECT_TRUE(st.left_error);
  EXPECT_TRUE(st.right_error);
  EXPECT_TRUE(contains(st.reason, "capture.sensor_mode")) << st.reason;
}
