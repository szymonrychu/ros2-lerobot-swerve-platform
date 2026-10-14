// Health: the state precedence of /health, the GPU temperature reader (/sys thermal zones), the nvgpu error counter
// (/dev/kmsg records) and the frame rate meter.
#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "depthd/health.hpp"

namespace fs = std::filesystem;

namespace
{
fs::path temp_dir(const std::string & name)
{
  const fs::path dir = fs::temp_directory_path() / ("depthd_health_" + name + "_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

void write_file(const fs::path & path, const std::string & text)
{
  fs::create_directories(path.parent_path());
  std::ofstream(path.string()) << text;
}

depthd::HealthInputs healthy()
{
  depthd::HealthInputs in;
  in.capture.state = depthd::CaptureState::ok;
  in.calibrated = true;
  in.depth_published = true;
  in.matcher = "cuda_sgm";
  in.mono_state = "disabled";
  in.fps = 14.5;
  return in;
}
}  // namespace

TEST(Health, OkWhenCalibratedCamerasRunningAndDepthFlowing)
{
  const depthd::Health h = depthd::compose_health(healthy());
  EXPECT_EQ(h.state, "ok");
  EXPECT_EQ(h.reason, "");
  EXPECT_DOUBLE_EQ(h.fps, 14.5);
  EXPECT_EQ(h.matcher, "cuda_sgm");
  EXPECT_EQ(h.mono_state, "disabled");
}

TEST(Health, ConfigErrorWinsOverEverything)
{
  depthd::HealthInputs in = healthy();
  in.config_error = "processing.width: must be a multiple of 8";
  in.capture.state = depthd::CaptureState::camera_error;
  in.calibrated = false;
  const depthd::Health h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "config_error");
  EXPECT_EQ(h.reason, "processing.width: must be a multiple of 8");
}

TEST(Health, CameraErrorBeforeUncalibrated)
{
  depthd::HealthInputs in = healthy();
  in.capture.state = depthd::CaptureState::camera_error;
  in.capture.reason = "left camera (sensor-id 0): stream ended";
  in.calibrated = false;
  in.calibration_reason = "calibration file /var/lib/depthd/stereo.yaml is missing or unreadable";
  const depthd::Health h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "camera_error");
  EXPECT_EQ(h.reason, "left camera (sensor-id 0): stream ended");
}

TEST(Health, UncalibratedNamesTheCalibrationReason)
{
  depthd::HealthInputs in = healthy();
  in.calibrated = false;
  in.depth_published = false;
  in.calibration_reason = "rms: 0.9 px exceeds calibration.max_rms_px 0.5";
  depthd::Health h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "uncalibrated");
  EXPECT_EQ(h.reason, "rms: 0.9 px exceeds calibration.max_rms_px 0.5");
  in.calibration_reason.clear();
  h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "uncalibrated");
  EXPECT_EQ(h.reason, "no calibration loaded");
}

TEST(Health, StartingUntilTheFirstDepthFrame)
{
  depthd::HealthInputs in = healthy();
  in.capture.state = depthd::CaptureState::starting;
  in.capture.reason = "waiting for the first stereo pair";
  in.depth_published = false;
  depthd::Health h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "starting");
  EXPECT_EQ(h.reason, "waiting for the first stereo pair");

  in.capture.state = depthd::CaptureState::ok;
  in.capture.reason.clear();
  h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "starting");
  EXPECT_EQ(h.reason, "waiting for the first depth frame");

  in.startup_reason = "binding the ZeroMQ PUB on tcp://10.77.0.2:5550 failed";
  h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "starting");
  EXPECT_EQ(h.reason, "binding the ZeroMQ PUB on tcp://10.77.0.2:5550 failed");
}

TEST(Health, OkStateNamesADepthStall)
{
  depthd::HealthInputs in = healthy();
  in.last_depth_age_ms = 3500.0;
  const depthd::Health h = depthd::compose_health(in);
  EXPECT_EQ(h.state, "ok");
  EXPECT_EQ(h.reason, "no depth frame for 3.5 s");
}

TEST(Health, GpuTemperatureFromTheGpuThermZone)
{
  const fs::path root = temp_dir("thermal");
  write_file(root / "thermal_zone0" / "type", "CPU-therm\n");
  write_file(root / "thermal_zone0" / "temp", "23500\n");
  write_file(root / "thermal_zone2" / "type", "GPU-therm\n");
  write_file(root / "thermal_zone2" / "temp", "41500\n");
  const auto t = depthd::read_gpu_temp_c(root.string());
  ASSERT_TRUE(t.has_value());
  EXPECT_DOUBLE_EQ(*t, 41.5);
  EXPECT_FALSE(depthd::read_gpu_temp_c((root / "missing").string()).has_value());
  fs::remove_all(root);
}

TEST(Health, GpuTemperatureOnThisBoard)
{
  if (!fs::exists("/sys/devices/virtual/thermal/thermal_zone2/type")) GTEST_SKIP() << "not a Jetson";
  const auto t = depthd::read_gpu_temp_c();
  ASSERT_TRUE(t.has_value());
  EXPECT_GT(*t, 0.0);
  EXPECT_LT(*t, 110.0);
}

TEST(Health, NvgpuErrorRecognition)
{
  EXPECT_TRUE(depthd::is_nvgpu_error("3,1234,5678,-;nvgpu: 57000000.gpu gk20a_fecs_intr: fecs error"));
  EXPECT_TRUE(depthd::is_nvgpu_error("2,1,2,-;gk20a 57000000.gpu: mmu fault"));
  EXPECT_TRUE(depthd::is_nvgpu_error("11,1,2,-;nvgpu: ctxsw timeout"));  // facility 1, level 3
  EXPECT_FALSE(depthd::is_nvgpu_error("6,1,2,-;nvgpu: 57000000.gpu railgating enabled"));  // info
  EXPECT_FALSE(depthd::is_nvgpu_error("3,1,2,-;sd 0:0:0:0: [sda] error"));                 // not the GPU
  EXPECT_FALSE(depthd::is_nvgpu_error(" SUBSYSTEM=nvgpu"));                              // continuation line
  EXPECT_FALSE(depthd::is_nvgpu_error("garbage"));
}

TEST(Health, NvgpuErrorCounterCountsRecordsAndFollowsAppends)
{
  const fs::path dir = temp_dir("kmsg");
  const fs::path log = dir / "kmsg";
  write_file(log, "3,1,10,-;nvgpu: fecs error\n SUBSYSTEM=platform\n6,2,11,-;usb 1-1: new device\n");
  depthd::NvgpuErrorCounter counter(log.string());
  EXPECT_TRUE(counter.available());
  EXPECT_EQ(counter.poll(), 1U);
  {
    std::ofstream(log.string(), std::ios::app) << "3,3,12,-;nvgpu: gk20a_fifo_handle_sched_error\n";
  }
  EXPECT_EQ(counter.poll(), 2U);
  EXPECT_EQ(counter.poll(), 2U);
  EXPECT_EQ(counter.total(), 2U);

  depthd::NvgpuErrorCounter missing((dir / "nope").string());
  EXPECT_FALSE(missing.available());
  EXPECT_NE(missing.error(), "");
  EXPECT_EQ(missing.poll(), 0U);
  fs::remove_all(dir);
}

TEST(Health, NvgpuErrorCounterReadsTheKernelLog)
{
  if (::access("/dev/kmsg", R_OK) != 0) GTEST_SKIP() << "/dev/kmsg not readable";
  depthd::NvgpuErrorCounter counter;
  EXPECT_TRUE(counter.available()) << counter.error();
  counter.poll();  // must return without blocking
  SUCCEED();
}

TEST(Health, RateMeterOverRecentFrames)
{
  depthd::RateMeter meter;
  EXPECT_DOUBLE_EQ(meter.rate(0), 0.0);
  const int64_t period = 100000000;  // 10 fps
  for (int i = 0; i < 20; ++i) meter.tick(i * period);
  EXPECT_NEAR(meter.rate(19 * period), 10.0, 1e-9);
  EXPECT_DOUBLE_EQ(meter.rate(19 * period + depthd::RateMeter::kStaleNs + 1), 0.0);
}
