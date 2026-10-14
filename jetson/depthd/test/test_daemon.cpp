// The depthd service around the pipeline: --check-config, startup errors that keep /health up instead of crash
// looping, the HTTP hooks wired to the pipeline, the retried camera start, and SIGTERM within 2 s.
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <climits>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "../third_party/httplib.h"
#include "depthd/daemon.hpp"
#include "fake_frame_source.hpp"

namespace fs = std::filesystem;

namespace
{
using nlohmann::json;
const std::string kSource = DEPTHD_SOURCE_DIR;
const std::string kFixtures = kSource + "/test/fixtures/";

fs::path temp_dir()
{
  const fs::path dir = fs::temp_directory_path() / ("depthd_daemon_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  return dir;
}

std::string write_config(const std::string & name, const std::string & yaml)
{
  const fs::path path = temp_dir() / name;
  std::ofstream(path.string()) << yaml;
  return path.string();
}

std::string valid_config(const std::string & calibration)
{
  return write_config(
    "valid_" + std::to_string(std::hash<std::string>{}(calibration)) + ".yaml",
    "processing:\n  num_disparities: 64\ncalibration:\n  path: " + calibration + "\n");
}

// The depthd executable next to this test binary (ctest runs tests from the build directory).
std::string depthd_binary()
{
  char buf[PATH_MAX] = {};
  const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  return (fs::path(std::string(buf, n > 0 ? static_cast<size_t>(n) : 0)).parent_path() / "depthd").string();
}

json get_health(int port)
{
  httplib::Client client("127.0.0.1", port);
  auto res = client.Get("/health");
  if (!res || res->status != 200) return json();
  return json::parse(res->body);
}

depthd::DaemonOptions test_options(const std::string & config_path, depthd_test::FakeFrameSource ** source_out)
{
  depthd::DaemonOptions o;
  o.config_path = config_path;
  o.bind_address = "127.0.0.1";
  o.http_port = 0;
  o.pub_port = 0;
  o.retry_interval_ms = 50;
  o.source_retry_interval_ms = 100;
  o.pipeline.kmsg_path = "";
  o.make_source = [source_out](const depthd::Config &, depthd::LogFn) {
    auto s = std::make_unique<depthd_test::FakeFrameSource>();
    *source_out = s.get();
    return std::unique_ptr<depthd::FrameSource>(std::move(s));
  };
  return o;
}

template <class Pred>
bool eventually(Pred pred, std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return pred();
}
}  // namespace

TEST(Daemon, CheckConfigAcceptsAValidFileAndNamesTheBadKey)
{
  std::string message;
  EXPECT_EQ(depthd::check_config(kSource + "/config/depthd.yaml", message), 0);
  EXPECT_NE(message.find("ok"), std::string::npos) << message;

  const std::string bad = write_config("bad_width.yaml", "processing:\n  width: 641\n");
  EXPECT_EQ(depthd::check_config(bad, message), 1);
  EXPECT_NE(message.find("processing.width"), std::string::npos) << message;

  EXPECT_EQ(depthd::check_config((temp_dir() / "missing.yaml").string(), message), 1);
  EXPECT_FALSE(message.empty());
}

TEST(Daemon, CheckConfigExecutableExitCodes)
{
  const std::string bin = depthd_binary();
  ASSERT_TRUE(fs::exists(bin)) << bin;
  const std::string bad = write_config("bad_matcher.yaml", "processing:\n  matcher: nope\n");
  EXPECT_EQ(WEXITSTATUS(std::system((bin + " --check-config " + kSource + "/config/depthd.yaml").c_str())), 0);
  EXPECT_EQ(WEXITSTATUS(std::system((bin + " --check-config " + bad + " 2>/dev/null").c_str())), 1);
  EXPECT_EQ(WEXITSTATUS(std::system((bin + " --no-such-flag 2>/dev/null").c_str())), 2);
}

TEST(Daemon, ConfigErrorKeepsServingHealthWithTheReason)
{
  depthd_test::FakeFrameSource * source = nullptr;
  const std::string bad = write_config("bad_rgb.yaml", "rgb:\n  jpeg_quality: 0\n");
  depthd::Daemon daemon(test_options(bad, &source));
  daemon.start();
  ASSERT_TRUE(eventually([&] { return daemon.http_port() > 0; }));
  const json h = get_health(daemon.http_port());
  ASSERT_TRUE(h.is_object());
  EXPECT_EQ(h["state"], "config_error");
  EXPECT_NE(h["reason"].get<std::string>().find("rgb.jpeg_quality"), std::string::npos) << h.dump();
  EXPECT_EQ(source, nullptr) << "no camera is opened with a bad config";
  EXPECT_EQ(daemon.pub_port(), 0) << "nothing is published with a bad config";
  daemon.stop();
}

TEST(Daemon, MissingCalibrationIsUncalibratedWithTheReasonAndRgbStillFlows)
{
  depthd_test::FakeFrameSource * source = nullptr;
  depthd::Daemon daemon(test_options(valid_config(kFixtures + "absent.yaml"), &source));
  daemon.start();
  ASSERT_TRUE(eventually([&] { return daemon.http_port() > 0 && daemon.pub_port() > 0 && source != nullptr; }));
  const json h = get_health(daemon.http_port());
  EXPECT_EQ(h["state"], "uncalibrated");
  EXPECT_NE(h["reason"].get<std::string>().find("absent.yaml"), std::string::npos) << h.dump();
  EXPECT_EQ(h["matcher"], "cuda_sgm");

  depthd_test::Sub sub(daemon.pub_port(), 1000);
  source->push_rgb(depthd_test::make_jpeg(640, 480, depthd_test::realtime_ns(), 1));
  ros2_common_cpp::depth_wire::Decoded msg;
  ASSERT_TRUE(sub.recv(msg));
  EXPECT_EQ(msg.header.kind, ros2_common_cpp::depth_wire::Kind::kRgb);
  EXPECT_FALSE(msg.header.calibrated);
  daemon.stop();
}

TEST(Daemon, CalibratedDaemonPublishesDepthAndServesTheCapturePair)
{
  depthd_test::FakeFrameSource * source = nullptr;
  depthd::Daemon daemon(test_options(valid_config(kFixtures + "stereo_valid.yaml"), &source));
  daemon.start();
  ASSERT_TRUE(eventually([&] { return daemon.http_port() > 0 && daemon.pub_port() > 0 && source != nullptr; }));
  depthd_test::Sub sub(daemon.pub_port(), 1500);

  const depthd::CalibrationLoad load =
    depthd::load_calibration(kFixtures + "stereo_valid.yaml", cv::Size(640, 480), 0.5);
  ASSERT_TRUE(load.ok());
  const auto raw = depthd_test::make_raw_pair(*load.calibration, 25.0);
  source->push_pair(depthd_test::upload_pair(raw, depthd_test::realtime_ns(), 3));
  ros2_common_cpp::depth_wire::Decoded msg;
  ASSERT_TRUE(sub.recv(msg));
  EXPECT_EQ(msg.header.kind, ros2_common_cpp::depth_wire::Kind::kDepth);
  EXPECT_TRUE(eventually([&] { return get_health(daemon.http_port())["state"] == "ok"; }));

  source->push_pair(depthd_test::upload_pair(raw, depthd_test::realtime_ns(), 4));
  httplib::Client client("127.0.0.1", daemon.http_port());
  auto res = client.Get("/calibration/capture");
  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  auto put = client.Put("/config", R"({"rgb": {"jpeg_quality": 61}})", "application/json");
  ASSERT_TRUE(put);
  EXPECT_EQ(put->status, 200);
  EXPECT_EQ(source->jpeg_quality(), 61);
  daemon.stop();
}

TEST(Daemon, CameraStartFailureIsRetriedAndReported)
{
  depthd_test::FakeFrameSource * source = nullptr;
  depthd::DaemonOptions o = test_options(valid_config(kFixtures + "stereo_valid.yaml"), &source);
  auto make = o.make_source;
  o.make_source = [make](const depthd::Config & c, depthd::LogFn log) {
    auto s = make(c, log);
    auto * fake = static_cast<depthd_test::FakeFrameSource *>(s.get());
    fake->set_start_error("left camera (sensor-id 0): no camera available");
    depthd::CaptureStatus st;
    st.state = depthd::CaptureState::camera_error;
    st.reason = "left camera (sensor-id 0): no camera available";
    st.left_error = st.right_error = true;
    fake->set_status(st);
    return s;
  };
  depthd::Daemon daemon(o);
  daemon.start();
  ASSERT_TRUE(eventually([&] { return source != nullptr && source->start_calls() >= 3; }))
    << "the camera start is retried";
  const depthd::Health h = daemon.health();
  EXPECT_EQ(h.state, "camera_error");
  EXPECT_NE(h.reason.find("no camera available"), std::string::npos);
  source->set_start_error("");
  source->set_status({});
  const int calls = source->start_calls();
  ASSERT_TRUE(eventually([&] { return source->start_calls() == calls + 1; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(source->start_calls(), calls + 1) << "a started source is not restarted";
  daemon.stop();
}

TEST(Daemon, StopIsPromptInProcess)
{
  depthd_test::FakeFrameSource * source = nullptr;
  depthd::Daemon daemon(test_options(valid_config(kFixtures + "stereo_valid.yaml"), &source));
  daemon.start();
  ASSERT_TRUE(eventually([&] { return daemon.pub_port() > 0; }));
  const auto t0 = std::chrono::steady_clock::now();
  daemon.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(2));
}

TEST(Daemon, ExecutableStaysUpOnAConfigErrorAndStopsOnSigtermWithin2s)
{
  const std::string bin = depthd_binary();
  ASSERT_TRUE(fs::exists(bin)) << bin;
  const std::string bad = write_config("bad_for_exe.yaml", "processing:\n  num_disparities: 7\n");
  const pid_t pid = ::fork();
  ASSERT_GE(pid, 0);
  if (pid == 0) {
    ::execl(bin.c_str(), bin.c_str(), "--config", bad.c_str(), static_cast<char *>(nullptr));
    ::_exit(127);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
  int status = 0;
  ASSERT_EQ(::waitpid(pid, &status, WNOHANG), 0) << "depthd exited on a config error instead of staying up";
  const auto t0 = std::chrono::steady_clock::now();
  ::kill(pid, SIGTERM);
  pid_t done = 0;
  while (done == 0 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(2)) {
    done = ::waitpid(pid, &status, WNOHANG);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (done == 0) {
    ::kill(pid, SIGKILL);
    ::waitpid(pid, &status, 0);
    FAIL() << "depthd did not stop within 2 s of SIGTERM";
  }
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}
