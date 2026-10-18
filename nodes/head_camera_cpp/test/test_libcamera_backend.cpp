// LibcameraBackend as a CameraBackend: it implements every method of the interface (checked at compile time) and,
// with no camera to open (the docker test container has none), every method fails safely through the interface: a
// logged false, nullopt, empty rectangle or Idle, never an exception or a hang. Real-camera behaviour is checked on the
// robot (final-suite-and-runbook).

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "head_camera_cpp/camera_backend.hpp"
#include "head_camera_cpp/libcamera_backend.hpp"

namespace
{

using head_camera_cpp::AfRequest;
using head_camera_cpp::AfState;
using head_camera_cpp::CameraBackend;
using head_camera_cpp::Config;
using head_camera_cpp::LibcameraBackend;
using head_camera_cpp::LogLevel;
using head_camera_cpp::Mode;
using head_camera_cpp::Rect;
using head_camera_cpp::VideoFrame;

constexpr const char * MISSING_CAMERA_ID = "/base/no-such-camera@0";
constexpr std::chrono::milliseconds SHORT_TIMEOUT{20};

// A pure virtual method left without an override makes LibcameraBackend abstract and fails this build.
static_assert(std::is_base_of_v<CameraBackend, LibcameraBackend>, "LibcameraBackend must be a CameraBackend");
static_assert(!std::is_abstract_v<LibcameraBackend>, "LibcameraBackend must implement every CameraBackend method");
static_assert(
  std::is_constructible_v<LibcameraBackend, const Config &>, "LibcameraBackend must be constructible from a Config");

/// A LibcameraBackend on a camera id that does not exist, collecting its log lines.
class LibcameraBackendWithoutCamera : public ::testing::Test
{
protected:
  void SetUp() override
  {
    Config config;
    config.camera_id = MISSING_CAMERA_ID;
    owned_ = std::make_unique<LibcameraBackend>(
      config, [this](LogLevel level, const std::string & message) { lines_.emplace_back(level, message); });
    backend_ = owned_.get();
  }

  /// @return True when an Error line containing `text` was logged.
  bool logged_error(const std::string & text) const
  {
    for (const auto & line : lines_) {
      if (line.first == LogLevel::Error && line.second.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::vector<std::pair<LogLevel, std::string>> lines_;
  std::unique_ptr<LibcameraBackend> owned_;
  CameraBackend * backend_ = nullptr;  // Every call goes through the interface.
};

}  // namespace

TEST_F(LibcameraBackendWithoutCamera, ConstructionLogsTheMissingCamera)
{
  EXPECT_TRUE(logged_error("no-such-camera") || logged_error("CameraManager start failed"));
}

TEST_F(LibcameraBackendWithoutCamera, EveryCameraBackendMethodFailsSafely)
{
  int callbacks = 0;
  backend_->set_video_callback([&callbacks](const VideoFrame &) { ++callbacks; });

  EXPECT_FALSE(backend_->configure(Mode::Video));
  EXPECT_FALSE(backend_->configure(Mode::Still));
  EXPECT_FALSE(backend_->start_video());
  EXPECT_FALSE(backend_->start_still());
  EXPECT_FALSE(backend_->set_af(AfRequest{}));
  EXPECT_EQ(backend_->af_state(), AfState::Idle);
  EXPECT_FALSE(backend_->wait_still_frame(SHORT_TIMEOUT).has_value());
  const Rect crop = backend_->scaler_crop_maximum();
  EXPECT_EQ(crop.width, 0);
  EXPECT_EQ(crop.height, 0);
  EXPECT_FALSE(backend_->reacquire());
  backend_->stop();
  backend_->stop();

  EXPECT_EQ(callbacks, 0);
  EXPECT_TRUE(logged_error("start_still"));
  EXPECT_TRUE(logged_error("set_af while no stream runs"));
  EXPECT_TRUE(logged_error("wait_still_frame"));
  EXPECT_TRUE(logged_error("scaler_crop_maximum"));
}

TEST_F(LibcameraBackendWithoutCamera, DestructionAfterFailuresIsClean)
{
  EXPECT_FALSE(backend_->start_video());
  owned_.reset();
  backend_ = nullptr;
  SUCCEED();
}
