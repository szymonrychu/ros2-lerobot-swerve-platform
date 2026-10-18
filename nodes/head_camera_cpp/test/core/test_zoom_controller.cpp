#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fake_camera_backend.hpp"
#include "head_camera_cpp/camera_backend.hpp"
#include "head_camera_cpp/config.hpp"
#include "head_camera_cpp/crop.hpp"
#include "head_camera_cpp/zoom_controller.hpp"
#include "head_camera_cpp/zoom_json.hpp"

namespace
{

using head_camera_cpp::af_window;
using head_camera_cpp::AfState;
using head_camera_cpp::carries_image;
using head_camera_cpp::output_size;
using head_camera_cpp::Rect;
using head_camera_cpp::Size;
using head_camera_cpp::zoom_region;
using head_camera_cpp::ZoomConfig;
using head_camera_cpp::ZoomController;
using head_camera_cpp::ZoomOutcome;
using head_camera_cpp::ZoomRequest;
using head_camera_cpp::ZoomStatus;
using head_camera_cpp::testing::FakeCameraBackend;
using head_camera_cpp::testing::FakeClock;
using std::chrono::milliseconds;
using Calls = std::vector<std::string>;

constexpr Size STILL{640, 360};
constexpr double U = 0.5;
constexpr double V = 0.5;
constexpr int ZOOM = 2;
constexpr int MAX_PX = 256;
const Calls RESTORE_CALLS{"stop", "configure(Video)", "start_video"};

ZoomConfig test_config()
{
  ZoomConfig config;
  config.still_size = {STILL.width, STILL.height};
  return config;
}

ZoomRequest request(bool af = true)
{
  ZoomRequest r;
  r.request_id = "r1";
  r.u = U;
  r.v = V;
  r.zoom = ZOOM;
  r.max_px = MAX_PX;
  r.af = af;
  return r;
}

double ms(FakeClock::Duration d) { return std::chrono::duration<double, std::milli>(d).count(); }

// Last n calls of the log.
Calls tail(const Calls & calls, size_t n)
{
  return n > calls.size() ? calls : Calls(calls.end() - static_cast<std::ptrdiff_t>(n), calls.end());
}

Size decoded_size(const std::vector<uint8_t> & jpeg)
{
  const cv::Mat image = cv::imdecode(jpeg, cv::IMREAD_COLOR);
  return Size{image.cols, image.rows};
}

// One fixture: a fake clock, a fake backend on it and a controller with a fatal counter.
class ZoomControllerTest : public ::testing::Test
{
protected:
  FakeClock clock;
  FakeCameraBackend backend{STILL, &clock};
  ZoomConfig config = test_config();
  int fatal_calls = 0;

  ZoomController controller()
  {
    return ZoomController(backend, clock, config, [this] { ++fatal_calls; });
  }

  void expect_restored(const Calls & calls)
  {
    EXPECT_EQ(tail(calls, RESTORE_CALLS.size()), RESTORE_CALLS);
    EXPECT_TRUE(backend.video_running());
    EXPECT_EQ(fatal_calls, 0);
  }

  void expect_no_image(const ZoomOutcome & outcome)
  {
    EXPECT_TRUE(outcome.jpeg.empty());
    EXPECT_FALSE(outcome.result.out.has_value());
    EXPECT_EQ(outcome.result.stamp_ns, 0);
  }
};

}  // namespace

TEST_F(ZoomControllerTest, HappyPathRunsPhasesInOrderAndReturnsJpeg)
{
  backend.af_states = {AfState::Scanning, AfState::Scanning, AfState::Focused};
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request());

  const Calls expected{
    "scaler_crop_maximum", "set_af",      "af_state",         "af_state",         "af_state",         "stop",
    "configure(Still)",    "start_still", "wait_still_frame", "wait_still_frame", "wait_still_frame", "stop",
    "configure(Video)",    "start_video"};
  EXPECT_EQ(backend.calls(), expected);

  const auto & r = outcome.result;
  EXPECT_EQ(r.request_id, "r1");
  EXPECT_EQ(r.status, ZoomStatus::Ok);
  EXPECT_EQ(r.af_state, "focused");
  ASSERT_TRUE(r.lens_position.has_value());
  EXPECT_DOUBLE_EQ(*r.lens_position, 2.5);
  const Rect region = zoom_region(U, V, ZOOM, STILL);
  const Size out = output_size(region, MAX_PX);
  ASSERT_TRUE(r.crop.has_value());
  EXPECT_EQ(*r.crop, (std::array<int, 4>{region.x, region.y, region.width, region.height}));
  EXPECT_EQ(r.full, (std::array<int, 2>{STILL.width, STILL.height}));
  ASSERT_TRUE(r.out.has_value());
  EXPECT_EQ(*r.out, (std::array<int, 2>{out.width, out.height}));
  ASSERT_EQ(backend.still_stamps().size(), 3U);
  EXPECT_EQ(r.stamp_ns, backend.still_stamps().back());

  ASSERT_FALSE(outcome.jpeg.empty());
  const Size decoded = decoded_size(outcome.jpeg);
  EXPECT_EQ(decoded.width, out.width);
  EXPECT_EQ(decoded.height, out.height);

  const auto requests = backend.af_requests();
  ASSERT_EQ(requests.size(), 1U);
  const Rect window = af_window(U, V, config.af_window_fraction, backend.scaler_crop);
  EXPECT_TRUE(requests[0].windowed);
  EXPECT_TRUE(requests[0].trigger);
  EXPECT_EQ(requests[0].window.x, window.x);
  EXPECT_EQ(requests[0].window.y, window.y);
  EXPECT_EQ(requests[0].window.width, window.width);
  EXPECT_EQ(requests[0].window.height, window.height);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, AfFalseSetsNoAf)
{
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::Ok);
  EXPECT_FALSE(outcome.jpeg.empty());
  EXPECT_EQ(backend.count("set_af"), 0);
  EXPECT_EQ(backend.count("af_state"), 0);
  EXPECT_EQ(backend.count("scaler_crop_maximum"), 0);
  EXPECT_EQ(backend.calls().front(), "stop");
  EXPECT_DOUBLE_EQ(outcome.result.phase_ms.af, 0.0);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, AfFailedGivesAfFailedWithImage)
{
  backend.af_states = {AfState::Scanning, AfState::Failed};
  backend.still_af_state = AfState::Failed;
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request());

  EXPECT_EQ(outcome.result.status, ZoomStatus::AfFailed);
  EXPECT_TRUE(carries_image(outcome.result.status));
  EXPECT_FALSE(outcome.jpeg.empty());
  EXPECT_EQ(outcome.result.af_state, "failed");
  EXPECT_EQ(backend.count("af_state"), 2);
  EXPECT_EQ(backend.count("wait_still_frame"), 3);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, AfTimeoutGivesAfFailedWithImage)
{
  backend.af_states = {AfState::Scanning};
  backend.still_af_state = AfState::Scanning;
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request());

  EXPECT_EQ(outcome.result.status, ZoomStatus::AfFailed);
  EXPECT_FALSE(outcome.jpeg.empty());
  EXPECT_GE(outcome.result.phase_ms.af, config.af_timeout_s * 1000.0);
  EXPECT_LT(outcome.result.phase_ms.af, config.af_timeout_s * 1000.0 + 100.0);
  EXPECT_EQ(backend.count("wait_still_frame"), 3);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, SetAfFailureContinuesAsAfFailed)
{
  backend.fail("set_af");
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request());

  EXPECT_EQ(outcome.result.status, ZoomStatus::AfFailed);
  EXPECT_FALSE(outcome.jpeg.empty());
  EXPECT_EQ(backend.count("af_state"), 0);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, SwitchTimeoutGivesTimeoutWithoutImageAndRestores)
{
  backend.still_frame_budget = 1;
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::Timeout);
  expect_no_image(outcome);
  EXPECT_FALSE(outcome.result.crop.has_value());
  EXPECT_GE(outcome.result.phase_ms.switch_mode + outcome.result.phase_ms.capture, config.switch_timeout_s * 1000.0);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, ConfigureStillFailureGivesCameraErrorAndRestores)
{
  backend.fail("configure(Still)");
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::CameraError);
  expect_no_image(outcome);
  EXPECT_EQ(backend.count("start_still"), 0);
  EXPECT_EQ(backend.count("wait_still_frame"), 0);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, StartStillFailureGivesCameraErrorAndRestores)
{
  backend.fail("start_still");
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::CameraError);
  expect_no_image(outcome);
  EXPECT_EQ(backend.count("wait_still_frame"), 0);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, EncodeErrorGivesCameraErrorAndRestores)
{
  backend.corrupt_still_frames = true;
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::CameraError);
  expect_no_image(outcome);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, RestoreFailsOnceThenSucceedsWithoutFatal)
{
  backend.fail("start_video");
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::Ok);
  EXPECT_FALSE(outcome.jpeg.empty());
  EXPECT_EQ(backend.count("start_video"), 2);
  EXPECT_EQ(backend.count("reacquire"), 0);
  const auto sleeps = clock.sleeps();
  ASSERT_FALSE(sleeps.empty());
  EXPECT_DOUBLE_EQ(ms(sleeps.back()), config.restore_retry_delay_s * 1000.0);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, RestoreFailsTwiceThenReacquiresAndSucceeds)
{
  backend.fail("configure(Video)", 2);
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::Ok);
  EXPECT_EQ(backend.count("configure(Video)"), 3);
  EXPECT_EQ(backend.count("reacquire"), 1);
  const Calls expected_tail{"reacquire", "stop", "configure(Video)", "start_video"};
  EXPECT_EQ(tail(backend.calls(), expected_tail.size()), expected_tail);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, ReacquireFailureCallsFatalOnce)
{
  backend.fail("start_video", 2);
  backend.fail("reacquire");
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(fatal_calls, 1);
  EXPECT_EQ(backend.count("reacquire"), 1);
  EXPECT_EQ(backend.calls().back(), "reacquire");
  EXPECT_EQ(outcome.result.status, ZoomStatus::CameraError);
  expect_no_image(outcome);
}

TEST_F(ZoomControllerTest, RestoreFailingAfterReacquireCallsFatalOnce)
{
  backend.fail("start_video", 3);
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(fatal_calls, 1);
  EXPECT_EQ(backend.count("reacquire"), 1);
  EXPECT_EQ(backend.count("start_video"), 3);
  EXPECT_EQ(outcome.result.status, ZoomStatus::CameraError);
  expect_no_image(outcome);
}

TEST_F(ZoomControllerTest, WatchdogGivesTimeoutAndRestores)
{
  // AF takes about 1 s, then the still configuration hangs for 5.5 s: past the 6 s watchdog.
  backend.af_states = std::vector<AfState>(100, AfState::Scanning);
  backend.af_states.back() = AfState::Focused;
  backend.delay("configure(Still)", std::chrono::milliseconds(5500));
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request());

  EXPECT_EQ(outcome.result.status, ZoomStatus::Timeout);
  expect_no_image(outcome);
  EXPECT_EQ(backend.count("start_still"), 0);
  EXPECT_EQ(backend.count("wait_still_frame"), 0);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, WatchdogBoundsFrameWaits)
{
  // The watchdog (1 s) runs out before the switch timeout (1.5 s) would: no frame wait goes past it.
  config.watchdog_s = 1.0;
  backend.still_frame_budget = 0;
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request(false));

  EXPECT_EQ(outcome.result.status, ZoomStatus::Timeout);
  expect_no_image(outcome);
  const auto & p = outcome.result.phase_ms;
  EXPECT_LE(p.af + p.switch_mode + p.capture, config.watchdog_s * 1000.0);
  EXPECT_GE(backend.count("wait_still_frame"), 1);
  expect_restored(backend.calls());
}

TEST_F(ZoomControllerTest, ConcurrentRunGetsBusyWithoutTouchingBackend)
{
  auto zoom = controller();
  std::optional<ZoomOutcome> second;
  size_t calls_before = 0;
  size_t calls_after = 0;
  bool claim_while_running = true;
  backend.on_call = [&](const std::string & name) {
    if (name != "set_af" || second) {
      return;
    }
    calls_before = backend.calls().size();
    ZoomRequest other = request();
    other.request_id = "r2";
    std::thread thread([&] {
      second = zoom.run(other);
      claim_while_running = zoom.try_begin();
    });
    thread.join();
    calls_after = backend.calls().size();
  };
  backend.af_states = {AfState::Focused};
  const ZoomOutcome first = zoom.run(request());

  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->result.request_id, "r2");
  EXPECT_EQ(second->result.status, ZoomStatus::Busy);
  EXPECT_TRUE(second->jpeg.empty());
  EXPECT_DOUBLE_EQ(second->result.phase_ms.af, 0.0);
  EXPECT_DOUBLE_EQ(second->result.phase_ms.restore, 0.0);
  EXPECT_EQ(calls_after, calls_before);
  EXPECT_FALSE(claim_while_running);
  EXPECT_EQ(first.result.status, ZoomStatus::Ok);

  // Free again afterwards: a claim then run_claimed works and releases.
  ASSERT_TRUE(zoom.try_begin());
  EXPECT_EQ(zoom.run_claimed(request(false)).result.status, ZoomStatus::Ok);
  EXPECT_TRUE(zoom.try_begin());
}

TEST_F(ZoomControllerTest, SettleFramesHonoured)
{
  for (int settle : {0, 4}) {
    FakeClock local_clock;
    FakeCameraBackend local{STILL, &local_clock};
    config.settle_frames = settle;
    ZoomController zoom(local, local_clock, config, [this] { ++fatal_calls; });
    const ZoomOutcome outcome = zoom.run(request(false));

    EXPECT_EQ(outcome.result.status, ZoomStatus::Ok);
    EXPECT_EQ(local.count("wait_still_frame"), settle + 1);
    ASSERT_EQ(local.still_stamps().size(), static_cast<size_t>(settle + 1));
    EXPECT_EQ(outcome.result.stamp_ns, local.still_stamps().back());
  }
}

TEST_F(ZoomControllerTest, PhaseMsNonNegativeAndSumToElapsed)
{
  backend.af_states = {AfState::Scanning, AfState::Scanning, AfState::Focused};
  backend.delay("set_af", milliseconds(7));
  backend.delay("stop", milliseconds(40));
  backend.delay("configure(Still)", milliseconds(120));
  backend.delay("start_still", milliseconds(30));
  backend.delay("configure(Video)", milliseconds(90));
  backend.delay("start_video", milliseconds(25));
  backend.still_frame_interval = milliseconds(70);
  auto zoom = controller();
  const ZoomOutcome outcome = zoom.run(request());

  ASSERT_EQ(outcome.result.status, ZoomStatus::Ok);
  const auto & p = outcome.result.phase_ms;
  EXPECT_GT(p.af, 0.0);
  EXPECT_GT(p.switch_mode, 0.0);
  EXPECT_GT(p.capture, 0.0);
  EXPECT_GT(p.restore, 0.0);
  EXPECT_DOUBLE_EQ(p.restore, 40.0 + 90.0 + 25.0);
  EXPECT_NEAR(p.af + p.switch_mode + p.capture + p.restore, ms(clock.elapsed()), 1e-6);
}

TEST(ZoomControllerBusyResult, CarriesRequestIdAndNoImage)
{
  const auto result = ZoomController::busy_result(request());
  EXPECT_EQ(result.request_id, "r1");
  EXPECT_EQ(result.status, ZoomStatus::Busy);
  EXPECT_FALSE(result.out.has_value());
  EXPECT_FALSE(result.crop.has_value());
  EXPECT_EQ(result.stamp_ns, 0);
}
