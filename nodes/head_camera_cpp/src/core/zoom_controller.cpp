#include "head_camera_cpp/zoom_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <exception>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "head_camera_cpp/crop.hpp"
#include "head_camera_cpp/encode.hpp"

namespace head_camera_cpp
{

namespace
{

using TimePoint = Clock::TimePoint;
using Duration = Clock::Duration;

/// How often af_state() is polled while AF scans (a 30 fps frame is 33 ms).
constexpr auto AF_POLL_INTERVAL = std::chrono::milliseconds(10);

Duration from_seconds(double seconds)
{
  return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(seconds));
}

/// @return The seconds as short text ("1.5 s").
std::string seconds_text(double seconds)
{
  std::ostringstream text;
  text << seconds << " s";
  return text.str();
}

/// Contiguous phase durations on the injected clock: each lap() is the time since the previous one.
class PhaseTimer
{
public:
  explicit PhaseTimer(Clock & clock) : clock_(clock), start_(clock.now()), mark_(start_) {}

  TimePoint start() const { return start_; }

  /// @return Milliseconds since the previous lap (or the start).
  double lap()
  {
    const TimePoint now = clock_.now();
    const double ms = std::chrono::duration<double, std::milli>(now - mark_).count();
    mark_ = now;
    return ms;
  }

private:
  Clock & clock_;
  TimePoint start_;
  TimePoint mark_;
};

/// Releases the controller's claim when a run ends, however it ends.
class ClaimRelease
{
public:
  explicit ClaimRelease(std::atomic<bool> & busy) : busy_(busy) {}
  ~ClaimRelease() { busy_.store(false); }
  ClaimRelease(const ClaimRelease &) = delete;
  ClaimRelease & operator=(const ClaimRelease &) = delete;

private:
  std::atomic<bool> & busy_;
};

}  // namespace

ZoomController::ZoomController(
  CameraBackend & backend, Clock & clock, const ZoomConfig & config, std::function<void()> fatal)
: backend_(backend), clock_(clock), config_(config), fatal_(std::move(fatal))
{
}

bool ZoomController::try_begin()
{
  bool expected = false;
  return busy_.compare_exchange_strong(expected, true);
}

ZoomOutcome ZoomController::run(const ZoomRequest & request)
{
  if (!try_begin()) {
    return ZoomOutcome{busy_result(request), {}};
  }
  return run_claimed(request);
}

ZoomResult ZoomController::busy_result(const ZoomRequest & request)
{
  ZoomResult result;
  result.request_id = request.request_id;
  result.status = ZoomStatus::Busy;
  result.message = "another zoom is running";
  return result;
}

ZoomOutcome ZoomController::run_claimed(const ZoomRequest & request)
{
  const ClaimRelease release(busy_);
  ZoomOutcome outcome;
  ZoomResult & result = outcome.result;
  result.request_id = request.request_id;
  result.full = config_.still_size;

  PhaseTimer timer(clock_);
  const TimePoint watchdog = timer.start() + from_seconds(config_.watchdog_s);
  const std::string watchdog_message = "watchdog: the zoom exceeded " + seconds_text(config_.watchdog_s);
  // The phase the time since the last lap belongs to; laps are contiguous, so the phases sum to the elapsed time.
  double * phase = &result.phase_ms.af;
  const auto enter = [&](double & next) {
    *phase += timer.lap();
    phase = &next;
  };
  std::string message;

  // Phases 1-3; returns the status, sets message on anything but ok.
  const auto capture = [&]() -> ZoomStatus {
    // 1. AF on the running stream.
    bool focused = true;
    if (request.af) {
      const Rect window = af_window(request.u, request.v, config_.af_window_fraction, backend_.scaler_crop_maximum());
      if (!backend_.set_af(AfRequest{true, window, true})) {
        focused = false;
        message = "autofocus request failed";
      } else {
        const TimePoint deadline = std::min(timer.start() + from_seconds(config_.af_timeout_s), watchdog);
        AfState state = backend_.af_state();
        while (state != AfState::Focused && state != AfState::Failed && clock_.now() < deadline) {
          clock_.sleep_for(std::min<Duration>(AF_POLL_INTERVAL, deadline - clock_.now()));
          state = backend_.af_state();
        }
        result.af_state = to_string(state);
        if (state != AfState::Focused) {
          focused = false;
          message = state == AfState::Failed ? "autofocus failed" : "autofocus timed out";
        }
      }
    }

    // 2. SWITCH to the still mode.
    enter(result.phase_ms.switch_mode);
    const TimePoint switch_deadline = clock_.now() + from_seconds(config_.switch_timeout_s);
    const TimePoint frame_deadline = std::min(switch_deadline, watchdog);
    const auto timed_out = [&]() {
      message =
        clock_.now() >= watchdog ? watchdog_message : "no still frame within " + seconds_text(config_.switch_timeout_s);
      return ZoomStatus::Timeout;
    };
    if (clock_.now() >= watchdog) {
      return timed_out();
    }
    backend_.stop();
    if (!backend_.configure(Mode::Still)) {
      message = "still configuration failed";
      return ZoomStatus::CameraError;
    }
    if (clock_.now() >= frame_deadline) {
      return timed_out();
    }
    if (!backend_.start_still()) {
      message = "still start failed";
      return ZoomStatus::CameraError;
    }

    // Keep the (settle_frames + 1)-th frame: the earlier ones carry unsettled exposure.
    enter(result.phase_ms.capture);
    std::optional<StillFrame> frame;
    for (int taken = 0; taken <= config_.settle_frames; ++taken) {
      const TimePoint now = clock_.now();
      if (now >= frame_deadline) {
        return timed_out();
      }
      frame = backend_.wait_still_frame(std::chrono::ceil<std::chrono::milliseconds>(frame_deadline - now));
      if (!frame) {
        return timed_out();
      }
    }

    // 3. CROP the kept frame before the next backend call invalidates it.
    const StillFrame & still = *frame;
    const Size full = still.view.size;
    result.full = {full.width, full.height};
    result.af_state = to_string(still.af_state);
    result.lens_position = still.lens_position;
    try {
      const Rect region = zoom_region(request.u, request.v, request.zoom, full);
      const Size out = output_size(region, request.max_px);
      outcome.jpeg = crop_yuv420_jpeg(still.view, region, out, config_.jpeg_quality);
      result.crop = std::array<int, 4>{region.x, region.y, region.width, region.height};
      result.out = std::array<int, 2>{out.width, out.height};
      result.stamp_ns = still.stamp_ns;
    } catch (const std::exception & e) {
      message = std::string("crop encoding failed: ") + e.what();
      return ZoomStatus::CameraError;
    }
    return focused ? ZoomStatus::Ok : ZoomStatus::AfFailed;
  };

  ZoomStatus status = ZoomStatus::CameraError;
  try {
    status = capture();
  } catch (const std::exception & e) {
    message = std::string("zoom failed: ") + e.what();
    status = ZoomStatus::CameraError;
  }
  *phase += timer.lap();

  // 4. RESTORE, always.
  const bool restored = restore();
  result.phase_ms.restore = timer.lap();
  if (!restored) {
    status = ZoomStatus::CameraError;
    message = "the video stream could not be restored";
  }

  result.status = status;
  result.message = message;
  if (!carries_image(status)) {
    outcome.jpeg.clear();
    result.crop.reset();
    result.out.reset();
    result.stamp_ns = 0;
  }
  return outcome;
}

bool ZoomController::restore()
{
  if (restore_once()) {
    return true;
  }
  clock_.sleep_for(from_seconds(config_.restore_retry_delay_s));
  if (restore_once()) {
    return true;
  }
  if (backend_.reacquire() && restore_once()) {
    return true;
  }
  fatal_();
  return false;
}

bool ZoomController::restore_once()
{
  backend_.stop();
  return backend_.configure(Mode::Video) && backend_.start_video();
}

}  // namespace head_camera_cpp
