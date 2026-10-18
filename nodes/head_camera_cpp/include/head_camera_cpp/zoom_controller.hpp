#pragma once

// The zoom state machine (design D6). One request at a time:
//   1. AF (skipped when the request has af false): windowed AF with a trigger on the running stream, polled until
//      Focused or Failed or zoom.af_timeout_s. Failed or timed out focus continues as af_failed (the picture is
//      still taken; the caller may retry).
//   2. SWITCH: stop, configure Still, start_still, then frames until the (settle_frames + 1)-th, all within
//      zoom.switch_timeout_s of the phase start (else timeout). A configure or start failure is camera_error.
//   3. CROP: zoom_region + output_size + crop_yuv420_jpeg at zoom.jpeg_quality (an encode error is camera_error).
//   4. RESTORE, always, after success and after every failure: stop, configure Video, set the mapping focus lock
//      (Manual at lens_position when active, else the configured AF mode), start_video. On failure sleep
//      zoom.restore_retry_delay_s and retry once, then reacquire() and try again, then call `fatal` (production:
//      exit(1), so systemd restarts the unit).
// A watchdog of zoom.watchdog_s over phases 1-3 turns into timeout; restore still runs.
// Serves G2, U2.

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <vector>

#include "head_camera_cpp/camera_backend.hpp"
#include "head_camera_cpp/config.hpp"
#include "head_camera_cpp/zoom_json.hpp"

namespace head_camera_cpp
{

/// What a zoom produced: the result and, only when carries_image(result.status), the JPEG.
struct ZoomOutcome
{
  ZoomResult result;
  std::vector<uint8_t> jpeg;
};

/// Runs zoom requests against a camera backend, one at a time.
class ZoomController
{
public:
  /**
   * @param backend The camera; it must be streaming video when a request starts, and streams video again after.
   * @param clock The time source for timeouts, the watchdog and phase_ms.
   * @param config The zoom settings.
   * @param fatal Called when the video stream cannot be restored; the default exits with status 1.
   * @param focus_lock_wanted Whether the mapping focus lock is active; asked at each restore attempt, so RESTORE
   *   returns the lens to the lock (Manual at lens_position) when true and to the configured AF mode when false.
   */
  ZoomController(
    CameraBackend & backend, Clock & clock, const ZoomConfig & config,
    std::function<void()> fatal = [] { std::exit(1); }, std::function<bool()> focus_lock_wanted = [] { return false; });

  /**
   * Claim the controller for one request without blocking.
   *
   * @return true when it was free and is now claimed (follow with run_claimed); false while a request runs.
   */
  bool try_begin();

  /**
   * Run a request: busy at once, without touching the backend, while another request runs.
   *
   * @param request A validated request.
   * @return The outcome; the jpeg is empty unless carries_image(result.status).
   */
  ZoomOutcome run(const ZoomRequest & request);

  /**
   * Run a request after a successful try_begin(), and release the claim when done. Lets a caller answer busy on
   * its own thread and hand the claimed request to a zoom thread.
   *
   * @param request A validated request.
   * @return The outcome; the jpeg is empty unless carries_image(result.status).
   */
  ZoomOutcome run_claimed(const ZoomRequest & request);

  /**
   * @param request The request answered.
   * @return Whether a request is running (claimed or in progress).
   */
  bool busy() const { return busy_.load(); }

  /**
   * @param request The request answered.
   * @return The busy result for it (no image, every phase 0 ms).
   */
  static ZoomResult busy_result(const ZoomRequest & request);

private:
  /// Stop, configure Video, start_video, retried as described above; calls fatal_ when everything fails.
  /// @return true when the video stream runs again.
  bool restore();

  /// One restore attempt: stop, configure Video, set the focus lock the caller wants, start_video.
  /// @return true when the video stream runs.
  bool restore_once();

  CameraBackend & backend_;
  Clock & clock_;
  ZoomConfig config_;
  std::function<void()> fatal_;
  std::function<bool()> focus_lock_wanted_;
  std::atomic<bool> busy_{false};
};

}  // namespace head_camera_cpp
