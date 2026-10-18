#pragma once

// The camera-backend-interface contract (design D6): what the zoom state machine and the ROS node need from a
// camera, without any libcamera type. LibcameraBackend implements it on the Pi; test/core/fake_camera_backend.hpp
// implements it for tests. Serves G1, G2.
//
// Threading: every method except the video callback is called from one thread at a time (the zoom thread, or the
// node while no zoom runs). The video callback runs on the backend's own thread and must not block.

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <thread>

#include "head_camera_cpp/crop.hpp"
#include "head_camera_cpp/encode.hpp"

namespace head_camera_cpp
{

/// Camera configuration: the 30 fps video stream or the full-resolution still capture.
enum class Mode
{
  Video,
  Still
};

/// Autofocus state as reported by the latest frame metadata.
enum class AfState
{
  Idle,
  Scanning,
  Focused,
  Failed
};

/**
 * @param state An autofocus state.
 * @return Its wire name: idle, scanning, focused or failed (the zoom result's af_state).
 */
inline std::string to_string(AfState state)
{
  switch (state) {
    case AfState::Idle:
      return "idle";
    case AfState::Scanning:
      return "scanning";
    case AfState::Focused:
      return "focused";
    case AfState::Failed:
      return "failed";
  }
  return "idle";
}

/// An autofocus request on the running stream. Setting one switches AF to the Auto mode, where a trigger works.
struct AfRequest
{
  bool windowed;  ///< Meter AF on `window` only (AfMetering Windows); false meters the whole frame.
  Rect window;    ///< AF window in sensor ScalerCrop coordinates (see scaler_crop_maximum()).
  bool trigger;   ///< Start a scan (AfTrigger Start).
};

/// One still frame. The view and its memory stay valid only until the next call on the backend.
struct StillFrame
{
  Yuv420View view;                      ///< The full-resolution YUV 4:2:0 frame.
  int64_t stamp_ns;                     ///< Sensor capture time, steady clock nanoseconds.
  AfState af_state;                     ///< AF state in this frame's metadata.
  std::optional<double> lens_position;  ///< Lens position in dioptres, when the metadata has it.
};

/// One video frame handed to the video callback, valid only during the callback.
struct VideoFrame
{
  const uint8_t * bgr;  ///< Packed BGR888 (libcamera RGB888).
  int stride;           ///< Bytes per row.
  Size size;            ///< Width and height in pixels.
  int64_t stamp_ns;     ///< Sensor capture time, steady clock nanoseconds.
};

/// A camera as the zoom state machine and the node see it. Every failure is returned (false or nullopt), never
/// thrown.
class CameraBackend
{
public:
  virtual ~CameraBackend() = default;

  /**
   * Start the stream in the configured mode for video: requests queued, AF Continuous, frames delivered to the
   * video callback.
   *
   * @return true when the stream runs.
   */
  virtual bool start_video() = 0;

  /// Stop the stream (any mode); a no-op when stopped. Frames from before the stop are never delivered after it.
  virtual void stop() = 0;

  /**
   * Configure the stopped camera for a mode (video: the 30 fps stream; still: the full sensor mode with its
   * buffers, freed again when leaving the mode).
   *
   * @param mode The mode.
   * @return true when configured.
   */
  virtual bool configure(Mode mode) = 0;

  /**
   * Start the stream in the still mode with AF Auto and no trigger, so the lens holds the position it has.
   *
   * @return true when the stream runs.
   */
  virtual bool start_still() = 0;

  /**
   * Apply an AF request to the running stream (AF Auto, the window, the trigger).
   *
   * After a triggered request, af_state() must not report a state that predates the request: until a frame
   * produced with the request arrives it reports Scanning.
   *
   * @param request The request.
   * @return true when the controls were queued.
   */
  virtual bool set_af(const AfRequest & request) = 0;

  /**
   * Engage or release the mapping focus lock: locked is AfMode Manual at the configured lens position, released is
   * the configured af_mode. Kept across stop/configure/start_video (the next start_video starts in the state set
   * here); on a running video stream it is queued for the next request.
   *
   * @param locked Whether the lens is locked.
   * @return true when the state was recorded (and queued when streaming).
   */
  virtual bool set_focus_lock(bool locked) = 0;

  /**
   * @return The AF state of the latest frame (Scanning after a trigger until a frame with the request arrives).
   */
  virtual AfState af_state() = 0;

  /**
   * Wait for the next still frame (still mode only).
   *
   * @param timeout Maximum wait.
   * @return The frame, valid until the next backend call; nullopt on timeout or error.
   */
  virtual std::optional<StillFrame> wait_still_frame(std::chrono::milliseconds timeout) = 0;

  /**
   * Register the callback that receives every video frame, on the backend's thread. It must not block.
   *
   * @param callback The callback; an empty function unregisters.
   */
  virtual void set_video_callback(std::function<void(const VideoFrame &)> callback) = 0;

  /**
   * Release and re-acquire the camera (the last resort of the restore path). The camera is left stopped and
   * unconfigured.
   *
   * @return true when the camera is acquired again.
   */
  virtual bool reacquire() = 0;

  /**
   * @return The sensor's maximum ScalerCrop rectangle, the coordinate system of AfRequest::window.
   */
  virtual Rect scaler_crop_maximum() = 0;
};

/// Monotonic time source, injectable so tests run on a fake clock.
class Clock
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using Duration = std::chrono::steady_clock::duration;

  virtual ~Clock() = default;

  /// @return The current monotonic time.
  virtual TimePoint now() = 0;

  /// @param duration How long to sleep; non-positive returns at once.
  virtual void sleep_for(Duration duration) = 0;
};

/// The production clock: std::chrono::steady_clock and std::this_thread::sleep_for.
class SteadyClock : public Clock
{
public:
  TimePoint now() override { return std::chrono::steady_clock::now(); }

  void sleep_for(Duration duration) override
  {
    if (duration > Duration::zero()) {
      std::this_thread::sleep_for(duration);
    }
  }
};

}  // namespace head_camera_cpp
