#pragma once

// The libcamera adapter of the camera-backend-interface (design D2, D6, D7): LibcameraBackend drives the head IMX708
// (30 fps video, a full-resolution still on request, windowed autofocus), and the pure control helpers build the
// libcamera control lists of each phase without a camera. Only src/libcamera_backend.cpp touches a camera. Serves G1,
// G2, U1, U2.
//
// libcamera version: the adapter uses only API present in both the docker image's rosdep libcamera (ros-jazzy-libcamera
// 0.7.2) and the Pi's fork (raspberrypi/libcamera v0.7.2+rpt20260817): both are libcamera 0.7.2 with identical public
// headers for every class and control used here, so no LIBCAMERA_VERSION_* guard is needed. Add one here if a pin bump
// ever splits them.

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "head_camera_cpp/camera_backend.hpp"
#include "head_camera_cpp/config.hpp"

namespace head_camera_cpp
{

/// Severity of a backend log line.
enum class LogLevel
{
  Info,
  Warn,
  Error
};

/// Receives the backend's log lines; called from the caller's thread and from libcamera's thread, so it must be
/// thread-safe.
using LogFunction = std::function<void(LogLevel, const std::string &)>;

/**
 * Controls of the 30 fps video stream (D2): FrameDurationLimits [frame_duration_us, frame_duration_us] and AfMode from
 * config.af_mode (continuous, auto or manual).
 *
 * @param config The node configuration.
 * @return The control list, keyed by libcamera::controls::controls.
 */
libcamera::ControlList video_controls(const Config & config);

/**
 * Controls of an AF request (the D6 AF step): AfMode Auto; AfMetering Windows plus AfWindows = the window when windowed,
 * AfMetering Auto otherwise; AfTrigger Start when trigger.
 *
 * @param request The AF request; its window is in ScalerCropMaximum coordinates.
 * @return The control list, keyed by libcamera::controls::controls.
 */
libcamera::ControlList af_controls(const AfRequest & request);

/**
 * Controls of the still stream (the D6 SWITCH step): AfMode Auto and no trigger, so the lens holds its position.
 *
 * @return The control list, keyed by libcamera::controls::controls.
 */
libcamera::ControlList still_controls();

/**
 * @param libcamera_af_state A libcamera::controls::AfStateEnum value from frame metadata.
 * @return The backend AF state; an unknown value maps to Idle (not converged, so a waiting zoom times out).
 */
AfState to_af_state(int32_t libcamera_af_state);

/**
 * @param rect A pixel rectangle.
 * @return The same rectangle as a libcamera::Rectangle.
 */
libcamera::Rectangle to_rectangle(Rect rect);

/**
 * @param rectangle A libcamera rectangle.
 * @return The same rectangle as a Rect.
 */
Rect from_rectangle(const libcamera::Rectangle & rectangle);

/// CameraBackend on a real libcamera camera. Owns the process's single CameraManager.
///
/// Threads: the public methods are called from one thread at a time (see CameraBackend). libcamera's thread runs
/// request_completed, which only hands frames off: a video frame goes to a one-slot mailbox (an unclaimed older frame
/// is recycled at once) that the backend's dispatcher thread passes to the video callback before re-queueing the
/// request; a still frame goes to a queue that wait_still_frame pops. No libcamera error crosses a thread as an
/// exception: each is logged and returned as false or nullopt.
class LibcameraBackend final : public CameraBackend
{
public:
  /**
   * Start the CameraManager and acquire the camera `config.camera_id`. A failure is logged and retried by the next
   * configure or start_video call, so the node can keep retrying.
   *
   * @param config The node configuration (camera_id, sensor_mode, width, height, frame_duration_us, af_mode,
   *   zoom.still_size, zoom.still_buffers).
   * @param log Receiver of log lines; empty logs to stderr.
   */
  explicit LibcameraBackend(const Config & config, LogFunction log = {});
  ~LibcameraBackend() override;

  LibcameraBackend(const LibcameraBackend &) = delete;
  LibcameraBackend & operator=(const LibcameraBackend &) = delete;

  /// Configures the video mode first when the camera is not configured for it (opening the camera when needed).
  bool start_video() override;
  void stop() override;
  bool configure(Mode mode) override;
  bool start_still() override;
  bool set_af(const AfRequest & request) override;
  AfState af_state() override;
  std::optional<StillFrame> wait_still_frame(std::chrono::milliseconds timeout) override;
  void set_video_callback(std::function<void(const VideoFrame &)> callback) override;
  bool reacquire() override;
  Rect scaler_crop_maximum() override;

private:
  /// Plane start addresses of one mmapped frame buffer.
  struct BufferPlanes
  {
    std::vector<const uint8_t *> data;
    std::vector<std::size_t> length;
  };

  /// One mmap of a dmabuf.
  struct Mapping
  {
    void * address;
    std::size_t length;
  };

  void log(LogLevel level, const std::string & message) const;
  bool ensure_open();
  void close_camera();
  void release_buffers();
  bool map_buffers();
  bool start_stream(const libcamera::ControlList & controls);
  void request_completed(libcamera::Request * request);
  void requeue_locked(libcamera::Request * request);
  void dispatch_video();
  std::optional<StillFrame> still_frame(const libcamera::Request & request) const;

  Config config_;
  LogFunction log_;

  // Camera ownership, touched only by the calling thread.
  std::unique_ptr<libcamera::CameraManager> manager_;
  std::shared_ptr<libcamera::Camera> camera_;
  std::unique_ptr<libcamera::CameraConfiguration> configuration_;
  std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
  std::vector<std::unique_ptr<libcamera::Request>> requests_;
  std::vector<Mapping> mappings_;
  // Immutable while streaming (written by configure only), so request_completed and the dispatcher read it unlocked.
  std::map<const libcamera::FrameBuffer *, BufferPlanes> planes_;
  libcamera::Stream * stream_ = nullptr;
  std::optional<Mode> mode_;
  Size frame_size_{0, 0};
  int stride_ = 0;

  // Shared with libcamera's thread and the dispatcher, under mutex_.
  std::mutex mutex_;
  std::condition_variable frame_ready_;
  bool running_ = false;
  bool shutdown_ = false;
  AfState af_state_ = AfState::Idle;
  libcamera::ControlList pending_controls_;
  bool af_trigger_pending_ = false;                 // a trigger waits for a request to carry it
  const libcamera::Request * af_marker_ = nullptr;  // the request carrying the latest trigger, until it completes
  libcamera::Request * video_slot_ = nullptr;
  bool video_busy_ = false;
  std::function<void(const VideoFrame &)> video_callback_;
  std::deque<libcamera::Request *> still_queue_;
  libcamera::Request * still_held_ = nullptr;

  std::thread dispatcher_;
};

}  // namespace head_camera_cpp
