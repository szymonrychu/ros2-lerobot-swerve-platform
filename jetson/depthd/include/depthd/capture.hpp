// Stereo capture on the Jetson Nano (part of depthd-capture).
//
// Two GStreamer pipelines, one per IMX219, built by build_pipeline_description():
//   nvarguscamerasrc (1640x1232 binned mode, ee-mode 0, tnr-mode 1) -> NV12 NVMM -> nvvidconv -> GRAY8 at
//   processing size -> appsink
// The left pipeline tees a second branch (NVMM throughout) to nvvidconv -> nvjpegenc -> appsink for the RGB overview
// stream, throttled to rgb.fps by a pad probe that drops buffers before scaling (no videorate: it loses the
// nvarguscamerasrc timestamp metadata, and no CPU copy).
//
// zero_copy true keeps the GRAY8 frame in NVMM and maps it into CUDA through nvbuf_utils + EGL
// (ExtractFdFromNvBuffer -> NvEGLImageFromFd -> cuGraphicsEGLRegisterImage); the frame is then copied device to device
// into a pooled GpuMat so the NVMM buffer goes straight back to the camera. zero_copy false outputs system memory and
// uploads it.
//
// Capture time is the sensor start-of-frame time that nvarguscamerasrc attaches as "GstBufferMetaData" qdata
// (CLOCK_MONOTONIC ns), read by a probe on the source pad; when it is absent the buffer PTS plus the pipeline base
// time (also CLOCK_MONOTONIC) is used. Times are moved to CLOCK_REALTIME with an offset refreshed every second.
#ifndef DEPTHD_CAPTURE_HPP_
#define DEPTHD_CAPTURE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "depthd/config.hpp"
#include "depthd/pairing.hpp"

namespace depthd
{

enum class CameraSide
{
  left,
  right
};

// Argus sensor-mode index of an IMX219 mode on the Jetson Nano, or -1 when the camera has no such mode.
// 0: 3264x2464@21, 1: 3264x1848@28, 2: 1920x1080@30, 3: 1640x1232@30, 4: 1280x720@60, 5: 1280x720@120.
int imx219_sensor_mode(const SensorMode & mode);

// GStreamer launch description of one camera. Throws std::invalid_argument naming the key when the config cannot be
// captured (unknown sensor mode). Elements are named <side>_src, <side>_sink, and for the left side with rgb.enabled
// rgb_queue, rgb_jpeg and rgb_sink.
std::string build_pipeline_description(const Config & config, CameraSide side);

// Capture time on CLOCK_MONOTONIC: the sensor timestamp when nvarguscamerasrc attached one, else PTS + base time.
// Returns nullopt when neither is valid.
std::optional<int64_t> capture_monotonic_ns(
  std::optional<int64_t> sensor_timestamp_ns, uint64_t pts_ns, uint64_t base_time_ns);

// CLOCK_MONOTONIC -> CLOCK_REALTIME. The offset is sampled with the tightest of three bracketing reads and refreshed
// at most once per refresh_interval_ns, so chrony slewing realtime onto the Pi's clock is followed.
class ClockOffset
{
public:
  using ClockFn = std::function<int64_t()>;
  static constexpr int64_t kRefreshIntervalNs = 1000000000;

  ClockOffset();  // the system clocks
  ClockOffset(ClockFn monotonic, ClockFn realtime);

  int64_t to_realtime(int64_t monotonic_ns);  // refreshes the offset when it is due
  int64_t offset_ns() const { return offset_ns_.load(); }
  void refresh();

private:
  ClockFn monotonic_;
  ClockFn realtime_;
  std::atomic<int64_t> offset_ns_{0};
  std::atomic<int64_t> sampled_at_mono_ns_{0};
  std::mutex refresh_mutex_;
};

// One JPEG of the RGB overview stream. data stays valid while any copy of this struct (its hold) is alive; it points
// into the encoder's output buffer, so there is no CPU copy.
struct RgbJpeg
{
  const uint8_t * data = nullptr;
  std::size_t size = 0;
  int64_t capture_realtime_ns = 0;
  uint64_t sequence = 0;
  std::shared_ptr<void> hold;
};

enum class CaptureState
{
  starting,
  ok,
  camera_error
};

struct CaptureStatus
{
  CaptureState state = CaptureState::starting;
  std::string reason;  // why camera_error (or starting); empty when ok
  bool exposure_locked = false;
};

const char * to_string(CaptureState state);

// AE/AWB lock bookkeeping: lock once both cameras delivered settle_frames frames.
class ExposureLockPolicy
{
public:
  ExposureLockPolicy(int settle_frames, bool enabled) : settle_frames_(settle_frames), enabled_(enabled) {}
  // Count a frame; returns true exactly once, on the frame after which both sources must be locked.
  bool on_frame(CameraSide side);
  void reset();  // a camera restarted: settle again
  bool locked() const { return locked_; }

private:
  int settle_frames_;
  bool enabled_;
  int64_t left_frames_ = 0;
  int64_t right_frames_ = 0;
  bool locked_ = false;
};

// Throttle of the RGB JPEG branch: admits one buffer per 1/rgb_fps of PTS (half a sensor frame of tolerance) and
// drops the rest. PTS is running time, so it restarts near 0 when the pipeline goes NULL -> PLAYING; a PTS behind the
// last admitted one resyncs the schedule instead of dropping until the old schedule is reached again.
class RgbThrottle
{
public:
  RgbThrottle(double rgb_fps, double sensor_fps);
  bool admit(uint64_t pts_ns);
  void reset();  // the pipeline restarted: the next buffer is admitted

private:
  uint64_t interval_ns_;
  uint64_t tolerance_ns_;
  uint64_t next_due_ns_ = 0;
  uint64_t last_admitted_ns_ = 0;
  bool due_set_ = false;
};

// Readback of the sensor's current exposure and gain from its V4L2 controls (/dev/video<sensor-id>), used to log the
// locked values. exposure is in microseconds; gain is in sensor units where 16 = 1x (IMX219 gain_factor).
struct SensorExposure
{
  int64_t exposure_us = 0;
  int64_t gain_raw = 0;
  double gain() const { return gain_raw / 16.0; }
};
std::optional<SensorExposure> read_sensor_exposure(int sensor_id, std::string * error = nullptr);

class StereoCapture
{
public:
  using PairCallback = std::function<void(const StereoPair &)>;
  using RgbCallback = std::function<void(const RgbJpeg &)>;
  using LogFn = std::function<void(const std::string &)>;

  static constexpr int64_t kCameraTimeoutNs = 2000000000;   // no frame for 2 s -> camera_error
  static constexpr int64_t kRestartBackoffNs = 5000000000;  // a failed camera pipeline is restarted at most every 5 s
  static constexpr int kMaxAlignAttempts = 20;              // right-camera restarts spent on finding an in-window phase

  explicit StereoCapture(const Config & config, LogFn log = nullptr);
  ~StereoCapture();
  StereoCapture(const StereoCapture &) = delete;
  StereoCapture & operator=(const StereoCapture &) = delete;

  // Callbacks run on GStreamer streaming threads and must return quickly. Set them before start().
  void set_pair_callback(PairCallback cb);
  void set_rgb_callback(RgbCallback cb);

  // Builds and starts both pipelines. Returns false with the reason when they cannot be started.
  bool start(std::string * error = nullptr);
  void stop();

  CaptureStatus status() const;
  PairingStats pairing_stats() const;
  // rgb.jpeg_quality is a live key: applied to the running encoder.
  void set_jpeg_quality(int quality);

  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

// True when a camera has delivered no frame for more than StereoCapture::kCameraTimeoutNs, counted from its last
// frame (last_frame_mono_ns, 0 = none yet) or else from its pipeline start.
bool camera_timed_out(int64_t now_mono_ns, int64_t last_frame_mono_ns, int64_t started_mono_ns);

}  // namespace depthd

#endif  // DEPTHD_CAPTURE_HPP_
