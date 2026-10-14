// depthd health: the state reported by GET /health and carried in every wire header, and the board readers that
// feed it (GPU temperature from /sys, nvgpu driver errors from /dev/kmsg).
//
// States, first match wins:
//   config_error  the config file (or the matcher it names) is invalid; nothing runs, /health says why
//   camera_error  a camera failed or could not be started (CaptureStatus); its messages stop
//   uncalibrated  no valid calibration; rgb flows, depth and fused do not
//   starting      binding the link, aligning the cameras, or waiting for the first depth frame
//   ok            depth is being published (reason names a stall when no depth came for over 2 s)
#ifndef DEPTHD_HEALTH_HPP_
#define DEPTHD_HEALTH_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "depthd/capture.hpp"
#include "depthd/http_api.hpp"

namespace depthd
{

// Everything compose_health() needs; the pipeline fills most of it, the daemon adds config, bind and calibration
// reasons.
struct HealthInputs
{
  std::string config_error;    // non-empty: state config_error
  std::string startup_reason;  // non-empty: still starting (e.g. a bind that is being retried)
  CaptureStatus capture;
  bool calibrated = false;
  std::string calibration_reason;  // why uncalibrated
  bool depth_published = false;    // at least one depth frame went out
  double last_depth_age_ms = -1.0; // -1: none yet
  double fps = 0.0;
  double last_pair_age_ms = -1.0;
  uint64_t pairs_dropped_total = 0;
  double gpu_temp_c = 0.0;
  uint64_t nvgpu_errors_total = 0;
  std::string matcher;
  std::string mono_state;
};

// Depth that stops for longer than this while ok is named in the reason.
constexpr double kDepthStallMs = 2000.0;

Health compose_health(const HealthInputs & in);

// Temperature of the thermal zone whose type is "GPU-therm" (Jetson Nano: thermal_zone2), in degrees C, or nullopt
// when there is none.
std::optional<double> read_gpu_temp_c(const std::string & thermal_root = "/sys/devices/virtual/thermal");

// True for a kernel log record ("<prio>,<seq>,<usec>,<flags>;<message>", /dev/kmsg format) of level err or worse
// (syslog level <= 3) from the GPU driver (message contains "nvgpu" or "gk20a"). Continuation lines (leading space)
// are not records.
bool is_nvgpu_error(const std::string & record);

// Counts nvgpu error records in the kernel log, from the start of the ring buffer, so errors from before depthd
// started (a FECS loop at boot) are counted too. poll() reads what is new without blocking. Works on /dev/kmsg (one
// record per read) and on a plain file (tests). An unreadable path leaves the counter unavailable at 0.
class NvgpuErrorCounter
{
public:
  explicit NvgpuErrorCounter(const std::string & path = "/dev/kmsg");
  ~NvgpuErrorCounter();
  NvgpuErrorCounter(const NvgpuErrorCounter &) = delete;
  NvgpuErrorCounter & operator=(const NvgpuErrorCounter &) = delete;

  uint64_t poll();  // returns total()
  uint64_t total() const { return total_; }
  bool available() const { return fd_ >= 0; }
  const std::string & error() const { return error_; }

private:
  void consume(const char * data, std::size_t size);

  int fd_ = -1;
  std::string error_;
  std::string partial_;  // an unterminated line of a plain file
  uint64_t total_ = 0;
};

// Frames per second over the last kWindow frames; 0 when the newest frame is older than kStaleNs. Not thread safe.
class RateMeter
{
public:
  static constexpr std::size_t kWindow = 16;
  static constexpr int64_t kStaleNs = 2000000000;

  void tick(int64_t now_ns);
  double rate(int64_t now_ns) const;

private:
  std::deque<int64_t> ticks_;
};

}  // namespace depthd

#endif  // DEPTHD_HEALTH_HPP_
