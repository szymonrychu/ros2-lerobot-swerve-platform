// The depthd pipeline (depthd-pipeline): cameras to the wire.
//
//   FrameSource (StereoCapture, or a test source)
//     pair callback --> input mailbox (1 pair; a newer pair replaces it, counted "busy") --+
//     rgb callback  --> gate (camera, age) --> Publisher::send_rgb                        |
//                                                                                          v
//   match thread, one CUDA stream: take a free slot, check age (<= 100 ms) and gates, rectify -> match ->
//     post-process -> depth mm -> download into the slot --> encode thread: zstd + Publisher::send_depth --> free
//
// Two slots (kSlots) circulate between the match and encode threads, so zstd and the send of one frame overlap the
// matching of the next and at most two frames are ever in flight. The capture callbacks only swap a pointer-sized
// pair into the mailbox: capture never waits on matching. A pair older than max_pair_age_ns (100 ms, the latency
// budget G5) when matching starts is dropped and counted; so is one more than 100 ms in the future.
//
// Nothing is published that the pipeline cannot vouch for: no depth or fused without a valid calibration
// (set_calibration), no depth or fused while either camera is in error, no rgb while the left camera is in error,
// nothing from a stale frame, and never a synthetic or placeholder frame.
#ifndef DEPTHD_PIPELINE_HPP_
#define DEPTHD_PIPELINE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthd/calibration.hpp"
#include "depthd/capture.hpp"
#include "depthd/config.hpp"
#include "depthd/health.hpp"
#include "depthd/http_api.hpp"
#include "depthd/pairing.hpp"
#include "depthd/publisher.hpp"
#include "ros2_common_cpp/depth_wire.hpp"

namespace depthd
{

using LogFn = std::function<void(const std::string &)>;

// Where pairs and JPEGs come from. Callbacks run on the source's threads and are set before start().
class FrameSource
{
public:
  using PairCallback = std::function<void(const StereoPair &)>;
  using RgbCallback = std::function<void(const RgbJpeg &)>;

  virtual ~FrameSource() = default;
  virtual void set_pair_callback(PairCallback cb) = 0;
  virtual void set_rgb_callback(RgbCallback cb) = 0;
  virtual bool start(std::string * error) = 0;
  virtual void stop() = 0;
  virtual CaptureStatus status() const = 0;
  virtual PairingStats pairing_stats() const = 0;
  virtual void set_jpeg_quality(int quality) = 0;
};

// The cameras: StereoCapture behind FrameSource.
class StereoCaptureSource : public FrameSource
{
public:
  explicit StereoCaptureSource(const Config & config, LogFn log = nullptr) : capture_(config, std::move(log)) {}
  void set_pair_callback(PairCallback cb) override { capture_.set_pair_callback(std::move(cb)); }
  void set_rgb_callback(RgbCallback cb) override { capture_.set_rgb_callback(std::move(cb)); }
  bool start(std::string * error) override { return capture_.start(error); }
  void stop() override { capture_.stop(); }
  CaptureStatus status() const override { return capture_.status(); }
  PairingStats pairing_stats() const override { return capture_.pairing_stats(); }
  void set_jpeg_quality(int quality) override { capture_.set_jpeg_quality(quality); }

private:
  StereoCapture capture_;
};

struct PipelineOptions
{
  int64_t max_pair_age_ns = 100000000;         // G5: a pair older than this when matching starts is dropped
  int64_t max_capture_pair_age_ns = 1000000000;  // latest_pair() freshness (GET /calibration/capture)
  std::function<int64_t()> realtime_now;        // CLOCK_REALTIME by default
  LogFn log;
  std::string thermal_root = "/sys/devices/virtual/thermal";
  std::string kmsg_path = "/dev/kmsg";  // empty: no nvgpu error counting
  int64_t monitor_interval_ms = 1000;   // GPU temperature, kernel log and metrics refresh
};

struct PipelineCounters
{
  uint64_t pairs_received = 0;
  uint64_t depth_published = 0;
  uint64_t rgb_published = 0;
  uint64_t fused_published = 0;
  uint64_t dropped_busy = 0;          // replaced in the mailbox by a newer pair
  uint64_t dropped_stale = 0;         // older than max_pair_age_ns (or in the future) when matching starts
  uint64_t dropped_uncalibrated = 0;  // no valid calibration
  uint64_t dropped_camera_error = 0;  // a camera is in error
  uint64_t dropped_bad_frame = 0;     // not CV_8UC1 at the calibrated size
  uint64_t dropped_error = 0;         // a GPU stage threw
  uint64_t dropped_send = 0;          // the publisher refused (full send HWM or encode error)
  uint64_t rgb_dropped_stale = 0;
  uint64_t rgb_dropped_camera_error = 0;
  uint64_t fused_rejected = 0;  // publish_fused refused (uncalibrated, camera error, stale)
};

// What a depth listener (the monocular stage, T18) sees for each published depth frame. The GPU images are valid
// only during the call and ordered on `stream`.
struct DepthFrame
{
  const cv::cuda::GpuMat & depth_mm;        // CV_16UC1, 0 = invalid
  const cv::cuda::GpuMat & rectified_left;  // CV_8UC1
  const ros2_common_cpp::depth_wire::Header & header;
  cv::cuda::Stream & stream;
};

class Pipeline
{
public:
  static constexpr int kSlots = 2;
  using DepthListener = std::function<void(const DepthFrame &)>;

  // Builds the matcher and the post-processor for `config` (throws std::invalid_argument or std::runtime_error when
  // the matcher cannot be built). `source` and `metrics` must outlive the pipeline.
  Pipeline(const Config & config, FrameSource & source, Metrics & metrics, PipelineOptions options = {});
  ~Pipeline();
  Pipeline(const Pipeline &) = delete;
  Pipeline & operator=(const Pipeline &) = delete;

  // Starts the worker threads and the source; `publisher` must outlive stop(). Returns false with the reason when
  // the source did not start: the workers keep running and start_source() retries.
  bool start(Publisher & publisher, std::string * error = nullptr);
  bool start_source(std::string * error = nullptr);
  bool source_running() const;
  // Stops the source and joins every thread; frames in flight are discarded. Idempotent.
  void stop();

  // A validated calibration (HttpApiHooks::on_calibration): builds the rectification maps and swaps them in.
  void set_calibration(const StereoCalibration & calibration);
  bool calibrated() const;

  // The runtime_mutable keys of an already validated config: postprocess.*, rgb.jpeg_quality, mono.enabled,
  // mono.rate_hz. Returns "" (HttpApiHooks::apply_config).
  std::string apply_config(const Config & config);

  void set_mono_state(const std::string & state);
  void set_depth_listener(DepthListener listener);

  // Publishes a fused depth frame (frame id, intrinsics and calibrated flag filled in here) only under the same
  // rules as depth: calibrated, both cameras healthy, capture time within max_pair_age_ns. False when refused or
  // dropped by the publisher.
  bool publish_fused(ros2_common_cpp::depth_wire::Header header, const std::vector<uint16_t> & depth_mm);

  // The newest unrectified pair if it is fresher than max_capture_pair_age_ns (GET /calibration/capture).
  std::optional<CapturedPair> latest_pair() const;

  HealthInputs health_inputs() const;
  Health health() const { return compose_health(health_inputs()); }
  PipelineCounters counters() const;
  std::string matcher_name() const;
  std::size_t in_flight_max() const;  // most slots ever taken at once (<= kSlots)

  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace depthd

#endif  // DEPTHD_PIPELINE_HPP_
