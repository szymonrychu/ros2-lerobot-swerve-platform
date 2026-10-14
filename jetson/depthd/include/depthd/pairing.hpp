// Left/right frame pairing by capture timestamp (part of depthd-capture).
//
// The two IMX219s are free-running (no hardware sync), so frames are paired in software by their sensor timestamps.
// StereoPairer keeps a short sorted queue per side, emits a pair only when the nearest left/right timestamps are within
// pair_window_ms, drops and counts everything else, never reuses a frame and records a skew histogram that the
// benchmark (T16) reads to judge the window. It is thread safe: the two capture threads push concurrently and /health
// reads the counters.
#ifndef DEPTHD_PAIRING_HPP_
#define DEPTHD_PAIRING_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <opencv2/core/cuda.hpp>
#include <optional>

namespace depthd
{

// One captured frame: the GRAY8 image at processing size on the GPU and its capture time.
struct Frame
{
  cv::cuda::GpuMat image;           // CV_8UC1, processing.width x processing.height
  int64_t capture_realtime_ns = 0;  // sensor start-of-frame time on CLOCK_REALTIME
  uint64_t sequence = 0;            // per-camera frame counter
};

// A left/right pair handed to the pipeline. capture_realtime_ns is the left frame's time.
struct StereoPair
{
  cv::cuda::GpuMat left;
  cv::cuda::GpuMat right;
  int64_t capture_realtime_ns = 0;
  int64_t right_minus_left_ns = 0;
  uint64_t left_sequence = 0;
  uint64_t right_sequence = 0;
};

// Histogram of |right - left| for every nearest-neighbour candidate the pairer judged (emitted or dropped for skew).
// counts[i] holds skews in (bounds[i-1], bounds[i]] ms; counts[kBuckets] holds everything above the last bound.
struct SkewHistogram
{
  static constexpr std::size_t kBuckets = 6;
  static constexpr std::array<double, kBuckets> kBoundsMs{{1.0, 2.0, 4.0, 8.0, 16.0, 33.0}};

  std::array<uint64_t, kBuckets + 1> counts{};
  uint64_t count = 0;
  double sum_ms = 0.0;

  void record(double skew_ms);
  // Prometheus-style cumulative counts: element i counts skews <= kBoundsMs[i]; the last element equals count.
  std::array<uint64_t, kBuckets + 1> cumulative() const;
};

// Continuous auto exposure state (filled in by StereoCapture, exposure_control.hpp). A camera's values are valid once
// its V4L2 controls were read back; awb gains are not here because nvarguscamerasrc exposes no way to read them.
struct CameraExposureStats
{
  bool valid = false;
  int64_t exposure_us = 0;
  double gain = 0.0;
};

struct ExposureStats
{
  CameraExposureStats left;
  CameraExposureStats right;
  bool saturation_valid = false;
  double saturated_fraction = 0.0;  // of the left grey image, latest measurement
  double compensation = 0.0;        // left Argus exposurecompensation, EV
  uint64_t ae_adjustments = 0;      // exposurecompensation steps taken against (or after) saturation
  uint64_t follow_pins = 0;         // right camera pinned to the left's exposure and gain
  uint64_t mismatch_drops = 0;      // pairs withheld because the right did not yet match the left
  uint64_t follow_fallbacks = 0;    // follow mode starved and fell back to lock_after_settle
};

struct PairingStats
{
  uint64_t pairs_emitted = 0;
  uint64_t dropped_skew = 0;    // nearest counterpart exists but lies outside pair_window_ms
  uint64_t dropped_orphan = 0;  // no counterpart: the other camera missed it, the queue overflowed, or it was stale
  SkewHistogram skew;
  // Runtime camera-phase re-alignments by reason (filled in by StereoCapture, not by the pairer).
  uint64_t realignments_skew = 0;
  uint64_t realignments_stall = 0;
  ExposureStats exposure;  // filled in by StereoCapture
};

class StereoPairer
{
public:
  static constexpr std::size_t kMaxQueue = 3;  // frames kept per side

  using SkewObserver = std::function<void(double skew_ms)>;

  explicit StereoPairer(double pair_window_ms);

  // Called with every skew the histogram records (under the pairer lock: it must be cheap and must not call back into
  // the pairer). Set before frames are pushed.
  void set_skew_observer(SkewObserver observer);

  // Each push returns the pair it completed, if any. A frame is part of at most one pair.
  std::optional<StereoPair> push_left(Frame frame);
  std::optional<StereoPair> push_right(Frame frame);

  // Drops every queued frame (counted as orphans), e.g. when a camera fails. Counters are kept.
  void reset();

  PairingStats stats() const;
  std::size_t queued_left() const;
  std::size_t queued_right() const;
  double pair_window_ms() const { return window_ns_ / 1e6; }

private:
  enum class Side
  {
    left,
    right
  };
  std::optional<StereoPair> push(Side side, Frame frame);
  std::optional<StereoPair> match();

  const int64_t window_ns_;
  mutable std::mutex mutex_;
  std::deque<Frame> left_;
  std::deque<Frame> right_;
  // Newest timestamp consumed (paired or dropped) per side: older arrivals are stale and never paired.
  std::optional<int64_t> left_consumed_ns_;
  std::optional<int64_t> right_consumed_ns_;
  PairingStats stats_;
  SkewObserver skew_observer_;
};

}  // namespace depthd

#endif  // DEPTHD_PAIRING_HPP_
