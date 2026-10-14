// Continuous auto exposure for the stereo pair (part of depthd-capture).
//
// capture.exposure.mode follow (the default): Argus AE and AWB run continuously on the left camera, capped at
// capture.exposure.max_exposure_us and max_gain, with the ISP digital gain pinned to 1x on both cameras (it cannot be
// read back, so it could not be matched). Every measure_interval_frames left frames StereoCapture reads both sensors'
// exposure and analog gain from their V4L2 controls and:
//   ExposureFollower      pins the right camera to the left's exposure and gain (exposuretimerange/gainrange with
//                         low == high) whenever the right's achieved readback is not within tolerance of the left's
//                         (follow_threshold relative, never tighter than the sensors' quantisation), or the left moved
//                         more than follow_threshold from a pin still being reached. A re-pin opens a transition:
//                         pairs are withheld until the right's readback reached the pin and follow_settle_frames more
//                         right frames arrived, but never for more than follow_transition_frames right frames. A
//                         right that did not take the pin is pinned again after the window.
//   FollowStarvationGuard watches whether the right converged; when it has not for follow_starvation_s, StereoCapture
//                         falls back to lock_after_settle.
//   SaturationCompensator measures the fraction of left grey pixels >= saturation_level and steps the left's Argus
//                         exposurecompensation down by compensation_step while it is above saturated_high (to
//                         compensation_min), and back up toward 0 while it is below saturated_low; in between it holds
//                         (hysteresis), and steps are at least compensation_hold_frames left frames apart.
// mode lock_after_settle is the old behaviour (ExposureLockPolicy in capture.hpp): AE/AWB settle, then aelock/awblock.
// Both policies are plain state machines without clocks or cameras, so tests drive them with fake readbacks.
#ifndef DEPTHD_EXPOSURE_CONTROL_HPP_
#define DEPTHD_EXPOSURE_CONTROL_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include <opencv2/core/cuda.hpp>

#include "depthd/config.hpp"

namespace depthd
{

// Exposure and analog gain of a sensor as its V4L2 controls report them: exposure in microseconds, gain in sensor
// units where 16 = 1x (IMX219 gain_factor).
struct SensorExposure
{
  int64_t exposure_us = 0;
  int64_t gain_raw = 0;
  double gain() const { return gain_raw / 16.0; }
  bool operator==(const SensorExposure & o) const { return exposure_us == o.exposure_us && gain_raw == o.gain_raw; }
};

// Argus limits that nvarguscamerasrc enforces (L4T R32.6.1, IMX219): exposuretimerange below 34000 ns is rejected
// ("Invalid Exposure Time Range Input"), analog gain is 1..10.625x.
constexpr int64_t kMinExposureNs = 34000;
constexpr double kMaxAnalogGain = 10.625;

// nvarguscamerasrc property values that pin a camera to `e`: {exposuretimerange, gainrange}, e.g.
// {"12000000 12000000", "3.3125 3.3125"}, clamped to the Argus limits above.
std::pair<std::string, std::string> pinned_range_properties(const SensorExposure & e);

struct ExposureFollowerOptions
{
  double threshold = 0.05;  // relative exposure or gain change that re-pins the right camera
  int settle_frames = 2;    // right frames after the right's readback reached the pin before pairs flow again
  int transition_frames = 30;  // right frames a re-pin withholds pairs for at most, matched or not

  static ExposureFollowerOptions from_config(const ExposureConfig & exposure);
};

class ExposureFollower
{
public:
  explicit ExposureFollower(ExposureFollowerOptions options);

  // A left readback (after the right's readback of the same measurement). Returns the values to pin the right camera
  // to, which starts a transition: when there is no pin yet; outside a transition, when the right's last readback is
  // not within tolerance of the left; inside one, when the left moved more than threshold away from the pin.
  std::optional<SensorExposure> on_left(const SensorExposure & left);
  // A right readback: the transition can end once it is within tolerance of the pin.
  void on_right(const SensorExposure & right);
  // A right frame arrived: counts down the settle frames once the readback matched, and the transition window.
  void on_right_frame();
  // True when a pair may be published: pinned and no transition open.
  bool pairs_flow() const;
  // True when the right's last readback is within tolerance of the pin and no transition is open.
  bool converged() const;
  // The right camera restarted (or a mode change): forget the pin, so the next left readback pins again.
  void reset();

  std::optional<SensorExposure> pin() const { return pin_; }
  uint64_t pins() const { return pins_; }  // right-camera re-pins so far

private:
  // True when `value` is within tolerance of `reference`: follow_threshold relative, but at least a few sensor lines
  // of exposure and one gain step (two IMX219s pinned alike read back that far apart).
  bool within(const SensorExposure & value, const SensorExposure & reference) const;
  void start_transition(const SensorExposure & pin);

  const ExposureFollowerOptions opt_;
  std::optional<SensorExposure> pin_;
  std::optional<SensorExposure> last_right_;
  bool in_transition_ = false;
  bool readback_matched_ = false;
  int settle_left_ = 0;
  int window_left_ = 0;
  uint64_t pins_ = 0;
};

// Follow mode's last resort: on every pair, whether the follower has converged. When it has not for limit_ns (since
// the last converged pair, or the first pair after construction or reset()), on_pair() returns true once and StereoCapture falls back to
// lock_after_settle. Clockless: the caller passes CLOCK_MONOTONIC ns.
class FollowStarvationGuard
{
public:
  explicit FollowStarvationGuard(int64_t limit_ns) : limit_ns_(limit_ns) {}
  bool on_pair(int64_t now_ns, bool converged);
  void reset();  // a right restart: the clock starts again at the next pair
  int64_t unconverged_ns(int64_t now_ns) const { return since_ns_ ? now_ns - *since_ns_ : 0; }

private:
  const int64_t limit_ns_;
  std::optional<int64_t> since_ns_;  // the last converged pair, or the first pair since construction or reset()
  bool fired_ = false;
};

struct SaturationOptions
{
  double high = 0.02;      // saturated fraction above which the compensation steps down
  double low = 0.005;      // ... below which it steps back up toward 0
  double step = 0.25;      // EV per step
  double min = -2.0;       // lowest compensation (Argus allows -2..2)
  int hold_frames = 15;    // left frames between two steps

  static SaturationOptions from_config(const ExposureConfig & exposure);
};

class SaturationCompensator
{
public:
  explicit SaturationCompensator(SaturationOptions options);

  // A measurement on left frame `frame` (a counter that only grows). Returns the new exposurecompensation to apply
  // when it changes.
  std::optional<double> on_measurement(double saturated_fraction, int64_t frame);
  double compensation() const { return compensation_; }
  uint64_t adjustments() const { return adjustments_; }

private:
  const SaturationOptions opt_;
  double compensation_ = 0.0;
  std::optional<int64_t> last_step_frame_;
  uint64_t adjustments_ = 0;
};

// Fraction of the pixels of a CV_8UC1 GPU image that are >= level, computed on the GPU (scratch is reused across
// calls). 0 for an empty image.
double saturated_fraction(const cv::cuda::GpuMat & grey, int level, cv::cuda::GpuMat & scratch);

}  // namespace depthd

#endif  // DEPTHD_EXPOSURE_CONTROL_HPP_
