// Continuous auto exposure for the stereo pair (part of depthd-capture).
//
// capture.exposure.mode follow (the default): Argus AE and AWB run continuously on the left camera, capped at
// capture.exposure.max_exposure_us and max_gain, with the ISP digital gain pinned to 1x on both cameras (it cannot be
// read back, so it could not be matched). Every measure_interval_frames left frames StereoCapture reads both sensors'
// exposure and analog gain from their V4L2 controls and:
//   ExposureFollower      pins the right camera to the left's exposure and gain (exposuretimerange/gainrange with
//                         low == high) whenever the left moved more than follow_threshold from the pinned values, and
//                         judges whether the two currently match: from a new pin until the right's readback equals it
//                         and follow_settle_frames more right frames arrived, every pair is withheld and counted.
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

  static ExposureFollowerOptions from_config(const ExposureConfig & exposure);
};

class ExposureFollower
{
public:
  explicit ExposureFollower(ExposureFollowerOptions options);

  // A left readback. Returns the values to pin the right camera to when there is no pin yet or the left moved more than
  // threshold (exposure or gain, relative) away from it; that starts a transition.
  std::optional<SensorExposure> on_left(const SensorExposure & left);
  // A right readback: the transition can end once it matches the pin.
  void on_right(const SensorExposure & right);
  // A right frame arrived (counts down the settle frames once the readback matched).
  void on_right_frame();
  // True when a pair may be published: pinned, the right's readback matched the pin and the settle frames passed.
  bool matched() const;
  // The right camera restarted (or a mode change): forget the pin, so the next left readback pins again.
  void reset();

  std::optional<SensorExposure> pin() const { return pin_; }
  uint64_t pins() const { return pins_; }  // right-camera re-pins so far

private:
  const ExposureFollowerOptions opt_;
  std::optional<SensorExposure> pin_;
  bool readback_matched_ = false;
  int settle_left_ = 0;
  uint64_t pins_ = 0;
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
