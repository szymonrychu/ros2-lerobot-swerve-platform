#include "depthd/exposure_control.hpp"

#include <opencv2/cudaarithm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace depthd
{
namespace
{
constexpr int64_t kExposureSlackUs = 50;  // a few sensor lines: two IMX219s at the same pin read back this far apart
constexpr int64_t kGainSlackRaw = 1;      // one IMX219 gain step (raw units, 16 = 1x)

// |value - reference| relative to reference (an absent reference counts as an infinite change).
double relative_change(int64_t value, int64_t reference)
{
  if (reference <= 0) return value == reference ? 0.0 : INFINITY;
  return std::abs(static_cast<double>(value - reference)) / static_cast<double>(reference);
}

std::string range(const std::string & v) { return v + " " + v; }
}  // namespace

std::pair<std::string, std::string> pinned_range_properties(const SensorExposure & e)
{
  const int64_t ns = std::max<int64_t>(e.exposure_us * 1000, kMinExposureNs);
  const double gain = std::min(std::max(e.gain(), 1.0), kMaxAnalogGain);
  std::ostringstream g;
  g << gain;
  return {range(std::to_string(ns)), range(g.str())};
}

ExposureFollowerOptions ExposureFollowerOptions::from_config(const ExposureConfig & exposure)
{
  ExposureFollowerOptions o;
  o.threshold = exposure.follow_threshold;
  o.settle_frames = exposure.follow_settle_frames;
  o.transition_frames = exposure.follow_transition_frames;
  return o;
}

ExposureFollower::ExposureFollower(ExposureFollowerOptions options) : opt_(options) {}

bool ExposureFollower::within(const SensorExposure & value, const SensorExposure & reference) const
{
  const bool exposure_ok = std::abs(value.exposure_us - reference.exposure_us) <=
                           std::max<int64_t>(kExposureSlackUs, std::llround(opt_.threshold * reference.exposure_us));
  const bool gain_ok = std::abs(value.gain_raw - reference.gain_raw) <= kGainSlackRaw ||
                       relative_change(value.gain_raw, reference.gain_raw) <= opt_.threshold;
  return exposure_ok && gain_ok;
}

void ExposureFollower::start_transition(const SensorExposure & pin)
{
  pin_ = pin;
  in_transition_ = true;
  readback_matched_ = last_right_ && within(*last_right_, pin);
  settle_left_ = opt_.settle_frames;
  window_left_ = opt_.transition_frames;
  ++pins_;
}

std::optional<SensorExposure> ExposureFollower::on_left(const SensorExposure & left)
{
  bool repin = !pin_;
  if (pin_ && in_transition_) {
    repin = relative_change(left.exposure_us, pin_->exposure_us) > opt_.threshold ||
            relative_change(left.gain_raw, pin_->gain_raw) > opt_.threshold;
  } else if (pin_) {
    repin = !last_right_ || !within(*last_right_, left);
  }
  if (!repin) return std::nullopt;
  start_transition(left);
  return left;
}

void ExposureFollower::on_right(const SensorExposure & right)
{
  last_right_ = right;
  if (pin_ && in_transition_ && !readback_matched_) readback_matched_ = within(right, *pin_);
}

void ExposureFollower::on_right_frame()
{
  if (!in_transition_) return;
  if (readback_matched_ && settle_left_ > 0) --settle_left_;
  if (window_left_ > 0) --window_left_;
  if ((readback_matched_ && settle_left_ == 0) || window_left_ == 0) in_transition_ = false;
}

bool ExposureFollower::pairs_flow() const { return pin_ && !in_transition_; }

bool ExposureFollower::converged() const
{
  return pin_ && !in_transition_ && last_right_ && within(*last_right_, *pin_);
}

void ExposureFollower::reset()
{
  pin_.reset();
  last_right_.reset();
  in_transition_ = false;
  readback_matched_ = false;
  settle_left_ = 0;
  window_left_ = 0;
}

bool FollowStarvationGuard::on_pair(int64_t now_ns, bool converged)
{
  if (converged) {
    since_ns_ = now_ns;  // the clock runs from the last converged pair
    fired_ = false;
    return false;
  }
  if (!since_ns_) since_ns_ = now_ns;
  if (fired_ || now_ns - *since_ns_ < limit_ns_) return false;
  fired_ = true;
  return true;
}

void FollowStarvationGuard::reset()
{
  since_ns_.reset();
  fired_ = false;
}

SaturationOptions SaturationOptions::from_config(const ExposureConfig & exposure)
{
  SaturationOptions o;
  o.high = exposure.saturated_high;
  o.low = exposure.saturated_low;
  o.step = exposure.compensation_step;
  o.min = exposure.compensation_min;
  o.hold_frames = exposure.compensation_hold_frames;
  return o;
}

SaturationCompensator::SaturationCompensator(SaturationOptions options) : opt_(options) {}

std::optional<double> SaturationCompensator::on_measurement(double saturated_fraction, int64_t frame)
{
  if (last_step_frame_ && frame - *last_step_frame_ < opt_.hold_frames) return std::nullopt;
  double next = compensation_;
  if (saturated_fraction > opt_.high) {
    next = std::max(opt_.min, compensation_ - opt_.step);
  } else if (saturated_fraction < opt_.low) {
    next = std::min(0.0, compensation_ + opt_.step);
  }
  if (next == compensation_) return std::nullopt;
  compensation_ = next;
  last_step_frame_ = frame;
  ++adjustments_;
  return compensation_;
}

double saturated_fraction(const cv::cuda::GpuMat & grey, int level, cv::cuda::GpuMat & scratch)
{
  if (grey.empty()) return 0.0;
  cv::cuda::compare(grey, cv::Scalar::all(level), scratch, cv::CMP_GE);
  return static_cast<double>(cv::cuda::countNonZero(scratch)) / (static_cast<double>(grey.rows) * grey.cols);
}

}  // namespace depthd
