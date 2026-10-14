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
  return o;
}

ExposureFollower::ExposureFollower(ExposureFollowerOptions options) : opt_(options) {}

std::optional<SensorExposure> ExposureFollower::on_left(const SensorExposure & left)
{
  if (pin_ && relative_change(left.exposure_us, pin_->exposure_us) <= opt_.threshold &&
      relative_change(left.gain_raw, pin_->gain_raw) <= opt_.threshold) {
    return std::nullopt;
  }
  pin_ = left;
  readback_matched_ = false;
  settle_left_ = opt_.settle_frames;
  ++pins_;
  return left;
}

void ExposureFollower::on_right(const SensorExposure & right)
{
  if (!pin_ || readback_matched_) return;
  const bool exposure_ok = std::abs(right.exposure_us - pin_->exposure_us) <=
                           std::max<int64_t>(kExposureSlackUs, std::llround(opt_.threshold * pin_->exposure_us));
  const bool gain_ok = std::abs(right.gain_raw - pin_->gain_raw) <= 1 ||
                       relative_change(right.gain_raw, pin_->gain_raw) <= opt_.threshold;
  readback_matched_ = exposure_ok && gain_ok;
}

void ExposureFollower::on_right_frame()
{
  if (readback_matched_ && settle_left_ > 0) --settle_left_;
}

bool ExposureFollower::matched() const { return pin_ && readback_matched_ && settle_left_ == 0; }

void ExposureFollower::reset()
{
  pin_.reset();
  readback_matched_ = false;
  settle_left_ = 0;
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
