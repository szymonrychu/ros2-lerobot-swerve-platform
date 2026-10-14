#include "depthd/phase_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace depthd
{

const char * to_string(RealignReason reason)
{
  switch (reason) {
    case RealignReason::skew:
      return "skew";
    case RealignReason::stall:
      return "stall";
    case RealignReason::none:
      break;
  }
  return "none";
}

PhaseDriftOptions PhaseDriftOptions::from_config(const Capture & capture)
{
  const auto ns = [](double s) { return static_cast<int64_t>(std::llround(s * 1e9)); };
  PhaseDriftOptions o;
  o.skew_threshold_ms = capture.realign_skew_ms;
  o.window_ns = ns(capture.realign_window_s);
  o.stall_ns = ns(capture.realign_stall_s);
  o.cooldown_ns = ns(capture.realign_cooldown_s);
  return o;
}

PhaseDriftMonitor::PhaseDriftMonitor(PhaseDriftOptions options) : opt_(options) {}

void PhaseDriftMonitor::arm(int64_t now_ns)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (armed_) return;
  armed_ = true;
  armed_at_ns_ = now_ns;
  last_pair_ns_.reset();
  samples_.clear();
}

void PhaseDriftMonitor::disarm()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  armed_ = false;
  last_pair_ns_.reset();
  samples_.clear();
}

bool PhaseDriftMonitor::armed() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return armed_;
}

void PhaseDriftMonitor::observe_skew(int64_t now_ns, double skew_ms)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!armed_) return;
  samples_.emplace_back(now_ns, std::abs(skew_ms));
  while (samples_.size() > kMaxSamples) samples_.pop_front();
}

void PhaseDriftMonitor::observe_pair(int64_t now_ns)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (armed_) last_pair_ns_ = now_ns;
}

void PhaseDriftMonitor::prune_locked(int64_t now_ns) const
{
  while (!samples_.empty() && now_ns - samples_.front().first > opt_.window_ns) samples_.pop_front();
}

double PhaseDriftMonitor::median_locked() const
{
  if (samples_.empty()) return 0.0;
  std::vector<double> v;
  v.reserve(samples_.size());
  for (const auto & s : samples_) v.push_back(s.second);
  const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
  std::nth_element(v.begin(), mid, v.end());
  return *mid;
}

PhaseDriftMonitor::Decision PhaseDriftMonitor::poll(int64_t now_ns)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  Decision d;
  if (!armed_) return d;
  prune_locked(now_ns);
  d.median_skew_ms = median_locked();
  d.since_pair_ns = now_ns - (last_pair_ns_ ? *last_pair_ns_ : armed_at_ns_);
  if (last_decision_ns_ && now_ns - *last_decision_ns_ < opt_.cooldown_ns) return d;
  if (d.since_pair_ns > opt_.stall_ns) {
    d.reason = RealignReason::stall;
  } else if (
    now_ns - armed_at_ns_ >= opt_.window_ns && samples_.size() >= kMinSamples &&
    d.median_skew_ms > opt_.skew_threshold_ms) {
    d.reason = RealignReason::skew;
  }
  if (d.reason != RealignReason::none) {
    last_decision_ns_ = now_ns;
    armed_ = false;
    last_pair_ns_.reset();
    samples_.clear();
  }
  return d;
}

double PhaseDriftMonitor::median_skew_ms(int64_t now_ns) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  prune_locked(now_ns);
  return median_locked();
}

}  // namespace depthd
