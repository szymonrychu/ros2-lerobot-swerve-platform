#include "head_aim_cpp/gate.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace head_aim_cpp
{

namespace
{

constexpr double DEG_TO_RAD = M_PI / 180.0;

}  // namespace

bool any_goal_active(const std::vector<int8_t> & statuses)
{
  return std::any_of(statuses.begin(), statuses.end(), [](int8_t s) {
    return s == GOAL_STATUS_ACCEPTED || s == GOAL_STATUS_EXECUTING || s == GOAL_STATUS_CANCELING;
  });
}

AimGate::AimGate(const Config & cfg) : cfg_(cfg) {}

void AimGate::on_goal_status(bool active, double /*now*/)
{
  if (active && !active_) {
    last_.reset();
    home_pending_ = false;
  } else if (!active && active_) {
    home_pending_ = true;
  }
  active_ = active;
}

void AimGate::on_foreign_command(const std::string & source, double now)
{
  if (source == cfg_.source_tag) {
    return;
  }
  if (std::find(cfg_.yield_sources.begin(), cfg_.yield_sources.end(), source) == cfg_.yield_sources.end()) {
    return;
  }
  yield_until_ = std::max(yield_until_, now + cfg_.yield_s);
  last_.reset();
}

bool AimGate::yielding(double now) const { return now < yield_until_; }

std::optional<Command> AimGate::decide(const std::optional<Aim> & target, double now)
{
  if (!active_) {
    const bool home = home_pending_ && !yielding(now);
    home_pending_ = false;
    if (!home) {
      return std::nullopt;
    }
    last_time_ = now;
    return Command{cfg_.home_pan_rad, cfg_.home_tilt_rad, true};
  }
  if (yielding(now) || !target) {
    return std::nullopt;
  }
  if (last_) {
    const bool moved = std::abs(target->pan - last_->pan) > cfg_.reaim_pan_deg * DEG_TO_RAD ||
                       std::abs(target->tilt - last_->tilt) > cfg_.reaim_tilt_deg * DEG_TO_RAD;
    if (!moved || now - last_time_ < cfg_.min_reaim_interval_s) {
      return std::nullopt;
    }
  }
  last_ = Command{target->pan, target->tilt, false};
  last_time_ = now;
  return last_;
}

}  // namespace head_aim_cpp
