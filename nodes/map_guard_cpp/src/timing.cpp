#include "map_guard_cpp/timing.hpp"

namespace map_guard_cpp
{

HoldTimer::HoldTimer(double duration_s) : duration_s_(duration_s) {}

bool HoldTimer::update(double now, bool condition)
{
  if (!condition) {
    since_.reset();
    return false;
  }
  if (!since_) {
    since_ = now;
  }
  return now - *since_ >= duration_s_ - TIME_EPS;
}

void HoldTimer::reset() { since_.reset(); }

RateLimiter::RateLimiter(double rate_hz) : period_s_(1.0 / rate_hz) {}

bool RateLimiter::ready(double now)
{
  if (last_ && now - *last_ < period_s_ - TIME_EPS) {
    return false;
  }
  last_ = now;
  return true;
}

}  // namespace map_guard_cpp
