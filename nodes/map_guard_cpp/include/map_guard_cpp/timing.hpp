// Time helpers of the map guard (monotonic seconds): a condition that must hold for a duration, and a rate limiter.
// Port of nodes/map_guard/map_guard/timing.py.
#ifndef MAP_GUARD_CPP__TIMING_HPP_
#define MAP_GUARD_CPP__TIMING_HPP_

#include <optional>

namespace map_guard_cpp
{

// Slack of every "held long enough" comparison, the 1e-9 of the Python helpers.
constexpr double TIME_EPS = 1e-9;

// True once a condition has held continuously for duration_s; any false sample restarts it.
class HoldTimer
{
public:
  explicit HoldTimer(double duration_s);

  // Feed one sample of the condition at now (s). Returns true when it has been true since at least duration_s ago.
  bool update(double now, bool condition);

  // Forget the running hold.
  void reset();

private:
  double duration_s_;
  std::optional<double> since_;
};

// Allows an action at most rate_hz times per second.
class RateLimiter
{
public:
  explicit RateLimiter(double rate_hz);

  // Whether the action may run at now (s); records the call when it may.
  bool ready(double now);

private:
  double period_s_;
  std::optional<double> last_;
};

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__TIMING_HPP_
