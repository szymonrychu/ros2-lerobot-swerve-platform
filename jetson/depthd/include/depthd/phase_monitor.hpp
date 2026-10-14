// Runtime camera-phase drift policy (part of depthd-capture).
//
// The two IMX219s are free-running. StereoCapture aligns their frame phase once when the cameras start (right-camera
// restarts until the phase is within half the pair window), but the phase walks away over minutes: the pair skew
// grows past pair_window_ms, the pairer drops nearly every candidate and depth throughput collapses until the stream
// stalls. PhaseDriftMonitor watches the skew of every candidate the pairer judges and the time since the last pair,
// and tells the capture watchdog when to re-run the alignment:
//   skew:  the median skew of the trailing window (capture.realign_window_s, judged only once a full window has passed
//          since arming) is above capture.realign_skew_ms;
//   stall: no pair for capture.realign_stall_s while both cameras deliver frames.
// A decision disarms the monitor; the watchdog arms it again once the alignment has finished, which starts a fresh
// window. Decisions are at least capture.realign_cooldown_s apart. Times are any monotonic clock (ns), so tests drive
// it with a fake clock. Thread safe: the capture streaming threads observe, the watchdog polls.
#ifndef DEPTHD_PHASE_MONITOR_HPP_
#define DEPTHD_PHASE_MONITOR_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

#include "depthd/config.hpp"

namespace depthd
{

enum class RealignReason
{
  none,
  skew,
  stall
};

const char * to_string(RealignReason reason);

struct PhaseDriftOptions
{
  double skew_threshold_ms = 4.0;
  int64_t window_ns = 10000000000;
  int64_t stall_ns = 5000000000;
  int64_t cooldown_ns = 30000000000;

  static PhaseDriftOptions from_config(const Capture & capture);
};

class PhaseDriftMonitor
{
public:
  static constexpr std::size_t kMinSamples = 10;     // fewer judged candidates in the window: no skew verdict
  static constexpr std::size_t kMaxSamples = 4096;  // bound on the window (30 fps gives ~60 candidates a second)

  struct Decision
  {
    RealignReason reason = RealignReason::none;
    double median_skew_ms = 0.0;  // of the trailing window, 0 when it holds no sample
    int64_t since_pair_ns = 0;    // since the last pair, or since arming when none came
  };

  explicit PhaseDriftMonitor(PhaseDriftOptions options);

  void arm(int64_t now_ns);  // the cameras are aligned: start judging (no-op when already armed)
  void disarm();             // aligning or a camera in error: judge nothing, forget the window
  bool armed() const;

  void observe_skew(int64_t now_ns, double skew_ms);  // every candidate the pairer judged
  void observe_pair(int64_t now_ns);                  // every pair the pairer emitted

  // The decision at now_ns; anything but none disarms the monitor and starts the cooldown.
  Decision poll(int64_t now_ns);
  double median_skew_ms(int64_t now_ns) const;

private:
  void prune_locked(int64_t now_ns) const;
  double median_locked() const;

  const PhaseDriftOptions opt_;
  mutable std::mutex mutex_;
  bool armed_ = false;
  int64_t armed_at_ns_ = 0;
  std::optional<int64_t> last_pair_ns_;
  std::optional<int64_t> last_decision_ns_;
  mutable std::deque<std::pair<int64_t, double>> samples_;  // (time, skew ms), oldest first
};

}  // namespace depthd

#endif  // DEPTHD_PHASE_MONITOR_HPP_
