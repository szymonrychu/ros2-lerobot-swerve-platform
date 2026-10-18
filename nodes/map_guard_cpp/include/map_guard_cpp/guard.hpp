// Map guard state machine. Port of nodes/map_guard/map_guard/guard.py:
//
//   OK --(lifted / carried / rotated)--> DISTURBED (pause new measurements, notify)
//   OK --(mismatch)--> EVALUATING (pause, notify; no settling: nobody is handling the robot)
//   DISTURBED --(no detector fires)--> SETTLING --(settle_s quiet)--> EVALUATING
//   SETTLING / EVALUATING --(any physical detector)--> DISTURBED
//   EVALUATING --(evaluate_s; median ratio >= healthy_ratio, or no sample at all)--> RESUMED (unpause) -> OK
//   EVALUATING --(evaluate_s; median ratio < healthy_ratio)--> RELOCALISING (stay paused, ask map_manager to
//       relocalise against the live map from the last pose before the move)
//   RELOCALISING --(result localised)--> RESUMED (map and POIs kept)
//   RELOCALISING --(result not localised / ambiguous / failed, or relocalise_timeout_s without an answer)--> RESET
//       (serialize backup, prune, reset, unpause, publish the reset topic; map_manager has already cleared the live
//       POIs) -> OK, with cooldown_s during which a mismatch cannot trigger
//
// The slam_toolbox pause service is a toggle: the guard reads the paused_new_measurements parameter (falling back to
// its own tracked state when the parameter cannot be read), toggles only when the state differs from the wanted one
// and re-reads it afterwards. In dry run no service is ever called and the reset topic is never published; the state
// machine, the logs and the metrics run unchanged.
#ifndef MAP_GUARD_CPP__GUARD_HPP_
#define MAP_GUARD_CPP__GUARD_HPP_

#include <prometheus/counter.h>
#include <prometheus/gauge.h>

#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "map_guard_cpp/agent.hpp"
#include "map_guard_cpp/agreement.hpp"
#include "map_guard_cpp/config.hpp"
#include "map_guard_cpp/detectors.hpp"
#include "ros2_common_cpp/metrics.hpp"

namespace map_guard_cpp
{

// Guard states (RESUMED and RESET last one tick; RELOCALISING lasts until the answer or the timeout).
enum class State
{
  OK,
  DISTURBED,
  SETTLING,
  EVALUATING,
  RELOCALISING,
  RESUMED,
  RESET
};
constexpr std::array<State, 7> ALL_STATES = {State::OK,           State::DISTURBED, State::SETTLING, State::EVALUATING,
                                             State::RELOCALISING, State::RESUMED,   State::RESET};

// "ok", "disturbed", ... (the map_guard_state label values).
std::string state_name(State state);

// Blocking slam_toolbox calls (the node implements them with service clients).
class SlamPort
{
public:
  virtual ~SlamPort() = default;
  // paused_new_measurements; nullopt when it cannot be read.
  virtual std::optional<bool> query_paused() = 0;
  // Call the Pause toggle; true when the service answered with status true.
  virtual bool toggle_pause() = 0;
  // Serialize the pose graph and map to filename (no extension); true on RESULT_SUCCESS.
  virtual bool serialize_map(const std::string & filename) = 0;
  // Reset the map and pose graph; true on RESULT_SUCCESS.
  virtual bool reset() = 0;
};

// The map_guard_* metrics of nodes/map_guard/map_guard/metrics.py, in a ros2_common_cpp::Metrics registry.
// map_guard_agreement_ratio, map_guard_tilt_deg and map_guard_unexplained_speed_mps appear with their first real
// value (no placeholder 0 before it); labelled counter children appear with their first increment.
class GuardMetrics
{
public:
  explicit GuardMetrics(ros2_common_cpp::Metrics & metrics);

  void set_agreement_ratio(double ratio);
  void set_tilt_deg(double tilt_deg);
  void set_unexplained_speed(double speed_mps);
  // 1 for state, 0 for the others.
  void set_state(State state);
  void set_dry_run(bool dry_run);
  void inc_trigger(const std::string & reason);
  void inc_pauses();
  void inc_resumes();
  void inc_resets(bool dry_run);

private:
  ros2_common_cpp::Metrics & metrics_;
  prometheus::Gauge * agreement_ratio_ = nullptr;
  prometheus::Gauge * tilt_deg_ = nullptr;
  prometheus::Gauge * unexplained_speed_ = nullptr;
  std::map<State, prometheus::Gauge *> state_;
  prometheus::Gauge & dry_run_;
  prometheus::Counter & pauses_;
  prometheus::Counter & resumes_;
};

// Python's "{:.2f}" of a number, "n/a" without one.
std::string fmt(const std::optional<double> & value);

// Python's repr of a float ("0.6", "1.0", "1e-05").
std::string float_repr(double value);

// Drives pause / resume / reset of slam_toolbox from the detector reports and agreement samples.
class MapGuard
{
public:
  using Clock = std::function<std::chrono::system_clock::time_point()>;
  // (ratio, known_cells) of one agreement sample.
  using Sample = std::pair<double, long>;  // NOLINT(runtime/int)

  // config, slam, notifier and metrics must outlive the guard. publish_reset publishes std_msgs/Empty on the reset
  // topic (never in dry run); request_relocalise publishes the relocalise command to map_manager with (request_id,
  // since: epoch seconds when the episode started; never called in dry run); warn logs a warning; utc_now gives the
  // backup names their time.
  MapGuard(
    const Config & config, SlamPort & slam, Notifier & notifier, std::function<void()> publish_reset,
    std::function<void(const std::string &, double)> request_relocalise, std::function<void(const std::string &)> warn,
    GuardMetrics & metrics, Clock utc_now = nullptr);

  // Advance the state machine at now (monotonic s) with the detector report and the new agreement samples.
  void tick(double now, const DisturbanceReport & report, const std::vector<Sample> & samples);

  // The localisation verdict of map_manager (design D9). While held the tick does nothing: no state change, pause,
  // serialize, reset, reset publish or event. On held -> released the mismatch monitor is reset (re-armed only once
  // the new map is healthy) and the state returns to OK. Never called: the guard behaves as without it.
  // A hold during our own relocalise (map_manager reports not localised while it works) only blocks new evaluations:
  // the pending request, its deadline and the state stay, and the result still resolves it.
  void set_hold(bool held);
  bool held() const { return held_; }

  State state() const { return state_; }
  // Tracked pause state of slam_toolbox (none until the first pause request).
  std::optional<bool> paused() const { return paused_; }
  // map_manager's answer to the pending relocalise request; answers to other ids (or with none pending) are ignored.
  // state is the localisation state ("localised", "not_localised", "ambiguous", ...); discarded_pois the POIs
  // map_manager cleared from the live set.
  void on_relocalise_result(const std::string & request_id, bool ok, const std::string & state, int discarded_pois);

  // Latest rolling median agreement.
  std::optional<double> ratio() const { return monitor_.median(); }

private:
  std::string dry() const;
  void transition(double now, State state, const std::string & reason, const std::string & detail = "");
  std::string reason_text() const;
  std::string primary_reason() const;
  void start_episode(double now, const std::set<std::string> & reasons, State state);
  void add_reasons(double now, const std::set<std::string> & reasons, bool log = true);
  void decide(double now);
  void resume(double now, const std::string & reason, const std::optional<double> & median);
  void start_relocalise(double now, double median);
  void finish_relocalise(double now, bool recovered, int discarded_pois);
  void check_relocalise_deadline(double now);
  void reset_map(double now, const std::string & reason, double median, int discarded_pois);
  void set_paused(bool desired);

  const Config & config_;
  SlamPort & slam_;
  Notifier & notifier_;
  std::function<void()> publish_reset_;
  std::function<void(const std::string &, double)> request_relocalise_;
  std::function<void(const std::string &)> warn_;
  GuardMetrics & metrics_;
  Clock utc_now_;
  std::set<std::string> enabled_;
  MismatchMonitor monitor_;
  bool held_ = false;
  State state_ = State::OK;
  double state_since_ = 0.0;
  std::optional<bool> paused_;
  std::set<std::string> episode_reasons_;
  std::vector<double> evaluation_;
  std::optional<double> cooldown_until_;
  double episode_started_ = 0.0;
  double last_now_ = 0.0;
  long request_seq_ = 0;  // NOLINT(runtime/int)
  std::optional<std::string> pending_request_;
  std::optional<double> relocalise_deadline_;
  std::optional<double> relocalise_median_;
};

}  // namespace map_guard_cpp

#endif  // MAP_GUARD_CPP__GUARD_HPP_
