#include "map_guard_cpp/guard.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include "map_guard_cpp/backup.hpp"

namespace map_guard_cpp
{

namespace
{

constexpr const char * DRY_RUN_LOG_PREFIX = "[dry run] ";

}  // namespace

std::string state_name(State state)
{
  switch (state) {
    case State::OK:
      return "ok";
    case State::DISTURBED:
      return "disturbed";
    case State::SETTLING:
      return "settling";
    case State::EVALUATING:
      return "evaluating";
    case State::RELOCALISING:
      return "relocalising";
    case State::RESUMED:
      return "resumed";
    case State::RESET:
      return "reset";
  }
  throw std::invalid_argument("unknown state");
}

std::string fmt(const std::optional<double> & value)
{
  if (!value) {
    return "n/a";
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.2f", *value);
  return buf;
}

std::string float_repr(double value)
{
  if (std::isnan(value)) {
    return "nan";
  }
  if (std::isinf(value)) {
    return value > 0 ? "inf" : "-inf";
  }
  // Python's repr: the shortest round-trip digits, scientific below 1e-4 and from 1e16 on, else fixed with ".0" for
  // integral values. to_chars gives the same shortest digits and a two-digit exponent ("1e-05").
  char buf[64];
  auto result = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::scientific);
  const std::string scientific(buf, result.ptr);
  const int exponent = std::stoi(scientific.substr(scientific.find('e') + 1));
  if (exponent < -4 || exponent >= 16) {
    return scientific;
  }
  result = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::fixed);
  std::string fixed(buf, result.ptr);
  if (fixed.find('.') == std::string::npos) {
    fixed += ".0";
  }
  return fixed;
}

GuardMetrics::GuardMetrics(ros2_common_cpp::Metrics & metrics)
: metrics_(metrics),
  dry_run_(metrics.gauge("map_guard_dry_run", "1 while the guard runs in dry run (no slam_toolbox calls)")),
  pauses_(metrics.counter("map_guard_pauses_total", "SLAM pauses (new measurements) requested; dry run included")),
  resumes_(metrics.counter("map_guard_resumes_total", "SLAM resumed after the scan agreed with the map again"))
{
  for (State state : ALL_STATES) {
    state_[state] = &metrics_.gauge(
      "map_guard_state", "Map guard state (1 for the current state, 0 for the others)", {{"state", state_name(state)}});
  }
}

void GuardMetrics::set_agreement_ratio(double ratio)
{
  if (agreement_ratio_ == nullptr) {
    agreement_ratio_ =
      &metrics_.gauge("map_guard_agreement_ratio", "Latest scan-to-map agreement ratio (agreeing / evaluated)");
  }
  agreement_ratio_->Set(ratio);
}

void GuardMetrics::set_tilt_deg(double tilt_deg)
{
  if (tilt_deg_ == nullptr) {
    tilt_deg_ = &metrics_.gauge("map_guard_tilt_deg", "Roll/pitch difference from the level baseline, deg");
  }
  tilt_deg_->Set(tilt_deg);
}

void GuardMetrics::set_unexplained_speed(double speed_mps)
{
  if (unexplained_speed_ == nullptr) {
    unexplained_speed_ = &metrics_.gauge(
      "map_guard_unexplained_speed_mps", "Lidar odometry minus wheel odometry speed while no drive command is active");
  }
  unexplained_speed_->Set(speed_mps);
}

void GuardMetrics::set_state(State state)
{
  for (auto & [s, gauge] : state_) {
    gauge->Set(s == state ? 1 : 0);
  }
}

void GuardMetrics::set_dry_run(bool dry_run) { dry_run_.Set(dry_run ? 1 : 0); }

void GuardMetrics::inc_trigger(const std::string & reason)
{
  metrics_.counter("map_guard_triggers_total", "Disturbance or mismatch triggers acted on", {{"reason", reason}})
    .Increment();
}

void GuardMetrics::inc_pauses() { pauses_.Increment(); }

void GuardMetrics::inc_resumes() { resumes_.Increment(); }

void GuardMetrics::inc_resets(bool dry_run)
{
  metrics_.counter("map_guard_resets_total", "Map backup + SLAM resets", {{"dry_run", dry_run ? "true" : "false"}})
    .Increment();
}

MapGuard::MapGuard(
  const Config & config, SlamPort & slam, Notifier & notifier, std::function<void()> publish_reset,
  std::function<void(const std::string &, double)> request_relocalise, std::function<void(const std::string &)> warn,
  GuardMetrics & metrics, Clock utc_now)
: config_(config),
  slam_(slam),
  notifier_(notifier),
  publish_reset_(std::move(publish_reset)),
  request_relocalise_(std::move(request_relocalise)),
  warn_(std::move(warn)),
  metrics_(metrics),
  utc_now_(utc_now ? std::move(utc_now) : Clock([]() { return std::chrono::system_clock::now(); })),
  enabled_(config.reasons.begin(), config.reasons.end()),
  monitor_(
    config.mismatch_ratio, config.healthy_ratio, config.mismatch_s, config.min_map_cells_known,
    static_cast<int>(config.median_window))
{
  metrics_.set_dry_run(config.dry_run);
  metrics_.set_state(state_);
}

std::string MapGuard::dry() const { return config_.dry_run ? DRY_RUN_LOG_PREFIX : ""; }

void MapGuard::transition(double now, State state, const std::string & reason, const std::string & detail)
{
  warn_(
    dry() + "map_guard " + state_name(state_) + " -> " + state_name(state) + ": reason " + reason + ", ratio " +
    fmt(ratio()) + (detail.empty() ? "" : ", " + detail));
  state_ = state;
  state_since_ = now;
  metrics_.set_state(state_);
}

void MapGuard::set_hold(bool held)
{
  if (held == held_) {
    return;
  }
  held_ = held;
  if (held) {
    warn_("map_guard: holding while the loaded map is not localised");
    return;
  }
  monitor_.reset();
  if (state_ != State::RELOCALISING) {  // our own relocalise stays pending: its answer decides the episode
    episode_reasons_.clear();
    evaluation_.clear();
    state_ = State::OK;
    metrics_.set_state(state_);
  }
  warn_("map_guard: loaded map localised, guard re-armed");
}

void MapGuard::tick(double now, const DisturbanceReport & report, const std::vector<Sample> & samples)
{
  last_now_ = now;
  if (held_) {
    check_relocalise_deadline(now);
    return;
  }
  if (report.tilt_deg) {
    metrics_.set_tilt_deg(*report.tilt_deg);
  }
  if (report.unexplained_speed_mps) {
    metrics_.set_unexplained_speed(*report.unexplained_speed_mps);
  }
  bool mismatch = false;
  for (const auto & [ratio, known_cells] : samples) {
    metrics_.set_agreement_ratio(ratio);
    mismatch = monitor_.update(now, ratio, known_cells) || mismatch;
    if (state_ == State::EVALUATING) {
      evaluation_.push_back(ratio);
    }
  }
  const bool in_cooldown = cooldown_until_ && now < *cooldown_until_;
  mismatch = mismatch && enabled_.count(REASON_MISMATCH) && !in_cooldown;
  std::set<std::string> physical;
  for (const auto & reason : report.reasons) {
    if (enabled_.count(reason) && reason != REASON_MISMATCH) {
      physical.insert(reason);
    }
  }

  if (state_ == State::RESUMED || state_ == State::RESET) {
    transition(now, State::OK, reason_text());
    episode_reasons_.clear();
  }
  if (state_ == State::OK) {
    if (!physical.empty()) {
      start_episode(now, physical, State::DISTURBED);
    } else if (mismatch) {
      start_episode(now, {REASON_MISMATCH}, State::EVALUATING);
    }
    return;
  }
  if (!physical.empty()) {
    add_reasons(now, physical);
    if (state_ != State::DISTURBED) {
      transition(now, State::DISTURBED, reason_text());
    }
    return;
  }
  if (state_ == State::DISTURBED) {
    transition(now, State::SETTLING, reason_text());
  } else if (state_ == State::SETTLING && now - state_since_ >= config_.settle_s - TIME_EPS) {
    evaluation_.clear();
    transition(now, State::EVALUATING, reason_text());
  } else if (state_ == State::EVALUATING && now - state_since_ >= config_.evaluate_s - TIME_EPS) {
    decide(now);
  } else {
    check_relocalise_deadline(now);
  }
}

void MapGuard::check_relocalise_deadline(double now)
{
  if (state_ == State::RELOCALISING && relocalise_deadline_ && now >= *relocalise_deadline_ - TIME_EPS) {
    warn_(
      "map_guard: no relocalise answer for " + float_repr(config_.relocalise_timeout_s) +
      " s; treated as unrecoverable");
    finish_relocalise(now, false, 0);
  }
}

std::string MapGuard::reason_text() const
{
  std::string out;
  for (const auto & reason : episode_reasons_) {
    out += (out.empty() ? "" : ",") + reason;
  }
  return out.empty() ? "none" : out;
}

std::string MapGuard::primary_reason() const
{
  for (const char * reason : {REASON_LIFTED, REASON_CARRIED, REASON_ROTATED, REASON_MISMATCH}) {
    if (episode_reasons_.count(reason)) {
      return reason;
    }
  }
  return REASON_MISMATCH;
}

void MapGuard::start_episode(double now, const std::set<std::string> & reasons, State state)
{
  episode_reasons_.clear();
  evaluation_.clear();
  episode_started_ = now;
  add_reasons(now, reasons, false);
  transition(now, state, reason_text());
  metrics_.inc_pauses();
  set_paused(true);
}

void MapGuard::add_reasons(double now, const std::set<std::string> & reasons, bool log)
{
  for (const auto & reason : reasons) {  // sorted, as Python's sorted(reasons - episode_reasons)
    if (episode_reasons_.count(reason)) {
      continue;
    }
    episode_reasons_.insert(reason);
    metrics_.inc_trigger(reason);
    if (log) {
      warn_(dry() + "map_guard " + state_name(state_) + ": also " + reason + ", ratio " + fmt(ratio()));
    }
    notifier_.notify(now, EVENT_DISTURBED, reason, ratio(), {});
  }
}

void MapGuard::decide(double now)
{
  const std::string reason = primary_reason();
  if (evaluation_.empty()) {
    transition(now, State::RESUMED, reason_text(), "no agreement sample (no map or too few points)");
    resume(now, reason, std::nullopt);
    return;
  }
  const double med = median(evaluation_);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.2f", med);
  const std::string detail = std::string("evaluation median ") + buf + " over " + std::to_string(evaluation_.size()) +
                             " samples (healthy >= " + float_repr(config_.healthy_ratio) + ")";
  if (med >= config_.healthy_ratio) {
    transition(now, State::RESUMED, reason_text(), detail);
    resume(now, reason, med);
  } else {
    transition(now, State::RELOCALISING, reason_text(), detail);
    start_relocalise(now, med);
  }
}

void MapGuard::start_relocalise(double now, double median)
{
  relocalise_median_ = median;
  pending_request_.reset();
  relocalise_deadline_.reset();
  if (config_.dry_run) {
    warn_(dry() + "map_guard would request relocalise on " + config_.command_topic + "; counted as recovered");
    transition(now, State::RESUMED, reason_text(), "dry run: relocalise assumed to succeed");
    resume(now, primary_reason(), median);
    return;
  }
  ++request_seq_;
  pending_request_ = "map_guard-" + std::to_string(request_seq_);
  relocalise_deadline_ = now + config_.relocalise_timeout_s;
  const double utc_s = std::chrono::duration<double>(utc_now_().time_since_epoch()).count();
  request_relocalise_(*pending_request_, utc_s - (now - episode_started_));
}

void MapGuard::on_relocalise_result(
  const std::string & request_id, bool ok, const std::string & state, int discarded_pois)
{
  if (state_ != State::RELOCALISING || !pending_request_ || request_id != *pending_request_) {
    return;
  }
  finish_relocalise(last_now_, ok && state == "localised", discarded_pois);
}

void MapGuard::finish_relocalise(double now, bool recovered, int discarded_pois)
{
  const std::string reason = primary_reason();
  const std::optional<double> median = relocalise_median_;
  pending_request_.reset();
  relocalise_deadline_.reset();
  if (recovered) {
    transition(now, State::RESUMED, reason_text(), "relocalised against the live map");
    resume(now, reason, median);
  } else {
    transition(
      now, State::RESET, reason_text(), "unrecoverable, " + std::to_string(discarded_pois) + " POIs discarded");
    reset_map(now, reason, median.value_or(0.0), discarded_pois);
  }
}

void MapGuard::resume(double now, const std::string & reason, const std::optional<double> & median)
{
  set_paused(false);
  monitor_.reset_hold();
  metrics_.inc_resumes();
  notifier_.notify(now, EVENT_RESUMED, reason, median, {});
}

void MapGuard::reset_map(double now, const std::string & reason, double median, int discarded_pois)
{
  const Config & cfg = config_;
  const std::string target =
    (std::filesystem::path(cfg.backup_dir) / backup_name(cfg.backup_prefix, utc_now_())).lexically_normal().string();
  metrics_.inc_resets(cfg.dry_run);
  cooldown_until_ = now + cfg.cooldown_s;
  monitor_.reset();
  if (!slam_.serialize_map(target)) {
    warn_("map_guard: serialize_map to " + target + " failed; map NOT reset");
    set_paused(false);
    return;
  }
  for (const auto & path : prune_backups(cfg.backup_dir, cfg.backup_prefix, cfg.keep_backups)) {
    warn_("map_guard: deleted old backup " + path.string());
  }
  if (!slam_.reset()) {
    warn_("map_guard: slam_toolbox reset failed");
    set_paused(false);
    return;
  }
  set_paused(false);
  publish_reset_();
  notifier_.notify(now, EVENT_RESET, reason, median, {{"backup", target}, {"discarded_pois", discarded_pois}});
}

void MapGuard::set_paused(bool desired)
{
  if (config_.dry_run) {
    warn_(dry() + "map_guard would " + (desired ? "pause" : "unpause") + " new measurements");
    paused_ = desired;
    return;
  }
  const std::optional<bool> actual = slam_.query_paused();
  const std::optional<bool> current = actual ? actual : paused_;
  if (current == desired) {
    paused_ = desired;
    return;
  }
  if (!slam_.toggle_pause()) {
    warn_(
      std::string("map_guard: pause toggle failed; new measurements stay ") +
      (current.value_or(false) ? "paused" : "active"));
    paused_ = current;
    return;
  }
  const std::optional<bool> verified = slam_.query_paused();
  if (verified && *verified != desired) {
    warn_(
      std::string("map_guard: pause toggle answered but new measurements are still ") +
      (*verified ? "paused" : "active"));
    paused_ = verified;
    return;
  }
  paused_ = desired;
}

}  // namespace map_guard_cpp
