// Port of nodes/map_guard/tests/test_guard.py (transitions, pause toggle tracking, reset/backup, dry run, cooldown)
// plus the guard.json golden fixture: for every scenario the per-tick states, slam_toolbox calls, agent events, log
// lines, reset publishes, metric counts and backups left on disk equal the Python guard's.
#include <gtest/gtest.h>
#include <prometheus/metric_family.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "fixture_util.hpp"
#include "map_guard_cpp/guard.hpp"
#include "ros2_common_cpp/metrics.hpp"

namespace mg = map_guard_cpp;
namespace fx = map_guard_cpp::test;
namespace fs = std::filesystem;

namespace
{

constexpr double TICK = 0.1;
const mg::MapGuard::Sample HEALTHY = {0.9, 5000};
const mg::MapGuard::Sample BAD = {0.1, 5000};
// 2026-10-10T12:00:00Z, the FIXED_UTC of the Python test.
const auto FIXED_UTC = std::chrono::system_clock::time_point(std::chrono::seconds(1791633600));
using Labels = std::map<std::string, std::string>;

// slam_toolbox stand-in: the pause service toggles, the parameter reports the real state.
class FakeSlam : public mg::SlamPort
{
public:
  bool paused = false;
  bool toggle_ok = true;
  bool toggle_works = true;
  bool serialize_ok = true;
  bool reset_ok = true;
  bool query_none = false;
  std::vector<std::string> calls;  // "query", "toggle", "reset", "serialize <filename>"

  std::optional<bool> query_paused() override
  {
    calls.push_back("query");
    if (query_none) {
      return std::nullopt;
    }
    return paused;
  }
  bool toggle_pause() override
  {
    calls.push_back("toggle");
    if (toggle_ok && toggle_works) {
      paused = !paused;
    }
    return toggle_ok;
  }
  bool serialize_map(const std::string & filename) override
  {
    calls.push_back("serialize " + filename);
    if (serialize_ok) {
      std::ofstream(filename + ".posegraph") << "g";
      std::ofstream(filename + ".data") << "d";
    }
    return serialize_ok;
  }
  bool reset() override
  {
    calls.push_back("reset");
    return reset_ok;
  }
  long count(const std::string & call) const  // NOLINT(runtime/int)
  {
    return std::count(calls.begin(), calls.end(), call);
  }
  bool called(const std::string & call) const { return count(call) > 0; }
};

struct Event
{
  std::string kind;
  std::string reason;
  std::optional<double> ratio;
  mg::EventData data;
};

class FakeNotifier : public mg::Notifier
{
public:
  std::vector<Event> events;
  void notify(
    double, const std::string & kind, const std::string & reason, const std::optional<double> & ratio,
    const mg::EventData & data) override
  {
    events.push_back({kind, reason, ratio, data});
  }
};

// Python's round(value, 6) (correctly rounded through the decimal text).
double round6(double value)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6f", value);
  return std::strtod(buf, nullptr);
}

// Value of a sample of the registry, 0 when absent (prometheus_client's get_sample_value() or 0.0).
double sample_value(ros2_common_cpp::Metrics & metrics, const std::string & name, const Labels & labels = {})
{
  for (const auto & family : metrics.registry()->Collect()) {
    if (family.name != name) {
      continue;
    }
    for (const auto & metric : family.metric) {
      Labels have;
      for (const auto & label : metric.label) {
        have[label.name] = label.value;
      }
      if (have == labels) {
        return family.type == prometheus::MetricType::Counter ? metric.counter.value : metric.gauge.value;
      }
    }
  }
  return 0.0;
}

bool has_family(ros2_common_cpp::Metrics & metrics, const std::string & name)
{
  for (const auto & family : metrics.registry()->Collect()) {
    if (family.name == name && !family.metric.empty()) {
      return true;
    }
  }
  return false;
}

// The guard with fakes around it, ticking at 10 Hz like the Python Harness.
class Harness
{
public:
  explicit Harness(
    const std::function<void(mg::Config &)> & overrides = nullptr,
    const std::function<void(FakeSlam &)> & slam_setup = nullptr)
  {
    char tmpl[] = "/tmp/map_guard_cpp_guard_XXXXXX";
    dir = mkdtemp(tmpl);
    config.dry_run = false;
    config.backup_dir = dir.string();
    if (overrides) {
      overrides(config);
    }
    if (slam_setup) {
      slam_setup(slam);
    }
    guard = std::make_unique<mg::MapGuard>(
      config, slam, notifier, [this]() { ++resets_published; },
      [this](const std::string & id, double since) { relocalise_requests.emplace_back(id, since); },
      [this](const std::string & line) { logs.push_back(line); }, guard_metrics, []() { return FIXED_UTC; });
  }
  ~Harness() { fs::remove_all(dir); }

  // Tick at 10 Hz for seconds; an agreement sample every 0.5 s when given.
  void run(
    double seconds, const std::set<std::string> & reasons = {},
    const std::optional<mg::MapGuard::Sample> & sample = std::nullopt)
  {
    const int ticks = static_cast<int>(std::round(seconds / TICK));
    for (int i = 0; i < ticks; ++i) {
      mg::DisturbanceReport report{reasons, std::nullopt, std::nullopt, std::nullopt};
      std::vector<mg::MapGuard::Sample> samples;
      if (sample && static_cast<long>(std::round(now / TICK)) % 5 == 0) {  // NOLINT(runtime/int)
        samples.push_back(*sample);
      }
      guard->tick(now, report, samples);
      now = round6(now + TICK);
    }
  }

  // Answer the newest relocalise request as map_manager would.
  void answer(bool ok, const std::string & state, int discarded_pois = 0)
  {
    guard->on_relocalise_result(relocalise_requests.back().first, ok, state, discarded_pois);
  }

  // Answer the newest request as unrecoverable and let the guard finish.
  void fail_relocalise(int discarded_pois = 0)
  {
    answer(false, "not_localised", discarded_pois);
    run(0.2, {}, BAD);
  }

  // Carry the robot, settle and let a poor evaluation window end (the guard asks for a relocalise).
  void poor_evaluation()
  {
    run(0.1, {mg::REASON_CARRIED});
    run(3.2, {}, BAD);
    run(4.2, {}, BAD);
  }

  bool logged(const std::function<bool(const std::string &)> & match) const
  {
    return std::any_of(logs.begin(), logs.end(), match);
  }

  double counter(const std::string & name, const Labels & labels = {}) { return sample_value(metrics, name, labels); }
  double state_gauge(mg::State state) { return counter("map_guard_state", {{"state", mg::state_name(state)}}); }

  fs::path dir;
  mg::Config config;
  FakeSlam slam;
  FakeNotifier notifier;
  int resets_published = 0;
  std::vector<std::pair<std::string, double>> relocalise_requests;
  std::vector<std::string> logs;
  ros2_common_cpp::Metrics metrics{"map_guard_cpp"};
  mg::GuardMetrics guard_metrics{metrics};
  std::unique_ptr<mg::MapGuard> guard;
  double now = 0.0;
};

bool contains(const std::string & line, const std::string & part) { return line.find(part) != std::string::npos; }

const std::set<std::string> LIFTED = {mg::REASON_LIFTED};
const std::set<std::string> CARRIED = {mg::REASON_CARRIED};

}  // namespace

TEST(Guard, StartsOk)
{
  Harness h;
  h.run(1.0, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_TRUE(h.slam.calls.empty());
  EXPECT_EQ(h.state_gauge(mg::State::OK), 1.0);
}

TEST(Guard, LiftedPausesSettlesAndResumes)
{
  Harness h;
  h.run(1.0, {}, HEALTHY);
  h.run(0.1, LIFTED, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
  EXPECT_TRUE(h.slam.paused);
  EXPECT_EQ(h.guard->paused(), std::optional<bool>(true));
  EXPECT_EQ(h.slam.calls, (std::vector<std::string>{"query", "toggle", "query"}));
  EXPECT_EQ(h.counter("map_guard_pauses_total"), 1.0);
  EXPECT_EQ(h.counter("map_guard_triggers_total", {{"reason", "lifted"}}), 1.0);
  ASSERT_FALSE(h.notifier.events.empty());
  EXPECT_EQ(h.notifier.events[0].kind, mg::EVENT_DISTURBED);
  EXPECT_EQ(h.notifier.events[0].reason, mg::REASON_LIFTED);
  EXPECT_EQ(h.state_gauge(mg::State::DISTURBED), 1.0);
  EXPECT_EQ(h.state_gauge(mg::State::OK), 0.0);

  h.run(2.0, LIFTED, HEALTHY);  // still lifted: stays disturbed, no extra trigger
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
  EXPECT_EQ(h.counter("map_guard_triggers_total", {{"reason", "lifted"}}), 1.0);

  h.run(0.1, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::SETTLING);
  h.run(2.8, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::SETTLING);
  h.run(0.4, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::EVALUATING);
  h.run(4.1, {}, HEALTHY);
  EXPECT_TRUE(h.guard->state() == mg::State::RESUMED || h.guard->state() == mg::State::OK);
  h.run(0.2, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_FALSE(h.slam.paused);
  EXPECT_EQ(h.counter("map_guard_resumes_total"), 1.0);
  EXPECT_EQ(h.notifier.events.back().kind, mg::EVENT_RESUMED);
  EXPECT_EQ(h.notifier.events.back().reason, mg::REASON_LIFTED);
  EXPECT_FALSE(h.slam.called("reset"));
  EXPECT_EQ(h.resets_published, 0);
  EXPECT_TRUE(h.logged([](const std::string & l) { return contains(l, "disturbed") && contains(l, "lifted"); }));
}

TEST(Guard, NewTriggerWhileSettlingGoesBackToDisturbed)
{
  Harness h;
  h.run(0.1, LIFTED);
  h.run(1.0);
  EXPECT_EQ(h.guard->state(), mg::State::SETTLING);
  h.run(0.1, CARRIED);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
  EXPECT_EQ(h.slam.count("toggle"), 1);  // already paused: no second toggle
}

TEST(Guard, TriggerWhileEvaluatingGoesBackToDisturbed)
{
  Harness h;
  h.run(0.1, LIFTED);
  h.run(3.2, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::EVALUATING);
  h.run(0.1, CARRIED, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
}

TEST(Guard, LowAgreementAfterSettlingBacksUpAndResets)
{
  Harness h;
  h.run(0.1, CARRIED);
  h.run(3.2, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::EVALUATING);
  h.run(4.2, {}, BAD);
  h.fail_relocalise();
  h.run(0.2, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  const std::string expected = (h.dir / "map_guard_backup_20261010T120000Z").string();
  const auto serialize = std::find(h.slam.calls.begin(), h.slam.calls.end(), "serialize " + expected);
  ASSERT_NE(serialize, h.slam.calls.end());
  EXPECT_LT(serialize, std::find(h.slam.calls.begin(), h.slam.calls.end(), "reset"));
  EXPECT_FALSE(h.slam.paused);
  EXPECT_EQ(h.resets_published, 1);
  EXPECT_EQ(h.counter("map_guard_resets_total", {{"dry_run", "false"}}), 1.0);
  const auto & last = h.notifier.events.back();
  EXPECT_EQ(last.kind, mg::EVENT_RESET);
  EXPECT_EQ(last.reason, mg::REASON_CARRIED);
  EXPECT_NEAR(last.ratio.value(), 0.1, 1e-12);
  EXPECT_EQ(last.data.at("backup"), expected);
}

TEST(Guard, ResetPrunesOldBackups)
{
  Harness h([](mg::Config & c) { c.keep_backups = 3; });
  for (int i = 0; i < 6; ++i) {
    const std::string stem = "map_guard_backup_2026010" + std::to_string(i) + "T000000Z";
    std::ofstream(h.dir / (stem + ".posegraph")) << "g";
    std::ofstream(h.dir / (stem + ".data")) << "d";
  }
  h.run(0.1, CARRIED);
  h.run(8.0, {}, BAD);
  h.fail_relocalise();
  std::set<std::string> stems;
  for (const auto & entry : fs::directory_iterator(h.dir)) {
    stems.insert(entry.path().stem().string());
  }
  EXPECT_EQ(
    stems,
    (std::set<std::string>{
      "map_guard_backup_20260104T000000Z", "map_guard_backup_20260105T000000Z", "map_guard_backup_20261010T120000Z"}));
}

TEST(Guard, FailedBackupSkipsTheReset)
{
  Harness h(nullptr, [](FakeSlam & s) { s.serialize_ok = false; });
  h.run(0.1, CARRIED);
  h.run(8.0, {}, BAD);
  h.fail_relocalise();
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_FALSE(h.slam.called("reset"));
  EXPECT_FALSE(h.slam.paused);
  EXPECT_EQ(h.resets_published, 0);
  EXPECT_TRUE(h.logged([](const std::string & l) { return contains(l, "serialize"); }));
}

TEST(Guard, FailedResetDoesNotPublish)
{
  Harness h(nullptr, [](FakeSlam & s) { s.reset_ok = false; });
  h.run(0.1, CARRIED);
  h.run(8.0, {}, BAD);
  h.fail_relocalise();
  EXPECT_TRUE(h.slam.called("reset"));
  EXPECT_EQ(h.resets_published, 0);
  EXPECT_FALSE(h.slam.paused);
}

TEST(Guard, NoAgreementSamplesDuringEvaluationResumes)
{
  Harness h;
  h.run(0.1, LIFTED);
  h.run(8.0);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_FALSE(h.slam.called("reset"));
  EXPECT_EQ(h.notifier.events.back().kind, mg::EVENT_RESUMED);
}

TEST(Guard, MismatchAlonePausesAndEvaluatesWithoutSettling)
{
  Harness h;
  h.run(5.0, {}, HEALTHY);
  h.run(7.5, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::EVALUATING);
  EXPECT_TRUE(h.slam.paused);
  EXPECT_EQ(h.counter("map_guard_triggers_total", {{"reason", "mismatch"}}), 1.0);
  EXPECT_EQ(h.notifier.events[0].kind, mg::EVENT_DISTURBED);
  EXPECT_EQ(h.notifier.events[0].reason, mg::REASON_MISMATCH);
  h.run(4.5, {}, BAD);
  h.fail_relocalise();
  EXPECT_TRUE(h.slam.called("reset"));
  EXPECT_EQ(h.notifier.events.back().kind, mg::EVENT_RESET);
  EXPECT_EQ(h.notifier.events.back().reason, mg::REASON_MISMATCH);
}

TEST(Guard, CooldownBlocksMismatchRetrigger)
{
  Harness h([](mg::Config & c) { c.cooldown_s = 30.0; });
  h.run(5.0, {}, HEALTHY);
  h.run(12.5, {}, BAD);  // mismatch -> evaluate -> relocalise
  h.fail_relocalise();   // unrecoverable -> reset
  EXPECT_EQ(h.slam.count("reset"), 1);
  h.run(4.0, {}, HEALTHY);  // the new map is healthy (re-arms the monitor)
  h.run(10.0, {}, BAD);     // still inside the 30 s cooldown
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_EQ(h.slam.count("reset"), 1);
  h.run(25.0, {}, BAD);  // cooldown over: the mismatch fires, is evaluated and asks to relocalise again
  h.fail_relocalise();
  EXPECT_EQ(h.slam.count("reset"), 2);
}

TEST(Guard, CooldownDoesNotBlockPhysicalTriggers)
{
  Harness h;
  h.run(0.1, CARRIED);
  h.run(8.0, {}, BAD);
  h.fail_relocalise();
  EXPECT_EQ(h.slam.count("reset"), 1);
  h.run(0.1, LIFTED);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
}

TEST(Guard, DisabledReasonsAreIgnored)
{
  Harness h([](mg::Config & c) { c.reasons = {"mismatch"}; });
  h.run(3.0, {mg::REASON_LIFTED, mg::REASON_CARRIED});
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_TRUE(h.slam.calls.empty());
}

TEST(Guard, DisabledMismatchIsIgnored)
{
  Harness h([](mg::Config & c) { c.reasons = {"lifted"}; });
  h.run(5.0, {}, HEALTHY);
  h.run(15.0, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
}

// --- Pause toggle tracking ------------------------------------------------------------------------------------------

TEST(GuardPause, AlreadyPausedIsNotToggled)
{
  Harness h(nullptr, [](FakeSlam & s) { s.paused = true; });
  h.run(0.1, LIFTED);
  EXPECT_FALSE(h.slam.called("toggle"));
  EXPECT_EQ(h.guard->paused(), std::optional<bool>(true));
}

TEST(GuardPause, FailedToggleKeepsTheTrackedState)
{
  Harness h(nullptr, [](FakeSlam & s) { s.toggle_ok = false; });
  h.run(0.1, LIFTED);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
  EXPECT_EQ(h.guard->paused(), std::optional<bool>(false));
  EXPECT_TRUE(h.logged([](const std::string & l) { return contains(l, "pause"); }));
}

TEST(GuardPause, ToggleThatDidNotTakeEffectIsDetected)
{
  Harness h(nullptr, [](FakeSlam & s) { s.toggle_works = false; });
  h.run(0.1, LIFTED);
  EXPECT_EQ(h.guard->paused(), std::optional<bool>(false));
  EXPECT_TRUE(h.logged([](const std::string & l) { return contains(l, "still"); }));
}

TEST(GuardPause, UnknownParameterStateFallsBackToTracking)
{
  Harness h(nullptr, [](FakeSlam & s) { s.query_none = true; });
  h.run(0.1, LIFTED);
  EXPECT_EQ(h.slam.count("toggle"), 1);
  EXPECT_EQ(h.guard->paused(), std::optional<bool>(true));
  h.run(0.1, CARRIED);
  h.run(1.0);
  h.run(0.1, LIFTED);
  EXPECT_EQ(h.slam.count("toggle"), 1);  // tracked as paused: no second toggle
  h.run(8.0, {}, HEALTHY);
  EXPECT_EQ(h.slam.count("toggle"), 2);
  EXPECT_EQ(h.guard->paused(), std::optional<bool>(false));
}

// --- Relocalise before reset (design D13, U8) -----------------------------------------------------------------------

namespace
{

bool has_serialize(const FakeSlam & slam)
{
  return std::any_of(
    slam.calls.begin(), slam.calls.end(), [](const std::string & c) { return c.rfind("serialize ", 0) == 0; });
}

}  // namespace

TEST(GuardRelocalise, PoorMedianRequestsRelocaliseAndStaysPaused)
{
  Harness h;
  h.poor_evaluation();
  EXPECT_EQ(h.guard->state(), mg::State::RELOCALISING);
  ASSERT_EQ(h.relocalise_requests.size(), 1u);
  EXPECT_EQ(h.relocalise_requests[0].first, "map_guard-1");
  EXPECT_LE(h.relocalise_requests[0].second, 1791633600.0);
  EXPECT_TRUE(h.slam.paused);
  EXPECT_FALSE(has_serialize(h.slam));
  EXPECT_FALSE(h.slam.called("reset"));
  EXPECT_EQ(h.resets_published, 0);
  h.run(5.0, {}, BAD);  // still waiting: no second request, still paused
  EXPECT_EQ(h.relocalise_requests.size(), 1u);
  EXPECT_EQ(h.guard->state(), mg::State::RELOCALISING);
  EXPECT_EQ(h.state_gauge(mg::State::RELOCALISING), 1.0);
}

TEST(GuardRelocalise, LocalisedResultResumesWithoutDeleting)
{
  Harness h;
  h.poor_evaluation();
  h.answer(true, "localised");
  EXPECT_EQ(h.guard->state(), mg::State::RESUMED);
  h.run(0.2, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_FALSE(h.slam.paused);
  EXPECT_FALSE(h.slam.called("reset"));
  EXPECT_FALSE(has_serialize(h.slam));
  EXPECT_EQ(h.resets_published, 0);
  EXPECT_EQ(h.counter("map_guard_resumes_total"), 1.0);
  EXPECT_EQ(h.notifier.events.back().kind, mg::EVENT_RESUMED);
  EXPECT_EQ(h.notifier.events.back().reason, mg::REASON_CARRIED);
}

TEST(GuardRelocalise, FailedResultBacksUpAndResetsOnceWithDiscardedPois)
{
  Harness h;
  h.poor_evaluation();
  h.answer(false, "not_localised", 7);
  h.run(0.5, {}, BAD);
  const std::string expected = (h.dir / "map_guard_backup_20261010T120000Z").string();
  EXPECT_EQ(h.slam.count("serialize " + expected), 1);
  EXPECT_EQ(h.slam.count("reset"), 1);
  EXPECT_EQ(h.resets_published, 1);
  EXPECT_FALSE(h.slam.paused);
  const auto & last = h.notifier.events.back();
  EXPECT_EQ(last.kind, mg::EVENT_RESET);
  EXPECT_EQ(last.reason, mg::REASON_CARRIED);
  EXPECT_EQ(last.data.at("discarded_pois"), 7);
  EXPECT_EQ(last.data.at("backup"), expected);
  h.answer(false, "not_localised", 7);  // late duplicate
  EXPECT_EQ(h.slam.count("reset"), 1);
}

TEST(GuardRelocalise, OkResultWithOtherStateIsUnrecoverable)
{
  Harness h;
  h.poor_evaluation();
  h.answer(true, "ambiguous");
  EXPECT_EQ(h.slam.count("reset"), 1);
}

TEST(GuardRelocalise, TimeoutCountsAsUnrecoverable)
{
  Harness h([](mg::Config & c) { c.relocalise_timeout_s = 10.0; });
  h.poor_evaluation();
  h.run(9.0, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::RELOCALISING);
  EXPECT_FALSE(h.slam.called("reset"));
  h.run(1.5, {}, BAD);
  EXPECT_EQ(h.slam.count("reset"), 1);
  EXPECT_EQ(h.resets_published, 1);
  const auto & last = h.notifier.events.back();
  EXPECT_EQ(last.kind, mg::EVENT_RESET);
  EXPECT_EQ(last.data.at("discarded_pois"), 0);
}

TEST(GuardRelocalise, ForeignRequestIdIsIgnored)
{
  Harness h;
  h.poor_evaluation();
  h.guard->on_relocalise_result("someone-else-1", true, "localised", 0);
  h.guard->on_relocalise_result("someone-else-2", false, "not_localised", 3);
  EXPECT_EQ(h.guard->state(), mg::State::RELOCALISING);
  EXPECT_FALSE(h.slam.called("reset"));
  EXPECT_EQ(h.resets_published, 0);
}

TEST(GuardRelocalise, ResultWithoutPendingRequestIsIgnored)
{
  Harness h;
  h.guard->on_relocalise_result("map_guard-1", false, "not_localised", 0);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_TRUE(h.slam.calls.empty());
}

TEST(GuardRelocalise, HealthyMedianResumesWithoutRelocalise)
{
  Harness h;
  h.run(0.1, CARRIED);
  h.run(8.0, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_TRUE(h.relocalise_requests.empty());
  EXPECT_EQ(h.notifier.events.back().kind, mg::EVENT_RESUMED);
}

TEST(GuardRelocalise, NewRequestIdPerEpisode)
{
  Harness h([](mg::Config & c) { c.cooldown_s = 1.0; });
  h.poor_evaluation();
  h.fail_relocalise();
  h.run(2.0, {}, HEALTHY);
  h.poor_evaluation();
  ASSERT_EQ(h.relocalise_requests.size(), 2u);
  EXPECT_EQ(h.relocalise_requests[0].first, "map_guard-1");
  EXPECT_EQ(h.relocalise_requests[1].first, "map_guard-2");
}

TEST(GuardRelocalise, HoldDuringOwnRelocaliseKeepsThePendingRequest)
{
  Harness h;
  h.poor_evaluation();
  h.guard->set_hold(true);  // map_manager reports not localised while it relocalises
  h.guard->set_hold(false);
  EXPECT_EQ(h.guard->state(), mg::State::RELOCALISING);
  h.guard->set_hold(true);
  h.answer(true, "localised");  // the result still resolves the request while held
  EXPECT_EQ(h.guard->state(), mg::State::RESUMED);
  EXPECT_FALSE(h.slam.called("reset"));
  EXPECT_EQ(h.relocalise_requests.size(), 1u);
}

TEST(GuardRelocalise, TimeoutStillFiresWhileHeld)
{
  Harness h([](mg::Config & c) { c.relocalise_timeout_s = 10.0; });
  h.poor_evaluation();
  h.guard->set_hold(true);
  h.run(11.0, {}, BAD);
  EXPECT_EQ(h.slam.count("reset"), 1);
}

// --- Dry run --------------------------------------------------------------------------------------------------------

TEST(GuardDryRun, NeverCallsServicesOrPublishes)
{
  Harness h([](mg::Config & c) { c.dry_run = true; });
  h.run(0.1, CARRIED);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
  h.run(8.0, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_TRUE(h.slam.calls.empty());
  EXPECT_EQ(h.resets_published, 0);
  EXPECT_TRUE(fs::is_empty(h.dir));
  EXPECT_TRUE(h.relocalise_requests.empty());
  EXPECT_EQ(h.counter("map_guard_resets_total", {{"dry_run", "true"}}), 0.0);
  EXPECT_EQ(h.counter("map_guard_pauses_total"), 1.0);
  ASSERT_EQ(h.notifier.events.size(), 2u);
  EXPECT_EQ(h.notifier.events[0].kind, mg::EVENT_DISTURBED);
  EXPECT_EQ(h.notifier.events[1].kind, mg::EVENT_RESUMED);
  EXPECT_TRUE(h.logged([](const std::string & l) { return contains(l, "would request relocalise"); }));
  EXPECT_TRUE(h.logged([](const std::string & l) { return contains(l, "dry run") && contains(l, "carried"); }));
  EXPECT_TRUE(h.logged([](const std::string & l) { return contains(l, "ratio"); }));
}

TEST(GuardDryRun, DryRunGauge)
{
  Harness dry([](mg::Config & c) { c.dry_run = true; });
  EXPECT_EQ(dry.counter("map_guard_dry_run"), 1.0);
  Harness wet;
  EXPECT_EQ(wet.counter("map_guard_dry_run"), 0.0);
}

TEST(GuardMetrics, ReportNumbersReachTheGauges)
{
  Harness h;
  EXPECT_FALSE(has_family(h.metrics, "map_guard_agreement_ratio"));  // no placeholder before the first sample
  EXPECT_FALSE(has_family(h.metrics, "map_guard_tilt_deg"));
  h.guard->tick(0.0, mg::DisturbanceReport{{}, 3.5, 0.04, 0.0}, {{0.77, 5000}});
  EXPECT_NEAR(h.counter("map_guard_tilt_deg"), 3.5, 1e-12);
  EXPECT_NEAR(h.counter("map_guard_unexplained_speed_mps"), 0.04, 1e-12);
  EXPECT_NEAR(h.counter("map_guard_agreement_ratio"), 0.77, 1e-12);
}

TEST(GuardMetrics, NodeInfoAndAllStatesAreExported)
{
  Harness h;
  EXPECT_EQ(h.counter("robot_node_info", {{"node", "map_guard_cpp"}}), 1.0);
  for (auto state : mg::ALL_STATES) {
    EXPECT_TRUE(has_family(h.metrics, "map_guard_state"));
    EXPECT_EQ(h.state_gauge(state), state == mg::State::OK ? 1.0 : 0.0) << mg::state_name(state);
  }
}

TEST(GuardFormat, FloatReprAndFmtMatchPython)
{
  EXPECT_EQ(mg::float_repr(0.6), "0.6");
  EXPECT_EQ(mg::float_repr(1.0), "1.0");
  EXPECT_EQ(mg::float_repr(1e-05), "1e-05");
  EXPECT_EQ(mg::float_repr(0.30000000000000004), "0.30000000000000004");
  EXPECT_EQ(mg::float_repr(0.0001), "0.0001");
  EXPECT_EQ(mg::float_repr(1e16), "1e+16");
  EXPECT_EQ(mg::float_repr(123.0), "123.0");
  EXPECT_EQ(mg::fmt(std::nullopt), "n/a");
  EXPECT_EQ(mg::fmt(0.125), "0.12");
  EXPECT_EQ(mg::fmt(0.9), "0.90");
}

// --- Golden fixture (Python outputs) --------------------------------------------------------------------------------

namespace
{

std::string untoken(std::string text, const std::string & token, const std::string & dir)
{
  for (auto pos = text.find(token); pos != std::string::npos; pos = text.find(token, pos + dir.size())) {
    text.replace(pos, token.size(), dir);
  }
  return text;
}

}  // namespace

TEST(GuardFixture, ScenariosMatchPython)
{
  const auto f = fx::load_fixture("guard.json");
  const std::string token = f["backup_dir_token"];
  ASSERT_EQ(f["fixed_utc_epoch_s"].get<double>(), 1791633600.0);
  for (const auto & s : f["scenarios"]) {
    const std::string name = s["name"];
    const nlohmann::json overrides = s.value("overrides", nlohmann::json::object());
    const nlohmann::json slam_spec = s.value("slam", nlohmann::json::object());
    Harness h(
      [&overrides](mg::Config & c) {
        if (overrides.contains("dry_run")) {
          c.dry_run = overrides["dry_run"];
        }
        if (overrides.contains("keep_backups")) {
          c.keep_backups = overrides["keep_backups"];
        }
        if (overrides.contains("relocalise_timeout_s")) {
          c.relocalise_timeout_s = overrides["relocalise_timeout_s"];
        }
        if (overrides.contains("cooldown_s")) {
          c.cooldown_s = overrides["cooldown_s"];
        }
        if (overrides.contains("reasons")) {
          c.reasons = overrides["reasons"].get<std::vector<std::string>>();
        }
      },
      [&slam_spec](FakeSlam & slam) {
        slam.paused = slam_spec.value("paused", false);
        slam.toggle_ok = slam_spec.value("toggle_ok", true);
        slam.toggle_works = slam_spec.value("toggle_works", true);
        slam.serialize_ok = slam_spec.value("serialize_ok", true);
        slam.reset_ok = slam_spec.value("reset_ok", true);
        slam.query_none = slam_spec.value("query_none", false);
      });
    const std::string dir = h.dir.string();
    for (const auto & stem : s.value("existing_backups", std::vector<std::string>{})) {
      std::ofstream(h.dir / (stem + ".posegraph")) << "g";
      std::ofstream(h.dir / (stem + ".data")) << "d";
    }
    std::vector<std::pair<std::string, int>> states;
    for (const auto & step : s["steps"]) {
      if (step.contains("answer")) {
        const auto & a = step["answer"];
        const std::string which = a.value("request", "last");
        std::string request_id = which;
        if (which == "last" || which == "first") {
          const auto & requests = h.relocalise_requests;
          request_id = requests.empty() ? "" : (which == "last" ? requests.back().first : requests.front().first);
        }
        h.guard->on_relocalise_result(
          request_id, a["ok"].get<bool>(), a["state"].get<std::string>(), a["discarded_pois"].get<int>());
      }
      std::set<std::string> reasons;
      for (const auto & r : step.value("reasons", nlohmann::json::array())) {
        reasons.insert(r.get<std::string>());
      }
      std::optional<mg::MapGuard::Sample> sample;
      if (step.contains("sample")) {
        sample = mg::MapGuard::Sample{step["sample"][0].get<double>(), step["sample"][1].get<long>()};  // NOLINT
      }
      const int ticks = static_cast<int>(std::round(step["seconds"].get<double>() / TICK));
      for (int i = 0; i < ticks; ++i) {
        h.run(TICK, reasons, sample);
        const std::string state = mg::state_name(h.guard->state());
        if (!states.empty() && states.back().first == state) {
          ++states.back().second;
        } else {
          states.emplace_back(state, 1);
        }
      }
    }
    std::vector<std::pair<std::string, int>> expected_states;
    for (const auto & run : s["states"]) {
      expected_states.emplace_back(run[0].get<std::string>(), run[1].get<int>());
    }
    EXPECT_EQ(states, expected_states) << name;

    std::vector<std::string> expected_calls;
    for (const auto & call : s["slam_calls"]) {
      expected_calls.push_back(
        call.is_string() ? call.get<std::string>()
                         : call[0].get<std::string>() + " " + untoken(call[1].get<std::string>(), token, dir));
    }
    EXPECT_EQ(h.slam.calls, expected_calls) << name;

    ASSERT_EQ(h.notifier.events.size(), s["events"].size()) << name;
    for (std::size_t i = 0; i < h.notifier.events.size(); ++i) {
      const auto & e = s["events"][i];
      const auto & got = h.notifier.events[i];
      EXPECT_EQ(got.kind, e[0].get<std::string>()) << name;
      EXPECT_EQ(got.reason, e[1].get<std::string>()) << name;
      if (e[2].is_null()) {
        EXPECT_FALSE(got.ratio) << name << " event " << i;
      } else {
        ASSERT_TRUE(got.ratio) << name << " event " << i;
        EXPECT_DOUBLE_EQ(*got.ratio, e[2].get<double>()) << name << " event " << i;
      }
      mg::EventData data;
      for (const auto & [k, v] : e[3].items()) {
        data[k] = v.is_string() ? nlohmann::json(untoken(v.get<std::string>(), token, dir)) : v;
      }
      EXPECT_EQ(got.data, data) << name;
    }

    std::vector<std::string> expected_logs;
    for (const auto & line : s["logs"]) {
      expected_logs.push_back(untoken(line.get<std::string>(), token, dir));
    }
    EXPECT_EQ(h.logs, expected_logs) << name;

    EXPECT_EQ(h.resets_published, s["resets_published"].get<int>()) << name;
    ASSERT_EQ(h.relocalise_requests.size(), s["relocalise_requests"].size()) << name;
    for (std::size_t i = 0; i < h.relocalise_requests.size(); ++i) {
      EXPECT_EQ(h.relocalise_requests[i].first, s["relocalise_requests"][i][0].get<std::string>()) << name;
      EXPECT_NEAR(h.relocalise_requests[i].second, s["relocalise_requests"][i][1].get<double>(), 1e-6) << name;
    }
    if (s["paused"].is_null()) {
      EXPECT_FALSE(h.guard->paused()) << name;
    } else {
      EXPECT_EQ(h.guard->paused(), std::optional<bool>(s["paused"].get<bool>())) << name;
    }
    EXPECT_EQ(h.slam.paused, s["slam_paused"].get<bool>()) << name;
    for (const auto & c : s["counters"]) {
      Labels labels = c["labels"].get<Labels>();
      EXPECT_EQ(h.counter(c["name"], labels), c["delta"].get<double>())
        << name << " " << c["name"] << " " << c["labels"].dump();
    }
    for (const auto & [state, value] : s["state_gauge"].items()) {
      EXPECT_EQ(h.counter("map_guard_state", {{"state", state}}), value.get<double>()) << name << " " << state;
    }
    EXPECT_EQ(h.counter("map_guard_dry_run"), s["dry_run_gauge"].get<double>()) << name;
    std::set<std::string> stems;
    for (const auto & entry : fs::directory_iterator(h.dir)) {
      const std::string file = entry.path().filename().string();
      stems.insert(file.substr(0, file.find('.')));
    }
    const auto expected_stems = s["backup_stems"].get<std::vector<std::string>>();
    EXPECT_EQ(stems, std::set<std::string>(expected_stems.begin(), expected_stems.end())) << name;
  }
  const auto & gauges = f["report_numbers_reach_the_gauges"];
  Harness h;
  h.guard->tick(0.0, mg::DisturbanceReport{{}, 3.5, 0.04, 0.0}, {{0.77, 5000}});
  EXPECT_DOUBLE_EQ(h.counter("map_guard_tilt_deg"), gauges["tilt_deg"].get<double>());
  EXPECT_DOUBLE_EQ(h.counter("map_guard_unexplained_speed_mps"), gauges["unexplained_speed_mps"].get<double>());
  EXPECT_DOUBLE_EQ(h.counter("map_guard_agreement_ratio"), gauges["agreement_ratio"].get<double>());
}
