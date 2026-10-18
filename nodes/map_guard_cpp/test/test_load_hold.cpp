// The localisation hold of the map guard (design D9): while map_manager reports a loaded map as not localised the
// guard does nothing, and on release the mismatch monitor is re-armed only once the new map is healthy. Plus the
// localised_topic setting.
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "map_guard_cpp/guard.hpp"
#include "ros2_common_cpp/metrics.hpp"

namespace mg = map_guard_cpp;

namespace
{

constexpr double TICK = 0.1;
const mg::MapGuard::Sample HEALTHY = {0.9, 5000};
const mg::MapGuard::Sample BAD = {0.1, 5000};
const std::set<std::string> LIFTED = {mg::REASON_LIFTED};

class FakeSlam : public mg::SlamPort
{
public:
  bool paused = false;
  std::vector<std::string> calls;

  std::optional<bool> query_paused() override
  {
    calls.push_back("query");
    return paused;
  }
  bool toggle_pause() override
  {
    calls.push_back("toggle");
    paused = !paused;
    return true;
  }
  bool serialize_map(const std::string & filename) override
  {
    calls.push_back("serialize " + filename);
    return true;
  }
  bool reset() override
  {
    calls.push_back("reset");
    return true;
  }
};

class FakeNotifier : public mg::Notifier
{
public:
  int events = 0;
  void notify(
    double, const std::string &, const std::string &, const std::optional<double> &, const mg::EventData &) override
  {
    ++events;
  }
};

class Harness
{
public:
  Harness()
  {
    config.dry_run = false;
    config.backup_dir = "/tmp/map_guard_cpp_load_hold_unused";
    guard = std::make_unique<mg::MapGuard>(
      config, slam, notifier, [this]() { ++resets_published; },
      [this](const std::string & line) { logs.push_back(line); }, guard_metrics,
      []() { return std::chrono::system_clock::time_point(std::chrono::seconds(1791633600)); });
  }

  // Tick at 10 Hz for seconds; an agreement sample every 0.5 s when given.
  void run(
    double seconds, const std::set<std::string> & reasons = {},
    const std::optional<mg::MapGuard::Sample> & sample = std::nullopt)
  {
    const int ticks = static_cast<int>(std::round(seconds / TICK));
    for (int i = 0; i < ticks; ++i) {
      mg::DisturbanceReport report{reasons, std::nullopt, std::nullopt, std::nullopt};
      std::vector<mg::MapGuard::Sample> samples;
      if (sample && i % 5 == 0) {
        samples.push_back(*sample);
      }
      guard->tick(now, report, samples);
      now += TICK;
    }
  }

  long count_logs(const std::string & text) const  // NOLINT(runtime/int)
  {
    return std::count(logs.begin(), logs.end(), text);
  }

  mg::Config config;
  FakeSlam slam;
  FakeNotifier notifier;
  int resets_published = 0;
  std::vector<std::string> logs;
  ros2_common_cpp::Metrics metrics{"map_guard_cpp_load_hold"};
  mg::GuardMetrics guard_metrics{metrics};
  std::unique_ptr<mg::MapGuard> guard;
  double now = 0.0;
};

const char * HOLD_LOG = "map_guard: holding while the loaded map is not localised";
const char * RELEASE_LOG = "map_guard: loaded map localised, guard re-armed";

}  // namespace

TEST(LoadHold, HeldGuardIgnoresLiftedAndMismatch)
{
  Harness h;
  h.run(1.0, {}, HEALTHY);
  h.guard->set_hold(true);
  h.run(2.0, LIFTED, BAD);
  h.run(10.0, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_EQ(h.guard->paused(), std::nullopt);
  EXPECT_TRUE(h.slam.calls.empty());
  EXPECT_EQ(h.resets_published, 0);
  EXPECT_EQ(h.notifier.events, 0);
  EXPECT_EQ(h.count_logs(HOLD_LOG), 1);
}

TEST(LoadHold, HoldIsLoggedOnceEvenWhenRepeated)
{
  Harness h;
  h.guard->set_hold(true);
  h.guard->set_hold(true);
  h.guard->set_hold(false);
  h.guard->set_hold(false);
  EXPECT_EQ(h.count_logs(HOLD_LOG), 1);
  EXPECT_EQ(h.count_logs(RELEASE_LOG), 1);
}

TEST(LoadHold, ReleaseRearmsTheMismatchMonitor)
{
  Harness h;
  h.run(5.0, {}, HEALTHY);  // armed on the old map
  h.guard->set_hold(true);
  h.guard->set_hold(false);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  // Bad samples straight after the release do not trigger: the monitor is disarmed until the map is healthy.
  h.run(12.0, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_TRUE(h.slam.calls.empty());
  // Healthy samples re-arm it, then a sustained mismatch triggers again.
  h.run(5.0, {}, HEALTHY);
  h.run(7.5, {}, BAD);
  EXPECT_EQ(h.guard->state(), mg::State::EVALUATING);
  EXPECT_TRUE(h.slam.paused);
}

TEST(LoadHold, ReleaseReturnsAnEpisodeInProgressToOk)
{
  Harness h;
  h.run(1.0, {}, HEALTHY);
  h.run(0.5, LIFTED, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
  h.guard->set_hold(true);
  h.guard->set_hold(false);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
}

TEST(LoadHold, NoMessageBehavesAsBefore)
{
  Harness h;  // set_hold never called: the lifted scenario of test_guard.cpp
  h.run(1.0, {}, HEALTHY);
  h.run(0.1, LIFTED, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::DISTURBED);
  EXPECT_TRUE(h.slam.paused);
  EXPECT_EQ(h.slam.calls, (std::vector<std::string>{"query", "toggle", "query"}));
  h.run(2.1, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::SETTLING);
  h.run(3.4, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::EVALUATING);
  h.run(4.3, {}, HEALTHY);
  EXPECT_EQ(h.guard->state(), mg::State::OK);
  EXPECT_FALSE(h.slam.paused);
  EXPECT_EQ(h.resets_published, 0);
  EXPECT_EQ(h.count_logs(HOLD_LOG), 0);
}

TEST(LoadHoldConfig, DefaultAndExplicitValue)
{
  EXPECT_EQ(mg::Config().localised_topic, "/map_manager/localised");
  ros2_common_cpp::Errors errors;
  const auto config = mg::parse_config(YAML::Load("localised_topic: /x/loc\n"), errors);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(config.localised_topic, "/x/loc");
  EXPECT_TRUE(mg::validate_config(YAML::Load("dry_run: false\n")).empty());
  EXPECT_FALSE(mg::validate_config(YAML::Load("localised_topic: [a]\n")).empty());
  const auto & keys = mg::config_keys();
  EXPECT_NE(std::find(keys.begin(), keys.end(), "localised_topic"), keys.end());
}
