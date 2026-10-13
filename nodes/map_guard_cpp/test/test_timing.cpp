// Port of nodes/map_guard/tests/test_timing.py (HoldTimer, RateLimiter) plus the timing.json golden fixture.
#include <gtest/gtest.h>

#include "fixture_util.hpp"
#include "map_guard_cpp/timing.hpp"

using map_guard_cpp::HoldTimer;
using map_guard_cpp::RateLimiter;

TEST(Timing, HoldTimerNeedsTheFullDuration)
{
  HoldTimer timer(1.0);
  EXPECT_FALSE(timer.update(0.0, true));
  EXPECT_FALSE(timer.update(0.5, true));
  EXPECT_FALSE(timer.update(0.99, true));
  EXPECT_TRUE(timer.update(1.0, true));
  EXPECT_TRUE(timer.update(3.0, true));
}

TEST(Timing, HoldTimerResetsOnAFalseSample)
{
  HoldTimer timer(1.0);
  timer.update(0.0, true);
  timer.update(0.8, false);
  EXPECT_FALSE(timer.update(1.2, true));
  EXPECT_TRUE(timer.update(2.2, true));
}

TEST(Timing, ShortSpikeNeverFires)
{
  HoldTimer timer(0.4);
  for (int t = 0; t < 40; ++t) {
    const double now = t / 10.0;
    EXPECT_FALSE(timer.update(now, 1.0 <= now && now < 1.2));
  }
}

TEST(Timing, HoldTimerReset)
{
  HoldTimer timer(0.5);
  timer.update(0.0, true);
  timer.reset();
  EXPECT_FALSE(timer.update(0.6, true));
}

TEST(Timing, RateLimiter)
{
  RateLimiter limiter(2.0);
  EXPECT_TRUE(limiter.ready(0.0));
  EXPECT_FALSE(limiter.ready(0.3));
  EXPECT_TRUE(limiter.ready(0.5));
  EXPECT_FALSE(limiter.ready(0.9));
  EXPECT_TRUE(limiter.ready(1.01));
}

TEST(TimingFixture, HoldSequencesMatchPython)
{
  const auto fx = map_guard_cpp::test::load_fixture("timing.json");
  for (const auto & c : fx["hold"]) {
    HoldTimer timer(c["duration_s"].get<double>());
    const auto & seq = c["sequence"];
    for (std::size_t i = 0; i < seq.size(); ++i) {
      if (seq[i].is_string()) {
        timer.reset();
        continue;
      }
      EXPECT_EQ(timer.update(seq[i][0].get<double>(), seq[i][1].get<bool>()), c["out"][i].get<bool>())
        << c["name"] << " step " << i;
    }
  }
}

TEST(TimingFixture, RateLimiterMatchesPython)
{
  const auto fx = map_guard_cpp::test::load_fixture("timing.json");
  for (const auto & c : fx["rate_limiter"]) {
    RateLimiter limiter(c["rate_hz"].get<double>());
    for (std::size_t i = 0; i < c["times"].size(); ++i) {
      EXPECT_EQ(limiter.ready(c["times"][i].get<double>()), c["out"][i].get<bool>()) << "rate " << c["rate_hz"];
    }
  }
}
