// The runtime phase-drift policy (PhaseDriftMonitor) on a fake clock: when it asks StereoCapture to re-run the
// right-camera phase alignment, and when it must not.
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "depthd/config.hpp"
#include "depthd/pairing.hpp"
#include "depthd/phase_monitor.hpp"

using depthd::PhaseDriftMonitor;
using depthd::PhaseDriftOptions;
using depthd::RealignReason;

namespace
{
constexpr int64_t kMs = 1000000;
constexpr int64_t kS = 1000000000;
constexpr int64_t kFrame = 33333333;  // 30 fps

PhaseDriftOptions options()
{
  PhaseDriftOptions o;
  o.skew_threshold_ms = 4.0;
  o.window_ns = 10 * kS;
  o.stall_ns = 5 * kS;
  o.cooldown_ns = 30 * kS;
  return o;
}

// Feeds one judged candidate and one emitted pair per frame from `from` to `to` (fake clock), polling every frame.
// Returns the first non-none decision and the time it was made (or none).
struct Decision
{
  RealignReason reason = RealignReason::none;
  int64_t at_ns = 0;
};

Decision run(PhaseDriftMonitor & m, int64_t from, int64_t to, double skew_ms, bool pairs = true)
{
  for (int64_t t = from; t < to; t += kFrame) {
    m.observe_skew(t, skew_ms);
    if (pairs) m.observe_pair(t);
    const auto d = m.poll(t);
    if (d.reason != RealignReason::none) return {d.reason, t};
  }
  return {};
}
}  // namespace

TEST(PhaseDriftMonitor, AlignedCamerasNeverRealign)
{
  PhaseDriftMonitor m(options());
  m.arm(0);
  EXPECT_EQ(run(m, 0, 120 * kS, 1.5).reason, RealignReason::none);
  EXPECT_NEAR(m.median_skew_ms(120 * kS), 1.5, 1e-9);
}

TEST(PhaseDriftMonitor, SustainedSkewAboveThresholdRealignsAfterTheWindow)
{
  PhaseDriftMonitor m(options());
  m.arm(0);
  const Decision d = run(m, 0, 60 * kS, 12.0);
  EXPECT_EQ(d.reason, RealignReason::skew);
  // Never before a full window of evidence, and promptly once it is there.
  EXPECT_GE(d.at_ns, 10 * kS);
  EXPECT_LT(d.at_ns, 11 * kS);
}

TEST(PhaseDriftMonitor, RollingMedianIgnoresAMinorityOfOutliers)
{
  PhaseDriftMonitor m(options());
  m.arm(0);
  for (int64_t t = 0, i = 0; t < 60 * kS; t += kFrame, ++i) {
    m.observe_skew(t, i % 3 == 0 ? 30.0 : 2.0);  // a third of the candidates far off: the median stays at 2 ms
    m.observe_pair(t);
    ASSERT_EQ(m.poll(t).reason, RealignReason::none) << "at " << t / kMs << " ms";
  }
}

TEST(PhaseDriftMonitor, DriftThatGrowsOverMinutesIsCaught)
{
  // The live failure: aligned at 1.5 ms after startup, then the phase walks away by ~0.1 ms/s.
  PhaseDriftMonitor m(options());
  m.arm(0);
  Decision d;
  for (int64_t t = 0; t < 600 * kS && d.reason == RealignReason::none; t += kFrame) {
    const double skew = 1.5 + 0.1 * static_cast<double>(t) / kS;
    m.observe_skew(t, skew);
    m.observe_pair(t);
    const auto p = m.poll(t);
    if (p.reason != RealignReason::none) d = {p.reason, t};
  }
  ASSERT_EQ(d.reason, RealignReason::skew);
  // Median of the trailing 10 s crosses 4 ms when the newest skew is ~4.5 ms: t ~ 30 s, well before 8 ms drops start.
  EXPECT_LT(d.at_ns, 40 * kS);
}

TEST(PhaseDriftMonitor, NoPairForTheStallTimeRealigns)
{
  PhaseDriftMonitor m(options());
  m.arm(0);
  ASSERT_EQ(run(m, 0, 2 * kS, 1.0).reason, RealignReason::none);
  // Frames keep arriving (candidates judged) but nothing pairs.
  const Decision d = run(m, 2 * kS, 20 * kS, 1.0, false);
  EXPECT_EQ(d.reason, RealignReason::stall);
  // The last pair came at ~1.97 s: the stall is declared once 5 s have passed since then.
  EXPECT_GT(d.at_ns, 6 * kS + 900 * kMs);
  EXPECT_LT(d.at_ns, 7 * kS + 100 * kMs);
}

TEST(PhaseDriftMonitor, StallCountsFromArmingWhenNoPairWasEverSeen)
{
  PhaseDriftMonitor m(options());
  m.arm(100 * kS);
  EXPECT_EQ(m.poll(104 * kS).reason, RealignReason::none);
  EXPECT_EQ(m.poll(106 * kS).reason, RealignReason::stall);
}

TEST(PhaseDriftMonitor, DisarmedMonitorNeverDecides)
{
  // While aligning or while a camera is in error nothing is judged.
  PhaseDriftMonitor m(options());
  EXPECT_FALSE(m.armed());
  EXPECT_EQ(run(m, 0, 60 * kS, 20.0, false).reason, RealignReason::none);
  m.arm(60 * kS);
  EXPECT_TRUE(m.armed());
  m.disarm();
  EXPECT_EQ(run(m, 60 * kS, 120 * kS, 20.0, false).reason, RealignReason::none);
}

TEST(PhaseDriftMonitor, ADecisionDisarmsAndArmingStartsAFreshWindow)
{
  PhaseDriftMonitor m(options());
  m.arm(0);
  const Decision first = run(m, 0, 60 * kS, 12.0);
  ASSERT_EQ(first.reason, RealignReason::skew);
  EXPECT_FALSE(m.armed());
  // Re-aligned: the old high-skew samples must not count against the new phase.
  m.arm(first.at_ns + kS);
  EXPECT_EQ(run(m, first.at_ns + kS, first.at_ns + 120 * kS, 1.0).reason, RealignReason::none);
}

TEST(PhaseDriftMonitor, CooldownSpacesRealignments)
{
  PhaseDriftMonitor m(options());
  m.arm(0);
  const Decision first = run(m, 0, 60 * kS, 12.0);
  ASSERT_EQ(first.reason, RealignReason::skew);
  // Re-alignment did not help: the skew stays high. The next attempt waits for the 30 s cooldown.
  m.arm(first.at_ns);
  const Decision second = run(m, first.at_ns, first.at_ns + 120 * kS, 12.0);
  ASSERT_EQ(second.reason, RealignReason::skew);
  EXPECT_GE(second.at_ns - first.at_ns, 30 * kS);
  // The cooldown also holds back a stall.
  m.arm(second.at_ns);
  EXPECT_EQ(m.poll(second.at_ns + 10 * kS).reason, RealignReason::none);
  EXPECT_EQ(m.poll(second.at_ns + 31 * kS).reason, RealignReason::stall);
}

TEST(PhaseDriftMonitor, DecisionCarriesTheEvidenceForTheLog)
{
  PhaseDriftMonitor m(options());
  m.arm(0);
  PhaseDriftMonitor::Decision d;
  for (int64_t t = 0; t < 60 * kS && d.reason == RealignReason::none; t += kFrame) {
    m.observe_skew(t, 12.0);
    m.observe_pair(t);
    d = m.poll(t);
  }
  ASSERT_EQ(d.reason, RealignReason::skew);
  EXPECT_NEAR(d.median_skew_ms, 12.0, 1e-9);
  EXPECT_LT(d.since_pair_ns, kFrame);
  EXPECT_STREQ(depthd::to_string(RealignReason::skew), "skew");
  EXPECT_STREQ(depthd::to_string(RealignReason::stall), "stall");
}

TEST(PhaseDriftMonitor, OptionsComeFromTheCaptureConfig)
{
  depthd::Capture c;
  c.realign_skew_ms = 3.0;
  c.realign_window_s = 7.5;
  c.realign_stall_s = 2.0;
  c.realign_cooldown_s = 45.0;
  const auto o = PhaseDriftOptions::from_config(c);
  EXPECT_DOUBLE_EQ(o.skew_threshold_ms, 3.0);
  EXPECT_EQ(o.window_ns, 7500 * kMs);
  EXPECT_EQ(o.stall_ns, 2 * kS);
  EXPECT_EQ(o.cooldown_ns, 45 * kS);
}

TEST(PairingSkewObserver, SeesEveryJudgedCandidate)
{
  depthd::StereoPairer p(8.0);
  std::vector<double> seen;
  p.set_skew_observer([&](double ms) { seen.push_back(ms); });
  depthd::Frame l;
  depthd::Frame r;
  l.capture_realtime_ns = 0;
  r.capture_realtime_ns = 2 * kMs;
  p.push_left(l);
  p.push_right(r);  // pairs, 2 ms
  l.capture_realtime_ns = kFrame;
  r.capture_realtime_ns = kFrame + 12 * kMs;
  p.push_left(l);
  p.push_right(r);  // dropped for skew, 12 ms
  ASSERT_EQ(seen.size(), 2u);
  EXPECT_NEAR(seen[0], 2.0, 1e-9);
  EXPECT_NEAR(seen[1], 12.0, 1e-9);
}
