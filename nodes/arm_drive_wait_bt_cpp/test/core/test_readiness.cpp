// readiness.hpp without ROS: the state JSON parse, the waiting states, receipt freshness and the timeout fallback.
#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <string>

#include "arm_drive_wait_bt_cpp/readiness.hpp"

namespace
{

using arm_drive_wait_bt_cpp::DrivePoseState;
using arm_drive_wait_bt_cpp::evaluate;
using arm_drive_wait_bt_cpp::parse_state;
using arm_drive_wait_bt_cpp::state_waits;

constexpr int64_t START_NS = 1700000000000000000;
constexpr double TIMEOUT_S = 8.0;
constexpr double FRESH_AGE_S = 0.5;
constexpr double STALE_AGE_S = 2.6;

std::string state_json(const std::string & state, int64_t stamp_ns)
{
  return "{\"state\":\"" + state + "\",\"reason\":\"\",\"enabled\":true,\"stamp_ns\":" + std::to_string(stamp_ns) + "}";
}

DrivePoseState fresh(const std::string & state, bool enabled = true) { return DrivePoseState{state, "", enabled, 1}; }

}  // namespace

TEST(ParseState, ReadsEveryField)
{
  const auto parsed =
    parse_state("{\"state\":\"skipped: torque off\",\"reason\":\"torque off\",\"enabled\":false,\"stamp_ns\":42}");
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->state, "skipped: torque off");
  EXPECT_EQ(parsed->reason, "torque off");
  EXPECT_FALSE(parsed->enabled);
  EXPECT_EQ(parsed->stamp_ns, 42);
}

TEST(ParseState, ReadsAWallClockStamp)
{
  const auto parsed = parse_state(state_json("lifted", START_NS));
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->stamp_ns, START_NS);
}

TEST(ParseState, MalformedJsonIsNullopt)
{
  EXPECT_FALSE(parse_state("").has_value());
  EXPECT_FALSE(parse_state("not json").has_value());
  EXPECT_FALSE(parse_state("{\"state\":\"lifted\"").has_value());
  EXPECT_FALSE(parse_state("[1, 2]").has_value());
}

TEST(ParseState, MissingOrMistypedFieldIsNullopt)
{
  EXPECT_FALSE(parse_state("{\"reason\":\"\",\"enabled\":true,\"stamp_ns\":1}").has_value());
  EXPECT_FALSE(parse_state("{\"state\":\"lifted\",\"reason\":\"\",\"enabled\":true}").has_value());
  EXPECT_FALSE(parse_state("{\"state\":\"lifted\",\"enabled\":true,\"stamp_ns\":1}").has_value());
  EXPECT_FALSE(parse_state("{\"state\":\"lifted\",\"reason\":\"\",\"stamp_ns\":1}").has_value());
  EXPECT_FALSE(parse_state("{\"state\":1,\"reason\":\"\",\"enabled\":true,\"stamp_ns\":1}").has_value());
  EXPECT_FALSE(parse_state("{\"state\":\"lifted\",\"reason\":\"\",\"enabled\":\"yes\",\"stamp_ns\":1}").has_value());
  EXPECT_FALSE(parse_state("{\"state\":\"lifted\",\"reason\":\"\",\"enabled\":true,\"stamp_ns\":\"1\"}").has_value());
}

TEST(StateWaits, LiftingLoweringAndEnabledIdleWait)
{
  EXPECT_TRUE(state_waits(fresh("lifting")));
  EXPECT_TRUE(state_waits(fresh("lowering")));
  EXPECT_TRUE(state_waits(fresh("idle", true)));
}

TEST(StateWaits, LiftedSkippedDisabledAndDisabledIdleGo)
{
  for (const std::string state : {"lifted", "skipped: torque off", "skipped: stopped", "disabled"}) {
    EXPECT_FALSE(state_waits(fresh(state))) << state;
  }
  EXPECT_FALSE(state_waits(fresh("idle", false)));
}

TEST(Evaluate, FreshReadyStatesGoAtOnce)
{
  for (const std::string state : {"lifted", "skipped: torque off", "disabled"}) {
    const auto readiness = evaluate(fresh(state), FRESH_AGE_S, 0.0, TIMEOUT_S);
    EXPECT_TRUE(readiness.ready) << state;
    EXPECT_EQ(readiness.reason, state);
  }
}

TEST(Evaluate, TheDisabledDefaultDoesNotWait)
{
  const auto readiness = evaluate(DrivePoseState{"disabled", "", false, 1}, FRESH_AGE_S, 0.0, TIMEOUT_S);
  EXPECT_TRUE(readiness.ready);
  EXPECT_EQ(readiness.reason, "disabled");
}

TEST(Evaluate, NoStateDoesNotWait)
{
  const auto readiness = evaluate(std::nullopt, 0.0, 0.0, TIMEOUT_S);
  EXPECT_TRUE(readiness.ready);
  EXPECT_EQ(readiness.reason, "no fresh state");
}

TEST(Evaluate, AStaleStateDoesNotWait)
{
  for (const std::string state : {"lifting", "lowering", "idle"}) {
    const auto readiness = evaluate(fresh(state), STALE_AGE_S, 0.0, TIMEOUT_S);
    EXPECT_TRUE(readiness.ready) << state;
    EXPECT_EQ(readiness.reason, "no fresh state") << state;
  }
}

TEST(Evaluate, FreshWaitingStatesWaitUntilTheTimeout)
{
  for (const auto & state : {fresh("lifting"), fresh("lowering"), fresh("idle", true)}) {
    EXPECT_FALSE(evaluate(state, FRESH_AGE_S, 7.9, TIMEOUT_S).ready) << state.state;
    const auto readiness = evaluate(state, FRESH_AGE_S, TIMEOUT_S, TIMEOUT_S);
    EXPECT_TRUE(readiness.ready) << state.state;
    EXPECT_EQ(readiness.reason, "timeout") << state.state;
  }
}

TEST(Evaluate, FreshnessBoundaryIsFreshS)
{
  EXPECT_FALSE(evaluate(fresh("lifting"), arm_drive_wait_bt_cpp::FRESH_S, 0.0, TIMEOUT_S).ready);
  EXPECT_TRUE(evaluate(fresh("lifting"), arm_drive_wait_bt_cpp::FRESH_S + 0.01, 0.0, TIMEOUT_S).ready);
}
