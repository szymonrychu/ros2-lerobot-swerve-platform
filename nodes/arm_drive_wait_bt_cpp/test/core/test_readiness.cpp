// readiness.hpp without ROS: the state JSON parse, the ready states and the timeout fallback.
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
using arm_drive_wait_bt_cpp::state_is_ready;

constexpr int64_t START_NS = 1700000000000000000;
constexpr double TIMEOUT_S = 8.0;

std::string state_json(const std::string & state, int64_t stamp_ns)
{
  return "{\"state\":\"" + state + "\",\"reason\":\"\",\"enabled\":true,\"stamp_ns\":" + std::to_string(stamp_ns) + "}";
}

DrivePoseState state_at(const std::string & state, int64_t stamp_ns)
{
  return DrivePoseState{state, "", true, stamp_ns};
}

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

TEST(StateIsReady, LiftedSkippedAndDisabledAtOrAfterTheStart)
{
  for (const std::string state : {"lifted", "skipped: torque off", "skipped: stopped", "disabled"}) {
    EXPECT_TRUE(state_is_ready(state_at(state, START_NS), START_NS)) << state;
    EXPECT_TRUE(state_is_ready(state_at(state, START_NS + 1), START_NS)) << state;
  }
}

TEST(StateIsReady, IdleLiftingAndLoweringAreNotReady)
{
  for (const std::string state : {"idle", "lifting", "lowering", "skipped", "", "LIFTED"}) {
    EXPECT_FALSE(state_is_ready(state_at(state, START_NS + 1), START_NS)) << state;
  }
}

TEST(StateIsReady, AStampBeforeTheStartIsNotReady)
{
  for (const std::string state : {"lifted", "skipped: torque off", "disabled"}) {
    EXPECT_FALSE(state_is_ready(state_at(state, START_NS - 1), START_NS)) << state;
  }
}

TEST(Evaluate, ReadyStateIsReadyWithTheStateAsReason)
{
  const auto readiness = evaluate(state_at("skipped: torque off", START_NS), START_NS, 0.1, TIMEOUT_S);
  EXPECT_TRUE(readiness.ready);
  EXPECT_EQ(readiness.reason, "skipped: torque off");
}

TEST(Evaluate, NotReadyBeforeTheTimeout)
{
  EXPECT_FALSE(evaluate(state_at("lifting", START_NS), START_NS, 7.9, TIMEOUT_S).ready);
  EXPECT_FALSE(evaluate(state_at("lifted", START_NS - 1), START_NS, 7.9, TIMEOUT_S).ready);
  EXPECT_FALSE(evaluate(std::nullopt, START_NS, 7.9, TIMEOUT_S).ready);
}

TEST(Evaluate, TimeoutIsReadyWithReasonTimeout)
{
  for (const auto & latest :
       {std::optional<DrivePoseState>{}, std::optional<DrivePoseState>{state_at("lifting", START_NS)},
        std::optional<DrivePoseState>{state_at("lifted", START_NS - 1)}}) {
    const auto readiness = evaluate(latest, START_NS, TIMEOUT_S, TIMEOUT_S);
    EXPECT_TRUE(readiness.ready);
    EXPECT_EQ(readiness.reason, "timeout");
  }
}

TEST(Evaluate, AReadyStateWinsOverTheTimeout)
{
  const auto readiness = evaluate(state_at("lifted", START_NS), START_NS, TIMEOUT_S + 1.0, TIMEOUT_S);
  EXPECT_TRUE(readiness.ready);
  EXPECT_EQ(readiness.reason, "lifted");
}
