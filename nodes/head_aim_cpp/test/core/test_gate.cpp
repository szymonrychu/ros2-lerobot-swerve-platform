#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

#include "head_aim_cpp/config.hpp"
#include "head_aim_cpp/gate.hpp"

namespace
{

using head_aim_cpp::Aim;
using head_aim_cpp::AimGate;
using head_aim_cpp::Command;
using head_aim_cpp::Config;

constexpr double DEG = M_PI / 180.0;
constexpr double TOL = 1e-12;

Aim target(double pan_deg, double tilt_deg) { return Aim{pan_deg * DEG, tilt_deg * DEG, false}; }

// A gate with the shipped defaults and a goal active since t = 0, its first target (0, 25) already sent at t = 0.
AimGate active_gate_after_first_command()
{
  AimGate gate{Config{}};
  gate.on_goal_status(true, 0.0);
  const auto first = gate.decide(target(0.0, 25.0), 0.0);
  EXPECT_TRUE(first);
  return gate;
}

}  // namespace

TEST(GoalStatus, AcceptedExecutingAndCancelingAreActive)
{
  EXPECT_FALSE(head_aim_cpp::any_goal_active({}));
  EXPECT_FALSE(head_aim_cpp::any_goal_active({0, 4, 5, 6}));  // UNKNOWN, SUCCEEDED, CANCELED, ABORTED
  EXPECT_TRUE(head_aim_cpp::any_goal_active({4, 1}));
  EXPECT_TRUE(head_aim_cpp::any_goal_active({2}));
  EXPECT_TRUE(head_aim_cpp::any_goal_active({3}));
}

TEST(AimGate, NoGoalNoCommand)
{
  AimGate gate{Config{}};
  EXPECT_FALSE(gate.decide(target(0.0, 25.0), 0.0));
  gate.on_goal_status(false, 1.0);
  EXPECT_FALSE(gate.decide(target(0.0, 25.0), 1.0));
}

TEST(AimGate, FirstTargetOfAGoalIsCommandedAtOnce)
{
  AimGate gate{Config{}};
  gate.on_goal_status(true, 3.0);
  const auto cmd = gate.decide(target(12.0, 25.0), 3.0);
  ASSERT_TRUE(cmd);
  EXPECT_NEAR(cmd->pan, 12.0 * DEG, TOL);
  EXPECT_NEAR(cmd->tilt, 25.0 * DEG, TOL);
  EXPECT_FALSE(cmd->home);
}

TEST(AimGate, SmallPanChangeIsNotCommanded)
{
  AimGate gate = active_gate_after_first_command();
  EXPECT_FALSE(gate.decide(target(5.0, 25.0), 2.0));
  EXPECT_FALSE(gate.decide(target(-9.9, 25.0), 3.0));
}

TEST(AimGate, LargePanChangeWaitsForTheInterval)
{
  AimGate gate = active_gate_after_first_command();
  EXPECT_FALSE(gate.decide(target(12.0, 25.0), 0.4));
  EXPECT_FALSE(gate.decide(target(12.0, 25.0), 0.9));
  const auto cmd = gate.decide(target(12.0, 25.0), 1.0);
  ASSERT_TRUE(cmd);
  EXPECT_NEAR(cmd->pan, 12.0 * DEG, TOL);
  // The next re-aim is measured from this command, and from its pan.
  EXPECT_FALSE(gate.decide(target(25.0, 25.0), 1.5));
  EXPECT_FALSE(gate.decide(target(20.0, 25.0), 2.5));
  EXPECT_TRUE(gate.decide(target(25.0, 25.0), 2.5));
}

TEST(AimGate, TiltChangeAboveItsThresholdIsCommanded)
{
  AimGate gate = active_gate_after_first_command();
  EXPECT_FALSE(gate.decide(target(0.0, 29.0), 2.0));
  const auto cmd = gate.decide(target(0.0, 31.0), 2.0);
  ASSERT_TRUE(cmd);
  EXPECT_NEAR(cmd->tilt, 31.0 * DEG, TOL);
}

TEST(AimGate, YieldSourcesPauseAndExtendTheWindow)
{
  AimGate gate = active_gate_after_first_command();
  gate.on_foreign_command("web_ui", 10.0);
  EXPECT_TRUE(gate.yielding(10.0));
  EXPECT_FALSE(gate.decide(target(40.0, 25.0), 11.0));
  gate.on_foreign_command("autonomy", 15.0);
  EXPECT_FALSE(gate.decide(target(40.0, 25.0), 20.5));  // past t + 10, inside t + 5 + 10
  EXPECT_FALSE(gate.decide(target(40.0, 25.0), 24.9));
  // After the window the head is wherever the foreign command left it: the target is sent at once, even unchanged.
  const auto cmd = gate.decide(target(0.0, 25.0), 25.0);
  ASSERT_TRUE(cmd);
  EXPECT_NEAR(cmd->pan, 0.0, TOL);
}

TEST(AimGate, OwnAndUnknownSourcesDoNotYield)
{
  AimGate gate = active_gate_after_first_command();
  gate.on_foreign_command("head_aim", 2.0);
  gate.on_foreign_command("calibration", 2.0);
  EXPECT_FALSE(gate.yielding(2.0));
  EXPECT_TRUE(gate.decide(target(20.0, 25.0), 2.0));
}

TEST(AimGate, GoalEndSendsExactlyOneHomeCommand)
{
  AimGate gate = active_gate_after_first_command();
  gate.on_goal_status(false, 5.0);
  const auto home = gate.decide(std::nullopt, 5.0);
  ASSERT_TRUE(home);
  EXPECT_TRUE(home->home);
  EXPECT_NEAR(home->pan, 0.0, TOL);
  EXPECT_NEAR(home->tilt, 0.5236, TOL);
  EXPECT_FALSE(gate.decide(std::nullopt, 5.2));
  EXPECT_FALSE(gate.decide(target(30.0, 25.0), 9.0));
  gate.on_goal_status(false, 9.5);  // repeated inactive status: no second home
  EXPECT_FALSE(gate.decide(std::nullopt, 9.5));
}

TEST(AimGate, GoalEndWhileYieldingDoesNotHome)
{
  AimGate gate = active_gate_after_first_command();
  gate.on_foreign_command("web_ui", 4.0);
  gate.on_goal_status(false, 5.0);
  EXPECT_FALSE(gate.decide(std::nullopt, 5.0));
  EXPECT_FALSE(gate.decide(std::nullopt, 20.0));  // not later either
}

TEST(AimGate, MissingTargetWhileActiveGivesNothing)
{
  AimGate gate{Config{}};
  gate.on_goal_status(true, 0.0);
  EXPECT_FALSE(gate.decide(std::nullopt, 0.0));
  EXPECT_FALSE(gate.decide(std::nullopt, 5.0));
  EXPECT_TRUE(gate.decide(target(0.0, 25.0), 5.1));  // still the first command of the goal
}

TEST(AimGate, NewGoalStartsFresh)
{
  AimGate gate = active_gate_after_first_command();
  gate.on_goal_status(false, 5.0);
  ASSERT_TRUE(gate.decide(std::nullopt, 5.0));  // home
  gate.on_goal_status(true, 5.2);
  const auto cmd = gate.decide(target(0.0, 25.0), 5.2);  // same target as the previous goal, sent at once
  ASSERT_TRUE(cmd);
  EXPECT_FALSE(cmd->home);
}
