// Step-and-settle and yield gate of the head aim: decides when an aim target becomes a head command. Time is passed in
// (seconds, any monotonic origin); no ROS types.
//
// - No goal active: nothing, except one home command on the active-to-inactive transition (none while yielding).
// - Yielding (a command from a yield source within yield_s): nothing.
// - Goal active: the first target of the goal at once; later targets only when pan moved more than reaim_pan_deg or
//   tilt more than reaim_tilt_deg from the last command, and never sooner than min_reaim_interval_s after it. The step
//   and the pause let the head settle, so ground_estimator (which drops frames while the camera turns) keeps frames.
// - A foreign command moves the head away from the last command, so the first target after the yield window is sent
//   at once, as at the start of a goal.
#ifndef HEAD_AIM_CPP__GATE_HPP_
#define HEAD_AIM_CPP__GATE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "head_aim_cpp/aim.hpp"
#include "head_aim_cpp/config.hpp"

namespace head_aim_cpp
{

// action_msgs/GoalStatus values that mean a goal is running (ACCEPTED, EXECUTING, CANCELING).
constexpr int8_t GOAL_STATUS_ACCEPTED = 1;
constexpr int8_t GOAL_STATUS_EXECUTING = 2;
constexpr int8_t GOAL_STATUS_CANCELING = 3;

// A head command to publish, rad (pan + = left, tilt + = down). home is true for the goal-end home command.
struct Command
{
  double pan = 0.0;
  double tilt = 0.0;
  bool home = false;
};

// True when any status is ACCEPTED, EXECUTING or CANCELING.
// @param statuses the status values of a GoalStatusArray (int8_t list).
// @return whether a goal is active (bool).
bool any_goal_active(const std::vector<int8_t> & statuses);

// The gate state machine; one instance per node.
class AimGate
{
public:
  // @param cfg yield_sources, yield_s, reaim thresholds, min_reaim_interval_s and the home pose (Config).
  explicit AimGate(const Config & cfg);

  // Record the goal state.
  // @param active whether a NavigateToPose goal is active (bool).
  // @param now seconds.
  void on_goal_status(bool active, double now);

  // Record a head command seen on the command topic; one from a yield source starts or extends the yield window.
  // @param source its header.frame_id (std::string); the node's own source_tag and unknown sources are ignored.
  // @param now seconds.
  void on_foreign_command(const std::string & source, double now);

  // The command to publish now, if any; a returned command counts as sent.
  // @param target the current aim (Aim), nullopt when an input is missing.
  // @param now seconds.
  // @return the command (Command) or nullopt.
  std::optional<Command> decide(const std::optional<Aim> & target, double now);

  // @return whether the yield window is open at now (bool).
  bool yielding(double now) const;

  // @return whether a goal is active (bool).
  bool goal_active() const { return active_; }

private:
  Config cfg_;
  bool active_ = false;
  bool home_pending_ = false;
  double yield_until_ = -1.0e300;
  std::optional<Command> last_;  // last command of this goal; reset at goal start and by a foreign command
  double last_time_ = 0.0;
};

}  // namespace head_aim_cpp

#endif  // HEAD_AIM_CPP__GATE_HPP_
