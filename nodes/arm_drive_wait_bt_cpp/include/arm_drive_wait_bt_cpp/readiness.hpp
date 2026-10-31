// Whether a NavigateToPose goal may drive, from the drive pose controller state (no ROS): mcp_server publishes the
// JSON {state, reason, enabled, stamp_ns} latched on /arm/drive_pose/state and republishes it at 1 Hz. A state counts
// only while fresh (received within FRESH_S on the plugin's clock). The goal waits only while the fresh state is
// lifting, lowering or idle with the drive pose enabled (the controller has not seen the goal yet); it goes at once on
// any other fresh state (lifted, "skipped: ...", disabled) and when there is no fresh state (mcp_server down), and on
// the timeout. It never fails: a down mcp_server or a stuck lift drives with today's view.
#ifndef ARM_DRIVE_WAIT_BT_CPP__READINESS_HPP_
#define ARM_DRIVE_WAIT_BT_CPP__READINESS_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace arm_drive_wait_bt_cpp
{

constexpr const char * IDLE = "idle";
constexpr const char * LIFTING = "lifting";
constexpr const char * LOWERING = "lowering";
constexpr const char * TIMEOUT = "timeout";
constexpr const char * NO_FRESH_STATE = "no fresh state";
// A received state counts this long (s): mcp_server republishes at 1 Hz, so 2.5 s tolerates one lost message.
constexpr double FRESH_S = 2.5;

// One /arm/drive_pose/state message (mcp_server models.DrivePoseState).
struct DrivePoseState
{
  std::string state;    // idle | lifting | lifted | lowering | disabled | "skipped: <reason>"
  std::string reason;   // why the arm was skipped or left alone ("" otherwise)
  bool enabled{false};  // whether new lifts are allowed
  int64_t stamp_ns{0};  // wall clock (ns since the epoch) of the last change
};

// The outcome of one readiness check: ready with the reason (the state, "no fresh state" or "timeout"), or not yet.
struct Readiness
{
  bool ready{false};
  std::string reason;
};

// Parse the state JSON.
// json: the std_msgs/String payload.
// Returns the state, or nullopt when the JSON is malformed or a field is missing or of the wrong type.
std::optional<DrivePoseState> parse_state(const std::string & json);

// Whether a fresh state holds the goal: lifting, lowering, or idle with the drive pose enabled.
// state: the parsed state.
// Returns true when the goal waits.
bool state_waits(const DrivePoseState & state);

// The readiness of a waiting goal.
// latest: the last parsed state (nullopt: none received, or the last one was malformed); age_s: seconds since it was
// received (ignored without a state); elapsed_s: seconds since the wait started; timeout_s: the longest wait.
// Returns not ready while a fresh state waits and elapsed_s < timeout_s; ready with "timeout" after that; ready with
// "no fresh state" without a state or with one older than FRESH_S; else ready with the state.
Readiness evaluate(const std::optional<DrivePoseState> & latest, double age_s, double elapsed_s, double timeout_s);

}  // namespace arm_drive_wait_bt_cpp

#endif  // ARM_DRIVE_WAIT_BT_CPP__READINESS_HPP_
