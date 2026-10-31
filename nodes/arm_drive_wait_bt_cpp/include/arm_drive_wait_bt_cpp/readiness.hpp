// Whether a NavigateToPose goal may drive, from the drive pose controller state (no ROS): mcp_server publishes the
// JSON {state, reason, enabled, stamp_ns} latched on /arm/drive_pose/state; the goal goes on lifted, "skipped: ..." or
// disabled stamped at or after the wait's start, and on the timeout (never a failure: a down mcp_server or a stuck lift
// drives with today's view).
#ifndef ARM_DRIVE_WAIT_BT_CPP__READINESS_HPP_
#define ARM_DRIVE_WAIT_BT_CPP__READINESS_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace arm_drive_wait_bt_cpp
{

constexpr const char * LIFTED = "lifted";
constexpr const char * DISABLED = "disabled";
constexpr const char * SKIPPED_PREFIX = "skipped: ";
constexpr const char * TIMEOUT = "timeout";

// One /arm/drive_pose/state message (mcp_server models.DrivePoseState).
struct DrivePoseState
{
  std::string state;    // idle | lifting | lifted | lowering | disabled | "skipped: <reason>"
  std::string reason;   // why the arm was skipped or left alone ("" otherwise)
  bool enabled{false};  // whether new lifts are allowed
  int64_t stamp_ns{0};  // wall clock (ns since the epoch) of the last change
};

// The outcome of one readiness check: ready with the reason (the state, or "timeout"), or not ready yet.
struct Readiness
{
  bool ready{false};
  std::string reason;
};

// Parse the state JSON.
// json: the std_msgs/String payload.
// Returns the state, or nullopt when the JSON is malformed or a field is missing or of the wrong type.
std::optional<DrivePoseState> parse_state(const std::string & json);

// Whether a state lets the goal drive: lifted, "skipped: ..." or disabled, stamped at or after start_ns.
// state: the parsed state; start_ns: wall clock (ns since the epoch) of the wait's start.
// Returns true when the goal may go.
bool state_is_ready(const DrivePoseState & state, int64_t start_ns);

// The readiness of a waiting goal.
// latest: the last parsed state (nullopt: none received, or the last one was malformed); start_ns: wall clock (ns) of
// the wait's start; elapsed_s: seconds since the start; timeout_s: the longest wait.
// Returns ready with the state on a ready state, ready with "timeout" once elapsed_s >= timeout_s, else not ready.
Readiness evaluate(const std::optional<DrivePoseState> & latest, int64_t start_ns, double elapsed_s, double timeout_s);

}  // namespace arm_drive_wait_bt_cpp

#endif  // ARM_DRIVE_WAIT_BT_CPP__READINESS_HPP_
