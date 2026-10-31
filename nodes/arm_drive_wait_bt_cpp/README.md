# arm_drive_wait_bt_cpp

Nav2 behavior tree plugin with one action node, `WaitForArmDriveReady`: it holds a NavigateToPose goal until mcp_server
lifted the arm to its drive pose (out of the head camera's view), skipped the lift or has the drive pose disabled, at
most `timeout_s`. It never fails a goal: a down mcp_server or a stuck lift drives on with today's view after the
timeout. The package builds the shared library `arm_drive_wait_bt_node`, which bt_navigator loads through
`plugin_lib_names`; it runs no process of its own.

## Layout

- `src/core/readiness.cpp`, `include/arm_drive_wait_bt_cpp/readiness.hpp`: the static library `arm_drive_wait_core`
  (no ROS): `parse_state` (the state JSON, nlohmann_json), `state_is_ready`, `evaluate` (ready state or timeout).
  `test/core/test_*.cpp` each become a gtest named after the file.
- `src/wait_for_arm_drive_ready.cpp`, `include/arm_drive_wait_bt_cpp/wait_for_arm_drive_ready.hpp`: the
  `BT::StatefulActionNode`, registered with `BT_REGISTER_NODES` in `libarm_drive_wait_bt_node.so`.
  `test/test_bt_node.cpp` loads that library into a `BT::BehaviorTreeFactory` (`registerFromPlugin`) and ticks a
  one-node tree against a test publisher.

## The node

| Port | Type | Default | Meaning |
|---|---|---|---|
| `topic` | string | `/arm/drive_pose/state` | the drive pose state topic mcp_server publishes |
| `timeout_s` | double | `8.0` | the longest wait; SUCCESS (reason `timeout`, logged as a warning) after it |

Input: `/arm/drive_pose/state`, `std_msgs/String` with the JSON `{state, reason, enabled, stamp_ns}` (mcp_server
`DrivePoseState`; `stamp_ns` is the wall clock of the last change), subscribed reliable + transient_local, depth 1, so
the latched state arrives at once. The subscription uses the rclcpp node of the blackboard entry `node`
(bt_navigator's client node) on its own callback group, spun on every tick.

Per goal (`onStart`, then `onRunning` on every BT tick):

- **SUCCESS** on `lifted`, `skipped: <reason>` or `disabled` stamped at or after the wait's start (wall clock, the same
  clock as mcp_server's stamps on the robot), or once `timeout_s` passed (steady clock);
- **RUNNING** on `idle`, `lifting`, `lowering`, an older stamp, malformed JSON or no message yet;
- never **FAILURE**. A halt (cancel, preempt) leaves nothing to undo.

The outcome is logged once per goal: INFO with the state and the wait, WARN on the timeout.

## Build and test

`./scripts/cpp-node-test.sh arm_drive_wait_bt_cpp` (docker, ros:jazzy-ros-base, the same arm64 as the Pi) builds the
package, runs the gtests and the ament linters (clang-format, cppcheck, xmllint) and ends
`cpp-node-test: arm_drive_wait_bt_cpp PASS`.
