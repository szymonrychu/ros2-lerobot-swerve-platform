---
title: Bridge torque release enable
status: done
repo: ros2-lerobot-sverve-platform
value: "A servo group can go limp and come back holding its present pose, with no goal ever written into a limp servo."
files: [nodes/bridges/feetech_servos/torque_control.py, nodes/bridges/feetech_servos/bridge_cycle.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/tests/test_torque_control.py, nodes/bridges/feetech_servos/tests/test_bridge_cycle.py]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_torque_control.py tests/test_bridge_cycle.py -q && uv run --frozen poe lint"
depends_on: []
owns: [torque-gate]
uses: []
model: opus
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--bridge-torque-release-enable
branch: servo-torque-off-and-neutral--bridge-torque-release-enable
verified_head: 8a06a5720db983236a8df9305a43394af2fbf5fd
run: servo-torque-off-and-neutral
provides: [torque-gate, present-read-retry]
acs_for_audit: []
process_gaps: []
---

# Bridge torque release enable

## Requirement

- New module `nodes/bridges/feetech_servos/torque_control.py` (no rclpy) with class `TorqueControl` (constructed from the servo bus, the config groups, the shared `last_written` cache and the `BridgeCycle`). Methods `release(namespace) -> TorqueResult` and `enable(namespace) -> TorqueResult`; properties `released_ids` (set of servo ids) and `joint_torque` (servo id -> last verified torque_enable value); `note_register_dump(servo_id, registers)` updates `joint_torque` from a register dump's `torque_enable`. `TorqueResult` is a dataclass `success: bool`, `message: str` (names every failed joint), `joints: dict[str, int]` (joint name -> verified torque value).
- release (D1): the group's position-mode joints are gated in `BridgeCycle` first (`BridgeCycle.release(ids)`: pending position targets dropped, later commands ignored, no goal refresh), then `torque_enable = 0` is written to each one bypassing the `last_written` cache (drop the cached `torque_enable` before `write_register`) and verified by readback with `startup_torque.set_startup_torque_state` (move `TORQUE_WRITE_ATTEMPTS` / `TORQUE_VERIFY_SLEEP_S` from bridge.py into torque_control.py and import them back). Velocity-mode joints are never released: when the group has any (the base `swerve_drive` group), they are stopped with `BridgeCycle.stop` and their velocity commands are dropped while the group is released; their torque stays on. Releasing an already released group succeeds and writes torque 0 again.
- enable (D1): per gated joint of the group: read present_position (up to 3 attempts, a `None` read is retried), write goal_position = present with `write_register` after dropping the cached `goal_position` (cache ends holding present), then `torque_enable = 1` bypassing the cache and verified by readback. The goal write always happens before the torque write. A joint whose read, goal write or verify fails stays gated (released) and is named in the message; the others are un-gated (`BridgeCycle.unrelease(ids)`). After enable, velocity commands of the group are accepted again.
- `BridgeCycle` (bridge_cycle.py) gains `released_ids`, `release(ids)`, `unrelease(ids)`; `handle_command` ignores released joints (position and velocity), `write_goal_positions` never includes a released id (also when the goal refresh is due), `queue_startup_goals` skips released ids.
- Tests first, seen failing: `tests/test_torque_control.py` (fake bus from `tests/sts_bus.py` or a MagicMock): release writes torque 0 although the cache already holds 0, and verifies it; after release, commands for the group over several cycles with the refresh due write no goal_position for the released ids; enable writes goal = present before torque 1 although the cache holds torque 1 (a servo that dropped torque by itself); a failing present read leaves that joint released, enables the rest and names it; base group: velocity joints stopped on release, their commands dropped while released, no torque write to velocity joints; another group on the same bus keeps receiving commands. `tests/test_bridge_cycle.py`: gate tests for `handle_command`, `write_goal_positions` with refresh due, `queue_startup_goals`.

## Acceptance conditions

- a1: release gates the group first, writes torque 0 bypassing the cache and verifies it; velocity joints are stopped, never released [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_control.py]
- a2: no goal_position is written for a released servo on any cycle path, refresh included [evidence: test: nodes/bridges/feetech_servos/tests/test_bridge_cycle.py]
- a3: enable writes goal = present before torque 1, both bypassing the cache, and leaves failed joints released [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_control.py]
- a4: TorqueControl exists in torque_control.py [evidence: symbol: nodes/bridges/feetech_servos/torque_control.py::TorqueControl]
- a5: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/torque_control.py`
- `nodes/bridges/feetech_servos/bridge_cycle.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_torque_control.py`
- `nodes/bridges/feetech_servos/tests/test_bridge_cycle.py`

## Interfaces

- `torque_control.TorqueControl.release(namespace: str) -> TorqueResult`, `.enable(namespace: str) -> TorqueResult`, `.released_ids: set[int]`, `.joint_torque: dict[int, int]`, `.note_register_dump(servo_id: int, registers: dict[str, int]) -> None` (part `torque-gate`).
- `bridge_cycle.BridgeCycle.release(ids: Iterable[int]) -> None`, `.unrelease(ids: Iterable[int]) -> None`, `.released_ids: set[int]`.
- Kept: `startup_torque.set_startup_torque_state`, `registers.write_register` signatures unchanged.

## Expected outcome

The bridge can release and re-enable a group without any goal reaching a limp servo and without a jump on re-enable; nothing is wired to ROS yet (bridge-torque-services-topic does that).

## Steps

- Write `tests/test_torque_control.py` and the new gate tests in `tests/test_bridge_cycle.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_torque_control.py tests/test_bridge_cycle.py -q`; expect failures (module missing, gate absent).
- Implement `torque_control.py` and the `BridgeCycle` gate; rerun; expect pass.
- Run the verify command; commit `feat(feetech): torque release and hold-present enable per group`.

## Notes

- Brief facts: write_register skips equal values (registers.py:272), so torque 1 after a self-dropped torque is a no-op without the bypass; hold-before-torque-on exists only at startup (bridge.py:213-222, startup_torque.py:44).
- Whether an STS3215 re-enables torque on a goal write while torque is off is unknown: the gate makes it moot, so test that no goal is written, not what the servo would do.
- Keep the module rclpy-free; bridge.py wiring is the next task.
