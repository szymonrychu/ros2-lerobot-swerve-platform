---
title: Bridge startup goal
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The head drives to its home pose (forward and down) whenever the base bus node `base_servos` starts."
files: [nodes/bridges/feetech_servos/config.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/bridge_cycle.py, nodes/bridges/feetech_servos/tests/test_startup_goal.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [bridge-position-speed-caps]
owns: [bridge-startup-position]
uses: [bridge-position-caps]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--bridge-startup-goal
branch: pan-tilt-head-control-with-a--bridge-startup-goal
verified_head: ""
run: ""
provides: []
---

# Bridge startup goal

## Requirement

- `config.py` (D4, U4): `JointEntry` gains `startup_position_rad: float | None = None`, allowed only on `mode: position` joints without a source range mapping, finite, and inside the joint's `command_min/max_steps` when those are set (else rejected).
- `bridge.py`: after the existing startup sequence (hold present position, torque on), each joint with `startup_position_rad` whose torque came on gets that position queued as its first pending target through `BridgeCycle` (the same path as a received command, `direct` semantics), so it goes out with the joint's caps (bridge-position-speed-caps) in the first write cycle. A servo left unheld or with torque off gets no startup goal (logged). Joints without the key are untouched (they hold their present position as today).
- The startup goal is logged once per joint (name, rad, steps).
- `nodes/bridges/feetech_servos/README.md`: `startup_position_rad` documented (the head uses it for its home pose; it moves the servo at every bridge start, with the caps).
- Tests first in new `tests/test_startup_goal.py` (`sts_bus.py`, `ros_stubs.py`), seen failing: a joint with the key gets exactly one goal write to the expected steps after torque-on, inside the capped block when capped; no write for joints without the key; no write for a servo whose torque failed; config rejection for velocity joints, source-mapped joints, non-finite and out-of-range values.

## Acceptance conditions

- a1: a joint with startup_position_rad is driven there once after torque-on, with its caps [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q]
- a2: no startup goal for joints without the key or whose torque did not come on [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q]
- a3: invalid startup_position_rad configs are rejected [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q]
- a4: the README documents startup_position_rad [evidence: command: grep -q 'startup_position_rad' nodes/bridges/feetech_servos/README.md]
- a5: feetech_servos tests and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/bridge_cycle.py`
- `nodes/bridges/feetech_servos/tests/test_startup_goal.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- `config.py:JointEntry.startup_position_rad` (part `bridge-startup-position`); startup queueing in `bridge.py:run_bridge` through `BridgeCycle.handle_command` or an equivalent public method.

## Expected outcome

A configured joint drives to its startup position once after torque-on, at its caps; other joints behave as today.

## Steps

- Write `tests/test_startup_goal.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q`; expect failures.
- Implement; run the verify command; commit `feat(feetech_servos): optional startup position per joint`.

## Notes

- Reuse `startup_torque.py` results (unheld ids, failed torque ids) to decide which joints get the goal.
- `tests/test_bridge_startup_torque.py` shows how the startup sequence is tested.
