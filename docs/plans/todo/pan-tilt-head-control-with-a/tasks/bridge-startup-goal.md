---
title: Bridge startup goal
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head drives to its home pose (forward and down) whenever the base bus node starts."
files: [nodes/bridges/feetech_servos/config.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/bridge_cycle.py, nodes/bridges/feetech_servos/tests/test_startup_goal.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [bridge-position-speed-caps]
owns: [bridge-startup-position]
uses: [bridge-position-caps]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
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

- a1: a joint with startup_position_rad is driven there once after torque-on, with its caps [evidence: test: nodes/bridges/feetech_servos/tests/test_startup_goal.py]
- a2: no startup goal for joints without the key or whose torque did not come on [evidence: test: nodes/bridges/feetech_servos/tests/test_startup_goal.py]
- a3: invalid startup_position_rad configs are rejected [evidence: test: nodes/bridges/feetech_servos/tests/test_startup_goal.py]
- a4: the README documents startup_position_rad [evidence: command: grep -q 'startup_position_rad' nodes/bridges/feetech_servos/README.md]
- a5: feetech_servos tests and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/bridge_cycle.py`
- `nodes/bridges/feetech_servos/tests/test_startup_goal.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
