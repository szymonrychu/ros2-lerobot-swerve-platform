---
title: Swerve released steering
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The swerve controller never drives limp steering servos and re-enable does not swing the wheels.
files: [nodes/swerve_drive_controller/swerve_drive_controller/control.py, nodes/swerve_drive_controller/swerve_drive_controller/node.py, nodes/swerve_drive_controller/swerve_drive_controller/config.py, nodes/swerve_drive_controller/tests/test_control_step.py, nodes/swerve_drive_controller/tests/test_config.py, nodes/swerve_drive_controller/README.md]
verify: "cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_control_step.py tests/test_config.py -q && uv run --frozen poe lint"
depends_on: [bridge-torque-services-topic]
owns: []
uses: [torque-state-topic]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--swerve-released-steering
branch: servo-torque-off-and-neutral--swerve-released-steering
verified_head: ""
run: ""
provides: []
---

# Swerve released steering

## Requirement

- Config (`config.py`): `torque_state_topic` (default `/swerve_drive/torque_state`; empty disables). `tests/test_config.py` covers default, override and disable.
- `node.py` subscribes with reliable + transient_local QoS depth 1, parses the `released` field (payload from bridge-torque-services-topic); an unparsable payload is logged and ignored. On release the latest cmd_vel sample is cleared.
- `control.control_step` (and `run_cycle`) gain `released: bool = False`. While released (D6): cmd_vel is ignored, every drive is commanded 0 rad/s, no finite steering position is published (NaN, which the bridge ignores) and there is no idle recentring. On the first cycle after release ends, the steering targets are seeded from the measured steering positions and the idle timer restarts, so the first published steering targets equal the measured positions and a recentre happens at the earliest `idle_recenter_s` later.
- Tests first, seen failing, in `tests/test_control_step.py`: released -> no finite steering, zero drive, cmd ignored; re-enable -> first steering targets equal measured; recentre not before `idle_recenter_s` after re-enable; tilt-guard inhibit behaviour unchanged.
- `README.md` `## Steering calibration` documents re-zeroing the steering from the web UI and the released behaviour.

## Acceptance conditions

- a1: while released no finite steering is published, drives are 0 and cmd_vel is ignored [evidence: test: nodes/swerve_drive_controller/tests/test_control_step.py]
- a2: after re-enable the steering targets start at the measured positions and the recentre waits idle_recenter_s [evidence: test: nodes/swerve_drive_controller/tests/test_control_step.py]
- a3: torque_state_topic config default, override and disable [evidence: test: nodes/swerve_drive_controller/tests/test_config.py]
- a4: README documents the web UI steering re-zero [evidence: doc-section: nodes/swerve_drive_controller/README.md#Steering calibration]
- a5: swerve suite and lint pass [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/swerve_drive_controller/swerve_drive_controller/control.py`
- `nodes/swerve_drive_controller/swerve_drive_controller/node.py`
- `nodes/swerve_drive_controller/swerve_drive_controller/config.py`
- `nodes/swerve_drive_controller/tests/test_control_step.py`
- `nodes/swerve_drive_controller/tests/test_config.py`
- `nodes/swerve_drive_controller/README.md`

## Interfaces

- Uses part `torque-state-topic` (`/swerve_drive/torque_state`, field `released`).
- `control_step(config, state, cmd, joints, now, inhibited=False, released=False)`; `run_cycle(..., released=False)`.
- Config key `torque_state_topic`.

## Expected outcome

The base steering can be released and re-zeroed without the controller driving limp servos, and re-enabling it does not swing the wheels.

## Steps

- Write the control-step and config tests; run `cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_control_step.py tests/test_config.py -q`; expect failures.
- Implement; run the verify command; commit `feat(swerve): no steering commands while the steering torque is released`.

## Notes

- Brief fact 14/20: the controller writes steering every 50 Hz cycle and recentres to 0 rad after 3 s idle; after a re-zero 0 rad is the new neutral.
- Odometry uses measured velocity only (node.py:56); keep it publishing from real joint states.
