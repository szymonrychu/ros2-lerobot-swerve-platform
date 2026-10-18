---
title: Tilt guard torque topic
status: done
repo: ros2-lerobot-sverve-platform
value: A tilt-guard torque_off reaches the drive servos on their own bus instead of being dropped by the arm bridge.
files: [nodes/swerve_drive_controller/swerve_drive_controller/tilt_guard.py, nodes/swerve_drive_controller/tests/test_tilt_guard.py, nodes/swerve_drive_controller/tests/test_node_tilt_guard.py, nodes/swerve_drive_controller/README.md]
verify: "cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q && uv run --frozen poe lint"
depends_on: []
owns: [TiltGuardConfig.torque_topic default]
uses: []
model: sonnet
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: 83af294455d8aa6986d54f93927266765dc6b374
run: split-servo-buses-arm-alone
provides: []
acs_for_audit: []
process_gaps: []
---

# Tilt guard torque topic

## Requirement

- `tilt_guard.py`: module constant `DEFAULT_TORQUE_TOPIC = "/swerve_drive/set_register"` (the `base_servos` feetech bridge's set_register topic, design D6); `TiltGuardConfig.torque_topic` defaults to it; the field docstring says it is the base bus bridge.
- Tests first, seen failing: `tests/test_tilt_guard.py` asserts the default is `/swerve_drive/set_register`; `tests/test_node_tilt_guard.py` reads the published torque payloads from `DEFAULT_TORQUE_TOPIC` (imported, no `/follower/set_register` literal left) and still sees `torque_enable` 0 then 1 for the four drive joints with `error_action: torque_off`.
- `README.md`: the overview bullet that says the 8 servos share the follower-arm serial bus is rewritten: they have their own bus (second USB adapter) served by the `base_servos` feetech bridge at 50 Hz, namespace `swerve_drive`; the Publishes line, the torque_off text and the `torque_topic` table row name `/swerve_drive/set_register`.

## Acceptance conditions

- a1: the tilt guard torque topic defaults to the base bus bridge [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_tilt_guard.py -q]
- a2: torque_off payloads for the four drive joints are published on the default topic [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_node_tilt_guard.py -q]
- a3: nothing in the swerve controller tree names the follower set_register or the follower-arm bus [evidence: command: ! git grep -n -E 'follower-arm serial bus|/follower/set_register|follower_set_register' -- nodes/swerve_drive_controller]
- a4: swerve_drive_controller suite and lint pass [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

## Files

- `nodes/swerve_drive_controller/swerve_drive_controller/tilt_guard.py`
- `nodes/swerve_drive_controller/tests/test_tilt_guard.py`
- `nodes/swerve_drive_controller/tests/test_node_tilt_guard.py`
- `nodes/swerve_drive_controller/README.md`

## Interfaces

- `swerve_drive_controller.tilt_guard.DEFAULT_TORQUE_TOPIC: str`, `TiltGuardConfig.torque_topic` (owned contract `TiltGuardConfig.torque_topic default`; split-bus-group-vars sets the same value in client.yml).

## Expected outcome

Without any config, the tilt guard's torque_off writes go to `/swerve_drive/set_register`, where the base bridge knows the drive joints.

## Steps

- Edit the two tests; run `cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_tilt_guard.py tests/test_node_tilt_guard.py -q`; expect the default assertion to fail.
- Change the default and the README; run the verify command; commit `fix(swerve_controller): tilt guard torque_off targets the base bus bridge`.

## Notes

- Design D6. The deployed `error_action` stays `brake`; client.yml's `torque_topic` is changed by split-bus-group-vars, not here.
- No other code change: the node already publishes on `tilt_cfg.torque_topic` (`node.py:162`).

## Ruling

Accepted by the orchestrator: verified and merged (feetech 286, mcp_server 2465, root 1149 passed); mcp-base-servo-registers gate-repeat findings refuted by the reviewer.
