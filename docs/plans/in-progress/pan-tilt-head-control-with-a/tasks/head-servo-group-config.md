---
title: Head servo group config
status: todo
repo: ros2-lerobot-sverve-platform
value: "The pan/tilt servos are driven on the base bus with their ranges, caps and home, without touching the arm bus."
files: [ansible/group_vars/client.yml, tests/test_head_servo_config.py, tests/README.md, nodes/bridges/feetech_servos/tests/test_bus_budget.py, nodes/bridges/feetech_servos/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible && cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q"
depends_on: [head-urdf, bridge-startup-goal]
owns: [head-servo-group]
uses: [head-urdf, bridge-position-caps, bridge-startup-position]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--head-servo-group-config
branch: pan-tilt-head-control-with-a--head-servo-group-config
verified_head: ""
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: []
process_gaps: []
---

# Head servo group config

## Requirement

- The split-bus epic leaves the base bus on `base_servos` (`node_type: feetech_servos`, primary namespace `swerve_drive`, 50 Hz, own `/dev/serial/by-id` adapter, `battery_interval_s: 0`, a client.yml comment marking where the head goes) and `nodes/bridges/feetech_servos/tests/test_bus_budget.py` (fake bus, configs loaded from client.yml, a stand-in `HEAD` constant). Re-read both first. The head group goes on `base_servos`, never on `lerobot_follower` (D1, U1).
- Its config gains `extra_groups` entry `namespace: head` with `head_pan` (id 20) and `head_tilt` (id 21) (D2 defaults; comment: set with `scripts/set_servo_id.py` before deploy, the node blocks at startup until every configured servo answers, so deploy only after the servos are installed), `command_min_steps`/`command_max_steps` from the ranges (pan 1024..3072 = +-90 deg; tilt 1707..2560 = -30..+45 deg, swapped consistently if `inverted` is set), `max_speed_rad_s: 1.0`, `max_acceleration_rad_s2: 3.0` (D3), `startup_position_rad` = the home pose (pan 0.0, tilt 0.5236) (D4). Its `direct_command_sources` includes `autonomy` and `web_ui`. A comment block states the sign conventions (pan + = left; tilt + = looking down, decided in ruling R3, confirmed by the user on 2026-10-18; flipped only via `inverted` and the URDF axis) and that the steps, the URDF limits and the web UI ranges are kept equal by tests.
- `test_bus_budget.py`: the base layout now takes the real head group from the loaded `base_servos` config (with its capped 7-byte goal block) instead of adding the stand-in `HEAD`, and still meets `BASE_HEADROOM` (0.8) at the base `control_loop_hz` of 50 Hz (split epic projection: base + head 14.4 ms per worst-case driving cycle vs the 20 ms period; the capped head block adds a few bytes); the arm + head option keeps using the stand-in. The feetech README `## Bus budget` table is recomputed from the test.
- Tests first in new root `tests/test_head_servo_config.py`, seen failing: the head group exists only on `base_servos` (not on lerobot_follower); joint names and ids unique on that bus; steps equal the URDF limits of `nodes/web_ui/urdf/head.urdf` converted with 4096 steps/turn around 2048 (+-1 step); caps present and > 0; startup positions inside the limits; `autonomy` and `web_ui` in direct_command_sources.
- `tests/README.md` lists the new test file and its tests.

## Acceptance conditions

- a1: the head group is on the base bus node `base_servos` with ids, steps, caps and startup home, not on lerobot_follower [evidence: test: tests/test_head_servo_config.py]
- a2: the head steps equal the head URDF limits [evidence: test: tests/test_head_servo_config.py]
- a3: the bus budget test uses the real head group and base + head still fits the base rate [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a4: tests/README.md lists test_head_servo_config.py [evidence: command: grep -q 'test_head_servo_config.py' tests/README.md]
- a5: root tests, feetech tests and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible && cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_head_servo_config.py`
- `tests/README.md`
- `nodes/bridges/feetech_servos/tests/test_bus_budget.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- client.yml `base_servos` config `extra_groups[namespace=head]` (part `head-servo-group`): joints head_pan (20), head_tilt (21), steps, caps, startup_position_rad; topics /head/joint_states, /head/joint_commands.

## Expected outcome

The base bus node `base_servos` config drives the head with its ranges, caps and home; the bus budget still holds with the real head.

## Steps

- Write `tests/test_head_servo_config.py`; run `uv run pytest tests/test_head_servo_config.py -q`; expect failures.
- Edit client.yml and the bus budget test; run the verify command; commit `feat(ansible): head servo group on the base bus`.

## Notes

- Do not deploy: the node blocks on missing servos; the orchestrator runbook deploys after the hardware gate.
- Steps from `command_mapping.position_to_raw_steps` (0 rad = 2048, 4096 steps per turn).

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-urdf
