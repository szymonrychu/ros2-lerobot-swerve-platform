---
title: Head servo group config
status: todo
repo: ros2-lerobot-sverve-platform
value: "The pan/tilt servos are driven on the base bus with their ranges, caps and home, without touching the arm bus."
files: [ansible/group_vars/client.yml, tests/test_head_servo_config.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible && cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q"
depends_on: [head-urdf, bridge-startup-goal]
owns: [head-servo-group]
uses: [head-urdf, bridge-position-caps, bridge-startup-position]
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

# Head servo group config

## Requirement

- Re-read the split-bus epic's end state first: the base bus node entry in `ansible/group_vars/client.yml` (expected `base_servos`, `node_type: feetech_servos`, primary namespace `swerve_drive`) and its bus-cost test (expected `nodes/bridges/feetech_servos/tests/test_bus_cost.py` or root `tests/test_base_bus_cost.py`). The head group goes on THAT node, never on `lerobot_follower` (D1, U1).
- Its config gains `extra_groups` entry `namespace: head` with `head_pan` (id 40) and `head_tilt` (id 41) (D2, my-call defaults; comment: set with `scripts/set_servo_id.py` before deploy, the node blocks at startup until every configured servo answers), `command_min_steps`/`command_max_steps` from the ranges (pan 1024..3072 = +-90 deg; tilt 1707..2560 = -30..+45 deg, swapped consistently if `inverted` is set), `max_speed_rad_s: 1.0`, `max_acceleration_rad_s2: 3.0` (D3), `startup_position_rad` = the home pose (pan 0.0, tilt 0.5236) (D4). `direct_command_sources` of that node includes `autonomy` and `web_ui`. A comment block states the sign conventions (pan + = left, tilt + = down) and that the steps, the URDF limits and the web UI ranges are kept equal by tests.
- The split epic's bus-cost test is pointed at the real head group from client.yml (instead of any stand-in head it carried) and still shows base + head inside the base bus rate target.
- Tests first in new root `tests/test_head_servo_config.py`, seen failing: the head group exists only on the base bus node (not on lerobot_follower); joint names and ids unique on that bus; steps equal the URDF limits of `nodes/web_ui/urdf/head.urdf` converted with 4096 steps/turn around 2048 (+-1 step); caps present and > 0; startup positions inside the limits; `autonomy` and `web_ui` in direct_command_sources.
- `tests/README.md` lists the new test file and its tests.

## Acceptance conditions

- a1: the head group is on the base bus node with ids, steps, caps and startup home, not on lerobot_follower [evidence: test: tests/test_head_servo_config.py]
- a2: the head steps equal the head URDF limits [evidence: test: tests/test_head_servo_config.py]
- a3: the split epic bus-cost test reads the real head group and passes [evidence: judgement]
- a4: tests/README.md lists test_head_servo_config.py [evidence: command: grep -q 'test_head_servo_config.py' tests/README.md]
- a5: root tests, feetech tests and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible && cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_head_servo_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
