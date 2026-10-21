---
title: Head pan inverted
status: todo
repo: ros2-lerobot-sverve-platform
value: "A positive pan command turns the head left again, so head TF, calibration and the agent's head moves point where they say."
files: [ansible/group_vars/client.yml, tests/test_head_servo_config.py, tests/README.md]
verify: "uv run pytest tests/test_head_servo_config.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: []
owns: [head-pan-sign]
uses: []
model: sonnet
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head pan inverted

## Requirement

Head pan direction fix (design D3; approval A2).

- `ansible/group_vars/client.yml` base_servos head group: `inverted: true` on head_pan (same pattern as head_tilt); the sign-convention comment records the finding (head sweep 2026-10-21: a +30 deg pan command turned the head to the robot's right, so the pan servo is mounted reversed; +pan stays left, CCW from above, matching head.urdf axis z+1). command_min_steps/command_max_steps stay 1024..3072 (symmetric, so only the sign flips) and the URDF is not changed.
- Tests first (red seen) in `tests/test_head_servo_config.py`: replace `test_head_tilt_servo_is_mounted_reversed_and_pan_is_not` with `test_head_pan_and_tilt_servos_are_mounted_reversed` (both `inverted: true`), plus `test_head_pan_steps_are_symmetric_around_centre` (min + max = 2 * 2048). `test_head_steps_equal_the_urdf_limits` and `test_head_startup_positions_are_inside_the_limits` keep passing with the inverted pan.
- Leave the `neutral_exclude_joints: [head_tilt]  # TEMP bench` line as it is (not this task).
- `tests/README.md` updated for the renamed and new tests.

## Acceptance conditions

- a1: head_pan and head_tilt are both inverted in the head group [evidence: test:tests/test_head_servo_config.py::test_head_pan_and_tilt_servos_are_mounted_reversed]
- a2: the pan step range is symmetric around 2048 so the inversion only flips the sign [evidence: test:tests/test_head_servo_config.py::test_head_pan_steps_are_symmetric_around_centre]
- a3: the URDF limits still equal the command steps with the inverted pan [evidence: test:tests/test_head_servo_config.py::test_head_steps_equal_the_urdf_limits]
- a4: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_head_servo_config.py -q && uv run poe lint && uv run poe test-ansible]

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
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
