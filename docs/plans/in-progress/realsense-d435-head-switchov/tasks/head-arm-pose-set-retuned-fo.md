---
title: Head-arm pose set retuned for the D435 view
status: todo
repo: ros2-lerobot-sverve-platform
value: Every calibration pose yields usable D435 views
files: [nodes/head_rgbd_calibration/config, nodes/head_rgbd_calibration/tests/test_arm_pose_set.py, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/README.md]
verify: "(cd nodes/head_rgbd_calibration && uv run pytest tests -q && uv run poe lint) && uv run pytest tests/test_arm_pose_check.py -q"
depends_on: [board-thickness-in-the-pose]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 25
expected_tokens: 969224
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Head-arm pose set retuned for the D435 view

## Requirement

With the D435 colour FOV and min-Z view rule (head-arm-calibration-on-d435), shipped head-arm tier 1 pose 8 loses all four views and the +6 deg tilt offset fails on most poses, although coverage limits still pass. Retune the shipped head-arm pose set (head aim per pose, and if needed arm poses within the existing safe envelope) so every pose keeps at least the configured number of usable views under nominal D435 intrinsics, coverage limits pass with margin, and every pose still passes the PoseChecker safety rules. No new arm poses outside the current envelope; offline only, no servo motion.

## Acceptance conditions

- a1: every shipped pose keeps the minimum usable views under the D435 FOV and min-Z rule [evidence: command:cd nodes/head_rgbd_calibration && uv run pytest tests/test_arm_pose_set.py -q]
- a2: safety and coverage tests pass [evidence: command:uv run pytest tests/test_arm_pose_check.py -q]

## Files

- `nodes/head_rgbd_calibration/config`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
