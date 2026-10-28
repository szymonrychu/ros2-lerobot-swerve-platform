---
title: Board thickness in the pose check and BoardPark
status: todo
repo: ros2-lerobot-sverve-platform
value: Arm collision checks use the real 4 mm board
files: [shared/ros2_common/head_rgbd_calibration/arm_pose_check.py, shared/ros2_common/head_rgbd_calibration/config.py, nodes/mcp_server/mcp_server/config.py, tests/test_board_park_config.py, tests/test_arm_pose_check.py, ansible/group_vars/client.yml]
verify: "uv run pytest tests/test_board_park_config.py tests/test_arm_pose_check.py -q && (cd nodes/head_rgbd_calibration && uv run pytest tests -q) && (cd nodes/mcp_server && uv run pytest tests -q) && uv run poe lint"
depends_on: [head-arm-calibration-on-d435]
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 16
expected_tokens: 1374904
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Board thickness in the pose check and BoardPark

## Requirement

The calibration board is the 3D-printed 6x5 ChArUco, 4 mm thick (user 2026-10-28). The arm pose check (shared/ros2_common/head_rgbd_calibration/arm_pose_check.py PoseChecker board geometry) and mcp_server BoardPark model the board without that thickness; tests/test_board_park_config.py pins the two equal. Add the 4 mm thickness to both board models (same value in both, config not literal), so collision and clearance checks use the real board volume.

## Acceptance conditions

- a1: both board models carry the 4 mm thickness and stay equal [evidence: command:uv run pytest tests/test_board_park_config.py tests/test_arm_pose_check.py -q]
- a2: the shipped pose set still passes safety [evidence: command:cd nodes/head_rgbd_calibration && uv run pytest tests/test_arm_pose_set.py -q]

## Files

- `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `nodes/mcp_server/mcp_server/config.py`
- `tests/test_board_park_config.py`
- `tests/test_arm_pose_check.py`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
