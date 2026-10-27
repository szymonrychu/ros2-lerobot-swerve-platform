---
title: Head chain overrides on d435_joint
status: todo
repo: ros2-lerobot-sverve-platform
value: A head calibration Apply moves the D435 frames every node uses.
files: [shared/ros2_common/head_rgbd_calibration/urdf_text.py, nodes/head_state_publisher/launch/head_description.py, tests/test_head_chain_override.py, tests/test_head_chain_launch.py]
verify: "uv run pytest tests/test_head_chain_override.py tests/test_head_chain_launch.py tests/test_head_state_publisher_config.py -q && uv run poe lint"
depends_on: [d435-frames-on-the-head-urdf]
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Head chain overrides on d435_joint

## Requirement

Goal G8: "Recalibration runs on the D435: head mount extrinsics through the pan/tilt chain, floor fit ..." Goal G2. Design D4: "HEAD_CHAIN_JOINTS (calibration overrides) gains d435_joint in place of head_camera_joint".

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`: `HEAD_CHAIN_JOINTS = ("head_mount_joint", "head_pan", "head_tilt", "d435_joint")`. `head_chain_json` and `apply_overrides` work on it. `TOF_JOINT` and the tof_joint.json handling stay until retire-tof-and-head-rgb-urdf removes them (head_description.py still applies an existing tof_joint.json to the old frames until then).
- `nodes/head_state_publisher/launch/head_description.py`: applies `head_chain.json` overrides to `d435_joint` (comment).
- Tests: `tests/test_head_chain_override.py` and `tests/test_head_chain_launch.py` use `d435_joint`; the head_camera_joint assertions are no longer accurate. New `test_head_chain_override.py::test_override_moves_the_d435_frames` checks that an override of d435_joint moves `d435_color_optical_frame` and `d435_depth_optical_frame` in the published URDF.

## Acceptance conditions

- AC1: the head chain overrides cover d435_joint and move every D435 frame [evidence: test:tests/test_head_chain_override.py::test_override_moves_the_d435_frames]
- AC2: head_state_publisher applies head_chain.json to the d435 joint [evidence: command:uv run pytest tests/test_head_chain_launch.py -q]
- AC3: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `nodes/head_state_publisher/launch/head_description.py`
- `tests/test_head_chain_override.py`
- `tests/test_head_chain_launch.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
