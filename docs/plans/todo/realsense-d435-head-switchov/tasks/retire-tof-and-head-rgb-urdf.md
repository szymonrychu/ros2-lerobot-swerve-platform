---
title: Retire ToF and head RGB URDF frames
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head model contains only the D435, so no node can resolve a frame of a sensor that is gone."
files: [nodes/web_ui/urdf/head.urdf, shared/ros2_common/head_rgbd_calibration/urdf_text.py, nodes/head_state_publisher/launch/head_description.py, nodes/head_state_publisher/launch/head_state_publisher.launch.py, tests/test_head_tof_joint_override.py, tests/test_head_chain_launch.py, nodes/web_ui/tests/test_head_urdf.py, tests/test_tof_paths_retired.py]
verify: "uv run pytest tests/test_tof_paths_retired.py tests/test_head_chain_launch.py tests/test_head_chain_override.py tests/test_head_state_publisher_config.py -q && uv run poe lint && cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q"
depends_on: [retire-tof-stereo-calibratio, mcp-head-kinematics-on-d435, marker-detector-on-d435, rtab-map-sources-from-config, map-3d-obstacles-layer-on-d4, web-ui-arm-tab-config-withou, mcp-depth-summary-on-d435-cl, delete-head-camera-cpp-sourc, delete-arducam-tof-source]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Retire ToF and head RGB URDF frames

## Requirement

Goal G2: "... replacing the ToF and head RGB frames." Design D4: "The old head frames stay in the URDF until their last consumer moves, and are then removed by retire-tof-and-head-rgb-urdf."

- `nodes/web_ui/urdf/head.urdf`: `head_camera_joint`/`head_camera_link`/`head_camera_optical_joint`/`head_camera_optical_frame` and `tof_joint`/`tof_link`/`tof_optical_joint`/`tof_optical_frame` are removed, with their visuals.
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`: `TOF_JOINT`, the `tof_joint.json` file name and `rewrite_tof_joint` are removed. `nodes/head_state_publisher/launch/head_description.py` and `head_state_publisher.launch.py` no longer read `tof_joint.json` (existing files on the robot are ignored, not deleted).
- `tests/test_head_tof_joint_override.py` is deleted (its feature is removed). `tests/test_head_chain_launch.py` drops the tof_joint cases. `nodes/web_ui/tests/test_head_urdf.py` gains `test_no_tof_or_head_camera_frames`. A new `tests/test_tof_paths_retired.py::test_no_tracked_code_names_tof_or_head_camera_frames` asserts that no tracked .py/.cpp/.hpp/.ts/.tsx/.yaml/.urdf file outside docs/ names `tof_optical_frame`, `tof_link`, `head_camera_optical_frame` or `head_camera_link`. tests/README.md updated.
TDD exception for the deletions: pinned by the retirement tests.

## Acceptance conditions

- AC1: head.urdf has no ToF or head camera frame [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_no_tof_or_head_camera_frames]
- AC2: no tracked code or config names the old frames [evidence: test:tests/test_tof_paths_retired.py::test_no_tracked_code_names_tof_or_head_camera_frames]
- AC3: head chain launch and override tests pass [evidence: command:uv run pytest tests/test_head_chain_launch.py tests/test_head_chain_override.py tests/test_head_state_publisher_config.py -q]

## Files

- `nodes/web_ui/urdf/head.urdf`
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `nodes/head_state_publisher/launch/head_description.py`
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `tests/test_head_tof_joint_override.py`
- `tests/test_head_chain_launch.py`
- `nodes/web_ui/tests/test_head_urdf.py`
- `tests/test_tof_paths_retired.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
