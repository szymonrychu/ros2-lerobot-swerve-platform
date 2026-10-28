---
title: Head-arm calibration on D435
status: done
repo: ros2-lerobot-sverve-platform
value: "The D435 mount is solved from the trusted arm holding the board, with every pose in the D435's range."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py, nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py, shared/ros2_common/head_rgbd_calibration/arm_pose_check.py, nodes/head_rgbd_calibration/tests/test_head_arm_results.py, nodes/head_rgbd_calibration/tests/test_sequencer.py, nodes/head_rgbd_calibration/tests/test_arm_pose_set.py, tests/test_arm_pose_check.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_arm_pose_check.py -q && uv run poe lint"
depends_on: [calibration-config-on-d435, calibration-library-d435-fra]
model: opus
review: ""
verified_head: 27e3bb63
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: 5
actual_minutes: 45
actual_tokens: 20728276
actual_roles: "impl 38, review 7"
actual_role_tokens: "impl 20380176, review 348100"
---

# Head-arm calibration on D435

## Requirement

Goal G8: "... head mount extrinsics through the pan/tilt chain ...". User decision 1: "Extrinsics solved per head pose through FK." Design D10. Prior-art fix-as-ac: "the gripper is kept beyond the D435 MinZ (about 17 cm at 848x480) in the head-view poses used for pose estimation" (150 mm at 640x360, D2).

- `head_rgbd_calibration/head_arm_results.py`: `fit_head_from_arm(..., camera_frame=cfg.camera_frame)` on D435 colour stills with the factory intrinsics (the CameraInfo recorded with each still). Results write `head_chain.json` (d435_joint and the chain) and no tof_joint.json.
- `head_rgbd_calibration/sequencer.py` and `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`: every board pose in the sequence keeps every board corner at >= 0.20 m from `d435_color_optical_frame` (0.15 m min-Z + 0.05 m margin) and inside the colour FOV; poses that fail are skipped with the reason "inside D435 min-Z" or "outside colour FOV". The tier and confirm structure for big joints is unchanged (servo safety).
- Tests: `tests/test_head_arm_results.py` (D435 frame, no tof_joint), `tests/test_sequencer.py` (min-Z skip), `tests/test_arm_pose_set.py`, root `tests/test_arm_pose_check.py` (min-Z rule; head_camera_link references are no longer accurate and become D435 frames).

## Acceptance conditions

- AC1: head-from-arm results fit d435_joint on D435 colour stills and write no tof_joint [evidence: test:nodes/head_rgbd_calibration/tests/test_head_arm_results.py::test_result_fits_the_d435_joint]
- AC2: board poses nearer than 0.20 m to the D435 or outside its colour FOV are skipped with the reason [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_poses_inside_min_z_are_skipped]
- AC3: the pose check enforces the min-Z rule [evidence: test:tests/test_arm_pose_check.py::test_board_beyond_d435_min_z]
- AC4: node suite and root lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py`
- `nodes/head_rgbd_calibration/tests/test_head_arm_results.py`
- `nodes/head_rgbd_calibration/tests/test_sequencer.py`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `tests/test_arm_pose_check.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the three tests and update the stale assertions; run them; expect failures.
- Change results, sequencer and the pose check; rerun the node suite and tests/test_arm_pose_check.py; expect pass.
- Run the verify command; commit `feat(head_rgbd_calibration): head-from-arm on the D435 with a min-Z rule`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28 (from calibration-config-on-d435): head-arm stills store no CameraInfo and read head_camera_calibration.yaml, which is no longer written; head-arm must take the D435 factory intrinsics from CameraInfo stored with each still (CameraIntrinsics.from_camera_info).
- 2026-10-28 (user): the board is the same 3D-printed 6x5 ChArUco (same dimensions as before), 4 mm thick; it is in front of the robot and the D435 head is mounted. Account for the 4 mm thickness where the board surface vs mount face matters (plate_mount / tool_in_board_m, board-plane depth bias).

- 2026-10-28 done
