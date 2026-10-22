---
title: MCP front camera applied
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The agent's front camera tools become calibrated: pixel rays use the applied intrinsics and the measured camera mount."
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/camera_scene.py, ansible/group_vars/client.yml, nodes/mcp_server/tests/test_camera_config.py, nodes/mcp_server/tests/test_camera_scene_head.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_config.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint"
depends_on: [mcp-head-kinematics-applied, arm-board-pose-set]
owns: []
uses: [head-chain-override]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--mcp-front-camera-applied
branch: drivable-ground-tof-filterin--mcp-front-camera-applied
verified_head: ""
run: ""
provides: []
---

# MCP front camera applied

## Requirement

The mcp_server front camera (head RGB) uses the applied head_camera_joint and the applied intrinsics (design D40; approval H4; today `cameras.front` duplicates head_camera_joint with null intrinsics, client.yml).

- `ansible/group_vars/client.yml` mcp_server `cameras.front`: `parent_frame: head_camera_link`, mount `parent_frame: head_camera_link` with x, y, z, roll, pitch, yaw all 0 (the applied `head_camera_joint` is then the mount, through the reloading HeadKinematics), intrinsics `calibration_file: /var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml`; the comment explains both.
- `nodes/mcp_server/mcp_server/config.py` (and `camera_scene.py` where the intrinsics are used): the front intrinsics file is read lazily at each camera call with an mtime cache, so an apply is picked up without a restart; a missing file means the camera tools answer 'not calibrated' (warning once), never a crash at startup. When the file's image size differs from the front image size with the same aspect ratio (within 0.5 %), fx and cx scale by the width ratio and fy and cy by the height ratio (distortion unchanged); a different aspect ratio keeps the front camera uncalibrated with a warning naming both sizes. Read the front stream size from the head_camera_cpp config in client.yml and pin it in the test.
- Tests: `tests/test_camera_config.py`: the client.yml front block validates (parent is a head URDF link); scaling from 576x324 to the front size doubles or scales K exactly; different aspect ratio -> uncalibrated; missing file -> uncalibrated. `tests/test_camera_scene_head.py`: the front camera pose with a zero mount on head_camera_link equals the old mount values at the URDF chain, and moves with an applied head_camera_joint.

## Acceptance conditions

- a1: applied intrinsics are scaled to the front image size [evidence: test:nodes/mcp_server/tests/test_camera_config.py::test_front_intrinsics_scaled_to_stream_size]
- a2: a missing or mismatched intrinsics file leaves the front camera uncalibrated [evidence: test:nodes/mcp_server/tests/test_camera_config.py::test_front_intrinsics_missing_or_mismatched_uncalibrated]
- a3: the front camera pose follows the applied head_camera_joint [evidence: test:nodes/mcp_server/tests/test_camera_scene_head.py::test_front_pose_follows_applied_head_camera_joint]
- a4: camera and config tests pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_config.py tests/test_camera_scene_head.py tests/test_config.py -q]
- a5: head consistency tests pass [evidence: command:uv run --frozen pytest tests/test_head_config_consistency.py -q]
- a6: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `ansible/group_vars/client.yml`
- `nodes/mcp_server/tests/test_camera_config.py`
- `nodes/mcp_server/tests/test_camera_scene_head.py`

## Interfaces

- client.yml `cameras.front` on `head_camera_link` with a zero mount and the applied intrinsics file; lazy intrinsics load with mtime cache and size scaling.

## Expected outcome

The agent's front camera tools become calibrated: pixel rays use the applied intrinsics and the measured camera mount.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_config.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path drivable-ground-tof-filterin design`) (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus). Ordered after arm-board-pose-set only because both edit ansible/group_vars/client.yml.

- 2026-10-22 re-plan: the floor fit epic delivered the override, the FK core and head_state_publisher; dependencies and design references updated to the head-from-arm plan.
