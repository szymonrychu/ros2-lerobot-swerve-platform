---
title: MCP front camera on head
status: todo
repo: ros2-lerobot-sverve-platform
value: Pixel-to-ground and calibration for the head camera use where the head is actually pointing.
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/camera_scene.py, nodes/mcp_server/mcp_server/camera_tools.py, nodes/mcp_server/tests/test_camera_scene_head.py, ansible/group_vars/client.yml, tests/test_mcp_camera_config.py, tests/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_camera_config.py tests/test_mcp_server_config.py -q && uv run poe lint"
depends_on: [mcp-head-controller, web-ui-head-command-backend]
owns: [front-camera-head-scene]
uses: [head-urdf, head-kinematics, head-controller]
model: opus
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--mcp-front-camera-on-head
branch: pan-tilt-head-control-with-a--mcp-front-camera-on-head
verified_head: ""
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: []
process_gaps: []
---

# MCP front camera on head

## Requirement

- `config.py` (D11): `FRONT_CAMERA_PARENT = "base_link"` and its check are replaced: `cameras.front.parent_frame` defaults to `head_link` and must be a link of the head URDF (`head.urdf_path`); a mount, when given, must agree with it (existing rule).
- `camera_scene.py`: `parent_transform(cfg, camera, kin, joints, head=None)` (or an equivalent explicit head argument) returns, for `front`, the pose of `cameras.front.parent_frame` in base_link from `HeadKinematics.link_pose` at the measured head angles; stale or missing head joints raise a ValueError naming the head joint states (never an identity fallback). The module docstring states the front camera now moves with the head.
- `camera_tools.py`: `scene_for` and the calibration sample path pass the measured head angles for `front` (the stored sample's t_parent is the head parent pose).
- `ansible/group_vars/client.yml` mcp_server config: `cameras.front.parent_frame: head_link`, `mount` = the URDF `head_link -> head_camera_link` origin (x 0.02, y -0.025, z 0, rpy 0), intrinsics stay null (not calibrated); the `head:` section with the deploy values (same as the code defaults).
- Root `tests/test_mcp_camera_config.py`: the front pin becomes front = head_link with the URDF mount and null intrinsics; a new test checks `cameras.front.parent_frame` is a link of head.urdf and the mount equals the URDF joint origin. `tests/README.md` rows follow.
- Tests first in new `nodes/mcp_server/tests/test_camera_scene_head.py`, seen failing: front scene at pan 0/tilt 0 equals the URDF camera pose; at pan +30 deg the optical axis yaws left 30 deg; at tilt +30 deg a centre pixel hits the floor about 0.6 m ahead (ground point); stale head joints raise; config rejects a front parent that is not a head URDF link.

## Acceptance conditions

- a1: the front camera scene follows the measured head pose and refuses stale head joints [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene_head.py -q]
- a2: cameras.front must be on a head URDF link; base_link-only validation is gone [evidence: command: ! grep -n 'FRONT_CAMERA_PARENT = "base_link"' nodes/mcp_server/mcp_server/config.py]
- a3: client.yml sets cameras.front on head_link with the URDF mount and a head section [evidence: test: tests/test_mcp_camera_config.py]
- a4: mcp_server and root camera config tests pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_camera_config.py tests/test_mcp_server_config.py -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/tests/test_camera_scene_head.py`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_camera_config.py`
- `tests/README.md`

## Interfaces

- `config.py:CamerasSettings` front parent validation; `camera_scene.py:parent_transform` (part `front-camera-head-scene`); `camera_tools.py` scene/sample paths.

## Expected outcome

Front camera geometry follows the measured head pose; client.yml mounts the camera on head_link.

## Steps

- Write `tests/test_camera_scene_head.py` and update `tests/test_mcp_camera_config.py`; run them; expect failures.
- Implement; run the verify command; commit `feat(mcp_server): front camera pose follows the head`.

## Notes

- The gripper camera path (`kin.link_frame`) is the model for a moving parent.
- Intrinsics stay null: tools keep saying not calibrated until a calibration exists.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-head-controller, web-ui-head-command-backend
