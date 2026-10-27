---
title: MCP front camera on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent's front camera is the D435 colour, posed through the pan/tilt chain."
files: [ansible/group_vars/client.yml, nodes/mcp_server/mcp_server/config.py, tests/test_mcp_camera_config.py, tests/test_mcp_server_config.py, nodes/mcp_server/tests/test_camera_scene_head.py, nodes/mcp_server/README.md, tests/README.md]
verify: "uv run pytest tests/test_mcp_camera_config.py tests/test_mcp_server_config.py tests/test_head_camera_live_refs.py tests/test_web_ui_agent_tab.py -q && uv run poe lint && uv run poe test-ansible && cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene_head.py -q && uv run poe lint"
depends_on: [mcp-intrinsics-from-camera-i, d435-frames-on-the-head-urdf, d435-live-bring-up-and-topic]
model: sonnet
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

# MCP front camera on D435

## Requirement

Goal G7. Design D8: "topics.head_camera: /head/d435/color/image_raw/compressed. cameras.front hangs on d435_color_frame with a zero mount (FK through head.urdf, D4), and its intrinsics come from /head/d435/color/camera_info".

- `ansible/group_vars/client.yml` mcp_server block: `topics.head_camera: /head/d435/color/image_raw/compressed` (comment: D435 colour 640x360 at 15 fps; the key name stays because tests/test_head_camera_live_refs.py forbids `front_camera` names); `cameras.front.parent_frame: d435_color_frame`, mount zero on `d435_color_frame`, `intrinsics: {camera_info_topic: /head/d435/color/camera_info}`. The comment block above `front` is rewritten for the D435.
- `nodes/mcp_server/mcp_server/config.py` defaults for `topics.head_camera` and `cameras.front` match.
- Tests: `tests/test_mcp_camera_config.py` and `tests/test_mcp_server_config.py` assertions pinning `/head_camera/image_raw/compressed`, `head_camera_link` or the applied head_camera_calibration.yaml are no longer accurate and are updated to the D435 values, each named in the commit. New `tests/test_mcp_camera_config.py::test_front_camera_is_the_d435_colour` (topic, parent frame is a link of head.urdf, camera_info intrinsics). `nodes/mcp_server/tests/test_camera_scene_head.py` uses `d435_color_frame`. tests/README.md and `nodes/mcp_server/README.md` ("Camera tools and calibration") updated.

## Acceptance conditions

- AC1: the front camera is the D435 colour stream on d435_color_frame with camera_info intrinsics [evidence: test:tests/test_mcp_camera_config.py::test_front_camera_is_the_d435_colour]
- AC2: the front camera pose follows the head angles through d435_color_frame [evidence: test:nodes/mcp_server/tests/test_camera_scene_head.py::test_front_pose_follows_the_head_through_the_d435_frame]
- AC3: mcp_server config pins pass [evidence: command:uv run pytest tests/test_mcp_server_config.py tests/test_head_camera_live_refs.py -q]
- AC4: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `nodes/mcp_server/mcp_server/config.py`
- `tests/test_mcp_camera_config.py`
- `tests/test_mcp_server_config.py`
- `nodes/mcp_server/tests/test_camera_scene_head.py`
- `nodes/mcp_server/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
