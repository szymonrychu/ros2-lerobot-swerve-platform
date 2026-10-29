---
title: Web UI arm tab config without gripper
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The web UI stops serving a camera that is gone.
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_config.py, nodes/web_ui/tests/test_bridge_on_demand.py, nodes/web_ui/tests/test_map3d_backend.py, nodes/web_ui/tests/test_msg_serializer.py]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible"
depends_on: [web-ui-obstacles-topic-on-d4, mcp-gripper-camera-config-re]
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

# Web UI arm tab config without gripper

## Requirement

Goal G9 / user decision 4: retire "... web_ui Arm-tab view ...". Design D12: "Arm tab: the gripper camera view, the 'gripper' 3D camera mode and gripper_camera_mount are removed."

Backend half (frontend in arm-tab-without-gripper-came):
- `web_ui/config.py` `ARM_TAB_DEFAULTS` and the arm tab model: `gripper_camera_topic` and `gripper_camera_mount` removed (rejected as unknown keys). `head_camera_topic` default `/head/d435/color/image_raw/compressed`.
- `web_ui/bridge.py`: the `/camera_0/image_raw[/compressed]` type hints are removed; the head camera hint names the D435 topic.
- `config/default.yaml` and the client.yml arm tab: gripper lines removed; `head_camera_topic: /head/d435/color/image_raw/compressed`; the comment no longer mentions mcp_server's gripper mount.
- Tests: `tests/test_config.py` (keys rejected, D435 head topic), `tests/test_bridge_on_demand.py`, `tests/test_map3d_backend.py` and `tests/test_msg_serializer.py`. Their `/camera_0` cases are no longer accurate and become the D435 colour topic.

## Acceptance conditions

- AC1: the arm tab has no gripper camera topic or mount and its head camera is the D435 colour [evidence: test:nodes/web_ui/tests/test_config.py::test_arm_tab_has_no_gripper_camera]
- AC2: no bridge type hint names /camera_0 [evidence: test:nodes/web_ui/tests/test_bridge_on_demand.py::test_no_gripper_camera_type_hint]
- AC3: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_bridge_on_demand.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `nodes/web_ui/tests/test_msg_serializer.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the two tests and convert /camera_0 cases; run the four test files; expect failures.
- Remove the fields and hints, update default.yaml and client.yml; rerun the web_ui suite; expect pass.
- Run the verify command; commit `refactor(web_ui): arm tab backend without the gripper camera`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-29: client.yml arm-tab comment (~line 2591) says "a test pins them equal" but that test was deleted by mcp-gripper-camera-config-re; fix the comment here.
