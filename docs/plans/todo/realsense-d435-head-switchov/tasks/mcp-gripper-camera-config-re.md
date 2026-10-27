---
title: MCP gripper camera config removed
status: todo
repo: ros2-lerobot-sverve-platform
value: No MCP config or calibration file refers to the retired gripper camera.
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/calibration/gripper_camera.yaml, ansible/group_vars/client.yml, nodes/mcp_server/tests/test_config.py, nodes/mcp_server/tests/test_on_demand_subscriptions.py, tests/test_mcp_server_config.py, tests/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py tests/test_on_demand_subscriptions.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py tests/test_web_ui_agent_tab.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [mcp-approach-tools-front-onl, mcp-grasp-check-on-d435, mcp-zoom-retired, web-ui-arm-tab-config-withou]
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

# MCP gripper camera config removed

## Requirement

Goal G9 / user decision 4: "Gripper UVC camera: retire now, in this epic (gripper_uvc_camera node, MCP 'gripper' camera, web_ui Arm-tab view, master2master steamdeck relay)". Design D8, D12.

- `mcp_server/config.py`: `Topics.gripper_camera` and `CamerasSettings.gripper` removed (no backward compatibility; config with them is rejected as an unknown key).
- `mcp_server/ros_iface.py`: `camera_topics` is `{"front": topics.head_camera}`.
- `ansible/group_vars/client.yml` mcp_server block: the `cameras.gripper` block, its comments and the `topics.gripper_camera` line are removed; the zoom_* lines too if still present.
- `nodes/mcp_server/calibration/gripper_camera.yaml` deleted (the 2026-10-10 gripper intrinsics; the session record stays under docs/calibration/2026-10-10/).
- Tests: `tests/test_config.py` and `tests/test_on_demand_subscriptions.py` drop the gripper topic (no longer accurate). Root `tests/test_mcp_server_config.py`: `test_web_ui_arm_tab_mount_equals_mcp_server_gripper_camera_mount` is deleted (both mounts are retired; no longer accurate), and gripper-camera pins are removed. New `test_mcp_server_has_no_gripper_camera`. `tests/README.md` updated.

## Acceptance conditions

- AC1: mcp_server config has no gripper camera topic or camera settings [evidence: test:tests/test_mcp_server_config.py::test_mcp_server_has_no_gripper_camera]
- AC2: config with a gripper camera key is rejected [evidence: test:nodes/mcp_server/tests/test_config.py::test_gripper_camera_key_is_rejected]
- AC3: the gripper intrinsics file is no longer tracked [evidence: test:nodes/mcp_server/tests/test_config.py::test_gripper_calibration_file_is_gone]
- AC4: suites and Ansible checks pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py tests/test_on_demand_subscriptions.py -q && cd ../.. && uv run poe test-ansible]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/calibration/gripper_camera.yaml`
- `ansible/group_vars/client.yml`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `tests/test_mcp_server_config.py`
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
