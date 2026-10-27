---
title: Retire gripper camera in Ansible
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The gripper camera stops running once nothing reads it.
files: [ansible/group_vars/client.yml, tests/test_old_sensors_retired.py, tests/test_node_metrics_wiring.py, tests/test_client_cpu_load.py, tests/test_nav_stack_config.py, tests/README.md]
verify: "uv run pytest tests/test_old_sensors_retired.py tests/test_node_metrics_wiring.py tests/test_client_cpu_load.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [mcp-gripper-camera-config-re, web-ui-arm-tab-config-withou, arm-tab-without-gripper-came, steamdeck-camera-relay-remov, retire-arducam-tof-in-ansibl]
model: opus
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

# Retire gripper camera in Ansible

## Requirement

Goal G9: "... gripper UVC camera are retired (present: false) with their consumers ... relays removed". User decision 4: "retire now, in this epic (gripper_uvc_camera node, MCP 'gripper' camera, web_ui Arm-tab view, master2master steamdeck relay)". Design D11, D14 (frees 72 MiB).

- `ansible/group_vars/client.yml`: the `gripper_uvc_camera` instance is `present: false` (env and comments removed, retirement comment added). The master2master relay entry `/camera_0/image_raw/compressed -> /controller/camera_0/image_compressed` is removed.
- `tests/test_old_sensors_retired.py::test_gripper_camera_is_retired`: present false, and no tracked config (client.yml, nodes/*/config, roles templates) names `/camera_0`.
- `tests/test_node_metrics_wiring.py` (`ENV_METRICS_PORTS` gripper entry), `tests/test_client_cpu_load.py` (reads the gripper env) and `tests/test_nav_stack_config.py` (lists gripper_uvc_camera) are updated: those pins are no longer accurate (named in the commit). tests/README.md updated.

## Acceptance conditions

- AC1: gripper_uvc_camera is present: false and nothing names /camera_0 [evidence: test:tests/test_old_sensors_retired.py::test_gripper_camera_is_retired]
- AC2: the updated pins pass [evidence: command:uv run pytest tests/test_node_metrics_wiring.py tests/test_client_cpu_load.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_old_sensors_retired.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_client_cpu_load.py`
- `tests/test_nav_stack_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add the retirement test and update the three pins; run them; expect failures.
- Edit client.yml; rerun; expect pass.
- Run the verify command; commit `chore(ansible): retire the gripper UVC camera and its relay`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
