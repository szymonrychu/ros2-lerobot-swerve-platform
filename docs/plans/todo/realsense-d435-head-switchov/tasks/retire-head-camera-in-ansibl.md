---
title: Retire head camera in Ansible
status: todo
repo: ros2-lerobot-sverve-platform
value: The Camera Module 3 stops running once nothing reads it.
files: [ansible/group_vars/client.yml, tests/test_old_sensors_retired.py, tests/test_head_camera_deploy.py, tests/test_cpp_cutover.py, tests/test_map3d_uplink_config.py, tests/README.md]
verify: "uv run pytest tests/test_old_sensors_retired.py tests/test_head_camera_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [mcp-front-camera-on-d435, mcp-zoom-retired, web-ui-head-tab-config-on-d4, web-ui-zoom-route-retired, head-tab-on-d435, marker-detector-zoom-retired, calibration-apply-restart-un, head-rgb-d-build-deps-cleanu, retire-gripper-camera-in-ans]
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

# Retire head camera in Ansible

## Requirement

Goal G9: "The ... head RGB (head_camera_cpp, Camera Module 3) ... retired (present: false) with their consumers". User decision 2. Design D11.

- `ansible/group_vars/client.yml`: the `head_camera_cpp` instance is `present: false` (config removed, retirement comment added). Its metrics target 19116 is removed.
- `tests/test_old_sensors_retired.py::test_head_camera_is_retired`: present false, no metrics target, and no enabled consumer names `/head_camera/`.
- `tests/test_head_camera_deploy.py`: `test_entry_is_live_after_pi_cameras_with_valid_config`, `test_camera_info_file_is_the_applied_file` and `test_monitoring_target_is_present` are no longer accurate and are removed. The node-type tests stay until delete-head-camera-cpp-sourc. `tests/test_cpp_cutover.py` and `tests/test_map3d_uplink_config.py` pins of head_camera_cpp as live are updated (named in the commit). tests/README.md updated.

## Acceptance conditions

- AC1: head_camera_cpp is present: false, unscraped, and no enabled consumer reads /head_camera [evidence: test:tests/test_old_sensors_retired.py::test_head_camera_is_retired]
- AC2: the updated pins pass [evidence: command:uv run pytest tests/test_head_camera_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_old_sensors_retired.py`
- `tests/test_head_camera_deploy.py`
- `tests/test_cpp_cutover.py`
- `tests/test_map3d_uplink_config.py`
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
