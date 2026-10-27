---
title: D435 node type and profile
status: todo
repo: ros2-lerobot-sverve-platform
value: "The D435 starts on the client from Ansible with a hardware-synced, Pi-sized profile and explicit names."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, nodes/bridges/realsense_d435/config/realsense_d435.yaml, tests/test_realsense_d435_deploy.py, tests/test_ansible_deploy_speed.py, tests/test_unit_cgroups.py, tests/README.md]
verify: "uv run pytest tests/test_realsense_d435_deploy.py tests/test_ansible_deploy_speed.py tests/test_unit_cgroups.py tests/test_web_ui_agent_tab.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: []
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

# D435 node type and profile

## Requirement

Goal G1 (README): "The D435 (serial 935723024187) runs on the client as the realsense2_camera node, deployed by Ansible, with udev for 0b07, a matched-fps profile, explicit camera_name/namespace and the live topic list pinned in config." Design D2, D3, D14.

- Replace the `realsense_d435i` node type and instance in `ansible/group_vars/client.yml` with `realsense_d435` (no backward compatibility: the old entry described a D435i with an IMU and is removed, not kept as a stub; it has been present: false since 4c6bf6cf).
- `node_launch_command`: `ros2 launch realsense2_camera rs_launch.py camera_namespace:=head camera_name:=d435 serial_no:=_935723024187 config_file:={{ ros2_repo_dest }}/nodes/bridges/realsense_d435/config/realsense_d435.yaml`. No `enable_accel`/`enable_gyro` (D435 has no IMU). `src_paths: [nodes/bridges/realsense_d435]`.
- New `nodes/bridges/realsense_d435/config/realsense_d435.yaml` (realsense2_camera parameters, commented per key): `depth_module.depth_profile: 640x360x15`, `rgb_camera.color_profile: 640x360x15` (one fps: RGB-depth hardware sync, datasheet Table 4-6), `align_depth.enable: true`, `pointcloud.enable: false`, `enable_infra1: false`, `enable_infra2: false`, `enable_color: true`, `enable_depth: true`, `publish_tf: false` (head.urdf owns the d435 frames, D4).
- Instance `realsense_d435`: `present: true`, `enabled: true`, in the NORMAL tier (memory_low 128M, memory_max 512M estimates until d435-memory-and-cpu-budget measures them; comment says so). The enabled memory_low sum must stay <= the 2560 MiB cap of tests/test_unit_cgroups.py.
- The playbook step `Deploy realsense_d435i` in `ansible/playbooks/deploy_nodes_client.yml` is renamed to `realsense_d435` (tags and `_deploy_node_name`).
- Tests: new `tests/test_realsense_d435_deploy.py` pins the launch arguments, the YAML profile (equal fps of depth and colour, align on, pointcloud/infra off, publish_tf false), the instance and the playbook step. `tests/test_ansible_deploy_speed.py` FOLDED_LAUNCH_COMMANDS entry is renamed and updated (the old one pins a launch line that no longer exists: no longer accurate). `tests/test_unit_cgroups.py` NORMAL tier list names `realsense_d435` (the old name is no longer accurate). tests/README.md documents every new test.

Notes: CLAUDE.md prefers file config over CLI: only the camera identity (namespace, name, serial, config_file) is a launch argument because rs_launch.py builds the node name from them. The bridge README is moved by top-level-docs-and-memory-on.

## Acceptance conditions

- AC1: the launch line names camera_namespace head, camera_name d435, the serial and the config file, and has no accel/gyro args [evidence: test:tests/test_realsense_d435_deploy.py::test_launch_command_names_the_camera_and_config_file]
- AC2: depth and colour profiles share one fps, align on, pointcloud and infra off, publish_tf false [evidence: test:tests/test_realsense_d435_deploy.py::test_profile_is_hardware_synced_and_minimal]
- AC3: the realsense_d435 instance is present and enabled in the normal tier, and no realsense_d435i entry remains [evidence: test:tests/test_realsense_d435_deploy.py::test_instance_enabled_and_old_name_gone]
- AC4: the enabled memory_low sum stays within the cap [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]
- AC5: Ansible lint and syntax check pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `nodes/bridges/realsense_d435/config/realsense_d435.yaml`
- `tests/test_realsense_d435_deploy.py`
- `tests/test_ansible_deploy_speed.py`
- `tests/test_unit_cgroups.py`
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
