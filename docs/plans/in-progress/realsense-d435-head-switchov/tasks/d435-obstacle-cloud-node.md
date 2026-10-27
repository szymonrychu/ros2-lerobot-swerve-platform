---
title: D435 obstacle cloud node
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The robot gets a 3D obstacle cloud from the D435 depth, the input of the self-filter, Nav2 and ground estimation."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml, nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml, tests/test_tof_obstacle_cloud_config.py, tests/test_d435_obstacle_cloud_config.py, tests/test_ansible_deploy_speed.py, tests/README.md]
verify: "uv run pytest tests/test_d435_obstacle_cloud_config.py tests/test_ansible_deploy_speed.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [d435-node-type-and-profile, d435-frames-on-the-head-urdf]
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

# D435 obstacle cloud node

## Requirement

Goal G3 (README): "Ground estimation is restored on D435 depth: ... obstacle cloud and self-filter re-pointed ..." Design D5.

- `ansible/group_vars/client.yml`: new node type `d435_obstacle_cloud` (copy of the tof_obstacle_cloud type): `ros2 run rtabmap_util point_cloud_xyz --ros-args --params-file {{ ros2_repo_dest }}/nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml -r depth/image:=/head/d435/depth/image_rect_raw -r depth/camera_info:=/head/d435/depth/camera_info -r cloud:=/head/d435/points -r __node:=d435_obstacle_cloud`, `src_paths: [nodes/d435_obstacle_cloud]`. New instance `d435_obstacle_cloud` present/enabled. The `tof_obstacle_cloud` instance becomes `present: false` (uninstall), and its node type stays only as the uninstall stub (deploy_mode + launch command, as the overview_camera stub).
- `git mv nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml`, re-derived for the D435 depth (640x360, min-Z 0.15 m): `decimation` so the decimated image is at most 160x90, `min_depth: 0.15`, `max_depth: 3.0`, `voxel_size: 0.05`, the radius noise filter kept, `approx_sync: false`. Each changed value has a comment with its reason.
- `ansible/playbooks/deploy_nodes_client.yml`: a `Deploy d435_obstacle_cloud` step (same shape as tof_obstacle_cloud's); the tof_obstacle_cloud step stays for the uninstall.
- `git mv tests/test_tof_obstacle_cloud_config.py tests/test_d435_obstacle_cloud_config.py` and rewrite it for the D435 inputs (the ToF assertions are no longer accurate): remaps, params file values, instance enabled, old instance present: false. `tests/test_ansible_deploy_speed.py` FOLDED_LAUNCH_COMMANDS gains the new line (the old one stays pinned while the stub exists). tests/README.md updated.
- No placeholder data: point_cloud_xyz publishes only from real depth frames (no change needed; the test pins no `publish_empty`-style option).
Notes: nodes/tof_obstacle_cloud/README.md is moved by node-readmes-and-diagrams-on. Check the realsense depth QoS against point_cloud_xyz's subscription (the bring-up topic list) and set `qos` in the params file if they differ.

## Acceptance conditions

- AC1: d435_obstacle_cloud turns /head/d435/depth/image_rect_raw into /head/d435/points [evidence: test:tests/test_d435_obstacle_cloud_config.py::test_remaps_read_the_d435_depth]
- AC2: the params are sized for 640x360 depth with min depth 0.15 m and max 3 m [evidence: test:tests/test_d435_obstacle_cloud_config.py::test_params_fit_the_d435_profile]
- AC3: tof_obstacle_cloud is present: false and d435_obstacle_cloud is enabled [evidence: test:tests/test_d435_obstacle_cloud_config.py::test_old_instance_uninstalls_new_enabled]
- AC4: Ansible checks and the deploy-speed pins pass [evidence: command:uv run pytest tests/test_ansible_deploy_speed.py tests/test_unit_cgroups.py -q && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`
- `nodes/d435_obstacle_cloud/config/point_cloud_xyz.yaml`
- `tests/test_tof_obstacle_cloud_config.py`
- `tests/test_d435_obstacle_cloud_config.py`
- `tests/test_ansible_deploy_speed.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- `git mv` the params file and the test; rewrite the test for the D435; run `uv run pytest tests/test_d435_obstacle_cloud_config.py -q`; expect failures.
- Add the node type, instance, playbook step and new params; rerun; expect pass.
- Run the verify command; commit `feat(ansible): d435_obstacle_cloud from the D435 depth stream`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
