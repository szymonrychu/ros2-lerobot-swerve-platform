---
title: Enable RGB-D and ground nodes
status: done
repo: ros2-lerobot-sverve-platform
value: Ground estimation and the RGB-D stage run again on the robot.
files: [ansible/group_vars/client.yml, tests/test_head_rgbd_deploy.py, tests/test_ground_estimator_deploy.py, tests/test_cpp_cutover.py, tests/test_map3d_uplink_config.py, tests/test_unit_cgroups.py, tests/README.md]
verify: "uv run pytest tests/test_head_rgbd_deploy.py tests/test_ground_estimator_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [head-rgb-d-build-deps-cleanu, ground-estimator-on-d435-inp, laser-filter-on-the-d435-clo, d435-live-bring-up-and-topic]
model: opus
review: ""
verified_head: ff6a33f3
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: 3
actual_minutes: 34
actual_tokens: 5804882
actual_roles: "impl 26, review 8"
actual_role_tokens: "impl 5401683, review 403199"
---

# Enable RGB-D and ground nodes

## Requirement

Goal G3: "Ground estimation is restored on D435 depth: ground_estimator_cpp (and head_rgbd_cpp or its replacement) enabled ..." Design D1, D7, D14.

- `ansible/group_vars/client.yml`:
  - `head_rgbd_cpp` and `ground_estimator_cpp` instances `enabled: true`; the "disabled (user 2026-10-24) ..." comments are removed.
  - Their metrics targets 19117 / 19119 are added back to `monitoring_agent_node_targets` (the comment there said "add their targets when enabling").
  - The ground_estimator_cpp `config` block is rewritten to the keys of ground-estimator-on-d435-inp (cloud_topic `/head/d435/points_filtered`, guidance on `/head_rgbd/depth/*`, `depth_info_topic`, `margin_px`, the scaled frame windows); the header comment says D435.
- The enabled memory_low sum stays <= 2560 MiB: this task runs after arducam_tof and gripper_uvc_camera are present: false (they free 136 MiB).
- Tests: `tests/test_head_rgbd_deploy.py::test_rgbd_fusion_and_ground_nodes_are_disabled_not_removed` and `test_metrics_target_is_absent_while_disabled` are no longer accurate. They are replaced by `test_rgbd_and_ground_nodes_are_enabled` and `test_metrics_targets_are_present`. `tests/test_ground_estimator_deploy.py`, `tests/test_cpp_cutover.py` and `tests/test_map3d_uplink_config.py` assertions that pin the disabled state are updated the same way, each named in the commit message. `tests/test_unit_cgroups.py` passes unchanged. tests/README.md updated.

Amended 2026-10-28 (user: "the stream seems very stable, enable slope and obstacle estimation from the D435 and include it in map and path planning"): this task no longer waits for retire-gripper-camera-in-ans. Instead it sets the gripper_uvc_camera instance `enabled: false` (present stays true; retire-gripper-camera-in-ans makes it present: false later) so the enabled memory_low sum stays <= 2560 MiB with head_rgbd_cpp and ground_estimator_cpp enabled; the comment names the reason. Tests that pin gripper_uvc_camera as enabled are updated (named in the commit).

Amended again 2026-10-28: retire-arducam-tof-in-ansibl is blocked on mcp-depth-summary-on-d435-cl (mcp_server still reads /tof/points_filtered). So this task also sets the arducam_tof instance `enabled: false` (present stays true; retire-arducam-tof-in-ansibl makes it present: false after the MCP re-point) together with gripper_uvc_camera, so the enabled memory_low sum stays <= 2560 MiB. mcp_server's ToF depth summary has no data until mcp-depth-summary-on-d435-cl lands; the comment says so.

## Acceptance conditions

- AC1: head_rgbd_cpp and ground_estimator_cpp are enabled with their metrics targets [evidence: test:tests/test_head_rgbd_deploy.py::test_rgbd_and_ground_nodes_are_enabled]
- AC2: the deployed ground config reads the D435 cloud and the /head_rgbd guidance [evidence: test:tests/test_ground_estimator_deploy.py::test_deployed_config_reads_the_d435]
- AC3: the enabled memory_low sum stays within the cap [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]
- AC4: cut-over and uplink tests pass with the nodes enabled [evidence: command:uv run pytest tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py -q]
- AC5: Ansible checks pass [evidence: command:uv run poe test-ansible]
- AC-mem: gripper_uvc_camera and arducam_tof are enabled: false (present true) and the enabled memory_low sum is <= 2560 MiB [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_ground_estimator_deploy.py`
- `tests/test_cpp_cutover.py`
- `tests/test_map3d_uplink_config.py`
- `tests/test_unit_cgroups.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update the four deploy tests; run `uv run pytest tests/test_head_rgbd_deploy.py tests/test_ground_estimator_deploy.py tests/test_cpp_cutover.py tests/test_map3d_uplink_config.py -q`; expect failures.
- Edit client.yml; rerun with tests/test_unit_cgroups.py; expect pass.
- Run the verify command; commit `feat(ansible): enable head_rgbd_cpp and ground_estimator_cpp on the D435`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: ground-estimator-on-d435-inp fix round brought forward the client.yml ground_estimator_cpp config block (D435 keys) and the README config-key names to keep the root suite green; this task keeps the rest.

- 2026-10-28 amended: user 2026-10-28: enable ground estimation now; gripper camera disabled for memory instead of waiting for its retirement

- 2026-10-28 amended: ToF retire blocked on MCP re-point; disable arducam_tof for memory instead

- 2026-10-28 done
