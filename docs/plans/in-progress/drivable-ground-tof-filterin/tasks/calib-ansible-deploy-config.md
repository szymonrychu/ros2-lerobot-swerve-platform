---
title: Calib ansible deploy config
status: done
repo: ros2-lerobot-sverve-platform
value: "The deployed node can call mcp_server, and its copies of the arm gauge, offsets, URDF and body obstacles cannot drift from mcp_server's."
files: [ansible/group_vars/client.yml, ansible/playbooks/tasks/head_rgbd_calibration_setup.yml, tests/test_head_rgbd_calibration_deploy.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [head-arm-sequence-http-api, web-ui-applied-head-urdf]
owns: []
uses: [arm-sequence-config]
model: ""
estimate: 45
actual: ""
review: ""
branch: main
verified_head: ""
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: []
---

# Calib ansible deploy config

## Requirement

Ansible keeps the node's permissions and its mirrored values in sync (CLAUDE.md Ansible rule; design D37, D39; approval
H2).

- The node's service gets group `mcp-token` so it can read `/etc/ros2/mcp_server/token`: `supplementary_groups:
  [mcp-token]` on the `head_rgbd_calibration` ros2_nodes entry in `ansible/group_vars/client.yml`, the mechanism
  claude_agent uses (its entry's `supplementary_groups: - mcp-token`, group created by
  `playbooks/tasks/mcp_server_setup.yml`). `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml` changes only if
  the role needs it (for example ordering after mcp_server_setup so the group exists). `restart_units` and the apply
  service stay as they are.
- `tests/test_head_rgbd_calibration_deploy.py` gains, all read from client.yml: `fit.joint_offsets_rad` equals the
  mcp_server `arm.joint_offsets_rad`; `fit.base_in_base_link` equals `arm.base_in_base_link`;
  `arm_sequence.pose_check.body` equals the mcp_server `body` boxes and cylinders; `arm_urdf_file` names the same URDF
  file mcp_server's arm config uses; `fit.board` equals `board`; the entry has `supplementary_groups` containing
  `mcp-token` and the group is created by `mcp_server_setup.yml`. `test_ros2_nodes_entry_matches_defaults` passes.
- `uv run poe lint-ansible` and `uv run poe test-ansible` pass; any new skip or warn is documented in
  `ansible/.ansible-lint`.

## Acceptance conditions

- a1: the mirrored arm gauge and offsets equal mcp_server's [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_fit_gauge_equals_mcp_server_arm]
- a2: the pose-check body equals mcp_server's body [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_pose_check_body_equals_mcp_server_body]
- a3: the node reads the mcp token through its group [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_node_in_mcp_token_group]
- a4: the config block equals the shipped YAML [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a5: ansible lint and syntax checks pass [evidence: command:uv run poe lint-ansible && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `tests/test_head_rgbd_calibration_deploy.py`

## Interfaces

- client.yml `head_rgbd_calibration` entry `supplementary_groups`; the mirrored keys above.

## Expected outcome

The deployed node can call mcp_server, and its copies of the arm gauge, offsets, URDF and body obstacles cannot drift
from mcp_server's.

## Steps

- Write the deploy tests first; run them: red on the group and any mismatch.
- Update client.yml (and the setup tasks only if needed); run
  `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint-ansible && uv run poe test-ansible`;
  expect every test passed and lint clean.

## Notes

- The node config keys themselves were added to client.yml by the tasks that added them; this task adds the group and
  the cross-config pins.
- The dependency on web-ui-applied-head-urdf is only file ordering: both edit `ansible/group_vars/client.yml`
  (web_ui `head_applied_calibration_dir` there, the node entry here), and ticket.py requires overlapping files to be
  ordered; no code of that task is used.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api, web-ui-applied-head-urdf

- 2026-10-23 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api
