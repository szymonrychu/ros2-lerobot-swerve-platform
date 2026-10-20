---
title: Calib Ansible deploy config
status: todo
repo: ros2-lerobot-sverve-platform
value: "A deploy ships the new calibration config and lets the node use mcp_server, with the arm gauge pinned to the mcp_server values."
files: [ansible/group_vars/client.yml, ansible/playbooks/tasks/head_rgbd_calibration_setup.yml, tests/test_head_rgbd_calibration_deploy.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py tests/test_head_config_consistency.py -q && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [sequence-http-api, web-ui-applied-head-urdf]
owns: []
uses: [calib-robot-client]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib Ansible deploy config

## Requirement

Ansible keeps the node config and permissions in sync with the new features (CLAUDE.md Ansible rule; design D6).

- `ansible/group_vars/client.yml` `head_rgbd_calibration` config block: every new key at its default with a one-line comment (`topics.head_joint_states`, `topics.arm_joint_states`, `still`, `fit` incl. `plate`, `plate_mount`, limits and the gauge, `robot`, `head_sequence`, `arm_sequence` incl. the pose set and `joint_offsets_rad`), `arm_urdf_file` pointing at the deployed checkout.
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`: the node's service user (`{{ ansible_user }}`) is in group `mcp-token` (`ansible.builtin.user`, `groups: mcp-token`, `append: true`) so it can read `/etc/ros2/mcp_server/token`; check first whether the user already is, and say so in the task comment either way. `restart_units` and the apply service stay as they are (mcp_server reloads itself, web_ui applies per request).
- `tests/test_head_rgbd_calibration_deploy.py` gains: the client.yml block validates with the node's config schema rules mirrored in the test (every key of the node's shipped YAML present; `fit.board` equals `board`); `arm_sequence.joint_offsets_rad` equals mcp_server `arm.joint_offsets_rad`; `fit.base_in_base_link` equals mcp_server `arm.base_in_base_link` and `fit.shoulder_pan_offset_rad` equals its `shoulder_pan` offset; the plate ids do not overlap the board's; the mcp-token membership task exists.
- `uv run poe lint-ansible` and `uv run poe test-ansible` pass; any new skip or warn is documented in `ansible/.ansible-lint`.

## Acceptance conditions

- a1: the arm gauge and offsets in the node config equal the mcp_server block [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_arm_gauge_and_offsets_match_mcp_server]
- a2: the client.yml node block carries every shipped key [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_client_block_has_every_shipped_key]
- a3: the node user can read the mcp token [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_node_user_in_mcp_token_group]
- a4: ansible lint and syntax checks pass [evidence: command:uv run poe lint-ansible && uv run poe test-ansible]
- a5: head consistency tests pass [evidence: command:uv run --frozen pytest tests/test_head_config_consistency.py tests/test_head_rgbd_calibration_deploy.py -q]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `tests/test_head_rgbd_calibration_deploy.py`

## Interfaces

- client.yml `head_rgbd_calibration` config keys; setup task adding the node user to `mcp-token`.

## Expected outcome

A deploy ships the new calibration config and lets the node use mcp_server, with the arm gauge pinned to the mcp_server values.

## Steps

- Tests first (red), implement, verify.
- Run `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py tests/test_head_config_consistency.py -q && uv run poe lint-ansible && uv run poe test-ansible`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

