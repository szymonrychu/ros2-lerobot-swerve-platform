---
title: Head state publisher node
status: done
repo: ros2-lerobot-sverve-platform
value: TF for head_camera_optical_frame and tof_optical_frame follows the real pan/tilt angles.
files: [nodes/head_state_publisher/launch/head_state_publisher.launch.py, nodes/head_state_publisher/README.md, ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_head_state_publisher_config.py, tests/README.md, ansible/README.md, .claude/skills/ansible-deploy/SKILL.md]
verify: "uv run pytest tests/test_head_state_publisher_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [head-servo-group-config]
owns: []
uses: [head-urdf, head-servo-group]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--head-state-publisher-node
branch: pan-tilt-head-control-with-a--head-state-publisher-node
verified_head: 456a20e614664ef5d27227f85c74b5783b25f015
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: []
process_gaps: []
---

# Head state publisher node

## Requirement

- New `nodes/head_state_publisher/launch/head_state_publisher.launch.py` (D6): starts `robot_state_publisher` (package `robot_state_publisher`, executable `robot_state_publisher`, node name `head_state_publisher`) with parameter `robot_description` = the text of `nodes/web_ui/urdf/head.urdf` (path resolved relative to the launch file, so it works from the deployed checkout), remappings `joint_states` -> `/head/joint_states` and `robot_description` -> `/head/robot_description`. New `nodes/head_state_publisher/README.md`: what it publishes (fixed joints on /tf_static at start; head_pan/head_tilt transforms only from real joint states), frames, how to check (`ros2 run tf2_ros tf2_echo base_link head_camera_optical_frame`).
- `ansible/group_vars/client.yml`: node type `head_state_publisher` (native, `node_launch_command: ros2 launch {{ ros2_repo_dest }}/nodes/head_state_publisher/launch/head_state_publisher.launch.py`, `node_src_dir: ""`, `src_paths: [nodes/head_state_publisher, nodes/web_ui/urdf/head.urdf]`, `apt_packages: [ros-jazzy-robot-state-publisher]`, cgroup mid tier: memory_low 32M, memory_max 128M, cpu_weight 500, io_weight 500, oom_score_adjust -500, comment "not measured yet"); ros2_nodes entry `head_state_publisher` present/enabled, placed after the base bus node `base_servos`.
- `ansible/playbooks/deploy_nodes_client.yml`: a `Deploy head_state_publisher` step in the existing pattern (tags include `head_state_publisher`, `apt`).
- `ansible/README.md` node table/section and `.claude/skills/ansible-deploy/SKILL.md` node list gain head_state_publisher.
- Tests first in new root `tests/test_head_state_publisher_config.py`, seen failing: node type fields and tier; entry present and enabled after the base bus node `base_servos`; the launch file (parsed with `ast`, no ROS import) names robot_state_publisher, reads head.urdf, remaps both topics; the playbook step exists with its tags; `tests/test_unit_cgroups.py` rules still pass.
- `tests/README.md` lists the new test file.

## Acceptance conditions

- a1: the head_state_publisher node type and entry deploy robot_state_publisher on head.urdf remapped to /head/joint_states [evidence: test: tests/test_head_state_publisher_config.py]
- a2: the playbook deploys head_state_publisher [evidence: test: tests/test_head_state_publisher_config.py]
- a3: ansible README and the ansible-deploy skill name head_state_publisher [evidence: command: grep -q head_state_publisher ansible/README.md && grep -q head_state_publisher .claude/skills/ansible-deploy/SKILL.md]
- a4: tests/README.md lists test_head_state_publisher_config.py [evidence: command: grep -q 'test_head_state_publisher_config.py' tests/README.md]
- a5: root tests, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible]

## Files

- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `nodes/head_state_publisher/README.md`
- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_head_state_publisher_config.py`
- `tests/README.md`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`

## Interfaces

- `nodes/head_state_publisher/launch/head_state_publisher.launch.py:generate_launch_description`; client.yml node type and entry `head_state_publisher`.

## Expected outcome

Ansible can deploy, enable and disable head_state_publisher; it publishes head TF from /head/joint_states.

## Steps

- Write `tests/test_head_state_publisher_config.py`; run `uv run pytest tests/test_head_state_publisher_config.py -q`; expect failures.
- Add the launch file, README, node type, entry, playbook step and docs; run the verify command; commit `feat(head_state_publisher): robot_state_publisher for the head`.

## Notes

- Patterns: `robot_localization_ekf` (launch-file native node), `joint_state_throttle_follower` (apt node type, src_paths).
- The launcher runs `bash -c` in single quotes: no single quote in node_launch_command.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-servo-group-config

## Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.
