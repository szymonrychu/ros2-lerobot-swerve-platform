---
title: Map manager Ansible wiring
status: escalated
repo: ros2-lerobot-sverve-platform
value: "map_manager deploys, enables and configures through the ros2_nodes schema like every other node."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_map_manager_config.py, ansible/README.md, .claude/skills/ansible-deploy/SKILL.md, nodes/README.md]
verify: "uv run pytest tests/test_map_manager_config.py -q && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [map-manager-ros-node]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--map-manager-ansible-wiring
branch: named-maps-with-pois-and-rel--map-manager-ansible-wiring
verified_head: ""
run: named-maps-with-pois-and-rel
provides: []
acs_for_audit: []
process_gaps: []
---

# Map manager Ansible wiring

## Requirement

Ansible for map_manager (CLAUDE.md "Ansible and ROS2 nodes").

- `ros2_node_type_defaults.map_manager` in `ansible/group_vars/client.yml`: `deploy_mode: native`,
  `node_launch_command: "python3 -m map_manager"`, `node_src_dir: "nodes/map_manager"`, `src_shared: true`,
  `config_path: /etc/ros2/map_manager`, env `MAP_MANAGER_CONFIG=/etc/ros2/map_manager/config.yaml`, normal tier
  (`cpu_weight: 500`, `io_weight: 500`, `oom_score_adjust: -500`), initial memory `memory_low: "64M"`,
  `memory_max: "256M"` with the "initial values, not measured" comment, `apt_packages: [ros-jazzy-slam-toolbox]`
  (service types).
- `ros2_nodes` entry `map_manager` (present, enabled) whose `config: |` block sets every key of `MapManagerConfig`
  that differs from or documents the defaults: `metrics_port: 19113`, `maps_root: /var/lib/ros2/maps/named`, topics,
  services, thresholds, with a comment per group.
- `ansible/playbooks/deploy_nodes_client.yml`: a map_manager block that includes `tasks/slam_maps_dir.yml` (the maps
  root parent) and then `tasks/resolve_and_deploy.yml`, tagged `map_manager`, placed after poi_store.
- `tests/test_map_manager_config.py` (test first): node type defaults, entry and config validate against
  `MapManagerConfig` defaults (parsed with yaml, compared to the keys), the maps root sits under `/var/lib/ros2/maps`,
  the deploy block exists and includes the maps dir task, docs list the node (nodes/README.md index,
  ansible/README.md, SKILL.md).
- Docs: `nodes/README.md` index row, `ansible/README.md` node table, `.claude/skills/ansible-deploy/SKILL.md` node list.

## Acceptance conditions

- a1: the map_manager node type, ros2_nodes entry, config and deploy block are pinned [evidence: test: tests/test_map_manager_config.py]
- a2: ansible-lint and the playbook syntax check pass [evidence: command: uv run poe lint-ansible && uv run poe test-ansible]
- a3: the node is listed in nodes/README.md [evidence: command: grep -q map_manager nodes/README.md && grep -q map_manager ansible/README.md && grep -q map_manager .claude/skills/ansible-deploy/SKILL.md]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_map_manager_config.py`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `nodes/README.md`

## Interfaces

- `ros2_node_type_defaults.map_manager` and the `ros2_nodes` `map_manager` entry in `ansible/group_vars/client.yml`.
- Deploy block in `ansible/playbooks/deploy_nodes_client.yml`.

## Expected outcome

`./scripts/deploy-nodes.sh client map_manager` would deploy, configure and start the node.

## Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_map_manager_config.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_map_manager_config.py -q && uv run poe lint-ansible && uv run poe test-ansible`; expect every test passed and lint clean.
- Commit: `feat(ansible): deploy map_manager`.

## Notes

Follow poi_store's node type and its deploy block; include `tasks/slam_maps_dir.yml` so `/var/lib/ros2/maps` exists owned by the node user (map_manager creates `named/` itself).
- Servo safety (CLAUDE.md): this task moves no servo.
