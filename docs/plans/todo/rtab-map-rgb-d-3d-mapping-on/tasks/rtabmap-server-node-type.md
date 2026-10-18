---
title: Rtabmap server node type
status: todo
repo: ros2-lerobot-sverve-platform
value: One deploy command puts RTAB-Map on the server Pi 4b with a database directory and hard CPU and memory caps that protect the leader arm and the monitoring stack.
files: [ansible/group_vars/server.yml, ansible/playbooks/deploy_nodes_server.yml, ansible/playbooks/tasks/rtabmap_db_dir.yml, tests/test_rtabmap_mapping_deploy.py]
verify: "uv run pytest tests/test_rtabmap_mapping_deploy.py tests/test_unit_cgroups.py tests/test_dds_discovery_config.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [rtabmap-server-launch, map3d-uplink-node-type]
owns: [rtabmap-mapping-node-type]
uses: [rtabmap-mapping-launch]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Rtabmap server node type

## Requirement

- server.yml node type `rtabmap_mapping` (D1, D8): native, `ros2 launch {{ ros2_repo_dest }}/nodes/rtabmap_mapping/launch/rtabmap_mapping.launch.py`, `node_src_dir: ""`, `src_paths: [nodes/rtabmap_mapping]`, apt `ros-jazzy-rtabmap-slam`, `ros-jazzy-rtabmap-sync`, `ros-jazzy-rtabmap-util`, `ros-jazzy-rtabmap-msgs`, `ros-jazzy-rtabmap-conversions`, `ros-jazzy-image-transport-plugins`, `ros-jazzy-rclcpp-components`, `python3-pydantic`, `python3-yaml` (never `ros-jazzy-rtabmap-ros` or `ros-jazzy-rtabmap-viz`), `cpu_quota: "200%"`, `memory_max: "900M"` (server rule: quota and cap, no tier keys), `config_path: /etc/ros2/rtabmap_mapping`, env `RTABMAP_MAPPING_CONFIG=/etc/ros2/rtabmap_mapping/config.yaml`, no `ROS_STATIC_PEERS`. ros2_nodes entry `rtabmap_mapping` present/enabled with an empty override config block commented with the tunable keys.
- `tasks/rtabmap_db_dir.yml`: `/var/lib/ros2/rtabmap` owned by the node user, mode 0755 (pattern: slam_maps_dir.yml), included in the server playbook before `Deploy rtabmap_mapping` with tags `[setup, rtabmap_mapping]`.
- `deploy_nodes_server.yml`: step `Deploy rtabmap_mapping` (resolve_and_deploy.yml, tags `[rtabmap_mapping, apt, python, build, config]`) after gps_rtk_base.
- Tests first in `tests/test_rtabmap_mapping_deploy.py`, seen failing: node type fields, apt list (and the forbidden metapackage/viz), quota/cap, no static peer, ros2_nodes schema, db dir task and its order, playbook step; existing `tests/test_unit_cgroups.py::test_server_keeps_its_limits_without_tiers` and the dds test still pass unchanged.

## Acceptance conditions

- a1: the server deploys rtabmap_mapping from the split apt packages with CPUQuota 200% and MemoryMax 900M [evidence: test: tests/test_rtabmap_mapping_deploy.py]
- a2: the database directory is created before the node deploy [evidence: test: tests/test_rtabmap_mapping_deploy.py]
- a3: server limits and discovery rules still hold [evidence: test: tests/test_unit_cgroups.py::test_server_keeps_its_limits_without_tiers]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

## Files

- `ansible/group_vars/server.yml`
- `ansible/playbooks/deploy_nodes_server.yml`
- `ansible/playbooks/tasks/rtabmap_db_dir.yml`
- `tests/test_rtabmap_mapping_deploy.py`

## Interfaces

- Unit `ros2-rtabmap_mapping` on the server, tag `rtabmap_mapping` for `scripts/deploy-nodes.sh server rtabmap_mapping`.

## Expected outcome

`./scripts/deploy-nodes.sh server rtabmap_mapping` installs and starts RTAB-Map with its caps.

## Steps

- Write the test; run `uv run pytest tests/test_rtabmap_mapping_deploy.py -q`; see it fail.
- Edit server.yml, the playbook and the db dir task; run the verify command.
- Commit `feat(ansible): deploy rtabmap_mapping on the server`.

## Notes

- Server budget: monitoring.slice up to 2.2 GB / 150 % CPU; ~1.1 GB left for ROS units (server.yml comment). 900M leaves room for lerobot_leader, gps_rtk_base, ros2-master and map3d_manager; the runbook measures and proposes final caps.
