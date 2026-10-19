---
title: Map3d manager node type
status: done
repo: ros2-lerobot-sverve-platform
value: The 3D map manager is built and deployed on the server by Ansible next to RTAB-Map.
files: [ansible/group_vars/server.yml, ansible/playbooks/deploy_nodes_server.yml, tests/test_map3d_manager_deploy.py, tests/test_cpp_node_deploy.py]
verify: "uv run pytest tests/test_map3d_manager_deploy.py tests/test_cpp_node_deploy.py tests/test_unit_cgroups.py tests/test_map3d_topics.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [map3d-manager-node, rtabmap-server-node-type]
owns: []
uses: [map3d-manager-node, rtabmap-mapping-node-type]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--map3d-manager-node-type
branch: rtab-map-rgb-d-3d-mapping-on--map3d-manager-node-type
verified_head: 6a1af434a70ac00ada34dc72db7aae78f6e6a93c
run: rtab-map-rgb-d-3d-mapping-on
provides: []
acs_for_audit: []
process_gaps: []
---

# Map3d manager node type

## Requirement

- server.yml node type `map3d_manager_cpp` (colcon in-repo C++ like map_guard_cpp on the client: `colcon_source` of `shared/ros2_common_cpp` then `nodes/map3d_manager_cpp`, Release, BUILD_TESTING off, jobs 2; `ros2 run map3d_manager_cpp map3d_manager_node`; apt for every package.xml dependency incl. `ros-jazzy-rtabmap-msgs`, `nlohmann-json3-dev`, `libyaml-cpp-dev`), `cpu_quota: "20%"`, `memory_max: "128M"`, config `/etc/ros2/map3d_manager/config.yaml` via `MAP3D_MANAGER_CONFIG`; ros2_nodes entry `map3d_manager` present/enabled after `rtabmap_mapping`, config block with the defaults that differ from code (none expected; comment says so).
- `deploy_nodes_server.yml`: `Deploy map3d_manager` after `Deploy rtabmap_mapping`.
- Tests first: `tests/test_map3d_manager_deploy.py` (type, apt, caps, order, config topics equal the map3d topic contract's manager-side topics: command, map_reset, result/status server names), `tests/test_cpp_node_deploy.py` gains the package in its C++ node list.

## Acceptance conditions

- a1: map3d_manager is built and deployed on the server after rtabmap_mapping with its caps [evidence: test: tests/test_map3d_manager_deploy.py]
- a2: its topics equal the contract's manager-side topics [evidence: test: tests/test_map3d_manager_deploy.py]
- a3: the C++ deploy invariants cover the new package [evidence: test: tests/test_cpp_node_deploy.py]
- a4: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

## Files

- `ansible/group_vars/server.yml`
- `ansible/playbooks/deploy_nodes_server.yml`
- `tests/test_map3d_manager_deploy.py`
- `tests/test_cpp_node_deploy.py`

## Interfaces

- Unit `ros2-map3d_manager` on the server, tag `map3d_manager`.

## Expected outcome

`./scripts/deploy-nodes.sh server map3d_manager` builds and starts the manager.

## Steps

- Write the tests; run them; see them fail.
- Edit server.yml and the playbook; run the verify command.
- Commit `feat(ansible): deploy map3d_manager on the server`.

## Notes

- The server has not built in-repo C++ before: check `ros2_node_deploy` colcon tasks need no client-only variable (e.g. nav2 overlay vars) and say what you found in the commit.
