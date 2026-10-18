---
title: Map3d docs and memory
status: todo
repo: ros2-lerobot-sverve-platform
value: "Anyone can find what the 3D mapping pieces are, how they connect and how they were tested, from the repo docs alone."
files: [tests/README.md, nodes/README.md, ansible/README.md, MEMORY.md, .claude/skills/ansible-deploy/SKILL.md, docs/diagrams/topic_flow.puml]
verify: "uv run pytest tests/test_node_lint_coverage.py -q && uv run poe lint && grep -q 'test_map3d_topics.py' tests/README.md && grep -q 'map3d_uplink' ansible/README.md && grep -q 'rtabmap_mapping' nodes/README.md"
depends_on: [map3d-topic-contract, map3d-uplink-node-type, rtabmap-server-node-type, map3d-manager-node-type, web-ui-map3d-save-reset, map3d-frontend-status, mcp-map3d-config-docs, mapping-cpu-dashboards]
owns: []
uses: []
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

# Map3d docs and memory

## Requirement

- `tests/README.md`: entries for every new or changed test file of the epic (test_map3d_topics.py, test_map3d_uplink_config.py, test_rtabmap_mapping_config.py, test_rtabmap_mapping_deploy.py, test_map3d_manager_deploy.py, and the changes to test_dds_discovery_config.py, test_unit_cgroups.py, test_cpp_node_deploy.py, test_mcp_server_config.py, test_monitoring_dashboards.py), each with what it covers.
- `nodes/README.md`: rows for rtabmap_mapping (server), map3d_manager_cpp (server), map3d_uplink (client) and the head camera mapping stream.
- `ansible/README.md`: "Cross-host" names map3d_uplink as the third static-peer process and why; the server node list gains rtabmap_mapping and map3d_manager with their caps; the memory budget table gains the new units.
- `MEMORY.md`: a "3D mapping (RTAB-Map on the server)" entry with the key decisions D1, D2, D7, D8, D13 (one line each).
- `.claude/skills/ansible-deploy/SKILL.md`: the new node names and their targets.
- `docs/diagrams/topic_flow.puml`: the uplink/downlink streams (regenerating the PNG with scripts/generate-diagrams.sh when plantuml is available; otherwise say so in the commit).

## Acceptance conditions

- a1: tests/README lists every new test file of the epic [evidence: command: grep -q 'test_map3d_topics.py' tests/README.md && grep -q 'test_rtabmap_mapping_config.py' tests/README.md && grep -q 'test_map3d_uplink_config.py' tests/README.md && grep -q 'test_map3d_manager_deploy.py' tests/README.md]
- a2: ansible README names map3d_uplink as a static-peer process [evidence: command: grep -q 'map3d_uplink' ansible/README.md]
- a3: MEMORY records the 3D mapping decisions [evidence: command: grep -q 'RTAB-Map' MEMORY.md]
- a4: docs read correctly for a mid-level ROS 2 developer [evidence: judgement]

## Files

- `tests/README.md`
- `nodes/README.md`
- `ansible/README.md`
- `MEMORY.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `docs/diagrams/topic_flow.puml`

## Interfaces

- Docs only.

## Expected outcome

The docs describe the 3D mapping stack end to end.

## Steps

- Edit the docs; run the verify command.
- Commit `docs: 3D mapping stack`.

## Notes

- Doc-only task (TDD exception: documentation).
