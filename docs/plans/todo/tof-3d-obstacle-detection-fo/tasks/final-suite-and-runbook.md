---
title: Final suite and runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "Every suite is green, the ToF pipeline is wired consistently end to end, and the robot verification is precise."
files: [tests/test_tof_obstacle_wiring.py, tests/README.md, scripts/node_cpu_compare.yaml, MEMORY.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [web-ui-obstacle-points-layer, map-summary-tof-obstacles]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Final suite and runbook

## Requirement

Test first: `tests/test_tof_obstacle_wiring.py` is written and seen failing on any break in the chain (D9, D12).

- `tests/test_tof_obstacle_wiring.py`: the topic chain is consistent across files - tof_obstacle_cloud remap output
  equals laser_filter `cloud_topic`; laser_filter `cloud_filtered_topic` equals every STVL source topic, both
  collision_monitor ToF sources, the web_ui map_nav `obstacle_cloud_topic` (client.yml and default.yaml) and the
  mcp_server `topics.tof_points` (client.yml and config default); arducam_tof `frame_id` is `tof_optical_frame`; the
  STVL and collision_monitor height bands agree; slam_toolbox still publishes map->odom (`transform_publish_period` > 0)
  and no other client node publishes it (U4); every new node/test is in `tests/README.md`.
- `scripts/node_cpu_compare.yaml`: entries `nav2_bringup`, `laser_filter`, `arducam_tof` (same unit before/after,
  `switch_time: 1` placeholder per the file's convention) with a comment that `ros2-tof_obstacle_cloud.service` is new
  and measured "after" only by the runbook query.
- `MEMORY.md`: one decision entry (ToF obstacles: point_cloud_xyz + laser_filter self filter + STVL local + collision
  monitor sources; height bands instead of obstacles_detection; lidar layers kept separate; fail-safe on ToF outage;
  CPU budget 0.15 core).
- The full verify command passes; the report quotes every summary line.
- The runbook below is complete and is executed by the orchestrator after merge (not by the implementer).

## Acceptance conditions

- a1: the ToF topic chain, frames, bands and map->odom ownership are consistent [evidence: test: tests/test_tof_obstacle_wiring.py]
- a2: node_cpu_compare.yaml has the nav2_bringup, laser_filter and arducam_tof entries [evidence: command: grep -q 'name: nav2_bringup' scripts/node_cpu_compare.yaml && grep -q 'name: laser_filter' scripts/node_cpu_compare.yaml && grep -q 'name: arducam_tof' scripts/node_cpu_compare.yaml && uv run pytest tests/test_node_cpu_compare.py -q]
- a3: MEMORY.md records the ToF obstacle decision [evidence: command: grep -q 'spatio_temporal_voxel_layer' MEMORY.md]
- a4: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a5: the runbook covers ToF/TF preconditions, obstacle above and below the lidar plane, cliff stop at low speed, empty floor, lidar survival and CPU before/after [evidence: judgement]

## Files

- `tests/test_tof_obstacle_wiring.py`
- `tests/README.md`
- `scripts/node_cpu_compare.yaml`
- `MEMORY.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
