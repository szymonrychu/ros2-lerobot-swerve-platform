---
title: Final suite and runbook
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-3d-obstacle-detection-fo--final-suite-and-runbook
branch: tof-3d-obstacle-detection-fo--final-suite-and-runbook
verified_head: 1f35ad68592fa438d2e1cd10f76606bad24846ac
run: tof-3d-obstacle-detection-fo
provides: []
acs_for_audit: []
process_gaps: ["The full verify command was not run by the implementer, so the report quotes no summary lines. Per the prompt, the full suite runs once over the integrated branch before merge."]
points_actual: 1
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
- a6: added CPU <= 0.15 core on the Pi 5, measured in Grafana after deploy and recorded, decided by the user on 2026-10-18 [evidence: judgement]

## Files

- `tests/test_tof_obstacle_wiring.py`
- `tests/README.md`
- `scripts/node_cpu_compare.yaml`
- `MEMORY.md`

## Interfaces

- None new; `tests/test_tof_obstacle_wiring.py`.

## Expected outcome

All suites green with an end-to-end wiring check; the orchestrator deploys and verifies on the robot with the runbook.

## Steps

Implementer:
- Write `tests/test_tof_obstacle_wiring.py`; run `uv run pytest tests/test_tof_obstacle_wiring.py -q`; fix real breaks;
  add the cpu-compare entries and the MEMORY.md line; run the full verify command and quote each summary line; commit
  `test: ToF obstacle pipeline wiring`.

Orchestrator runbook (after merge, robot reachable; every deploy via the ansible-deploy skill, logged to `.logs/`):
1. Preconditions: pan/tilt epic deployed (`/head/joint_states` live, `tof_optical_frame` in TF: ansible ad-hoc
   `ros2 run tf2_ros tf2_echo base_link tof_optical_frame` for 3 s, no manual SSH); ToF streaming: Grafana MCP
   `rate(tof_frames_published_total{host="client"}[1m])` about 10. If either fails, stop and report the robot steps
   pending on it.
2. CPU before: note `date +%s` as switch time; record 30 min of `rate(container_cpu_usage_seconds_total{host="client",
   id="/system.slice/<unit>"}[1m])` for ros2-nav2_bringup, ros2-laser_filter, ros2-arducam_tof (robot idle, nodes up),
   and client total CPU from the "Robot resources" dashboard panel query.
3. Check no agent session is running (claude_agent `GET http://client.ros2.lan:18300/api/state`); if one is, ask before
   deploying mcp_server. Deploy: `./scripts/deploy-nodes.sh client tof_obstacle_cloud laser_filter nav2_bringup 2>&1 |
   tee .logs/deploy-client-tof-obstacles.log`, then `./scripts/deploy-nodes.sh client web_ui mcp_server 2>&1 | tee
   .logs/deploy-client-tof-consumers.log`. Units active; nav2 lifecycle active (no STVL plugin load error in journal via
   the deploy log / Loki).
4. Rates (topic_scraper_api enabled per CLAUDE.md, `scripts/topic_scraper_collect.py --source
   client=http://client.ros2.lan:18100 --select /tof/points_filtered:.header.stamp --select /scan_filtered:.header.stamp
   --interval 0.1`): `/tof/points_filtered` about 10 Hz, `/scan_filtered` unchanged rate.
5. FOV: read `/tof/camera_info` fx, fy (scraper); compute hfov/vfov; if below 1.0 / 0.78 rad, lower the STVL FOV, commit,
   redeploy nav2_bringup.
6. Empty floor: robot stationary, head home pose, 60 s: no ToF cells in `/local_costmap/costmap` on the floor ahead and
   no CliffStop/StopBox action in `/collision_monitor_state` (web UI 3D obstacles layer on, Map tab, Claude in Chrome
   screenshot).
7. Above the lidar plane: a box edge or table top at 0.3-0.5 m height, 1.0 m ahead (nothing at 0.20 m): cells appear in
   the local costmap within 1 s (agent `get_topdown_view` costmap layer and `get_map_summary.obstacles_3d`); remove it:
   cells gone within ~15 s. Below the lidar plane: an object 5-15 cm high: same check.
8. Lidar survival (U5): a lidar-visible obstacle outside the ToF view (behind the robot) stays marked while the ToF
   looks at empty space for 30 s.
9. Cliff at low speed: with a spotter and a physical stop, approach a 10-20 cm step down (or a platform edge) with the
   head in home pose at <= 0.1 m/s (Nav2 goal beyond the edge or `move_relative` with speed limit): the robot stops
   before the front reaches the edge (CliffStop in `/collision_monitor_state`); a backward command still moves it.
10. Outage: `ros2 param set /collision_monitor tof_obstacles.enabled false` and true again works; stopping arducam_tof
    stops Nav2 motion ("invalid source") and restarting it resumes.
11. CPU after: from switch + 10 min, 30 min windows as in step 2 plus ros2-tof_obstacle_cloud; run `uv run python -m
    scripts.node_cpu_compare` with the switch time set (local edit, not committed); added CPU (new unit + deltas) <= 0.15
    core mean and EKF `/odometry/filtered` still ~30 Hz; record the table in the record. If over budget, report with the
    numbers (do not tighten cgroups).
12. Disable topic_scraper_api again if it was disabled before.

## Notes

- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- If the robot is offline, runbook steps are reported pending on the robot, with everything else done.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on web-ui-obstacle-points-layer, map-summary-tof-obstacles

- 2026-10-27 done: superseded by realsense-d435-head-switchov/nav2-live-check-on-d435

- 2026-10-27 actuals: no agent records
