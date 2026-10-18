---
title: Collision monitor ToF stop
status: todo
repo: ros2-lerobot-sverve-platform
value: "The robot stops for 3D obstacles and refuses to drive toward a cliff, while the lidar keeps guarding all around."
files: [nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_tof_collision.py, tests/test_nav_stack_config.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint"
depends_on: [nav2-tof-voxel-layer]
owns: []
uses: [tof-points-filtered-topic, cliffstop-polygon]
model: opus
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-3d-obstacle-detection-fo--collision-monitor-tof-stop
branch: tof-3d-obstacle-detection-fo--collision-monitor-tof-stop
verified_head: ""
run: tof-3d-obstacle-detection-fo
provides: []
acs_for_audit: []
process_gaps: []
---

# Collision monitor ToF stop

## Requirement

Test first: `tests/test_nav_tof_collision.py` fails before the config exists (D4, D5; U1, U5).

- `nav2_params.yaml` collision_monitor: `observation_sources: ["scan", "tof_obstacles", "tof_cliff"]`;
  `tof_obstacles` (`type: "pointcloud"`, `topic: "/tof/points_filtered"`, `min_height: 0.06`, `max_height: 0.6`,
  `min_range: 0.0`, `enabled: true`) and `tof_cliff` (same topic, `min_height: -1.0`, `max_height: -0.10`); both keep
  the node `source_timeout` 1.0 (no per-source override), with a comment that a ToF or head-TF outage stops motion
  ("invalid source", collision_monitor_node.cpp 1.3.13) and how to disable a source at runtime (`ros2 param set
  /collision_monitor tof_cliff.enabled false`).
- `polygons: ["StopBox", "ArmStop", "CliffStop"]`; `StopBox.sources_names: ["scan", "tof_obstacles"]`;
  `ArmStop.sources_names: ["scan"]` (comment: ToF edge noise around the released arm must not freeze it; the costmap
  footprint includes the arm shadow); `CliffStop.sources_names: ["tof_cliff"]` (block from cliff-stop-polygon-generator,
  unchanged otherwise). Generated regions untouched.
- U5: the `scan` source stays `type: scan`, `/scan_filtered`, enabled, and stays in StopBox.
- `tests/test_nav_tof_collision.py`: sources and bands (obstacle band equals the STVL `tof_mark` band, cliff band equals
  `tof_cliff_mark`), all on `/tof/points_filtered`; every polygon's `sources_names` names only declared sources; StopBox
  / ArmStop / CliffStop source sets as above; CliffStop is `approach` with `time_before_collision` > 0 and its points
  equal the generator's cliff polygon; no ToF source sets `source_timeout: 0.0`. Existing
  `test_nav2_collision_monitor_uses_scan` stays green (CliffStop is a static polygon with >= 4 points).
- Docker param-load check, as in the README "Validation in Docker" practice: arm64 `ros:jazzy` with
  `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-spatio-temporal-voxel-layer`; load the repo
  `nav2_params.yaml` into a local costmap and the collision_monitor under a lifecycle manager with static TFs
  (base_link -> tof_optical_frame looking forward-down) and a synthetic `/tof/points_filtered` publisher: both configure
  and activate without errors; a box of points 0.6 m ahead at z 0.3 marks local costmap cells; points at z -0.3 inside
  the CliffStop polygon make the monitor stop a forward `cmd_vel_smoothed` and pass a backward one; stopping the
  synthetic publisher for > 1 s makes it stop ("invalid source"). Results recorded in the README section "ToF
  validation in Docker" (date, image, package versions, each outcome). If Docker is not available, report `blocked`
  with that reason (never claim it ran).
- `nodes/nav2_bringup/README.md`: "Collision monitor" section updated (sources table, polygon/source matrix, fail-safe
  stop on ToF outage, cliff limitation: only while the ToF looks at it), plus the Docker section; `tests/README.md` rows.

## Acceptance conditions

- a1: tof_obstacles and tof_cliff pointcloud sources on /tof/points_filtered with the D5 bands [evidence: test: tests/test_nav_tof_collision.py]
- a2: StopBox uses scan + tof_obstacles, ArmStop scan only, CliffStop tof_cliff with approach [evidence: test: tests/test_nav_tof_collision.py]
- a3: CliffStop is listed in polygons and its points equal the generated cliff polygon [evidence: test: tests/test_nav_tof_collision.py]
- a4: the scan source and its StopBox use are unchanged (U5) [evidence: test: tests/test_nav_stack_config.py::test_nav2_collision_monitor_uses_scan]
- a5: Docker param-load run recorded with every outcome [evidence: doc-section: nodes/nav2_bringup/README.md#ToF validation in Docker]
- a6: nav and footprint tests pass [evidence: command: uv run pytest tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]
- a7: a ToF or head-TF outage stops the robot while Nav2 drives: both ToF sources keep a non-zero source_timeout (stale-source stop), decided by the user on 2026-10-18 [evidence: test: tests/test_nav_tof_collision.py]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_tof_collision.py`
- `tests/test_nav_stack_config.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

- Consumes `tof-points-filtered-topic` and `cliffstop-polygon`.
- collision_monitor source names `tof_obstacles`, `tof_cliff` (runtime-toggled via `<source>.enabled`).

## Expected outcome

The collision monitor stops for ToF obstacles inside the swept StopBox, blocks motion toward below-floor points within
0.25 m of the footprint while allowing motion away, and stops Nav2 motion when the ToF cloud or its TF goes stale.

## Steps

- Write `tests/test_nav_tof_collision.py`; `uv run pytest tests/test_nav_tof_collision.py -q` fails.
- Edit the collision_monitor block (outside generated regions); rerun the verify command.
- Run the Docker param-load check; record the outcomes in the README; tests/README rows.
- Commit `feat(nav2): collision monitor ToF obstacle and cliff sources`.

## Notes

- Verified in navigation2 1.3.13 source: pointcloud heights compare in the base frame (negative bands work);
  `sources_names` per polygon; an enabled source with non-zero timeout and no fresh data or TF forces STOP.
- The approach model needs the ToF to see the lower floor; the polygon margin (0.25 m) covers the occlusion offset.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on nav2-tof-voxel-layer
