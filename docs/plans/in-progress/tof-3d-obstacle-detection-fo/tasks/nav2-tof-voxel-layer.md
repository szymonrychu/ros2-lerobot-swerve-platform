---
title: Nav2 ToF voxel layer
status: todo
repo: ros2-lerobot-sverve-platform
value: 3D obstacles and cliffs seen by the head ToF appear in the local costmap next to the lidar obstacles.
files: [nodes/nav2_bringup/config/nav2_params.yaml, ansible/group_vars/client.yml, tests/test_nav_tof_voxel_layer.py, tests/test_nav_stack_config.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [cliff-stop-polygon-generator]
owns: []
uses: [tof-points-filtered-topic]
model: opus
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-3d-obstacle-detection-fo--nav2-tof-voxel-layer
branch: tof-3d-obstacle-detection-fo--nav2-tof-voxel-layer
verified_head: ""
run: tof-3d-obstacle-detection-fo
provides: []
acs_for_audit: []
process_gaps: []
---

# Nav2 ToF voxel layer

## Requirement

Test first: `tests/test_nav_tof_voxel_layer.py` fails before the config exists (D3, D4, D8; U1, U2, U5).

- `nav2_params.yaml` local costmap: `plugins: ["obstacle_layer", "tof_voxel_layer", "keepout_filter",
  "inflation_layer"]`; `tof_voxel_layer` with exactly the D3 settings: `plugin:
  "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"`, `enabled true`, `voxel_decay 15.0`, `decay_model 0`,
  `voxel_size 0.05`, `track_unknown_space false`, `mark_threshold 0`, `update_footprint_enabled false` (comment: STVL
  clears the footprint with setConvexPolygonCost, convex like the ObstacleLayer, would erase obstacles in the H
  notches), `combination_method 1`, `publish_voxel_map false`, `transform_tolerance 0.3`, `mapping_mode false`,
  `observation_sources: tof_mark tof_cliff_mark tof_clear`; `tof_mark` (marking, not clearing, PointCloud2,
  `/tof/points_filtered`, `min_obstacle_height 0.06`, `max_obstacle_height 0.6`, `obstacle_range 2.0`, `filter voxel`,
  `voxel_min_points 0`, `observation_persistence 0.0`, `clear_after_reading true`, `expected_update_rate 0.0`,
  `inf_is_valid false`); `tof_cliff_mark` (same but `min_obstacle_height -1.0`, `max_obstacle_height -0.10`);
  `tof_clear` (clearing, not marking, `model_type 0`, `min_z 0.2`, `max_z 3.0`, `horizontal_fov_angle 1.0`,
  `vertical_fov_angle 0.78`, `decay_acceleration 1.0`). Each value commented with its reason (floor z = 0 in odom,
  ToF noise, robot height, FOV under the 70 deg diagonal).
- U5 kept: `obstacle_layer` with the `/scan_filtered` LaserScan source unchanged in both costmaps; the global costmap
  plugins unchanged (no STVL there); no STVL source uses `/scan_filtered` or a LaserScan (STVL is not for planar lidars).
- `ansible/group_vars/client.yml` nav2_bringup `apt_packages` gains `ros-jazzy-spatio-temporal-voxel-layer` (comment).
- `tests/test_nav_stack_config.py`: the plugin-order parametrisation of
  `test_nav2_keepout_filter_sits_right_before_inflation` uses the new local list; everything else unchanged and green.
- `tests/test_nav_tof_voxel_layer.py`: every value above; mark/clear split; heights: obstacle band above the floor and
  below robot height, cliff band strictly below `-0.05`; ranges: `max_z` and `obstacle_range` <= the
  tof_obstacle_cloud `max_depth` (read from `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`); clearing FOV below
  the ToF FOV constants; lidar layer still present in both costmaps with the scan source; apt package listed.
- `nodes/nav2_bringup/README.md`: new section "ToF obstacle layer" (pipeline, layer settings table, U5 separation:
  ToF clearing only decays ToF voxels, lidar raytracing never clears ToF voxels, why local only, limitation: a cliff
  into a void with no returns is not seen); `tests/README.md` rows.

## Acceptance conditions

- a1: local costmap plugins are obstacle_layer, tof_voxel_layer, keepout_filter, inflation_layer [evidence: test: tests/test_nav_stack_config.py::test_nav2_keepout_filter_sits_right_before_inflation]
- a2: tof_voxel_layer settings, mark/clear sources and height bands match D3 [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a3: STVL footprint clearing is disabled [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a4: the lidar obstacle_layer keeps /scan_filtered in both costmaps and no STVL source uses a LaserScan (U5) [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a5: nav2_bringup installs ros-jazzy-spatio-temporal-voxel-layer [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a6: README documents the ToF obstacle layer and the lidar separation [evidence: doc-section: nodes/nav2_bringup/README.md#ToF obstacle layer]
- a7: nav, footprint and ansible checks pass [evidence: command: uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe test-ansible]
- a8: ToF voxels live only in the local costmap; the global costmap has no STVL plugin or ToF source, decided by the user on 2026-10-18 [evidence: test: tests/test_nav_tof_voxel_layer.py]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_nav_stack_config.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

- Consumes `tof-points-filtered-topic` (`/tof/points_filtered`).
- Layer name `tof_voxel_layer` in `local_costmap` (mcp_server and web_ui read the merged `/local_costmap/costmap`; no
  change there).

## Expected outcome

On the robot the local costmap marks ToF obstacles between 6 and 60 cm and below-floor cliff points, decaying 15 s after
they leave view, while lidar cells behave as before.

## Steps

- Write `tests/test_nav_tof_voxel_layer.py` and update the parametrisation; `uv run pytest tests/test_nav_tof_voxel_layer.py -q` fails.
- Edit `nav2_params.yaml` (outside generated regions) and the apt list; rerun the verify command.
- README section and tests/README rows; commit `feat(nav2): ToF spatio-temporal voxel layer in the local costmap`.

## Notes

- Verified: `ros-jazzy-spatio-temporal-voxel-layer 2.5.5` on Jazzy arm64; parameter names from the STVL jazzy source
  (design, Verified facts). Heights in STVL are in the costmap global frame (odom), where z = 0 is the floor.
- Do not touch the collision_monitor here (next task) or generated regions.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on cliff-stop-polygon-generator
