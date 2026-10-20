---
title: Nav2 ground voxel layer
status: todo
repo: ros2-lerobot-sverve-platform
value: "Nav2 avoids plane-relative obstacles and drop-offs alongside the fixed bands, with a runtime switch and written switch-over criteria."
files: [nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_ground_voxel_layer.py, tests/test_nav_tof_voxel_layer.py, tests/test_tof_obstacle_wiring.py, tests/test_nav_tof_collision.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_tof_voxel_layer.py tests/test_tof_obstacle_wiring.py tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint"
depends_on: [ground-node-type, costmap-ground-warnings]
owns: []
uses: [ground-topics, ground-node-unit, ground-cost-mask]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rgb-d-ground-estimation-and--nav2-ground-voxel-layer
branch: rgb-d-ground-estimation-and--nav2-ground-voxel-layer
verified_head: ""
run: rgb-d-ground-estimation-and
provides: []
acs_for_audit: []
process_gaps: []
---

# Nav2 ground voxel layer

## Requirement

- `nodes/nav2_bringup/config/nav2_params.yaml` (design D8, D13, D15): local costmap `plugins: ["obstacle_layer", "tof_voxel_layer", "ground_voxel_layer", "keepout_filter", "ground_cost_filter", "inflation_layer"]`; new local-only `ground_cost_filter` (`nav2_costmap_2d::KeepoutFilter` as a layer below inflation like keepout_filter, `enabled: true`, `filter_info_topic: /costmap_filter_info/ground_cost`, `transform_tolerance: 0.2`, `override_lethal_cost: false`) carrying the costmap_zones ground warning mask: a non-lethal cost (mask 40 -> ~101) that MPPI and the planner avoid when cheap but may drive through, while the unchanged `speed_filter` applies the warning speed percent from the shared speed mask to `/speed_limit`, which controller_server honours (slow-down, not a stop); new `ground_voxel_layer` (`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`, `enabled: true`) with the tof_voxel_layer layer settings (voxel_decay 15.0, decay_model 0, voxel_size 0.05, track_unknown_space false, mark_threshold 0, update_footprint_enabled false, combination_method 1, publish_voxel_map false, transform_tolerance 0.3, mapping_mode false) and `observation_sources: ground_obstacle_mark ground_drop_mark ground_clear`: `ground_obstacle_mark` (topic `/ground/obstacle_points`) and `ground_drop_mark` (topic `/ground/drop_points`), PointCloud2, marking true, clearing false, `min_obstacle_height: -1.5`, `max_obstacle_height: 1.5` (pass-through: ground_estimator_cpp already applied the plane-relative bands), obstacle_range 2.0, filter voxel, voxel_min_points 0, observation_persistence 0.0, clear_after_reading true, expected_update_rate 0.0, inf_is_valid false; `ground_clear` = the tof_clear frustum on `/tof/points_filtered` (model_type 0, min_z 0.2, max_z 3.0, horizontal_fov_angle 1.0, vertical_fov_angle 0.78, decay_acceleration 1.0). Every value commented with its reason, a header comment naming the design (alongside the fixed bands, switch-over criteria in the README, runtime switch `ros2 param set /local_costmap/local_costmap ground_voxel_layer.enabled false`).
- Unchanged and pinned: `tof_voxel_layer` (all keys and the three sources), the collision_monitor `observation_sources` `["scan", "tof_obstacles", "tof_cliff"]` with their bands and the stale-source stop, CliffStop's source set (tof_cliff only), the global costmap plugins `[static_layer, obstacle_layer, keepout_filter, inflation_layer]` with no STVL and no ground topic, and the `gen_nav_footprint` generated blocks (not hand-edited; tests/test_nav_footprint.py stays green). A comment at the collision_monitor ToF sources states that the ground clouds do not feed the monitor (user decision D-A1: the 10 Hz ToF bands with the fail-safe stale stop stay) - the CliffStop review.
- Tests first, seen failing: new `tests/test_nav_ground_voxel_layer.py` (plugin order including ground_cost_filter between keepout_filter and inflation_layer; ground_cost_filter is a KeepoutFilter on `/costmap_filter_info/ground_cost` with override_lethal_cost false, absent from the global costmap, and its info topic equals the costmap_zones `ground_cost_info_topic` default; the costmap_zones `ground_warning_mask_value` maps (value * 254 / 100) below INSCRIBED 253; the local costmap still has exactly one SpeedFilter on `/costmap_filter_info/speed`; layer settings equal the tof layer's; sources, topics and pass-through limits; the ground marking topics equal the ground_estimator_cpp config `output_namespace` + `/obstacle_points` / `/drop_points` in client.yml and in `nodes/ground_estimator_cpp/config/ground_estimator.yaml`; ground_clear equals tof_clear; global costmap untouched; collision monitor sources and CliffStop sources unchanged and free of `/ground/` topics; values commented with their reasons; README section present and free of parking words; tests/README lists the file). `tests/test_nav_tof_voxel_layer.py`: `test_only_one_stvl_layer_in_the_local_costmap` becomes exactly `[tof_voxel_layer, ground_voxel_layer]`; the plugin-order expectations include the ground layer. `tests/test_tof_obstacle_wiring.py`: the STVL-source check stays scoped to `tof_voxel_layer`; the chain adds laser_filter `cloud_filtered_topic` == ground_estimator_cpp `cloud_topic` == `ground_clear` topic. `tests/test_nav_tof_collision.py`: an explicit assertion that no collision_monitor source topic starts with `/ground/`.
- `nodes/nav2_bringup/README.md`: section "Ground layer (plane-relative ToF marks)": what it marks, why a separate layer, the pass-through limits, what stays on the fixed bands (collision monitor, CliffStop), the obstacle vs warning rule (one third of the wheel diameter, max slope) and the slow-down mechanism (ground_cost_filter cost band plus the speed mask / speed_limit from costmap_zones), the runtime switch, and the switch-over criteria: (1) runbook level-floor check passes (plane height within 1 cm, slope and plane_vs_base below 1 deg); (2) the 3 cm sill, 5 cm sill, 10 cm drop at 1 m, box and ramp scenes mark as expected with the ground layer and the fixed-band layer disabled at runtime; (3) no false marks on the floor over a 5 min drive; (4) CPU within the budget; then a separate change disables `tof_mark` and `tof_cliff_mark`.
- `tests/README.md`: the new file and the changed tests described.

## Acceptance conditions

- a1: the local costmap has ground_voxel_layer after tof_voxel_layer with the specified sources, topics and settings [evidence: test: tests/test_nav_ground_voxel_layer.py]
- a2: tof_voxel_layer, collision monitor sources, CliffStop and the global costmap are unchanged and carry no ground topic [evidence: command: uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_tof_obstacle_wiring.py tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]
- a3: the ground marking topics are consistent with the ground_estimator_cpp config [evidence: test: tests/test_nav_ground_voxel_layer.py]
- a4: the nav2_bringup README documents the ground layer and the switch-over criteria [evidence: doc-section: nodes/nav2_bringup/README.md#Ground layer (plane-relative ToF marks)]

- a5: ground warning areas reach the local costmap only as a non-lethal ground_cost_filter band, and the single speed filter applies the slow-down [evidence: test: tests/test_nav_ground_voxel_layer.py]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_ground_voxel_layer.py`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_tof_obstacle_wiring.py`
- `tests/test_nav_tof_collision.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

nav2_params.yaml `local_costmap.local_costmap.ros__parameters.ground_voxel_layer`; consumes part `ground-topics`.

## Expected outcome

`uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_tof_voxel_layer.py tests/test_tof_obstacle_wiring.py tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q` passes.

## Steps

- write tests/test_nav_ground_voxel_layer.py and the test changes
- run the expected-outcome command, see the new tests fail
- edit nav2_params.yaml until green
- write the README section and tests/README entries
- commit `feat(nav2): ground voxel layer alongside the ToF bands`.

## Notes

nav2_params.yaml:242-329 (tof_voxel_layer), 579/686 (CliffStop), 732-750 (collision monitor sources). The tof epic's escalated `final-suite-and-runbook` also edits tests/test_tof_obstacle_wiring.py: rebase onto whatever landed, keep both sets of checks.

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-node-type, costmap-ground-warnings
