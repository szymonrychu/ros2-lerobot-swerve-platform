---
title: ToF mark band ten cm
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The local costmap and the collision monitor stop collecting floor noise as obstacles in front of the robot.
files: [nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_tof_voxel_layer.py, tests/test_nav_tof_collision.py, tests/test_nav_ground_voxel_layer.py, nodes/mcp_server/mcp_server/perception.py, nodes/mcp_server/mcp_server/tools.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_tof_collision.py tests/test_nav_ground_voxel_layer.py tests/test_tof_obstacle_wiring.py -q && uv run poe lint && cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: [tof-mark-band]
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-depth-fix-and-ground-noi--tof-mark-band-ten-cm
branch: tof-depth-fix-and-ground-noi--tof-mark-band-ten-cm
verified_head: ""
run: ""
provides: []
---

# ToF mark band ten cm

## Requirement

ToF obstacle band from 6 cm to 10 cm (design D5; approval A4).

- `nodes/nav2_bringup/config/nav2_params.yaml`: local costmap `tof_voxel_layer.tof_mark.min_obstacle_height` 0.06 -> 0.10 and collision_monitor `tof_obstacles.min_height` 0.06 -> 0.10; the comments give the reason (floor noise after the head remount: ~7 % of floor points above 6 cm per frame, the 15 s voxel decay unions them; 10 cm with the depth fix leaves ~1.3 % of floor cells over 10 frames). max_obstacle_height 0.6, the cliff band (-1.0..-0.10), voxel_decay and mark_threshold stay.
- `nodes/mcp_server/mcp_server/perception.py` `TOF_OBSTACLE_BAND_M` = (0.10, 0.6) and the two band mentions in `nodes/mcp_server/mcp_server/tools.py` docstrings, so the agent's fixed-band summary matches the costmap.
- Tests first (red seen): `tests/test_nav_tof_voxel_layer.py` expects 0.10; `tests/test_nav_tof_collision.py` expects 0.10; `tests/test_nav_ground_voxel_layer.py` line pinning (0.06, 0.6) expects (0.10, 0.6); new `test_mcp_tof_obstacle_band_equals_tof_mark` in `tests/test_nav_tof_voxel_layer.py` reads TOF_OBSTACLE_BAND_M from perception.py (ast, no import) and asserts it equals the tof_mark band. `tests/test_tof_obstacle_wiring.py::test_stvl_and_collision_monitor_height_bands_agree` keeps passing unchanged.
- `nodes/nav2_bringup/README.md` (tof_mark row, band paragraph, collision table row) and `tests/README.md` updated.

Notes: this supersedes the 0.06 of contract nav2-tof-voxel-bands (owner epic tof-3d-obstacle-detection-fo); the new contract is tof-mark-band. Deploying mcp_server kills a running agent session (MEMORY): the runbook task checks first.

## Acceptance conditions

- a1: tof_mark starts at 0.10 m [evidence: test:tests/test_nav_tof_voxel_layer.py::test_tof_sources_settings]
- a2: collision monitor tof_obstacles starts at 0.10 m and agrees with tof_mark [evidence: test:tests/test_tof_obstacle_wiring.py::test_stvl_and_collision_monitor_height_bands_agree]
- a3: the mcp fixed-band summary uses the tof_mark band [evidence: test:tests/test_nav_tof_voxel_layer.py::test_mcp_tof_obstacle_band_equals_tof_mark]
- a4: root nav tests, root lint and mcp_server tests pass [evidence: command:uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_tof_collision.py tests/test_nav_ground_voxel_layer.py tests/test_tof_obstacle_wiring.py -q && uv run poe lint && cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_nav_tof_collision.py`
- `tests/test_nav_ground_voxel_layer.py`
- `nodes/mcp_server/mcp_server/perception.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

Part `tof-mark-band` (contract): `nav2_params.yaml` local_costmap `tof_voxel_layer.tof_mark.min_obstacle_height: 0.10`, collision_monitor `tof_obstacles.min_height: 0.10`; `nodes/mcp_server/mcp_server/perception.py::TOF_OBSTACLE_BAND_M = (0.10, 0.6)`. Keep max heights, the cliff band, voxel_decay, mark_threshold and the generated footprint blocks (scripts/gen_nav_footprint.py owns them).

## Expected outcome

The four root nav test files pass with 0.10, mcp_server tests and lint pass.

## Steps

- Update the expected values in tests/test_nav_tof_voxel_layer.py, tests/test_nav_tof_collision.py, tests/test_nav_ground_voxel_layer.py and add test_mcp_tof_obstacle_band_equals_tof_mark; run the verify pytest, see FAIL.
- Change nav2_params.yaml (values and reason comments), perception.py and the tools.py docstrings; rerun, see PASS.
- `uv run poe lint`; mcp_server tests and lint; update nodes/nav2_bringup/README.md and tests/README.md.
- Commit `fix(nav2): ToF obstacle band starts at 10 cm`.

## Notes

`test_layer_values_are_commented_with_their_reason` requires the floor reason comment on min_obstacle_height. Brief numbers: ~7 % of floor points above 6 cm per frame; 10-frame union 24 % -> 1.3 % with the depth and pan fixes and 0.10.
