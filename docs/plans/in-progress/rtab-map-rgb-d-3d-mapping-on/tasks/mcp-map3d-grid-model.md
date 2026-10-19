---
title: Mcp map3d grid model
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The agent's 3D answers come from one tested model of measured voxels and lidar-only walls that never invents a height it did not measure."
files: [nodes/mcp_server/mcp_server/map3d.py, nodes/mcp_server/tests/test_map3d.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: [map3d-voxel-query]
uses: []
model: opus
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--mcp-map3d-grid-model
branch: rtab-map-rgb-d-3d-mapping-on--mcp-map3d-grid-model
verified_head: ""
run: ""
provides: []
---

# Mcp map3d grid model

## Requirement

- `map3d.py` (numpy/scipy only, no ROS imports, D11 and U6): `VoxelMap.from_points(xyz: np.ndarray, voxel_m: float, stamp_s: float, frame_id: str)` (occupied voxel set keyed by integer cells, column index for x/y lookups), `LidarGrid.from_occupancy(data, width, height, resolution, origin_xy, occupied_thresh=65)` (the /map grid), and pure queries:
  - `query_at(vmap, lidar, x, y, radius_m, floor_band_m=0.05) -> AtResult`: within the radius, measured obstacle voxels above the floor band (count, min/max z of their tops, highest point, colour summary when colours are given), measured floor voxels count, lidar-only occupied cells (occupied in /map with no measured voxel in the column) count, and `source` `depth_3d` | `lidar_2d` | `both` | `unknown`. Lidar-only obstacles report `height_m: null` and `min_height_m` = the lidar plane height passed in (from config), never a guessed height.
  - `free_space(vmap, lidar, x, y, radius_m, z_min, z_max, sectors=8) -> FreeSpaceResult`: per sector, the distance to the nearest measured obstacle voxel inside [z_min, z_max] or lidar-only occupied cell, `free_m` = radius when none, and `observed` (fraction of the sector's cells seen by depth or lidar; `free_m: null` when below 0.2, an unknown sector is never reported as free).
  - `status_summary(vmap, lidar) -> dict` (voxel count, bounds, age).
- Frames: the cloud must be in the configured map3d frame (`rtabmap_map`, identical to `map` by D7); `from_points(..., expected_frame)` refuses another frame_id with ValueError.
- `alignment_agreement(vmap, lidar, lidar_plane_height_m, band_m=0.10, tolerance_cells=1) -> float | None` (D7 check): of the measured obstacle columns that have a voxel within +-band_m of the lidar plane, the fraction lying on (within tolerance_cells of) an occupied /map cell; `None` when fewer than 50 such columns exist (not enough evidence, never a made-up score).
- Tests first in `tests/test_map3d.py`, seen failing: a synthetic table (top at 0.72 m) reports max height 0.72 +- voxel; a wall only in /map reports `lidar_2d` with null height; a column with both reports `both`; free_space sees the table inside the robot band and not a 2 m high shelf above `z_max`; unseen sectors give null; another frame is refused; empty map answers `unknown`; alignment_agreement is ~1.0 for a cloud whose walls coincide with /map walls, drops below 0.5 for the same cloud shifted 0.3 m, and is None with too few columns.

## Acceptance conditions

- a1: query_at reports measured heights and lidar-only cells with null height [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a2: free_space respects the height band and never calls unobserved space free [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a3: a cloud in a frame other than the configured map3d frame is refused [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a4: alignment agreement separates an aligned cloud from a shifted one and abstains on too little evidence [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/map3d.py`
- `nodes/mcp_server/tests/test_map3d.py`

## Interfaces

- `nodes/mcp_server/mcp_server/map3d.py`: `VoxelMap`, `LidarGrid`, `query_at`, `free_space`, `status_summary`, `alignment_agreement`, result dataclasses `AtResult`, `FreeSpaceResult`: the `map3d-voxel-query` part.

## Expected outcome

Pure, fast queries over the 3D map that separate measured 3D structure from lidar-only structure.

## Steps

- Write `tests/test_map3d.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_map3d.py -q`; see it fail.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Commit `feat(mcp_server): 3D map voxel model and queries`.

## Notes

- Reuse `frontiers.MapGrid` for /map decoding if it fits instead of a second OccupancyGrid parser.
- Opus: the honest-unknown rules are where the value is.
