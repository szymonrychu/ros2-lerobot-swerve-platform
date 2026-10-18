---
title: MCP exploration coverage
status: done
repo: ros2-lerobot-sverve-platform
value: mcp_server can tell how much of the reachable space has been surveyed and where the gaps are.
files: [nodes/mcp_server/mcp_server/exploration.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/test_exploration.py, nodes/mcp_server/tests/test_config.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py tests/test_config.py tests/test_frontiers.py -q && uv run --frozen poe lint"
depends_on: []
owns: [exploration-coverage]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--mcp-exploration-coverage
branch: driving-agent-skills-and-wor--mcp-exploration-coverage
verified_head: 3afdc710fe3fb9836874365da3d5029c0c6157a4
run: driving-agent-skills-and-wor
provides: [exploration-coverage]
acs_for_audit: []
process_gaps: []
---

# MCP exploration coverage

## Requirement

- `config.py` (D6, U8): new `ExplorationSettings(StrictModel)` under `McpServerConfig.exploration`: `survey_radius_m` 2.5 (gt 0), `rays` 72 (ge 8), `min_unsurveyed_area_m2` 0.5 (gt 0), `downsample` 2 (ge 1), `stop_margin_cell_v` 0.3 (gt 0), `max_unsurveyed_listed` 10 (ge 1), each commented.
- New pure module `exploration.py` (numpy, no ROS), constants at the top, module-level functions: `reachable_mask(grid: MapGrid, trav: np.ndarray, robot_xy: tuple[float, float]) -> np.ndarray | None` (the traversable connected component of the robot cell, snapped with frontiers.snap_to_traversable; None when the robot is outside the map); `visible_cells(grid: MapGrid, spot_xy: tuple[float, float], radius_m: float, rays: int) -> np.ndarray` (rays cast from the spot, each stopping at the first occupied or unknown cell or at the radius; the free cells passed are visible); `coverage(grid: MapGrid, reachable: np.ndarray, spots: list[tuple[float, float]], settings: ExplorationSettings) -> Coverage` with pydantic `Coverage` {`reachable_free_m2`, `surveyed_m2`, `fraction` (null when nothing is reachable), `unsurveyed_areas`: list of `UnsurveyedArea` {x, y, area_m2} largest first, at most max_unsurveyed_listed, only components of reachable unsurveyed cells >= min_unsurveyed_area_m2}. Work on a grid downsampled by `downsample` (cell is free when all source cells are free) so a Pi-sized map stays fast; reuse frontiers.py MapGrid, traversability, snap_to_traversable, cell/map conversions.
- Tests first in new `tests/test_exploration.py` and `tests/test_config.py`, seen failing: a walled room with one spot in the centre is fully covered; a wall blocks visibility (the far side is unsurveyed and reported as one area with the right centroid); unknown cells stop rays; small unsurveyed pockets below the minimum are not listed; no spots -> fraction 0 and the whole reachable region is one area; robot outside the map -> None; a 400x400 cell map with 20 spots computes in under 1 s on the dev machine; config defaults and bounds.

## Acceptance conditions

- a1: a single central spot covers a walled room fully [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k full_room]
- a2: walls and unknown cells stop visibility and the hidden side is one unsurveyed area [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k 'wall or unknown']
- a3: pockets below min_unsurveyed_area_m2 are not listed and no spots means fraction 0 [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k 'pocket or no_spots']
- a4: a 400x400 map with 20 spots computes under 1 s [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration.py -q -k performance]
- a5: exploration settings defaults and bounds [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py -q -k exploration]
- a6: mcp_server frontier tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_frontiers.py -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/exploration.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_exploration.py`
- `nodes/mcp_server/tests/test_config.py`

## Interfaces

- `mcp_server/exploration.py:reachable_mask`, `visible_cells`, `coverage`, `Coverage`, `UnsurveyedArea` (contract `exploration-coverage`); `config.py:ExplorationSettings`.

## Expected outcome

A tested coverage computation the exploration-status tool calls.

## Steps

- Write `tests/test_exploration.py` and the config tests; run them; expect failures.
- Implement; run the verify command; commit `feat(mcp_server): exploration coverage maths`.

## Notes

- Documentation of the settings lands with the tool (mcp-exploration-status-tool README AC).
- Grid conventions: frontiers.py (row = y index, UNKNOWN -1, FREE 0, OCCUPIED 100).

## Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).
