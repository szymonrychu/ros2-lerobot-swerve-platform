---
title: ToF obstacle sector summary
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent gets a compact, testable summary of nearby 3D obstacles and cliffs per direction."
files: [nodes/mcp_server/mcp_server/perception.py, nodes/mcp_server/mcp_server/perception_models.py, nodes/mcp_server/tests/test_tof_obstacles.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_tof_obstacles.py tests/test_perception.py -q && uv run --frozen poe lint"
depends_on: []
owns: [tof-sector-summary]
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# ToF obstacle sector summary

## Requirement

Test first: `tests/test_tof_obstacles.py` fails before the function exists (D11).

- `perception.py`: `tof_obstacle_sectors(points_base: np.ndarray, obstacle_band: tuple[float, float], cliff_max_z:
  float, max_range_m: float) -> list[TofSector]` over the existing eight 45 deg `SECTOR_NAMES` (same sector maths as the
  scan summary): per sector the nearest obstacle point (horizontal distance from base_link, bearing, min and max z of
  obstacle points within 0.1 m of that distance) and the nearest cliff point distance; None fields when a sector has no
  such point; non-finite points ignored; points beyond `max_range_m` ignored.
- `perception_models.py`: pydantic `TofSector` and `Obstacles3d` (`sectors`, `cloud_age_s`, `frame`, `point_count`)
  with field descriptions written for the agent.
- `tests/test_tof_obstacles.py`: sector assignment at boundaries, nearest selection, height range, cliff detection,
  empty cloud gives all-None sectors, NaN handling, range cut.
- `tests/test_perception.py` (existing scan-sector suite) stays green.

## Acceptance conditions

- a1: tof_obstacle_sectors returns per-sector nearest obstacle with height range and nearest cliff [evidence: test: nodes/mcp_server/tests/test_tof_obstacles.py]
- a2: empty, non-finite and out-of-range points yield None fields, never zeros [evidence: test: nodes/mcp_server/tests/test_tof_obstacles.py]
- a3: Obstacles3d model exists for get_map_summary [evidence: symbol: mcp_server.perception_models.Obstacles3d]
- a4: mcp_server lint passes [evidence: command: cd nodes/mcp_server && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/perception.py`
- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/tests/test_tof_obstacles.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
