---
title: MCP floor guard ground
status: todo
repo: ros2-lerobot-sverve-platform
value: "Arm motions can opt in to the measured steps and drop-offs as floor_guard terrain, refusing when the estimate is stale."
files: [nodes/mcp_server/mcp_server/floor_guard.py, nodes/mcp_server/mcp_server/grasp_tools.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/arm.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/test_floor_guard_ground.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-ground-summary]
owns: []
uses: [mcp-ground-state-client, ground-state]
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

# MCP floor guard ground

## Requirement

- floor_guard (design D9, Q2): `FloorOverride.use_ground_estimate: bool = False`; the arm tools that take `surface_z_m` / `tilt_override_deg` / `surfaces` (tools.py via `grasp_tools.floor_override`) gain a `use_ground_estimate` parameter (default false) documented for the agent: "use the measured floor regions (steps, drop-offs) from the head ToF ground estimate; refused when no fresh estimate exists".
- `FloorGuard.surface(...)` receives the latest ground state (from `RosInterface.ground_state()`, part `mcp-ground-state-client`) and, when `use_ground_estimate` is true: state fresh (age <= `floor_guard.ground_max_age_s`, default 1.0, > 0, in the floor_guard settings) and `plane` not null -> the Terrain is built from the state's regions converted to `SurfaceRegion` (frame base_link, edge, height_m relative to the robot floor) followed by the call's explicit `surfaces` (listed later, so they win), at most MAX_REGIONS (explicit ones kept first when trimming); stale, absent or plane-null -> the motion is refused with a `GuardReport`/error whose reason names `/ground/state` and the cause. With `use_ground_estimate` false nothing changes (byte-identical behaviour; existing floor_guard tests unchanged).
- Tests first in `nodes/mcp_server/tests/test_floor_guard_ground.py`, seen failing: a state with a drop region 0.3 m ahead gives a Terrain whose lookup beyond the edge returns the drop height; explicit surfaces override an overlapping ground region; region count trimmed to MAX_REGIONS keeping explicit ones; stale/absent/plane-null refusals with the reason; default false leaves the surface model equal to the current one; the tool parameter is passed through.
- `nodes/mcp_server/README.md`: the floor_guard `use_ground_estimate` option, freshness rule and refusal.

## Acceptance conditions

- a1: use_ground_estimate builds the floor_guard Terrain from fresh ground regions with explicit surfaces winning [evidence: test: nodes/mcp_server/tests/test_floor_guard_ground.py]
- a2: a requested but stale, absent or plane-less estimate refuses the motion with a reason naming /ground/state [evidence: test: nodes/mcp_server/tests/test_floor_guard_ground.py]
- a3: with the default false the floor_guard behaviour is unchanged and the full suite passes [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: the README documents the option [evidence: command: grep -q 'use_ground_estimate' nodes/mcp_server/README.md]

## Files

- `nodes/mcp_server/mcp_server/floor_guard.py`
- `nodes/mcp_server/mcp_server/grasp_tools.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/arm.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_floor_guard_ground.py`
- `nodes/mcp_server/README.md`

## Interfaces

`FloorOverride.use_ground_estimate`; `floor_guard.ground_max_age_s`; `ground_regions_to_surfaces(state) -> list[SurfaceRegion]` (floor_guard.py).

## Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_floor_guard_ground.py -q` passes and the full mcp suite stays green.

## Steps

- write tests/test_floor_guard_ground.py
- run it, see FAIL
- implement floor_guard, grasp_tools, tools, arm wiring
- run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- update the README
- commit `feat(mcp_server): floor_guard terrain from the ground estimate`.

## Notes

surfaces.py `SurfaceRegion`/`Terrain` (46-330), floor_guard.py `surface()` (550-580); arm.py passes `FloorOverride` to the guard (arm.py:1155-1190).
