---
title: Costmap slow zone cost
status: done
repo: ros2-lerobot-sverve-platform
value: "Nav2 routes around slow (risky) zones when a reasonable detour exists, while keep-outs stay hard."
files: [nodes/costmap_zones/costmap_zones/raster.py, nodes/costmap_zones/costmap_zones/config.py, nodes/costmap_zones/costmap_zones/node.py, nodes/costmap_zones/tests/test_raster.py, nodes/costmap_zones/tests/test_config.py, nodes/costmap_zones/README.md, nodes/README.md, ansible/group_vars/client.yml]
verify: "cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_costmap_zones_config.py -q && uv run poe lint-ansible"
depends_on: []
owns: [slow-zone-mask-value]
uses: []
model: opus
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--costmap-slow-zone-cost
branch: driving-agent-skills-and-wor--costmap-slow-zone-cost
verified_head: ba0f748a7be6b831cbef6ea3dec92a9162f30b11
run: driving-agent-skills-and-wor
provides: [slow-zone-mask-value]
acs_for_audit: []
process_gaps: []
---

# Costmap slow zone cost

## Requirement

- `costmap_zones/config.py` (D5, U3): new field `slow_zone_mask_value: int` with `ge=1, le=99`, default 60, documented in the class docstring: KeepoutFilter maps a mask value v to cost round(v * 254 / 100) (60 -> 152), 1..99 stay below MAX_NON_OBSTACLE 252 (passable), 100 is lethal, 0 is refused because the filter writes any non -1 value into unknown cells (0 would turn unknown space inside a slow zone into FREE).
- `raster.py`: `rasterise_keepout(geometry, zones, slow_value)` fills slow-zone cells with `slow_value` and keepout cells with KEEPOUT_INSIDE (100); keepout always wins where a keepout and a slow zone overlap (whatever the zone order); the background stays KEEPOUT_BACKGROUND (-1); a slow zone without speed_limit_pct is still costed (the Zone model already requires it). `rasterise_speed` is unchanged. Module docstring explains the three values and cites keepout_filter.cpp (process loop: -1 skipped, other values written when above the cell or the cell is unknown) and costmap_filter.cpp getMaskCost (linear 0..100 -> 0..254).
- `node.py` passes `config.slow_zone_mask_value` to `rasterise_keepout`; the mask digest/republish logic keeps working (a zone change from keepout to slow republishes).
- `ansible/group_vars/client.yml` costmap_zones config gets `slow_zone_mask_value: 60` with a one-line comment.
- `nodes/costmap_zones/README.md`: the keepout mask now holds -1 / slow value / 100, the cost mapping, how Smac Lattice (cost_penalty 2.0) weighs it (about 2.2x travel cost at 60), that the speed cap is unchanged, the unknown-space effect, and how to tune the value. `nodes/README.md` costmap_zones line updated (keepout mask: -1 outside, slow value in slow zones, 100 in keep-outs).
- Tests first in `tests/test_raster.py` and `tests/test_config.py`, seen failing: slow zone cells carry the slow value; overlapping keepout and slow -> 100 for both zone orders; background -1 outside every zone; the mask holds only {-1, slow value, 100}; speed mask unchanged for the same zones; config rejects 0 and 100 and accepts 1 and 99, default 60; node test (existing harness) publishes the slow value for a slow POI.

## Acceptance conditions

- a1: slow-zone cells carry slow_zone_mask_value in the keepout mask [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_raster.py -q -k slow]
- a2: keepout 100 wins over an overlapping slow zone in either order and the background stays -1 [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_raster.py -q -k overlap]
- a3: slow_zone_mask_value accepts 1..99 only, default 60 [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests/test_config.py -q -k slow_zone_mask_value]
- a4: client.yml sets slow_zone_mask_value and the root config test passes [evidence: command: grep -q 'slow_zone_mask_value: 60' ansible/group_vars/client.yml && uv run pytest tests/test_costmap_zones_config.py -q]
- a5: costmap_zones tests and lint pass [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a6: README explains the cost mapping and the unknown-space effect [evidence: command: grep -q 'slow_zone_mask_value' nodes/costmap_zones/README.md && grep -qi 'unknown' nodes/costmap_zones/README.md]

## Files

- `nodes/costmap_zones/costmap_zones/raster.py`
- `nodes/costmap_zones/costmap_zones/config.py`
- `nodes/costmap_zones/costmap_zones/node.py`
- `nodes/costmap_zones/tests/test_raster.py`
- `nodes/costmap_zones/tests/test_config.py`
- `nodes/costmap_zones/README.md`
- `nodes/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

- `costmap_zones/config.py:CostmapZonesConfig.slow_zone_mask_value` (contract `slow-zone-mask-value`); `raster.py:rasterise_keepout(geometry, zones, slow_value)`.

## Expected outcome

The keepout mask carries a non-lethal cost in slow zones; Nav2's planner prefers detours; keep-outs remain lethal; speed caps unchanged.

## Steps

- Extend `tests/test_raster.py` and `tests/test_config.py`; run `cd nodes/costmap_zones && uv run --frozen pytest tests -q`; expect failures.
- Implement raster, config, node, client.yml and READMEs; run the verify command; commit `feat(costmap_zones): slow zones add non-lethal path cost`.

## Notes

- Nav2 Jazzy KeepoutFilter semantics are recorded in design.md D5 (resolved unknown).
- KeepoutFilter is a plugin below inflation in both costmaps (nav2_params.yaml); inflation inflates only lethal cells.
- Do not touch the SpeedFilter or the speed mask.

## Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (real merges of the original tips; integration fixes on main).
