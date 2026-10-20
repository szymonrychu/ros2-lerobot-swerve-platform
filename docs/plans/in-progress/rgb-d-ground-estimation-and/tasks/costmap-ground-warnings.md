---
title: Costmap ground warnings
status: todo
repo: ros2-lerobot-sverve-platform
value: "Nav2 slows down over traversable steps, ramps and flagged flat objects through the existing speed limit, with a passable cost band, and keeps the global costmap untouched."
files: [nodes/costmap_zones/costmap_zones/config.py, nodes/costmap_zones/costmap_zones/node.py, nodes/costmap_zones/costmap_zones/raster.py, nodes/costmap_zones/costmap_zones/ground_warnings.py, nodes/costmap_zones/tests/test_ground_warnings.py, nodes/costmap_zones/README.md, ansible/group_vars/client.yml]
verify: "(cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint) && uv run poe test-ansible"
depends_on: [ground-node-type]
owns: [ground-cost-mask]
uses: [ground-warning-areas, ground-topics, ground-node-unit]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rgb-d-ground-estimation-and--costmap-ground-warnings
branch: rgb-d-ground-estimation-and--costmap-ground-warnings
verified_head: ""
run: rgb-d-ground-estimation-and
provides: []
acs_for_audit: []
process_gaps: []
---

# Costmap ground warnings

## Requirement

- costmap_zones (design D15) stays the single writer of the Nav2 speed mask and adds the ground warning areas (slow-down, clear to ride through) next to the POI slow zones:
  - config (`costmap_zones/config.py`, pydantic, and the client.yml costmap_zones config block with comments): `ground_warnings_topic` `/ground/warning_areas`, `ground_warning_ttl_s` 3.0 (> 0), `ground_warning_speed_pct` 40 (10..100), `ground_warning_mask_value` 40 (1..99; KeepoutFilter maps 40 to cost ~101, far below INSCRIBED 253, so the area stays passable), `ground_cost_mask_topic` `/costmap_zones/ground_cost_mask`, `ground_cost_info_topic` `/costmap_filter_info/ground_cost`, `ground_mask_max_rate_hz` 2.0 (> 0).
  - `costmap_zones/ground_warnings.py`: pydantic model of the `/ground/warning_areas` JSON (`{stamp_ns, frame, areas: [{kind, polygon: [[x, y], ...], height_m, slope_deg, source}]}`, extra forbidden, contract `ground-warning-areas` of ground-state-json); `GroundWarningStore` keeping each received message's areas until `ground_warning_ttl_s` after receipt (monotonic), `active()` pruning expired ones; a message whose `frame` is not the `/map` header frame is ignored, logged once per frame and counted.
  - masks: the speed mask gets `ground_warning_speed_pct` inside active warning polygons (lowest percent wins with POI slow zones; keep-outs untouched); a new local-only ground cost mask (background `-1`, `ground_warning_mask_value` inside the polygons) on `ground_cost_mask_topic` with its CostmapFilterInfo (type 0, base 0.0, multiplier 1.0) on `ground_cost_info_topic`, published once with the first mask like the existing infos. Masks are republished when the active area set changes (receipt or expiry), at most `ground_mask_max_rate_hz`; an empty set after expiry republishes the cleared masks (real state); nothing is published before `/map` (no placeholder masks). The keepout mask (global and local costmaps) never carries ground areas: the global costmap stays static + lidar.
  - metrics: `costmap_zones_ground_areas` gauge (active areas), `costmap_zones_ground_ignored_total{reason}`.
- Tests first in `nodes/costmap_zones/tests/test_ground_warnings.py`, seen failing: message parsing and rejection; TTL expiry; frame mismatch ignored; a warning polygon sets the speed mask to 40 and the ground cost mask to 40 inside, -1/0 outside; overlap with a 30 % POI slow zone keeps 30; keepout mask unchanged by ground areas; rate limit; expiry republishes cleared masks; no mask before /map; config defaults equal the ground_estimator_cpp topic (`nodes/ground_estimator_cpp/config/ground_estimator.yaml` output_namespace + `/warning_areas`) and the client.yml block.
- `nodes/costmap_zones/README.md`: the ground warning input, masks, TTL, values and why the keepout mask is not used.

## Acceptance conditions

- a1: active ground warning areas lower the speed mask to ground_warning_speed_pct (lowest wins with POI slow zones) and fill a local-only ground cost mask with a non-lethal value [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py]
- a2: areas expire after the TTL, frame mismatches are ignored, masks are rate-limited and never published before /map; the keepout mask never carries ground areas [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py]
- a3: the costmap_zones suite and lint pass [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: the README documents the ground warning input and masks [evidence: command: grep -q ground_warning nodes/costmap_zones/README.md]

## Files

- `nodes/costmap_zones/costmap_zones/config.py`
- `nodes/costmap_zones/costmap_zones/node.py`
- `nodes/costmap_zones/costmap_zones/raster.py`
- `nodes/costmap_zones/costmap_zones/ground_warnings.py`
- `nodes/costmap_zones/tests/test_ground_warnings.py`
- `nodes/costmap_zones/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

`/costmap_zones/ground_cost_mask`, `/costmap_filter_info/ground_cost` (part `ground-cost-mask`); consumes `/ground/warning_areas` (part `ground-warning-areas`).

## Expected outcome

`cd nodes/costmap_zones && uv run --frozen pytest tests -q` passes; with ground_estimator_cpp running, /costmap_zones/ground_cost_mask and the speed mask carry the warning areas.

## Steps

- write tests/test_ground_warnings.py
- run `cd nodes/costmap_zones && uv run --frozen pytest tests/test_ground_warnings.py -q`, see FAIL
- implement ground_warnings.py, config, raster and node wiring until green; add the client.yml keys; run the verify command
- update the README; commit `feat(costmap_zones): ground warning slow-down masks`

## Notes

costmap_zones README "Slow-zone path cost" (mask 0..100 -> cost 0..254, value 60 -> 152) and raster.py `rasterise_speed` / `fill_zone`. The SpeedFilter of the local costmap turns the speed mask into `/speed_limit`, which controller_server (MPPI) honours (nav2_params.yaml speed_limit_topic).

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on ground-node-type
