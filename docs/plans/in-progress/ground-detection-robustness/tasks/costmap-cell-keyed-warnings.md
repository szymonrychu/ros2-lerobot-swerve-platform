---
title: Costmap cell keyed warnings
status: done
repo: ros2-lerobot-sverve-platform
value: "Slow-down masks stop being republished at the 2 Hz cap from 30-60 overlapping areas; they change only when the warned cells change."
files: [nodes/costmap_zones/costmap_zones/ground_warnings.py, nodes/costmap_zones/costmap_zones/node.py, nodes/costmap_zones/costmap_zones/raster.py, nodes/costmap_zones/costmap_zones/metrics.py, nodes/costmap_zones/costmap_zones/config.py, nodes/costmap_zones/tests/test_ground_warnings.py, nodes/costmap_zones/README.md, ansible/group_vars/client.yml]
verify: "(cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint) && uv run poe test-ansible"
depends_on: [slope-and-smoothing-defaults]
owns: []
uses: []
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--costmap-cell-keyed-warnings
branch: ground-detection-robustness--costmap-cell-keyed-warnings
verified_head: 9938a5348c1ebe05b02ebc2e5a404400292492bf
run: ground-detection-robustness
provides: []
acs_for_audit: []
process_gaps: []
---

# Costmap cell keyed warnings

## Requirement

costmap_zones keys ground warnings by map cell instead of the 3 s union of whole messages, and republishes only when the active cell set changes (design D9, brief fix 5).

- `costmap_zones/ground_warnings.py`: `GroundWarningStore` keeps, per map-grid cell (pixel of the current /map geometry), the monotonic time it was last covered by an area of an accepted message (`raster.polygon_pixels` / `fill_zone` for the rasterisation). `receive(data, map_frame, geometry)` stores the cells of every area of the message; `active_cells()` returns the cells covered within the last `ground_warning_ttl_s` seconds as a boolean mask of the map shape (or an equivalent frozenset of flat indices) and drops expired ones; a geometry change (geometry_key) clears the store. `active_key` becomes a digest of the active cell set. Parsing and the frame-mismatch and no-map handling are unchanged.
- `costmap_zones/node.py`: `update()` keys on (geometry, zones digest, active cell digest); the speed and ground cost masks are built from the active cell mask (`raster.py`: add the cell-mask variants of `rasterise_speed` / `rasterise_ground_cost`, or make them accept a mask; the keepout mask never carries ground cells); the rate limit `ground_mask_max_rate_hz` and the per-mask digest dedupe stay. `GROUND_AREAS` keeps reporting the areas of the latest accepted message; add a `GROUND_CELLS` gauge (active cells) in `metrics.py`.
- `costmap_zones/config.py`: `ground_warning_ttl_s` documented as the seconds a map cell stays active after the last message that covered it; default 3.0 unchanged (my call). `ansible/group_vars/client.yml` costmap_zones comment for `ground_warning_ttl_s` updated to match (value unchanged).
- Tests first in `tests/test_ground_warnings.py`, seen failing first: `test_jittered_polygons_same_cells_publish_once` (20 messages whose polygons jitter by +-1 mm inside the same cells -> one mask publish); `test_alternating_area_sets_same_cells_no_churn` (messages alternating between two area lists that cover the same cells -> no republish after the first); `test_cell_expires_after_ttl` (a cell not covered again for ttl_s leaves the masks, one republish); `test_many_messages_bounded_by_cell_union` (30 messages each with a different small area at the same place -> the active cells equal the union of their pixels, not 30 entries); `test_geometry_change_clears_cells`; `test_cells_gauge`; the existing ground tests adapted to the cell store with the same intent.
- `nodes/costmap_zones/README.md`: a `## Ground warning cells` section: the cell store, the TTL per cell and the new gauge.

Interfaces: input contract ground-warning-areas unchanged (`GroundWarningMessage`, `GroundArea`, extra="forbid"); the published masks and topics unchanged.

Notes: live: costmap_zones kept each message 3 s, so 30-60 areas were active at once and masks republished at the 2 Hz cap. The changed-mask publishing of client-cpu-budget-recovery-a (costmap-zones-changed-masks, merged) stays. client.yml is edited after slope-and-smoothing-defaults (dependsOn).

## Acceptance conditions

- a1: polygons that jitter inside the same map cells publish the masks once [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_jittered_polygons_same_cells_publish_once]
- a2: area lists alternating over the same cells do not churn the masks [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_alternating_area_sets_same_cells_no_churn]
- a3: a cell expires ttl_s after the last message covering it [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_cell_expires_after_ttl]
- a4: many messages at one place are bounded by their cell union [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_many_messages_bounded_by_cell_union]
- a5: a geometry change clears the cell store [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_geometry_change_clears_cells]
- a6: the active cells gauge is published [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_cells_gauge]
- a7: the costmap_zones README documents the cell store [evidence: doc-section: nodes/costmap_zones/README.md#Ground warning cells]

## Files

- `nodes/costmap_zones/costmap_zones/ground_warnings.py`
- `nodes/costmap_zones/costmap_zones/node.py`
- `nodes/costmap_zones/costmap_zones/raster.py`
- `nodes/costmap_zones/costmap_zones/metrics.py`
- `nodes/costmap_zones/costmap_zones/config.py`
- `nodes/costmap_zones/tests/test_ground_warnings.py`
- `nodes/costmap_zones/README.md`
- `ansible/group_vars/client.yml`

## Interfaces

Input contract ground-warning-areas unchanged (GroundWarningMessage, GroundArea); GroundWarningStore keyed by map cell; new GROUND_CELLS gauge.

## Expected outcome

`cd nodes/costmap_zones && uv run --frozen pytest tests -q` PASS with the new cell-store tests; masks republish only when the active cell set changes.

## Steps

- write the new tests in tests/test_ground_warnings.py and adapt the existing ground tests
- run `cd nodes/costmap_zones && uv run --frozen pytest tests/test_ground_warnings.py -q`, see FAIL
- implement the cell store, the mask key and the raster helper; update config.py, metrics.py, README and the client.yml comment
- run the verify command, see PASS; commit `feat(costmap_zones): cell-keyed ground warnings`

## Notes

See the Notes paragraph of the Requirement; design: docs/plans/todo/ground-detection-robustness/design.md.

- 2026-10-22 run ground-detection-robustness: dependency-failed on slope-and-smoothing-defaults
