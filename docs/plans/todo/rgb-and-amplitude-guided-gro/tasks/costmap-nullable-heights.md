---
title: Costmap nullable heights
status: todo
repo: ros2-lerobot-sverve-platform
value: The costmap slows down in no_return areas instead of rejecting the whole warning message
files: [nodes/costmap_zones/costmap_zones/ground_warnings.py, nodes/costmap_zones/tests/test_ground_warnings.py, nodes/costmap_zones/README.md]
verify: "cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Costmap nullable heights

## Requirement

costmap_zones accepts warning areas with null height and slope, and treats a `no_return` area like any warning area (design D5, D6, cross-epic contract ground-warning-no-return).

- `costmap_zones/ground_warnings.py` `GroundArea`: `height_m: float | None`, `slope_deg: float | None`, documented as "null: not measured (kind no_return)". The cell-keyed store and the rasterisation are unchanged: a no_return area lowers the speed mask and fills the ground cost like other warning areas, never lethal.
- Tests first (seen failing) in `tests/test_ground_warnings.py`: `test_null_heights_area_parses`, `test_no_return_area_is_a_slow_down_area` (its cells get the warning mask value, not lethal), and the existing tests unchanged.
- README `## Ground warning areas`: the no_return kind and the nullable fields.

## Acceptance conditions

- a1: an area with null height and slope parses [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_null_heights_area_parses]
- a2: a no_return area becomes slow-down cells, never lethal [evidence: test: nodes/costmap_zones/tests/test_ground_warnings.py::test_no_return_area_is_a_slow_down_area]
- a3: costmap_zones tests and lint pass [evidence: command: cd nodes/costmap_zones && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a4: the README documents the no_return kind [evidence: doc-section: nodes/costmap_zones/README.md#Ground warning areas]

## Files

- `nodes/costmap_zones/costmap_zones/ground_warnings.py`
- `nodes/costmap_zones/tests/test_ground_warnings.py`
- `nodes/costmap_zones/README.md`

## Interfaces

`costmap_zones/ground_warnings.py:GroundArea` with `height_m: float | None`, `slope_deg: float | None` (contract ground-warning-no-return). The rasterisation and the cell-keyed store keep their behaviour.

## Expected outcome

`cd nodes/costmap_zones && uv run --frozen pytest tests -q` PASS with the two new tests; lint clean.

## Steps

- write the two tests; `cd nodes/costmap_zones && uv run --frozen pytest tests/test_ground_warnings.py -q` FAILS (null rejected)
- make the fields optional; the command PASSES; `uv run --frozen pytest tests -q && uv run --frozen poe lint` PASS
- README; commit `feat(costmap_zones): accept no_return warning areas with null heights`

## Notes

Deploy with `scripts/deploy-nodes.sh client costmap_zones` before or with ground_estimator_cpp (orchestrator).
