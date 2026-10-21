---
title: Ground robustness docs
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The README, the ground runbook and MEMORY.md describe the rule and the noise handling that actually run on the robot."
files: [nodes/ground_estimator_cpp/README.md, tests/test_ground_estimator_readme.py, docs/runbooks/ground-estimation.md, tests/test_ground_docs.py, MEMORY.md, tests/README.md]
verify: "uv run pytest tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q && uv run poe lint"
depends_on: [node-stable-classes, costmap-cell-keyed-warnings]
owns: []
uses: [stable-warning-areas, ground-traversability-rule]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground robustness docs

## Requirement

Document the robustness changes and move every doc pin of the old 8 deg rule to 15 deg (design D2-D11).

- `nodes/ground_estimator_cpp/README.md`: a `## Robustness` section covering the temporal spread gate (`temporal.max_spread_m`, gated cells are unknown, never lethal, never an edge), the median edge step, the robust slope and `slope_known` (`edges.slope_bin_m`, `edges.slope_min_cells`), warning-only colour clusters, `traversability.lethal_min_points`, the cell class hysteresis (`hysteresis.*`, enter/exit rule, the 0.3 s lethal latency at 10 Hz) and the cell-aligned warning areas; the rule text says 15 deg and that `max_slope_deg` is bounded by the tilt_guard error limits; `fit.smoothing` 0.2; the three new gauges in the metrics table.
- `tests/test_ground_estimator_readme.py`: `test_readme_documents_robustness` pins `max_spread_m`, `lethal_min_points`, `slope_min_cells`, `hysteresis`, `enter_frames`, `roll_error_deg` and `15 deg` in the README.
- `docs/runbooks/ground-estimation.md`: the rule text, the threshold table and the Scenes use 15 deg (a 13 deg ramp warning, a ramp steeper than 15 deg an obstacle) instead of 8 / 6 deg; `tests/test_ground_docs.py::test_runbook_scenes_cover_both_sides_of_the_rule` pins `13 deg ramp` and `15 deg` instead of `6 deg ramp` and `8 deg`.
- `MEMORY.md`: a `## Ground detection robustness (2026-10-21)` section: the decisions (5 cm box stays an obstacle under the 1/3-wheel rule; max_slope_deg 15 deg (was 8 deg), bounded by the tilt_guard error limits, because the user calls the ~14 deg ramp safe; spread gate; median edge step; cell hysteresis; colour warning-only; costmap cell store) and the runbook path `docs/runbooks/ground-robustness.md`.
- `tests/README.md`: the changed and added tests.

Notes: `tests/test_ground_docs.py` and `docs/runbooks/ground-estimation.md` belong to the escalated ground-final-suite-runbook task of rgb-d-ground-estimation-and; change only the threshold pins and text named here. The MEMORY.md term `8 deg` stays present (the new section says "was 8 deg").

## Acceptance conditions

- a1: the node README documents every robustness setting and the 15 deg rule [evidence: test: tests/test_ground_estimator_readme.py::test_readme_documents_robustness]
- a2: the ground estimation runbook scenes use the 15 deg rule [evidence: test: tests/test_ground_docs.py::test_runbook_scenes_cover_both_sides_of_the_rule]
- a3: MEMORY.md records the decisions [evidence: doc-section: MEMORY.md#Ground detection robustness (2026-10-21)]
- a4: the README has the robustness section [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Robustness]

## Files

- `nodes/ground_estimator_cpp/README.md`
- `tests/test_ground_estimator_readme.py`
- `docs/runbooks/ground-estimation.md`
- `tests/test_ground_docs.py`
- `MEMORY.md`
- `tests/README.md`

## Interfaces

Docs only; doc pins in tests/test_ground_estimator_readme.py and tests/test_ground_docs.py.

## Expected outcome

`uv run pytest tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q` PASS; README, runbook and MEMORY.md state the 15 deg rule and the robustness settings.

## Steps

- add test_readme_documents_robustness and move the 6 / 8 deg scene pins to 13 / 15 deg
- run `uv run pytest tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q`, see FAIL
- write the README Robustness section, the runbook threshold text, the MEMORY.md section and tests/README.md
- run the verify command, see PASS; commit `docs(ground_estimator_cpp): robustness settings and the 15 deg rule`

## Notes

See the Notes paragraph of the Requirement; design: docs/plans/todo/ground-detection-robustness/design.md.
