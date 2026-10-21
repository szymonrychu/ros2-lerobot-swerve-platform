---
title: Ground robustness docs
status: todo
repo: ros2-lerobot-sverve-platform
value: "The README, the ground runbook and MEMORY.md describe the rule and the noise handling that actually run on the robot."
files: [nodes/ground_estimator_cpp/README.md, tests/test_ground_estimator_readme.py, docs/runbooks/ground-estimation.md, tests/test_ground_docs.py, MEMORY.md, tests/README.md]
verify: "uv run pytest tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q && uv run poe lint"
depends_on: [node-stable-classes, costmap-cell-keyed-warnings]
owns: []
uses: [ground-warning-areas, ground-traversability-rule]
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
