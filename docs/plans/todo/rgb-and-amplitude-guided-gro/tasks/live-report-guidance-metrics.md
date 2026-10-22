---
title: Live report guidance metrics
status: todo
repo: ros2-lerobot-sverve-platform
value: The live check judges no_return areas against the same truth as the replay
files: [scripts/ground_noise_report.py, scripts/ground_noise_report.yaml, tests/test_ground_noise_report.py, tests/README.md]
verify: "uv run pytest tests/test_ground_noise_report.py -q && uv run poe lint && uv run ruff check scripts/ground_noise_report.py && uv run ruff format --check scripts/ground_noise_report.py"
depends_on: [capture-baseline-recordings]
owns: [live-report]
uses: [scene-truth, baseline-fixtures]
model: ""
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Live report guidance metrics

## Requirement

`scripts/ground_noise_report.py` reports the guidance metrics of a live `/ground/state` capture (part live-report, design D5, D15). It uses part scene-truth (the committed `truth.yaml` of part baseline-fixtures).

- `scripts/ground_noise_report.yaml`:
  - `truth: nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml`. When it is set, the scene ROIs (box, board, ramp, empty) come from the truth polygons instead of the hand-written rectangles. The rectangles stay as the fallback for older captures;
  - scene `ramp` gains `no_return_cover_min: 0.90`; every scene gains `no_return_stray_max: 0.05`.
- New metrics from the state's `warnings`, after the warm-up, as in the C++ harness:
  - `no_return_cover_share`: the share of states whose no_return areas cover >= 70 % of the ramp polygon, rasterised at 0.01 m;
  - `no_return_stray_share`: the share of states with a no_return area cell centre more than 0.10 m outside the ramp polygon.

  The state carries the regions but not the lethal cloud, so the edge metrics stay replay-only.
- Null heights and slopes in warnings are read as unknown (no crash, no 0).
- The summary JSON gains the new metrics; the pass/fail exit includes their thresholds.
- Tests first (seen failing) in `tests/test_ground_noise_report.py`: `test_rois_from_truth_file`, `test_no_return_cover_and_stray`, `test_null_warning_heights_are_unknown`; existing tests unchanged.
- `tests/README.md` entry updated.

## Acceptance conditions

- a1: the scene ROIs come from the truth file [evidence: test: tests/test_ground_noise_report.py::test_rois_from_truth_file]
- a2: the no-return cover and stray shares are computed and judged [evidence: test: tests/test_ground_noise_report.py::test_no_return_cover_and_stray]
- a3: null warning heights are read as unknown [evidence: test: tests/test_ground_noise_report.py::test_null_warning_heights_are_unknown]
- a4: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_noise_report.py && uv run ruff format --check scripts/ground_noise_report.py]

## Files

- `scripts/ground_noise_report.py`
- `scripts/ground_noise_report.yaml`
- `tests/test_ground_noise_report.py`
- `tests/README.md`

## Interfaces

Builds part live-report: `scripts/ground_noise_report.py` gains the `truth` config key and the `no_return_cover_share` and `no_return_stray_share` metrics in its summary JSON. The capture formats it reads (topic_scraper NDJSON and web_ui `/ws` lines) are unchanged.

## Expected outcome

`uv run pytest tests/test_ground_noise_report.py -q` PASS with the three new tests and the existing ones; root lint and ruff clean.

## Steps

- write the three tests; `uv run pytest tests/test_ground_noise_report.py -q` FAILS
- implement; the command PASSES; the ruff commands PASS
- tests/README entry; commit `feat(scripts): no_return metrics and truth ROIs in the ground report`

## Notes

The ROI rectangles were corrected by ground-detection-robustness/runbook-rois-and-ramp-limit; keep them as the fallback. Read the truth YAML with pyyaml; shapely is available for the polygon coverage.
