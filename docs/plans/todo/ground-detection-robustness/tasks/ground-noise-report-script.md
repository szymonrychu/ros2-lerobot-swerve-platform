---
title: Ground noise report script
status: todo
repo: ros2-lerobot-sverve-platform
value: Each runbook scene gives pass/fail numbers from one command instead of ad-hoc scratch analysis.
files: [scripts/ground_noise_report.py, scripts/ground_noise_report.yaml, tests/test_ground_noise_report.py, tests/README.md]
verify: "uv run pytest tests/test_ground_noise_report.py -q && uv run poe lint && uv run ruff check scripts/ground_noise_report.py && uv run ruff format --check scripts/ground_noise_report.py"
depends_on: []
owns: []
uses: []
model: ""
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground noise report script

## Requirement

A report script that turns topic_scraper captures of `/ground/state` into the runbook's pass/fail numbers (design D12, brief fix 8).

- `scripts/ground_noise_report.py` (type hints, pydantic config, module-level functions, constants at the top, standard logging): reads one NDJSON capture file and the YAML config, prints one JSON summary and exits 0 when every metric of the selected scene passes, 1 otherwise. Input lines are records of `scripts/topic_scraper_collect.py` (read its output format; `value` holds the `/ground/state` JSON as an object or as a JSON string) and, for compatibility, web_ui `/ws` capture lines `{t, topic, data}`; lines of other topics are skipped, malformed lines are counted and reported, never fatal.
- `scripts/ground_noise_report.yaml`: the user's scene (base_link ROIs): box x 0.62-0.93, y 0.10-0.35, height 0.05 m; ramp x 1.00-1.25, y -0.45..-0.20, slope 14 deg; board x 0.40-0.55, y -0.20..0.20 (my call for y); thresholds: box_obstacle_share >= 0.95, ramp_warning_share >= 0.90, ramp_slope_abs_error_p50 <= 3 deg, board_lethal_states == 0, empty_lethal_points_p95 == 0, warning_churn_share < 0.10. CLI arguments are only the config path, the capture path and the scene name (box, ramp, board, empty).
- Metrics, computed over states that carry a plane: `box_obstacle_share` = share of states in which an obstacle-class region has its edge point in the box ROI, or obstacle_points > 0 while no warning-class region or warning-area centroid lies in the box ROI; `ramp_warning_share` = share with a `ramp` warning area whose centroid is in the ramp ROI and no obstacle-class region in the ramp ROI; `ramp_slope_abs_error_p50` and `_p90` against the configured slope; `board_lethal_states` = states with an obstacle-class region in the board ROI; `empty_lethal_points_p95` = p95 of obstacle_points + drop_points; `warning_churn_share` = share of consecutive state pairs whose warning set (kind plus polygon rounded to 1 cm) differs. Every scene reports churn.
- Tests first in `tests/test_ground_noise_report.py` with NDJSON fixtures written to tmp_path: `test_box_scene_pass_and_fail`, `test_ramp_scene_share_and_slope_error`, `test_board_scene_lethal_states`, `test_empty_scene_lethal_p95`, `test_warning_churn_share`, `test_reads_scraper_and_ws_lines`, `test_malformed_lines_counted`, `test_exit_code_reflects_thresholds`.
- `tests/README.md`: the new test file.

Notes: root `poe lint` covers only tests and shared, so the verify command lints the script explicitly with ruff. numpy, pydantic and pyyaml are root dependencies.

## Acceptance conditions

- a1: the box scene metric passes at >= 95% obstacle states and fails below [evidence: test: tests/test_ground_noise_report.py::test_box_scene_pass_and_fail]
- a2: the ramp scene reports the warning share and the slope error [evidence: test: tests/test_ground_noise_report.py::test_ramp_scene_share_and_slope_error]
- a3: the board and empty scenes report lethal states and the lethal points p95 [evidence: test: tests/test_ground_noise_report.py::test_empty_scene_lethal_p95]
- a4: warning churn is the share of consecutive states whose warning set changed [evidence: test: tests/test_ground_noise_report.py::test_warning_churn_share]
- a5: topic_scraper and web_ui capture lines both parse, malformed lines are counted [evidence: test: tests/test_ground_noise_report.py::test_reads_scraper_and_ws_lines]
- a6: the exit code is 0 only when every threshold passes [evidence: test: tests/test_ground_noise_report.py::test_exit_code_reflects_thresholds]

## Files

- `scripts/ground_noise_report.py`
- `scripts/ground_noise_report.yaml`
- `tests/test_ground_noise_report.py`
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
