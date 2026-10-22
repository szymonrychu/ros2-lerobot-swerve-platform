---
title: Runbook ROIs and ramp limit
status: todo
repo: ros2-lerobot-sverve-platform
value: The live runbook measures the scene where the objects really are and accepts the ramp the ToF cannot see as long as it is never lethal.
files: [scripts/ground_noise_report.py, scripts/ground_noise_report.yaml, tests/test_ground_noise_report.py, tests/test_ground_robustness_runbook.py, docs/runbooks/ground-robustness.md, tests/README.md]
verify: "uv run pytest tests/test_ground_noise_report.py tests/test_ground_robustness_runbook.py -q && uv run poe lint"
depends_on: [ground-robustness-runbook]
owns: []
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

# Runbook ROIs and ramp limit

## Requirement

Move the static runbook and its report to the measured ground truth and make the ramp acceptance "never lethal" with a documented sensor limit (design D19, brief D2).

- `scripts/ground_noise_report.yaml` scene ROIs (base_link): box x 0.60-0.78, y 0.05-0.33; board x 0.37-0.61, y -0.12..+0.14; ramp x 0.75-1.10, y -0.45..-0.15 (height and slope fields kept). Thresholds: `ramp_lethal_states_max: 0` replaces `ramp_warning_share_min` and `ramp_slope_abs_error_p50_max` as the ramp gates.
- `scripts/ground_noise_report.py`: `ReportConfig`/thresholds model updated (pydantic, extra keys rejected); ramp scene metric `ramp_lethal_states` = states with an obstacle region whose edge point lies in the ramp ROI (like `board_lethal_states`), gated `<= ramp_lethal_states_max`; `ramp_warning_share` and the slope error p50/p90 stay in the report as informational metrics, not gated. Warning churn stays a gate in every scene.
- Tests first (seen failing) in `tests/test_ground_noise_report.py`: `test_ramp_scene_never_lethal` (passes with no ramp area at all, fails with one obstacle region in the ramp ROI) and `test_scene_rois_match_ground_truth` (the yaml ROIs equal the values above); the existing ramp test updated to the informational metrics.
- `docs/runbooks/ground-robustness.md`: Scene table to the measured truth (and the robot's own parked gripper at x 0.30-0.36 reading +3..+11 cm until the head floor fit of head-chain-floor-plane-fit-a is applied); Acceptance: ramp never lethal; a new `## Sensor limits` section (no ToF returns on black rubber below the confidence threshold, so the ramp is invisible to the ToF and only "never lethal" is checkable; the ChArUco board reads -4..+3 cm from albedo bias); Results rows for the ramp become `ramp_lethal_states` (gate) plus `ramp_warning_share` (info). `tests/test_ground_robustness_runbook.py`: thresholds-match test updated, new `test_runbook_documents_ramp_sensor_limit`, `## Sensor limits` added to the required sections.
- `tests/README.md`: entries of the changed and new tests.

## Acceptance conditions

- a1: the ramp scene passes without any ramp area and fails on an obstacle region in the ramp ROI [evidence: test: tests/test_ground_noise_report.py::test_ramp_scene_never_lethal]
- a2: the report ROIs are the measured ground truth [evidence: test: tests/test_ground_noise_report.py::test_scene_rois_match_ground_truth]
- a3: the runbook documents the black rubber sensor limit and the ramp acceptance [evidence: test: tests/test_ground_robustness_runbook.py::test_runbook_documents_ramp_sensor_limit]
- a4: the runbook has a Sensor limits section [evidence: doc-section: docs/runbooks/ground-robustness.md#Sensor limits]
- a5: root lint passes [evidence: command: uv run poe lint]

## Files

- `scripts/ground_noise_report.py`
- `scripts/ground_noise_report.yaml`
- `tests/test_ground_noise_report.py`
- `tests/test_ground_robustness_runbook.py`
- `docs/runbooks/ground-robustness.md`
- `tests/README.md`

## Interfaces

`scripts/ground_noise_report.yaml` thresholds gain `ramp_lethal_states_max` and lose `ramp_warning_share_min` and `ramp_slope_abs_error_p50_max`; the report JSON keeps every metric name and adds `ramp_lethal_states`. The runbook commands are unchanged.

## Expected outcome

`uv run pytest tests/test_ground_noise_report.py tests/test_ground_robustness_runbook.py -q` PASS with the new tests; `uv run poe lint` clean; the runbook shows the measured scene and the ramp sensor limit.

## Steps

- write the new and updated tests; `uv run pytest tests/test_ground_noise_report.py tests/test_ground_robustness_runbook.py -q` FAILS on them
- update the yaml, the report and the runbook; the command PASSES; `uv run poe lint` PASSES
- update tests/README.md; commit `fix(ground_noise_report): measured ROIs and ramp never lethal`

## Notes

Ground truth and the ramp ruling are in the brief (ground-live.md) and design D14, D19. The runbook stays static: no driving, head moves only.
