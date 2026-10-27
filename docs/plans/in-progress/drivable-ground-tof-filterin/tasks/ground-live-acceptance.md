---
title: Ground live acceptance
status: done
repo: ros2-lerobot-sverve-platform
value: "The robot's ground detection is proven on the real scene, so Nav2 can trust it: the acceptance moved from ground-detection-robustness, with real numbers"
files: [docs/runbooks/ground-robustness.md, tests/test_ground_robustness_runbook.py, scripts/ground_acceptance_record.yaml, tests/README.md]
verify: "uv run pytest tests/test_ground_robustness_runbook.py -q && uv run poe lint"
depends_on: [guidance-live-check-and-cpu, ground-estimator-live-scene, guidance-replay-acceptance, live-report-guidance-metrics]
model: ""
review: ""
verified_head: 1f35ad68592fa438d2e1cd10f76606bad24846ac
points: 5
expected_minutes: 60
expected_tokens: 3750000
points_actual: 1
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Ground live acceptance

## Requirement

Live task: the live ground acceptance moved here from ground-detection-robustness (its approval A8, runbook
`docs/runbooks/ground-robustness.md`, Results table still empty) passes on the user's scene (design D24; approval G1).
It uses parts record-script, live-report and baseline-fixtures (`truth.yaml`).

Precondition, checked first and stopped on when not met (report `blocked` with who can clear it):
- ground-estimator-live-scene and guidance-replay-acceptance are merged and the orchestrator has deployed
  ground_estimator_cpp, costmap_zones, mcp_server, web_ui, head_rgbd_cpp and arducam_tof from main (live step 3);
- the user: the scene unchanged since capture-gated-recordings (board corners within 5 mm of `truth.yaml`, checked
  with `scripts/ground_scene_truth.py` on a fresh home recording), the light on, the robot static and charging, head
  moves allowed; no arm motion, no drive command.

- `scripts/ground_acceptance_record.yaml` (the config format of `scripts/ground_record.py`, its only argument): scenes
  `box-1`, `box-2`, `board-1`, `board-2`, `empty-1`, `empty-2`, `ramp-1`, `ramp-2`, each 60 s with
  `capture_topics: [/ground/state]`, at the head poses of the gated recordings (box at the pan -30 pose of the live
  flip, board at home). Two 60 s windows per scene keep the runbook's 2 minutes per object within the recorder's
  60 s cap.
- `uv run python scripts/ground_record.py scripts/ground_acceptance_record.yaml`, then for every capture
  `uv run python scripts/ground_noise_report.py --config scripts/ground_noise_report.yaml --capture
  .logs/ground_recordings/<scene>-<n>.ndjson --scene <scene>` with the truth ROIs of `truth.yaml`.
- Acceptance, every window: box `box_obstacle_share >= 0.95` (the box is lethal); board `board_lethal_states == 0`;
  empty `empty_lethal_points_p95 == 0`; ramp `ramp_lethal_states == 0`; every scene `warning_churn_share < 0.10`.
  Thresholds stay those of `scripts/ground_noise_report.yaml`; none is loosened.
- `docs/runbooks/ground-robustness.md`: `## Scene` rewritten from `truth.yaml` (the new layout, provenance and the
  user's measured placements), `## Prerequisites` and `## Procedure` switched to the web_ui capture above (no
  topic_scraper_api toggling, no rclpy, no SSH), `## Results` filled with date, scene, window, metric, value,
  threshold and PASS/FAIL per row. The Safety section (no driving) stays.
- `tests/test_ground_robustness_runbook.py`: `test_runbook_commands_exist` covers the new config,
  `test_runbook_procedure_uses_web_ui_only` (no `ros2 topic`, `rclpy`, `ssh` or `topic_scraper_collect` in the
  Procedure) is added, the other tests keep passing; `tests/README.md` entry updated.
- A FAIL row names the metric and value; the task then escalates with the capture paths so the estimator is fixed
  again (a discovered task on ground-estimator-live-scene), never closed on a failing row.

## Acceptance conditions

- a1: the runbook Results table holds the live numbers of every scene and window [evidence: doc-section:docs/runbooks/ground-robustness.md#Results]
- a2: the procedure captures live data through web_ui only [evidence: test:tests/test_ground_robustness_runbook.py::test_runbook_procedure_uses_web_ui_only]
- a3: the runbook thresholds still equal the report config and every named script is tracked [evidence: command:uv run pytest tests/test_ground_robustness_runbook.py -q && uv run poe lint]
- a4: live, the box is lethal in >= 95 % of states, the board is never lethal, empty floor has 0 lethal points at p95 and warning churn is < 10 % in every window, measured on the robot after the deploy [evidence: judgement]

## Files

- `docs/runbooks/ground-robustness.md`
- `tests/test_ground_robustness_runbook.py`
- `scripts/ground_acceptance_record.yaml`
- `tests/README.md`

## Interfaces

Builds part live-acceptance: the PASS rows of `docs/runbooks/ground-robustness.md#Results` and
`scripts/ground_acceptance_record.yaml`. Uses parts record-script (`scripts/ground_record.py`), live-report
(`scripts/ground_noise_report.py` with truth ROIs) and baseline-fixtures (`truth.yaml`).

## Expected outcome

The ground-robustness runbook's Results table holds 8 windows x their metrics, all PASS, measured live on the new
layout; `uv run pytest tests/test_ground_robustness_runbook.py -q` PASS.

## Steps

- Check the preconditions; `uv run python scripts/ground_scene_truth.py` on a fresh home recording: board within 5 mm
  of `truth.yaml`, else stop and report the offset.
- Write the record config and the runbook test changes; `uv run pytest tests/test_ground_robustness_runbook.py -q`
  FAILS (procedure still names topic_scraper).
- Rewrite Scene, Prerequisites and Procedure; the same command PASSES.
- `uv run python scripts/ground_record.py scripts/ground_acceptance_record.yaml`; expect 8 `.ndjson` files; run the
  report per window; fill Results; commit `docs(runbooks): live ground acceptance on the new scene` with the summary
  lines in the message.

## Notes

- Live task at live step 3 (design D29): needs the user (scene in place, light on, robot static and charging, head
  moves allowed). No arm motion, no drive commands.
- Runs right after guidance-live-check-and-cpu on the same deploy; it may reuse that task's captures only when they
  cover 2 x 60 s per scene at the poses above.
- Live data only through the web_ui websocket/API (user decision, approval A8); no rclpy, `ros2` CLI or SSH.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/ground-live-acceptance-on-d4

- 2026-10-27 actuals: no agent records
