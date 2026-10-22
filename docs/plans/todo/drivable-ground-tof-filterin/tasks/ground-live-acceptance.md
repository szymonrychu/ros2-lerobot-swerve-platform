---
title: Ground live acceptance
status: todo
repo: ros2-lerobot-sverve-platform
value: "The robot's ground detection is proven on the real scene, so Nav2 can trust it: the acceptance moved from ground-detection-robustness, with real numbers"
files: [docs/runbooks/ground-robustness.md, tests/test_ground_robustness_runbook.py, scripts/ground_acceptance_record.yaml, tests/README.md]
verify: "uv run pytest tests/test_ground_robustness_runbook.py -q && uv run poe lint"
depends_on: [guidance-live-check-and-cpu, ground-estimator-live-scene, guidance-replay-acceptance, live-report-guidance-metrics]
owns: [live-acceptance]
uses: [record-script, live-report, baseline-fixtures, live-scene-estimator]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
