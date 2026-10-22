---
title: Guidance live check and CPU
status: todo
repo: ros2-lerobot-sverve-platform
value: Proof on the robot that the guidance works and what it costs on the Pi 5
files: [docs/runbooks/ground-guidance.md, tests/test_ground_guidance_runbook.py, tests/README.md, scripts/node_cpu_compare.yaml, MEMORY.md]
verify: "uv run pytest tests/test_ground_guidance_runbook.py -q && uv run poe lint"
depends_on: [guidance-replay-acceptance, live-report-guidance-metrics]
owns: []
uses: [record-script, live-report]
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

# Guidance live check and CPU

## Requirement

Live task: verify the guidance on the robot and measure the Pi 5 CPU (design D11, D13; goal G6). It uses parts record-script and live-report.

Precondition, checked first:
- the orchestrator has deployed ground_estimator_cpp, costmap_zones, mcp_server, web_ui, head_rgbd_cpp and arducam_tof from main after guidance-replay-acceptance, and recorded the deploy time;
- the scene is unchanged (as in capture-gated-recordings);
- robot static and charging; only the head moves; live data only through the web_ui websocket/API.

- Live check:
  1. `uv run python scripts/ground_record.py` with the four scenes, each capturing `/ground/state` lines;
  2. `uv run python scripts/ground_noise_report.py --config scripts/ground_noise_report.yaml --capture .logs/ground_recordings/<scene>.ndjson --scene <scene>` for each scene;
  3. record each summary line.
- CPU: add entries for head_rgbd_cpp, ground_estimator_cpp and arducam_tof (switch_time = the deploy time) to `scripts/node_cpu_compare.yaml` and run `uv run python -m scripts.node_cpu_compare`, as in `docs/runbooks/client-cpu-budget.md`. The before window is the 6 h before the deploy.
- Runbook `docs/runbooks/ground-guidance.md` with these sections:
  - `## Scene`: the sheet values, the head poses, the truth provenance;
  - `## Procedure`: the commands above;
  - `## Acceptance`: the report thresholds, plus a CPU added <= 0.15 core in total (D13);
  - `## Results`: a table per scene with the live numbers next to the replay ones, the CPU before/after per node and in total, and PASS/FAIL per row. A FAIL row names the metric; it is not hidden.
- `tests/test_ground_guidance_runbook.py`: `test_runbook_sections` (the four sections exist) and `test_runbook_commands_use_web_ui_only` (no `ros2 topic`, `rclpy` or `ssh` in the runbook). `tests/README.md` entry.
- `MEMORY.md`: one decision line (guidance on: amplitude gate, guided upsampling edge points, no_return areas; heights from depth only; the runbook path).

## Acceptance conditions

- a1: the runbook has its sections [evidence: test: tests/test_ground_guidance_runbook.py::test_runbook_sections]
- a2: the runbook uses web_ui only for live data [evidence: test: tests/test_ground_guidance_runbook.py::test_runbook_commands_use_web_ui_only]
- a3: the runbook records the live results and CPU numbers [evidence: doc-section: docs/runbooks/ground-guidance.md#Results]
- a4: root tests and lint pass [evidence: command: uv run pytest tests/test_ground_guidance_runbook.py -q && uv run poe lint]
- a5: every live and CPU row is measured on the robot after the deploy, failing rows named [evidence: judgement]

## Files

- `docs/runbooks/ground-guidance.md`
- `tests/test_ground_guidance_runbook.py`
- `tests/README.md`
- `scripts/node_cpu_compare.yaml`
- `MEMORY.md`

## Interfaces

No code interfaces. It produces the runbook `docs/runbooks/ground-guidance.md`, the `scripts/node_cpu_compare.yaml` entries and a MEMORY.md line. It uses part record-script and part live-report.

## Expected outcome

The runbook with Scene, Procedure, Acceptance and Results (live numbers per scene, CPU before/after per node, PASS/FAIL rows); `uv run pytest tests/test_ground_guidance_runbook.py -q` PASS.

## Steps

- write `tests/test_ground_guidance_runbook.py`; `uv run pytest tests/test_ground_guidance_runbook.py -q` FAILS (no runbook)
- check the preconditions; run the live check (record script, then the report per scene) and the CPU compare
- write the runbook with the measured numbers; the test PASSES; `uv run poe lint` PASSES
- MEMORY.md line and tests/README entry; commit `docs(runbooks): ground guidance live check and CPU`

## Notes

Live task (D11). If the robot is unreachable, report it as a blocker naming the orchestrator/user; do not write numbers that were not measured. The CPU windows follow `docs/runbooks/client-cpu-budget.md` (Mimir keeps 1 day).
