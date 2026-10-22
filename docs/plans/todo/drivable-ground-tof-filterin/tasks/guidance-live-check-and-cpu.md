---
title: Guidance live check and CPU
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Proof on the robot that the guidance works and what it costs on the Pi 5
files: [docs/runbooks/ground-guidance.md, tests/test_ground_guidance_runbook.py, tests/README.md, scripts/ground_guidance_cpu.yaml, MEMORY.md]
verify: "uv run pytest tests/test_ground_guidance_runbook.py -q && uv run poe lint"
depends_on: [guidance-replay-acceptance, live-report-guidance-metrics]
owns: []
uses: [record-script, live-report]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--guidance-live-check-and-cpu
branch: drivable-ground-tof-filterin--guidance-live-check-and-cpu
verified_head: ""
run: ""
provides: []
---

# Guidance live check and CPU

## Requirement

Live task: verify the guidance on the robot and measure the Pi 5 CPU (design D11, D13; goal G6). It uses parts record-script and live-report.

Precondition, checked first:
- the orchestrator has deployed ground_estimator_cpp, costmap_zones, mcp_server, web_ui, head_rgbd_cpp and arducam_tof from main after guidance-replay-acceptance, and recorded the time of each switching deploy (head_rgbd_cpp, arducam_tof, ground_estimator_cpp);
- the scene is unchanged (as in capture-gated-recordings);
- robot static and charging; only the head moves; live data only through the web_ui websocket/API.

- Live check:
  1. `uv run python scripts/ground_record.py` with the four scenes, each capturing `/ground/state` lines;
  2. `uv run python scripts/ground_noise_report.py --config scripts/ground_noise_report.yaml --capture .logs/ground_recordings/<scene>.ndjson --scene <scene>` for each scene;
  3. record each summary line.
- CPU, one switch per node: the three features were switched on by three deploys at different times. The orchestrator records each deploy's `date +%s` in the run notes:
  - head_rgbd_cpp: the live step 1 deploy (upsampling on);
  - arducam_tof: the live step 2 deploy (amplitude gate on);
  - ground_estimator_cpp: the final deploy (guidance on).

  A new config `scripts/ground_guidance_cpu.yaml`, read by the existing `scripts/node_cpu_compare.py` (`uv run python -m scripts.node_cpu_compare scripts/ground_guidance_cpu.yaml`), sets `window_h: 1` and `settle_min: 10`, and holds three entries (same unit before and after), each with its own `switch_time`. Per node, the before window is [switch - 1 h, switch] and the after window is [switch + 10 min, switch + 70 min]. The robot is static throughout, so the load is steady.

  The added CPU of a node is its after mean minus its before mean, in cores. The total is the sum of the three deltas and must be <= 0.15 core (D13).

  Mimir keeps 1 day, so every switch must lie between 70 min and 23 h before the run. A node outside that range gets a FAIL row "not measurable: switch outside retention" with the switch time. It is never estimated.
- Runbook `docs/runbooks/ground-guidance.md` with these sections:
  - `## Scene`: the sheet values, the head poses, the truth provenance;
  - `## Procedure`: the commands above;
  - `## Acceptance`: the report thresholds, plus a CPU added <= 0.15 core in total (D13);
  - `## Results`: a table per scene with the live numbers next to the replay ones, a CPU table per node (switch time, before mean, after mean, delta) and the total of the deltas, and PASS/FAIL per row. A FAIL row names the metric; it is not hidden.
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
- `scripts/ground_guidance_cpu.yaml`
- `MEMORY.md`

## Interfaces

No code interfaces. It produces the runbook `docs/runbooks/ground-guidance.md`, the `scripts/ground_guidance_cpu.yaml` entries and a MEMORY.md line. It uses part record-script and part live-report.

## Expected outcome

The runbook with Scene, Procedure, Acceptance and Results (live numbers per scene, CPU before/after per node, PASS/FAIL rows); `uv run pytest tests/test_ground_guidance_runbook.py -q` PASS.

## Steps

- write `tests/test_ground_guidance_runbook.py`; `uv run pytest tests/test_ground_guidance_runbook.py -q` FAILS (no runbook)
- check the preconditions; run the live check (record script, then the report per scene) and the CPU compare
- write the runbook with the measured numbers; the test PASSES; `uv run poe lint` PASSES
- MEMORY.md line and tests/README entry; commit `docs(runbooks): ground guidance live check and CPU`

## Notes

Live task (D11). If the robot is unreachable, report it as a blocker naming the orchestrator/user; do not write numbers that were not measured. The CPU windows follow `docs/runbooks/client-cpu-budget.md` (Mimir keeps 1 day).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)
