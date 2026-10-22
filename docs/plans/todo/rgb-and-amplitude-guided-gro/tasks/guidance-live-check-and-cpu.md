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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
