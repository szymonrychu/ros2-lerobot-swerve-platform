---
title: "On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell."
status: done
repo: ros2-lerobot-sverve-platform
value: "Approved item A7's value (the 14 deg ramp reads about 14 deg on the robot) depends on the temporal cell geometry, not only on the cluster slope estimator."
files: [nodes/ground_estimator_cpp/src/core/cell_median.cpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp, nodes/ground_estimator_cpp/test/core/test_cell_median.cpp, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [cell-spread-gate, edge-step-and-ramp-slope]
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--on-the-robot-ramp-surrounded
branch: ground-detection-robustness--on-the-robot-ramp-surrounded
verified_head: 2fad7eab580a00d7de146c9c20e57afc6ebbc9bd
run: ground-detection-robustness
provides: []
acs_for_audit: [a3]
process_gaps: []
---

# On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell.

## Requirement

On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell.

Where: nodes/ground_estimator_cpp/src/core/cell_median.cpp and the temporal.cell_size_m default (config / slope-and-smoothing-defaults, runbook)

Why: Approved item A7's value (the 14 deg ramp reads about 14 deg on the robot) depends on the temporal cell geometry, not only on the cluster slope estimator.

Orchestrator scope (2026-10-22): keep the slope inside each temporal cell so a 25 cm ramp of ~14 deg (63 mm over 250 mm, user ground truth) reads 14 +- 3 deg on the robot scene: per cell, keep a robust local plane (or the per-frame median height plus the in-cell height gradient) instead of collapsing a cell to one height, so the edges/cluster slope estimator sees the ramp's slope, OR reduce temporal.cell_size_m to 0.05 if the simulation shows it reaches 14 +- 3 deg with misaligned cells and the spread gate and CPU stay within budget (ground_estimator <= 0.10 core); pick the option with evidence (Python simulation of the scene with misaligned cells, as in the finding) and record it in the README. Tests: synthetic 25 cm, 14 deg ramp surrounded by floor with per-point noise sigma 0.02 m, random cell alignment over 20 seeds: slope p50 within 14 +- 3 deg; flat floor and the 5 cm box unaffected (existing tests).

## Acceptance conditions

- a1: a synthetic 25 cm, 14 deg ramp surrounded by floor (sigma 0.02 m per point, random cell alignment, 20 seeds) reads slope p50 within 14 +- 3 deg through the temporal filter and the cluster slope estimator [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: flat floor, the 5 cm box and the existing filter tests stay green [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the README records the chosen option (in-cell slope or smaller cells) with the simulation evidence [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/src/core/cell_median.cpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_median.cpp`
- `nodes/ground_estimator_cpp/README.md`



## Interfaces

Keep the interfaces around nodes/ground_estimator_cpp/src/core/cell_median.cpp and the temporal.cell_size_m default (config / slope-and-smoothing-defaults, runbook) unchanged unless resolving it requires otherwise: On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell..

## Expected outcome

The 14 deg ramp on the robot reads about 14 deg instead of 4-9 deg, so the slope rule classifies it.

## Steps

1. Simulate the scene (misaligned cells) for both options and pick one with evidence.
2. Write the failing ramp test, implement, keep the existing tests green.
3. Update the README, run the verify command, commit.

## Notes

Discovered in task `edge-step-and-ramp-slope` during run `ground-detection-robustness`.

- 2026-10-22 run ground-detection-robustness: dependency-failed on cell-spread-gate
