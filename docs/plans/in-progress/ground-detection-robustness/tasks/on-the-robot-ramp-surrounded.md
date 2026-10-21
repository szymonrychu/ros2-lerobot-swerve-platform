---
title: "On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell."
status: todo
repo: ros2-lerobot-sverve-platform
value: "Approved item A7's value (the 14 deg ramp reads about 14 deg on the robot) depends on the temporal cell geometry, not only on the cluster slope estimator."
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell.

## Requirement

On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell.

Where: nodes/ground_estimator_cpp/src/core/cell_median.cpp and the temporal.cell_size_m default (config / slope-and-smoothing-defaults, runbook)

Why: Approved item A7's value (the 14 deg ramp reads about 14 deg on the robot) depends on the temporal cell geometry, not only on the cluster slope estimator.

## Acceptance conditions

- AC1: On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell. at nodes/ground_estimator_cpp/src/core/cell_median.cpp and the temporal.cell_size_m default (config / slope-and-smoothing-defaults, runbook) is resolved [evidence: judgement]

## Files



## Interfaces

Keep the interfaces around nodes/ground_estimator_cpp/src/core/cell_median.cpp and the temporal.cell_size_m default (config / slope-and-smoothing-defaults, runbook) unchanged unless resolving it requires otherwise: On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell..

## Expected outcome

On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell. at nodes/ground_estimator_cpp/src/core/cell_median.cpp and the temporal.cell_size_m default (config / slope-and-smoothing-defaults, runbook) is resolved, and Approved item A7's value (the 14 deg ramp reads about 14 deg on the robot) depends on the temporal cell geometry, not only on the cluster slope estimator. no longer applies.

## Steps

1. Write a failing test that shows: On the robot (ramp surrounded by floor, 10 cm temporal cells, 0.02 m floor band), a 25 cm ramp leaves only 1-2 cell columns above the floor band, and the top cell mixes ramp and floor. A Python simulation of that scene gives slope p50 4-9 deg for any estimator. This matches the measured 5.7 deg. The edges fix makes the slope unknown in that case (classified on step height only) instead of wrong. Reading about 14 deg on the robot needs temporal.cell_size_m around 0.05: simulated p50 13.1 deg with aligned cells, but only 8-10 deg with misaligned cells. Or it needs a filter that keeps the slope inside each cell..
2. Fix it at nodes/ground_estimator_cpp/src/core/cell_median.cpp and the temporal.cell_size_m default (config / slope-and-smoothing-defaults, runbook).
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `edge-step-and-ramp-slope` during run `ground-detection-robustness`.

