---
title: "Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label"
status: todo
repo: ros2-lerobot-sverve-platform
value: head-rgbd-node-wiring should register the pair_dt label value for head_rgbd_pairs_dropped_total
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

# Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label

## Requirement

Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label

Where: docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10

Why: head-rgbd-node-wiring should register the pair_dt label value for head_rgbd_pairs_dropped_total

## Acceptance conditions

- AC1: Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label at docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10 is resolved [evidence: judgement]

## Files



## Interfaces

Keep the interfaces around docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10 unchanged unless resolving it requires otherwise: Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label.

## Expected outcome

Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label at docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10 is resolved, and head-rgbd-node-wiring should register the pair_dt label value for head_rgbd_pairs_dropped_total no longer applies.

## Steps

1. Write a failing test that shows: Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label.
2. Fix it at docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md D10.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `rgbd-motion-compensation` during run `head-rgb-d-fusion-of-camera`.

