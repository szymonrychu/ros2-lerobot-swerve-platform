---
title: "Design D10 drop-reason label list (no_tf, motion, no_calibration, no_camera_info, size_mismatch) omits pair_dt, which this task's requirement adds as a metric label"
status: done
repo: ros2-lerobot-sverve-platform
value: head-rgbd-node-wiring should register the pair_dt label value for head_rgbd_pairs_dropped_total
files: [docs/head-rgbd-fusion-methodology.md]
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
verified_head: c152d490bdb67a0c5265659d53469f49cd09bca7
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

- docs/head-rgbd-fusion-methodology.md


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


## Ruling

Folded into rgbd-final-suite-and-runbook (run 3 planText note) or a workflow process note; not a separate task.

- 2026-10-22 closing (user answer A): no code change; resolved by c152d490bdb67a0c5265659d53469f49cd09bca7 (pair_dt documented in docs/head-rgbd-fusion-methodology.md and nodes/head_rgbd_cpp/README.md), recorded as verified_head.
