---
title: "GroundNode::stable_areas_fixed still applies its own kind hold (hysteresis.enter_frames) on top of the filter's kind persistence; now redundant and adds extra lag on a real kind change"
status: todo
repo: ros2-lerobot-sverve-platform
value: "Outside this task's file list; simplify once the filter's area_kind_frames is the only kind hold."
files: ["nodes/ground_estimator_cpp/src/ground_node.cpp:636-670"]
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

# GroundNode::stable_areas_fixed still applies its own kind hold (hysteresis.enter_frames) on top of the filter's kind persistence; now redundant and adds extra lag on a real kind change

## Requirement

GroundNode::stable_areas_fixed still applies its own kind hold (hysteresis.enter_frames) on top of the filter's kind persistence; now redundant and adds extra lag on a real kind change

Where: nodes/ground_estimator_cpp/src/ground_node.cpp:636-670

Why: Outside this task's file list; simplify once the filter's area_kind_frames is the only kind hold.

## Acceptance conditions

- AC1: GroundNode::stable_areas_fixed still applies its own kind hold (hysteresis.enter_frames) on top of the filter's kind persistence; now redundant and adds extra lag on a real kind change at nodes/ground_estimator_cpp/src/ground_node.cpp:636-670 is resolved [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/src/ground_node.cpp:636-670`

## Interfaces

Keep the interfaces around nodes/ground_estimator_cpp/src/ground_node.cpp:636-670 unchanged unless resolving it requires otherwise: GroundNode::stable_areas_fixed still applies its own kind hold (hysteresis.enter_frames) on top of the filter's kind persistence; now redundant and adds extra lag on a real kind change.

## Expected outcome

GroundNode::stable_areas_fixed still applies its own kind hold (hysteresis.enter_frames) on top of the filter's kind persistence; now redundant and adds extra lag on a real kind change at nodes/ground_estimator_cpp/src/ground_node.cpp:636-670 is resolved, and Outside this task's file list; simplify once the filter's area_kind_frames is the only kind hold. no longer applies.

## Steps

1. Write a failing test that shows: GroundNode::stable_areas_fixed still applies its own kind hold (hysteresis.enter_frames) on top of the filter's kind persistence; now redundant and adds extra lag on a real kind change.
2. Fix it at nodes/ground_estimator_cpp/src/ground_node.cpp:636-670.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `warning-area-stability` during run `ground-detection-robustness`.

