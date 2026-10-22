---
title: "The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells."
status: todo
repo: ros2-lerobot-sverve-platform
value: "Operators who tune these keys would get the wrong meaning. Those files belong to guidance-config-keys, not this task."
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

# The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells.

## Requirement

The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells.

Where: nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments

Why: Operators who tune these keys would get the wrong meaning. Those files belong to guidance-config-keys, not this task.

## Acceptance conditions

- AC1: The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells. at nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments is resolved [evidence: judgement]

## Files



## Interfaces

Keep the interfaces around nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments unchanged unless resolving it requires otherwise: The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells..

## Expected outcome

The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells. at nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments is resolved, and Operators who tune these keys would get the wrong meaning. Those files belong to guidance-config-keys, not this task. no longer applies.

## Steps

1. Write a failing test that shows: The documentation for guidance.no_return.cell_min_share and clearance_cells describes different semantics from the approved requirement. It calls cell_min_share the share of samples without a return, but the requirement uses the share of no-return samples whose colour does not match the floor. It calls clearance_cells the cells kept clear of valid returns, but the requirement uses the distance from blocked lethal or warning ToF cells..
2. Fix it at nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp:NoReturnConfig comments; nodes/ground_estimator_cpp/config/ground_estimator.yaml guidance.no_return comments.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `no-return-core` during run `drivable-ground-tof-filterin`.

