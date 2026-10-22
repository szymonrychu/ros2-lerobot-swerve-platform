---
title: "test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251"
status: todo
repo: ros2-lerobot-sverve-platform
value: "The full ground_estimator_cpp suite reports 8 failures there. This task changed no node code, so they come from the merged base or from timing under load."
files: [nodes/ground_estimator_cpp/test/test_node.cpp]
verify: ""
depends_on: [guidance-node-wiring]
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

# test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251

## Requirement

test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251

Where: nodes/ground_estimator_cpp/test/test_node.cpp

Why: The full ground_estimator_cpp suite reports 8 failures there. This task changed no node code, so they come from the merged base or from timing under load.

## Acceptance conditions

- AC1: test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251 at nodes/ground_estimator_cpp/test/test_node.cpp is resolved [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/test/test_node.cpp`

## Interfaces

Keep the interfaces around nodes/ground_estimator_cpp/test/test_node.cpp unchanged unless resolving it requires otherwise: test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251.

## Expected outcome

test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251 at nodes/ground_estimator_cpp/test/test_node.cpp is resolved, and The full ground_estimator_cpp suite reports 8 failures there. This task changed no node code, so they come from the merged base or from timing under load. no longer applies.

## Steps

1. Write a failing test that shows: test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251.
2. Fix it at nodes/ground_estimator_cpp/test/test_node.cpp.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `edge-points-core` during run `drivable-ground-tof-filterin`.

