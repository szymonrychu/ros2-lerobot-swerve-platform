---
title: "test_node.cpp has failing tests: NodeTest stamp-limit checks at lines 997 and 1002, and PlaneLossResetsTheCellHistoryAndGauge at line 1251"
status: done
repo: ros2-lerobot-sverve-platform
value: "The full ground_estimator_cpp suite reports 8 failures there. This task changed no node code, so they come from the merged base or from timing under load."
files: [nodes/ground_estimator_cpp/test/test_node.cpp]
verify: ""
depends_on: [guidance-node-wiring]
model: ""
review: ""
verified_head: e358f01af44ec6e19b191646d795e57257e376ce
points: ""
expected_minutes: ""
expected_tokens: ""
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
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


## Resolution

Resolved 2026-10-23 by the orchestrator: on main after the run-1 merge, scripts/cpp-node-test.sh ground_estimator_cpp gave 456 tests, 0 failures (and 376/0 before it); the stamp-limit and PlaneLoss failures reproduced only under machine load in one worktree run. No code change.
