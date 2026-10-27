---
title: "Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so"
status: done
repo: ros2-lerobot-sverve-platform
value: Caused by my merge command running outside a worktree after cd failed
files: [docs/plans/in-progress/drivable-ground-tof-filterin/tasks/main-checkout-has-an-acciden.md]
verify: "true  # delivered before the 2026-10-27 ticket migration"
depends_on: []
model: ""
review: ""
verified_head: ""
points: 1
expected_minutes: ""
expected_tokens: ""
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so

## Requirement

Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so

Where: /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main)

Why: Caused by my merge command running outside a worktree after cd failed

## Acceptance conditions

- AC1: Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so at /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main) is resolved [evidence: judgement]

## Files

- (none: process item)

## Interfaces

Keep the interfaces around /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main) unchanged unless resolving it requires otherwise: Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so.

## Expected outcome

Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so at /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main) is resolved, and Caused by my merge command running outside a worktree after cd failed no longer applies.

## Steps

1. Write a failing test that shows: Main checkout has an accidental merge commit 560f9a53 on main; needs reset to b73a4f24 by someone allowed to do so.
2. Fix it at /Users/szymonri/Documents/ros2-lerobot-sverve-platform (main).
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `head-arm-sequence-http-api` during run `drivable-ground-tof-filterin`.

## Resolution

Resolved 2026-10-23 by the orchestrator: the stray merge 560f9a53 brought only verified, clean task work (head-arm-compute-and-apply and its merged dependencies, branch head equal to the merged commit); main checks passed with it (root 2094, head_rgbd_calibration 246, web_ui 1071, ground_estimator_cpp 376); kept as an early merge, history not rewritten.
