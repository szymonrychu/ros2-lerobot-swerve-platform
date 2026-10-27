---
title: "The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead."
status: done
repo: ros2-lerobot-sverve-platform
value: A literal placeholder ended up in the merge command where the branch name should be.
files: [docs/plans/in-progress/drivable-ground-tof-filterin/tasks/the-dependency-merge-command.md]
verify: "true  # delivered before the 2026-10-27 ticket migration"
depends_on: []
model: ""
review: ""
verified_head: "e6cdcaed"
points: 1
expected_minutes: ""
expected_tokens: ""
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead.

## Requirement

The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead.

Where: workflow prompt generation (dependency merge commands)

Why: A literal placeholder ended up in the merge command where the branch name should be.

## Acceptance conditions

- AC1: The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead. at workflow prompt generation (dependency merge commands) is resolved [evidence: judgement]

## Files

- (none: process item)

## Interfaces

Keep the interfaces around workflow prompt generation (dependency merge commands) unchanged unless resolving it requires otherwise: The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead..

## Expected outcome

The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead. at workflow prompt generation (dependency merge commands) is resolved, and A literal placeholder ended up in the merge command where the branch name should be. no longer applies.

## Steps

1. Write a failing test that shows: The dependency merge command for costmap-nullable-heights was malformed: 'git merge --no-edit current worktree branch (committed)'. The agent merged branch drivable-ground-tof-filterin--costmap-nullable-heights instead..
2. Fix it at workflow prompt generation (dependency merge commands).
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `guidance-node-wiring` during run `drivable-ground-tof-filterin`.

## Resolution

Resolved 2026-10-23 by the orchestrator: the agent merged the right branch by hand; process issue in the workflow prompt, no code change.
