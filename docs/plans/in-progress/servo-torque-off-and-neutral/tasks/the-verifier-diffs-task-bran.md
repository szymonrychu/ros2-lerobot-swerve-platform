---
title: "The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing."
status: todo
repo: ros2-lerobot-sverve-platform
value: "This produces false high findings for any task whose branch was merged before its review round. The diff base should be the task's recorded base commit (here e5f2231f)."
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

# The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing.

## Requirement

The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing.

Where: workbench plan-execute verifier / scope audit diff base

Why: This produces false high findings for any task whose branch was merged before its review round. The diff base should be the task's recorded base commit (here e5f2231f).

## Acceptance conditions

- AC1: The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing. at workbench plan-execute verifier / scope audit diff base is resolved [evidence: judgement]

## Files



## Interfaces

Keep the interfaces around workbench plan-execute verifier / scope audit diff base unchanged unless resolving it requires otherwise: The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing..

## Expected outcome

The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing. at workbench plan-execute verifier / scope audit diff base is resolved, and This produces false high findings for any task whose branch was merged before its review round. The diff base should be the task's recorded base commit (here e5f2231f). no longer applies.

## Steps

1. Write a failing test that shows: The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing..
2. Fix it at workbench plan-execute verifier / scope audit diff base.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `bridge-runtime-neutral-save` during run `servo-torque-off-and-neutral`.

