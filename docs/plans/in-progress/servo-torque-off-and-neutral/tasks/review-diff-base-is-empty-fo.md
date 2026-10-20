---
title: "Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing"
status: todo
repo: ros2-lerobot-sverve-platform
value: "This produced false 'not found in any file the diff adds or changes' findings for bridge-runtime-neutral-save; the base should be the task's recorded start commit (here e5f2231f), not merge-base with main"
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

# Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing

## Requirement

Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing

Where: workbench plan-execute review step (diff base selection)

Why: This produced false 'not found in any file the diff adds or changes' findings for bridge-runtime-neutral-save; the base should be the task's recorded start commit (here e5f2231f), not merge-base with main

## Acceptance conditions

- AC1: Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing at workbench plan-execute review step (diff base selection) is resolved [evidence: judgement]

## Files



## Interfaces

Keep the interfaces around workbench plan-execute review step (diff base selection) unchanged unless resolving it requires otherwise: Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing.

## Expected outcome

Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing at workbench plan-execute review step (diff base selection) is resolved, and This produced false 'not found in any file the diff adds or changes' findings for bridge-runtime-neutral-save; the base should be the task's recorded start commit (here e5f2231f), not merge-base with main no longer applies.

## Steps

1. Write a failing test that shows: Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing.
2. Fix it at workbench plan-execute review step (diff base selection).
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `bridge-runtime-neutral-save` during run `servo-torque-off-and-neutral`.

