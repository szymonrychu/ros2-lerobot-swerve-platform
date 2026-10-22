---
title: Calib robot runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: The head chain measured against the arm is applied on the robot and checked with real numbers.
files: [docs/runbooks/head-arm-calibration.md]
verify: "grep -q '^## Results' docs/runbooks/head-arm-calibration.md"
depends_on: [calib-docs-and-test-index]
owns: []
uses: []
model: ""
estimate: 90
actual: ""
review: skip
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib robot runbook

## Requirement



## Acceptance conditions



## Files

- `docs/runbooks/head-arm-calibration.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
