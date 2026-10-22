---
title: Board on gripper hardware step
status: todo
repo: ros2-lerobot-sverve-platform
value: "The board clip, its pose on the jaw and the mount's tilt limit are recorded from the user, so the pose set and the safety check use real geometry."
files: [docs/runbooks/head-arm-calibration.md]
verify: "test -f docs/runbooks/head-arm-calibration.md && grep -q '^## Hardware' docs/runbooks/head-arm-calibration.md"
depends_on: []
owns: []
uses: []
model: ""
estimate: 30
actual: ""
review: skip
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Board on gripper hardware step

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
