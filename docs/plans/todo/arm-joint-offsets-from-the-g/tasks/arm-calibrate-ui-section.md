---
title: Arm calibrate UI section
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user calibrates the arm offsets from a browser, confirming every big-joint motion, and gets the offsets to review."
files: [nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx, nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [arm-offsets-compute-apply]
owns: []
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Arm calibrate UI section

## Requirement



## Acceptance conditions



## Files

- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
