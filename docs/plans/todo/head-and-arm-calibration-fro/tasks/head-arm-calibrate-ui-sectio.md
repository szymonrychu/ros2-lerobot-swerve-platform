---
title: Head arm calibrate UI section
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user runs the head-from-arm calibration from a browser, step by step, confirming every big-joint motion, and applies only a passing result."
files: [nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx, nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [calib-frontend-api]
owns: []
uses: [head-arm-calib-api]
model: ""
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head arm calibrate UI section

## Requirement



## Acceptance conditions



## Files

- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`
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
