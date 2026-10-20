---
title: Head calibrate UI section
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user calibrates the head from a browser with one button, moving only the board."
files: [nodes/web_ui/frontend/src/calibrate/HeadCalibration.tsx, nodes/web_ui/frontend/src/calibrate/HeadCalibration.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [calib-frontend-api]
owns: []
uses: [head-arm-calib-api]
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

# Head calibrate UI section

## Requirement

The "Head calibrate" button and section on the Calibrate page (design D9; approval A3).

- New `nodes/web_ui/frontend/src/calibrate/HeadCalibration.tsx`, built like the RGB-D section (MUI, `TOUCH_TARGET_PX` buttons):
  - "Head calibrate" starts `startHeadSequence()` (disabled while a capture, compute or sequence runs); while running: spot N of `max_spots`, step/total, captured, skipped, the status line; polling `GET /api/state` at the RGB-D section's interval.
  - After a spot: the status hint ("place the board at ...") and a "Next spot" button; "Stop" whenever running.
  - "Compute" (enabled once 3 spots are done) -> waits for the compute in the state -> result card: pass/fail, RMS, held-out RMS, views and spots, a parameter table (value +- sigma, change against the URDF in mm or deg), failed limits, the report (collapsed).
  - "Apply" only for a passed result -> shows the applied record and the unit restart states; node error messages shown in an Alert.
- `CalibratePage.tsx` renders `HeadCalibration` when `config.rgbd` is true (after the RGB-D section).
- New `HeadCalibration.test.tsx` (mocked api): start, progress, Next spot, Stop, Compute, result table, Apply disabled for a failed result, error alert. `CalibratePage.test.tsx`: the section shows with rgbd and not without.

Steps: tests first (red), implement, verify (`npm run build` too).

## Acceptance conditions

- a1: Head calibrate starts the sequence and shows progress [evidence: test:nodes/web_ui/frontend/src/calibrate/HeadCalibration.test.tsx::starts the head sequence and shows progress]
- a2: Next spot appears after a spot and calls the route [evidence: test:nodes/web_ui/frontend/src/calibrate/HeadCalibration.test.tsx::next spot after a finished spot]
- a3: Apply is disabled for a failed result [evidence: test:nodes/web_ui/frontend/src/calibrate/HeadCalibration.test.tsx::apply disabled for a failed result]
- a4: the page shows the section only with rgbd [evidence: test:nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx::shows head calibration with rgbd]
- a5: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/calibrate/HeadCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadCalibration.test.tsx`
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
