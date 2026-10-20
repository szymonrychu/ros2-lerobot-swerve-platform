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

The "Head calibrate" button and section on the Calibrate page, a guided step-by-step process (design D9, D15; approval A3).

- New `nodes/web_ui/frontend/src/calibrate/HeadCalibration.tsx`, built like the RGB-D section (MUI, `TOUCH_TARGET_PX` buttons):
  - "Head calibrate" starts `startHeadSequence()` (disabled while a capture, compute or sequence runs); while running: spot N of `max_spots`, step/total, captured, skipped, the status line; polling `GET /api/state` at the RGB-D section's interval.
  - Guided steps: before the first spot a prompt card "Lay the ChArUco board flat on the floor at <hint 1>, then press Start"; after each spot the card "Move the board by hand to <next hint>, then press Next spot" (the base never drives); "Stop" whenever running.
  - "Compute" (enabled once 3 spots are done) -> waits for the compute in the state -> result card: pass/fail, RMS, held-out RMS, views and spots, a parameter table (value +- sigma, change against the URDF in mm or deg), failed limits, the report (collapsed).
  - "Apply" only for a passed result -> shows the applied record and the unit restart states; node error messages shown in an Alert.
- `CalibratePage.tsx` renders `HeadCalibration` when `config.rgbd` is true (after the RGB-D section).
- New `HeadCalibration.test.tsx` (mocked api): start, progress, Next spot, Stop, Compute, result table, Apply disabled for a failed result, error alert. `CalibratePage.test.tsx`: the section shows with rgbd and not without.

## Acceptance conditions

- a1: Head calibrate starts the sequence and shows progress [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadCalibration.test.tsx -t "starts the head sequence and shows progress"]
- a2: Next spot appears after a spot and calls the route [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadCalibration.test.tsx -t "next spot after a finished spot"]
- a3: Apply is disabled for a failed result [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadCalibration.test.tsx -t "apply disabled for a failed result"]
- a4: the page shows the section only with rgbd [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx -t "shows head calibration with rgbd"]
- a5: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/calibrate/HeadCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

## Interfaces

- `HeadCalibration.tsx::HeadCalibration` rendered by `CalibratePage` when `config.rgbd`.

## Expected outcome

The user calibrates the head from a browser with one button, moving only the board.

## Steps

- Tests first (red), implement, verify (`npm run build` too).
- Run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
