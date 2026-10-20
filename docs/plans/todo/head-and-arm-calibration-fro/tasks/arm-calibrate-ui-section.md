---
title: Arm calibrate UI section
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user calibrates the arm from a browser, confirming every arm motion tier, and gets the offsets to review."
files: [nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx, nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [head-calibrate-ui-section]
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

# Arm calibrate UI section

## Requirement

The "Arm calibrate" button and section as a guided process (design D7, D9, D15, D16; approval A3; CLAUDE.md servo safety).

- New `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`, a step-by-step guide:
  - "Arm calibrate" is disabled with the reason "apply a head calibration first" until the applied record's kind is `head_chain` or `arm`.
  - Step 1 (prompt card): "Take the ChArUco board off the floor and clip it to the fixed jaw; keep the area clear. The arm moves to the calibration pose, closes the gripper on the clip and runs the wrist and elbow poses slowly." A "Start" button calls `startArmSequence({allowBigJoints: false})` (tiers 0 and 1; no confirm dialog).
  - Step 2: after tiers 0 and 1, "Allow shoulder moves" opens a confirm dialog naming shoulder_lift and shoulder_pan; only that Confirm calls `startArmSequence({sessionId: <same session>, allowBigJoints: true})`. No other path sends `allowBigJoints: true`.
  - "Stop" is visible whenever a sequence runs; progress shows tier, step/total, captured, skipped, status.
  - Step 3: "Compute" -> result card: offsets table (current, new, change in deg, sigma), head x/y/yaw change, held-out tool error in mm, wrist_roll fitted or fixed at the recorded neutral, pass/fail and failed limits; the snippet in a monospace block with "Copy" (`navigator.clipboard.writeText`) and "Download" (zip); "Apply head part" only for a passed result, with a note that the offsets are committed by hand after review.
- `CalibratePage.tsx`: renders `ArmCalibration` after `HeadCalibration` when `config.rgbd`; the "Arm calibration - deferred" line is removed.
- New `ArmCalibration.test.tsx`: disabled without an applied head chain; Start sends `allowBigJoints: false` without a dialog; `allowBigJoints` true only after the shoulder confirm (never on Start); Copy writes the snippet; Apply disabled on a failed result; Stop calls the route. `CalibratePage.test.tsx`: the deferred line is gone.

## Acceptance conditions

- a1: Start runs tiers 0 and 1 without a dialog and without shoulder moves [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "start runs wrist and elbow tiers"]
- a2: shoulder moves are sent only after the shoulder confirm [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "allow big joints only after the shoulder confirm"]
- a3: Arm calibrate is disabled until a head chain is applied [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "disabled without an applied head chain"]
- a4: Copy writes the offsets snippet [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "copy writes the snippet"]
- a5: the deferred placeholder line is gone [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx -t "has no deferred arm line"]
- a6: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

## Interfaces

- `ArmCalibration.tsx::ArmCalibration` rendered by `CalibratePage` after `HeadCalibration`.

## Expected outcome

The user calibrates the arm from a browser, confirming every arm motion tier, and gets the offsets to review.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: builder check: head tools need a lease, three arm tiers with confirms, gripper set once, mcp-token via supplementary_groups, optional drive check

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
