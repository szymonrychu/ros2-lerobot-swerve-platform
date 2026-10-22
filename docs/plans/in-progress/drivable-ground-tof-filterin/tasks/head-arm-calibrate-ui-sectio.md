---
title: Head arm calibrate UI section
status: in-progress
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--head-arm-calibrate-ui-sectio
branch: drivable-ground-tof-filterin--head-arm-calibrate-ui-sectio
verified_head: ""
run: ""
provides: []
---

# Head arm calibrate UI section

## Requirement

The "Head from arm" section on the Calibrate page, a guided step-by-step process (design D38, D41, D46; approvals H2,
H3, H5; CLAUDE.md servo safety).

- New `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`, built like `HeadFloorFit.tsx` (MUI,
  `TOUCH_TARGET_PX` buttons, polling `getHeadArmState()` at its interval), step cards driven by `sequence.phase`:
  1. Idle: "The arm moves to the calibration pose without the board. Keep the area around the robot clear." Button
     "Move to base pose" opens a confirm dialog naming the joints that move (shoulder_pan, shoulder_lift, elbow_flex,
     wrist_flex, wrist_roll, gripper) and only on "Move" calls `moveToBase(true)`.
  2. `waiting_board`: "Clip the ChArUco board to the fixed jaw." Button "Board attached".
  3. Running tiers 0 and 1: tier, pose i of N, captured, skipped, the status line; "Stop" always visible.
  4. `waiting_big_joints`: a confirm dialog "Allow elbow and shoulder moves" naming `sequence.big_joints`; "Allow" calls
     `allowBigJoints(true)`, "Skip" calls `allowBigJoints(false)`.
  5. `holding`: "Remove the board from the gripper." Button "Park arm" opens a confirm "The board is removed" and only
     then calls `park(true)`.
  6. Compute (enabled when the sequence is not running and the session has views) -> result card: pass/fail, RMS,
     held-out RMS and board error, views, arm poses, pan/tilt span, largest correlation, a parameter table (start,
     fitted, delta, sigma, floor value and delta; mm for x/y/z, deg for angles), failed limits, the report collapsed.
  7. "Apply" only for a passed result -> the applied record (session, result, source) and the unit restart states;
     node error messages in an Alert.
- `CalibratePage.tsx` renders `HeadArmCalibration` when `config.rgbd` is true, after `HeadFloorFit`; the "Arm
  calibration - deferred" line stays for epic arm-joint-offsets-from-the-g.
- New `HeadArmCalibration.test.tsx` (mocked api): no motion call without the dialog confirm; Board attached; the
  big-joint dialog Allow and Skip; Park only after its confirm; Stop; Compute and the result table; Apply disabled for a
  failed result; error alert. `CalibratePage.test.tsx`: the section shows with rgbd and not without.

## Acceptance conditions

- a1: section tests pass (confirm dialogs gate every motion call) [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadArmCalibration.test.tsx]
- a2: the page renders the section with rgbd only [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx]
- a3: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]
- a4: Apply is offered only for a passed result [evidence: judgement]

## Files

- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

## Interfaces

- `HeadArmCalibration` component; uses `headArmApi.ts`.

## Expected outcome

The user runs the head-from-arm calibration from a browser, step by step, confirming every big-joint motion, and
applies only a passing result.

## Steps

- Write `HeadArmCalibration.test.tsx` and the page test first; run them: red.
- Implement; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx && npx tsc --noEmit && npm run lint && npm run build`;
  expect every test passed and lint clean.

## Notes

- The node enforces the confirms too (head-arm-sequence-http-api); the dialogs are the user's OK, not the only guard.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
