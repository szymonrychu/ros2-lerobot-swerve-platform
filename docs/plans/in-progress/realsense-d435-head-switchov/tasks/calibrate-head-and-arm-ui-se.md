---
title: Calibrate head and arm UI sections
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The user calibrates head and arm on the D435 from a browser, confirming every big-joint motion."
files: [nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx, nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx, nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx, nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx, nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx, nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx src/calibrate/HeadFloorFit.test.tsx src/calibrate/HeadArmCalibration.test.tsx && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [arm-offsets-compute-apply, floor-sweep-on-d435-depth, head-arm-calibration-on-d435, calibrate-rgb-d-section-on-d]
model: opus
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Calibrate head and arm UI sections

## Requirement

Goal G6 and G8: "... the web_ui calibration screen drives it." User decision 3 (absorbed arm-joint-offsets-from-the-g task `arm-calibrate-ui-section`). Design D9, D10. CLAUDE.md servo safety.

- `HeadFloorFit.tsx`: text says the floor plane comes from the D435 depth; no ToF wording. `HeadArmCalibration.tsx`: text says the D435 watches the board on the fixed jaw, and shows the "inside D435 min-Z / outside colour FOV" skip reasons from the sequencer.
- New `ArmCalibration.tsx` (the absorbed task, re-based on the D435), a step-by-step guide:
  - "Arm calibrate" is disabled with the reason "apply a head calibration first" until the applied record's kind is `head_chain` or `arm`.
  - Step 1: "Take the ChArUco board off the floor and clip it to the fixed jaw; keep the area clear. The arm moves to the calibration pose, closes the gripper on the clip and runs the wrist and elbow poses slowly." "Start" calls `startArmSequence({allowBigJoints: false})` (tiers 0 and 1, no confirm dialog).
  - Step 2: after tiers 0 and 1, "Allow shoulder moves" opens a confirm dialog naming shoulder_lift and shoulder_pan. Only that Confirm calls `startArmSequence({sessionId, allowBigJoints: true})`; no other path sends `allowBigJoints: true`.
  - "Stop" is visible while a sequence runs; progress shows tier, step/total, captured, skipped (with reasons), status.
  - Step 3: "Compute" -> a result card: offsets table (current, new, change in deg, sigma); tool point (current, new, held-out mm, fitted or kept); head x/y/yaw change; held-out tool error in mm; wrist_roll fitted or fixed; pass/fail and failed limits; the snippet in a monospace block with "Copy" (`navigator.clipboard.writeText`) and "Download" (zip). "Apply head part" appears only for a passed result, with a note that the offsets are committed by hand after review.
- `CalibratePage.tsx` renders `ArmCalibration` after the head sections when `config.rgbd`; the "Arm calibration - deferred" line is removed.
- Tests: new `ArmCalibration.test.tsx` (disabled without an applied head chain; Start sends `allowBigJoints: false` without a dialog; `allowBigJoints` true only after the shoulder confirm; Copy writes the snippet; Apply disabled on a failed result; Stop calls the route). `CalibratePage.test.tsx`: the deferred line is gone. `HeadFloorFit.test.tsx` and `HeadArmCalibration.test.tsx`: the D435 wording and the skip reasons.

## Acceptance conditions

- AC1: Start runs tiers 0 and 1 without a dialog and without shoulder moves [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "start runs wrist and elbow tiers"]
- AC2: shoulder moves are sent only after the shoulder confirm [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "allow big joints only after the shoulder confirm"]
- AC3: Arm calibrate is disabled until a head chain is applied [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "disabled without an applied head chain"]
- AC4: Copy writes the offsets snippet and Apply is disabled on a failed result [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "copy writes the snippet"]
- AC5: the deferred placeholder line is gone [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx -t "has no deferred arm line"]
- AC6: floor and head-arm sections describe the D435 and show skip reasons [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx src/calibrate/HeadArmCalibration.test.tsx]
- AC7: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadArmCalibration.test.tsx`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write `ArmCalibration.test.tsx` and update the three other tests; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx src/calibrate/CalibratePage.test.tsx src/calibrate/HeadFloorFit.test.tsx src/calibrate/HeadArmCalibration.test.tsx`; expect failures.
- Implement ArmCalibration and update the page and the two sections; rerun; expect pass.
- Run the verify command; commit `feat(web_ui): Calibrate page arm section and D435 head sections`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: the fit now frees d435_joint x/z/roll/yaw (camera_* params, fit_camera_mount flag; design D32b); HeadArmCalibration.tsx PARAM_ROWS must show them.
