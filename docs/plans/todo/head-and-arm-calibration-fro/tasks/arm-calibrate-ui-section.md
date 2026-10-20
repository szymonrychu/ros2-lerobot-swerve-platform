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

The "Arm calibrate" button and section, with a confirmation before each arm motion tier (design D7, D9; approval A3; CLAUDE.md servo safety).

- New `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`:
  - "Arm calibrate" is disabled with the reason "apply a head calibration first" until the applied record's kind is `head_chain` or `arm`.
  - Tier 0: clicking opens a confirm dialog: "The arm moves to the calibration pose, closes the gripper on the plate clip and turns the wrist roll only, slowly. Keep the area clear." Only Confirm calls `startArmSequence({allowWristElbow: false, allowBigJoints: false})`.
  - Tier 1: after tier 0, a checkbox "I confirm wrist flex and elbow moves (small amplitude, tool at least 10 cm above the floor)" enables "Run wrist and elbow poses"; only with the box ticked does that button call `startArmSequence({sessionId: <same session>, allowWristElbow: true})`.
  - Tier 2: after tier 1, "Allow shoulder moves" opens a second confirm dialog naming shoulder_lift and shoulder_pan; only that Confirm calls `startArmSequence({sessionId: <same session>, allowWristElbow: true, allowBigJoints: true})`.
  - No other path sends `allowWristElbow` or `allowBigJoints` true. "Stop" is visible whenever a sequence runs; progress shows tier, step/total, captured, skipped, status.
  - "Compute" -> result card: offsets table (current, new, change in deg, sigma), head x/y/yaw change, held-out tool error in mm, wrist_roll fitted or not (not fitted until the plate CAD rotation is configured), pass/fail and failed limits; the snippet in a monospace block with "Copy" (`navigator.clipboard.writeText`) and "Download" (zip); "Apply head part" only for a passed result, with a note that the offsets are committed by hand after review.
- `CalibratePage.tsx`: renders `ArmCalibration` after `HeadCalibration` when `config.rgbd`; the "Arm calibration - deferred" line is removed.
- New `ArmCalibration.test.tsx`: disabled without an applied head chain; no start without the tier 0 confirm; `allowWristElbow` true only with the checkbox ticked; `allowBigJoints` true only after the shoulder confirm (never on an earlier start); Copy writes the snippet; Apply disabled on a failed result; Stop calls the route. `CalibratePage.test.tsx`: the deferred line is gone.

## Acceptance conditions

- a1: the arm only starts after the tier 0 confirm dialog [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "no arm start without confirm"]
- a2: wrist and elbow poses are sent only with the confirm checkbox ticked [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "wrist and elbow only with the checkbox"]
- a3: shoulder moves are sent only after the shoulder confirm [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "allow big joints only after the second confirm"]
- a4: Arm calibrate is disabled until a head chain is applied [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "disabled without an applied head chain"]
- a5: Copy writes the offsets snippet [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/ArmCalibration.test.tsx -t "copy writes the snippet"]
- a6: the deferred placeholder line is gone [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx -t "has no deferred arm line"]
- a7: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

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
