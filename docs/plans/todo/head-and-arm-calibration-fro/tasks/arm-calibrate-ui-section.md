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

The "Arm calibrate" button and section, with confirmations before any arm motion (design D7, D9; approval A3; CLAUDE.md servo safety).

- New `nodes/web_ui/frontend/src/calibrate/ArmCalibration.tsx`:
  - "Arm calibrate" is disabled with the reason "apply a head calibration first" until the applied record's kind is `head_chain` or `arm`.
  - Clicking opens a confirm dialog: "The arm will move: elbow, wrist flex and wrist roll, small amplitude, slow. Keep the area clear." Only Confirm calls `startArmSequence({allowBigJoints: false})`.
  - After the small tier finishes: "Allow shoulder moves" opens a second confirm naming shoulder_lift and shoulder_pan; only that Confirm calls `startArmSequence({sessionId: <same session>, allowBigJoints: true})`. No other path sends `allowBigJoints: true`.
  - "Stop" is visible whenever a sequence runs; progress shows tier, step/total, captured, skipped, status.
  - "Compute" -> result card: offsets table (current, new, change in deg, sigma), head x/y/yaw change, held-out tool error in mm, wrist_roll fitted or not, pass/fail and failed limits; the snippet in a monospace block with "Copy" (`navigator.clipboard.writeText`) and "Download" (zip); "Apply head part" only for a passed result, with a note that the offsets are committed by hand after review.
- `CalibratePage.tsx`: renders `ArmCalibration` after `HeadCalibration` when `config.rgbd`; the "Arm calibration - deferred" line is removed.
- New `ArmCalibration.test.tsx`: disabled without an applied head chain; no start without the first confirm; `allowBigJoints` true only after the second confirm (and never on the first start); Copy writes the snippet; Apply disabled on a failed result; Stop calls the route. `CalibratePage.test.tsx`: the deferred line is gone.

Steps: tests first (red), implement, verify.

## Acceptance conditions

- a1: the arm only starts after the confirm dialog [evidence: test:nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx::no arm start without confirm]
- a2: shoulder moves are sent only after the second confirm [evidence: test:nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx::allow big joints only after the second confirm]
- a3: Arm calibrate is disabled until a head chain is applied [evidence: test:nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx::disabled without an applied head chain]
- a4: Copy writes the offsets snippet [evidence: test:nodes/web_ui/frontend/src/calibrate/ArmCalibration.test.tsx::copy writes the snippet]
- a5: the deferred placeholder line is gone [evidence: test:nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx::has no deferred arm line]
- a6: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

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
