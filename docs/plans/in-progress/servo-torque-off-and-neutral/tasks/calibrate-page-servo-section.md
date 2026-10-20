---
title: Calibrate page servo section
status: done
repo: ros2-lerobot-sverve-platform
value: "The user releases each group, records a neutral and re-enables with or without saving from the Calibrate page."
files: [nodes/web_ui/frontend/src/calibrate/servoApi.ts, nodes/web_ui/frontend/src/calibrate/servoApi.test.ts, nodes/web_ui/frontend/src/calibrate/ServoNeutral.tsx, nodes/web_ui/frontend/src/calibrate/ServoNeutral.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint"
depends_on: [web-ui-servo-routes]
owns: [servo-torque-hook]
uses: [servo-calibration-api]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--calibrate-page-servo-section
branch: servo-torque-off-and-neutral--calibrate-page-servo-section
verified_head: unknown
run: servo-torque-off-and-neutral
provides: [servo-torque-hook]
acs_for_audit: []
process_gaps: []
---

# Calibrate page servo section

## Requirement

- New `frontend/src/calibrate/servoApi.ts`: types for the `GET /api/calibration/servos` body (from web-ui-servo-routes), `fetchServoGroups()`, `servoAction(groupId, action)` (`release`, `enable`, `record`, `save`; returns `{ok, message}`), and the hook `useServoGroups(pollMs = 1000)` (404 = not configured, no error; network errors surfaced).
- New `frontend/src/calibrate/ServoNeutral.tsx`: a "Servo torque and neutral" section, one card per group (arm, base, head): a torque chip (`Torque on`, `Torque off`, `Partly off`, `Unknown` when state is null or stale), per-joint torque, and buttons (D8):
  - `Torque off` -> confirm dialog. Arm: "Support the arm: it goes limp and will drop." Head: "Support the head: it goes limp." Base: "The steering servos go limp; the wheels stay braked."
  - `Record neutral` (only for `neutral` groups, enabled while released) -> shows each joint's shift in degrees from the capture.
  - `Re-enable torque + save` (enabled when a capture exists) -> confirm dialog "Writes the new neutral into the servo EEPROM." The arm adds "Arm joint offsets and the stored home become stale until re-calibrated. Teleop needs the leader arm re-zeroed in the same pose."
  - `Re-enable torque` (enabled while released) -> no save.
  - The arm card shows a warning when `calibration_stale` is non-empty; backend messages are shown in an Alert; buttons are disabled while a call is in flight.
- `CalibratePage.tsx` reads `servos` from `/api/calibration/config` and renders the section when true.
- Tests first, seen failing (vitest + testing-library): `servoApi.test.ts` (fetch shapes, action URLs, 404 handling); `ServoNeutral.test.tsx` (chip states, confirm dialogs with the exact texts, record shows degrees, save confirm with the arm warnings, buttons enabled/disabled by state, error Alert); `CalibratePage.test.tsx` (section shown only with `servos: true`).

## Acceptance conditions

- a1: servoApi fetches state, posts actions and treats 404 as not configured [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/servoApi.test.ts]
- a2: the section shows per-group chips, confirm dialogs with the documented texts and enables buttons by state [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/ServoNeutral.test.tsx]
- a3: the Calibrate page renders the section only when servos is configured [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx]
- a4: frontend typecheck, tests and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint]

## Files

- `nodes/web_ui/frontend/src/calibrate/servoApi.ts`
- `nodes/web_ui/frontend/src/calibrate/servoApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/ServoNeutral.tsx`
- `nodes/web_ui/frontend/src/calibrate/ServoNeutral.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

## Interfaces

- Uses part `servo-calibration-api`.
- Owns part `servo-torque-hook`: `useServoGroups(pollMs?: number)` and the `ServoGroup` type in `servoApi.ts` (used by released-group-banners).

## Expected outcome

On the Calibrate page the user can release each group, record a neutral, and re-enable with or without saving, with the confirm dialogs and warnings the user asked for.

## Steps

- Write the three test files; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): servo torque and neutral section on the Calibrate page`.

## Ruling

- 2026-10-20 accepted: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean

## Notes

- Follow `ImuCalibration.tsx` / `imuApi.ts` for structure and `TOUCH_TARGET_PX` from `../theme` for touch targets.
- The page still shows "Arm calibration - deferred"; leave it (epic head-and-arm-calibration-fro replaces it).

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on web-ui-servo-routes
