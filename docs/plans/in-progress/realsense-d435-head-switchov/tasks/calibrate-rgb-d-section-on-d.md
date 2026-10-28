---
title: Calibrate RGB-D section on D435
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The calibration screen runs and shows the D435 intrinsics check.
files: [nodes/web_ui/frontend/src/calibrate/rgbdApi.ts, nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx, nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCalibration.test.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx, nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx, nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCalibration.test.tsx src/calibrate/RgbdCapture.test.tsx src/calibrate/RgbdResults.test.tsx src/calibrate/rgbdApi.test.ts && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [d435-intrinsics-check-comput]
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

# Calibrate RGB-D section on D435

## Requirement

Goal G8: "... intrinsics check against factory; the web_ui calibration screen drives it." Design D10, D12.

The Calibrate page "RGB-D calibration" section becomes "D435 intrinsics check":
- `rgbdApi.ts`: types follow the node's new contract fixtures (`nodes/head_rgbd_calibration/tests/fixtures/api/`). Pose summaries carry colour corners and plane-bias samples, with no `tof_corners`. The result is `kind: "intrinsics"` with rms_px, focal_diff_pct, principal_diff_px, plane bias median/spread, failed_limits and passed. Thumbnails are `rgb` and `depth`.
- `RgbdCalibration.tsx`: section title and help text describe the D435 check (hold the board at 0.3-0.6 m, cover the coverage grid). `RgbdCapture.tsx` shows colour corners and a depth thumbnail, with no ToF thumbnail or ToF corner count. `RgbdResults.tsx` shows the check rows (rms, focal diff, principal diff, plane bias, pass/fail per limit) and no stereo/ToF rows. "Apply" records a passed check (label "Accept check"; it restarts nothing, so the `RESTART_UNITS` list and its text are removed).
- Tests: `RgbdCalibration.test.tsx`, `RgbdCapture.test.tsx`, `RgbdResults.test.tsx`, `rgbdApi.test.ts`. ToF/stereo cases are no longer accurate and are replaced. New cases: the result rows of a passed and a failed check; Accept disabled on a failed check; no ToF thumbnail.

## Acceptance conditions

- AC1: the results show the D435 check rows and no ToF or stereo row [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx -t "D435 check rows"]
- AC2: Accept is disabled on a failed check [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx -t "accept disabled on a failed check"]
- AC3: capture shows colour corners and a depth thumbnail, no ToF [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx -t "no ToF thumbnail"]
- AC4: the API client parses the node's contract fixtures [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts]
- AC5: type check, lint and build pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update the four tests for the D435 check; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/Rgbd*.test.tsx src/calibrate/rgbdApi.test.ts`; expect failures.
- Change the API types and the three components; rerun; expect pass.
- Run the verify command; commit `feat(web_ui): Calibrate page D435 intrinsics check`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: calibration-config-on-d435 made only the minimal frontend changes the new fixtures need; the RGB-D section redesign is here.
- 2026-10-29 (from calibration-apply-restart-un): an intrinsics Accept now restarts nothing (restart only on head_chain.json change), but RgbdResults.tsx still polls and shows "restarting" until timeout; fix the UI to not wait for a restart after an intrinsics accept.
