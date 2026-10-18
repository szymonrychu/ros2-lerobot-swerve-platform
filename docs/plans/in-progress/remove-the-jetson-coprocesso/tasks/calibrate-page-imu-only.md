---
title: Calibrate page IMU only
status: escalated
repo: ros2-lerobot-sverve-platform
value: "The Calibrate page shows only the IMU section; no stereo calibration UI or API client remains."
files: [nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibrateRouting.test.tsx, nodes/web_ui/frontend/src/calibrate/stereoApi.ts, nodes/web_ui/frontend/src/calibrate/stereoApi.test.ts, nodes/web_ui/frontend/src/calibrate/StereoCalibration.tsx, nodes/web_ui/frontend/src/calibrate/StereoCalibration.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [calibration-config-no-depthd]
owns: []
uses: [calibration-config-endpoint]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--calibrate-page-imu-only
branch: remove-the-jetson-coprocesso--calibrate-page-imu-only
verified_head: b0f4b86e0c97a3607d147ca69cabf95177f10616
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: []
process_gaps: []
---

# Calibrate page IMU only

## Requirement

- `stereoApi.ts`, `stereoApi.test.ts`, `StereoCalibration.tsx` and `StereoCalibration.test.tsx` are deleted.
- `CalibratePage.tsx` parses `{imu}` from `/api/calibration/config` (its config type has no `stereo` field), renders only the IMU section when `imu` is true, and keeps its existing empty state when it is false.
- Tests first, seen failing: `CalibratePage.test.tsx` responds `{imu: true, arm: 'deferred'}` and asserts the IMU section and no "Stereo calibration" text, plus the `imu: false` case; `CalibrateRouting.test.tsx` responds `{imu: true}` and finds the IMU section heading instead of "Stereo calibration".
- No file under `nodes/web_ui/frontend/src` mentions stereo.
- `nodes/web_ui/README.md` Calibrate page section: the Stereo section bullet and the `stereoApi.test.ts` / `StereoCalibration.test.tsx` entries in the vitest list are removed.

## Acceptance conditions

- a1: the page renders the IMU section and no stereo section for an imu-only config [evidence: test: nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx]
- a2: routing to #/calibrate shows the IMU section [evidence: test: nodes/web_ui/frontend/src/calibrate/CalibrateRouting.test.tsx]
- a3: no frontend source mentions stereo [evidence: command: ! git grep -n -i stereo -- nodes/web_ui/frontend/src]
- a4: typecheck, vitest, eslint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibrateRouting.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/stereoApi.ts`
- `nodes/web_ui/frontend/src/calibrate/stereoApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/StereoCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/StereoCalibration.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces


- Consumes `GET /api/calibration/config` -> `{imu: boolean, arm: string}` (owned by move-calibrate-config-to-imu).

## Expected outcome


The Calibrate page is IMU-only in code, tests and README.

## Steps


- Rewrite the two tests; run `cd nodes/web_ui/frontend && npm test -- src/calibrate` and see them fail.
- Delete the four stereo files; edit CalibratePage.tsx.
- Run the verify command; update the README; commit `refactor(web_ui): Calibrate page is IMU-only`.

## Notes


- `CalibratePage.tsx:11,15,30,72` (discovery brief). The page is off on the robot today (no `calibration:` block in client.yml).
