---
title: Calibrate page IMU only
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
