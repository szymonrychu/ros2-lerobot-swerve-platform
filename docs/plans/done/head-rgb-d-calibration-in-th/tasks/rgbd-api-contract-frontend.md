---
title: RGBD API contract frontend
status: done
repo: ros2-lerobot-sverve-platform
value: "The Calibrate page accepts exactly what the node returns: no false errors on Start, Stop or Apply, and limits shown from the result."
files: [nodes/web_ui/frontend/src/calibrate/rgbdApi.ts, nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts, nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx, nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint"
depends_on: [rgbd-api-contract-node]
owns: []
uses: [rgbd-api-fixtures]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--rgbd-api-contract-frontend
branch: head-rgb-d-calibration-in-th--rgbd-api-contract-frontend
verified_head: "c4ef29de"
run: ""
provides: []
---

# RGBD API contract frontend

## Requirement

The Calibrate page frontend must accept exactly what the node returns. Consume the contract fixtures owned by rgbd-api-contract-node (`nodes/head_rgbd_calibration/tests/fixtures/api/*.json`) in the frontend tests by importing those JSON files directly (relative import from `nodes/web_ui/frontend/src/calibrate/`; vitest can read outside src; if the vite/ts config blocks it, add the minimal config change), never a hand-written copy.

- `rgbdApi.ts`: `startCapture` / `stopCapture` / `applyResult` / `getApplied` validators accept the fixture bodies; `AppliedState` uses the node's key names; a malformed body still yields the "Unexpected calibration response" error.
- `RgbdResults.tsx`: shows each metric next to its limit read from `result.json`'s `limits` object (including plane bias against `plane_max_bias_mm`); the hard-coded `DEFAULT_LIMITS` copy is removed; Apply success shows the applied state and unit restarts from the documented response.
- `RgbdCapture.tsx` / `RgbdCalibration.tsx`: Start and Stop update the shown state from the response (no error when the node accepted the call).
- Tests first, seen failing against the fixtures: rgbdApi validators accept each fixture; Start/Stop and Apply component flows driven by fixture bodies show no error; results table shows the plane bias limit from the fixture.
- `npx tsc --noEmit && npm test && npm run lint` pass; README section of nodes/web_ui updated if wording changes.

## Acceptance conditions

- a1: every rgbdApi call accepts its node fixture body [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts]
- a2: Start, Stop and Apply driven by fixture bodies show the new state and no error [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx]
- a3: the results table shows each limit from result.json limits, including plane bias, and DEFAULT_LIMITS is gone [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx]

## Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

The Calibrate page shows no false errors on Start, Stop or Apply and shows each limit from result.json next to its metric.

## Steps

- Import the node fixtures in rgbdApi/RgbdCapture/RgbdResults tests; run `npm test` and see them fail.
- Fix the validators, AppliedState names, results limits and Start/Stop state handling; run the verify command (all pass) and commit `fix(web_ui): Calibrate RGB-D section follows the node API contract`.

## Ruling

- 2026-10-20 accepted: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
