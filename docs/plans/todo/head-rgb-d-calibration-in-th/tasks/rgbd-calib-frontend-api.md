---
title: RGBD calib frontend API
status: todo
repo: ros2-lerobot-sverve-platform
value: "The page has a typed, tested client for every calibration call, with server messages shown verbatim."
files: [nodes/web_ui/frontend/src/calibrate/rgbdApi.ts, nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts]
verify: "(cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts && npx tsc --noEmit && npm run lint)"
depends_on: [web-ui-rgbd-calib-proxy]
owns: [rgbd-calib-frontend-api]
uses: [rgbd-calib-web-api]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# RGBD calib frontend API

## Requirement

A typed, tested frontend client for every RGB-D calibration call (design D10, Contracts `rgbd-calib-web-api`).

- New `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts` following `imuApi.ts` (`call<T>(url, init, validator)` returning `{ok: true, value} | {ok: false, status, message}` with the server `message` passed through):
  - types `CaptureState`, `ComputeState`, `RgbdState`, `SessionSummary`, `PoseSummary`, `Coverage` (`{cols, rows, counts: number[][]}`), `SessionDetail`, `ResultSummary`, `ResultDetail` (with `report`), `UnitRestart`, `AppliedState`;
  - `getState`, `startCapture(sessionId?)`, `stopCapture`, `listSessions`, `getSession`, `deletePose`, `computeSession`, `getResult`, `applyResult`, `getApplied` against `/api/calibration/rgbd/...` (paths of the design's Node HTTP API table with the `/api` prefix replaced);
  - `thumbnailUrl(sid, pid, kind: 'rgb' | 'tof')` (maps to the node routes `rgb.jpg` / `tof.jpg`) and `downloadUrl(sid, rid)` with every id `encodeURIComponent`-ed;
  - pure helpers `emptyCells(coverage): number` and `statusSeverity(status): 'success' | 'info' | 'warning'` (captured is success, moving and waiting info, the three `still - ...` messages warning).
- Validators reject wrong shapes (a 200 with a malformed body gives `ok: false`).
- New `rgbdApi.test.ts`: URL, method and body of every call; validator failure; server message passed through; helpers.

## Acceptance conditions

- a1: every call uses the right URL, method and body and validates the response [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts]
- a2: the client compiles and lints clean [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a3: the API client module exists [evidence: symbol:nodes/web_ui/frontend/src/calibrate/rgbdApi.ts::startCapture]

## Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`

## Interfaces

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`: the calls, types and helpers in the requirement.
- Contract `rgbd-calib-frontend-api` (owned here).

## Expected outcome

Every component task imports its calls and types from one module; malformed responses never reach the UI as data.

## Steps

- Write `rgbdApi.test.ts`; `npx vitest run src/calibrate/rgbdApi.test.ts`: red.
- Implement `rgbdApi.ts`.
- Run the verify command: tests pass, tsc and eslint clean.

## Notes

- Pattern: `src/calibrate/imuApi.ts` and its test.
