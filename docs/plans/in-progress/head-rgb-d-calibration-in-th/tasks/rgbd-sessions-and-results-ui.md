---
title: RGBD sessions and results UI
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The operator reopens any past session, computes on it, reads the report, applies a passing result and downloads the files to commit."
files: [nodes/web_ui/frontend/src/calibrate/RgbdSessions.tsx, nodes/web_ui/frontend/src/calibrate/RgbdSessions.test.tsx, nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx, nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx, nodes/web_ui/README.md]
verify: "(cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint)"
depends_on: [rgbd-capture-section-ui]
owns: []
uses: [rgbd-calib-frontend-api]
model: ""
estimate: 80
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--rgbd-sessions-and-results-ui
branch: head-rgb-d-calibration-in-th--rgbd-sessions-and-results-ui
verified_head: ""
run: ""
provides: []
---

# RGBD sessions and results UI

## Requirement

Sessions and results in the RGB-D section: reopen any session, compute, read the report, apply a passing result, download (design D10, D6).

- New `RgbdSessions.tsx`: the session list (id, created, pose and result counts, newest first); Open sets the open session in `RgbdCalibration` (its thumbnails and coverage show through `RgbdCapture`); Resume capture calls `startCapture(id)`; Compute calls `computeSession(id)`, disabled while a capture or a compute runs, and shows a 409 server message; `RgbdSessions` receives `{capture, compute}` from `RgbdCalibration` as props (the poll is owned there and runs while a capture or compute runs); the result list refreshes when the compute in the props ends (an error from the job is shown); after Resume capture it calls `onStarted` so the root polls.
- New `RgbdResults.tsx`: results of the open session newest first: a passed or failed chip; a table of RGB, ToF and stereo RMS, plane bias and view counts next to their limits; failed limits listed; `report` as preformatted text in a collapsible; Apply only for passed results, through `ConfirmDialog` naming the restart of head_camera_cpp, head_rgbd_cpp and head_state_publisher, then `getApplied` polled every 2000 ms for at most 60 s until every unit is restarted, showing each unit's state; Download as an anchor to `downloadUrl` with the `download` attribute.
- `RgbdCalibration.tsx` renders `RgbdSessions` and `RgbdResults` next to `RgbdCapture`.
- Tests (jsdom, fetch stubbed): new `RgbdSessions.test.tsx` (open, resume, compute disabled while busy, 409 message, refresh at the end) and `RgbdResults.test.tsx` (table and chip, Apply hidden for failed, Apply through the dialog then per-unit restart state, Download href).
- `nodes/web_ui/README.md` describes the RGB-D section and lists the tests (doc file).

## Acceptance conditions

- a1: sessions can be reopened, resumed and computed, with busy and 409 handling [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdSessions.test.tsx]
- a2: results show limits, apply passed ones through the dialog with restart state, and link the download [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx]
- a3: the whole frontend suite, typecheck and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint]

## Files

- `nodes/web_ui/frontend/src/calibrate/RgbdSessions.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdSessions.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- `RgbdSessions` (props: `openSessionId`, `onOpen(id)`, `captureRunning`, `compute`, `onStarted()`), `RgbdResults` (props: `sessionId`).

## Expected outcome

From the page the operator reopens an old session, recomputes, reads the report, applies a passing result and sees the three head nodes restart, or downloads the zip.

## Steps

- Write `RgbdSessions.test.tsx` and `RgbdResults.test.tsx`; run: red.
- Implement the components and wire them into `RgbdCalibration`.
- Run the verify command: whole frontend suite, tsc and eslint clean.

## Notes

- Apply is offered only for `passed: true` results (design D12).
