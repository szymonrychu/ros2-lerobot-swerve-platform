---
title: RGBD capture section UI
status: todo
repo: ros2-lerobot-sverve-platform
value: "Standing at the robot, the operator sees each pose land live with thumbnails and coverage, and prunes bad ones."
files: [nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx, nodes/web_ui/frontend/src/calibrate/CoverageGrid.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx, nodes/web_ui/README.md]
verify: "(cd nodes/web_ui/frontend && npx vitest run src/calibrate src/layout && npx tsc --noEmit && npm run lint)"
depends_on: [rgbd-calib-frontend-api]
owns: []
uses: [rgbd-calib-frontend-api]
model: ""
estimate: 80
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--rgbd-capture-section-ui
branch: head-rgb-d-calibration-in-th--rgbd-capture-section-ui
verified_head: ""
run: head-rgb-d-calibration-in-th
provides: []
acs_for_audit: []
process_gaps: []
---

# RGBD capture section UI

## Requirement

The capture part of the RGB-D section: live status, pose count, thumbnails with corners, delete, and coverage (design D10). The IMU section keeps working.

- `CalibratePage.tsx`: `CalibrationConfig` gains `rgbd: boolean` (`body.rgbd === true`); renders `ImuCalibration` when `imu` and the new `RgbdCalibration` when `rgbd`.
- New `RgbdCalibration.tsx`: section root holding the open session id; calls `getState` on mount and owns `{capture, compute}`; passes them to `RgbdCapture` (and later `RgbdSessions`) as props; runs the 1000 ms poll (visible document only) while a capture or compute is running, so a page reload mid-capture or a Resume capture from `RgbdSessions` keeps polling; renders `RgbdCapture`.
- New `RgbdCapture.tsx`: Start capture (new session) and Stop buttons (`TOUCH_TARGET_PX`); while a capture runs, an `Alert` with the status line (severity from `statusSeverity`), the pose count and the auto-stop time; reads the capture state from props (the poll itself lives in `RgbdCalibration`: every 1000 ms only while a capture or compute runs and `document.visibilityState === 'visible'`, `visibilitychange` listener, interval cleared when both stop and on unmount); when `pose_count` changes the session detail is refetched; a thumbnail grid of the open session (RGB and ToF `img` per pose via `thumbnailUrl`, pose id, corner counts, a depth marker) with a delete `IconButton` opening `layout/ConfirmDialog`, then `deletePose` and a refresh; API errors shown in an error `Alert` with the server message.
- New `CoverageGrid.tsx`: RGB and ToF grids (cells shaded by corner count, empty cells marked) and the text `N of M image areas without corners` from `emptyCells`.
- Tests (`// @vitest-environment jsdom`, fetch stubbed with `vi.stubGlobal`): new `RgbdCapture.test.tsx` (a reload mid-capture resumes polling from the mounted state; start shows the status and count; a new pose's thumbnails appear; delete through the dialog calls DELETE and refreshes; polling stops on stop and while hidden; coverage hint text) and `CalibratePage.test.tsx` (both sections with `{imu: true, rgbd: true}`, IMU only with `rgbd: false`; existing assertions kept).
- `src/layout/designRules.test.ts` and `hardcodedSizes.test.ts` stay green; `nodes/web_ui/README.md` test list updated (doc file).

## Acceptance conditions

- a1: the capture section shows live status, count and thumbnails and deletes through the dialog [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx]
- a2: the page renders the IMU and RGB-D sections from the config flags [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx src/calibrate/ImuCalibration.test.tsx src/calibrate/CalibrateRouting.test.tsx]
- a3: design rules still hold [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/layout/designRules.test.ts src/layout/hardcodedSizes.test.ts]
- a4: the frontend compiles and lints clean [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CoverageGrid.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- `RgbdCalibration` (props: none; owns `{capture, compute}` and the poll), `RgbdCapture` (props: `sessionId: string | null`, `onSession(id)`, `capture`, `onStarted()`), `CoverageGrid` (props: `coverage: Coverage`, `label`).
- `CalibratePage` `CalibrationConfig { imu: boolean; rgbd: boolean }`.

## Expected outcome

On the robot the operator presses Start, sees `waiting for frames`, then `moving` / `still - ...` / `pose N captured - move to the next pose`, the count rising and thumbnails with corners appearing, and can delete a pose.

## Steps

- Write `RgbdCapture.test.tsx` and extend `CalibratePage.test.tsx`; run: red.
- Implement the components.
- Run the verify command: tests pass, tsc and eslint clean.

## Notes

- Poll-on-press pattern and visibility handling: `ImuCalibration.tsx`; dialogs: `src/layout/ConfirmDialog.tsx`; sizes: `TOUCH_TARGET_PX`.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on rgbd-calib-frontend-api
