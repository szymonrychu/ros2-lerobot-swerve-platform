---
title: Calibrate pose list hints
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Calibrate page shows which pose is weak or an outlier, right next to its delete button."
files: [nodes/web_ui/frontend/src/calibrate/rgbdApi.ts, nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts, nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx, nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate && npx tsc --noEmit && npm run lint"
depends_on: [capture-corner-hints, compute-result-pose-errors]
owns: []
uses: [pose-capture-hints, rgbd-result-pose-errors]
model: ""
estimate: 55
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calibrate pose list hints

## Requirement

Show the capture hints and the per-pose Compute errors in the Calibrate page RGB-D pose list (design D6). Delete-pose stays the only action; nothing is pruned automatically (D1).

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`: `PoseSummary` gains `hints?: string[]`; new `PoseError {id: string; rgb_error_px: number | null; tof_error_px: number | null; stereo_error_px: number | null; outlier: string[]}`; `ResultSummary` gains `poses?: PoseError[]` and `outlier_poses?: string[]`. The guards accept the fields when absent (a node or result from before this change) and reject them when present with the wrong type. Exported helper `poseErrors(detail: SessionDetail): Map<string, PoseError>` returns the `poses` of the newest result (`detail.results[0]`, the node lists newest first) keyed by pose id, empty when there is no result or it has no `poses`.
- `RgbdCapture.tsx`, per pose card: every capture hint as its own warning-coloured caption line under the corner counts; when the newest result has the pose, a caption `RGB 0.210 / ToF 0.120 / stereo 0.400 px` (3 decimals, `-` for null); when its `outlier` is non-empty, the card border uses the error colour and a line `high error - delete and Compute again` is shown. A pose missing from the newest result (captured after it) shows no error line. The delete button and confirm dialog are unchanged.
- The session detail is fetched again when a compute finishes (`compute.running` goes from true to false), so the errors appear without reopening the session.
- Tests: `rgbdApi.test.ts` accepts the node fixtures `nodes/head_rgbd_calibration/tests/fixtures/api/session.json` and `result.json` (with `hints`, `poses`, `outlier_poses`), accepts a session and a result without them, rejects a non-array `hints` / `poses`, and checks `poseErrors` in a `describe('poseErrors')` block; `RgbdCapture.test.tsx` renders the hint line, the error caption, the highlighted flagged card with its one-line hint, no error line for a pose absent from the result, and the refetch after a compute finishes.
- `npx vitest run src/calibrate`, `npx tsc --noEmit` and `npm run lint` pass in `nodes/web_ui/frontend`.

## Acceptance conditions

- a1: the api guards and poseErrors accept the node fixtures and older bodies [evidence: test:nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts::poseErrors]
- a2: the pose list shows hints, errors and the flagged-pose highlight with its hint [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx]
- a3: poseErrors is exported from the api module [evidence: symbol:nodes/web_ui/frontend/src/calibrate/rgbdApi.ts::poseErrors]
- a4: calibrate tests, typecheck and lint pass [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
