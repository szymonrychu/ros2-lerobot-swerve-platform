---
title: Head floor fit UI step
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user runs the floor fit from the Calibrate page and sees fitted vs current values before applying; the runbook proves the floor reads flat."
files: [nodes/web_ui/frontend/src/calibrate/headFloorApi.ts, nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts, nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx, nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx, nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx, nodes/web_ui/README.md, docs/runbooks/head-floor-fit.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/headFloorApi.test.ts src/calibrate/HeadFloorFit.test.tsx src/calibrate/CalibratePage.test.tsx && npm run lint"
depends_on: [floor-fit-http-api]
owns: []
uses: [head-floor-fit-api]
model: ""
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head floor fit UI step

## Requirement

The "Head floor fit" step on the web UI Calibrate page and the robot runbook (uses `head-floor-fit-api`; design D9, D10). The web UI proxy `/api/calibration/rgbd/{path}` already forwards `floor/...` to the node; no backend change.

- New `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`: typed calls under `/api/calibration/rgbd/floor` (`getFloorState`, `startSweep`, `stopSweep`, `computeFloor(sid)`, `getFloorResult(sid, rid)`, `applyFloor(sid, rid)`) with runtime type guards in the style of `rgbdApi.ts`; a body that fails its guard is an error, never shown as data.
- New `HeadFloorFit.tsx`: a step card: the precondition text (robot on a flat floor, not on blocks; about 2 m in front and 1.5 m to each side clear; only the head moves); Start sweep; progress (pose i of N, observations, skipped poses with reasons) polled while running; Stop; Compute when a sweep has ended; the result: a table of mount z (mm), mount roll, mount pitch, tilt zero (deg) with current, fitted, delta and sigma, height and normal RMS, per-pose residuals, pass/fail with the failed limits; Apply enabled only for a passed result, then the unit restart state; the currently applied head chain (session, result, note).
- `CalibratePage.tsx`: renders `<HeadFloorFit />` after the RGB-D section when `config.rgbd` is set.
- Tests (vitest): `headFloorApi.test.ts` (guards accept `floor_state.json` and `floor_result.json` from `nodes/head_rgbd_calibration/tests/fixtures/api/`, reject a malformed body); `HeadFloorFit.test.tsx` (start, progress, compute, the table values, Apply disabled for a failed result and enabled for a passed one, errors shown); `CalibratePage.test.tsx` (the step shows with rgbd configured and not without).
- `nodes/web_ui/README.md`: the Calibrate page section names the Head floor fit step.
- New `docs/runbooks/head-floor-fit.md`: preconditions (flat floor, robot not on blocks, area clear, no agent session if mcp_server is deployed), deploy via `scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher web_ui` with output in `.logs/` (invoke the `ansible-deploy` skill first), the UI steps, and the acceptance checks with `scripts/topic_scraper_collect.py` on `/ground/state` at home and at two other pan/tilt poses: plane height 0 +- 0.01 m, plane_vs_base < 0.6 deg, fewer than 10 obstacle points on the flat floor; a results table filled when the orchestrator runs it with the user after merge.

## Acceptance conditions

- a1: the API guards accept the node fixtures and reject malformed bodies [evidence: test:nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts::accepts the floor fixtures and rejects malformed bodies]
- a2: the step runs start, progress, compute and apply with Apply only for a passed result [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx]
- a3: the Calibrate page shows the step only with rgbd configured [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx]
- a4: the runbook holds the acceptance checks [evidence: doc-section:docs/runbooks/head-floor-fit.md#Acceptance checks]
- a5: the web UI README names the step [evidence: judgement:]
- a6: frontend lint passes [evidence: command:cd nodes/web_ui/frontend && npm run lint]

## Files

- `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/README.md`
- `docs/runbooks/head-floor-fit.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
