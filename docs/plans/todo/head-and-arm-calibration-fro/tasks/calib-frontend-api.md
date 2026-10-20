---
title: Calib frontend API
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Calibrate page talks to the calibration node through typed, contract-checked calls."
files: [nodes/web_ui/frontend/src/calibrate/headArmApi.ts, nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts, nodes/web_ui/frontend/src/calibrate/rgbdApi.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts src/calibrate/rgbdApi.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [sequence-http-api]
owns: []
uses: [head-arm-calib-api]
model: ""
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib frontend API

## Requirement

Typed frontend client for the head and arm calibration routes (uses the `head-arm-calib-api` contract; design D9).

- New `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`, all under `/api/calibration/rgbd/` (the existing proxy forwards any path), reusing the request and error helpers of `rgbdApi.ts`:
  - types `SequenceState` (`running, kind, session_id, spot, step, total, captured, skipped, status, error, tier, allow_wrist_elbow, allow_big_joints`), `AppliedRecord` (adds `kind`, `head_session`), `HeadChainResult` (`params: Record<string, {value, sigma, urdf}>`, `rms_px`, `holdout_rms_px`, `views`, `spots`, `passed`, `failed_limits`, `report`), `ArmResult` (`offsets: Record<string, {value, sigma, current}>`, `head`, `holdout_tool_mm`, `wrist_roll_fitted`, `snippet`, ...);
  - `startHeadSequence(sessionId?)`, `nextSpot()`, `startArmSequence({sessionId, allowWristElbow, allowBigJoints})` (both default false), `stopSequence()`, `computeSession(sid, floorSession?)`, `getHeadResult(sid, rid)`, `getArmResult(sid, rid)`, `applyResult(sid, rid)`, `resultDownloadUrl(sid, rid)`.
  - Parsing rejects a body missing a key with an error naming it (as `rgbdApi.ts` does).
- `rgbdApi.ts`: the state type gains `sequence` and the applied record `kind`/`head_session`.
- New `headArmApi.test.ts`: each function's method, path and body; parsing of the node fixtures `nodes/head_rgbd_calibration/tests/fixtures/api/state.json`, `result_head_chain.json`, `result_arm.json` (imported as JSON); a missing key rejected; error bodies surface `message`. `rgbdApi.test.ts` passes.

## Acceptance conditions

- a1: each call uses the documented method, path and body [evidence: test:nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts::calls use the documented routes]
- a2: the node fixtures parse [evidence: test:nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts::parses the node fixtures]
- a3: a body missing a key is rejected naming it [evidence: test:nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts::rejects a body missing a key]
- a4: frontend type check, lint and the rgbd api tests pass [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts src/calibrate/headArmApi.test.ts && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`

## Interfaces

- `headArmApi.ts::startHeadSequence`, `nextSpot`, `startArmSequence`, `stopSequence`, `computeSession`, `getHeadResult`, `getArmResult`, `applyResult`, `resultDownloadUrl`; types `SequenceState`, `HeadChainResult`, `ArmResult`.

## Expected outcome

The Calibrate page talks to the calibration node through typed, contract-checked calls.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts src/calibrate/rgbdApi.test.ts && npx tsc --noEmit && npm run lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: builder check: head tools need a lease, three arm tiers with confirms, gripper set once, mcp-token via supplementary_groups, optional drive check
