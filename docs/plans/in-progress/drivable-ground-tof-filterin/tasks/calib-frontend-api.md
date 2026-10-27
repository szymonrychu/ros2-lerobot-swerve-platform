---
title: Calib frontend API
status: done
repo: ros2-lerobot-sverve-platform
value: "The Calibrate page talks to the head-from-arm routes through one typed client checked against the node's fixtures."
files: [nodes/web_ui/frontend/src/calibrate/headArmApi.ts, nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts, nodes/web_ui/frontend/src/calibrate/headFloorApi.ts, nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts src/calibrate/headFloorApi.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [head-arm-sequence-http-api]
model: ""
review: ""
verified_head: "9cf952c6"
points: 3
expected_minutes: 45
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Calib frontend API

## Requirement

Typed frontend client for the head-from-arm routes (uses the `head-arm-calib-api` contract; design D41; approval H5).

- New `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`, base `/api/calibration/rgbd/head-arm` (the existing proxy
  forwards it as it does `/api/calibration/rgbd/floor`), reusing the call and result helpers of `headFloorApi.ts`:
  - types `HeadArmSequence` (`running, phase, session_id, tier, arm_pose, total_poses, captured, skipped, status,
    error, big_joints, waiting_since`), `HeadArmState` (`sequence`, `applied`), `HeadArmParam` (`value, sigma, start,
    floor, floor_delta`), `HeadArmResult` (`id, session, kind, passed, failed_limits, error, rms_px, holdout_rms_px,
    holdout_board_mm, views, arm_poses, pan_span_deg, tilt_span_deg, max_correlation, params, board_mount, per_view,
    floor`), `HeadArmSession`;
  - `getHeadArmState()`, `moveToBase(confirm)`, `boardAttached()`, `allowBigJoints(confirm)`, `stopSequence()`,
    `park(boardRemoved)`, `listHeadArmSessions()`, `computeHeadArm(sid)`, `getHeadArmResult(sid, rid)`,
    `applyHeadArm(sid, rid)`;
  - parsing rejects a body missing a key with an error naming it (as `headFloorApi.ts` does).
- `headFloorApi.ts`: `AppliedHeadChain` gains optional `source` and `floor` (the extended applied record);
  `headFloorApi.test.ts` passes with the fixtures as the node now writes them.
- New `headArmApi.test.ts`: each function's method, path and body; parsing of the node fixtures
  `nodes/head_rgbd_calibration/tests/fixtures/api/head_arm_state.json`, `head_arm_result.json`,
  `head_arm_sessions.json` (imported as JSON); a missing key rejected; error bodies surface `message`.

## Acceptance conditions

- a1: client tests pass against the node fixtures [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts]
- a2: the floor client still parses the extended applied record [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/headFloorApi.test.ts]
- a3: type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: the client module exists [evidence: symbol:nodes/web_ui/frontend/src/calibrate/headArmApi.ts::computeHeadArm]

## Files

- `nodes/web_ui/frontend/src/calibrate/headArmApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headArmApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`

## Interfaces

- `headArmApi.ts` exports above; `headFloorApi.ts::AppliedHeadChain` with `source?`, `floor?`.

## Expected outcome

The Calibrate page talks to the head-from-arm routes through one typed client checked against the node's fixtures.

## Steps

- Write `headArmApi.test.ts` first; run it: red.
- Implement; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/headArmApi.test.ts src/calibrate/headFloorApi.test.ts && npx tsc --noEmit && npm run lint`;
  expect every test passed and lint clean.

## Notes

- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api

- 2026-10-23 run drivable-ground-tof-filterin: dependency-failed on head-arm-sequence-http-api
