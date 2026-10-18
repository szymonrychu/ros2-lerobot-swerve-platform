---
title: Web UI obstacle points layer
status: todo
repo: ros2-lerobot-sverve-platform
value: The operator can switch on a 3D obstacles view that shows what the ToF and the lidar see around the robot.
files: [nodes/web_ui/frontend/src/types.ts, nodes/web_ui/frontend/src/map3d/obstaclePoints.ts, nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts, nodes/web_ui/frontend/src/map3d/layers.ts, nodes/web_ui/frontend/src/map3d/layers.test.ts, nodes/web_ui/frontend/src/map3d/sceneLayers.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.tsx, nodes/web_ui/frontend/src/hooks/subscription.ts]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [web-ui-obstacle-points-api]
owns: []
uses: [obstacle-points-payload]
model: sonnet
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI obstacle points layer

## Requirement

Test first: `map3d/obstaclePoints.test.ts` and the new `layers.test.ts` cases fail before the code exists (D10, U5).

- `types.ts`: tab config fields `obstacle_cloud_topic`, `scan_points_topic`; payload types for roles `cloud` and
  `scan_points` matching the backend contract.
- `map3d/obstaclePoints.ts`: pure functions converting a payload into map-frame positions (apply `frame_pose`
  quaternion + translation to every point; scan points at z = 0 in the scan frame) and per-point colours (ToF by height
  band: cliff below floor, obstacle 0.06-0.6 m, other; lidar one colour); rejects payloads with an odd/mismatched length.
- `map3d/layers.ts`: one layer id `obstacles3d` labelled "3D obstacles (ToF + lidar)", default off; `layers.test.ts`
  asserts the label, default off, and that the layer depends on both topics.
- `map3d/sceneLayers.tsx`: draws both point sets as three.js `points` when the layer is on.
- `tabs/MapNavTab.tsx` / `hooks/subscription.ts`: both topics are requested only while the layer is on (on-demand), and
  released when it is turned off or the tab unmounts.
- `obstaclePoints.test.ts`: identity and 90 deg yaw + translation poses, scan conversion, colour bands, malformed
  payloads.

## Acceptance conditions

- a1: payload points are transformed into map positions with frame_pose and coloured by band [evidence: test: nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts]
- a2: one default-off layer 3D obstacles (ToF + lidar) drives both topics [evidence: test: nodes/web_ui/frontend/src/map3d/layers.test.ts]
- a3: topics are subscribed only while the layer is on [evidence: judgement]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
