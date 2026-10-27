---
title: Map 3D obstacles layer on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "The 3D view says what it shows: the D435 cloud and the lidar."
files: [nodes/web_ui/frontend/src/map3d/layers.ts, nodes/web_ui/frontend/src/map3d/obstaclePoints.ts, nodes/web_ui/frontend/src/map3d/sceneLayers.tsx, nodes/web_ui/frontend/src/map3d/MapScene.tsx, nodes/web_ui/frontend/src/map3d/layers.test.ts, nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts, nodes/web_ui/frontend/src/hooks/subscription.test.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts src/map3d/obstaclePoints.test.ts src/hooks/subscription.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-obstacles-topic-on-d4]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Map 3D obstacles layer on D435

## Requirement

Goal G5: "Every web_ui 3D view feature (Head RGB-D layer, Ground layer, ToF obstacles layer ...) shows D435 data." Design D12.

- `src/map3d/layers.ts`: label "3D obstacles (D435 + lidar)". The "Head RGB-D" and "Ground" layer labels are unchanged; they get data once head_rgbd_cpp and ground_estimator_cpp run.
- `src/map3d/obstaclePoints.ts`: `tofColors` renamed `depthColors`; doc comments say D435 depth cloud; the height bands still match the Nav2 depth layer.
- `src/map3d/sceneLayers.tsx` and `src/map3d/MapScene.tsx`: identifiers and comments `tof` -> `depth`.
- Tests: `src/map3d/layers.test.ts` (label), `src/map3d/obstaclePoints.test.ts` (renamed function; frame `d435_depth_optical_frame` in fixtures), `src/hooks/subscription.test.ts` (fixture topic `/head/d435/points_filtered`). The ToF fixtures are no longer accurate.

## Acceptance conditions

- AC1: the obstacles layer is labelled as D435 + lidar [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts]
- AC2: depth points are coloured by height band with depthColors [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/map3d/obstaclePoints.test.ts]
- AC3: no frontend identifier or label in map3d says ToF [evidence: command:! grep -rn "tof\|ToF" nodes/web_ui/frontend/src/map3d --include=*.ts --include=*.tsx]
- AC4: type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/map3d/MapScene.tsx`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.test.ts`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
