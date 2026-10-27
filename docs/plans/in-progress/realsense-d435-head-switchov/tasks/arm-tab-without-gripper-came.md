---
title: Arm tab without gripper camera
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The Arm tab no longer shows a dead camera.
files: [nodes/web_ui/frontend/src/tabs/ArmTab.tsx, nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx, nodes/web_ui/frontend/src/arm/ArmView.tsx, nodes/web_ui/frontend/src/arm/ArmView.test.tsx, nodes/web_ui/frontend/src/map3d/armCamera.ts, nodes/web_ui/frontend/src/map3d/armCamera.test.ts, nodes/web_ui/frontend/src/types.ts, nodes/web_ui/frontend/src/hooks/subscription.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx src/arm/ArmView.test.tsx src/hooks/subscription.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-arm-tab-config-withou, head-tab-on-d435]
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

# Arm tab without gripper camera

## Requirement

Goal G9 / user decision 4: retire the "web_ui Arm-tab view" of the gripper camera. Design D12.

- `src/tabs/ArmTab.tsx`: the gripper `CameraStream` is removed; the head camera stream (D435 colour) stays.
- `src/arm/ArmView.tsx`: the 'gripper' 3D camera mode is removed (the remaining modes unchanged).
- `src/map3d/armCamera.ts` and `src/map3d/armCamera.test.ts` deleted (they place the gripper camera from `gripper_camera_mount`).
- `src/types.ts`: `gripper_camera_topic` and `gripper_camera_mount` removed. `src/hooks/subscription.ts` stops subscribing `gripper_camera_topic`.
- Tests: `src/tabs/ArmTab.test.tsx` and `src/arm/ArmView.test.tsx`: the gripper cases are no longer accurate and are removed. New cases: only the head stream is rendered; the camera mode list has no gripper.

## Acceptance conditions

- AC1: the Arm tab renders only the head camera stream [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx -t "only the head camera stream"]
- AC2: the arm 3D view offers no gripper camera mode [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/arm/ArmView.test.tsx -t "no gripper camera mode"]
- AC3: the gripper camera placement module is no longer tracked [evidence: command:test -z "$(git ls-files nodes/web_ui/frontend/src/map3d/armCamera.ts)"]
- AC4: type check and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/arm/ArmView.tsx`
- `nodes/web_ui/frontend/src/arm/ArmView.test.tsx`
- `nodes/web_ui/frontend/src/map3d/armCamera.ts`
- `nodes/web_ui/frontend/src/map3d/armCamera.test.ts`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update the two tests; run `cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx src/arm/ArmView.test.tsx`; expect failures.
- Remove the stream, the mode, the types and the subscription; `git rm` armCamera.ts and its test; rerun; expect pass.
- Run the verify command; commit `refactor(web_ui): Arm tab without the gripper camera`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
