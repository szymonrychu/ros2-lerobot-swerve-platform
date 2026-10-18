---
title: Web UI head camera frontend
status: done
repo: ros2-lerobot-sverve-platform
value: "The Arm tab's second stream reads the head camera field the backend now sends."
files: [nodes/web_ui/frontend/src/types.ts, nodes/web_ui/frontend/src/tabs/ArmTab.tsx, nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx, nodes/web_ui/frontend/src/hooks/subscription.ts, nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx, nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [web-ui-head-camera-topics]
owns: []
uses: [web-head-camera-fields]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--web-ui-head-camera-frontend
branch: front-camera-30-fps-stream-w--web-ui-head-camera-frontend
verified_head: ce6bbb01ea3a96cd33820d39329d2510348f649f
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Web UI head camera frontend

## Requirement

- `types.ts`: `front_camera_topic` renamed `head_camera_topic` (comment: head camera).
- `ArmTab.tsx`: the second stream reads `tab.head_camera_topic`, alt text "head camera", doc comment says gripper and head camera. `hooks/subscription.ts` subscribes `head_camera_topic` and no `front_camera_topic`.
- Fixtures: `CameraTab.test.tsx` single-stream fixture and `HeadTab.test.tsx` RGB fixture use `/head_camera/image_raw/compressed`.
- `nodes/web_ui/README.md`: Arm row/section say head camera / `head_camera_topic`; Head row/section say RGB from the head camera (`/head_camera/image_raw/compressed`, head_camera_cpp); the Front/Back tab row is removed.
- Tests first, seen failing: `ArmTab.test.tsx` uses `head_camera_topic` and alt "head camera"; the subscription test (in `ArmTab.test.tsx` or the existing subscription test) expects `head_camera_topic`.

## Acceptance conditions

- a1: the Arm tab shows the head camera from head_camera_topic [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx]
- a2: no frontend source names front_camera_topic or /front_camera [evidence: command: ! git grep -n -E 'front_camera_topic|/front_camera/' -- nodes/web_ui/frontend/src]
- a3: the web_ui README has no Front/Back tab row and names head_camera_topic [evidence: command: grep -q head_camera_topic nodes/web_ui/README.md && ! grep -n '| Front/Back |' nodes/web_ui/README.md]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`
- `nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- `TabConfig.head_camera_topic?: string` in `frontend/src/types.ts` (matches `web-head-camera-fields`).

## Expected outcome

Frontend code, tests and README use head_camera_topic and /head_camera.

## Steps

- Update ArmTab.test.tsx and fixtures; run `cd nodes/web_ui/frontend && npm test -- ArmTab`; see it fail.
- Rename in types.ts, ArmTab.tsx, subscription.ts; update the README; run the verify command; commit `refactor(web_ui): frontend head_camera_topic`.

## Notes

- Builds on the Jetson epic's `head-and-arm-tabs-frontend` end state.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on web-ui-head-camera-topics
