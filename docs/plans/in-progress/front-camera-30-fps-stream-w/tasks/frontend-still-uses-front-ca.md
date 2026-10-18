---
title: "Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale."
status: done
repo: ros2-lerobot-sverve-platform
value: "The backend field is now head_camera_topic, so the Arm tab front stream breaks until the frontend follows. The task web-ui-head-camera-frontend should cover it."
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale.

## Requirement

Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale.

Where: nodes/web_ui/frontend/src, nodes/web_ui/README.md

Why: The backend field is now head_camera_topic, so the Arm tab front stream breaks until the frontend follows. The task web-ui-head-camera-frontend should cover it.

## Acceptance conditions

- AC1: Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale. at nodes/web_ui/frontend/src, nodes/web_ui/README.md is resolved [evidence: judgement]

## Files

- (none)

## Interfaces


Keep the interfaces around nodes/web_ui/frontend/src, nodes/web_ui/README.md unchanged unless resolving it requires otherwise: Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale..

## Expected outcome


Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale. at nodes/web_ui/frontend/src, nodes/web_ui/README.md is resolved, and The backend field is now head_camera_topic, so the Arm tab front stream breaks until the frontend follows. The task web-ui-head-camera-frontend should cover it. no longer applies.

## Steps


1. Write a failing test that shows: Frontend still uses front_camera_topic and /front_camera literals in subscription.ts, ArmTab.tsx, types.ts, and the ArmTab, CameraTab and HeadTab tests. nodes/web_ui/README.md lines 13-15, 537 and 541 are also stale..
2. Fix it at nodes/web_ui/frontend/src, nodes/web_ui/README.md.
3. Run the task verify command and confirm it passes.

## Notes


Discovered in task `web-ui-head-camera-topics` during run `front-camera-30-fps-stream-w`.


## Resolution

Owned by web-ui-head-camera-frontend (frontend /front_camera -> /head_camera rename).
