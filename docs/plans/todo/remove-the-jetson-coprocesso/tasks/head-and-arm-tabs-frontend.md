---
title: Head and Arm tabs frontend
status: todo
repo: ros2-lerobot-sverve-platform
value: The browser Head tab has no fused toggle and the Arm tab shows the front camera.
files: [nodes/web_ui/frontend/src/tabs/HeadTab.tsx, nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx, nodes/web_ui/frontend/src/tabs/ArmTab.tsx, nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx, nodes/web_ui/frontend/src/types.ts, nodes/web_ui/frontend/src/hooks/subscription.ts, nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [arm-tab-front-camera-field]
owns: []
uses: [arm-front-camera-field]
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head and Arm tabs frontend

## Requirement

- `types.ts`: no `fused_depth_topic`; `overview_camera_topic` renamed `front_camera_topic`; the `tof_depth_topic` / `tof_amplitude_topic` comments describe the ToF with no stereo wording.
- `HeadTab.tsx`: no raw/fused state, toggle or fused topic selection; the depth preview always reads `tab.tof_depth_topic`; the amplitude switch is unchanged; the doc comment names no stereo_depth or fused topic.
- `ArmTab.tsx`: the second stream reads `tab.front_camera_topic` with alt text "front camera"; the doc comment says gripper and front camera.
- `hooks/subscription.ts` subscribes `front_camera_topic` and no `overview_camera_topic` or `fused_depth_topic`.
- Tests first, seen failing: `HeadTab.test.tsx` drops `FUSED` and the toggle tests (~114-140), adds `renders no raw/fused toggle` with the ToF tab config, and the no-amplitude test name and fixture no longer say stereo; `ArmTab.test.tsx` uses `front_camera_topic` and alt "front camera"; `CameraTab.test.tsx`'s `SINGLE` fixture uses `/front_camera/image_raw/compressed` with label "Front".
- No file under `nodes/web_ui/frontend/src` matches `stereo`, `fused_depth`, `raw/fused` or `overview_camera`.
- `nodes/web_ui/README.md` frontend notes for the Head and Arm tabs updated.

## Acceptance conditions

- a1: the Head tab renders no raw/fused toggle and reads tof_depth_topic [evidence: test: nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx]
- a2: the Arm tab shows the front camera from front_camera_topic [evidence: test: nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx]
- a3: no frontend source mentions stereo, fused depth or overview_camera [evidence: command: ! git grep -n -i -E 'stereo|fused_depth|raw/fused|overview_camera' -- nodes/web_ui/frontend/src]
- a4: typecheck, vitest, eslint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`
- `nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
