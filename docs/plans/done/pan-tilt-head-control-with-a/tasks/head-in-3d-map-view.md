---
title: Head in 3D map view
status: done
repo: ros2-lerobot-sverve-platform
value: The 3D view shows the head where it really points.
files: [nodes/web_ui/frontend/src/map3d/headJoints.ts, nodes/web_ui/frontend/src/map3d/headJoints.test.ts, nodes/web_ui/frontend/src/map3d/layers.ts, nodes/web_ui/frontend/src/map3d/layers.test.ts, nodes/web_ui/frontend/src/map3d/MapScene.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.tsx]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [head-tab-pan-tilt-controls]
owns: []
uses: [head-urdf, web-head-fields, head-tab-types]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--head-in-3d-map-view
branch: pan-tilt-head-control-with-a--head-in-3d-map-view
verified_head: 6a401f5a9ad7d5b1e54cb6bf8bd34c987194e0a0
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: [a3]
process_gaps: []
---

# Head in 3D map view

## Requirement

- New `frontend/src/map3d/headJoints.ts` (D14, U7): `headJointValues(msg)` maps a `/head/joint_states` message to URDF joint values for `head_pan`/`head_tilt`, returning undefined before a message or when either value is missing or non-finite (the head is then not drawn: no placeholder pose).
- `map3d/layers.ts`: a `head` robot-part layer toggle (default on), next to body/wheels/arm.
- `map3d/MapScene.tsx`: `RobotLayer` draws `RobotModel` for `headUrdf` at the base origin (its URDF root is base_link) with `headJointValues(...)`, visible when the head layer is on and values exist.
- `tabs/MapNavTab.tsx`: passes `tab.head_urdf` and the `head_joint_states_topic` data to `MapScene`.
- Tests first in new `map3d/headJoints.test.ts` and `map3d/layers.test.ts`, seen failing: values from a message; undefined before a message and for missing/non-finite entries; the head layer exists and toggles.

## Acceptance conditions

- a1: head joint values come only from a real head joint state [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/headJoints.test.ts]
- a2: a head layer toggle exists [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts]
- a3: MapScene draws the head URDF with the measured joints [evidence: symbol: headJointValues]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/map3d/headJoints.ts`
- `nodes/web_ui/frontend/src/map3d/headJoints.test.ts`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/MapScene.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

## Interfaces

- `map3d/headJoints.ts:headJointValues`; `map3d/layers.ts` head layer; `MapScene.tsx` RobotLayer head model.

## Expected outcome

The map 3D view draws the head at its measured angles once a head joint state arrived.

## Steps

- Write `map3d/headJoints.test.ts` and extend `map3d/layers.test.ts`; run `cd nodes/web_ui/frontend && npm test -- headJoints layers`; expect failures.
- Implement; run `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`; commit `feat(web_ui): head in the 3D map view`.

## Notes

- `RobotModel` takes `jointStates` ({name, position}); the head URDF root is base_link, so it sits at the base origin like the base URDF.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-tab-pan-tilt-controls
