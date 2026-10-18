---
title: Head in 3D map view
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head in 3D map view

## Requirement

- New `frontend/src/map3d/headJoints.ts` (D14, U7): `headJointValues(msg)` maps a `/head/joint_states` message to URDF joint values for `head_pan`/`head_tilt`, returning undefined before a message or when either value is missing or non-finite (the head is then not drawn: no placeholder pose).
- `map3d/layers.ts`: a `head` robot-part layer toggle (default on), next to body/wheels/arm.
- `map3d/MapScene.tsx`: `RobotLayer` draws `RobotModel` for `headUrdf` at the base origin (its URDF root is base_link) with `headJointValues(...)`, visible when the head layer is on and values exist.
- `tabs/MapNavTab.tsx`: passes `tab.head_urdf` and the `head_joint_states_topic` data to `MapScene`.
- Tests first in new `map3d/headJoints.test.ts` and `map3d/layers.test.ts`, seen failing: values from a message; undefined before a message and for missing/non-finite entries; the head layer exists and toggles.

## Acceptance conditions

- a1: head joint values come only from a real head joint state [evidence: test: nodes/web_ui/frontend/src/map3d/headJoints.test.ts]
- a2: a head layer toggle exists [evidence: test: nodes/web_ui/frontend/src/map3d/layers.test.ts]
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
