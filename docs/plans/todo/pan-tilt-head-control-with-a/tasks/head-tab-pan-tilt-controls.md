---
title: Head tab pan tilt controls
status: todo
repo: ros2-lerobot-sverve-platform
value: The user points the head from the web UI Head tab with two sliders and a Home button.
files: [nodes/web_ui/frontend/src/tabs/HeadTab.tsx, nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx, nodes/web_ui/frontend/src/types.ts, nodes/web_ui/frontend/src/hooks/subscription.ts, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [web-ui-head-command-backend]
owns: [head-tab-types]
uses: [web-head-fields]
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

# Head tab pan tilt controls

## Requirement

- `HeadTab.tsx` (D13, U7): the disabled placeholder (`pan/tilt: no head mount configured`) is replaced by live controls when the tab has `head_command_topic`: pan and tilt sliders with the configured ranges in degrees, initialised from the latest `head_joint_states_topic` sample and disabled (with a "waiting for /head/joint_states" line) until the first sample; dragging publishes `sensor_msgs/JointState` `{name: [head_pan, head_tilt], position: [rad, rad]}` at most every 200 ms and once on release (`onChangeCommitted`); a Home button publishes `head_home_deg`; the measured angles are shown as text. Without `head_command_topic` no controls are shown. The RGB click-to-zoom and ToF views from the blocking epics stay unchanged.
- `types.ts`: head tab fields (`head_command_topic`, `head_joint_states_topic`, `head_pan_range_deg`, `head_tilt_range_deg`, `head_home_deg`) and map_nav `head_urdf`, `head_joint_states_topic`.
- `hooks/subscription.ts`: head tabs subscribe `head_joint_states_topic`; map_nav tabs subscribe theirs.
- `nodes/web_ui/README.md`: Head tab controls section.
- Tests first in `HeadTab.test.tsx` (the "publishes nothing" assertions are replaced), seen failing: sliders disabled before the first sample and enabled after with the measured values; release publishes radians with both names; drag publishes no faster than 5 Hz; Home publishes the home pose; no controls without head_command_topic; tab design rules still pass.

## Acceptance conditions

- a1: Head tab sliders publish head_pan/head_tilt radians on release and throttled while dragging [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a2: Home publishes the configured home pose and sliders stay disabled until the first head joint state [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a3: the placeholder text is gone [evidence: command: ! grep -rn 'no head mount configured' nodes/web_ui/frontend/src]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`
- `nodes/web_ui/README.md`

## Interfaces

- `HeadTab.tsx` controls; `types.ts` head fields (part `head-tab-types`); `hooks/subscription.ts` head topics.

## Expected outcome

The Head tab moves the head with sliders and Home and shows the measured angles.

## Steps

- Update `HeadTab.test.tsx`; run `cd nodes/web_ui/frontend && npm test -- HeadTab`; expect failures.
- Implement; run `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`; commit `feat(web_ui): Head tab pan/tilt controls`.

## Notes

- Keep the RGB click-to-zoom and ToF parts from the blocking epics untouched.
- Respect `layout/designRules` (tabsDesign.test.ts) and TOUCH_TARGET_PX.
