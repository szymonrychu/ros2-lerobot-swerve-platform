---
title: Released group banners
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Wherever a released group could be commanded in the UI, the control is disabled and the user sees why."
files: [nodes/web_ui/frontend/src/calibrate/ReleasedBanner.tsx, nodes/web_ui/frontend/src/tabs/HeadTab.tsx, nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx, nodes/web_ui/frontend/src/tabs/ArmTab.tsx, nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/tabs src/calibrate && npm run lint"
depends_on: [calibrate-page-servo-section]
owns: []
uses: [servo-torque-hook]
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

# Released group banners

## Requirement

- New `frontend/src/calibrate/ReleasedBanner.tsx`: given a group id, uses `useServoGroups` (calibrate-page-servo-section) and renders a warning banner `<Label> torque is off - controls disabled (Calibrate page)` when that group's fresh state is released; renders nothing when not configured, unknown or enabled. Exposes `useGroupReleased(groupId) -> boolean`.
- `tabs/HeadTab.tsx`: banner for `head`; sliders, Home and every head command disabled while released.
- `tabs/ArmTab.tsx` (and the jog panel it renders): banner for `arm`; jog, Home, Set home, Take control and grasp actions disabled while released; Release stays enabled.
- `tabs/MapNavTab.tsx`: banner for `base`; sending a nav goal disabled while released; Stop stays enabled.
- Tests first, seen failing: `tabs/HeadTab.test.tsx`, `tabs/ArmTab.test.tsx`, `tabs/MapNavTab.toolbar.test.tsx` each mock the servo state: banner shown and controls disabled when released; no banner and controls enabled when enabled, unknown or not configured.
- `nodes/web_ui/README.md` `### Servo torque and neutral` documents the banners.

## Acceptance conditions

- a1: the Head tab shows the banner and disables head controls while the head is released [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a2: the Arm tab shows the banner and disables arm motion controls while the arm is released, Release stays enabled [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx]
- a3: the Map tab disables nav goals while the base is released, Stop stays enabled [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/calibrate/ReleasedBanner.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- Uses part `servo-torque-hook` (`useServoGroups`).
- `ReleasedBanner({ groupId })`, `useGroupReleased(groupId: string): boolean` in `calibrate/ReleasedBanner.tsx`.

## Expected outcome

Wherever a released group could be commanded in the UI, the control is disabled and the user sees why.

## Steps

- Extend the three tab tests; run `cd nodes/web_ui/frontend && npx vitest run src/tabs`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): released servo banners on the head, arm and map tabs`.

## Notes

- One shared hook instance per tab; poll interval from `useServoGroups` (1 s). The backend refuses the same commands (web-ui-quiet-released-groups); this is the visible half.
