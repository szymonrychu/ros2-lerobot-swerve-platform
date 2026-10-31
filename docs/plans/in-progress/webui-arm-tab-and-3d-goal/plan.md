---
title: Web UI arm controls on the ARM tab, goals in 3D
repos: []
created: 2026-10-31
owner: ""
rung: "1"
---

# Web UI arm controls on the ARM tab, goals in 3D

## What and why

What: the ARM tab gets every arm control the MAP tab has (draggable 3D arm, Arm home, Set home, Grasp), working and
with a visible reason whenever one is disabled; the MAP tab drops them; Set goal works in the 3D (orbit) view, not
only in Top view.

Why, from the user: "manual use of the arm is impossible. 'ARM' tab is missing controls from 'MAP' tab. MAP's arm
controls are always grey'ed out. Migrate the controls over to ARM tab. Allow to set goal also in 3d view."

Facts (discovery, static code; robot unreachable, runtime not observed):

- MAP toolbar: Grasp `disabled={armReleased}`, Arm home / Set home `disabled={busy || armReleased}`
  (nodes/web_ui/frontend/src/tabs/MapNavTab.tsx:790-826). `armReleased` is true when `/follower/torque_state` reports
  released, which the bridge sets when ANY arm joint is torque-off or unverified
  (nodes/bridges/feetech_servos/torque_control.py:71-77,150-158). One stuck joint greys them permanently.
- Map-tab drags go to `/filter/web_ui_joint_commands`; filter_node priority autonomy > web_ui > leader, so while the
  mcp_server arm lease is held (agent, or ARM tab Take control) drags are silently ignored.
- ARM tab: ArmView can drag the arm but the tab config has no `arm_command_topic` (ansible/group_vars/client.yml
  :2637-2648, web_ui/config.py ARM_TAB_DEFAULTS :116-128), so it is display-only. ARM_TAB_DEFAULTS already carries
  grasp and arm home/set-home topics and services, unused by ArmTab.tsx. ArmJogPanel jogs only after Take control.
- Set goal is disabled outside Top view (frontend/src/map/goalControl.ts:12-15) and the map opens in orbit view;
  README pins "Goal setting only in Top view" (nodes/web_ui/README.md:256). Ground pick `pickGround` already works for
  a perspective camera (ray vs z = 0).
- Related live epic: in-progress `servo-torque-off-and-neutral` (14/16 done) owns the released guard.

## Scope ladder

Rungs are cumulative: each rung includes everything above it.

### Rung 1: MVP - controls migrated, grey-out explained and fixed, goal in 3D (recommended)

- 1.1: ARM tab config gets `arm_command_topic` (client.yml, config/default.yaml, ARM_TAB_DEFAULTS): the 3D arm is
  draggable there.
- 1.2: Arm home / Set home (two-click) and Grasp move to the ARM tab; Grasp picks the object on the ArmView ground
  plane in base_link and converts with the robot pose; the MAP toolbar loses them (toolbarGroups.ts `armHome`,
  `grasp`) and the MAP 3D arm becomes display-only.
- 1.3: every arm control shows why it is disabled (released joint names from torque_state, action in flight, waiting
  for joint states, lease held by agent or Take control), and drags are disabled with that message while the lease
  is held instead of being silently ignored.
- 1.4: live diagnosis of "always greyed": read GET /api/calibration/servos and the Control chip; fix the cause in
  web_ui, or in the bridge torque verification if one joint is stuck released (coordinated with the
  servo-torque-off epic).
- 1.5: Set goal in the Map orbit view: drop the Top-view requirement in goalButtonState, keep goal mode when leaving
  Top view, in goal mode left button / one finger is the press-drag goal gesture (right button / two fingers still
  orbit and pan), picks clamped to a max distance near the horizon, draft arrow drawn.
- 1.6: tests (ArmTab.test.tsx, ArmView, ArmJogPanel, MapNavTab.toolbar, goalControl, goalGesture, picking,
  toolbarGroups, backend test_config) and web_ui README (Arm tab, Map tab, line 256) plus tests/README.md; deploy
  web_ui; live check on the robot with the gripper joint only (servo safety rule).
- Tradeoff: Grasp leaves the map, so the object is picked in the arm-glued view, not on the map; shallow 3D angles
  make far goal drops jumpy (clamped).
- Size: medium. Frontend tabs and tests, three config files, README; bridge fix size unknown until diagnosed.

### Rung 2: one control model for drag and jog

- 2.1: drags on the ARM tab route through mcp_server under the arm lease (new backend route plus mcp tool path), so
  drag and jog work together after Take control instead of drag being disabled.
- Tradeoff: new backend and mcp_server path; mcp_server deploy ends an agent session. Size: medium.

### Rung 3: complete - goals and picks on 3D content everywhere

- 3.1: Set goal also from the ARM tab 3D view; Add point / Add area in orbit view like goals.
- 3.2: picks hit the 3D map cloud surface instead of the z = 0 plane.
- Tradeoff: not asked explicitly; more gesture code. Size: medium.

## Agreed scope

Not agreed yet.

## Non-goals

- No fix for the dead ARM tab camera streams (gripper_uvc_camera disabled, /head_camera retired by the D435 epic).
- No change to filter_node arbitration priorities.
- No new arm motions or grasp strategies.

## Design

Controls are moved, not duplicated: the ARM tab reuses the existing components and API calls (`/api/arm/home`,
`/api/arm/set_home`, useGrasp), and the MAP tab code paths for them are deleted with their tests updated.

## Decisions

- D1: Grasp moves to the ARM tab with a base_link ground pick, rather than staying on MAP, because the request says
  migrate the controls. (my call, see Q2)

## Open questions

- Q1: "3d view" means the MAP tab orbit view (where goals already live)? [recommended: yes; the ARM tab 3D view is
  rung 3]
- Q2: Move Grasp to the ARM tab too (pick in the arm view), not keep it on MAP? [recommended: yes, move it]
- Q3: Remove the draggable arm from the MAP 3D view (display only there)? [recommended: yes, one place to drive the
  arm]
- Q4: If the grey-out is a stuck released joint in the bridge, fix it inside this plan? [recommended: yes, it is
  what makes manual use impossible]

## Research

- start-discovery brief: brief-webui.md.
- research-docs, research-code-external, research-papers: skipped, existing React/three code paths only.
- Verified here: MAP toolbar disable conditions (MapNavTab.tsx:790-826), goalButtonState Top-view rule
  (goalControl.ts), ARM tab config without arm_command_topic (client.yml:2637-2648), ARM_TAB_DEFAULTS (config.py:116).

## Irreversible operations

- Deploy of web_ui to the client (frontend rebuilt on the Pi); a bridge fix redeploys base/arm servos. Reversible by
  redeploying.

## Gate decision (2026-10-31)

User: "all with recommendations, go". Agreed scope is rung 1 only; every open question takes its recommended answer.
