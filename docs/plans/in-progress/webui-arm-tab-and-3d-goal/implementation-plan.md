# Web UI arm controls on the ARM tab, goals in 3D implementation plan

## Agreed rung

Rung 1 of plan.md (items 1.1 to 1.6), every open question on its recommended answer (Q1 "3d view" is the MAP orbit
view, Q2 Grasp moves to the ARM tab, Q3 the MAP 3D arm becomes display-only, Q4 a stuck released joint in the bridge
is fixed inside this plan). Rung 2 (drags under the lease through mcp_server) and rung 3 (goal from the ARM tab, Add
point / Add area in orbit view, picks on the 3D cloud) are not built. Anything found on the way goes to the
implementer's `discovered` list, not into the diff.

The robot is unreachable, so steps S1 to S11 are everything that can be built and verified offline. Item 1.4 (live
diagnosis of the grey-out and the bridge fix if a joint is stuck), the deploy and the live check are in
"Post-merge, robot online" at the end.

## Approach

- One tree, branch `webui-arm-tab-and-3d-goal`, base `main`, created with `workbench:worktree-create`. `<tree>` below
  is its path. Feature branch stays local (memory: no remote feature branches); merge to main locally, then push main.
- Chunks run sequentially in `<tree>`, one `workbench:implementer` per chunk, model **sonnet** for every chunk (no
  step needs opus). Prompt: this file's path and the chunk number.
- One review after chunk 4 (sonnet), at most two fix rounds by continuing the same agents, then
  `workbench:worktree-merger` (sonnet) merges with the Final verify as its check.

Rules for every step:

- Test first: write the test, run it, see it fail for the expected reason, then change the code. A step that only
  deletes behaviour names the "deletion pinned by a retirement test" exception and its pin. Doc-only steps name the
  doc exception.
- Commit per step: `<type>(web_ui): S<n> <title>` (scope `ansible` or `tests` where only those change).
- Commands (repo-defined):
  - Backend, from `<tree>/nodes/web_ui`: `uv run pytest tests/<file> -v`, lint `uv run poe lint`.
  - Frontend, from `<tree>/nodes/web_ui/frontend`: one file `npx vitest run <path>`, all `npm test`, types
    `npx tsc --noEmit`, lint `npm run lint` (0 warnings), build `npm run build`.
  - Root, from `<tree>`: `uv run pytest tests -q`, `uv run poe lint`, `uv run poe lint-ansible`,
    `uv run poe test-ansible`.
- Quote the pass/fail summary line of every verify. Python: type hints, docstrings with types, constants at the top.
  Plain ASCII in code and docs.

## Chunk 1: config and backend (model: sonnet)

### S1: The ARM tab owns arm_command_topic, the MAP tab drops it

- Files: `nodes/web_ui/web_ui/config.py` (`ARM_TAB_DEFAULTS` gains `"arm_command_topic":
  "/filter/web_ui_joint_commands"`; `MAP_NAV_DEFAULTS` loses `arm_command_topic`), `nodes/web_ui/config/default.yaml`
  (arm tab gains it, map tab loses it if set), `ansible/group_vars/client.yml` (move `arm_command_topic:
  /filter/web_ui_joint_commands` from the map tab, line ~2576, to the arm tab at ~2637-2648, with a one-line comment),
  `nodes/web_ui/tests/test_config.py`, `tests/test_servo_neutral_config.py`, plus any of
  `nodes/web_ui/tests/test_servo_quiet.py`, `tests/test_map3d_backend.py` that pin the map default.
- Covers: 1.1, Q3 (config half).
- Test first: in test_config.py, `test_arm_tab_defaults` expects `arm_command_topic ==
  "/filter/web_ui_joint_commands"` on an arm tab with no value set; a new test pins that a map_nav tab with no
  `arm_command_topic` keeps it None and that `publish_topics()` still lists the topic (from the arm tab); the default.yaml
  test pins the arm tab holds it and the map tab does not. In the root test_servo_neutral_config.py, a new assertion
  that the single tab carrying `arm_command_topic` (the existing `tab_value` rule) has `type: arm`. Run red.
- Change: the config moves above. `tab_value("arm_command_topic")` keeps its exactly-one rule.
- Verify: `cd <tree>/nodes/web_ui && uv run pytest tests/test_config.py tests/test_servo_quiet.py tests/test_map3d_backend.py -v`
  and `cd <tree> && uv run pytest tests/test_servo_neutral_config.py -q`

### S2: /api/grasp and /api/grasp/stop accept an arm tab id

- Files: `nodes/web_ui/web_ui/server.py` (`grasp_command`, `grasp_stop`: `find_map_nav_tab(tab) or
  find_arm_tab(tab)`, as `call_arm_trigger` already does; 404 message `no map_nav or arm tab ...`),
  `nodes/web_ui/tests/test_grasp.py`.
- Covers: 1.2 (backend half of Grasp on the ARM tab).
- Test first: POST `/api/grasp?tab=arm` with a plan body reaches `bridge_node.grasp_request_async` and answers like the
  map tab; POST `/api/grasp/stop?tab=arm` forwards the stop; an unknown tab id answers 404 with the new message; the
  released-arm 409 still applies to an arm tab id. Run red (404 today).
- Change: the lookup above; grasp timeouts come from the arm tab's TabConfig (shared fields, same defaults).
- Verify: `cd <tree>/nodes/web_ui && uv run pytest tests/test_grasp.py -v && uv run poe lint`

## Chunk 2: ARM tab controls (model: sonnet)

### S3: Disabled-reason model for arm controls

- Files: `nodes/web_ui/frontend/src/arm/armControls.ts` (new), `arm/armControls.test.ts` (new).
- Covers: 1.3.
- Test first: `releasedJoints(servo, 'arm')` returns the joint names whose `torque_state.joints` reading is 0, empty
  for a fresh enabled or a stale/missing state; `armControlState({ released, releasedJoints, busy, armReady,
  activeSource, graspBusy })` returns `{ disabled, reason }` for `drag`, `armHome`, `setHome`, `grasp`:
  released wins with `Arm torque is off: <joints> released (Calibrate page)` (joint list omitted when empty);
  then `Arm action in flight` (busy, for home/set home/grasp; grasp also on plan/execute running);
  drag only: `Arm held by the agent or Take control - Release to drag` when `activeSource === 'autonomy'`
  (reuse `HELD_SOURCE` from ArmJogPanel) and `Waiting for arm joint states` until `armReady`; enabled gives
  reason `''`. Run red (module missing).
- Change: the two pure functions, constants at the top, TSDoc with types.
- Verify: `cd <tree>/nodes/web_ui/frontend && npx vitest run src/arm/armControls.test.ts`

### S4: Draggable arm on the ARM tab with a visible reason when it is not

- Files: `frontend/src/arm/ArmView.tsx`, `arm/ArmView.test.tsx`, `tabs/ArmTab.tsx`, `tabs/ArmTab.test.tsx`.
- Covers: 1.1 (draggable there), 1.3 (drag reasons, no silent ignore while the lease is held).
- Test first: ArmView with `arm_command_topic` and `dragDisabledReason: null` renders the interactive arm; with a
  reason it renders the display-only arm and shows the reason text (`data-testid="arm-drag-reason"`); until
  `onArmReady(true)` it shows `Waiting for arm joint states`. ArmTab: with `/filter/active_source` = `autonomy` the
  drag reason names the lease; with a released arm torque state the reason lists the released joint names; a drag
  publish never reaches `publish` while released (existing guard kept). Run red.
- Change: ArmView takes `dragDisabledReason: string | null` and reports readiness from `InteractiveArm.onArmReady`;
  when a reason is set it renders `RobotModel` (so no drag can be sent) and an overlay caption. ArmTab computes the
  state with S3 from `useServoGroups`, `active_source_topic` and readiness.
- Verify: `cd <tree>/nodes/web_ui/frontend && npx vitest run src/arm/ArmView.test.tsx src/tabs/ArmTab.test.tsx`

### S5: Arm home and Set home on the ARM tab

- Files: `frontend/src/arm/ArmActions.tsx` (new), `arm/ArmActions.test.tsx` (new), `tabs/ArmTab.tsx`,
  `tabs/ArmTab.test.tsx`, `tabs/tabsDesign.test.ts` if it pins the ARM tab slots.
- Covers: 1.2 (home controls), 1.3 (their reasons).
- Test first: ArmActions renders `Arm home` and `Set home`; `Arm home` posts `/api/arm/home?tab=<arm tab id>` on one
  click; `Set home` needs two clicks within `RESET_CONFIRM_MS` (reuse `confirmClick`) and posts
  `/api/arm/set_home?tab=<id>`; while a call is in flight both are disabled with `Arm action in flight`; released
  disables both and shows the joint-name reason as visible text (`data-testid="arm-actions-reason"`, not only a
  tooltip, so phones see it) and as the button tooltip; the reply message shows as an alert. ArmTab renders
  ArmActions in a new `arm-actions-slot`. Run red.
- Change: ArmActions with the same request helper and result display the MAP toolbar used (`runAction` pattern,
  `ActionResult` from `map/mapActions`), reasons from S3.
- Verify: `cd <tree>/nodes/web_ui/frontend && npx vitest run src/arm/ArmActions.test.tsx src/tabs/ArmTab.test.tsx src/tabs/tabsDesign.test.ts`

### S6: Grasp on the ARM tab, picked on the ArmView ground plane

- Files: `frontend/src/arm/armViewPick.ts` (new), `arm/armViewPick.test.ts` (new), `arm/ArmView.tsx`,
  `arm/ArmView.test.tsx`, `arm/ArmActions.tsx`, `arm/ArmActions.test.tsx`, `tabs/ArmTab.tsx`, `tabs/ArmTab.test.tsx`,
  `grasp/useGrasp.ts` (doc comment only: no longer map-tab specific).
- Covers: 1.2 (Grasp), Q2, D1.
- Test first:
  - armViewPick.test.ts: `armViewGroundY(armOffset)` is `-armOffset[2]` (the floor, base_link z = 0, under the arm
    root that ArmView draws at the origin); `armViewHitToBaseLink(hit, armOffset)` adds the mount x, y; a hit above
    the horizon stays null. Use `pickGround` from `map3d/picking.ts` with that ground height.
  - ArmView.test.tsx: with `pickMode` a pointer click that does not move calls `onPick` with a base_link point; a
    press that moves (orbit) does not; the grasp preview group renders when `graspPreview` is set.
  - ArmActions/ArmTab tests: the Grasp menu lists `GRASP_MENU` strategies and is disabled with the S3 reason when
    released or a plan/execute runs; choosing one enters pick mode (hint `Click the object in the arm view`,
    `Cancel grasp pick` button, Esc cancels); the pick calls `grasp.applyClick(point, BASE_LINK_POSE)` with the
    identity pose (the point is already base_link, so `formFromClick` keeps it) and posts
    `/api/grasp?tab=<arm tab id>`; `GraspPanel` opens in the ARM tab; `STOP GRASP` shows only while a grasp
    executes and posts `/api/grasp/stop?tab=<arm tab id>`; a released arm closes the grasp flow. Run red.
- Change: ArmView takes `pickMode`, `onPick`, `graspPreview`; draws a floor grid at `armViewGroundY` and the
  `GraspPreview` inside a group at `rosToThree(-armOffset)` (base_link coordinates); in pick mode the arm is
  display-only. ArmTab owns `useGrasp({ tabId: tab.id, topicData, mount: tab.arm_offset ?? DEFAULT_ARM_OFFSET,
  notify })` and wires ArmActions, ArmView and GraspPanel. `BASE_LINK_POSE = { x: 0, y: 0, yaw: 0 }` is a named
  constant in armViewPick.ts. No robot pose is needed: the grasp request frame is base_link.
- Verify: `cd <tree>/nodes/web_ui/frontend && npx vitest run src/arm src/tabs/ArmTab.test.tsx src/grasp`

### S7: Jog panel says why it is disabled

- Files: `frontend/src/arm/ArmJogPanel.tsx`, `arm/ArmJogPanel.test.tsx`.
- Covers: 1.3 (jog controls).
- Test first: with the lease not held the jog buttons are disabled and a visible caption reads `Take control to
  jog`; released shows the S3 joint-name reason (Release stays enabled); busy shows `Arm action in flight`. Run red.
- Change: one reason caption under the jog rows, from S3 plus the not-held case; existing enable rules unchanged.
- Verify: `cd <tree>/nodes/web_ui/frontend && npx vitest run src/arm/ArmJogPanel.test.tsx`

## Chunk 3: MAP tab (model: sonnet)

### S8: MAP loses Grasp, Arm home, Set home and the draggable arm

- Files: `frontend/src/tabs/MapNavTab.tsx` (delete the grasp hook, menu, pick mode, panel, preview, STOP GRASP, Arm
  home / Set home, `armReleased` guard, `scenePublish` arm filter, `onArmReady`, the arm `ReleasedBannerView`; the base
  banner stays), `map/toolbarGroups.ts` (drop `grasp`, `armHome`, `stopGrasp`; `STOP_GROUPS` is `['stop']`,
  `stopGroupIds` takes no grasp flag), `map/toolbarGroups.test.ts`, `tabs/MapNavTab.toolbar.test.tsx`,
  `map3d/MapScene.tsx` (drop `armCommandTopic`, `onArmReady` and the `InteractiveArm` branch; always `RobotModel`),
  `map/LayersPanel.tsx` and its test (drop the arm drag help line).
- Covers: 1.2 (MAP toolbar loses them), Q3.
- Test first (retirement pins): MapNavTab.toolbar.test.tsx asserts no `Grasp`, `Arm home`, `Set home`, `Cancel grasp
  pick` or `STOP GRASP` buttons and no `released-banner-arm` even with a released arm state, and that nothing is posted
  to `/api/grasp`; toolbarGroups.test.ts pins `TOOLBAR_GROUPS` ids without `grasp`, `armHome`, `stopGrasp` and
  `ALWAYS_VISIBLE_GROUPS` as `['stop', 'goal']`; LayersPanel test pins no arm drag line. Replace the old grasp and arm
  home assertions in those files (they moved to S5/S6). Run red. Exception named: deletion pinned by a retirement test.
- Change: the deletions above; no new MAP behaviour.
- Verify: `cd <tree>/nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx src/map && npx tsc --noEmit`

### S9: Set goal in the MAP orbit view

- Files: `frontend/src/map/goalControl.ts`, `map/goalControl.test.ts`, `map3d/cameraBindings.ts` (new, extracted from
  `CameraRig` in `MapScene.tsx`), `map3d/cameraBindings.test.ts` (new), `map3d/MapScene.tsx`, `map3d/picking.ts`,
  `map3d/picking.test.ts`, `map3d/goalGesture.test.ts` (only if it pins Top view), `tabs/MapNavTab.tsx`,
  `tabs/MapNavTab.toolbar.test.tsx`, `map/LayersPanel.tsx` (orbit goal-mode help text).
- Covers: 1.5, Q1.
- Test first:
  - goalControl.test.ts: `goalButtonState` is enabled with tooltip `''` in orbit view; a tilt block still wins; no
    goal topic still disables. The `topView` input goes away.
  - cameraBindings.test.ts: `orbitBindings({ topView, goalMode })` returns today's four cases for the non-goal and
    Top-view-goal modes, and for orbit plus goal mode has no LEFT button and no ONE touch (left / one finger belong to
    the goal gesture) while RIGHT rotates, MIDDLE pans and TWO is DOLLY_PAN.
  - picking.test.ts: `pickGround(..., groundY, maxDistanceM)` clamps a hit farther than `maxDistanceM` (horizontal,
    from the camera's ground projection) to that distance on the same bearing; within range it is unchanged; above
    the horizon it is still null.
  - MapNavTab.toolbar.test.tsx: `Set goal` is enabled in the default (orbit) view, and goal mode survives toggling
    Top view on and off.
  Run red.
- Change: drop the Top-view rule and its hint; delete the effect that clears goal mode on leaving Top view; CameraRig
  uses `orbitBindings`; the controller pick passes `MAX_GOAL_PICK_M` (constant, 40 m); the draft arrow already renders
  from `draftGoal` in both views (confirm, fix if gated on Top view); Add point / Add area keep their Top-view rule
  (rung 3).
- Verify: `cd <tree>/nodes/web_ui/frontend && npx vitest run src/map src/map3d src/tabs/MapNavTab.toolbar.test.tsx`

## Chunk 4: docs and full green (model: sonnet)

### S10: README and tests README

- Files: `nodes/web_ui/README.md` (Map tab: replace line 256 "Goal setting only in Top view" with the orbit goal
  gesture and its bindings; remove Grasp and Arm home / Set home from the Map toolbar text; move the Grasp panel
  section under the Arm tab with the arm-view pick; Arm tab: draggable arm, Arm home / Set home, Grasp, disabled
  reasons, drag disabled while the lease is held (replaces line ~759); Calibrate line ~650: the draggable arm now
  sends while not released; Fields: `arm_command_topic` belongs to the arm tab; routes: `/api/grasp` accepts an arm
  tab id), `tests/README.md` (the new assertion in test_servo_neutral_config.py).
- Covers: 1.6 (docs).
- Test first: none, doc exception.
- Change: the edits above, plain ASCII.
- Verify: `grep -n "only in Top view" <tree>/nodes/web_ui/README.md` prints nothing.

### S11: Full green

- Files: only what the commands below flag.
- Covers: 1.6 (tests).
- Test first: none (verification step).
- Change: fix any failure from the earlier steps' changes; nothing new.
- Verify (Final verify):
  - `cd <tree>/nodes/web_ui && uv run pytest tests/ -v && uv run poe lint`
  - `cd <tree>/nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
  - `cd <tree> && uv run poe lint && uv run pytest tests -q && uv run poe lint-ansible && uv run poe test-ansible`

## Post-merge, robot online

Run after the merge to main and push, once `client.ros2.lan` answers. One general-purpose agent (sonnet); stop and
ask the user before any arm or base motion other than the gripper joint.

- P1 Reachability: `curl -s -o /dev/null -w '%{http_code}' http://client.ros2.lan:8080/` is 200. If not, report the
  plan as merged and pending the robot.
- P2 Deploy web_ui: invoke the `ansible-deploy` skill, then `mkdir -p .logs && ./scripts/deploy-nodes.sh client web_ui
  2>&1 | tee .logs/deploy-client-web_ui.log`. web_ui only: claude_agent and mcp_server are not restarted (memory:
  deploying them kills an agent session). Fix and redeploy on failure.
- P3 Diagnose the grey-out (1.4, Q4), no rclpy probes on the client and no manual SSH:
  - `curl -s http://client.ros2.lan:8080/api/calibration/servos` - arm group `state.released`, `stale`, and every
    joint whose `joints` reading is 0.
  - The ARM tab Control chip (`/filter/active_source`) and the new disabled reasons in the browser.
  - For servo telemetry use topic_scraper_api plus `scripts/topic_scraper_collect.py` (enable it in
    `ansible/group_vars/client.yml` and deploy it first), or the `teleop-diagnostics` skill.
- P4 Act on what P3 shows:
  - Not released, reason is the lease or orbit view: the new reasons explain it; no further fix; record it in the
    report.
  - Arm left released from the Calibrate page: ask the user to enable torque there (or approve doing it).
  - One joint stuck released or unverified in the bridge: `workbench:start-debugging` on
    `nodes/bridges/feetech_servos/torque_control.py` (lines 71-77, 150-158). First check the
    `servo-torque-off-and-neutral` epic tree for uncommitted or unmerged changes to that file and coordinate (rebase on
    it, do not duplicate). Regression test first in `nodes/bridges/feetech_servos/tests/`, run
    `cd nodes/bridges/feetech_servos && uv run pytest tests/ -v && uv run poe lint`, commit `fix(feetech_servos): ...`,
    push, then deploy the bridge node named by the `ansible-deploy` skill with logs in `.logs/`.
- P5 Live check (servo safety: gripper joint only, minimal amplitude):
  - ARM tab: the draggable arm moves the gripper joint a few degrees and back; with Take control held, drag shows the
    lease reason and sends nothing; Arm home / Set home / Grasp show their reasons when disabled. Do not press Arm home,
    Set home or execute a grasp without the user's go-ahead (full-arm motion or saved state).
  - MAP tab: no arm controls; in orbit view Set goal is enabled and the draft arrow follows press and drag. Publishing
    a real goal drives the base: only with the user's go-ahead.
  - Report each check with its observed result.
