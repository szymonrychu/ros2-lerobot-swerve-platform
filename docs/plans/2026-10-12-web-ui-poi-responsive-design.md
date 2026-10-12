# Web UI POI tab, map pictograms, job dropdown, responsive overhaul, agent POI placement design

## Summary

The IMU tab is removed entirely. A new POI tab takes its place: one list holding every POI, with photo POIs alongside
points, objects and areas. Each row has a "show on map" button. It switches to the map, centres the POI and opens a
popup anchored to it.

On the map, point POIs are coloured by type and area POIs by zone. Photos keep their viewing-direction triangles, made
smaller.

The agent's job phases collapse into a dropdown behind the header chip.

The whole UI gets one layout model with three classes (phone, tablet, desktop): shared overlay panels that become
bottom sheets on phones, a map toolbar with an overflow menu, and relative sizes instead of hard-coded pixels.

The agent is made to place POIs reliably (D8).

## Intent and why

The user runs the robot from a phone, a tablet and a desktop. Today:

- POIs and photos live in separate overlay lists on the map tab.
- The IMU tab is of no use to them.
- The plan-tree panel takes a fixed 220 px of the chat on a phone.
- Only the `sm` breakpoint exists, so tablets get the desktop layout and the map toolbar wraps to several rows on
  phones.

The agent almost never marks POIs: over 7 days, `add_poi` ran 0 times and `update_poi` 2 times. That leaves the map
empty of the knowledge the agent gathers.

## Goals

- The IMU tab type (`imu_orientation`) no longer exists in the frontend, backend whitelist, default.yaml, Ansible
  client.yml, tests or README.
- A `poi` tab type exists and is deployed in the IMU tab's slot.
- The POI tab lists every POI from /poi/list and every photo POI from the photos API in one list, with:
  - kind filter chips (point, area, object, photo)
  - text search
  - a sort that matches the map's (sortPois)
- Each located row has a "show on map" button. It switches to the map tab, centres the POI (SceneController.centerOn)
  and opens a popup anchored at the POI with an arrow pointing at it.
- An unlocated photo (x/y null) shows with the button disabled and a reason.
- The map popup shows:
  - the name, kind, zone and status
  - the note
  - for a photo, the thumbnail and the capture time
  - actions that open the existing editor or PhotoPanel
- The popup closes with an X, a map tap or Escape, and stops pointer propagation so it never starts a map gesture.
- On the map, point and object POIs are filled with a per-type colour, and the status stays as the outline ring.
- Area POIs are filled and outlined with a per-zone colour (none, slow, keepout).
- Photo triangles are smaller (WEDGE_LENGTH_M reduced).
- The layers panel shows a legend for the type and zone colours.
- The chat's PhasePanel is collapsed by default. A header chip shows the active phase and its progress, and opens a
  dropdown with the full phase rows. The open or closed state is remembered per viewer in localStorage, wrapped in
  try/catch.
- Layout classes:
  - phone: below 600 px wide
  - tablet: 600 to 1199 px
  - desktop: 1200 px and above
- Each layout class is covered by tests of the layout hook and the shared panel component.
- The map toolbar fits one row on a phone: secondary groups go to an overflow menu.
- Overlay panels (POI list and editor, photo panel, grasp, layers) render as bottom sheets on phones and as floating
  cards on tablet and desktop. On desktop the POI side column can stay docked.
- Hard-coded px sizes from the brief's list are replaced by theme spacing, rem, %, dvh or clamp(). Touch targets stay
  at 44 px or more.
- The duplicated PANEL_BG moves to the theme.
- The agent places POIs as described in D8.

## Non-goals

- No change to the poi_store data model or its /poi/* topics, except where D8 needs it.
- No new map renderer: the map stays R3F/three.
- No offline or PWA work.
- The map tab keeps its own POI editor. The POI tab is a list plus a jump to the map, not a second editor.

## Anti-goals

- No regression in map gesture handling: goal setting, POI add and grasp pick still work on touch and mouse.
- No placeholder data on topics.
- No frame-rate regression: frameloop stays "demand", and markers are not one DOM node per photo.
- The deployed config must never reference a tab type the backend rejects.

## Constraints

- Tests first (vitest; component tests use `// @vitest-environment jsdom`).
- `npm test`, `npm run lint` and `npm run build` must stay green.
- Backend `uv run poe lint` / `uv run poe test` must stay green.
- Root `uv run pytest tests/`, `poe lint-ansible` and `poe test-ansible` must stay green.
- MUI 6 `sx` and theme breakpoints. No new UI library.
- Ansible and default.yaml stay in sync with the backend whitelist.
- Deploy goes through `./scripts/deploy-nodes.sh client web_ui` (plus mcp_server/claude_agent for D8, behind the
  agent-idle guard).

## Decisions

### D1: The poi tab type replaces imu_orientation everywhere

The new tab type is `poi` (id `poi`, label "POI"), in the IMU tab's position. `imu_orientation` is deleted:

- ImuOrientationTab.tsx
- the App lazy import and icon
- the SUPPORTED_TAB_TYPES entry
- the VALID_TAB_TYPES entry
- default.yaml and client.yml
- the pinned tests
- the README rows

The GPS-compass `valid_imu_orientation` in gps_anchor and bridge is unrelated and stays. The user chose full removal.

Serves A1.

### D2: POI data and focus requests are lifted to App; the map consumes a pending focus (my call)

Only the active tab is mounted (App.tsx:276).

- App parses /poi/list, holds the photo index, and holds `pendingFocus: {kind: 'poi'|'photo', id}`.
- "Show on map" sets pendingFocus and switches to the map tab.
- MapNavTab, once its SceneController exists, centres the POI, opens the popup and clears the request. It uses the
  same wait-for-controller pattern as the initial fit (MapNavTab.tsx:216-225).

The alternative was keeping MapNavTab mounted but hidden. It was rejected because a display:none R3F canvas measures
0x0 and then resizes, and a hidden map keeps its websocket-driven re-renders running.

Serves A1.

### D3: One list model for POIs and photos

A pure `poiItems.ts` merges Poi[] and Photo[] into `PoiItem {key, kind, name, x|null, y|null, status?, zone?, note,
updated, source}`, with filtering, search and sort as pure functions.

Photos come from one shared hook that pages through the photos API up to the layer limit. The POI tab and the map
photo layer share this hook rather than adding a third fetch path; usePhotoLayer is reused (my call).

Serves A1.

### D4: The popup is a drei Html anchored in the scene (my call)

- `MapPopup` renders a drei `<Html>` at the POI with pointerEvents auto and stopPropagation.
- It calls invalidate() on open and on content change, because frameloop is "demand".
- It has a CSS arrow pointing at the anchor.
- One popup at a time, so there is no per-marker DOM cost.

The alternative was a DOM overlay positioned by projecting the point. It was rejected because no project helper
exists and it would need a re-projection on every camera move.

Serves A1.

### D5: Markers coloured by type and zone; photo triangles smaller (my call)

- New `KIND_COLORS` in poi/style.ts: point, object, photo, area-none.
- `zoneStyle` colours stay: slow amber, keepout red, none neutral.
- PointMarker and ObjectMarker fill comes from the kind colour; the status colour moves to the outline ring.
- Text tags stay.
- WEDGE_LENGTH_M is halved.
- A legend goes in the layers panel.

The user said photos are triangles and could be smaller; points coloured by POI type; areas coloured by zone.

Serves A2.

### D6: The job phases become a header dropdown (my call)

- PhasePanel moves behind a header chip that reads "Phase n/m: name" plus PhaseProgress.
- A click toggles a Collapse below the header, of max height 50dvh, with the existing rows.
- It is closed by default on every layout class, and the state is remembered per viewer.
- The test hooks `phase-panel`, `phase-row-N` and `active-phase` stay.

Serves A3.

### D7: One layout model: useLayout plus OverlayPanel (my call)

- `layout/useLayout.ts` returns 'phone' | 'tablet' | 'desktop' from the theme breakpoints (sm=600, lg=1200).
- `layout/OverlayPanel.tsx` takes a title, open state, an anchor (left, right or bottom) and children. It renders:
  - on a phone: a bottom sheet with a drag handle, max height 60dvh
  - on a tablet or desktop: a floating card of width clamp(280px, 30vw, 380px)
- OverlayPanel owns stopPropagation and the PANEL_BG theme token.
- The map toolbar uses a priority list: on a phone, low-priority groups move into a "more" menu.
- App replaces its `narrow` check with useLayout: the drawer on a phone, scrollable icon+label tabs on a tablet, and
  full labels on a desktop.
- Hard-coded sizes are migrated per file, and each migration is an acceptance condition of the task that owns the file.

Serves A4.

### D8: The agent places POIs reliably

Root cause (brief docs/plans/2026-10-12-web-ui-poi-responsive-briefs/agent-poi-brief.md):
- The prompt (prompt.py:261-273) says agent POIs vanish at a new session and to keep things in NOTES.md, and the tool
  descriptions repeat it.
- Nothing in the code places POIs, nudges the agent to place them, or keeps them.

Decisions, with the user's answers:
- **D8a, persistence.** Agent POIs survive new sessions. Runner.reset() clears only agent POIs whose status is done or
  cancelled. A config flag `agent_poi_reset: keep_open | clear_all` (default keep_open) in claude_agent config and
  Ansible keeps the old behaviour available. Tool descriptions and ZONE_HELP stop calling agent POIs session-scoped.
- **D8b, auto stuck point.** mcp_server `auto_poi.py` AutoPoiMarker turns a collision_stop or a Nav2 abort, with a
  fresh pose, into a point POI:
  - created_by agent, name "stuck: <reason>", with a time and count note
  - merged within merge_radius_m when it repeats
  - no POI without a pose (no placeholder)
  - auto_poi_id in the digest event, so the agent can upgrade it to a keepout area
  - never a keepout by itself
- **D8c, nudges in tool results.** After `hint_after_images` (default 3) camera or look_around results without a POI
  write, the result carries a `poi_hint`. Writes reset the counter. complete_phase flags map coordinates in its
  summary that have no POI within poi_hint_radius_m. A failing list_pois never breaks complete_phase.
- **D8d, one turn-end POI review.** At most one extra turn, only for an instruction that used camera or motion tools
  and made 0 POI writes. Never after a user stop or the turn cap. Config flag poi_review_followup (default true).
- **D8e, prompt.** A short "Map marking" section right after the safety rules, with event triggers:
  - look_around, or a camera image that shows an object
  - a collision_stop or Nav2 abort (upgrade the auto point)
  - before complete_phase
  - an object that moved

  It states that POIs persist and that NOTES.md holds no map coordinates.
- No category field: the user chose to colour by the existing kind (D5).
- Agent-made zones stay active immediately, as today (my call).
- Explored rooms are not auto-marked (my call).

Serves A5.

### D9: Arm tab: two cameras, arm-glued views, jog panel, control handover, leader teleop restored

Brief: docs/plans/2026-10-12-web-ui-poi-responsive-briefs/arm-brief.md.

- **D9a, tab.** A new `arm` tab type replaces the gripper `camera` tab in default.yaml, client.yml and the pinned
  config tests. It always shows the gripper camera stream and the overview camera stream; on a phone they stack, on a
  tablet and desktop they sit side by side with the 3D view.
- **D9b, arm-glued views (Arm tab only, the user's choice).** The Arm tab has its own 3D robot view with camera modes:
  - side-left and side-right, keeping the arm at the screen side
  - above
  - gripper

  Each pose is computed in shoulder_link, the frame after shoulder_pan, so a pan does not move the arm relative to the
  camera. The gripper mode uses the gripper link and the mount from client.yml. The pure pose maths lives in
  armCamera.ts and is unit tested, including invariance under shoulder_pan. The solid arm model is exposed through an
  onRobotLoaded passthrough. The map tab keeps only top-down and orbit.
- **D9c, jog (the user said both).** The jog panel has:
  - cartesian steps in the gripper frame: up, down, forward, back, left, right
  - shoulder-pan steps: left and right
  - wrist pitch: up and down
  - wrist roll: left and right
  - the gripper: open and close

  All steps go through mcp_server: cartesian via IK (`move_cartesian` / `solve_cartesian`) with the body-collision,
  floor and reach checks, and joint steps via move_joints with limit checks. Step sizes are capped in config. A jog is
  refused while joints are stale, under the battery cut-off, while another motion runs (coalesce: the last request
  wins), or when the target collides or is unreachable.
- **D9d, control.** Take control, Release and Home buttons. Take holds the arm lease for the web UI; the active source
  shows from /filter/active_source. Release hands the follower back to the leader. Home reuses /arm/home. The routes
  are battery-gated, and the agent's lease and the web UI lease arbitrate through the existing control-lease mechanism.
- **D9e, leader teleop restored (the user's choice).** A dedicated, purpose-built leader-to-follower bridge carries
  only /leader/joint_states from the server to /filter/input_joint_updates on the client. It replaces the use of the
  disabled master2master. It goes through Ansible on both hosts, with the existing proximity hand-back (0.15 rad)
  respected. The design for the transport (a minimal DDS relay or the existing ros2-master discovery) is left to the
  builder, and it must be verified on the robot without motion: joint_states flow observed, the follower stays
  unpowered or holding.

Serves A6.

### D10: Agent job limits are generous and set per task; no fixed 15-minute timer

The user reported that jobs keep hitting the timer and caps although the agent was capable of finishing them. They
asked for a timer per task or none, and for generous limits: overshooting and delivering beats undershooting.

Today:
- a fixed watchdog of instruction_timeout_s 900 s
- min_effector_time_left_s 60 s
- phase caps of 40 effector calls and 40 turns, instruction maxima of 150 and 200
- raise_phase_budget allowed only once per phase

Decision:
- **D10a, timer per task.** agent.set_task_plan takes a required `time_budget_min` for the whole task.
  - The watchdog fires at that budget, not at a fixed 900 s. Before a plan exists the old 900 s pre-plan watchdog
    applies.
  - The budget is bounded by a config ceiling max_instruction_time_min (default 120).
  - raise_phase_budget (and revise_plan) can raise the time budget, as can a new rationale-carrying field.
  - min_effector_time_left_s counts against the declared budget.
  - The web UI job header shows the declared budget and the time left.
- **D10b, generous caps.** max_phase_rw_cap 100, max_phase_turn_cap 100, max_rw_cap 400 and max_turn_cap 500, in
  config defaults and Ansible. raise_phase_budget may be called more than once per phase, within the maxima.
- **D10c, prompt.** The planning section tells the agent to set caps and time at about twice its honest estimate,
  that overshooting and delivering is preferred to undershooting and stopping, and to raise budgets early rather than
  end unfinished.

Serves A7.

## Research

| lens | ran or skipped | adopted | rejected |
|---|---|---|---|
| research-code-internal | ran (codebase-cartographer brief: docs/plans/2026-10-12-web-ui-poi-responsive-briefs/brief.md) | SceneController.centerOn, the usePoiEditor.focus pattern, the drei Html label pattern, poi/style.ts, usePhotoLayer, the Menu pattern from the grasp dropdown | a project-point helper (not present) |
| research-code-external | skipped: standard MUI responsive patterns, no open fork | - | - |
| research-papers | skipped: UI work, no algorithmic core | - | - |
| research-docs | skipped: MUI 6 and drei Html are used in the same way as in existing code | - | - |
| agent POI research | ran (docs/plans/2026-10-12-web-ui-poi-responsive-briefs/agent-poi-brief.md) | see D8 | see D8 |

## Left to the builder

- The exact colours. They must be distinguishable on the dark map and must not reuse the status colours as fills.
- Which toolbar groups count as low priority. STOP and the goal or arm-control buttons always stay visible.
- Animation details of the bottom sheet.

## Assumptions and unknowns

- The photos API can page up to the existing usePhotoLayer limit within acceptable latency. This is confirmed by the
  existing layer, which does it already.
- Desktop docking of the POI column is optional polish within D7. A floating card is acceptable.

## Irreversible operations in scope

None. Removing the IMU tab is code and config only and can be reverted with git.
