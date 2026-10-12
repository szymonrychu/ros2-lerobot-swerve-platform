# Web UI POI tab, map colours, job dropdown, responsive overhaul and agent POI placement Implementation Plan

> For agentic workers: execute through `workbench:plan-execute` with
> `docs/plans/2026-10-12-web-ui-poi-responsive-tasks.json` as `args.tasks`. Each task is test-first: the test is written and
> seen failing before the code that makes it pass. Steps use checkbox syntax.

**Goal:** The IMU tab is replaced by a POI tab whose rows jump to an anchored popup on the map; markers are coloured by kind and zone with smaller photo triangles; job phases sit in a header dropdown; the UI works on phone, tablet and desktop; the agent places POIs reliably; and an Arm tab shows both cameras, arm-glued views and a safe jog panel with control handover, with leader teleop restored; and agent jobs run under a per-task time budget with generous caps.

**Architecture:** web_ui frontend (React 18 + MUI 6 + R3F) gets a `layout/` model (useLayout, OverlayPanel, PANEL_BG token) that the panel migrations build on. POI and photo data plus a pending focus request are lifted to App; MapNavTab consumes the request once its SceneController exists and opens a drei `<Html>` popup. The backend whitelist, default.yaml and Ansible change with the frontend in one task. D8 spans poi_store (clear by status), mcp_server (admin clear filter, AutoPoiMarker, poi_hint) and claude_agent (agent_poi_reset, prompt, complete_phase hint, turn-end review), with Ansible config last. D9 adds an `arm` tab type (replacing the gripper camera tab), pure arm-camera maths, a jog action and take/release services in mcp_server behind web_ui routes, and a new leader_bridge node deployed on both hosts. D10 replaces the fixed 900 s watchdog with a time budget declared in set_task_plan, raises the cap defaults and shows the budget in the job header. The poi_store model, /poi/* topics and the R3F renderer stay as they are, apart from the D8 clear filter.

**Tech stack:** TypeScript, React 18, MUI 6, @react-three/fiber/drei, vitest (`npx vitest run <file>`, `npm run lint`, `npx tsc --noEmit`, `npm run build`); Python 3 with pydantic, pytest and ruff via per-node uv (`uv run pytest tests/<file> -q && uv run poe lint`); Ansible (`uv run poe lint-ansible`).

**Spec:** `docs/plans/2026-10-12-web-ui-poi-responsive-design.md` (executors read both: the design doc for the why and the decisions, this plan for the what).

## Project brief

What: a POI tab replacing the IMU tab, kind/zone-coloured map markers, a job-phase dropdown, a responsive web UI, an agent that reliably marks the map, and an Arm tab for safe manual arm control with leader teleop back.
How: a shared layout model and OverlayPanel, App-level POI data with a pending map focus and drei popup, and agent-side persistence, auto stuck points, tool-result nudges, a turn-end review and a restructured prompt.
Why: the user drives the robot from phone, tablet and desktop, and the map is empty of what the agent learns (add_poi ran 0 times in 7 days), so places and photos are hard to find and reuse.

## Global constraints

- Tests first: every task writes its test, sees it fail for the stated reason, then implements. Component tests start with `// @vitest-environment jsdom`.
- Verify commands run from the repo root, as written in each task.
- Writing: no em dashes, smart quotes, arrows or decorative Unicode.
- Python: type hints everywhere, constants in SCREAMING_SNAKE_CASE at the top, docstrings with typed params and returns, pydantic config, no placeholder data on topics.
- No new UI library; MUI 6 `sx` and theme breakpoints only. frameloop stays "demand"; markers are never one DOM node per photo.
- The deployed config never references a tab type the backend rejects.
- Model: sonnet per task, opus only for T8 (popup plus focus flow across an unmount), T22 (monitor sink fan-out and the auto_poi wiring), T31 (mcp_server jog through IK and safety checks) and T35 (leader-teleop bridge).
- Motion on the robot: any live verification is gripper only, at minimal amplitude, with no base motion; the leader bridge is verified without motion (follower unpowered or holding).
- Docs (node READMEs, tests/README.md) are acceptance conditions of the task that owns the code.
- Deploy through `scripts/deploy-nodes.sh` only; deploying mcp_server or claude_agent kills a running agent session, so check it is idle first.

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| agent-poi-brief Plan inputs 1 | keep agent POIs across sessions, config-gated (design D8a changed the flag to agent_poi_reset keep_open / clear_all) | T17, T18, T19, T26 |
| agent-poi-brief Plan inputs 2 | tool descriptions stop calling agent POIs session-scoped | T20 |
| agent-poi-brief Plan inputs 3 | prompt "Map marking" section with event triggers; NOTES.md holds no coordinates | T21 |
| agent-poi-brief Plan inputs 4 | auto stuck-spot POIs in mcp_server (AutoPoiMarker, auto_poi_id in the digest) | T22, T26 |
| agent-poi-brief Plan inputs 5 | poi_hint nudge in camera and look_around results | T23, T26 |
| agent-poi-brief Plan inputs 6 | complete_phase nudge for unmarked map coordinates | T24, T26 |
| agent-poi-brief Plan inputs 7 | turn-end POI review follow-up (approved by the user) | T25, T26 |
| agent-poi-brief Plan inputs 8 | POI category field | rejected: the user chose to colour by the existing kind and zone (D5, D8) |
| agent-poi-brief recommended design 5 | Grafana panel on POI writes per instruction | rejected: not part of D8a-e as approved |
| brief.md section 11 | SceneController.centerOn, usePoiEditor.focus, drei Html label pattern, invalidate pattern | T8 |
| brief.md section 11 | poi/style.ts (poiStyle, zoneStyle, STATUS_COLORS, sortPois, formatUpdated), KIND_TITLES | T2, T3, T5, T6 |
| brief.md section 11 | photos helpers (photoWedge, capturedAt, photoThumbUrl, usePhotoLayer) | T4, T7, T8 |
| brief.md section 11 | PoiListPanel, PoiEditorPanel, PhotoPanel, PhotoList, GraspPanel | T12, T13, T14, T15 |
| brief.md section 11 | theme TOUCH_TARGET_PX, MUI useMediaQuery, Menu pattern of the grasp dropdown | T1, T9, T11 |
| brief.md section 6 | hard-coded px list and duplicated PANEL_BG | T1, T9, T10, T11, T12, T13, T14, T15, T16 |
| brief.md section 1 and 9 | every imu_orientation reference (frontend, whitelist, default.yaml, client.yml, pinned tests, README) | T7 |
| arm-brief Plan inputs 1 | arm tab type plumbing, gripper and overview cams, pinned config tests | T27, T28 |
| arm-brief Plan inputs 2 | arm-following camera modes, solid arm onRobotLoaded passthrough, armCamera.ts with shoulder_pan invariance | T29, T30 |
| arm-brief Plan inputs 3 | cartesian jog backend in mcp_server via IK with safety checks and step caps | T31 |
| arm-brief Plan inputs 4 | web_ui jog and control routes, /filter/active_source role, battery gate, 404/503 | T33 |
| arm-brief Plan inputs 5 | take/release Trigger services in mcp_server, home reuses /arm/home | T32 |
| arm-brief Plan inputs 6 | jog and control panel UI with active source and disabled states | T34 |
| arm-brief Plan inputs 7 | docs and deploy: READMEs land with T28, T31-T36; deploy and gripper-only live check after merge (Execution) | T28, T31, T32, T33, T34, T35, T36 |
| arm-brief open decisions | left/right jog: both (cartesian lateral and shoulder pan, D9c); leader teleop: a dedicated bridge, not master2master (D9e); map tab keeps its own arm controls | T31, T34, T35, T36 |
| arm-brief Reusable parts | CameraTab image rendering, RobotModel/InteractiveArm, call_arm_trigger, /api/grasp channel, move_cartesian, acquire/release, /filter/active_source | T27, T30, T31, T32, T33 |
| design D10 and coordinator code facts | per-task time budget, generous caps, repeatable raises, prompt headroom, job-header time left | T37, T38, T39, T40 |
| design Research | a project-point helper for a DOM popup | rejected: none exists; D4 uses a drei Html anchored in the scene |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 poi tab type replaces imu_orientation | T6, T7 |
| D2 POI data and focus lifted to App | T7, T8 |
| D3 one list model for POIs and photos | T2, T6, T7 |
| D4 popup as drei Html | T8 |
| D5 kind and zone colours, smaller triangles | T3, T4, T5 |
| D6 job phases as a header dropdown | T9 |
| D7 useLayout plus OverlayPanel | T1, T10, T11, T12, T13, T14, T15, T16 |
| D8 agent places POIs reliably (D8a-e) | T17, T18, T19, T20, T21, T22, T23, T24, T25, T26 |
| D9 Arm tab (D9a-e) | T27, T28, T29, T30, T31, T32, T33, T34, T35, T36 |
| D10 per-task time budget and generous caps (D10a-c) | T37, T38, T39, T40 |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| use-layout | T1 | T10, T11, T12, T16, T27 | `nodes/web_ui/frontend/src/layout/useLayout.ts` |
| overlay-panel | T1 | T12, T13, T14, T15, T34 | `nodes/web_ui/frontend/src/layout/OverlayPanel.tsx` |
| panel-bg-token | T1 | T12, T13, T14, T15, T16 | `nodes/web_ui/frontend/src/theme.ts` |
| poi-items | T2 | T6, T7, T8 | `nodes/web_ui/frontend/src/poi/poiItems.ts` |
| pending-focus-type | T2 | T6, T7, T8 | `nodes/web_ui/frontend/src/poi/poiItems.ts` |
| kind-colors | T3 | T5, T6 | `nodes/web_ui/frontend/src/poi/style.ts` |
| poi-tab-component | T6 | T7 | `nodes/web_ui/frontend/src/tabs/PoiTab.tsx` |
| app-poi-state | T7 | T8 | `nodes/web_ui/frontend/src/App.tsx` |
| map-tab-index | T7 | none | `nodes/web_ui/frontend/src/tabSelection.ts` |
| map-focus | T8 | none | `nodes/web_ui/frontend/src/map3d/mapFocus.ts` |
| map-popup | T8 | none | `nodes/web_ui/frontend/src/map3d/MapPopup.tsx` |
| phase-dropdown | T9 | none | `nodes/web_ui/frontend/src/agent/PhasePanel.tsx` |
| toolbar-groups | T11 | none | `nodes/web_ui/frontend/src/map/toolbarGroups.ts` |
| poi-clear-statuses | T17 | T18 | `nodes/poi_store/poi_store/models.py` |
| admin-clear-statuses | T18 | T19 | `nodes/mcp_server/mcp_server/tools.py` |
| agent-poi-reset-flag | T19 | T26 | `nodes/claude_agent/claude_agent/config.py` |
| auto-poi-marker | T22 | none | `nodes/mcp_server/mcp_server/auto_poi.py` |
| auto-poi-settings | T22 | T23, T26 | `nodes/mcp_server/mcp_server/config.py` |
| poi-write-counter | T23 | none | `nodes/mcp_server/mcp_server/tool_context.py` |
| poi-hint-radius | T24 | T26 | `nodes/claude_agent/claude_agent/config.py` |
| poi-review-followup-flag | T25 | T26 | `nodes/claude_agent/claude_agent/config.py` |
| arm-tab-component | T27 | T28, T30 | `nodes/web_ui/frontend/src/tabs/ArmTab.tsx` |
| camera-stream | T27 | none | `nodes/web_ui/frontend/src/components/CameraStream.tsx` |
| arm-tab-config | T28 | T30, T33 | `nodes/web_ui/web_ui/config.py` |
| arm-camera-pose | T29 | T30 | `nodes/web_ui/frontend/src/map3d/armCamera.ts` |
| arm-view | T30 | T34 | `nodes/web_ui/frontend/src/arm/ArmView.tsx` |
| jog-action | T31 | T33 | `nodes/mcp_server/mcp_server/jog.py` |
| arm-control-services | T32 | T33 | `nodes/mcp_server/mcp_server/ros_iface.py` |
| arm-routes | T33 | T34 | `nodes/web_ui/web_ui/server.py` |
| plan-time-budget | T37 | T38, T39, T40 | `nodes/claude_agent/claude_agent/plan_tools.py` |
| leader-bridge | T35 | T36 | `nodes/leader_bridge/leader_bridge/bridge.py` |

## Review focus

- T7: the whitelist, default.yaml, client.yml and every pinned test change together; no reference to imu_orientation remains outside the GPS-compass valid_imu_orientation and the legacy steamdeck_ui node.
- T8: the pending focus survives the tab switch and is consumed exactly once; the popup stops propagation, calls invalidate, and never breaks goal, POI-add or grasp gestures.
- T12 and T15: the phone panel defaults and the data-grasp-panel gesture check still work after the OverlayPanel move.
- T22: the monitor fan-out keeps ros_iface.publish_event receiving every event; no POI without a fresh pose; never a keepout.
- T19 and T25: a clear or follow-up failure never breaks reset or the turn; the follow-up fires at most once.
- T28: the GPS-fix fields in config.py, default.yaml and client.yml survive the arm tab change; the gripper camera mount matches mcp_server's.
- T31 and T32: every jog goes through IK plus collision, floor and reach checks; stale joints, battery cut-off and a foreign lease refuse; coalescing never queues unbounded.
- T35: the bridge carries only /leader/joint_states and never bypasses filter_node arbitration.
- T37: the watchdog moves to the declared budget only after set_task_plan, never past max_instruction_time_min; the pre-plan 900 s watchdog still holds before a plan.
- Hot files: MapNavTab.tsx (T5, T8, T11, T12, T15) and App.tsx (T7, T8, T10) are chained by dependsOn; the D9 tasks that share App.tsx, tabSelection, VALID_TAB_TYPES, client.yml and test_mcp_server_config.py (T28, T36) come after T7, T10, T16 and T26.

### Task 1: Layout foundation: useLayout hook, OverlayPanel component, PANEL_BG theme token

**Files:**
- Modify: `nodes/web_ui/frontend/src/layout/useLayout.ts`
- Modify: `nodes/web_ui/frontend/src/layout/OverlayPanel.tsx`
- Test: `nodes/web_ui/frontend/src/layout/layout.test.tsx`
- Modify: `nodes/web_ui/frontend/src/theme.ts`
- Modify: `nodes/web_ui/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: use-layout, overlay-panel, panel-bg-token

**Ownership:**
- owns: use-layout, overlay-panel, panel-bg-token
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/web_ui/frontend/src/layout/layout.test.tsx (first line `// @vitest-environment jsdom`), seen failing before the code exists. (1) `layout/useLayout.ts` exports `type LayoutClass = 'phone' | 'tablet' | 'desktop'` and `useLayout(): LayoutClass` built on the theme breakpoints: width < 600 -> 'phone', 600..1199 -> 'tablet', >= 1200 -> 'desktop'; the test covers each class and both boundaries (599/600, 1199/1200) by mocking matchMedia. (2) `layout/OverlayPanel.tsx` exports `OverlayPanel({title, open, anchor: 'left' | 'right' | 'bottom', onClose?, children})`: on 'phone' it renders a bottom sheet with a drag handle and max height 60dvh; on 'tablet' and 'desktop' a floating card of width clamp(280px, 30vw, 380px) anchored per `anchor`; closed renders nothing; the test covers each layout class. (3) OverlayPanel stops pointerdown propagation (test: a parent onPointerDown handler is not called on a pointerdown inside the panel). (4) theme.ts exports `PANEL_BG` ('rgba(22, 27, 34, 0.92)') and OverlayPanel uses it as its background (test asserts the token is applied). (5) Touch targets in OverlayPanel (close button, handle) are at least TOUCH_TARGET_PX. (6) nodes/web_ui/README.md gains a short 'Layout model' paragraph naming the three classes and OverlayPanel. npm run lint stays clean.

Notes: D7. Default MUI breakpoints already give sm=600, lg=1200 (brief section 6); use theme.breakpoints.up/down with useMediaQuery, no new library. TOUCH_TARGET_PX lives in theme.ts. Panels must stopPropagation like existing panels (brief section 6, MapNavTab.tsx:409). Do not migrate any existing panel here; that is T12-T15.

**Value:** One layout model every responsive change builds on, so phone, tablet and desktop behave consistently.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/layout/layout.test.tsx && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 2: poiItems: one list model merging POIs and photos, with filter, search and sort

**Files:**
- Modify: `nodes/web_ui/frontend/src/poi/poiItems.ts`
- Test: `nodes/web_ui/frontend/src/poi/poiItems.test.ts`

**Interfaces:**
- Consumes: existing code only
- Produces: poi-items, pending-focus-type

**Ownership:**
- owns: poi-items, pending-focus-type
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/web_ui/frontend/src/poi/poiItems.test.ts, seen failing. `poi/poiItems.ts` exports: (1) `type PoiItem = {key, kind: 'point'|'area'|'object'|'photo', name, x: number|null, y: number|null, status?, zone?, note, updated, source: 'poi'|'photo'}` and `type PendingFocus = {kind: 'poi'|'photo', id: string}`. (2) `toPoiItems(pois: Poi[], photos: Photo[]): PoiItem[]` - an area item takes its centroid as x/y; a photo with null x/y keeps null x/y; keys are unique across both sources. (3) `filterItems(items, kinds: Set<kind>, query: string)` - kind filter plus case-insensitive search over name and note. (4) `sortItems(items)` uses the same order as poi/style.ts sortPois for the POI part and places photos consistently (by updated, newest first). (5) `isLocated(item)` and `focusRequestFor(item): PendingFocus | null` (null for an unlocated item). Each function has JSDoc @param/@returns with types. npm run lint clean.

Notes: D3. Reuse sortPois / formatUpdated from poi/style.ts, Poi from poi/types.ts, Photo from photos/api.ts (x,y nullable). Area centroid: reuse a helper from poi/geometry.ts if one exists. Pure .ts, node test environment.

**Value:** POIs and photos show up in one list, so the user finds any place or picture in one spot.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/poi/poiItems.test.ts && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 3: Map markers coloured by POI kind and area zone

**Files:**
- Modify: `nodes/web_ui/frontend/src/poi/style.ts`
- Test: `nodes/web_ui/frontend/src/poi/style.test.ts`
- Modify: `nodes/web_ui/frontend/src/poi/PoiLayer.tsx`

**Interfaces:**
- Consumes: existing code only
- Produces: kind-colors

**Ownership:**
- owns: kind-colors
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/web_ui/frontend/src/poi/style.test.ts (extend), seen failing. (1) poi/style.ts exports `KIND_COLORS` with entries point, object, photo, area-none; no fill colour equals any STATUS_COLORS value. (2) A pure `markerColors(poi): {fill, outline}`: point and object fill comes from KIND_COLORS, outline from the status colour; area fill and outline come from zoneStyle (slow amber, keepout red, none neutral). The test covers point, object and each zone. (3) poi/PoiLayer.tsx PointMarker and ObjectMarker use markerColors (fill from kind, status ring as outline); AreaMarker uses the zone colour for fill and outline. Text tags stay. (4) frameloop 'demand' is respected: any colour change path keeps the existing invalidate pattern (PoiLayer.tsx:166-167). npm run lint clean.

Notes: D5. Exact colours are left to the builder: distinguishable on the dark map (CANVAS_BG) and not reusing status colours as fills. Existing: STATUS_COLORS, OBJECT_COLOR, CREATOR_OUTLINE, ZONE_COLORS, poiStyle, zoneStyle in poi/style.ts; markers at PoiLayer.tsx:55-152.

**Value:** The user tells a point, an object and a slow or keepout area apart at a glance on the map.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 4: Smaller photo triangles on the map

**Files:**
- Modify: `nodes/web_ui/frontend/src/photos/geometry.ts`
- Test: `nodes/web_ui/frontend/src/photos/geometry.test.ts`

**Interfaces:**
- Consumes: existing code only
- Produces: no shared part

**Ownership:**
- owns: none
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/web_ui/frontend/src/photos/geometry.test.ts (extend), seen failing: WEDGE_LENGTH_M is half its current value and photoWedge produces a wedge whose apex-to-edge length equals the new WEDGE_LENGTH_M. photos/geometry.ts halves WEDGE_LENGTH_M. Wedges stay one ShapeGeometry layer (no DOM node per photo). npm run lint clean.

Notes: D5. WEDGE_LENGTH_M at photos/geometry.ts:5, photoWedge :34.

**Value:** Photo markers stop covering the map, so POIs and the robot stay visible.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/photos/geometry.test.ts && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 5: Kind and zone colour legend in the map layers panel

**Files:**
- Modify: `nodes/web_ui/frontend/src/poi/PoiLegend.tsx`
- Test: `nodes/web_ui/frontend/src/poi/PoiLegend.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

**Interfaces:**
- Consumes: kind-colors
- Produces: no shared part

**Ownership:**
- owns: none
- uses: kind-colors
- dependsOn: T3

**Requirement:**
Test first in nodes/web_ui/frontend/src/poi/PoiLegend.test.tsx (jsdom), seen failing. (1) `poi/PoiLegend.tsx` renders one swatch plus label per KIND_COLORS entry (point, object, photo) and per zone (none, slow, keepout), with colours taken from KIND_COLORS and zoneStyle (the test asserts each swatch colour matches the source token). (2) tabs/MapNavTab.tsx renders PoiLegend inside the layers panel under the POI and photo layer toggles. (3) The legend swatch and line sizes use theme spacing or rem, not the hard-coded 14 px swatch / 22px legend line (MapNavTab.tsx:141, :953) for the new legend. npm run lint clean.

Notes: D5. MapNavTab.tsx is a hot file: chain T5 -> T8 -> T11 -> T12 -> T15. Existing swatch pattern MapNavTab.tsx:90-99.

**Value:** The user can read what each marker colour means without guessing.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/poi/PoiLegend.test.tsx && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 6: PoiTab component: unified list with kind chips, search, sort and show-on-map

**Files:**
- Modify: `nodes/web_ui/frontend/src/tabs/PoiTab.tsx`
- Test: `nodes/web_ui/frontend/src/tabs/PoiTab.test.tsx`

**Interfaces:**
- Consumes: poi-items, pending-focus-type, kind-colors
- Produces: poi-tab-component

**Ownership:**
- owns: poi-tab-component
- uses: poi-items, pending-focus-type, kind-colors
- dependsOn: T2, T3

**Requirement:**
Test first in nodes/web_ui/frontend/src/tabs/PoiTab.test.tsx (jsdom), seen failing. `tabs/PoiTab.tsx` default-exports a component taking `items: PoiItem[]` and `onShowOnMap(focus: PendingFocus)`. (1) It lists every item (points, areas, objects and photos together) in sortItems order. (2) Kind filter chips point, area, object, photo toggle filterItems; the test toggles each. (3) A text search filters by name and note. (4) Each row shows name, kind (with its KIND_COLORS swatch), zone and status where present, and the updated time. (5) A located row has a 'Show on map' button that calls onShowOnMap with focusRequestFor(item). (6) An unlocated photo (x/y null) shows the button disabled with a visible reason ('no map position'). (7) An empty list shows an empty state, not placeholder rows. (8) Sizes are relative (theme spacing, rem, %); buttons are at least TOUCH_TARGET_PX. npm run lint clean.

Notes: D1, D3. The tab is a list plus a jump, not a second editor (non-goal). Reuse KIND_TITLES pattern (PoiPanels.tsx:30), formatUpdated, capturedAt.

**Value:** The user browses every POI and photo in one tab and jumps to any of them on the map.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/tabs/PoiTab.test.tsx && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 7: Remove the IMU tab everywhere and deploy the poi tab in its slot

**Files:**
- Modify: `nodes/web_ui/frontend/src/App.tsx`
- Modify: `nodes/web_ui/frontend/src/tabSelection.ts`
- Test: `nodes/web_ui/frontend/src/tabSelection.test.ts`
- Modify: `nodes/web_ui/frontend/src/tabs/ImuOrientationTab.tsx`
- Modify: `nodes/web_ui/web_ui/config.py`
- Modify: `nodes/web_ui/config/default.yaml`
- Modify: `ansible/group_vars/client.yml`
- Test: `tests/test_mcp_server_config.py`
- Test: `nodes/web_ui/tests/test_map3d_backend.py`
- Modify: `nodes/web_ui/README.md`
- Modify: `tests/README.md`

**Interfaces:**
- Consumes: poi-items, pending-focus-type, poi-tab-component
- Produces: app-poi-state, map-tab-index

**Ownership:**
- owns: app-poi-state, map-tab-index
- uses: poi-items, pending-focus-type, poi-tab-component
- dependsOn: T1, T2, T6
- size: large
- sizeReason: The backend rejects unknown tab types, so frontend, backend whitelist, default.yaml, Ansible client.yml and the tests that pin them must change in one commit or the deployed UI fails config load (anti-goal; user rule: IMU removal is one task).

**Requirement:**
Tests first, seen failing: nodes/web_ui/frontend/src/tabSelection.test.ts (SUPPORTED_TAB_TYPES contains 'poi' and not 'imu_orientation'; a new `mapTabIndex(tabs)` returns the index of the map_nav tab or -1), nodes/web_ui/tests/test_map3d_backend.py (VALID_TAB_TYPES accepts 'poi' and rejects 'imu_orientation'; the default config tab types are map_nav, camera, camera, poi), tests/test_mcp_server_config.py (WEB_UI_TABS ends with 'poi' and tabs[4] type is 'poi', id 'poi', label 'POI'). Then: (1) tabs/ImuOrientationTab.tsx is deleted; App.tsx loses its lazy import, its TAB_ICONS entry (ThreeDRotationIcon import removed if unused) and its renderTab case. (2) App.tsx lazily imports tabs/PoiTab, adds a 'poi' icon and renderTab case; App parses /poi/list (parsePoiList of the tab's poi_list topic) and fetches photos through the shared usePhotoLayer hook (revision from /poi/photos), builds items with toPoiItems, and passes them to PoiTab. (3) App holds `pendingFocus: PendingFocus | null`; PoiTab's onShowOnMap sets it and switches activeTab to mapTabIndex(tabs); App passes `pendingFocus` and `onFocusConsumed` props to MapNavTab (MapNavTab consumption is T8; unused props must not break lint or tsc). (4) Backend nodes/web_ui/web_ui/config.py VALID_TAB_TYPES: 'imu_orientation' replaced by 'poi'. (5) nodes/web_ui/config/default.yaml and ansible/group_vars/client.yml replace the imu_graphs tab with `id: poi, type: poi, label: POI` in the same position, with the poi topics it needs. (6) nodes/web_ui/README.md tab table: IMU row removed, POI row added; tests/README.md row for test_web_ui_tab_set updated (imu_graphs -> poi). (7) The GPS-compass valid_imu_orientation in gps_anchor/bridge is untouched. (8) Ansible lint passes and no deployed config references a tab type the backend rejects.

Notes: D1, D2 (App side). References: App.tsx:46-51, :60-67, :77-100, :276; tabSelection.ts:11; config.py:30-39; default.yaml:63-67; client.yml:1339-1344; test_mcp_server_config.py:66, :469; test_map3d_backend.py:242, :251; README.md:15,17; tests/README.md:1574. nodes/steamdeck_ui and ansible/roles/steamdeck_ui also mention imu_graphs: that is a separate legacy node, out of scope - noted, not doing now. Usage of TabConfig poi topic fields: types.ts:50-52, config.py:70-76. Settled: the poi tab needs only poi_list_topic /poi/list and poi_photos_topic /poi/photos (config.py defaults and role mappings); no tab-level topics list.

**Value:** The useless IMU tab is gone and the user gets a POI tab in its place, with no config mismatch on deploy.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`(cd nodes/web_ui/frontend && npx vitest run src/tabSelection.test.ts && npm run lint && npx tsc --noEmit) && (cd nodes/web_ui && uv run pytest tests/test_map3d_backend.py -q && uv run poe lint) && uv run pytest tests/test_mcp_server_config.py -q && bash -o pipefail -c 'uv run poe lint-ansible </dev/null 2>&1 | tail -2'` - the test summary line shows all passed and lint is clean.

### Task 8: Show on map: the map consumes a pending focus, centres the POI and opens an anchored popup

**Files:**
- Modify: `nodes/web_ui/frontend/src/map3d/mapFocus.ts`
- Modify: `nodes/web_ui/frontend/src/map3d/MapPopup.tsx`
- Test: `nodes/web_ui/frontend/src/map3d/mapFocus.test.tsx`
- Modify: `nodes/web_ui/frontend/src/map3d/MapScene.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- Modify: `nodes/web_ui/frontend/src/App.tsx`
- Modify: `nodes/web_ui/README.md`

**Interfaces:**
- Consumes: app-poi-state, pending-focus-type, poi-items
- Produces: map-popup, map-focus

**Ownership:**
- owns: map-popup, map-focus
- uses: app-poi-state, pending-focus-type, poi-items
- dependsOn: T5, T7
- model: opus

**Requirement:**
Test first in nodes/web_ui/frontend/src/map3d/mapFocus.test.tsx (jsdom), seen failing. (1) Pure `map3d/mapFocus.ts` exports `resolveFocus(request: PendingFocus, pois: Poi[], photos: Photo[]): {anchor: Vec2, item: PoiItem} | null` (null for an unknown id or an unlocated photo) and the wait-for-controller helper used by MapNavTab, which retries until the SceneController exists, then calls centerOn once and reports consumption; the test covers immediate controller, late controller and unknown id. (2) `map3d/MapPopup.tsx` renders one drei `<Html>` at the anchor with pointerEvents auto, stops pointer propagation (test: a parent pointerdown handler is not called), and has a CSS arrow pointing at the anchor. It shows name, kind, zone, status, the note, and for a photo the thumbnail (photoThumbUrl) and capture time (capturedAt); its actions open the existing POI editor (usePoiEditor select) or PhotoPanel (openPhoto). (3) It closes on its X, on Escape and on a map tap; the test covers X and Escape. (4) It calls invalidate() on open and on content change (frameloop 'demand'); the test asserts invalidate is called. (5) tabs/MapNavTab.tsx takes `pendingFocus` and `onFocusConsumed` props from App: once controllerRef exists it centres the POI, opens the popup and calls onFocusConsumed, using the same wait pattern as the initial fit (MapNavTab.tsx:216-225). (6) MapNavTab consumes the photos App already fetches (passed as a prop) instead of its own usePhotoLayer call, so there is no third fetch path; App.tsx passes them. (7) Only one popup at a time; markers stay non-DOM. (8) No regression in goal setting, POI add or grasp pick gestures: the popup never starts a map gesture. (9) nodes/web_ui/README.md documents the show-on-map flow. npm run lint and tsc clean.

Notes: D2, D4. Only the active tab is mounted (App.tsx:276), so the request must survive the switch. SceneController.centerOn at map3d/MapScene.tsx:51-58, :168-177; usePoiEditor.focus :156-161; drei Html label pattern PoiLayer.tsx:27-53 and sceneLayers.tsx:412-432; invalidate pattern PoiLayer.tsx:166-167; openPhoto MapNavTab.tsx:269-275; gesture ownership MapNavTab.tsx:287-305, :409. Rejected alternatives: keep MapNavTab mounted hidden (D2), DOM overlay with projection (D4).

**Value:** One tap in the POI tab takes the user to that place on the map with its details in a popup.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/map3d/mapFocus.test.tsx && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 9: Agent job phases as a header dropdown, closed by default and remembered per viewer

**Files:**
- Modify: `nodes/web_ui/frontend/src/agent/PhasePanel.tsx`
- Test: `nodes/web_ui/frontend/src/agent/PhasePanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/AgentChatTab.tsx`

**Interfaces:**
- Consumes: existing code only
- Produces: phase-dropdown

**Ownership:**
- owns: phase-dropdown
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/web_ui/frontend/src/agent/PhasePanel.test.tsx (jsdom), seen failing. (1) The phase panel moves out of AgentChatTab.tsx into `agent/PhasePanel.tsx`: a header chip reads 'Phase n/m: name' plus PhaseProgress; a click toggles a Collapse below the header with max height 50dvh and the existing phase rows. (2) Closed by default on every layout class. (3) The open/closed state is read from and written to localStorage through helpers that take a storage stand-in and wrap every access in try/catch (a throwing storage still renders, closed); the test covers persisted open, persisted closed and throwing storage. (4) The test hooks data-testid `phase-panel`, `phase-row-N` and `active-phase` stay. (5) tabs/AgentChatTab.tsx renders the dropdown in the header and drops the always-open 220 px PhasePanel box. (6) AgentChatTab hard-coded layout sizes are migrated: RESULT_MAX_HEIGHT_PX and THUMB_MAX_PX to rem/dvh/clamp, PhasePanel maxHeight 220 removed, header buttons minHeight 36 raised to TOUCH_TARGET_PX; STICK_THRESHOLD_PX (a scroll threshold, not a layout size) may stay. (7) The existing agent tests (PhaseProgress.test.tsx, agentModel.test.ts) stay green. npm run lint clean.

Notes: D6, D7 (AgentChatTab migration). PhasePanel AgentChatTab.tsx:183-191, rows :154-180, header :397-415, sizes :66-68, :185, :251, :407, :410. Menu/Collapse precedent: grasp dropdown MapNavTab.tsx:642. Storage stand-in pattern: tabSelection.ts, layers.ts readLayerState.

**Value:** The plan tree no longer eats a third of the chat on a phone; the user opens it when needed.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/agent/PhasePanel.test.tsx src/agent/PhaseProgress.test.tsx src/agent/agentModel.test.ts && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 10: App shell per layout class: drawer on phone, scrollable icon+label tabs on tablet, full labels on desktop

**Files:**
- Modify: `nodes/web_ui/frontend/src/layout/tabBarMode.ts`
- Test: `nodes/web_ui/frontend/src/layout/tabBarMode.test.ts`
- Modify: `nodes/web_ui/frontend/src/App.tsx`

**Interfaces:**
- Consumes: use-layout
- Produces: no shared part

**Ownership:**
- owns: none
- uses: use-layout
- dependsOn: T1, T8

**Requirement:**
Test first in nodes/web_ui/frontend/src/layout/tabBarMode.test.ts, seen failing. (1) Pure `layout/tabBarMode.ts` exports `tabBarMode(layout: LayoutClass): {drawer: boolean, showIcons: boolean, showLabels: boolean, scrollable: boolean}`: phone -> drawer, tablet -> scrollable icon+label tabs, desktop -> full labels; the test covers all three. (2) App.tsx replaces its `narrow` useMediaQuery check (App.tsx:107) with useLayout and tabBarMode. (3) App.tsx hard-coded sizes are migrated: Toolbar minHeight 48 (:210) to the theme toolbar mixin or TOUCH_TARGET_PX-based value, Tab minWidth {xs 72, sm 90} (:238) to rem, Drawer list width 260 / maxWidth 85vw (:255) to clamp(). (4) Touch targets stay at least 44 px. npm run lint and tsc clean.

Notes: D7. App.tsx is a hot file: chain T7 -> T8 -> T10. Shell at App.tsx:209-265.

**Value:** Tablets stop getting the cramped desktop tab bar and phones keep a clean drawer.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/layout/tabBarMode.test.ts && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 11: Map toolbar fits one row on a phone: priority groups with a 'more' overflow menu

**Files:**
- Modify: `nodes/web_ui/frontend/src/map/toolbarGroups.ts`
- Test: `nodes/web_ui/frontend/src/map/toolbarGroups.test.ts`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

**Interfaces:**
- Consumes: use-layout
- Produces: toolbar-groups

**Ownership:**
- owns: toolbar-groups
- uses: use-layout
- dependsOn: T1, T8

**Requirement:**
Test first in nodes/web_ui/frontend/src/map/toolbarGroups.test.ts, seen failing. (1) Pure `map/toolbarGroups.ts` exports the toolbar group priority list and `splitToolbar(groups, layout: LayoutClass): {visible, overflow}`: on phone low-priority groups go to overflow; on tablet and desktop overflow is empty; STOP and the goal / arm-control groups are always visible on every class (the test asserts it). (2) tabs/MapNavTab.tsx renders visible groups in one row and the overflow groups in a 'more' MUI Menu; on a phone the toolbar no longer wraps. (3) MapNavTab toolbar sizes are migrated: toolbar Paper (:484), STOP px 3 (:494, :506), menu item 44 (:648), waiting text top/left 8 (:760), row minHeight 40 (:864), swatch 14 (:141), legend line 22px (:953) to theme spacing / rem; touch targets stay at least 44 px. (4) MapNavTab's useMediaQuery(down('sm')) (:175) is replaced by useLayout. npm run lint and tsc clean.

Notes: D7. Which groups are low priority is left to the builder, but STOP and goal / arm-control stay visible. Menu precedent MapNavTab.tsx:642. Hot file chain T5 -> T8 -> T11 -> T12 -> T15. Settled: the low-priority toolbar groups that go to the overflow menu are layers, POI and photo toggles, and camera view. STOP and goal/arm-control stay visible.

**Value:** On a phone the map controls stay in one row, so the map is not hidden behind wrapped buttons.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/map/toolbarGroups.test.ts && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 12: POI list and editor panels and the layers panel on OverlayPanel

**Files:**
- Modify: `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- Test: `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

**Interfaces:**
- Consumes: use-layout, overlay-panel, panel-bg-token
- Produces: no shared part

**Ownership:**
- owns: none
- uses: use-layout, overlay-panel, panel-bg-token
- dependsOn: T1, T11

**Requirement:**
Test first in nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx (jsdom, new), seen failing. (1) PoiListPanel and PoiEditorPanel render inside OverlayPanel: a bottom sheet on phone, a floating card on tablet and desktop; the test covers phone and desktop. (2) The local PANEL_BG constant in poi/PoiPanels.tsx (:28) is removed and the theme token used. (3) PoiPanels hard-coded sizes are migrated: header minHeight 40 (:67) to at least TOUCH_TARGET_PX, list maxHeight {xs 28vh, sm 40vh} (:78) and editor maxHeight {xs 60vh, sm 75vh} (:165) to dvh, row 48 (:85) to theme spacing; minHeight 44 (:253, :256) uses TOUCH_TARGET_PX. (4) In tabs/MapNavTab.tsx the POI column (:767-779, width {xs calc(100%-16px), sm 300}) and the layers Collapse (:846-851, width {xs calc(100%-16px), sm 260}, maxHeight {xs 55vh, sm 70vh}, PANEL_BG at :851) move onto OverlayPanel; on desktop the POI column may stay docked. (5) The phone panel defaults are kept: layers and POI list start closed on phone, layers closes when a POI is selected on phone, grasp open closes the others (MapNavTab.tsx:176-179, :284-286, :322-326). (6) Panels never start a map gesture. npm run lint and tsc clean.

Notes: D7. Hot file chain T5 -> T8 -> T11 -> T12 -> T15.

**Value:** POI panels become proper bottom sheets on a phone instead of covering the whole map.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 13: PhotoPanel on OverlayPanel with relative sizes

**Files:**
- Modify: `nodes/web_ui/frontend/src/photos/PhotoPanel.tsx`
- Test: `nodes/web_ui/frontend/src/photos/PhotoPanel.test.tsx`

**Interfaces:**
- Consumes: overlay-panel, panel-bg-token
- Produces: no shared part

**Ownership:**
- owns: none
- uses: overlay-panel, panel-bg-token
- dependsOn: T1, T12

**Requirement:**
Test first in nodes/web_ui/frontend/src/photos/PhotoPanel.test.tsx (extend), seen failing. (1) PhotoPanel renders inside OverlayPanel (bottom sheet on phone, floating card on tablet/desktop); the test covers phone and desktop. (2) The local PANEL_BG (:17) is removed and the theme token used. (3) The links list maxHeight 130 (:98) moves to rem/dvh. PANEL_IMAGE_PX 640 is an image request size, not a layout size, and may stay; the displayed image is sized with % / max-width. (4) Existing PhotoPanel tests stay green. npm run lint clean.

Notes: D7. Brief section 6: PhotoPanel.tsx:17, :19, :98. Ordered after T12 because the panel sits in the POI column T12 rebuilds.

**Value:** Photo details open as a usable sheet on a phone.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/photos/PhotoPanel.test.tsx && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 14: PhotoList on OverlayPanel with relative sizes

**Files:**
- Modify: `nodes/web_ui/frontend/src/photos/PhotoList.tsx`
- Test: `nodes/web_ui/frontend/src/photos/PhotoList.test.tsx`

**Interfaces:**
- Consumes: overlay-panel, panel-bg-token
- Produces: no shared part

**Ownership:**
- owns: none
- uses: overlay-panel, panel-bg-token
- dependsOn: T1, T12

**Requirement:**
Test first in nodes/web_ui/frontend/src/photos/PhotoList.test.tsx (extend), seen failing. (1) PhotoList renders inside OverlayPanel (bottom sheet on phone, floating card on tablet/desktop); the test covers phone and desktop. (2) The local PANEL_BG (:15) is removed and the theme token used. (3) Sizes migrate: list maxHeight {xs 24vh, sm 32vh} (:161) to dvh, TextField width 130 (:205, :209) to rem, THUMB_WIDTH_PX 72 (:16) to rem for the displayed width. (4) Existing PhotoList tests stay green. npm run lint clean.

Notes: D7. Ordered after T12 because the list sits in the POI column T12 rebuilds.

**Value:** The photo list fits a phone screen without covering the map.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/photos/PhotoList.test.tsx && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 15: GraspPanel on OverlayPanel with relative sizes

**Files:**
- Modify: `nodes/web_ui/frontend/src/grasp/GraspPanel.tsx`
- Test: `nodes/web_ui/frontend/src/grasp/GraspPanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

**Interfaces:**
- Consumes: overlay-panel, panel-bg-token
- Produces: no shared part

**Ownership:**
- owns: none
- uses: overlay-panel, panel-bg-token
- dependsOn: T1, T12

**Requirement:**
Test first in nodes/web_ui/frontend/src/grasp/GraspPanel.test.tsx (extend), seen failing. (1) GraspPanel renders inside OverlayPanel (anchor bottom; bottom sheet on phone, floating card on tablet/desktop); the test covers phone and desktop. (2) Its own 0.94 panel background is replaced by the PANEL_BG theme token. (3) Sizes migrate: field flex 1 1 90px (:58) to rem, maxHeight {xs 62vh, sm 70vh} (:134) to dvh. (4) In tabs/MapNavTab.tsx the GraspPanel wrapper (bottom/left 8, width {xs calc(100%-16px), sm 360}, :838) and the grasp alert (:828) use OverlayPanel / theme spacing; the data-grasp-panel gesture check (:409) keeps working. (5) Existing GraspPanel tests stay green. npm run lint and tsc clean.

Notes: D7. Hot file chain T5 -> T8 -> T11 -> T12 -> T15.

**Value:** The grasp controls work as a bottom sheet on a phone without blocking the pick gesture.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/grasp/GraspPanel.test.tsx && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 16: Remaining size migrations and a guard test against hard-coded sizes and duplicated PANEL_BG

**Files:**
- Test: `nodes/web_ui/frontend/src/layout/hardcodedSizes.test.ts`
- Modify: `nodes/web_ui/frontend/src/tabs/RgbdCameraTab.tsx`
- Modify: `nodes/web_ui/frontend/src/components/GpsChip.tsx`
- Modify: `nodes/web_ui/frontend/src/overlays/OverlayBar.tsx`
- Modify: `nodes/web_ui/README.md`

**Interfaces:**
- Consumes: use-layout, panel-bg-token
- Produces: no shared part

**Ownership:**
- owns: none
- uses: use-layout, panel-bg-token
- dependsOn: T1, T8, T9, T10, T13, T14, T15

**Requirement:**
Test first in nodes/web_ui/frontend/src/layout/hardcodedSizes.test.ts, seen failing. (1) The test reads the source of App.tsx, tabs/MapNavTab.tsx, poi/PoiPanels.tsx, photos/PhotoList.tsx, photos/PhotoPanel.tsx, grasp/GraspPanel.tsx, tabs/AgentChatTab.tsx, tabs/RgbdCameraTab.tsx, components/GpsChip.tsx and overlays/OverlayBar.tsx and asserts none of the brief's listed hard-coded layout values remain (for example width 260/300/360 px panels, maxHeight 220, vh-only maxHeights, calc(100% - 16px)), with an explicit allow-list for TOUCH_TARGET_PX, image request sizes and scroll thresholds. (2) It asserts the PANEL_BG rgba literal appears only in theme.ts. (3) The remaining migrations land: tabs/RgbdCameraTab.tsx preview height {xs 40%, sm 25%} and minHeight 120 (:86) to clamp()/dvh; components/GpsChip.tsx maxWidth {xs 120} (:42) to rem; overlays/OverlayBar.tsx replaces useMediaQuery(down('sm')) (:32) with useLayout and its spacing (:44, :54) uses theme spacing. (4) Full frontend `npm test`, `npm run lint` and `npm run build` are green. (5) tests/README.md is not affected (frontend tests are not root tests); nodes/web_ui/README.md 'Layout model' paragraph mentions the guard test.

Notes: D7. Runs after every migration task so the guard sees the finished tree. Brief section 6 is the list of values. Settled: the hard-coded sizes to migrate are listed in docs/plans/2026-10-12-web-ui-poi-responsive-briefs/brief.md section 6, with the line refs in T9-T15.

**Value:** The responsive overhaul stays done: a test fails the moment a fixed pixel size creeps back in.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/layout/hardcodedSizes.test.ts && npm test && npm run lint && npm run build` - the test summary line shows all passed and lint is clean.

### Task 17: poi_store clear command can be limited to statuses

**Files:**
- Modify: `nodes/poi_store/poi_store/models.py`
- Modify: `nodes/poi_store/poi_store/store.py`
- Test: `nodes/poi_store/tests/test_store.py`
- Modify: `nodes/poi_store/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: poi-clear-statuses

**Ownership:**
- owns: poi-clear-statuses
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/poi_store/tests/test_store.py (extend), seen failing. (1) The clear command in poi_store/models.py gains an optional `statuses: list[Literal['open','done','cancelled']] | None` (None keeps today's behaviour: every POI of that creator). (2) store.op_clear deletes only that creator's POIs whose status is in statuses when given; photos are never cleared; the result persists. The test covers None, ['done','cancelled'] (open agent POIs survive, done and cancelled ones go, user POIs untouched) and an empty list (nothing deleted). (3) Type hints and docstrings with types. (4) nodes/poi_store/README.md documents the field. Lint clean.

Notes: D8a. models.py:156-180 clear op, store.py:115, :339 op_clear; existing test test_clear_removes_only_pois_of_that_creator_and_persists (test_store.py:258). This is the only poi_store data-model change, allowed by the non-goal exception for D8.

**Value:** The store can drop finished agent POIs while keeping open ones, which session persistence needs.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/poi_store && uv run pytest tests/test_store.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 18: mcp_server admin clear route forwards a status filter

**Files:**
- Modify: `nodes/mcp_server/mcp_server/tools.py`
- Modify: `nodes/mcp_server/mcp_server/tool_context.py`
- Modify: `nodes/mcp_server/mcp_server/ros_iface.py`
- Test: `nodes/mcp_server/tests/test_tools.py`
- Modify: `nodes/mcp_server/README.md`

**Interfaces:**
- Consumes: poi-clear-statuses
- Produces: admin-clear-statuses

**Ownership:**
- owns: admin-clear-statuses
- uses: poi-clear-statuses
- dependsOn: T17

**Requirement:**
Test first in nodes/mcp_server/tests/test_tools.py (extend), seen failing. (1) POST /admin/clear_agent_pois accepts an optional JSON body `{statuses: [...]}`; without a body it clears every agent POI as today; with statuses it forwards them to poi_store's clear command. (2) ros_iface.poi_clear and tool_context.poi_clear take `statuses: list[str] | None = None` and put it on the /poi/command clear. (3) Invalid statuses return HTTP 400 with an error, never a partial clear. (4) nodes/mcp_server/README.md documents the body. Lint clean.

Notes: D8a. tools.py:81, :1190-1204; tool_context.py:175; ros_iface.py:1441.

**Value:** claude_agent can ask the robot to clear only finished agent POIs.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/mcp_server && uv run pytest tests/test_tools.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 19: claude_agent keeps open agent POIs across new sessions (agent_poi_reset flag)

**Files:**
- Modify: `nodes/claude_agent/claude_agent/config.py`
- Modify: `nodes/claude_agent/claude_agent/runner.py`
- Modify: `nodes/claude_agent/claude_agent/poi_clear.py`
- Test: `nodes/claude_agent/tests/test_runner.py`
- Test: `nodes/claude_agent/tests/test_poi_clear.py`
- Modify: `nodes/claude_agent/README.md`

**Interfaces:**
- Consumes: admin-clear-statuses
- Produces: agent-poi-reset-flag

**Ownership:**
- owns: agent-poi-reset-flag
- uses: admin-clear-statuses
- dependsOn: T18

**Requirement:**
Test first in nodes/claude_agent/tests/test_runner.py (extend), seen failing. (1) claude_agent/config.py gains `agent_poi_reset: Literal['keep_open', 'clear_all'] = 'keep_open'`. (2) Runner.reset() with keep_open asks the clearer to clear only agent POIs with status done or cancelled; with clear_all it clears every agent POI as before; the test covers both through the injected poi_clearer. (3) poi_clear.call_clear_agent_pois takes the statuses and sends them as the JSON body of the admin route; tests/test_poi_clear.py is extended for the body. (4) A clear failure still never breaks reset. (5) nodes/claude_agent/README.md documents the flag. Lint clean.

Notes: D8a. The agent brief proposed `clear_agent_pois_on_reset: bool`; the design replaced it with agent_poi_reset keep_open | clear_all. runner.py:62, :298, :492; poi_clear.py:45-70. The Ansible value is set in T26.

**Value:** POIs the agent places survive 'New session', so the map keeps what the agent learned.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/claude_agent && uv run pytest tests/test_runner.py tests/test_poi_clear.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 20: Tool descriptions stop calling agent POIs session-scoped

**Files:**
- Modify: `nodes/mcp_server/mcp_server/perception_tools.py`
- Test: `nodes/mcp_server/tests/test_perception_tools.py`

**Interfaces:**
- Consumes: existing code only
- Produces: no shared part

**Ownership:**
- owns: none
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/mcp_server/tests/test_perception_tools.py (extend), seen failing: no tool description or ZONE_HELP text says agent POIs are 'removed' at a new session or 'session-scoped'; the remember_object description and ZONE_HELP say agent POIs persist (open ones survive a new session). perception_tools.py ZONE_HELP and the remember_object description are updated accordingly. Lint clean.

Notes: D8a. perception_tools.py:56-58, :322-327.

**Value:** The tools stop telling the agent its POIs are throwaway, removing a reason it skips them.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/mcp_server && uv run pytest tests/test_perception_tools.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 21: Prompt 'Map marking' section with event triggers

**Files:**
- Modify: `nodes/claude_agent/claude_agent/prompt.py`
- Test: `nodes/claude_agent/tests/test_prompt.py`
- Modify: `nodes/claude_agent/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: no shared part

**Ownership:**
- owns: none
- uses: none
- dependsOn: T19

**Requirement:**
Test first in nodes/claude_agent/tests/test_prompt.py (replace the assertions at :209-215), seen failing. (1) prompt.py splits the Memory paragraph (:261-273): a short 'Map marking' section comes right after the safety rules. (2) It names the triggers: look_around or a camera image that shows an object; a collision_stop or Nav2 abort (upgrade the auto stuck point to a keepout or slow area if lasting); before complete_phase; an object that moved. (3) It states that POIs persist across sessions and that NOTES.md holds no map coordinates. (4) No prompt text says agent POIs are removed at a new session. (5) The test asserts section order, each trigger and both statements. (6) nodes/claude_agent/README.md prompt section is updated. Lint clean.

Notes: D8e. Ordered after T19 (README).

**Value:** The agent gets clear moments to mark the map instead of a buried optional paragraph.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/claude_agent && uv run pytest tests/test_prompt.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 22: mcp_server AutoPoiMarker: collision_stop and Nav2 abort become stuck point POIs

**Files:**
- Modify: `nodes/mcp_server/mcp_server/auto_poi.py`
- Modify: `nodes/mcp_server/mcp_server/monitor.py`
- Modify: `nodes/mcp_server/mcp_server/ros_iface.py`
- Modify: `nodes/mcp_server/mcp_server/config.py`
- Modify: `nodes/mcp_server/mcp_server/tools.py`
- Test: `nodes/mcp_server/tests/test_auto_poi.py`
- Modify: `nodes/mcp_server/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: auto-poi-marker, auto-poi-settings

**Ownership:**
- owns: auto-poi-marker, auto-poi-settings
- uses: none
- dependsOn: T18
- model: opus

**Requirement:**
Test first in nodes/mcp_server/tests/test_auto_poi.py (new), seen failing; tests/test_monitor.py stays green. (1) New `mcp_server/auto_poi.py` with class `AutoPoiMarker` and `on_event(event: RobotEvent, pose) -> str | None`: a collision_stop or a Nav2 navigate_to_pose abort with a fresh pose adds a point POI with created_by 'agent', name 'stuck: <reason>', and a note with the time and a count. (2) A repeat within merge_radius_m updates that POI (count and time in the note) instead of adding one. (3) No pose, or a stale pose, means no POI (no placeholder). (4) It never creates a keepout or any area. (5) The created or updated id is returned and put on the digest event as `auto_poi_id` in robot_events_since_last_call. (6) monitor.py gets a sink fan-out (or a second sink) so ros_iface.publish_event and the marker both receive events (ros_iface.py:371 wiring). (7) Nav2 aborts are caught in tools.py after base-motion results. (8) config.py gains `AutoPoiSettings` (`auto_poi`: enabled, merge_radius_m, event types) with defaults; disabled means no POIs. (9) A poi_store failure is logged and never breaks the event path or the tool result. (10) nodes/mcp_server/README.md documents the marker and auto_poi_id. Lint clean.

Notes: D8b. monitor.py:329 set_sink, :358 emit, :831-852 collision_stop; ros_iface.py:371, :728; tools.py:190 DIGEST_EVENTS_KEY, :415 call_tool_with_digest; merge pattern object_memory.py:200-218 (objects.merge_radius_m). Config is a StrictModel: the Ansible block lands in T26. Settled: Nav2 aborts are hooked on the navigate_to_pose result status in call_tool_with_digest. The fresh-pose age limit is 2 s, as config field auto_poi.max_pose_age_s. The default event types are collision_stop and nav_abort.

**Value:** Every spot where the robot got stuck lands on the map automatically, so the agent and the user can fence it off.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/mcp_server && uv run pytest tests/test_auto_poi.py tests/test_monitor.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 23: poi_hint nudge in camera and look_around results

**Files:**
- Modify: `nodes/mcp_server/mcp_server/tool_context.py`
- Modify: `nodes/mcp_server/mcp_server/tools.py`
- Modify: `nodes/mcp_server/mcp_server/perception_tools.py`
- Modify: `nodes/mcp_server/mcp_server/lookup_tools.py`
- Modify: `nodes/mcp_server/mcp_server/config.py`
- Test: `nodes/mcp_server/tests/test_digest.py`
- Modify: `nodes/mcp_server/README.md`

**Interfaces:**
- Consumes: auto-poi-settings
- Produces: poi-write-counter

**Ownership:**
- owns: poi-write-counter
- uses: auto-poi-settings
- dependsOn: T18, T20, T22

**Requirement:**
Test first in nodes/mcp_server/tests/test_digest.py (extend), seen failing. (1) ToolContext counts camera and look_around results since the last POI write. (2) call_tool_with_digest adds a `poi_hint` string to camera and look_around results once the count reaches `auto_poi.hint_after_images` (default 3, in AutoPoiSettings). (3) add_poi, update_poi, remember_object and annotate_lookup reset the counter (perception_tools.py, lookup_tools.py); the next camera result after a write has no poi_hint. (4) Other tools never carry poi_hint. (5) nodes/mcp_server/README.md documents poi_hint. Lint clean.

Notes: D8c (mcp_server part).

**Value:** The agent is reminded to mark the map right when it has been looking around without marking.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/mcp_server && uv run pytest tests/test_digest.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 24: complete_phase flags map coordinates that have no POI nearby

**Files:**
- Modify: `nodes/claude_agent/claude_agent/plan_tools.py`
- Modify: `nodes/claude_agent/claude_agent/runner.py`
- Modify: `nodes/claude_agent/claude_agent/config.py`
- Test: `nodes/claude_agent/tests/test_poi_hint.py`
- Modify: `nodes/claude_agent/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: poi-hint-radius

**Ownership:**
- owns: poi-hint-radius
- uses: none
- dependsOn: T21

**Requirement:**
Test first in nodes/claude_agent/tests/test_poi_hint.py (new), seen failing. (1) plan_tools.complete_phase parses map coordinates in its summary (e.g. 'map (2.56,-0.79)'). (2) For each with no POI within `poi_hint_radius_m` (new claude_agent config field, default 0.5), the tool result appends 'Not on the map yet: (x, y) ... call remember_object / add_poi'. (3) A summary with a POI nearby, or with no coordinates, gives an unchanged result. (4) It reads POIs through the runner's robot tool caller (list_pois); a failing or timed-out list_pois never breaks complete_phase and adds no hint. (5) nodes/claude_agent/README.md documents the hint and the field. Lint clean.

Notes: D8c (claude_agent part). plan_tools.complete_phase around :434; runner robot_tool_caller runner.py:284-304; call_robot_tool in robot_stop.py. Settled: the coordinate regex is \(\s*-?\d+(\.\d+)?\s*,\s*-?\d+(\.\d+)?\s*\), matched only after the word 'map'.

**Value:** Places the agent writes into its phase summaries end up on the map instead of only in text.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/claude_agent && uv run pytest tests/test_poi_hint.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 25: One turn-end POI review follow-up

**Files:**
- Modify: `nodes/claude_agent/claude_agent/runner.py`
- Modify: `nodes/claude_agent/claude_agent/config.py`
- Test: `nodes/claude_agent/tests/test_job_end.py`
- Modify: `nodes/claude_agent/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: poi-review-followup-flag

**Ownership:**
- owns: poi-review-followup-flag
- uses: none
- dependsOn: T24

**Requirement:**
Test first in nodes/claude_agent/tests/test_job_end.py (extend), seen failing. (1) claude_agent/config.py gains `poi_review_followup: bool = True`. (2) Beside check_job_end, the runner sends at most one 'POI REVIEW' follow-up per instruction, only when the instruction used camera or motion tools and made 0 POI writes (add_poi, update_poi, remember_object, annotate_lookup). (3) Never after a user stop or the turn cap, never a second time, and never when the flag is false. (4) It does not interfere with the JOB END REFUSED follow-ups. The test covers each case. (5) nodes/claude_agent/README.md documents it. Lint clean.

Notes: D8d (agent brief item 7, approved). runner.py:105 JOB_END_FOLLOWUP_MARKER, :1097-1117 check_job_end. Settled: 'camera or motion tools' means the tool-name sets the digest already tracks (reuse them; no new list).

**Value:** An instruction that explored without marking anything gets one last chance to put it on the map.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/claude_agent && uv run pytest tests/test_job_end.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 26: Ansible client.yml carries the D8 config and root tests pin it

**Files:**
- Modify: `ansible/group_vars/client.yml`
- Test: `tests/test_claude_agent_config.py`
- Test: `tests/test_mcp_server_config.py`
- Modify: `tests/README.md`

**Interfaces:**
- Consumes: agent-poi-reset-flag, auto-poi-settings, poi-hint-radius, poi-review-followup-flag
- Produces: no shared part

**Ownership:**
- owns: none
- uses: agent-poi-reset-flag, auto-poi-settings, poi-hint-radius, poi-review-followup-flag
- dependsOn: T7, T19, T22, T23, T25

**Requirement:**
Tests first, seen failing: tests/test_claude_agent_config.py asserts the deployed claude_agent config sets agent_poi_reset: keep_open, poi_review_followup: true and poi_hint_radius_m, and validates against ClaudeAgentConfig; tests/test_mcp_server_config.py asserts the deployed mcp_server config has an auto_poi block (enabled, merge_radius_m, event types, hint_after_images) that validates against the mcp_server config model. (1) ansible/group_vars/client.yml gains those keys in the claude_agent and mcp_server node configs. (2) tests/README.md documents the new tests. (3) Ansible lint passes.

Notes: D8a-d. CLAUDE.md: Ansible stays in sync with node config. Deploying mcp_server/claude_agent kills a running agent session: deploy behind the agent-idle guard (memory).

**Value:** The robot actually runs with the new POI behaviour after deploy, and a test catches config drift.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q && bash -o pipefail -c 'uv run poe lint-ansible </dev/null 2>&1 | tail -2'` - the test summary line shows all passed and lint is clean.

### Task 27: ArmTab cameras: gripper and overview streams, stacked on phone, side by side otherwise

**Files:**
- Modify: `nodes/web_ui/frontend/src/components/CameraStream.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/CameraTab.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- Test: `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`

**Interfaces:**
- Consumes: use-layout
- Produces: arm-tab-component, camera-stream

**Ownership:**
- owns: arm-tab-component, camera-stream
- uses: use-layout
- dependsOn: T1

**Requirement:**
Test first in nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx (jsdom), seen failing. (1) The image rendering of tabs/CameraTab.tsx is extracted into components/CameraStream.tsx (props: topic data entry, alt label) and CameraTab uses it unchanged in behaviour. (2) tabs/ArmTab.tsx default-exports the Arm tab: it always renders the gripper camera stream (tab.gripper_camera_topic) and the overview camera stream (tab.overview_camera_topic) via CameraStream, plus a slot for the 3D view (filled in T30) and the jog panel (T34). (3) Layout by useLayout: phone stacks the streams and the view vertically; tablet and desktop place the streams side by side with the 3D view slot. The test covers phone and desktop. (4) A stream with no frame yet shows the existing waiting message, never a placeholder image. (5) Sizes are relative (%, dvh, clamp); no hard-coded px layout. npm run lint clean.

Notes: D9a. CameraTab.tsx renders topicData[tab.topic].jpeg_b64 (42 lines). WaitingMessage.tsx exists. The tab config fields are defined in T28; until then type the props locally from TabConfig with optional fields so tsc stays clean.

**Value:** The user sees the gripper and overview cameras together on any device.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 28: arm tab type replaces the gripper camera tab in frontend, backend, default.yaml and Ansible

**Files:**
- Modify: `nodes/web_ui/web_ui/config.py`
- Modify: `nodes/web_ui/config/default.yaml`
- Modify: `ansible/group_vars/client.yml`
- Modify: `nodes/web_ui/frontend/src/tabSelection.ts`
- Test: `nodes/web_ui/frontend/src/tabSelection.test.ts`
- Modify: `nodes/web_ui/frontend/src/App.tsx`
- Modify: `nodes/web_ui/frontend/src/types.ts`
- Test: `tests/test_mcp_server_config.py`
- Test: `nodes/web_ui/tests/test_config.py`
- Modify: `nodes/web_ui/README.md`
- Modify: `tests/README.md`

**Interfaces:**
- Consumes: arm-tab-component
- Produces: arm-tab-config

**Ownership:**
- owns: arm-tab-config
- uses: arm-tab-component
- dependsOn: T7, T10, T16, T26, T27
- size: large
- sizeReason: The backend rejects unknown tab types, so the whitelist, the TabConfig fields, the frontend types and tab registry, default.yaml, Ansible client.yml and the pinned tests must change in one commit or the deployed UI fails config load.

**Requirement:**
Tests first, seen failing: nodes/web_ui/frontend/src/tabSelection.test.ts (SUPPORTED_TAB_TYPES contains 'arm'), nodes/web_ui/tests/test_config.py (VALID_TAB_TYPES accepts 'arm'; an arm TabConfig validates gripper_camera_topic, overview_camera_topic, arm_joint_states_topic, gripper_camera_mount {xyz, rpy}, arm_offset, active_source_topic and the arm Trigger service fields; trigger_services() includes the arm tab's services), tests/test_mcp_server_config.py (WEB_UI_TABS has 'arm' in place of 'camera'; the arm tab carries both camera topics and its gripper_camera_mount equals the mcp_server gripper camera mount in client.yml). Then: (1) web_ui/config.py adds 'arm' to VALID_TAB_TYPES and the arm TabConfig fields with inline comments and arm defaults. (2) frontend types.ts gains the same fields; tabSelection.ts adds 'arm'; App.tsx lazily imports tabs/ArmTab, adds an icon and a renderTab case. (3) nodes/web_ui/config/default.yaml and ansible/group_vars/client.yml replace the `id: camera` Gripper Cam tab with `id: arm, type: arm, label: Arm` in the same position; the overview_camera tab stays. (4) nodes/web_ui/README.md tab table: Gripper Cam row replaced by the Arm row; tests/README.md rows for the changed tests updated. (5) Ansible lint passes; no deployed config references a tab type the backend rejects. (6) The GPS-fix fields recently added to config.py, default.yaml and client.yml are kept intact.

Notes: D9a. Re-read after the GPS fix (0d3452b touched config.py, default.yaml, client.yml, test_config.py, README.md, tests/README.md). Gripper cam tab: default.yaml tabs[1], client.yml:1334-1337; test_web_ui_tab_set test_mcp_server_config.py:455-469 asserts tabs[2] is the camera tab. Gripper camera mount client.yml:1517-1528 and mcp_server/config.py:25. Grep for other references to tab id 'camera' (tests/test_web_ui_*.py, overlays config) and update them. Ordered after T7, T10, T16 and T26, which share these files. Settled: the arm tab takes the gripper camera tab slot (tabs[1]). The old tab's topic is kept as gripper_camera_topic.

**Value:** The Arm tab is deployed where the gripper camera tab was, with no config mismatch.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`(cd nodes/web_ui/frontend && npx vitest run src/tabSelection.test.ts && npm run lint && npx tsc --noEmit) && (cd nodes/web_ui && uv run pytest tests/test_config.py -q && uv run poe lint) && uv run pytest tests/test_mcp_server_config.py -q && bash -o pipefail -c 'uv run poe lint-ansible </dev/null 2>&1 | tail -2'` - the test summary line shows all passed and lint is clean.

### Task 29: armCamera: pure camera poses glued to the arm (side-left, side-right, above, gripper)

**Files:**
- Modify: `nodes/web_ui/frontend/src/map3d/armCamera.ts`
- Test: `nodes/web_ui/frontend/src/map3d/armCamera.test.ts`

**Interfaces:**
- Consumes: existing code only
- Produces: arm-camera-pose

**Ownership:**
- owns: arm-camera-pose
- uses: none
- dependsOn: none

**Requirement:**
Test first in nodes/web_ui/frontend/src/map3d/armCamera.test.ts, seen failing. `map3d/armCamera.ts` exports `type ArmCameraMode = 'side-left' | 'side-right' | 'above' | 'gripper'` and `armCameraPose(mode, shoulderWorld: Matrix4, gripperWorld: Matrix4, mount: {xyz, rpy}): {position: Vector3, target: Vector3, up: Vector3}`. (1) side-left, side-right and above are computed in shoulder_link (the frame after shoulder_pan): the test shows the camera pose relative to shoulder_link is invariant under a shoulder_pan change, and the arm stays at the expected screen side for side-left vs side-right. (2) gripper mode composes gripper_link with the mount (xyz, rpy as in client.yml) and looks along the camera optical axis; the test checks the pose against a hand-computed mount transform. (3) URDF-to-three rotation (coords.ts URDF_TO_THREE_ROTATION_X) is applied consistently. JSDoc with types. npm run lint clean.

Notes: D9b. URDF chain so101_arm.urdf: shoulder_pan (:434) -> shoulder_link; gripper_link; mount client.yml:1517-1528 (x -0.0046, y 0.0364, z -0.0364, rpy -1.5622/1.1950/-1.4466). Pure three.js maths, node environment.

**Value:** Arm views that stay put when the arm pans, so the user can judge the gripper from a stable angle.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/map3d/armCamera.test.ts && npm run lint` - the test summary line shows all passed and lint is clean.

### Task 30: Arm tab 3D view with arm-glued camera modes

**Files:**
- Modify: `nodes/web_ui/frontend/src/arm/armView.ts`
- Test: `nodes/web_ui/frontend/src/arm/armView.test.ts`
- Modify: `nodes/web_ui/frontend/src/arm/ArmView.tsx`
- Modify: `nodes/web_ui/frontend/src/components3d/InteractiveArm.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`

**Interfaces:**
- Consumes: arm-camera-pose, arm-tab-config, arm-tab-component
- Produces: arm-view

**Ownership:**
- owns: arm-view
- uses: arm-camera-pose, arm-tab-config, arm-tab-component
- dependsOn: T28, T29

**Requirement:**
Test first in nodes/web_ui/frontend/src/arm/armView.test.ts, seen failing. (1) Pure `arm/armView.ts` exports `applyArmCamera(camera, controlsTarget, robot, mode, mount)` that reads the shoulder_link and gripper_link world matrices from the loaded robot (findLink pattern) and sets camera position, up and target from armCameraPose; the test drives it with a stub robot and asserts the camera follows a joint change. (2) components3d/InteractiveArm.tsx exposes the solid arm RobotModel through an `onRobotLoaded` passthrough prop (today only the ghost gets one, :695, :705). (3) `arm/ArmView.tsx` is the Arm tab's own Canvas (frameloop 'demand') with the arm from arm_joint_states_topic and mode buttons side-left, side-right, above, gripper; the camera is updated in the same frame as setJointValue and invalidate() is called on joint updates; OrbitControls are disabled while a glued mode is active. (4) tabs/ArmTab.tsx renders ArmView in its 3D slot. (5) The map tab keeps only top-down and orbit (MapScene CameraRig unchanged). npm run lint and tsc clean.

Notes: D9b. Risks from the arm brief: useFrame priority > 0 disables auto-render (keep 0 or render explicitly); OrbitControls makeDefault fights a glued camera. RobotModel onRobotLoaded :56; findLink InteractiveArm.tsx:112-115.

**Value:** The user watches the arm from the side, above or the gripper while jogging it.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/arm/armView.test.ts src/map3d/armCamera.test.ts && npm run lint && npx tsc --noEmit` - the test summary line shows all passed and lint is clean.

### Task 31: mcp_server jog: cartesian steps via IK and joint steps, safety-checked and coalesced

**Files:**
- Modify: `nodes/mcp_server/mcp_server/jog.py`
- Modify: `nodes/mcp_server/mcp_server/grasp_tools.py`
- Modify: `nodes/mcp_server/mcp_server/config.py`
- Test: `nodes/mcp_server/tests/test_jog.py`
- Modify: `nodes/mcp_server/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: jog-action

**Ownership:**
- owns: jog-action
- uses: none
- dependsOn: T23
- model: opus

**Requirement:**
Test first in nodes/mcp_server/tests/test_jog.py (new), seen failing. (1) GraspServiceRequest gains action 'jog' with a `jog` spec (extra='forbid'): cartesian axis up, down, forward, back, left, right in the gripper frame; shoulder_pan left/right; wrist_pitch up/down; wrist_roll left/right; gripper open/close; with an optional step. (2) New `mcp_server/jog.py`: a cartesian step reads ArmState.tool_pose, adds the delta in the gripper frame and goes through solve_cartesian / move_cartesian, so the self-collision, floor guard and reach (IK unreachable) checks apply; joint steps go through move_joints with joint-limit checks. (3) Step sizes are capped by a new `jog` config section (max cartesian step m, max joint step rad, gripper step); a larger requested step is clamped to the cap. (4) A jog is refused with a reason while joints are stale, under the battery cut-off, when the target collides or is unreachable; the roll guard still refuses roll with the gripper open past its limit. (5) While another motion runs, jogs coalesce: the last request wins and earlier pending ones are dropped, never queued unbounded. (6) Each refusal and result is published on /grasp/result like other actions. (7) nodes/mcp_server/README.md Web-UI contract documents the jog action. Lint clean.

Notes: D9c. GraspServiceRequest grasp_tools.py:727-741, GraspService.handle :770-834; ArmController.solve_cartesian arm.py:1307, move_cartesian :1435, state().tool_pose :2287; SelfCollisionChecker arm.py:2518-2574; roll guard arm.py:1408; ArmBusyError. Jog IK belongs in mcp_server (measured space with joint offsets). Ordered after T23 (config.py, README). Settled defaults (config): cartesian step max 0.01 m, joint step max 0.05 rad, gripper step 0.1.

**Value:** The user can nudge the arm in small safe steps from the browser, with the same safety checks the agent gets.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/mcp_server && uv run pytest tests/test_jog.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 32: mcp_server take and release control services for the web UI

**Files:**
- Modify: `nodes/mcp_server/mcp_server/ros_iface.py`
- Modify: `nodes/mcp_server/mcp_server/control.py`
- Test: `nodes/mcp_server/tests/test_control.py`
- Modify: `nodes/mcp_server/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: arm-control-services

**Ownership:**
- owns: arm-control-services
- uses: none
- dependsOn: T31

**Requirement:**
Test first in nodes/mcp_server/tests/test_control.py (extend), seen failing. (1) ros_iface serves std_srvs/Trigger `/arm/take_control` and `/arm/release_control`. (2) take acquires the arm lease for holder 'web_ui' through the existing control-lease mechanism (active_source becomes autonomy, measured pose commanded); it is refused when the agent holds the lease and under the battery cut-off. (3) release publishes the autonomy release so the follower hands back to the leader (proximity 0.15 rad in filter_node applies) and clears the holder; check_orphan_lease does not drop a web_ui lease it owns. (4) /arm/home is reused unchanged (home then release). (5) get_control_state reports the web_ui holder. (6) nodes/mcp_server/README.md documents both services. Lint clean.

Notes: D9d. ArmController.acquire arm.py:973, release :985; control.py docstring :1-23; ros_iface.py:433-434 Trigger services, :910 check_orphan_lease, :942-970 handlers; arbitration filter_node/arbitration.py. Settled (my call): take_control is refused while the leader is actively moving (leader joint motion above a threshold in the last 0.5 s), with a clear message.

**Value:** The user takes the arm from the leader or the agent and hands it back with one button.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/mcp_server && uv run pytest tests/test_control.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 33: web_ui backend routes for jog, take control, release and home on the arm tab

**Files:**
- Modify: `nodes/web_ui/web_ui/server.py`
- Modify: `nodes/web_ui/web_ui/bridge.py`
- Modify: `nodes/web_ui/web_ui/config.py`
- Test: `nodes/web_ui/tests/test_arm_api.py`
- Modify: `nodes/web_ui/README.md`

**Interfaces:**
- Consumes: jog-action, arm-control-services, arm-tab-config
- Produces: arm-routes

**Ownership:**
- owns: arm-routes
- uses: jog-action, arm-control-services, arm-tab-config
- dependsOn: T28, T30, T31, T32

**Requirement:**
Test first in nodes/web_ui/tests/test_arm_api.py (new), seen failing. (1) server.py adds POST /api/arm/jog?tab= (forwards a jog request on /grasp/command), /api/arm/take_control and /api/arm/release (call the Trigger services), and /api/arm/home accepts an arm tab as well as map_nav (tab lookup generalised). (2) Every route is battery-gated (battery_block), returns 404 for an unknown or wrong-type tab, 503 without the bridge or when the service is unavailable, and uses action_response. (3) bridge.py adds 'jog' to the grasp motion actions and subscribes /filter/active_source (std_msgs/String) through a role so the frontend receives the active source. (4) config.py trigger_services() and the role list include the arm tab's services and active_source_topic. (5) nodes/web_ui/README.md documents the routes. Lint clean.

Notes: D9c, D9d. server.py:678 find_map_nav_tab, :765-787 call_arm_trigger and home routes, :828-875 grasp routes, :927 battery block; bridge.py:134 grasp actions, ROLE_SPECS :154; config.py:406-414 trigger_services.

**Value:** The browser can jog the arm and take or hand back control through checked backend routes.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui && uv run pytest tests/test_arm_api.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 34: Arm tab jog panel with Take control, Release and Home

**Files:**
- Modify: `nodes/web_ui/frontend/src/arm/armApi.ts`
- Modify: `nodes/web_ui/frontend/src/arm/ArmJogPanel.tsx`
- Test: `nodes/web_ui/frontend/src/arm/ArmJogPanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- Modify: `nodes/web_ui/README.md`

**Interfaces:**
- Consumes: arm-routes, overlay-panel, arm-view
- Produces: no shared part

**Ownership:**
- owns: none
- uses: arm-routes, overlay-panel, arm-view
- dependsOn: T1, T30, T33

**Requirement:**
Test first in nodes/web_ui/frontend/src/arm/ArmJogPanel.test.tsx (jsdom), seen failing. (1) `arm/armApi.ts` posts jog, take_control, release and home and never throws (returns {ok, message}, like poi/api.ts). (2) `arm/ArmJogPanel.tsx` has buttons for cartesian up, down, forward, back, left, right; shoulder-pan left/right; wrist pitch up/down; wrist roll left/right; gripper open/close; plus Take control, Release and Home (Home with the existing two-click confirm pattern). (3) The active source from /filter/active_source is shown; jog buttons are disabled unless the web UI holds control; Release is disabled when it does not. (4) Refusals and errors from the backend are shown to the user. (5) Buttons are at least TOUCH_TARGET_PX and the panel uses OverlayPanel on phone. (6) tabs/ArmTab.tsx renders the panel. (7) nodes/web_ui/README.md documents the Arm tab (camera modes and jog panel). The test covers each api call, the disabled states and an error message. npm test, npm run lint and npm run build green.

Notes: D9c, D9d. Arm home buttons today MapNavTab.tsx:657-675 (two-click confirm pattern); map tab arm controls stay. Once the Arm tab holds the lease, map-tab InteractiveArm drags are ignored by filter_node: say so in the README.

**Value:** The user drives the arm in small steps from any device and sees who is in control.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/arm/ArmJogPanel.test.tsx && npm test && npm run lint && npm run build` - the test summary line shows all passed and lint is clean.

### Task 35: leader_bridge node: relays only /leader/joint_states to /filter/input_joint_updates

**Files:**
- Modify: `nodes/leader_bridge/pyproject.toml`
- Modify: `nodes/leader_bridge/uv.lock`
- Modify: `nodes/leader_bridge/leader_bridge/__main__.py`
- Modify: `nodes/leader_bridge/leader_bridge/bridge.py`
- Modify: `nodes/leader_bridge/leader_bridge/config.py`
- Test: `nodes/leader_bridge/tests/test_bridge.py`
- Modify: `nodes/leader_bridge/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: leader-bridge

**Ownership:**
- owns: leader-bridge
- uses: none
- dependsOn: none
- size: large
- sizeReason: A new node scaffold (pyproject, lock, package, tests) cannot be split from its first test.
- model: opus

**Requirement:**
Test first in nodes/leader_bridge/tests/test_bridge.py (new), seen failing. (1) A new purpose-built node nodes/leader_bridge relays /leader/joint_states (server) to /filter/input_joint_updates (client) and nothing else: the test asserts no other topic is subscribed or published. (2) Transport is the builder's choice (a minimal DDS relay or the existing ros2-master discovery), documented in the README with the reason. (3) Messages are forwarded unchanged (names, positions, stamp); stale or malformed messages are dropped and logged, never replaced with placeholder data. (4) Config via pydantic from a YAML file (topics, QoS, staleness); type hints and docstrings. (5) The filter_node proximity hand-back (takeover_threshold_rad 0.15) is respected: the bridge does not bypass arbitration. (6) numpy is in the node's dependencies if sensor_msgs needs it. (7) nodes/leader_bridge/README.md documents it. uv run poe lint clean.

Notes: D9e. master2master (client.yml:650-673, enabled: false since 2026-10-08) carried /leader/joint_states -> /filter/input_joint_updates as JointState among other topics; reuse its proxy pattern (nodes/master2master/master2master/proxy.py) if it fits. Follow an existing node's pyproject layout (ruff pinned in mise.toml, poe lint). Settled: nodes/leader_bridge does not exist; scaffold it (pyproject, uv.lock, package, tests, README). Registration also touches ansible/group_vars/all.yml, ansible/playbooks/deploy_nodes_client.yml and deploy_nodes_server.yml, scripts/lint-all-nodes.sh, scripts/deploy-node.sh, tests/test_dds_discovery_config.py, tests/test_unit_cgroups.py, nodes/README.md and the CLAUDE.md node-directory count. Split registration into T36 where the files allow.

**Value:** Leader teleop works again without re-enabling the broad master2master relay.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/leader_bridge && uv run pytest tests/test_bridge.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 36: Deploy leader_bridge through Ansible on both hosts

**Files:**
- Modify: `CLAUDE.md`
- Modify: `ansible/group_vars/all.yml`
- Modify: `ansible/group_vars/client.yml`
- Modify: `ansible/group_vars/server.yml`
- Modify: `ansible/playbooks/deploy_nodes_client.yml`
- Modify: `ansible/playbooks/deploy_nodes_server.yml`
- Modify: `nodes/README.md`
- Modify: `pyproject.toml`
- Modify: `scripts/lint-all-nodes.sh`
- Modify: `tests/README.md`
- Test: `tests/test_dds_discovery_config.py`
- Test: `tests/test_leader_bridge_config.py`
- Test: `tests/test_unit_cgroups.py`

**Interfaces:**
- Consumes: leader-bridge
- Produces: no shared part

**Ownership:**
- owns: none
- uses: leader-bridge
- dependsOn: T28, T35
- size: large
- sizeReason: Registering a new node touches every per-node list in Ansible, scripts and the root tests together.

**Requirement:**
Test first in tests/test_leader_bridge_config.py (new), seen failing: the node type and ros2_nodes entry for leader_bridge exist in ansible/group_vars/client.yml (and ansible/group_vars/server.yml where the chosen transport needs a server side), with present/enabled/config following the ros2_nodes schema; the deployed config validates against the node's pydantic config; master2master stays disabled. (1) group_vars carry the entries. (2) The root pyproject.toml lint-nodes task covers nodes/leader_bridge (tests/test_node_lint_coverage.py stays green). (3) nodes/README.md lists the node; tests/README.md documents the new test file. (4) uv run poe lint-ansible and uv run poe test-ansible pass. (5) Post-merge robot verification, without motion: after deploy, joint_states flow on /filter/input_joint_updates is observed with topic_scraper_api plus scripts/topic_scraper_collect.py from both hosts, and the follower stays unpowered or holding.

Notes: D9e. Ordered after T28 (client.yml, tests/README.md). Never run ansible-playbook directly; deploy with scripts/deploy-nodes.sh after merge.

**Value:** The leader arm drives the follower again after a normal deploy.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`uv run pytest tests/test_leader_bridge_config.py tests/test_node_lint_coverage.py -q && bash -o pipefail -c 'uv run poe test-ansible </dev/null 2>&1 | tail -2'` - the test summary line shows all passed and lint is clean.

### Task 37: Per-task time budget and generous caps in claude_agent

**Files:**
- Modify: `nodes/claude_agent/claude_agent/config.py`
- Modify: `nodes/claude_agent/claude_agent/plan_tools.py`
- Modify: `nodes/claude_agent/claude_agent/plan_tree.py`
- Modify: `nodes/claude_agent/claude_agent/runner.py`
- Modify: `nodes/claude_agent/claude_agent/tools.py`
- Test: `nodes/claude_agent/tests/test_time_budget.py`
- Test: `nodes/claude_agent/tests/test_plan_tree.py`
- Modify: `nodes/claude_agent/README.md`

**Interfaces:**
- Consumes: existing code only
- Produces: plan-time-budget

**Ownership:**
- owns: plan-time-budget
- uses: none
- dependsOn: T25

**Requirement:**
Test first in nodes/claude_agent/tests/test_time_budget.py (new), seen failing; tests/test_plan_tree.py is updated where it pins the once-per-phase raise. (1) set_task_plan requires `time_budget_min` (> 0) for the whole task; a value above the new config ceiling `max_instruction_time_min` (default 120) is refused with a reason. (2) Before a plan exists the 900 s pre-plan watchdog (instruction_timeout_s) applies; once the plan is set the runner's watchdog fires at the declared budget measured from the instruction start, not at 900 s. (3) raise_phase_budget and revise_plan accept `time_budget_min` plus a required rationale and extend the watchdog deadline, never past the ceiling. (4) raise_phase_budget may be called more than once per phase, within max_phase_rw_cap / max_phase_turn_cap and the instruction maxima. (5) instruction_time_left and tools.time_budget_denial count min_effector_time_left_s against the declared budget. (6) Config defaults become max_phase_rw_cap 100, max_phase_turn_cap 100, max_rw_cap 400, max_turn_cap 500. (7) The plan event carries time_budget_min and the deadline (seconds left at emit time plus the emit timestamp) so the web UI can show it. (8) nodes/claude_agent/README.md documents the time budget and the new defaults. The test covers pre-plan watchdog, plan watchdog, a raise extending it, the ceiling, repeated raises and the effector gate. Lint clean.

Notes: D10a, D10b. config.py:190-193 caps, :221-222 instruction_timeout_s / min_effector_time_left_s; tools.py:282 time_budget_denial; runner.py:334, :378 instruction_time_left, :613 plan event, :943-966 watchdog (asyncio.timeout: reschedule() moves the deadline); plan_tools.py:183 schema, :424 set_task_plan, :461 raise_phase_budget; plan_tree.py:70, :95 `raised` one-raise flag. Ordered after T25 (runner.py, config.py, README.md chain T19, T21, T24, T25).

**Value:** Jobs the agent can finish are no longer cut off by a fixed 15-minute timer or tight caps.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/claude_agent && uv run pytest tests/test_time_budget.py tests/test_plan_tree.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 38: Prompt planning section: budgets at about twice the honest estimate

**Files:**
- Modify: `nodes/claude_agent/claude_agent/prompt.py`
- Test: `nodes/claude_agent/tests/test_prompt.py`

**Interfaces:**
- Consumes: plan-time-budget
- Produces: no shared part

**Ownership:**
- owns: none
- uses: plan-time-budget
- dependsOn: T21, T37

**Requirement:**
Test first in nodes/claude_agent/tests/test_prompt.py (extend), seen failing. The planning section of prompt.py (lines ~52-80) tells the agent to (1) set time_budget_min and the caps at about twice its honest estimate, (2) prefer overshooting and delivering to undershooting and stopping, and (3) raise budgets early with raise_phase_budget rather than end unfinished; it names time_budget_min as required in set_task_plan. The test asserts each statement. Lint clean.

Notes: D10c. Ordered after T21 (prompt.py, test_prompt.py) and T37 (tool field name).

**Value:** The agent plans with headroom instead of running out of time mid-job.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/claude_agent && uv run pytest tests/test_prompt.py -q && uv run poe lint` - the test summary line shows all passed and lint is clean.

### Task 39: Ansible client.yml carries the generous caps and the time ceiling

**Files:**
- Modify: `ansible/group_vars/client.yml`
- Test: `tests/test_claude_agent_config.py`
- Modify: `tests/README.md`

**Interfaces:**
- Consumes: plan-time-budget
- Produces: no shared part

**Ownership:**
- owns: none
- uses: plan-time-budget
- dependsOn: T26, T36, T37

**Requirement:**
Test first in tests/test_claude_agent_config.py (extend), seen failing: the deployed claude_agent config sets max_rw_cap 400, max_turn_cap 500, max_phase_rw_cap 100, max_phase_turn_cap 100 and max_instruction_time_min 120, keeps instruction_timeout_s as the pre-plan watchdog, and validates against ClaudeAgentConfig. (1) ansible/group_vars/client.yml (~1944-1950, ~1992) is updated. (2) tests/README.md documents the changed test. (3) Ansible lint passes.

Notes: D10b. Ordered after T36 (last task on client.yml and tests/README.md) and T26 (test_claude_agent_config.py). Deploying claude_agent kills a running agent session: check it is idle first.

**Value:** The robot actually runs with the generous limits after deploy.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`uv run pytest tests/test_claude_agent_config.py -q && bash -o pipefail -c 'uv run poe lint-ansible </dev/null 2>&1 | tail -2'` - the test summary line shows all passed and lint is clean.

### Task 40: Web UI job header shows the time budget and time left

**Files:**
- Modify: `nodes/web_ui/frontend/src/agent/agentModel.ts`
- Test: `nodes/web_ui/frontend/src/agent/agentModel.test.ts`
- Modify: `nodes/web_ui/frontend/src/agent/PhasePanel.tsx`

**Interfaces:**
- Consumes: plan-time-budget
- Produces: no shared part

**Ownership:**
- owns: none
- uses: plan-time-budget
- dependsOn: T9, T37

**Requirement:**
Test first in nodes/web_ui/frontend/src/agent/agentModel.test.ts (extend), seen failing. (1) agentModel.ts reads time_budget_min and the deadline from the plan event (and from later raises) and exports `timeBudget(state, now): {budgetMin, leftS} | null` (null before a plan, never a placeholder value); the test covers no plan, a plan, a raise and an expired budget (leftS 0). (2) agent/PhasePanel.tsx shows the declared budget and the time left in the header chip next to the phase, updating at least once per second while the job runs. (3) The data-testid hooks phase-panel, phase-row-N and active-phase stay. npm run lint clean.

Notes: D10a. The header is the T9 phase dropdown. Plan event contract from T37.

**Value:** The user sees how much time the agent gave itself and how much is left.

**Steps:**
- [ ] Write the test.
- [ ] Run it. Expected: FAIL for the stated reason.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint.
- [ ] Commit: `type(scope): message`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/agent/agentModel.test.ts src/agent/PhasePanel.test.tsx && npm run lint` - the test summary line shows all passed and lint is clean.

## Execution

- Run name: `web-ui-poi-responsive`.
- Model per task: sonnet unless a task states otherwise; opus for T8 (focus across an unmount plus an interactive scene popup), T22 (event sink fan-out plus auto_poi wiring), T31 (mcp_server jog through IK and the safety checks) and T35 (leader-teleop bridge).
- Lenses run: research-code-internal (codebase-cartographer briefs docs/plans/2026-10-12-web-ui-poi-responsive-briefs/brief.md and docs/plans/2026-10-12-web-ui-poi-responsive-briefs/arm-brief.md) and agent POI research (docs/plans/2026-10-12-web-ui-poi-responsive-briefs/agent-poi-brief.md); research-code-external, research-papers and research-docs skipped (see the design doc's Research table).
- Post-merge deploy (scripts/deploy-nodes.sh only, output to .logs/): client web_ui, mcp_server, claude_agent, leader_bridge (and claude_agent again for T37-T39) and the server side of leader_bridge if any. Deploying mcp_server or claude_agent kills a running agent session: check it is idle first.
- Post-merge robot verification: arm motion is gripper only, at minimal amplitude (one small open/close jog from the Arm tab), with no base motion and no other joints; the leader bridge is verified without motion (topic_scraper_api plus scripts/topic_scraper_collect.py on both hosts shows /filter/input_joint_updates flowing, the follower stays unpowered or holding).
- Lint: `node /Users/szymonri/Documents/claude-infra-plugin/workflows/validate-tasks.mjs docs/plans/2026-10-12-web-ui-poi-responsive-tasks.json --plan docs/plans/2026-10-12-web-ui-poi-responsive-plan.md --approvals docs/plans/2026-10-12-web-ui-poi-responsive-approvals.json --design docs/plans/2026-10-12-web-ui-poi-responsive-design.md`
- Tasks JSON: `docs/plans/2026-10-12-web-ui-poi-responsive-tasks.json`.
- Approvals: `docs/plans/2026-10-12-web-ui-poi-responsive-approvals.json`.
