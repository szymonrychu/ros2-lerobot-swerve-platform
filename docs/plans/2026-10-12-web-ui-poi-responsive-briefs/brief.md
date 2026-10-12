# web_ui frontend map (read-only recon, 2026-10-12)

Root: nodes/web_ui/frontend (Vite 5 + React 18 + TS + MUI 6 + @react-three/fiber/drei + three 0.167).
Paths below are relative to nodes/web_ui/frontend/src unless noted. graphify graph exists at repo root (not queried; direct reads were cheaper for one subtree). No LSP used.

## 1. Entry points, tabs, panels
- main.tsx -> App.tsx (ThemeProvider + CssBaseline from theme.ts).
- App.tsx:46-51 lazy tab imports; TAB_ICONS App.tsx:60-67; renderTab switch App.tsx:77-100 (`imu_orientation` -> ImuOrientationTab at :92).
- Tab list comes from backend GET /api/config (App.tsx:121-130). Order/filter: tabSelection.ts:11 SUPPORTED_TAB_TYPES, orderTabs (map_nav first), initialTabIndex, localStorage key `web_ui.activeTabId` (tabSelection.ts:8).
- Shell: AppBar with MUI Tabs (scrollable) App.tsx:209-243; on `narrow` (theme.breakpoints.down('sm'), App.tsx:107) a hamburger Drawer replaces tab icons (App.tsx:252-265, width 260 / 85vw). main Box App.tsx:267 (flex:1, overflow hidden, position relative). OverlayBar bottom App.tsx:305-310.
- Only the active tab is mounted (renderTab of `tabs[activeTab]`, App.tsx:276). Switching tabs UNMOUNTS MapNavTab and its MapScene/controllerRef.
- IMU tab: tabs/ImuOrientationTab.tsx (124 lines, own R3F Canvas). Configured as tab id `imu_graphs`, type `imu_orientation`, label "IMU" in:
  - nodes/web_ui/config/default.yaml:63-67
  - ansible/group_vars/client.yml:1339-1344 (deployed config)
  - backend whitelist nodes/web_ui/web_ui/config.py:30-39 VALID_TAB_TYPES (unknown types are REJECTED by backend validation)
  - tests that pin it: tests/test_mcp_server_config.py:469 (root, `tabs[4]["type"] == "imu_orientation"`), nodes/web_ui/tests/test_map3d_backend.py:242,251, frontend tabSelection.test.ts:14,112, README nodes/web_ui/README.md:15,17.
  - Note: `valid_imu_orientation` in web_ui/gps_anchor.py/bridge.py is the GPS compass, unrelated to the tab; keep it.

## 2. POI data flow
- Topics: backend subscribes by role, bridge.py:163-165 (`poi_list`, `poi_photos`, both latched MAP_SUB_QOS). Defaults config.py:70-76 (/poi/list, /poi/command, /poi/result, /poi/photos). TabConfig fields types.ts:50-52.
- Frontend store: no global store; `topicData` from useRosBridge (hooks/useRosBridge.ts) in App, passed as prop. POI state lives in hook poi/usePoiEditor.ts (parsePoiList of topicData[listTopic], :76-79; selection, drag, pending override, `focus` :156-161 = select + controllerRef.centerOn). Hook is instantiated inside MapNavTab.tsx:253-263, so POI state is map-tab-local today.
- Commands: POST /api/poi?tab=<id> via poi/api.ts:12 (postPoi); backend server.py:791.
- Types: poi/types.ts (Poi, kinds point|area|object|photo; photo never on /poi/list :5-6; `poi_ids` links).
- Photos: separate REST, not on /poi/list. photos/api.ts (Photo :24-37 has x,y nullable + view {x,y,yaw,pitch,hfov}; fetchPhotos :174, deletePhotos :197, thumb/image urls :102,113). /poi/photos summary only carries `revision`; MapNavTab.tsx:265-266 reads it and passes to usePhotoLayer (photos/usePhotoLayer.ts, refetch on revision/age) and PhotoList.
- Existing list UIs (all overlaid on the map, MapNavTab.tsx:766-816, left column width {xs: full, sm: 300}):
  - PoiListPanel poi/PoiPanels.tsx:32-104 (collapsible, sortPois, row click -> onFocus; Chip by status colour). Non-photo POIs only.
  - PoiEditorPanel poi/PoiPanels.tsx:107+.
  - PhotoList photos/PhotoList.tsx (separate paginated list, own fetch, Details button -> onSelect).
  - PhotoPanel photos/PhotoPanel.tsx (detail/editor of picked photo; `openPhoto` MapNavTab.tsx:269-275).
  => "photos in the same list as POIs" means merging Poi[] (topic) with Photo[] (REST, paged) - two sources, two shapes.

## 3. Map rendering (Three.js via R3F, NOT 2D canvas/SVG)
- MapScene map3d/MapScene.tsx: Canvas frameloop="demand", dpr [1,2] (:382-392). Layers placed at :350-351.
- POIs: poi/PoiLayer.tsx - PointMarker disc+ring (:55-84), ObjectMarker diamond (:87-111), AreaMarker polygon+outline+handles (:113-152), labels via drei `<Html>` PoiLabel (:27-53, text tag 'obj'/'A'/'U', no icon). Colours/styles poi/style.ts (STATUS_COLORS, OBJECT_COLOR, CREATOR_OUTLINE, ZONE_COLORS, poiStyle, zoneStyle).
- Photo wedges: photos/PhotoLayer.tsx (ShapeGeometry wedge, click -> onSelect, :16-48), geometry photos/geometry.ts (photoWedge :34, cameraColor :71, WEDGE_LENGTH_M :5).
- Layer toggles: map3d/layers.ts (LayerKey incl `pois`, `photos`); swatches MapNavTab.tsx:90-99; groups :109-114.
- Popup support: none as a component. Only precedent = drei `<Html>` anchored at a map point (PoiLabel; RTK base label map3d/sceneLayers.tsx:412-432), pointerEvents none. An interactive popup would be a new `<Html>` in the scene (pointerEvents auto, frameloop demand needs invalidate) or a DOM overlay positioned by projecting the point (no project helper exists; controller only has pick).
- PICTOGRAMS: none exist. Repo-wide grep for pictogram/glyph/kind icon found nothing; only svg is components/RobotIcon.tsx:25. KIND_TITLES (text) at PoiPanels.tsx:30. MUI icons used for toolbar: AddLocationAlt (point), Polyline (area) MapNavTab.tsx:39-40. Planned change 2 assumes something that is not there - needs a decision (MUI icons per kind? new SVG set?).

## 4. Map-centering API (shared)
- SceneController map3d/MapScene.tsx:51-58: `pick(clientX, clientY)`, `centerOn(p: Vec2)` (keeps camera offset, :168-177), `fit(b: Bounds)` (:178-196). Held in `controllerRef` created in MapNavTab.tsx:154; null when MapNavTab is unmounted.
- Callers: centerOnRobot MapNavTab.tsx:464, fit :465-471, initial fit poll :216-225 (setInterval waiting for controller), usePoiEditor.focus.

## 5. Agent job / plan tree
- tabs/AgentChatTab.tsx: PhasePanel :183-191 (always-open Box, maxHeight 220, overflowY auto, borderBottom) rendered at :415 between header chips row (:397-414) and transcript. Rows PhaseRowView :154-180 (status chip, name, PhaseProgress, goal, sensor count, UsageBarView bars minWidth 88 :145-151, summary).
- Header also shows active-phase chip + PhaseProgress (:401-402) - natural dropdown trigger/summary.
- PlanCard (:193-213) / PhaseStarted/Completed inline transcript items (:216-240) are separate.
- Model: agent/agentModel.ts phaseRows :618, PhaseRow :173, headerStatus :642. PhaseProgress agent/PhaseProgress.tsx (chips). Tests: agent/PhaseProgress.test.tsx, agentModel.test.ts (data-testid `phase-panel`, `phase-row-N`, `active-phase` are test hooks).

## 6. Styling / layout system
- MUI `sx` props + one theme (theme.ts); default MUI breakpoints (xs 0, sm 600, md 900, lg 1200, xl 1536), only `sm` used anywhere. No md/lg handling -> tablet = desktop layout.
- index.css: html/body/#root 100vh then 100dvh, overflow hidden, #root flex column (index.css:4-15).
- useMediaQuery(down('sm')): App.tsx:107, MapNavTab.tsx:175, OverlayBar.tsx:32.
- theme constants: TOUCH_TARGET_PX 44, MuiTab minHeight 48, Toolbar minHeight 48 (App.tsx:210).
- Hard-coded layout sizes:
  - App.tsx:210 Toolbar minHeight 48; :238 Tab minWidth {xs 72, sm 90}; :255 Drawer list width 260 / maxWidth 85vw.
  - MapNavTab.tsx:484 toolbar Paper (wrapping Stack of ~10 button groups, no overflow menu - wraps to several rows on phones); :494,:506 STOP px 3; :648 menu item 44; :760 waiting text top/left 8; :767-779 POI column top/left 8, width {xs calc(100%-16px) or 0, sm 300}, display none on xs when layers panel or grasp open; :828 grasp alert; :838 GraspPanel bottom/left 8 width {xs calc(100%-16px), sm 360}; :846 layers Collapse top/right 8 width {xs calc(100%-16px), sm 260}; :851 maxHeight {xs 55vh, sm 70vh}; :864 row minHeight 40; :141 swatch 14; :953 legend line 22px.
  - Panel defaults: narrow -> layers panel and POI list start closed (MapNavTab.tsx:176-179), layers closes when a POI is selected on phone (:284-286), grasp open closes others (:322-326).
  - poi/PoiPanels.tsx:67 header minHeight 40; :78 list maxHeight {xs 28vh, sm 40vh}; :85 row 48; :165 editor maxHeight {xs 60vh, sm 75vh}; :253,256 minHeight 44.
  - photos/PhotoList.tsx:16 THUMB_WIDTH_PX 72; :161 list maxHeight {xs 24vh, sm 32vh}; :205,:209 TextField width 130.
  - photos/PhotoPanel.tsx:19 PANEL_IMAGE_PX 640; :98 links maxHeight 130.
  - grasp/GraspPanel.tsx:58 field flex 1 1 90px; :134 maxHeight {xs 62vh, sm 70vh}.
  - tabs/AgentChatTab.tsx:66-68 STICK_THRESHOLD_PX 48, RESULT_MAX_HEIGHT_PX 220, THUMB_MAX_PX 120; :185 PhasePanel maxHeight 220; :251 bubble maxWidth 85%; :407,:410 buttons minHeight 36.
  - tabs/RgbdCameraTab.tsx:86 preview height {xs 40%, sm 25%}, minHeight 120. ImuOrientationTab.tsx:110 legend bottom/left 12.
  - components/GpsChip.tsx:42 maxWidth {xs 120}. OverlayBar.tsx:44,54 spacing.
  - PANEL_BG 'rgba(22, 27, 34, 0.92)' duplicated in PoiPanels.tsx:28, PhotoList.tsx:15, PhotoPanel.tsx:17, MapNavTab.tsx:851 (GraspPanel 0.94).
- Map canvas sizing: MapNavTab root column flex (:480), canvas container flex:1 minHeight 0 relative (:681-693); R3F Canvas fills it (auto-resize via ResizeObserver). All panels are absolutely positioned overlays ON the canvas; nothing docks beside it.
- Panels stop pointer propagation (`onPointerDown stopPropagation`) so map gestures do not fire - new overlays must do the same; grasp uses `data-grasp-panel` attr check (MapNavTab.tsx:409).

## 7. Verification (verbatim)
Frontend (nodes/web_ui/frontend/package.json scripts):
- `npm test` -> "vitest run"  (baseline today: Test Files 38 passed (38), Tests 424 passed (424))
- `npm run lint` -> "eslint src --ext ts,tsx --max-warnings 0"  (baseline: clean)
- `npm run build` -> "tsc && vite build"  (baseline `npx tsc --noEmit` exit 0; vite build not run)
- vitest env default node (vite.config.ts test.environment 'node'); component tests opt in with first line `// @vitest-environment jsdom` + @testing-library/react (e.g. photos/PhotoList.test.tsx:1).
Backend (nodes/web_ui/pyproject.toml [tool.poe.tasks]):
- `cd nodes/web_ui && uv run poe lint` -> ruff check web_ui tests; ruff format --check web_ui tests; vulture web_ui tests --min-confidence 80
- `cd nodes/web_ui && uv run poe test` -> pytest tests/ -v   (not run in recon)
Root: `uv run pytest tests/ -v` (covers tests/test_mcp_server_config.py), `uv run poe lint-ansible`, `uv run poe test-ansible` (client.yml change).
Deploy: `./scripts/deploy-nodes.sh client web_ui 2>&1 | tee .logs/deploy-client.log` (ansible role builds frontend: ansible/roles/ros2_node_deploy/tasks/main.yml:232-237, npm ci + npm run build on robot).
Git: main @ 94375c2, tree clean; many local worktrees under .claude/worktrees (e1/e3/e5/e9-*) and .worktrees/rgbd-*; none inspected.

## 8. Conventions
- One component/concern per folder (poi/, photos/, grasp/, agent/, map3d/); pure logic in .ts with sibling .test.ts; components export named functions, tabs default-export.
- Constants SCREAMING_SNAKE at top; module docblock comment first line; JSDoc @param/@returns on helpers.
- Errors: fetch helpers never throw, return {ok,message} (poi/api.ts:12-29). Topics: no placeholder data.
- localStorage helpers take TabStorage stand-in (tabSelection.ts, layers.ts readLayerState/writeLayerState).
- README sections must be updated (nodes/web_ui/README.md tab table :15, tests/README.md if root tests change).

## 9. Blast radius per planned change
1. IMU -> POI tab: App.tsx (lazy import, TAB_ICONS, renderTab), tabSelection.ts:11 (+test), backend config.py:30-39 VALID_TAB_TYPES (+ new type name), default.yaml:63-67, ansible client.yml:1339-1344, tests/test_mcp_server_config.py:469, nodes/web_ui/tests/test_map3d_backend.py:242,251, README.md:15,17. Delete ImuOrientationTab.tsx (check nothing else imports it: only App.tsx). The POI tab needs POI list + photo data + the map; since only one tab mounts, "button centres map and opens popup" requires either (a) lifting usePoiEditor/topic parsing + a pending "focus target" to App and switching activeTab to map then calling centerOn once controllerRef appears (pattern: initial-fit poll MapNavTab.tsx:216-225), or (b) keeping MapNavTab mounted (hidden) across tabs. Decision needed.
2. Pictograms on map: PoiLayer.tsx markers / PoiLabel; PhotoLayer.tsx wedges. Pictogram source does not exist (see 3).
3. Plan dropdown: AgentChatTab.tsx PhasePanel :183-191 and header :397-415; tests keyed on data-testid.
4. Responsive: App.tsx shell, MapNavTab.tsx toolbar + overlay column (largest surface), PoiPanels, PhotoList/Panel, GraspPanel, AgentChatTab, OverlayBar, theme.ts (breakpoints if customised), index.css.

## 10. Risks
- POI tab vs map tab mount model (above) - the core architectural decision.
- Photos and POIs are different shapes/sources (Poi from latched topic, Photo from paged REST; Photo.x/y nullable, unlocated photos cannot be centred). PhotoList paging (PHOTO_LIMIT 10) vs usePhotoLayer (up to 1000, age-filtered) - two fetch paths already.
- frameloop="demand": any new Html popup/marker must call invalidate (PoiLayer.tsx:166-167 pattern) or it will not render/update.
- drei Html per POI is a DOM node per POI; with many photos (up to 1000) Html pictograms are costly - sprites/textures may be needed.
- Gesture ownership: goal/POI-add/grasp share one-finger input (MapNavTab.tsx:287-305); a popup must stopPropagation like other panels.
- Backend rejects unknown tab types: frontend, backend, default.yaml and client.yml must change together or the deployed UI fails config load. Root test pins tab index 4.
- Tablet: only `sm` breakpoint is used; md handling is new work.
- Deploy of web_ui is safe re agent sessions (only claude_agent/mcp_server deploys kill sessions per memory).

## 11. Shared parts to reuse
- SceneController.centerOn / fit / pick (map3d/MapScene.tsx:51-58) and usePoiEditor.focus.
- rosToThree (map3d/coords.ts), drei Html label pattern (PoiLayer.tsx:27-53, sceneLayers.tsx:412-432).
- poi/style.ts: poiStyle, zoneStyle, zoneLabel, STATUS_COLORS, OBJECT_COLOR, sortPois, formatUpdated, objectDetails; PoiPanels KIND_TITLES :30.
- photos: cameraColor, photoWedge, capturedAt, photoThumbUrl, fetchPhotos, usePhotoLayer.
- PoiListPanel / PoiEditorPanel / PhotoPanel / PhotoList components.
- theme.ts TOUCH_TARGET_PX, CANVAS_BG, MONO_FONT; MUI useMediaQuery(theme.breakpoints...).
- MUI Collapse/Menu/Popover already imported widely (Menu used for grasp dropdown MapNavTab.tsx:642 - a template for the plan dropdown).

## Unknowns
- What "existing pictograms" refers to - none on main, in any .claude/worktrees or .worktrees frontend, or in any commit message (git log --all --grep=pictogram). Settle: ask the user.
- Whether the POI tab should replace or also keep the map overlay POI list.
- Backend pytest / poe lint baseline not run.
