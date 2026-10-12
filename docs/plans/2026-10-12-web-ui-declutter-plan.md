# Web UI declutter Implementation Plan

> For agentic workers: execute through `workbench:plan-execute` with
> `docs/plans/2026-10-12-web-ui-declutter-tasks.json` as `args.tasks`. Each task is test-first: the test is written and
> seen failing before the code that makes it pass. Steps use checkbox syntax.

**Goal:** The web UI becomes utilitarian: one neutral dark token set with a white accent and colour only for warning and error, a map where only zones and nav paths are coloured and POI kinds differ by shape, a docked sidebar on tablet and desktop with no map popup, one status line, STOP plus STOP GRASP in the toolbar only, a flat agent log, one confirm pattern, and a guard test that keeps it that way.

**Architecture:** All work is in `nodes/web_ui/frontend/src`. T1 sets the tokens in theme.ts; T2 adds `designViolations()` in layout/designRules.ts, which every later task runs on its own files and T20 runs on every source. Shared primitives are built once: map3d/palette.ts (T3), MapLabel (T4), KIND_SHAPE (T5), OverlayPanel chrome and ConfirmDialog (T6), MapSidebar (T7), StatusItem (T9), StateBanner (T10), ToolButton and MapStatusLine (T11). Unchanged: robot behaviour, safety logic, stop handlers, backend APIs, R3F frameloop "demand", map gestures.

**Tech stack:** TypeScript, React 18, MUI 6 (`sx`, theme overrides), @react-three/fiber and drei, vitest. Per task: `cd nodes/web_ui/frontend && npx vitest run <test files> && npm run lint && npx tsc --noEmit`; final: `npx vitest run && npm run lint && npx tsc --noEmit && npm run build`.

**Spec:** `docs/plans/2026-10-12-web-ui-declutter-design.md` (executors read both: the design doc for the why and the decisions, this plan for the what).

## Project brief

What: a declutter and deslopify pass over the web UI, in the style of Grafana without its colour detail.
How: one theme token set, one map palette, shared StatusItem, ToolButton, StateBanner, ConfirmDialog and MapSidebar primitives, and a design-rules guard test.
Why: the operator drives the robot from phone, tablet and desktop; repeated status, 25 hues and panels covering the map hide what matters, including the stop buttons.

## Global constraints

- Tests first: each task writes its test, sees it fail for the stated reason, then implements. Component tests start with `// @vitest-environment jsdom`.
- Round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing on main. Every implementer re-reads its files on current main at start. Arm-tab files (arm/*.tsx, tabs/ArmTab.tsx, components3d/InteractiveArm.tsx) and round-3 camera files (components/CameraStream.tsx, tabs/CameraTab.tsx) are only in the late tasks T18 and T19.
- Shared files are chained: tabs/MapNavTab.tsx T3 -> T7 -> T8 -> T11 -> T12 -> T13; tabs/AgentChatTab.tsx T10 -> T15 -> T16; App.tsx only in T10; map3d/palette.ts T3 -> T17 -> T18.
- Colour: the accent is neutral white or light grey; healthy is neutral text, never green; only error and warning carry hue, plus the keepout and slow zones and the nav path on the map.
- Font weights 400 and 500 only (STOP included). Radius 2, the 8 px grid, 44 px targets on a coarse pointer and 32 px on a fine one. Opaque panels, no blur, shadow or gradient.
- Writing: no em dashes, ellipsis characters, smart quotes, arrows, emoji or decorative Unicode, in code copy and in docs.
- No new UI library. No change to robot behaviour, safety logic, stop handlers or backend APIs. No light theme.
- Previous-round tests (layout classes, OverlayPanel, PhasePanel test ids, the daa1be6 touch test) are updated, not deleted, where behaviour moves.
- Model: sonnet per task; opus only for T7 (docked sidebar plus map resize under frameloop demand).
- Deploy after the run: web_ui only, `scripts/deploy-nodes.sh client web_ui 2>&1 | tee .logs/deploy-client.log`, through the ansible-deploy skill.

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| declutter-brief P1 | theme tokens and MUI mapping, adapted: white accent, no ok green, opaque PANEL_BG, pointer-split targets | T1 |
| declutter-brief P2 | design rules guard, adapted: per-task designViolations() calls instead of a shared GUARDED_FILES list; weights 400 and 500 only | T2, T20 |
| declutter-brief P3 | one map palette, adapted to D2: zones and nav paths coloured, kinds and photos greyscale, accent selection | T3, T4 |
| declutter-brief P4 | MapLabel; MapPopup restyle rejected because D3 removes MapPopup | T4, T8 |
| declutter-brief P5 | flat Panel chrome; docked sidebar mode split out | T6, T7 |
| declutter-brief P6 | StatusItem and status bar, ok neutral | T9 |
| declutter-brief P7 | StateBanner, no tilt toast, composer caption, tab bar without icons | T10 |
| declutter-brief P8 | icon-only toolbar, STOP and STOP GRASP in the toolbar only, mode hint status line | T11 |
| declutter-brief P9 | Layers panel extraction and declutter, pose into the status line | T12 |
| declutter-brief P10 | grasp panel (no panel stops, one hint, compact result) and POI and photo panels (one confirm pattern) | T13, T14 |
| declutter-brief P11 | agent header (state, phase dropdown, time left) and flat chat log | T15, T16 |
| declutter-brief P12 | PoiTab, glyphs, SensorGraphTab, then guard everything; CameraStream moved late for round 3 | T17, T19, T20 |
| declutter-brief section 1.7 note | tabs/ArmTab.tsx unreachable on main: rejected as dead code, round 3 routes the arm tab to it | T18 |
| design D2 | marker shapes per POI kind | T5 |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Neutral theme tokens with a white accent; colour for warning and error only | T1, T2, T6, T9, T10, T17, T19, T20 |
| D2 Map: only zones and paths are coloured; POI kinds greyscale by shape | T3, T4, T5, T17, T18 |
| D3 Docked sidebar on tablet and desktop; bottom sheets on phones; no map popup | T6, T7, T8 |
| D4 One status line, one stop pair, flat chat log, one confirm pattern | T9, T10, T11, T12, T13, T14, T15, T16, T18 |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| theme-tokens | T1 | T2, T3, T4, T6, T7, T9, T10, T11, T17 | `nodes/web_ui/frontend/src/theme.ts` |
| design-rules | T2 | T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15, T16, T17, T18, T19, T20 | `nodes/web_ui/frontend/src/layout/designRules.ts` |
| map-palette | T3 | T4, T5, T12, T17, T18 | `nodes/web_ui/frontend/src/map3d/palette.ts` |
| map-label | T4 | none | `nodes/web_ui/frontend/src/map3d/MapLabel.tsx` |
| poi-kind-shape | T5 | T12, T17 | `nodes/web_ui/frontend/src/poi/style.ts` |
| panel-chrome | T6 | T7, T8, T12, T13, T14, T18 | `nodes/web_ui/frontend/src/layout/OverlayPanel.tsx` |
| confirm-dialog | T6 | T11, T14, T15 | `nodes/web_ui/frontend/src/layout/ConfirmDialog.tsx` |
| map-sidebar | T7 | T8 | `nodes/web_ui/frontend/src/layout/MapSidebar.tsx` |
| status-item | T9 | T11, T12, T13, T15, T16, T18 | `nodes/web_ui/frontend/src/components/StatusItem.tsx` |
| state-banner | T10 | none | `nodes/web_ui/frontend/src/components/StateBanner.tsx` |
| tool-button | T11 | T12, T18 | `nodes/web_ui/frontend/src/components/ToolButton.tsx` |
| map-status-line | T11 | T12, T13 | `nodes/web_ui/frontend/src/map/MapStatusLine.tsx` |

## Review focus

- Stops (T11, T13): STOP and STOP GRASP stay in the primary toolbar on every layout class, call the same handlers as before, and no panel stop remains. Any change to what a stop does is a defect.
- T7: the sidebar shrinks the canvas without breaking frameloop demand (no render loop, no stale canvas) and without regressing map gestures.
- T8: show on map still centres exactly once with a late controller; the migrated popup tests keep their behaviours.
- Information loss: deduplication removes only repeats (model and usage stay in a tooltip, pose moves to the status line, tilt stays in the banner).
- Colour: nothing green for healthy; the accent has no hue; only zones and nav paths are coloured on the map.
- designRules.ts false negatives and false positives (rule g on legitimate Unicode, rule a on non-colour hex).

### Task 1: Neutral theme tokens with a white accent and pointer-split touch sizes

**Files:**
- Modify: `nodes/web_ui/frontend/src/theme.ts`
- Create: `nodes/web_ui/frontend/src/theme.test.ts`
- Modify: `nodes/web_ui/frontend/src/main.tsx`
- Test: `nodes/web_ui/frontend/src/layout/hardcodedSizes.test.ts`

**Interfaces:**
- Consumes: existing theme.ts, main.tsx.
- Produces: `TOKENS`, `MONO_FONT`, `CONTROL_PX`, `TOUCH_TARGET_PX`, `CANVAS_BG`, `PANEL_BG` from `src/theme.ts`.

**Ownership:**
- owns: theme-tokens
- uses: none
- dependsOn: none

**Requirement:**
Test first in src/theme.test.ts (first line `// @vitest-environment jsdom` if it renders), seen failing before theme.ts changes. (1) theme.ts exports `TOKENS` with the brief 2.1 surface (bgCanvas, bgPanel, bgRaised), border (border, borderStrong), text (textPrimary, textSecondary, textDisabled), error and warning values, radius 2, borderWidth 1, panelHeaderPx 32, controlPx 32, touchTargetPx 44, fontSizes 11/12/13/14/16 and fontWeights { regular: 400, medium: 500 } only; there is no `ok` token and no stop weight. (2) `TOKENS.accent` is a neutral white or light grey with no hue (test: R, G and B differ by at most 8 and each is at least 0xd0) and `TOKENS.accentText` is dark (each channel at most 0x30). (3) The MUI theme: palette.mode 'dark'; primary.main === TOKENS.accent; secondary.main === TOKENS.textSecondary; info.main === TOKENS.textSecondary; success.main === TOKENS.textPrimary (healthy is neutral text, never green); error.main and warning.main take the state tokens; background { default: bgCanvas, paper: bgPanel }; divider === TOKENS.border; text uses the three text tokens; shape.borderRadius === 2. (4) Typography: fontSize 13, a system sans fontFamily (no web font); every variant's fontWeight is 400 or 500; overline has textTransform 'none' and letterSpacing 0; button has textTransform 'none'; theme.ts exports `MONO_FONT` (system mono stack). (5) Component overrides: MuiChip root borderRadius 2 with defaultProps variant 'outlined' and size 'small'; MuiAlert defaultProps variant 'outlined' and root borderRadius 2 with borderLeftWidth 3; MuiPaper defaultProps elevation 0 and root backgroundImage 'none' with a 1 px TOKENS.border border; MuiButton defaultProps disableElevation; MuiButton and MuiIconButton root minHeight/minWidth TOUCH_TARGET_PX (44) under `@media (pointer: coarse)` and CONTROL_PX (32) otherwise; MuiTooltip tooltip fontSize 11 on bgRaised with a border; MuiTab minHeight 40 and fontWeight 500 with the indicator in the accent; MuiLinearProgress height 2, radius 0. (6) MuiCssBaseline sets body fontVariantNumeric 'tabular-nums' and body background bgCanvas. (7) `CANVAS_BG === TOKENS.bgCanvas` and `PANEL_BG === TOKENS.bgPanel` (opaque, no alpha); `TOUCH_TARGET_PX` stays 44 and `CONTROL_PX` (32) is exported; layout/hardcodedSizes.test.ts PANEL_BG_LITERAL is updated to the new literal and the test stays green. (8) main.tsx keeps ThemeProvider plus CssBaseline. The full suite `npx vitest run` stays green (the 44 px assertions hold on a coarse pointer). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Every later task builds on one neutral token set, so the white accent and state-only colour are decided in one place.

**Notes:** Brief P1 adapted: accent white, no green ok (D1). F5 b: 44 px coarse, 32 px fine. F7 a: opaque PANEL_BG. F12 a: system sans plus mono. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/theme.test.ts.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): neutral theme tokens with a white accent and state-only colour`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/theme.test.ts src/layout/hardcodedSizes.test.ts && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 2: Design rules guard: designViolations() plus its rule tests

**Files:**
- Create: `nodes/web_ui/frontend/src/layout/designRules.ts`
- Create: `nodes/web_ui/frontend/src/layout/designRules.test.ts`

**Interfaces:**
- Consumes: T1 theme.ts (asserted clean).
- Produces: `designViolations(path, source): string[]` and `SOURCES` from `src/layout/designRules.ts`.

**Ownership:**
- owns: design-rules
- uses: theme-tokens
- dependsOn: T1

**Requirement:**
Test first in src/layout/designRules.test.ts, seen failing before designRules.ts exists. (1) layout/designRules.ts exports `designViolations(path: string, source: string): string[]`, one message per violation naming the rule and line, and `SOURCES: Record<string, string>` built with the same `import.meta.glob` raw pattern as layout/hardcodedSizes.test.ts, excluding test files. (2) Rules, each with a failing and a passing fixture in the test: (a) no `#hex`, `rgb(`, `rgba(`, `hsl(` or `0x` colour literal outside theme.ts and map3d/palette.ts; (b) no boxShadow, backdropFilter, textShadow or gradient; (c) no `color="secondary"`; (d) no Alert with `variant="filled"`; (e) fontWeight only 400 or 500 (600, 700, 800 and 'bold' are violations, STOP included); (f) no off-grid spacing values 1.25, 0.75, 1.5 or 2.5 in sx spacing keys (m, p, gap and their side variants); (g) no em dash, ellipsis character, arrows (U+2190 to U+21FF), the glyphs U+25CF and U+25CB, emoji or other decorative Unicode in string literals or JSX text (the degree sign and plain Latin text are allowed); (h) no `title=` on a `<Button` or `<IconButton` line. (3) designRules.test.ts asserts `designViolations` returns [] for theme.ts and main.tsx from SOURCES. No allow-list of all files yet: later tasks assert their own files, and T20 flips the test to every source. Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** One machine check that holds the declutter rules, so slop cannot creep back file by file.

**Notes:** Brief P2, adapted: instead of a shared GUARDED_FILES list (which every task would edit and conflict on), each later task asserts designViolations() for its own files in its own test; T20 runs it over all sources. Rule (e) follows the design (400 and 500 only), not the brief's 600-for-STOP. Rule (g) covers the user's writing rule. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/layout/designRules.test.ts.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `test(web_ui): design rules guard with per-rule fixtures`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/layout/designRules.test.ts && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 3: One map palette: zones and nav paths coloured, POI kinds and photos greyscale

**Files:**
- Create: `nodes/web_ui/frontend/src/map3d/palette.ts`
- Create: `nodes/web_ui/frontend/src/map3d/palette.test.ts`
- Modify: `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- Modify: `nodes/web_ui/frontend/src/poi/style.ts`
- Test: `nodes/web_ui/frontend/src/poi/style.test.ts`
- Modify: `nodes/web_ui/frontend/src/photos/geometry.ts`
- Test: `nodes/web_ui/frontend/src/photos/geometry.test.ts`

**Interfaces:**
- Consumes: T1 TOKENS, T2 designViolations.
- Produces: `map3d/palette.ts`: ZONE_COLORS, NAV_PATH_COLOR, KIND_COLORS, OBJECT_COLOR, PHOTO_COLOR, SELECTED_COLOR, STATUS_COLORS, LAYER_SWATCH, MAP_LEGEND.

**Ownership:**
- owns: map-palette
- uses: theme-tokens, design-rules
- dependsOn: T2

**Requirement:**
Test first in src/map3d/palette.test.ts, seen failing before palette.ts exists. (1) map3d/palette.ts is the one file that defines every map, scene and marker colour: zone colours (keepout from TOKENS.error, slow from TOKENS.warning), the nav path colour (keeps its hue), KIND_COLORS for every POI kind, OBJECT_COLOR, the photo marker colour, the selection highlight (=== TOKENS.accent), STATUS outline colours, and LAYER_SWATCH and MAP_LEGEND. (2) LAYER_SWATCH (moved out of tabs/MapNavTab.tsx) and the scene colours in map3d/sceneLayers.tsx (SCENE_COLORS) import from palette.ts with no copy; poi/style.ts re-exports or imports KIND_COLORS, ZONE_COLORS, STATUS_COLORS and OBJECT_COLOR from palette.ts and keeps its function API (poiStyle, zoneStyle, markerColors). (3) palette.test.ts asserts: every LAYER_SWATCH entry equals its scene colour; the photos swatch equals the photo marker colour; the POIs swatch equals KIND_COLORS.point; zone keepout and slow equal TOKENS.error and TOKENS.warning; every colour other than the two zones and the nav path is achromatic (R = G = B), including every KIND_COLORS entry, OBJECT_COLOR, the photo colour and STATUS outline colours; POI status stays distinguishable on markers by a neutral lightness step, not hue. (4) photos/geometry.ts: the per-camera hash palette is replaced by the one photo colour from palette.ts (cameraColor removed or returning it); photos/geometry.test.ts is updated to that rule. poi/style.test.ts is updated to the new greyscale values, keeping its zone and markerColors behaviour tests. (5) designViolations() from layout/designRules.ts returns [] for map3d/palette.ts, map3d/sceneLayers.tsx colour constants, poi/style.ts and photos/geometry.ts (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Map colour means something again: red and amber zones and the nav path stand out because nothing else competes.

**Notes:** Brief P3 adapted to D2 (F2 c): kinds greyscale; the previous round's kind fills are removed. Selection highlight uses the white accent. MapNavTab chain: T3 -> T7 -> T8 -> T11 -> T12 -> T13. Arm and robot model colours are not here (T17 adds RobotModel, T18 adds InteractiveArm, after round 3). GraspPreview and PhotoLayer colours move in T4. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/map3d/palette.test.ts.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): one map palette, colour only for zones and nav paths`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/map3d/palette.test.ts src/poi/style.test.ts src/photos/geometry.test.ts && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 4: MapLabel: one map label style; grasp preview and photo layer colours from the palette

**Files:**
- Create: `nodes/web_ui/frontend/src/map3d/MapLabel.tsx`
- Create: `nodes/web_ui/frontend/src/map3d/MapLabel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- Modify: `nodes/web_ui/frontend/src/poi/PoiLayer.tsx`
- Modify: `nodes/web_ui/frontend/src/grasp/GraspPreview.tsx`
- Modify: `nodes/web_ui/frontend/src/photos/PhotoLayer.tsx`

**Interfaces:**
- Consumes: T3 palette.ts, T1 TOKENS.
- Produces: `MapLabel` from `src/map3d/MapLabel.tsx`.

**Ownership:**
- owns: map-label
- uses: theme-tokens, map-palette, design-rules
- dependsOn: T3

**Requirement:**
Test first in src/map3d/MapLabel.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) map3d/MapLabel.tsx exports `MapLabel({ children, zone?: 'keepout' | 'slow' })`: SANS_FONT at 11 px, weight 500, radius 2, an opaque TOKENS.bgPanel background with a 1 px TOKENS.border border, TOKENS.textPrimary text; a zone label carries a border in the zone colour from palette.ts and no other label is coloured. (2) The three inline label styles in map3d/sceneLayers.tsx, poi/PoiLayer.tsx and grasp/GraspPreview.tsx are replaced by `<MapLabel>`; the 800-weight coloured zone word is gone. (3) grasp/GraspPreview.tsx takes OBJECT_COLOR and its path colour from palette.ts (the magenta grasp path becomes the selection accent); photos/PhotoLayer.tsx takes its selected colour from palette.ts. (4) designViolations() from layout/designRules.ts returns [] for map3d/MapLabel.tsx, map3d/sceneLayers.tsx, grasp/GraspPreview.tsx and photos/PhotoLayer.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Map labels read as one calm instrument style instead of three ad hoc ones.

**Notes:** Brief P4 minus the MapPopup restyle: D3 removes MapPopup (T8). Labels keep frameloop demand: no per-frame state. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/map3d/MapLabel.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): one MapLabel style, grasp preview and photo layer colours from the palette`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/map3d/MapLabel.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 5: POI kinds told apart by marker shape, greyscale

**Files:**
- Modify: `nodes/web_ui/frontend/src/poi/style.ts`
- Test: `nodes/web_ui/frontend/src/poi/style.test.ts`
- Modify: `nodes/web_ui/frontend/src/poi/PoiLayer.tsx`
- Modify: `nodes/web_ui/frontend/src/poi/PoiLegend.tsx`
- Test: `nodes/web_ui/frontend/src/poi/PoiLegend.test.tsx`

**Interfaces:**
- Consumes: T3 palette.ts.
- Produces: `KIND_SHAPE`, `MarkerShape` from `src/poi/style.ts`.

**Ownership:**
- owns: poi-kind-shape
- uses: map-palette, design-rules
- dependsOn: T4

**Requirement:**
Test first in src/poi/style.test.ts (new describe block) and src/poi/PoiLegend.test.tsx, seen failing. (1) poi/style.ts exports `KIND_SHAPE: Record<PoiKind, MarkerShape>` with `type MarkerShape` (for example circle, square, diamond, triangle, ring); every kind maps to a distinct shape (test asserts uniqueness over all kinds). (2) poi/PoiLayer.tsx draws each marker in its kind's shape, greyscale from palette.ts, sized to stay distinguishable at phone zoom (the existing on-screen marker size rule is kept). (3) poi/PoiLegend.tsx shows each kind's shape (data-shape attribute) and greyscale colour from palette.ts; PoiLegend.test.tsx asserts each legend swatch's shape equals KIND_SHAPE and its colour equals KIND_COLORS; zone swatches stay red and amber. (4) designViolations() from layout/designRules.ts returns [] for poi/style.ts, poi/PoiLayer.tsx and poi/PoiLegend.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Operators still tell points, objects and photos apart at a glance after the hues are gone.

**Notes:** D2: marker shapes per kind are left to the builder (design 'Left to the builder'). PoiLegend is reused by the Layers panel in T12. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/poi/style.test.ts, src/poi/PoiLegend.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): POI kinds told apart by greyscale marker shape`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts src/poi/PoiLegend.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 6: Flat Panel chrome and the ConfirmDialog

**Files:**
- Modify: `nodes/web_ui/frontend/src/layout/OverlayPanel.tsx`
- Test: `nodes/web_ui/frontend/src/layout/layout.test.tsx`
- Create: `nodes/web_ui/frontend/src/layout/ConfirmDialog.tsx`
- Create: `nodes/web_ui/frontend/src/layout/ConfirmDialog.test.tsx`

**Interfaces:**
- Consumes: T1 TOKENS, PANEL_BG, TOUCH_TARGET_PX.
- Produces: `OverlayPanel` with `actions` slot; `ConfirmDialog` from `src/layout/ConfirmDialog.tsx`.

**Ownership:**
- owns: panel-chrome, confirm-dialog
- uses: theme-tokens, design-rules
- dependsOn: T2

**Requirement:**
Test first in src/layout/layout.test.tsx and src/layout/ConfirmDialog.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) OverlayPanel header is TOKENS.panelHeaderPx (32) high on a fine pointer and TOUCH_TARGET_PX on a coarse pointer, with a sentence-case title (12/500, text.secondary) and an `actions?: ReactNode` slot on the right; close is an IconButton with aria-label 'Close'. (2) No backdropFilter, no shadow, radius 2, opaque PANEL_BG, 1 px TOKENS.border; the phone bottom sheet has a square top and the decorative handle is removed (the `overlay-panel-handle` test id is gone; its test is updated to assert absence). The body padding is `p: 1`. (3) layout/ConfirmDialog.tsx exports `ConfirmDialog({ open, title, body?, confirmLabel, onConfirm, onCancel })`: an MUI Dialog with a Cancel button and an error-coloured confirm button; Escape and Cancel call onCancel and never onConfirm; confirm calls onConfirm once. (4) The existing layout class, pointerdown isolation and touch target tests stay green. (5) designViolations() from layout/designRules.ts returns [] for layout/OverlayPanel.tsx and layout/ConfirmDialog.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Every panel looks and behaves the same, and destructive actions get one confirm pattern.

**Notes:** Brief P5 without the docked mode, which is T7. F9 b: Dialog for destructive actions; ConfirmDialog is consumed by T11 (map reset), T14 (POI and photo deletes) and T15 (reset session). Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/layout/layout.test.tsx, src/layout/ConfirmDialog.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): flat panel chrome and one ConfirmDialog`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/layout/layout.test.tsx src/layout/ConfirmDialog.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 7: Docked map sidebar on tablet and desktop; the map shrinks instead of being covered

**Files:**
- Create: `nodes/web_ui/frontend/src/layout/MapSidebar.tsx`
- Create: `nodes/web_ui/frontend/src/layout/MapSidebar.test.tsx`
- Modify: `nodes/web_ui/frontend/src/layout/OverlayPanel.tsx`
- Test: `nodes/web_ui/frontend/src/layout/layout.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- Modify: `nodes/web_ui/frontend/src/map3d/MapScene.tsx`
- Create: `nodes/web_ui/frontend/src/map3d/useInvalidateOnResize.ts`
- Create: `nodes/web_ui/frontend/src/map3d/useInvalidateOnResize.test.tsx`

**Interfaces:**
- Consumes: T6 OverlayPanel, T1 PANEL_BG.
- Produces: `MapSidebar` from `src/layout/MapSidebar.tsx`; OverlayPanel docked mode; `useInvalidateOnResize(ref)`.

**Ownership:**
- owns: map-sidebar
- uses: panel-chrome, theme-tokens, design-rules
- dependsOn: T3, T6
- model: opus

**Requirement:**
Test first in src/layout/MapSidebar.test.tsx and src/map3d/useInvalidateOnResize.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) layout/MapSidebar.tsx exports `MapSidebar({ children })`: on tablet and desktop a docked right column of width clamp(280px, 30vw, 380px), full height of the map area, opaque PANEL_BG, a 1 px TOKENS.border left border, scrolling internally; on phone it renders its children unchanged so panels stay bottom sheets. (2) OverlayPanel gains a docked mode: inside MapSidebar on tablet and desktop it renders in flow (no absolute position) as a stacked section with the T6 header; phone behaviour is unchanged. (3) tabs/MapNavTab.tsx: on tablet and desktop the map container and MapSidebar sit in one flex row; the sidebar renders only while at least one panel is open (POI list, POI editor, photo panel, photo list, layers, grasp), so with all closed the map takes the full width; the photo list renders only when asked for. A test renders MapNavTab on tablet with each panel open and asserts the panel is inside the sidebar element and not inside the map container. (4) map3d/useInvalidateOnResize.ts exports `useInvalidateOnResize(ref)`: a ResizeObserver on the canvas container calls R3F `invalidate()` once per size change (test with a mocked ResizeObserver: one call per change, none without a change, observer disconnected on unmount). MapScene uses it and keeps frameloop="demand"; the Canvas resizes with its container. (5) Map gestures do not regress: the existing goalGesture, picking, mapFocus and MapNavTab tests stay green; a pointerdown inside the sidebar does not reach the map. (6) designViolations() from layout/designRules.ts returns [] for layout/MapSidebar.tsx and map3d/useInvalidateOnResize.ts (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** On tablet and desktop nothing covers the map any more: panels sit beside it and the map uses the rest of the width.

**Notes:** D3, brief P5 docked mode and O1-O3. Opus: layout rewiring of the largest file plus the R3F resize under frameloop demand, where a wrong fix either stalls the canvas or renders every frame. MapNavTab chain: after T3, before T8. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/layout/MapSidebar.test.tsx, src/map3d/useInvalidateOnResize.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): docked map sidebar on tablet and desktop, map resizes on demand`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/layout/MapSidebar.test.tsx src/map3d/useInvalidateOnResize.test.tsx src/layout/layout.test.tsx src/map3d/mapFocus.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 8: Remove MapPopup: show on map centres and opens the item in the sidebar

**Files:**
- Delete: `nodes/web_ui/frontend/src/map3d/MapPopup.tsx`
- Modify: `nodes/web_ui/frontend/src/map3d/MapScene.tsx`
- Modify: `nodes/web_ui/frontend/src/map3d/mapFocus.ts`
- Test: `nodes/web_ui/frontend/src/map3d/mapFocus.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

**Interfaces:**
- Consumes: T7 MapSidebar, existing consumeFocus and resolveFocus.
- Produces: show-on-map opens the item in the sidebar.

**Ownership:**
- owns: none
- uses: map-sidebar, panel-chrome, design-rules
- dependsOn: T7

**Requirement:**
Test first in src/map3d/mapFocus.test.tsx, seen failing. (1) map3d/MapPopup.tsx is deleted; MapScene has no `popup`, `onPopupClose` or `onPopupOpen` props and no popup render. (2) Show on map (App pendingFocus) still centres once via consumeFocus when the SceneController exists, then opens the item: a POI in the POI editor panel, a photo in the photo panel, in the MapSidebar on tablet and desktop and as a bottom sheet on phone. A map tap on a POI or photo marker opens it the same way. (3) mapFocus.test.tsx: the `MapPopup` describe block is removed and its behaviours are migrated to the sidebar flow: centres once and consumes once with an immediate and with a late controller; never consumes while the controller is missing; an unknown id opens nothing; a photo from App's photos opens the photo panel; only one item is open at a time; a pointerdown inside the opened panel does not reach the map; the opened panel's action buttons meet TOUCH_TARGET_PX on a coarse pointer (migrated from the daa1be6 popup test). (4) designViolations() from layout/designRules.ts returns [] for map3d/MapScene.tsx and map3d/mapFocus.ts (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Finding a POI from the list lands it centred with its details beside the map, with no card covering it.

**Notes:** D3 (F13 b follows from F6 b). MapNavTab chain: after T7, before T11. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/map3d/mapFocus.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): show on map opens the item in the sidebar; MapPopup removed`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/map3d/mapFocus.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 9: StatusItem and the status bar: battery, GPS, link and time budget as text

**Files:**
- Create: `nodes/web_ui/frontend/src/components/StatusItem.tsx`
- Create: `nodes/web_ui/frontend/src/components/StatusItem.test.tsx`
- Modify: `nodes/web_ui/frontend/src/overlays/OverlayBar.tsx`
- Modify: `nodes/web_ui/frontend/src/components/BatteryChip.tsx`
- Modify: `nodes/web_ui/frontend/src/components/GpsChip.tsx`
- Test: `nodes/web_ui/frontend/src/components/GpsChip.test.tsx`
- Modify: `nodes/web_ui/frontend/src/agent/TimeBudgetChip.tsx`
- Test: `nodes/web_ui/frontend/src/agent/TimeBudgetChip.test.tsx`

**Interfaces:**
- Consumes: T1 TOKENS, MONO_FONT.
- Produces: `StatusItem` from `src/components/StatusItem.tsx`.

**Ownership:**
- owns: status-item
- uses: theme-tokens, design-rules
- dependsOn: T2

**Requirement:**
Test first in src/components/StatusItem.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) components/StatusItem.tsx exports `StatusItem({ label, value, unit?, state?: 'ok' | 'warn' | 'error' | 'unknown', tooltip? })`: label 11 px text.secondary, value 13 px MONO_FONT with tabular numerals, unit 11 px text.secondary; warn and error colour the value text with the state token and add a 6 px square state mark on the left; ok and unknown render the value in text.primary with no mark (never green). (2) BatteryChip, GpsChip (rover and base) and TimeBudgetChip keep their names and props but render a StatusItem: no `<Chip`, no status icons (the full-battery icon on a low level is gone), GPS quality 1 is not coloured; the tooltips keep their details. (3) overlays/OverlayBar.tsx renders its values as StatusItems; the link shows `Live` or `Connecting` with no glyphs and no green; an empty value is `--`; the overline header style is gone. (4) GpsChip.test.tsx and TimeBudgetChip.test.tsx are updated to the StatusItem rendering and keep their state mapping cases. (5) designViolations() from layout/designRules.ts returns [] for StatusItem.tsx, OverlayBar.tsx, BatteryChip.tsx, GpsChip.tsx and TimeBudgetChip.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Battery, GPS and link read as one quiet instrument line where only a problem draws the eye.

**Notes:** Brief P6 with D1 F4 a: ok is neutral. App.tsx is not edited here (the chip components keep their names); App.tsx belongs to T10. Each fact is shown once: tilt shows only through the T10 StateBanner. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/components/StatusItem.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): StatusItem status line for battery, GPS, link and time budget`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/components/StatusItem.test.tsx src/components/GpsChip.test.tsx src/agent/TimeBudgetChip.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 10: App chrome: one StateBanner, no duplicate tilt toast, tab bar without icons

**Files:**
- Modify: `nodes/web_ui/frontend/src/App.tsx`
- Create: `nodes/web_ui/frontend/src/components/StateBanner.tsx`
- Create: `nodes/web_ui/frontend/src/components/StateBanner.test.tsx`
- Delete: `nodes/web_ui/frontend/src/components/TiltBanner.tsx`
- Delete: `nodes/web_ui/frontend/src/components/TiltBanner.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/AgentChatTab.tsx`

**Interfaces:**
- Consumes: T1 theme Alert overrides.
- Produces: `StateBanner` from `src/components/StateBanner.tsx`.

**Ownership:**
- owns: state-banner
- uses: theme-tokens, design-rules
- dependsOn: T2

**Requirement:**
Test first in src/components/StateBanner.test.tsx (`// @vitest-environment jsdom`, migrated from TiltBanner.test.tsx), seen failing. (1) components/StateBanner.tsx exports `StateBanner({ state: 'warn' | 'error', children })`: a full-width standard (outlined) Alert with a 3 px left border in the state colour, no `variant="filled"`. components/TiltBanner.tsx and its test are removed; their cases live in StateBanner.test.tsx. (2) App.tsx: the battery cut-off banner and the tilt banner both render through StateBanner; the tilt entry Snackbar is removed (the banner covers it); the error toast is outlined. (3) The tab bar shows text labels without icons; the drawer keeps its icons (test on the tab bar and the drawer). (4) tabs/AgentChatTab.tsx: the composer's second cut-off Alert becomes a one-line caption with the block reason. (5) No ellipsis character in App.tsx. designViolations() from layout/designRules.ts returns [] for App.tsx and components/StateBanner.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** A problem is announced once, in one consistent strip, and the tab bar stops shouting.

**Notes:** Brief P7 with F8 a. Round 3 T28 edits App.tsx (arm tab type): re-read it and keep the arm tab entry. AgentChatTab chain: T10 -> T15 -> T16. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/components/StateBanner.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): one StateBanner, no duplicate tilt toast, tab bar labels only`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/components/StateBanner.test.tsx src/tabSelection.test.ts && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 11: Icon-only map toolbar with STOP and STOP GRASP, plus the map status line

**Files:**
- Create: `nodes/web_ui/frontend/src/components/ToolButton.tsx`
- Create: `nodes/web_ui/frontend/src/components/ToolButton.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- Modify: `nodes/web_ui/frontend/src/map/toolbarGroups.ts`
- Test: `nodes/web_ui/frontend/src/map/toolbarGroups.test.ts`
- Create: `nodes/web_ui/frontend/src/map/MapStatusLine.tsx`
- Create: `nodes/web_ui/frontend/src/map/MapStatusLine.test.tsx`

**Interfaces:**
- Consumes: T9 StatusItem, T6 ConfirmDialog.
- Produces: `ToolButton`, `ToolGroup` from `src/components/ToolButton.tsx`; `MapStatusLine` from `src/map/MapStatusLine.tsx`.

**Ownership:**
- owns: tool-button, map-status-line
- uses: status-item, confirm-dialog, theme-tokens, design-rules
- dependsOn: T8, T9, T6

**Requirement:**
Test first in src/components/ToolButton.test.tsx, src/map/MapStatusLine.test.tsx and src/map/toolbarGroups.test.ts, seen failing. (1) components/ToolButton.tsx exports `ToolButton({ label, icon, onClick, pressed?, disabled? })` (an IconButton in a Tooltip whose aria-label equals the tooltip text; `aria-pressed` set only for toggles; active state is the accent icon colour on bgRaised; no `title=`) and `ToolGroup` (grouped buttons share one 1 px bordered strip). (2) tabs/MapNavTab.tsx toolbar: every control except STOP and STOP GRASP is a ToolButton; no startIcon text buttons, no `title=`, no label morphing ('Tap map to set goal', 'Click map', 'Area: n vertices' are gone from labels); goal and POI modes use the accent active state, not `secondary`; the overflow menu is kept. (3) STOP and STOP GRASP are the only filled-colour controls: contained error, weight 500, text label, no letter spacing. Both sit in the primary toolbar on every layout class, never in the overflow menu, whenever their action applies (map/toolbarGroups.ts encodes this and toolbarGroups.test.ts asserts it per layout class). Safety behaviour is unchanged: each stop calls the same handler and publish as before (test). (4) map/MapStatusLine.tsx exports `MapStatusLine({ items })`: one line in a map corner rendering StatusItems or plain text; MapNavTab shows the mode hint there (goal mode, POI placement with the vertex count, grasp pick hint), and the floating grasp pick Alert over the map is removed. (5) Map reset uses ConfirmDialog (destructive); set home and release keep the two-click arm (robot motion). (6) designViolations() from layout/designRules.ts returns [] for components/ToolButton.tsx, map/toolbarGroups.ts and map/MapStatusLine.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** The toolbar becomes a quiet strip of icons where STOP and STOP GRASP are the only things that look urgent.

**Notes:** Brief P8 with D4 F10 b (STOP plus STOP GRASP, toolbar only) and F9 b. Anti-goal: a stop must never get harder to reach. MapNavTab chain: after T8, before T12. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/components/ToolButton.test.tsx, src/map/MapStatusLine.test.tsx, src/map/toolbarGroups.test.ts.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): icon-only map toolbar with STOP and STOP GRASP, map status line`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/components/ToolButton.test.tsx src/map/MapStatusLine.test.tsx src/map/toolbarGroups.test.ts && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 12: Layers panel extracted and decluttered

**Files:**
- Create: `nodes/web_ui/frontend/src/map/LayersPanel.tsx`
- Create: `nodes/web_ui/frontend/src/map/LayersPanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- Modify: `nodes/web_ui/frontend/src/poi/PoiLegend.tsx`
- Test: `nodes/web_ui/frontend/src/poi/PoiLegend.test.tsx`

**Interfaces:**
- Consumes: T11 ToolButton and MapStatusLine, T5 KIND_SHAPE, T3 palette.
- Produces: `LayersPanel` from `src/map/LayersPanel.tsx`.

**Ownership:**
- owns: none
- uses: tool-button, map-status-line, status-item, map-palette, poi-kind-shape, panel-chrome, design-rules
- dependsOn: T11, T5

**Requirement:**
Test first in src/map/LayersPanel.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) map/LayersPanel.tsx holds the Layers panel moved out of tabs/MapNavTab.tsx, on OverlayPanel (docked in the sidebar on tablet and desktop). (2) Each disabled layer's reason is the row's tooltip, not a caption; the gesture and arm help paragraphs move behind one info ToolButton in the panel header actions (their content kept in its tooltip); no `mt: -0.5` or `ml: 4` offsets. (3) Layer toggles show their on state with the accent, not primary fill; swatches and the PoiLegend come from palette.ts and KIND_SHAPE. (4) The robot pose line leaves the panel and shows in the MapStatusLine as a mono StatusItem (the fact keeps one display). (5) designViolations() from layout/designRules.ts returns [] for map/LayersPanel.tsx and poi/PoiLegend.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** The Layers panel becomes a short list of toggles, and MapNavTab sheds about 125 lines.

**Notes:** Brief P9. MapNavTab chain: after T11, before T13. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/map/LayersPanel.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `refactor(web_ui): Layers panel extracted, reasons in tooltips, pose in the status line`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/map/LayersPanel.test.tsx src/poi/PoiLegend.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 13: Grasp panel: no panel stops, one pick hint, compact result

**Files:**
- Modify: `nodes/web_ui/frontend/src/grasp/GraspPanel.tsx`
- Test: `nodes/web_ui/frontend/src/grasp/GraspPanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

**Interfaces:**
- Consumes: T11 MapStatusLine and toolbar STOP GRASP, T9 StatusItem.
- Produces: none.

**Ownership:**
- owns: none
- uses: map-status-line, status-item, panel-chrome, design-rules
- dependsOn: T12

**Requirement:**
Test first in src/grasp/GraspPanel.test.tsx, seen failing. (1) Both grasp-panel stop buttons (STOP GRASP and the outlined Stop) are removed; the panel has no button whose name matches /stop/i (test); STOP GRASP in the toolbar (T11) is the stop. (2) The pick hint shows once, in the MapStatusLine; the panel's hint Alert is removed. (3) The grasp result renders as compact key-value rows (StatusItem), not stacked Alerts; 'Planning' and 'Running' are one status row; the panel has at most one helper caption. (4) The panel uses the T6 chrome and one default button size; the existing phone pick compaction test is updated, not deleted. (5) designViolations() from layout/designRules.ts returns [] for grasp/GraspPanel.tsx and tabs/MapNavTab.tsx (last task in the MapNavTab chain) (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** One place to stop a grasp and a result that reads in one glance.

**Notes:** Brief P10 grasp part with F10 b. No change to grasp safety logic or the stop handler. MapNavTab chain ends here. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/grasp/GraspPanel.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): grasp panel without duplicate stops, one pick hint, compact result`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/grasp/GraspPanel.test.tsx src/map/toolbarGroups.test.ts && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 14: POI and photo panels: status text, quiet editor, one confirm pattern

**Files:**
- Modify: `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- Test: `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`
- Modify: `nodes/web_ui/frontend/src/photos/PhotoPanel.tsx`
- Test: `nodes/web_ui/frontend/src/photos/PhotoPanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/photos/PhotoList.tsx`
- Test: `nodes/web_ui/frontend/src/photos/PhotoList.test.tsx`

**Interfaces:**
- Consumes: T6 ConfirmDialog and OverlayPanel.
- Produces: none.

**Ownership:**
- owns: none
- uses: confirm-dialog, panel-chrome, design-rules
- dependsOn: T6

**Requirement:**
Test first in src/poi/PoiPanels.test.tsx, src/photos/PhotoPanel.test.tsx and src/photos/PhotoList.test.tsx, seen failing. (1) The POI list status is text, with a state mark only for `open`; no filled status chip and no hard-coded text colour. (2) The POI editor shows a character counter only within 10% of the limit; the drag hint paragraph is removed; the list's 'Collapse' text button is removed and the count shows once. (3) POI delete, photo delete and photo cleanup use ConfirmDialog; the inline confirm rows are removed. (4) One button size (default) across the three panels; headers use the T6 actions slot. (5) designViolations() from layout/designRules.ts returns [] for poi/PoiPanels.tsx, photos/PhotoPanel.tsx and photos/PhotoList.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** POI and photo panels show the data, not instructions, and confirm destructive actions the same way.

**Notes:** Brief P10 POI and photo part with F9 b. Not in the MapNavTab chain (no MapNavTab edit). Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/poi/PoiPanels.test.tsx, src/photos/PhotoPanel.test.tsx, src/photos/PhotoList.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): POI and photo panels with status text and one confirm dialog`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx src/photos/PhotoPanel.test.tsx src/photos/PhotoList.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 15: Agent header: state, phase dropdown and time left only

**Files:**
- Create: `nodes/web_ui/frontend/src/agent/AgentHeader.tsx`
- Create: `nodes/web_ui/frontend/src/agent/AgentHeader.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/AgentChatTab.tsx`
- Modify: `nodes/web_ui/frontend/src/agent/PhasePanel.tsx`
- Test: `nodes/web_ui/frontend/src/agent/PhasePanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/agent/PhaseProgress.tsx`
- Test: `nodes/web_ui/frontend/src/agent/PhaseProgress.test.tsx`

**Interfaces:**
- Consumes: T9 StatusItem and TimeBudgetChip, T6 ConfirmDialog.
- Produces: `AgentHeader` from `src/agent/AgentHeader.tsx`.

**Ownership:**
- owns: none
- uses: status-item, confirm-dialog, design-rules
- dependsOn: T10, T9, T6

**Requirement:**
Test first in src/agent/AgentHeader.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) agent/AgentHeader.tsx (extracted from tabs/AgentChatTab.tsx) shows at most three things: the state, the phase dropdown and the time left, as StatusItems with zero `<Chip`. Model and usage move into the state item's tooltip, so no fact is lost; complexity leaves the header. (2) The busy state is shown once (the duplicate 'working' chip or progress bar goes). (3) PhasePanel transcript rows show status as text, coloured only for failed and skipped; 'raised', attempt, iteration and p are plain text joined by ' - '; the existing PhasePanel and PhaseProgress test ids are kept or updated, not deleted. (4) Reset session uses ConfirmDialog. (5) designViolations() from layout/designRules.ts returns [] for agent/AgentHeader.tsx, agent/PhasePanel.tsx and agent/PhaseProgress.tsx (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** The agent header fits one line and says what the agent is doing and how long is left.

**Notes:** Brief P11 header part with D4. TimeBudgetChip (T9) already renders a StatusItem. AgentChatTab chain: after T10, before T16. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/agent/AgentHeader.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): agent header with state, phase and time left only`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/agent/AgentHeader.test.tsx src/agent/PhasePanel.test.tsx src/agent/PhaseProgress.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 16: Agent chat as a flat log with a role column

**Files:**
- Create: `nodes/web_ui/frontend/src/agent/ChatLog.tsx`
- Create: `nodes/web_ui/frontend/src/agent/ChatLog.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/AgentChatTab.tsx`

**Interfaces:**
- Consumes: T15 AgentHeader layout.
- Produces: `ChatLog` from `src/agent/ChatLog.tsx`.

**Ownership:**
- owns: none
- uses: status-item, design-rules
- dependsOn: T15

**Requirement:**
Test first in src/agent/ChatLog.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) agent/ChatLog.tsx renders the transcript as a flat log: one row per entry with a fixed-width role column (User, Agent, Tool, System) and the content beside it; no bubbles and no primary.dark fill. (2) Tool entries show the tool kind as a mono text label with no colour; tool status is coloured only on error. (3) The PlanCard has no secondary border; complexity shows only in the plan card, as text; phase start and complete entries are single log rows without chips. (4) The composer placeholder is short (no keyboard help) and the empty state is one line. (5) designViolations() from layout/designRules.ts returns [] for agent/ChatLog.tsx and tabs/AgentChatTab.tsx (last task in the AgentChatTab chain) (asserted in the task's test). Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** The chat reads like an operations log: scannable, dense and without decoration.

**Notes:** Brief P11 transcript part with F11 b. AgentChatTab chain ends here. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/agent/ChatLog.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): agent chat as a flat log with a role column`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/agent/ChatLog.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 17: POI tab filters, sensor graph and RGBD tabs, robot model colours

**Files:**
- Modify: `nodes/web_ui/frontend/src/tabs/PoiTab.tsx`
- Test: `nodes/web_ui/frontend/src/tabs/PoiTab.test.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/RgbdCameraTab.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/SensorGraphTab.tsx`
- Modify: `nodes/web_ui/frontend/src/components3d/RobotModel.tsx`
- Modify: `nodes/web_ui/frontend/src/map3d/palette.ts`
- Create: `nodes/web_ui/frontend/src/tabs/tabsDesign.test.ts`

**Interfaces:**
- Consumes: T3 palette, T5 KIND_SHAPE.
- Produces: none.

**Ownership:**
- owns: none
- uses: map-palette, poi-kind-shape, theme-tokens, design-rules
- dependsOn: T5

**Requirement:**
Test first in src/tabs/PoiTab.test.tsx and src/tabs/tabsDesign.test.ts, seen failing. (1) PoiTab's kind filter is a ToggleButtonGroup or outlined radius-2 chips with the accent selected state, not primary-filled chips; each kind swatch shows its KIND_SHAPE in its palette.ts greyscale colour. (2) RgbdCameraTab's ellipsis characters become '...'. (3) SensorGraphTab axis, grid and series colours come from TOKENS or palette.ts. (4) RobotModel colours move into palette.ts as achromatic or token values. (5) tabs/tabsDesign.test.ts asserts designViolations() returns [] for PoiTab.tsx, RgbdCameraTab.tsx, SensorGraphTab.tsx and components3d/RobotModel.tsx. Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** The remaining tabs follow the same quiet rules as the map.

**Notes:** Brief P12 minus the all-files guard (T20) and minus components/CameraStream.tsx (round 3 edits it; T19). Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/tabs/PoiTab.test.tsx, src/tabs/tabsDesign.test.ts.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): POI tab filters and remaining tabs on tokens`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/tabs/PoiTab.test.tsx src/tabs/tabsDesign.test.ts && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 18: Arm tab, ArmView and ArmJogPanel on the declutter rules (after round 3)

**Files:**
- Modify: `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- Test: `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- Modify: `nodes/web_ui/frontend/src/arm/ArmView.tsx`
- Modify: `nodes/web_ui/frontend/src/arm/ArmJogPanel.tsx`
- Test: `nodes/web_ui/frontend/src/arm/ArmJogPanel.test.tsx`
- Modify: `nodes/web_ui/frontend/src/components3d/InteractiveArm.tsx`
- Modify: `nodes/web_ui/frontend/src/map3d/palette.ts`

**Interfaces:**
- Consumes: T11 ToolButton, T9 StatusItem, T6 chrome, T3 palette.
- Produces: none.

**Ownership:**
- owns: none
- uses: tool-button, status-item, panel-chrome, map-palette, design-rules
- dependsOn: T13, T16, T17

**Requirement:**
Test first in src/tabs/ArmTab.test.tsx and src/arm/ArmJogPanel.test.tsx, seen failing. Starts only once round 3 has landed the arm tab on main. (1) tabs/ArmTab.tsx is the live arm tab after round 3, so it is kept and restyled, not deleted: panels use the T6 chrome; no tab icon in the tab bar. (2) arm/ArmJogPanel.tsx: jog and mode controls are ToolButtons; Take control, Release and Home keep the two-click arm (robot motion); the control owner (active source) shows once, as a StatusItem; no status chips. (3) arm/ArmView.tsx and components3d/InteractiveArm.tsx take every colour from palette.ts (arm colours added there, achromatic except a state colour) and the camera-mode switch is a ToolGroup. (4) The tests assert designViolations() returns [] for tabs/ArmTab.tsx, arm/ArmView.tsx, arm/ArmJogPanel.tsx and components3d/InteractiveArm.tsx; round 3's arm behaviour tests stay green. Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** The arm tab, including the control owner, matches the rest of the UI once round 3 lands.

**Notes:** Late on purpose: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel) is landing on main. If round 3 named a file differently, use its real path and say so in the report. Arm motion safety: no live motion in this task. Design assumption 'ArmTab.tsx may be dead code' is settled: round 3 T28 routes the arm tab to it, so it is restyled. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/tabs/ArmTab.test.tsx, src/arm/ArmJogPanel.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `feat(web_ui): arm tab, view and jog panel on the declutter rules`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx src/arm/ArmJogPanel.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 19: Camera stream and camera tab copy and chrome (after round 3)

**Files:**
- Modify: `nodes/web_ui/frontend/src/components/CameraStream.tsx`
- Modify: `nodes/web_ui/frontend/src/tabs/CameraTab.tsx`
- Create: `nodes/web_ui/frontend/src/components/CameraStream.test.tsx`

**Interfaces:**
- Consumes: T2 designViolations.
- Produces: none.

**Ownership:**
- owns: none
- uses: design-rules
- dependsOn: T18

**Requirement:**
Test first in src/components/CameraStream.test.tsx (`// @vitest-environment jsdom`), seen failing. (1) CameraStream's ellipsis character becomes '...' and its loading and error text is one short line. (2) CameraTab and CameraStream use theme tokens only (no raw colours, no filled Alerts). (3) The test asserts designViolations() returns [] for components/CameraStream.tsx and tabs/CameraTab.tsx and that the loading text has no ellipsis character. Existing tests touched by moved behaviour are updated, not deleted. npm run lint and npx tsc --noEmit are clean.

**Value:** Camera views lose the last stray glyphs and follow the shared rules.

**Notes:** Late on purpose: round 3 T27 edits CameraStream.tsx and CameraTab.tsx. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/components/CameraStream.test.tsx.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `fix(web_ui): camera stream copy and chrome on the declutter rules`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run src/components/CameraStream.test.tsx && npm run lint && npx tsc --noEmit` - every test passes, lint prints no warnings, tsc prints nothing.

### Task 20: Guard every source with the design rules; README design section

**Files:**
- Test: `nodes/web_ui/frontend/src/layout/designRules.test.ts`
- Modify: `nodes/web_ui/frontend/src/layout/designRules.ts`
- Modify: `nodes/web_ui/README.md`

**Interfaces:**
- Consumes: every earlier task.
- Produces: none.

**Ownership:**
- owns: none
- uses: design-rules
- dependsOn: T13, T14, T16, T17, T18, T19, T10, T12

**Requirement:**
Test first: flip src/layout/designRules.test.ts to iterate every non-test source in SOURCES with no allow-list, and see it fail if any file still violates (it must pass once all earlier tasks landed; fix any stray violation found in a file no earlier task listed and name it in the report). (1) designRules.test.ts asserts designViolations() returns [] for every non-test source under src. (2) nodes/web_ui/README.md gains a short 'Design rules' section: TOKENS and the white accent, colour only for warning and error, map palette.ts (zones and paths coloured, POI kinds by shape), StatusItem, ToolButton, StateBanner, ConfirmDialog versus two-click arm, MapSidebar, and the guard test. (3) The full suite, lint, type check and build pass: `npx vitest run && npm run lint && npx tsc --noEmit && npm run build`.

**Value:** The rules hold for the whole UI from now on, and the next developer can read them in one place.

**Notes:** Brief P12 final step. Deploy (scripts/deploy-nodes.sh client web_ui, logged to .logs/) is the orchestrator's, after the run. Re-read every file in `files` at start and plan against what is on main then: round 3 (arm tab type, ArmView, arm routes, ArmJogPanel, leader_bridge Ansible) is still landing, so brief line numbers may be stale.

**Steps:**
- [ ] Re-read every listed file on current main.
- [ ] Write the test in src/layout/designRules.test.ts.
- [ ] Run it. Expected: FAIL, because the behaviour in the requirement does not exist yet.
- [ ] Implement.
- [ ] Run it. Expected: PASS.
- [ ] Lint and type check.
- [ ] Commit: `test(web_ui): design rules guard covers every source; README design rules`

**Verify:**
`cd nodes/web_ui/frontend && npx vitest run && npm run lint && npx tsc --noEmit && npm run build` - every test passes, lint prints no warnings, tsc prints nothing.
## Execution

- Run name: `web-ui-declutter`.
- Model per task: sonnet, except T7 on opus (docked sidebar plus R3F resize under frameloop demand in the largest file).
- Lenses run: research-code-internal (declutter-brief.md); research-code-external, research-papers and research-docs skipped, reasons in the design doc Research table.
- Lint: `node /Users/szymonri/Documents/claude-infra-plugin/workflows/validate-tasks.mjs docs/plans/2026-10-12-web-ui-declutter-tasks.json --plan docs/plans/2026-10-12-web-ui-declutter-plan.md --approvals docs/plans/2026-10-12-web-ui-declutter-approvals.json --design docs/plans/2026-10-12-web-ui-declutter-design.md`
- Tasks JSON: `docs/plans/2026-10-12-web-ui-declutter-tasks.json`.
- Approvals: `docs/plans/2026-10-12-web-ui-declutter-approvals.json`.
- After the run: push to main, then deploy web_ui to the client with `scripts/deploy-nodes.sh client web_ui`, logged to `.logs/`.
