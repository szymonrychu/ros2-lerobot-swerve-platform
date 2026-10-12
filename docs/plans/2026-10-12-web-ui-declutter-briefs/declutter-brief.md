# Web UI declutter brief

Date: 2026-10-12. Base: main at 63669e1. Scope: `nodes/web_ui/frontend/src` (React 18, MUI 6, R3F). Read-only research, nothing edited.

User request: "Declutter + deslopify. Make it utilitarian and professional. Like Grafana, but without color details."

All paths below are relative to `nodes/web_ui/frontend/src`.

## 1. Inventory of clutter and slop

### 1.1 Theme and global chrome

| # | Finding | Evidence |
|---|---|---|
| T1 | Global radius 8 makes every Paper, Button and card rounded. MUI Chips stay pill-shaped because they have their own 16 px radius. | `theme.ts:29` |
| T2 | There are two brand colours, cyan primary and yellow secondary. Secondary is used for decoration (the Set goal button, plan cards, complexity chips), not for state. | `theme.ts:20-21`; `tabs/MapNavTab.tsx:599`; `tabs/AgentChatTab.tsx:141,144,345` |
| T3 | Overlay panels are translucent (`PANEL_BG` at 0.92) with a 6 px backdrop blur. This is glass decoration over the map. | `theme.ts:15`; `layout/OverlayPanel.tsx:56-57` |
| T4 | The phone sheet has 16 px top corners and a drag handle. The handle has no drag behaviour, so it is decoration only. | `layout/OverlayPanel.tsx:42,59-67` |
| T5 | Panel headers use MUI `overline` (uppercase, 0.1em letter spacing, weight 600). The same style is used for 7 different heading roles. | `theme.ts:32`; `layout/OverlayPanel.tsx:69`; `overlays/OverlayBar.tsx:66`; `tabs/MapNavTab.tsx:1073,1167`; `poi/PoiPanels.tsx:65`; `tabs/AgentChatTab.tsx:109,117` |
| T6 | Banners and toasts use the saturated `variant="filled"` Alert: the battery cut-off banner, the tilt banner, the tilt toast, the error toast and the map action toast. | `App.tsx:279,317,330`; `components/TiltBanner.tsx:8`; `tabs/MapNavTab.tsx:1203` |
| T7 | The tab bar draws a decorative icon on every tab (SmartToy, Camera and others). | `App.tsx:68-79,267` |
| T8 | The PoiPanels test wraps components in a ThemeProvider, but only `main.tsx` has CssBaseline. This is fine and is noted only so a theme-token test does not depend on it. | `main.tsx`, `poi/PoiPanels.test.tsx:29` |

### 1.2 Status displays (chips, badges, duplicates)

| # | Finding | Evidence |
|---|---|---|
| S1 | Battery and GPS are filled colour chips with a status icon. The battery shows a full-battery icon even when the level is `low`, and GPS `quality 1` is coloured `info` (blue) although it is not a state. | `components/BatteryChip.tsx:17-28`; `components/GpsChip.tsx:12-46`; `gps/gpsStatus.ts:101-107`; `battery/batteryStatus.ts:35-38` |
| S2 | The link chip uses decorative glyphs (`'● live'`, `'○ connecting…'`) and is green when healthy. | `overlays/OverlayBar.tsx:82-88` |
| S3 | The battery cut-off is shown three times: a filled red banner, the red chip, and a second Alert in the agent composer. | `App.tsx:278-282`; `components/BatteryChip.tsx`; `tabs/AgentChatTab.tsx:392-396` |
| S4 | Tilt is shown twice with the same text: a persistent filled banner, plus a filled Snackbar toast on entry. | `App.tsx:283` (TiltBanner) and `App.tsx:322-336` |
| S5 | The agent header holds up to about 10 chips in one row: model, state, complexity, phase, time budget, attempt, iteration and N usage chips. The busy state is shown twice, as the 'working' chip and as a LinearProgress bar. | `tabs/AgentChatTab.tsx:342-358`; `agent/PhasePanel.tsx:136-147`; `agent/PhaseProgress.tsx:16-18`; `agent/TimeBudgetChip.tsx:40` |
| S6 | Complexity is shown as a coloured chip in the header and again in every PlanCard. | `tabs/AgentChatTab.tsx:345` and `:144` |
| S7 | Phase transcript rows draw 2 to 5 chips each: a coloured status chip, 'raised', attempt, iteration and p. Phase start and complete cards add more chips. | `agent/PhasePanel.tsx:98-101`; `tabs/AgentChatTab.tsx:164,178` |
| S8 | Tool cards carry a filled colour chip for the tool kind (sensor is `info`, effector is `warning`, notes is `secondary`) plus a status chip. Colour encodes category, not state. | `tabs/AgentChatTab.tsx:49-55,91-92` |
| S9 | The POI list status chip is filled with a raw status colour and has hard-coded `#0d1117` text. | `poi/PoiPanels.tsx:104-108` |
| S10 | There are three STOP buttons for one grasp: STOP GRASP in the toolbar, STOP GRASP in the panel, and a third outlined Stop in the panel. | `tabs/MapNavTab.tsx:571-582`; `grasp/GraspPanel.tsx:135-147,323-335` |
| S11 | The grasp pick hint is shown twice: an Alert floating over the map and an Alert inside the grasp panel. | `tabs/MapNavTab.tsx:1047-1061`; `grasp/GraspPanel.tsx:149-159` |
| S12 | The grasp result is a full Alert with bold text, bullet lines and a step list, stacked under further Alerts ('Planning...', 'Running...'). | `grasp/GraspPanel.tsx:62-117,337-343` |

### 1.3 Verbose helper text and instruction labels

| # | Finding | Evidence |
|---|---|---|
| V1 | The Layers panel ends with paragraphs about gestures and arm help, plus a pose line. | `tabs/MapNavTab.tsx:1176-1190` |
| V2 | Each disabled layer gets a caption explaining why, with `ml: 4, mt: -0.5` offset hacks. | `tabs/MapNavTab.tsx:1093-1101,1123-1131,1155-1159` |
| V3 | Button labels morph into instructions: 'Tap map to set goal', 'Click map', 'Area: n vertices'. | `tabs/MapNavTab.tsx:609,627,645` |
| V4 | Several buttons have a Tooltip and also a `title=` attribute, so the user gets a double tooltip. | `tabs/MapNavTab.tsx:595-607` (also 615-649) |
| V5 | The POI editor always shows character counters and a drag hint paragraph. | `poi/PoiPanels.tsx:184,196,248-250` |
| V6 | The grasp panel has four explanatory captions. | `grasp/GraspPanel.tsx:161-165,190-193,231-233,296-299` |
| V7 | The chat placeholder repeats the keyboard help; empty-state prose. | `tabs/AgentChatTab.tsx:362-364,402` |
| V8 | The POI list has a 'Collapse' text button under a header that already names the panel; the count appears in both headers. | `poi/PoiPanels.tsx:44-85` |

### 1.4 Inconsistent buttons, type and spacing

| # | Finding | Evidence |
|---|---|---|
| B1 | Toolbar buttons use six styles: contained error large weight 800 with letter spacing (STOP), contained/outlined secondary yellow (goal), outlined ButtonGroups, contained warning when armed, an icon group in a bordered Stack, and a dropdown button. Each carries a start icon; MapNavTab imports 18 icons. | `tabs/MapNavTab.tsx:560-776` |
| B2 | Three different confirm patterns: a Dialog (POI delete, agent reset), inline confirm rows (photo delete, photo cleanup), and two-click arming (map reset, set home, release). | `poi/PoiPanels.tsx:260-273`; `tabs/AgentChatTab.tsx:416-425`; `photos/PhotoPanel.tsx:107-123`; `photos/PhotoList.tsx:215-223`; `tabs/MapNavTab.tsx:702-713,751-762`; `grasp/GraspPanel.tsx:285-294` |
| B3 | Button sizes are mixed. The photo panel uses `size="small"` while the POI editor uses default size with `minHeight`. The map popup hand-rolls HTML buttons with its own primary `#4cc9f0`, which differs from the theme's `#4fc3f7`. | `photos/PhotoPanel.tsx:104-121`; `poi/PoiPanels.tsx:252-257`; `map3d/MapPopup.tsx:174-184` |
| B4 | Font weights in use are 600 (6 places), 700 (7) and 800 (4), on top of the MUI defaults. Raw font sizes are 11, 12, 13 and `'0.65rem'`. The type scale is used ad hoc: caption appears 38 times and body2 32 times, often for the same role. | grep across `*.tsx`; `tabs/AgentChatTab.tsx:110,120`; `tabs/MapNavTab.tsx:973`; `map3d/MapPopup.tsx:43,123,182` |
| B5 | Spacing is off the 8 px grid: 1.5 (12 px) is used 22 times, plus 1.25, 0.75, 0.25 and 2.5. Paddings are mixed: px 1 / 1.5 / 2 (`overlays/OverlayBar.tsx:42`, `tabs/AgentChatTab.tsx:342,386`, `tabs/PoiTab.tsx:57`, `layout/OverlayPanel.tsx:68,76`). | grep across `*.tsx` |
| B6 | The map's HTML labels repeat one inline style three times, with raw `rgba(13, 17, 23, 0.78)`, a bare `sans-serif` font (not `SANS_FONT`), and an 800-weight coloured zone word. | `map3d/sceneLayers.tsx:418-430`; `poi/PoiLayer.tsx:33-49`; `grasp/GraspPreview.tsx:45-53` |
| B7 | The map popup has a drop shadow, a 6 px radius, a raw border colour and a raw text colour. | `map3d/MapPopup.tsx:26,113-125` |

### 1.5 Colour used where text or a neutral treatment would do

| # | Finding | Evidence |
|---|---|---|
| C1 | There are about 25 distinct saturated hues across the map. Raw hex or rgba appears in 13 files outside theme.ts: MapNavTab 14, sceneLayers 13, InteractiveArm 7, poi/style 6, MapPopup 6, PoiLayer 4, GraspPreview 4, SensorGraphTab 3, RobotModel 2, and others. | grep `#hex/rgba/0x` |
| C2 | Colour tables are duplicated and inconsistent. `LAYER_SWATCH` copies `SCENE_COLORS` (the comment admits it). The photos swatch is `#4cc9f0`, but `KIND_COLORS.photo` is `#ff7ac6` and the map wedges use a per-camera hash palette, so photos have three colours. The POIs layer swatch `#ffb000` is the open-status colour, not the point kind colour `#2fd0e8`. `OBJECT_COLOR` is `#b967ff` in poi/style but `#ffb000` in GraspPreview. | `tabs/MapNavTab.tsx:94-112`; `map3d/sceneLayers.tsx:27-39`; `poi/style.ts:7-17`; `photos/geometry.ts:71-75`; `grasp/GraspPreview.tsx:10-11` |
| C3 | Magenta highlight colours (`#ff4fd8` selected, `#ff4dd2` grasp path) add two more hues for selection. | `poi/style.ts:13`; `grasp/GraspPreview.tsx:11` |
| C4 | Several things are coloured that are not state: user chat bubbles in `primary.dark`, the PoiTab filter chips in primary, the phase chip in primary, `armReady` as success-green text, ok usage bars in primary, and the layers toggle in primary when on. | `tabs/AgentChatTab.tsx:199`; `tabs/PoiTab.tsx:67`; `agent/PhasePanel.tsx:76,138`; `tabs/MapNavTab.tsx:688,1180` |
| C5 | Healthy states are painted green: the link 'live', GPS RTK fixed, battery ok, and tool 'ok'. | `overlays/OverlayBar.tsx:85`; `gps/gpsStatus.ts:104`; `battery/batteryStatus.ts:36`; `tabs/AgentChatTab.tsx:81` |

### 1.6 Overlay panels covering the map

| # | Finding | Evidence |
|---|---|---|
| O1 | On tablet and desktop the left column floats over the map. It holds the POI list, POI editor, photo panel and photo list, and the photo list renders even when not asked for. | `tabs/MapNavTab.tsx:993-1045` |
| O2 | The Layers card floats over the map on the right, and the grasp card floats bottom centre. On a 1024 px tablet, three floating panels plus the popup can cover most of the map. | `tabs/MapNavTab.tsx:1069,1063-1067`; `layout/OverlayPanel.tsx:11,46-48` |
| O3 | Also over the map: the grasp pick Alert at top centre, the 'Waiting for map' text, the tile attribution, the action Snackbar, and the MapPopup card with an arrow. | `tabs/MapNavTab.tsx:960-990,1047-1061,1192-1210`; `map3d/MapPopup.tsx` |

### 1.7 Copy glyphs (user writing rule: no em dashes, ellipsis characters or decorative Unicode)

- `'—'` as an empty value: `overlays/OverlayBar.tsx:63`.
- The `'…'` character: `App.tsx:216`, `overlays/OverlayBar.tsx:86`, `tabs/RgbdCameraTab.tsx:96,107,131`, `components/CameraStream.tsx:36`.
- `'●'` and `'○'`: `overlays/OverlayBar.tsx:86`.
- A literal `x` as the close glyph: `map3d/MapPopup.tsx:128`. This should be an icon button.

Noted, not doing now: `tabs/ArmTab.tsx` is not reachable from `App.tsx` renderTab, so it is dead code with a test attached.

## 2. Design spec: "Grafana-like, without colour details"

What it means concretely:

- **Neutral dark surfaces with three levels.** Canvas, panel and raised surface, separated by 1 px borders, not by shadows or blur.
- **One accent colour.** It is used only for focus, the selected tab or toggle, links, and the single primary action in a panel.
- **Colour means state, and only state.** Error, warning and ok. Healthy is shown as neutral text by default (see fork F4). Info is never coloured; it is plain secondary text.
- **Flat panel chrome.**
  - Every panel is one component: a 32 px header with the title in sentence case (12/500, secondary text) and icon actions on the right, a 1 px border, 2 px radius, an opaque panel background, and an 8 px body padding.
- **Values look like instruments.**
  - Numbers use tabular numerals.
  - Raw or technical values (coordinates, voltages, JSON, topic names) use the mono font.
  - A value has a dim label, then the value, then a dim unit. A status is text, not a chip.
- **Compact type scale.** Sizes 11, 12, 13 (body), 14 and 16. Weights are 400 and 500 only; 600 is reserved for the STOP button.
- **8 px grid.** Spacing steps are 0.5, 1, 2, 3 and 4 (4, 8, 16, 24 and 32 px). 1.5 and 1.25 are off-grid and banned.
- **Icon-only toolbars.** Every toolbar control is an IconButton with an aria-label and a Tooltip, except STOP, which keeps a text label. There is no `title=` attribute and no start icons on text buttons.
- **One confirm pattern** (see fork F9). **One banner style**: the standard (not filled) Alert, a full-width strip with a 3 px state-coloured left border. **No toasts that duplicate a banner.**

### 2.1 Theme tokens (proposed exact values)

```ts
export const TOKENS = {
  // surfaces
  bgCanvas:    '#0f1013', // body, map and 3D background (replaces CANVAS_BG and background.default)
  bgPanel:     '#16181d', // panels, app bar, status bar (background.paper)
  bgRaised:    '#1d2026', // hover, selected row, code blocks, input fill
  border:      '#2a2e36', // 1 px panel and divider line
  borderStrong:'#3a3f49', // input outline, focused group
  // text
  textPrimary:  '#d6d9de',
  textSecondary:'#8d939c',
  textDisabled: '#5a6069',
  // the one accent (see fork F3)
  accent:       '#5b8def',
  accentText:   '#0f1013', // text on accent fill
  // state only
  error:        '#e5484d',
  warning:      '#e2a336',
  ok:           '#46a758', // used only if fork F4 = "ok is green"
  // shape and density
  radius: 2,               // px, everything incl. Chip and Alert
  borderWidth: 1,
  spacingUnit: 8,          // MUI default; ban off-grid multiples
  panelHeaderPx: 32,
  controlPx: 32,           // dense control on fine pointers (fork F5)
  touchTargetPx: 44,       // kept on coarse pointers and phone
  // type
  fontSizes: { xs: 11, sm: 12, md: 13, lg: 14, xl: 16 },
  fontWeights: { regular: 400, medium: 500, stop: 600 },
  numeric: 'tabular-nums',
} as const
// Map overlay surface: opaque bgPanel (fork F7 may allow rgba(22, 24, 29, 0.96), no blur).
```

MUI mapping:

- **Palette.** `mode: 'dark'`, `primary.main = accent`. `secondary` is set equal to the text-secondary grey so that it cannot add colour. `error`, `warning` and `success` take the state tokens. `info` is set to `textSecondary`. `background` is `{ default: bgCanvas, paper: bgPanel }`, `divider = border`, and `text` uses the three text tokens.
- **Shape.** `shape.borderRadius = 2`.
- **Typography.**
  - `fontSize: 13`, with `body1 = 13/400` and `body2 = 12/400`, `caption = 11/400` and `subtitle2 = 13/500`.
  - `overline` restyled as `12/500`, `letterSpacing: 0`, `textTransform: 'none'`, so existing call sites calm down at once.
  - `button = 13/500` with `textTransform: 'none'`.
  - `MuiCssBaseline` sets `body { fontVariantNumeric: 'tabular-nums' }`.
- **Component overrides.**
  - Paper `{ backgroundImage: 'none', border: 1px solid border }`, defaultProps `elevation: 0` and `variant: 'outlined'`.
  - Chip `{ borderRadius: 2, height: 20, fontSize: 11 }`, defaultProps `variant: 'outlined', size: 'small'`.
  - Alert defaultProps `variant: 'outlined'`, styled `{ borderRadius: 2, borderLeftWidth: 3 }`.
  - Button `disableElevation`, `minHeight` from a coarse-pointer media query (44, else 32).
  - IconButton the same.
  - Tooltip `{ fontSize: 11, bgcolor: bgRaised, border }`.
  - Tab `{ minHeight: 40, fontWeight: 500 }`, indicator in the accent.
  - LinearProgress `{ height: 2, borderRadius: 0 }`.
  - Snackbar Alert `variant: 'outlined'`.

### 2.2 Component rules

1. **Panel** (rewrite of `layout/OverlayPanel.tsx`).
   - Header: 32 px high, sentence-case title, actions slot, close as an IconButton.
   - Body: `p: 1` (8 px), opaque `bgPanel`, 1 px `border`, radius 2. No blur and no shadow.
   - Phone: a bottom sheet with a square top. The handle is removed unless a real drag is implemented.
2. **StatusItem** (new).
   - Takes `label`, `value`, `unit` and an optional `state: 'ok' | 'warn' | 'error' | 'unknown'`.
   - Renders the label at 11/secondary, the value at 13 mono tabular, and the unit at 11/secondary.
   - Non-ok states colour the value text and add a 6 px square state mark on the left. Ok stays neutral (fork F4).
   - Used by the OverlayBar, battery, GPS rover and base, link, and time budget.
3. **ToolButton** (new). An IconButton plus a Tooltip; the aria-label is the same string as the tooltip. `aria-pressed` marks toggles, and the active state is the accent icon colour on `bgRaised`. Grouped buttons share one 1 px bordered strip.
4. **StateBanner** (one component). This replaces the cut-off banner, TiltBanner and the agent block notice. It is a standard Alert with a left state border, and there is no duplicate toast for a condition that has a banner.
5. **Chips** are allowed only as toggle filters (outlined, radius 2). Status and metadata are rendered as StatusItem or plain text joined by `" - "`.
6. **Map overlays and labels.**
   - One `MapLabel` style: `bgPanel` at 0.9, a 1 px border, 11 px `SANS_FONT`, weight 500, radius 2.
   - A label can carry a state-coloured border only for zones (keepout = error, slow = warning).
   - The MapPopup reuses the Panel header and theme buttons, with no shadow and no arrow (fork).
7. **Helper text.** Explanations go into the Tooltip of the control they explain, or into an `InfoOutlined` ToolButton. Inline prose is allowed only for an actionable error.
8. **STOP** remains the one big red filled button: error fill, 600 weight, no letter spacing, text label. It is the only filled-colour control.

## 3. Design forks only the user can decide

| ID | Fork | Options | Default proposal |
|---|---|---|---|
| F1 | Light theme | (a) dark only; (b) add light mode from the same tokens, following the system setting | (a). The robot is used outdoors on a phone and tablet, which may argue for (b). |
| F2 | Map marker colours | (a) keep semantic hues for POI kinds, zones and nav paths, but from one palette file; (b) greyscale everything except state (keepout red, slow amber, goal accent) and tell kinds apart by shape or dash; (c) keep zones and nav paths coloured, greyscale POI kinds | (c) |
| F3 | Accent colour | (a) Grafana-like blue `#5b8def`; (b) keep the current cyan `#4fc3f7`; (c) neutral white accent (no hue at all) | (a) |
| F4 | Is "ok" coloured green? | (a) neutral text when healthy, colour only warn and error; (b) green ok, as Grafana does | (a) |
| F5 | Density versus touch | (a) 44 px targets everywhere, as today; (b) 44 px on a coarse pointer, 32 px on a fine pointer (desktop dense) | (b). The existing 44 px tests need a pointer split. |
| F6 | Panels on tablet and desktop | (a) keep floating cards over the map; (b) a docked right sidebar that shrinks the map, so nothing covers it | (b) |
| F7 | Overlay translucency | (a) opaque panels; (b) 0.96 alpha, no blur | (a) |
| F8 | Tab bar icons | (a) drop them in the tab bar, keep them in the drawer; (b) drop them everywhere; (c) keep them | (a) |
| F9 | Confirm pattern | (a) two-click arm everywhere (fast, already used for reset, home and release); (b) a Dialog for anything destructive (delete, reset session) and two-click for robot motion | (b) |
| F10 | Grasp stop | (a) one STOP in the toolbar that stops nav and grasp; (b) keep a separate STOP GRASP, but only in the toolbar, removing both panel stops | (b). Merging (a) changes safety behaviour and needs the user's approval. |
| F11 | Chat transcript | (a) keep user and assistant bubbles, neutral colours; (b) a flat log (role label column, no bubbles) like Grafana Explore | (b) |
| F12 | Typeface | (a) system sans plus mono for values (no web font, offline and CSP safe); (b) mono for all data panels | (a) |
| F13 | MapPopup | (a) keep the anchored popup card with an arrow; (b) drop the popup and open the item straight into the side Panel | (a) restyled; (b) if F6 = docked |

## Plan inputs

Default verify for every task, run from `nodes/web_ui/frontend`:

```
npx vitest run <test> && npm run lint && npx tsc --noEmit
```

For the full suite, use `npx vitest run`.

Project rules:

- Lint and test from the repo root as well: `uv run poe lint-nodes` covers the Python side only, so the frontend uses the commands above.
- Then commit, push, and `scripts/deploy-nodes.sh client web_ui` (and `server` if web_ui is deployed there), logged to `.logs/`.

The tasks share `tabs/MapNavTab.tsx`: P3, P6, P8, P9 and P10 touch it. Those must run in sequence, not in parallel.

### P1 - Theme tokens

- **Files:**
  - `theme.ts`
  - `theme.test.ts` (new)
  - `main.tsx`
- **Acceptance:**
  - `theme.ts` exports `TOKENS` with the exact section 2.1 values.
  - In the theme: `shape.borderRadius === 2`, and `palette.secondary.main` equals `TOKENS.textSecondary`.
  - Typography `fontWeight` values are all in {400, 500}, and `overline.textTransform === 'none'`.
  - Overrides set `MuiChip.styleOverrides.root.borderRadius === 2`, `MuiAlert.defaultProps.variant === 'outlined'` and `MuiPaper.defaultProps.elevation === 0`.
  - CssBaseline sets body `fontVariantNumeric: 'tabular-nums'`.
  - `CANVAS_BG === TOKENS.bgCanvas`, and `PANEL_BG` is opaque or 0.96 per F7. Update `layout/hardcodedSizes.test.ts:23` `PANEL_BG_LITERAL` if the literal changes.
- **Test:** `src/theme.test.ts`.

### P2 - Design rules guard (lint-like test)

- **Files:**
  - `layout/designRules.test.ts` (new)
  - `layout/hardcodedSizes.test.ts`
- **Acceptance:** for every non-test source, using the same `import.meta.glob` raw pattern as `hardcodedSizes.test.ts:6-10`:
  - (a) no `#hex`, `rgb(`, `rgba(` or `0x` colour literal outside `theme.ts` and `map3d/palette.ts`;
  - (b) no `boxShadow`, `backdropFilter`, `textShadow` or `gradient`;
  - (c) no `color="secondary"`;
  - (d) no Alert with `variant="filled"`;
  - (e) no `fontWeight` 700 or 800, and 600 only on a line containing `STOP`;
  - (f) no off-grid spacing values 1.25, 0.75, 1.5 or 2.5 in sx spacing keys;
  - (g) no `—`, `…`, `●` or `○` in string literals;
  - (h) no `title=` on a `<Button` or `<IconButton` line.
- **Rollout:** use a per-file `GUARDED_FILES` list, like the existing guard. P3 to P11 each add their files, and P12 flips the guard to cover all files.
- **Test:** `src/layout/designRules.test.ts`.

### P3 - One map palette

- **Files:**
  - `map3d/palette.ts` (new)
  - `map3d/palette.test.ts` (new)
  - `map3d/sceneLayers.tsx`
  - `tabs/MapNavTab.tsx`
  - `poi/style.ts`
  - `grasp/GraspPreview.tsx`
  - `photos/geometry.ts`
  - `components3d/InteractiveArm.tsx`
- **Acceptance:**
  - All map, scene and marker colours are defined once in `palette.ts`. `LAYER_SWATCH` and `MAP_LEGEND` import from it, with no copy.
  - The photos swatch equals the photo marker colour. The POIs swatch equals the point kind colour. There is one `OBJECT_COLOR`.
  - The selection highlight uses `TOKENS.accent`.
  - Hue reduction follows F2.
  - `palette.test.ts` asserts that every swatch equals its scene colour and that the zone colours equal the theme error and warning tokens.
- **Test:** `src/map3d/palette.test.ts`.

### P4 - MapLabel and MapPopup restyle

- **Files:**
  - `map3d/MapLabel.tsx` (new)
  - `map3d/MapLabel.test.tsx` (new)
  - `map3d/sceneLayers.tsx`
  - `poi/PoiLayer.tsx`
  - `grasp/GraspPreview.tsx`
  - `map3d/MapPopup.tsx`
- **Acceptance:**
  - The three inline label styles are replaced by `<MapLabel>`, which uses `SANS_FONT` at 11 px, weight 500, radius 2 and theme tokens.
  - The MapPopup has no `boxShadow` and no raw colours. Its close control is an icon with `aria-label="Close popup"`. Its buttons use theme tokens and stay 44 px on touch, so the `daa1be6` touch test stays green.
- **Test:** `src/map3d/MapLabel.test.tsx`, plus the existing MapPopup touch-target test.

### P5 - Flat Panel chrome

- **Files:**
  - `layout/OverlayPanel.tsx`
  - `layout/layout.test.tsx`
- **Acceptance:**
  - The header is 32 px (44 px on coarse pointer if F5 = b) with a sentence-case title.
  - No `backdropFilter`. Radius 2 and square on phone. The decorative handle is removed (the `overlay-panel-handle` testid is gone).
  - The body padding is `p: 1`, and there is an `actions` prop slot.
  - If F6 = b, a `docked` sidebar mode resizes its sibling instead of floating.
- **Test:** `src/layout/layout.test.tsx`.

### P6 - StatusItem and status bar

- **Files:**
  - `components/StatusItem.tsx` (new)
  - `components/StatusItem.test.tsx` (new)
  - `overlays/OverlayBar.tsx`
  - `components/BatteryChip.tsx`
  - `components/GpsChip.tsx`
  - `components/GpsChip.test.tsx`
  - `agent/TimeBudgetChip.tsx`
  - `agent/TimeBudgetChip.test.tsx`
- **Acceptance:**
  - Battery, GPS rover and base, link and time budget render as StatusItem: no `<Chip`, and no status icons except one optional state mark.
  - Ok renders with `color: text.primary` (F4).
  - The link shows `Live` or `Connecting`, with no glyphs.
  - The empty value is `--`.
  - The tooltips keep their details.
- **Test:** `src/components/StatusItem.test.tsx`.

### P7 - App chrome: banners, tabs, toasts

- **Files:**
  - `App.tsx`
  - `components/TiltBanner.tsx`, renamed or extended to `components/StateBanner.tsx`
  - `components/TiltBanner.test.tsx`
  - `tabs/AgentChatTab.tsx`, for the composer block notice only
- **Acceptance:**
  - One StateBanner component for the cut-off and the tilt, with no `variant="filled"`.
  - The tilt entry Snackbar is removed (the banner covers it).
  - The agent composer shows the block reason as a one-line caption, not a second Alert.
  - Tab icons follow F8.
  - No `…` in `App.tsx`.
- **Test:** `src/components/TiltBanner.test.tsx` (or `StateBanner.test.tsx`).

### P8 - Map toolbar: icon-only

- **Files:**
  - `components/ToolButton.tsx` (new)
  - `components/ToolButton.test.tsx` (new)
  - `tabs/MapNavTab.tsx`
  - `map/toolbarGroups.ts`
  - `map/toolbarGroups.test.ts`
- **Acceptance:**
  - Every toolbar control except STOP (and STOP GRASP per F10) is a ToolButton with an aria-label and a Tooltip. There are no `startIcon` text buttons in the toolbar, no `title=`, and no label morphing.
  - The mode hint ('Tap map to set goal') moves into a single map-corner status line.
  - Goal and POI modes use the accent active state, not `secondary`.
  - STOP uses weight 600 with no letter spacing.
- **Test:** `src/components/ToolButton.test.tsx`, plus `map/toolbarGroups.test.ts`.

### P9 - Layers panel extraction and declutter

- **Files:**
  - `map/LayersPanel.tsx` (new)
  - `map/LayersPanel.test.tsx` (new)
  - `tabs/MapNavTab.tsx`
  - `poi/PoiLegend.tsx`
  - `poi/PoiLegend.test.tsx`
- **Acceptance:**
  - The Layers panel moves out of MapNavTab (about 125 lines).
  - Disabled-layer reasons become row tooltips. The gesture and arm paragraphs are removed, or moved behind one info ToolButton.
  - The legend swatches come from `palette.ts`.
  - No `mt: -0.5` or `ml: 4` hacks.
  - The robot pose line moves to the OverlayBar or is dropped (F-free: it is duplicated data).
- **Test:** `src/map/LayersPanel.test.tsx`.

### P10 - POI, photo and grasp panels

- **Files:**
  - `poi/PoiPanels.tsx`
  - `poi/PoiPanels.test.tsx`
  - `photos/PhotoPanel.tsx`
  - `photos/PhotoPanel.test.tsx`
  - `photos/PhotoList.tsx`
  - `grasp/GraspPanel.tsx`
  - `grasp/GraspPanel.test.tsx`
  - `tabs/MapNavTab.tsx` (to drop the duplicate pick Alert)
- **Acceptance:**
  - The POI status is text, with a state mark only for `open`.
  - Character counters show only within 10% of the limit. The drag hint is removed.
  - One confirm pattern per F9, and one button size (default).
  - The grasp panel has no stop buttons when the toolbar has STOP GRASP (F10). One pick hint only.
  - The Result is shown as compact key-value rows, not stacked Alerts. At most one caption of helper text.
- **Tests:** `src/poi/PoiPanels.test.tsx`, `src/photos/PhotoPanel.test.tsx`, `src/grasp/GraspPanel.test.tsx`.

### P11 - Agent chat header, cards and transcript

- **Files:**
  - `tabs/AgentChatTab.tsx`
  - `agent/AgentHeader.tsx` (new, extracted)
  - `agent/AgentHeader.test.tsx` (new)
  - `agent/PhasePanel.tsx`
  - `agent/PhasePanel.test.tsx`
  - `agent/PhaseProgress.tsx`
  - `agent/PhaseProgress.test.tsx`
- **Acceptance:**
  - The header is one line of StatusItems (model, state, phase n/m, budget, usage), with zero `<Chip`.
  - The busy state is shown once.
  - Complexity is shown only in the plan card, as text.
  - The tool kind is a mono text label with no colour, and the tool status is coloured only on error.
  - The phase status uses the state colour only for failed and skipped.
  - Bubbles follow F11, with no `primary.dark` fill.
  - The PlanCard has no `secondary` border.
- **Test:** `src/agent/AgentHeader.test.tsx`, plus the PhasePanel and PhaseProgress tests.

### P12 - PoiTab and copy glyphs, then guard everything

- **Files:**
  - `tabs/PoiTab.tsx`
  - `tabs/PoiTab.test.tsx`
  - `tabs/RgbdCameraTab.tsx`
  - `components/CameraStream.tsx`
  - `tabs/SensorGraphTab.tsx`
  - `layout/designRules.test.ts`
- **Acceptance:**
  - The kind filter is a ToggleButtonGroup (or outlined radius-2 chips). The swatch is square and comes from `palette.ts`.
  - The `…` glyphs are replaced with `...`.
  - The SensorGraphTab axis and grid colours come from tokens.
  - `designRules.test.ts` runs over all sources with no `GUARDED_FILES` allow-list.
- **Verify:** `npx vitest run && npm run lint && npx tsc --noEmit`.
- **Test:** `src/layout/designRules.test.ts`.

Order: P1, P2, then P3 to P5, then P6 to P11 (with the MapNavTab tasks in sequence: P3, P8, P9, P10), and P12 last.
