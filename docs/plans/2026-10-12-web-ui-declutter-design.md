# Web UI declutter design

## Summary

A utilitarian, professional pass over the whole web UI, in the style of Grafana but without its colour detail:

- a dense, neutral dark theme with a white accent
- colour only for warning and error state
- flat, opaque panel chrome with thin borders
- one status line instead of repeated chips
- an icon-only map toolbar
- panels docked beside the map on tablet and desktop
- a flat log for the agent chat

Brief with evidence: docs/plans/2026-10-12-web-ui-declutter-briefs/declutter-brief.md.

## Intent and why

The user: "Declutter + deslopify. Make it utilitarian and professional. Like Grafana, but without color details."

The audit (brief section 1) found:
- status shown several times (three grasp stops, battery cut-off three times, about 10 agent header chips)
- about 25 saturated hues, with raw hex in 13 files
- disagreeing colour tables
- translucent blurred panels and shadows
- six button styles, three confirm patterns and font weights up to 800
- up to four overlays covering the map

## Goals

- One theme token set (brief 2.1) in theme.ts, with the accent white or neutral and no hue. State colours exist only
  for warning and error. Healthy or ok state is neutral text, never green.
- A guard test fails on:
  - a raw hex, rgb or hsl colour outside theme.ts and the one map palette file
  - box-shadow, backdrop-filter or gradients
  - font weights other than 400 and 500
  - banned glyphs (em dash, ellipsis character, arrows, decorative Unicode, emoji) in UI copy
- Map colour (D2): keepout red, slow amber and nav paths keep their colour. POI kinds are greyscale and told apart by
  shape. Everything else is neutral. All of it comes from one palette file. This supersedes the per-kind fill colours
  of the previous round.
- Panels (D3):
  - tablet and desktop: a docked right sidebar that shrinks the map, so nothing covers it
  - phones: keep the bottom sheets
  - the map popup is replaced by opening the item in the sidebar (bottom sheet on a phone); centring is kept
- One status line (StatusItem) replaces the duplicated chips. Each fact is shown once: battery, tilt, GPS, agent busy
  state, control owner.
- Map toolbar: icon-only buttons with tooltips, one button style, with the overflow menu kept. One STOP and one STOP
  GRASP, both in the toolbar; both grasp-panel stops are removed. There is no change to safety behaviour.
- The agent chat becomes a flat log with a role column and no bubbles. The header shows at most: state, phase
  dropdown, time left.
- Confirms: a Dialog for destructive actions (delete, reset session), and the two-click arm for robot motion.
- Copy:
  - helper text cut to what an operator needs
  - no em dashes, ellipsis characters or decorative Unicode
  - tab bar without icons (the drawer keeps them)
- Type: system sans, with mono and tabular numerals for values. Weights 400 and 500. The 8 px grid and radius 2.
- Touch: 44 px targets on a coarse pointer, 32 px on a fine pointer.
- Panels are opaque, with no translucency or blur.
- Every task has its tests; npm test, lint and build stay green.

## Non-goals

- No light theme, which the user did not ask for (my call, default F1 a).
- No change to robot behaviour, safety logic or backend APIs.
- No new UI library.

## Anti-goals

- Losing information an operator needs. Deduplication removes only repeats, never the last display of a fact.
- A stop button that becomes harder to reach.
- Map gestures regressing.

## Constraints

- Tests first.
- MUI 6 theme and `sx`.
- The previous round's tests (layout classes, OverlayPanel, PhasePanel test ids) are updated, not deleted, where
  behaviour moves.
- The deploy covers web_ui only.

## Decisions

### D1: Neutral theme tokens with a white accent; colour for warning and error only

The user chose a white accent and no green ok. The token values come from brief 2.1, with the accent replaced by a
neutral white or light grey. The guard test enforces it.

Serves B1.

### D2: Map: only zones and paths are coloured; POI kinds greyscale by shape

This is the user's choice: one palette file, keepout and slow zone colours, and a nav path colour. Kinds are told
apart by marker shape. The previous round's kind fills are removed.

Serves B2.

### D3: Docked sidebar on tablet and desktop; bottom sheets on phones; no map popup

This is the user's choice. The sidebar shrinks the map canvas, and its resize keeps the frameloop on demand. Show on
map centres the POI and opens it in the sidebar. MapPopup is removed and its tests are migrated (F13 b follows from
F6 b).

Serves B3.

### D4: One status line, one stop pair, flat chat log, one confirm pattern (my calls)

These are the brief defaults for F5, F7, F8, F9, F11 and F12. The user chose F10 b: STOP plus STOP GRASP, toolbar only.

Serves B4.

## Research

| lens | ran or skipped | adopted | rejected |
|---|---|---|---|
| research-code-internal | ran (declutter-brief.md) | inventory, tokens, component rules, plan inputs | - |
| research-code-external | skipped: the user named Grafana as the reference; no open fork | - | - |
| research-papers | skipped: visual design work | - | - |
| research-docs | skipped: MUI 6 theme API used as in existing code | - | - |

## Left to the builder

- The exact grey steps within the brief's token ranges.
- The marker shapes per POI kind, as long as they are distinguishable at phone zoom.

## Assumptions and unknowns

- tabs/ArmTab.tsx may be dead code after round 3. Its fate is settled at plan time against main.

## Irreversible operations in scope

None.
