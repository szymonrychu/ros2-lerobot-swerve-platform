---
title: Web UI maps dialog
status: todo
repo: ros2-lerobot-sverve-platform
value: "From the Map tab the user names and saves a map, sees saved maps with their POI counts, loads or deletes one, and sees whether the robot is localised."
files: [nodes/web_ui/frontend/src/tabs/MapNavTab.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx, nodes/web_ui/frontend/src/map/MapsDialog.tsx, nodes/web_ui/frontend/src/map/MapsDialog.test.tsx, nodes/web_ui/frontend/src/map/mapsApi.ts, nodes/web_ui/frontend/src/map/MapStatusLine.tsx, nodes/web_ui/frontend/src/types.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx src/tabs/MapNavTab.toolbar.test.tsx && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-named-maps-backend]
owns: []
uses: []
model: sonnet
estimate: 85
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI maps dialog

## Requirement

Map tab UI for named maps (design D12).

- `mapsApi.ts`: typed calls for `GET /api/maps`, `POST /api/maps/save|load|delete` with the tab id; types for the
  status (maps, active map, load_seq, localisation) in `types.ts`.
- Toolbar: "Save map as" opens a name dialog (client-side check of the name rule with the message; existing name asks
  "Overwrite <name>?" and resends with `overwrite: true`; optional note); "Maps" opens `MapsDialog`. "Save map" and
  "Reset map" keep their current behaviour.
- `MapsDialog`: lists maps (name, saved time, POI count, invalid ones marked with their error and no Load button),
  Load (confirmation text: the current map and POIs are replaced; an autosave is kept), Delete (confirmation that names the map and its POI count: "Delete <name> and its N POIs?", U7), a busy
  state while a request runs, the result message, and for the active map in state `not_localised`/`ambiguous` a
  "Retry at saved pose" action (load again with the saved pose).
- `MapStatusLine` shows the active map and the localisation state ("Localised on <map>", "Not localised on <map>:
  <reason>"); nothing is shown before a status arrives. Status reaches the page by polling `GET /api/maps` at most
  every 5 s while the Map tab is visible, or a WS topic if the builder adds one.
- `MapsDialog.test.tsx` (test first) and updated toolbar tests: save-as with overwrite flow, list rendering including an
  invalid map, load and delete confirmations post the right bodies, retry posts the saved pose, status line text for
  each state and nothing before the first status.

- Explicit Save (Save map as), Load and Delete buttons (U7). After an unrecoverable move (state `not_localised` with message "map discarded") the status line says "Map cleared after the robot was moved; load a saved map" and the dialog stays usable; saved maps are still listed.

## Acceptance conditions

- a1: save-as, overwrite, load, delete and retry post the right requests [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx]
- a2: the toolbar keeps Save map and Reset map and adds Save map as and Maps [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx]
- a3: types and lint are clean [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: Delete asks with the POI count, and the status line text after a discarded map is shown with saved maps still listed [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx -t 'delete confirmation|discarded']

## Files

- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`
- `nodes/web_ui/frontend/src/map/MapsDialog.tsx`
- `nodes/web_ui/frontend/src/map/MapsDialog.test.tsx`
- `nodes/web_ui/frontend/src/map/mapsApi.ts`
- `nodes/web_ui/frontend/src/map/MapStatusLine.tsx`
- `nodes/web_ui/frontend/src/types.ts`

## Interfaces

- `nodes/web_ui/frontend/src/map/mapsApi.ts`, `MapsDialog.tsx`; status types in `src/types.ts`.

## Expected outcome

The Map tab offers Save map as and Maps (load, delete, retry) and shows the localisation state.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx src/tabs/MapNavTab.toolbar.test.tsx && npx tsc --noEmit && npm run lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): named maps dialog in the Map tab`.

## Notes

Use the existing `runAction`, ToolButton and dialog patterns of `MapNavTab.tsx`; keep the toolbar groups test (`toolbarGroups.ts`) consistent.
- Servo safety (CLAUDE.md): this task moves no servo.
