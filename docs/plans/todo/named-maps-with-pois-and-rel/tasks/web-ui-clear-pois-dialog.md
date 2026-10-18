---
title: Web UI clear POIs dialog
status: todo
repo: ros2-lerobot-sverve-platform
value: "The user clears POIs from the POI panel in one step, choosing whose and which kinds, with zones kept unless they opt in."
files: [nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx, nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx, nodes/web_ui/frontend/src/poi/api.ts, nodes/web_ui/frontend/src/poi/PoiPanels.tsx]
verify: "cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-poi-clear-route]
owns: []
uses: []
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI clear POIs dialog

## Requirement

"Clear POIs" in the POI panel (design D7, D12).

- `api.ts`: `clearPois(tab, {createdBy, kinds, statuses, keepZones, ids})` posting `op: clear` (the same function the skills epic's "delete marked" calls with `ids`; this dialog never sends `ids` and never implements per-POI marking, category filtering or a second deletion UI).
- `ClearPoisDialog`: creator choice (all / mine (user) / agent's), kinds checkboxes (point, area, object; all
  checked by default), "Keep keep-out and slow zones" (checked by default), a count preview computed from the current
  POI list with the same filter rules, confirm button disabled when the preview is 0, the result message
  ("Removed n POIs") or the error.
- The dialog is a thin "clear all / clear by filter" action. Optional prop `onChooseIndividually?: () => void`: when given, the dialog shows a "Choose individual POIs" link that closes it and calls it (the skills epic's POI list passes it to focus its filter/mark mode; absent, the link is not rendered).
- `PoiPanels.tsx` gets the "Clear POIs" button that opens the dialog.
- `ClearPoisDialog.test.tsx` (test first): preview counts per filter (zones kept by default), posted body per choice,
  disabled confirm at 0, error display, the link renders only when `onChooseIndividually` is given; `PoiPanels.test.tsx` still passes.

## Acceptance conditions

- a1: the dialog previews the count, posts the chosen filters and keeps zones by default [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx]
- a2: the POI panel tests still pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx]
- a3: types and lint are clean [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/api.ts`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`

## Interfaces

- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`; `clearPois` in `src/poi/api.ts`.

## Expected outcome

The POI panel clears POIs with a previewed count and zones kept by default.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint`; expect every test passed and lint clean.
- Commit: `feat(web_ui): clear POIs dialog`.

## Notes

Shared-operation contract (`poi-clear-filters`) is owned by poi-store-clear-filters; epic driving-agent-skills-and-wor reuses `clearPois` and the route for its POI list (U10) - keep `clearPois` the single API function.

The preview applies the same rules as poi_store `op_clear` (creator, kinds, keep_zones) to the current POI list.
- Servo safety (CLAUDE.md): this task moves no servo.
