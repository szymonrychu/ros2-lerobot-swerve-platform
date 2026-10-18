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

- `api.ts`: `clearPois(tab, {createdBy, kinds, statuses, keepZones})` posting `op: clear`.
- `ClearPoisDialog`: creator choice (all / mine (user) / agent's), kinds checkboxes (point, area, object; all
  checked by default), "Keep keep-out and slow zones" (checked by default), a count preview computed from the current
  POI list with the same filter rules, confirm button disabled when the preview is 0, the result message
  ("Removed n POIs") or the error.
- `PoiPanels.tsx` gets the "Clear POIs" button that opens the dialog.
- `ClearPoisDialog.test.tsx` (test first): preview counts per filter (zones kept by default), posted body per choice,
  disabled confirm at 0, error display; `PoiPanels.test.tsx` still passes.

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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
