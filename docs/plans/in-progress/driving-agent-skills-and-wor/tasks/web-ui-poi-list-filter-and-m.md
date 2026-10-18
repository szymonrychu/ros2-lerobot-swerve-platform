---
title: Web UI POI list filter and mark
status: done
repo: ros2-lerobot-sverve-platform
value: "In the POI tab the user filters the list by category, marks POIs one by one, or marks everything visible."
files: [nodes/web_ui/frontend/src/poi/PoiListToolbar.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-poi-selection-logic, web-ui-poi-category-chips]
owns: []
uses: [poi-selection]
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--web-ui-poi-list-filter-and-m
branch: driving-agent-skills-and-wor--web-ui-poi-list-filter-and-m
verified_head: 9db2c43926c0d802ada0677202101b3544d77c4e
run: driving-agent-skills-and-wor
provides: []
acs_for_audit: []
process_gaps: []
---

# Web UI POI list filter and mark

## Requirement

- New `PoiListToolbar.tsx` (U10 b): category filter chips (one per category in `CATEGORIES` plus `uncategorised`, each with the live count of POIs in it; toggling selects/deselects, none selected = all shown), a "Mark all visible" button, a "Clear marks" button, and the marked count. Pure view: state is passed in by props.
- `PoiPanels.tsx`: holds the filter set and the marked set (`useState`); the list shows only `filterPois(...)` rows; every row gets a checkbox bound to `toggleMarked`; the filter change runs `pruneMarked`, and a POI deleted elsewhere is pruned too; "Mark all visible" sets `markAllVisible(visible)` (marks every POI passing the CURRENT filter, including ones scrolled out of view, none outside it). The selection is local UI state, never sent anywhere here.
- `nodes/web_ui/README.md`: the POI section documents the category filter and marking.
- Tests first in new `PoiPanels.mark.test.tsx` (leave `PoiPanels.test.tsx` untouched, it must keep passing), seen failing: filtering by one category hides the others and shows per-category counts; clicking a row checkbox marks only that POI; "Mark all visible" with a filter active marks exactly the visible POIs; changing the filter unmarks POIs that became hidden; "Clear marks" empties the marks.

## Acceptance conditions

- a1: the toolbar filters the list by category and shows per-category counts [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t filter]
- a2: rows can be marked individually and Mark all visible marks exactly the visible POIs [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t 'mark']
- a3: a filter change unmarks hidden POIs [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t prune]
- a4: existing POI panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint]
- a5: README documents the filter and marking [evidence: command: grep -qi 'mark all visible' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/frontend/src/poi/PoiListToolbar.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- `poi/PoiListToolbar.tsx:PoiListToolbar` props (filter, counts, marked count, callbacks); `PoiPanels` marked-set state.

## Expected outcome

The POI tab filters by category and holds a set of marked POIs.

## Steps

- Write `PoiPanels.mark.test.tsx`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): POI list filter and marking`.

## Notes

- PoiPanels.tsx is also edited by web-ui-poi-category-chips (chips), web-ui-clear-pois-dialog (named-maps epic: Clear POIs button) and web-ui-poi-delete-marked; depends_on orders them with the chips task, the others rebase and keep both.
- Servo safety (CLAUDE.md): this task moves no servo.
