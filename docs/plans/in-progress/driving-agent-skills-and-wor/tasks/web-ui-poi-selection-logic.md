---
title: Web UI POI selection logic
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The POI list can be filtered by category and individually marked through small tested pure functions.
files: [nodes/web_ui/frontend/src/poi/selection.ts, nodes/web_ui/frontend/src/poi/selection.test.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-poi-category-chips]
owns: [poi-selection]
uses: [poi-category-tags]
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI POI selection logic

## Requirement

- New `poi/selection.ts` (U10 b), pure and typed, built on `parsePoiTags` from `poi/categories.ts`. Constant `UNCATEGORISED = "uncategorised"`. `poiCategory(poi: Poi): string`: the tag category when the note carries one, else `keepout` when `poi.zone === "keepout"`, else `slow` when `poi.zone === "slow"`, else `UNCATEGORISED` (user POIs without tags stay filterable).
- `filterPois(pois: Poi[], categories: ReadonlySet<string>): Poi[]` (an empty set means no filter: every POI passes); `toggleMarked(marked: ReadonlySet<string>, id: string): Set<string>`; `markAllVisible(visible: Poi[]): Set<string>` (exactly the ids passing the current filter, nothing else); `pruneMarked(marked: ReadonlySet<string>, visible: Poi[]): Set<string>` (marks never outlive the filter: a marked POI that is no longer visible, or no longer exists, is unmarked, so "delete marked" can only hit POIs the user can see); `summariseMarked(pois: Poi[], marked: ReadonlySet<string>): { count: number; byCategory: Record<string, number>; keepouts: Poi[] }` where `keepouts` are marked POIs with `zone === "keepout"` (zone decides, not the tag, so an untagged keep-out is still caught).
- Tests first in new `selection.test.ts`, seen failing before the module exists.

## Acceptance conditions

- a1: poiCategory resolves the tag category, falls back to the zone, else uncategorised [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t poiCategory]
- a2: filterPois filters by any selected category and an empty set passes everything [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t filterPois]
- a3: markAllVisible marks exactly the POIs passing the filter and pruneMarked drops hidden or vanished marks [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t 'markAllVisible|pruneMarked']
- a4: summariseMarked counts per category and lists marked keep-outs by zone even without a tag [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts -t summariseMarked]
- a5: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/poi/selection.ts`
- `nodes/web_ui/frontend/src/poi/selection.test.ts`

## Interfaces

- `poi/selection.ts`: `UNCATEGORISED`, `poiCategory`, `filterPois`, `toggleMarked`, `markAllVisible`, `pruneMarked`, `summariseMarked`.

## Expected outcome

The filter and marking rules are fixed in one tested module the UI components consume.

## Steps

- Write `selection.test.ts`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/selection.test.ts`; expect failures (module missing).
- Implement the smallest module that passes; run the verify command; commit `feat(web_ui): POI selection logic`.

## Notes

- The category set equals `CATEGORIES` in `poi/categories.ts` plus `UNCATEGORISED`; do not redefine the grammar.
- Servo safety (CLAUDE.md): this task moves no servo.
