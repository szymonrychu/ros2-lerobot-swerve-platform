---
title: Web UI POI category chips
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The user sees at a glance which POIs are targets, risky zones, keep-outs, problem spots or survey findings."
files: [nodes/web_ui/frontend/src/poi/categories.ts, nodes/web_ui/frontend/src/poi/categories.test.ts, nodes/web_ui/frontend/src/poi/PoiPanels.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx, nodes/web_ui/frontend/src/poi/style.ts, nodes/web_ui/frontend/src/poi/style.test.ts, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/poi && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-retros-tab, shared-poi-category-tags, mcp-poi-category-args]
owns: []
uses: [poi-category-tags]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI POI category chips

## Requirement

- New `poi/categories.ts` (D4): mirror of the shared tag grammar: `parsePoiTags(note: string): PoiTags` ({category, problemType, survey, surveySpot}), `stripPoiTags(note)`, constants CATEGORIES and PROBLEM_TYPES equal to shared/ros2_common/poi_categories.py (the test reads the Python file text with a regex and compares the tuples, so the two never drift).
- `PoiPanels.tsx`: each POI row/editor shows a category chip (label, colour) and the problem type, a "survey" marker, and the note without the tag line in the text view (the editor keeps editing the full note); `style.ts`: map style per category for point/area POIs without a zone (problem spots stand out; survey spots drawn small and muted).
- `nodes/web_ui/README.md`: the POI section documents category chips and survey markers.
- Tests first in `categories.test.ts`, `PoiPanels.test.tsx`, `style.test.ts`, seen failing: parsing every category and problem type; constants equal the Python tuples; the chip and survey marker render; the tag line is hidden in the text view; styles per category.

## Acceptance conditions

- a1: tags parse like the shared Python grammar and the constants never drift [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/categories.test.ts]
- a2: POI panels show category chips and survey markers and hide the tag line [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx]
- a3: map styles distinguish categories and survey spots [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts]
- a4: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

## Files

- `nodes/web_ui/frontend/src/poi/categories.ts`
- `nodes/web_ui/frontend/src/poi/categories.test.ts`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`
- `nodes/web_ui/frontend/src/poi/style.ts`
- `nodes/web_ui/frontend/src/poi/style.test.ts`
- `nodes/web_ui/README.md`

## Interfaces

- `poi/categories.ts:parsePoiTags`, `stripPoiTags`, `CATEGORIES`, `PROBLEM_TYPES`.

## Expected outcome

Category information is visible in the POI tab and on the map.

## Steps

- Write the tests; run `cd nodes/web_ui/frontend && npx vitest run src/poi`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): POI category chips`.

## Notes

- The grammar owner is shared/ros2_common/poi_categories.py; this file mirrors it.
