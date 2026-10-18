---
title: Web UI delete marked photos
status: done
repo: ros2-lerobot-sverve-platform
value: Delete marked in the POI list removes marked photos too
files: [nodes/web_ui/frontend/src/poi/PoiPanels.tsx, nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx, nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-photo-pois-cleanup]
owns: []
uses: [poi-clear-photo-kind]
model: ""
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--web-ui-delete-marked-photos
branch: see git branch of worktree
verified_head: eddfa6bce59c2af4b24cd8766629b53cdc9a4dd5
run: poi-fixes-photos-as-pois-and
provides: []
acs_for_audit: []
process_gaps: []
---

# Web UI delete marked photos

## Requirement

D7, D9 (frontend): delete-marked deletes marked photos too, on the existing `clearPois` ids path.

- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`: photo rows in the POI list can be marked like POI rows (the mark set holds the photo id; ids are unique across POIs and photos because both are uuid4 hex).
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`: new prop `photos` (or the marked photo items) so the summary reads "Delete N POIs and M photos?" with the per-category list for the POIs; Confirm posts every marked id (POIs and photos) through the existing `clearPois(tab, { ids })` (no new API, route or poi_store op); the result line shows both counts from the response data. The keep-out consent rule is unchanged.
- Tests first, seen failing: `DeleteMarkedDialog.test.tsx`: marked photos are counted in the summary and their ids are posted with the POI ids; the result line shows both counts; only photos marked still enables Confirm. `PoiPanels.mark.test.tsx`: a photo row can be marked and counts in "Delete marked (N)".
- `nodes/web_ui/README.md`: delete marked covers photos.

## Acceptance conditions

- a1: marked photos are summarised and their ids posted with the POI ids, and both counts are reported [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx -t photo]
- a2: photo rows can be marked in the POI list [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.mark.test.tsx -t photo]
- a3: existing delete-marked and panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint]
- a4: README says delete marked covers photos [evidence: command: grep -qi 'delete marked.*photo' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx`
- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.mark.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- `nodes/web_ui/frontend/src/poi/DeleteMarkedDialog.tsx` props gain the marked photos (shape left to the builder).
- Uses `clearPois(tab, { ids })` unchanged (part `poi-clear-photo-kind`).

## Expected outcome

Marking photos in the POI list and pressing Delete marked removes them with the marked POIs and shows both counts.

## Steps

- Write the photo cases in `DeleteMarkedDialog.test.tsx` and `PoiPanels.mark.test.tsx`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/DeleteMarkedDialog.test.tsx src/poi/PoiPanels.mark.test.tsx`; see them fail.
- Allow marking photo rows and include them in the dialog; rerun; run `npx tsc --noEmit && npm run lint`.
- Update the README; commit `feat(web_ui): delete marked covers photos`.

## Notes

- Do not add an API function, route or poi_store op: the ids path already deletes photos after poi-cleanups-delete-photos.
- The keep-out consent checkbox is unchanged.
