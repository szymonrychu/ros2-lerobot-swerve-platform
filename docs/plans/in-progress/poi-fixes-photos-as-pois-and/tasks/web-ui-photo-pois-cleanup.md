---
title: Web UI photo POIs cleanup
status: done
repo: ros2-lerobot-sverve-platform
value: The Clear POIs dialog can clear photos and each photo shows once in the POI tab
files: [nodes/web_ui/frontend/src/poi/style.test.ts, nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx, nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/style.test.ts && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-reset-clears-pois]
owns: []
uses: [poi-list-photo-entries, poi-clear-photo-kind]
model: ""
estimate: 55
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--web-ui-photo-pois-cleanup
branch: poi-fixes-photos-as-pois-and--web-ui-photo-pois-cleanup
verified_head: 8e15e0becbc20f4cf991086631c1b79a74ae6bb8
run: poi-fixes-photos-as-pois-and
provides: []
acs_for_audit: []
process_gaps: ["The implementer's report names only photos/api.test.ts, but the task's diff also includes the ClearPoisDialog, PoiPanels, api/types, MapNavTab and README changes (earlier commits). The report understates the change set.", "TDD skipped without a listed exception; prove the tests catch the change - commit first, then for each changed non-test file: git show <merge-base>:<path> > <path>; run the test (must fail); git show HEAD:<path> > <path>; run again (must pass); paste both tails into tests.red and verification.evidence"]
---

# Web UI photo POIs cleanup

## Requirement

D2, D7, D9 (frontend): photo entries in `/poi/list` never show twice, and the Clear POIs dialog can clear photos and reports both counts.

- `nodes/web_ui/frontend/src/poi/style.ts` `parsePoiList` already drops kind "photo" through `isPoi`; pin it in `style.test.ts` (a `/poi/list` payload with photo entries, including one with null x/y, parses to the non-photo POIs only; photos keep coming from `/api/photos`, so the map and the POI tab show each photo once).
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`: `CLEARABLE_KINDS` adds "photo" (checked by default, so "clear all" includes photos); new prop `photos: Photo[]` (from the photo list the POI tab already has) so the preview counts matching photos by creator and status like POIs (keep zones never keeps a photo); the preview reads "Removes N POIs and M photos"; after confirming, the result line shows both counts from the response data (`pois`, `photos`), falling back to `removed`.
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`: pass the photos to `ClearPoisDialog`.
- Tests first, seen failing, in `ClearPoisDialog.test.tsx` (clearPois mocked): photo is a selectable kind and is checked by default; the preview counts photos by creator; unchecking photo sends kinds without "photo"; the result shows "Removed 3 POIs and 2 photos"; keep zones does not keep photos in the preview.
- `nodes/web_ui/README.md`: the Clear POIs entry no longer says photos are never cleared; it describes the photo kind and the counts.

## Acceptance conditions

- a1: photo entries in /poi/list are kept out of the POI list [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts -t photo]
- a2: the Clear POIs dialog selects photos, previews and reports POI and photo counts [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx]
- a3: POI panel tests, types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx && npx tsc --noEmit && npm run lint]
- a4: README documents clearing photos [evidence: command: grep -qi 'clear pois.*photo' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/frontend/src/poi/style.test.ts`
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx`
- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- `nodes/web_ui/frontend/src/poi/ClearPoisDialog.tsx:ClearPoisDialogProps.photos: Photo[]`; `CLEARABLE_KINDS` with "photo".
- Uses `clearPois` from `poi/api.ts` unchanged and the clear result data `pois`/`photos` (part `poi-clear-photo-kind`).

## Expected outcome

The Clear POIs dialog previews and clears photos together with POIs and reports both counts; photos from `/poi/list` never appear as duplicate POIs.

## Steps

- Write the tests in `ClearPoisDialog.test.tsx` and `style.test.ts`; run `cd nodes/web_ui/frontend && npx vitest run src/poi/ClearPoisDialog.test.tsx src/poi/style.test.ts`; see the dialog tests fail.
- Implement the photo kind, the photos prop and the counts; pass photos from `PoiPanels.tsx`; rerun; run `npx tsc --noEmit && npm run lint`.
- Update the README; commit `feat(web_ui): Clear POIs dialog clears photos`.

## Notes

- `isPoi` in `style.ts` already rejects kind "photo"; this task only pins it with a test.
- The photo list refreshes by itself: deleting photos moves the `/poi/photos` revision, which refetches `/api/photos`.
- Built on the driving-agent PoiPanels changes; do not redo its category chips or marking.
