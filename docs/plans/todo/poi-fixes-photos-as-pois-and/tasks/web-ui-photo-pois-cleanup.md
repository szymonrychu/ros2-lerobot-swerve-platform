---
title: Web UI photo POIs cleanup
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
