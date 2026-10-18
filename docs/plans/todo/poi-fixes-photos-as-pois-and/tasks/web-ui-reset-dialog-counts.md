---
title: Web UI reset dialog counts
status: todo
repo: ros2-lerobot-sverve-platform
value: The user sees before confirming that Reset map and map delete remove POIs and photos
files: [nodes/web_ui/frontend/src/tabs/MapNavTab.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx, nodes/web_ui/frontend/src/types.ts, nodes/web_ui/frontend/src/map/MapsDialog.tsx, nodes/web_ui/frontend/src/map/MapsDialog.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx src/map/MapsDialog.test.tsx && npx tsc --noEmit && npm run lint"
depends_on: [web-ui-reset-clears-pois]
owns: []
uses: [map-contract-photos-clear-live]
model: ""
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI reset dialog counts

## Requirement

D6, D9 (frontend): the Reset map dialog says what it deletes, and the maps dialog shows photo counts and honest delete results.

- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`: the Reset map `ConfirmDialog` body says: "Discards the current SLAM map and starts a new one, and deletes the live POIs and photos. Saved maps are kept." The action result line shows the server message (which carries the counts).
- `nodes/web_ui/frontend/src/types.ts`: `MapSummary` gains `photo_count?: number`.
- `nodes/web_ui/frontend/src/map/MapsDialog.tsx`: each saved map shows "N POIs, M photos" (M 0 when absent); the delete confirmation says the map's POIs and photos are deleted with it, and that the live POIs and photos go too when it is the active map; the delete result message from map_manager is shown as is.
- Tests first, seen failing: `MapNavTab.toolbar.test.tsx`: the reset dialog text names POIs, photos and saved maps; `MapsDialog.test.tsx`: the photo count is shown (and 0 for an old summary without it), the delete confirmation names POIs and photos and the active-map case.
- `nodes/web_ui/README.md`: the Reset map and maps dialog entries.

## Acceptance conditions

- a1: the reset dialog says live POIs and photos are deleted and saved maps kept [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx -t reset]
- a2: the maps dialog shows photo counts and the delete confirmation names POIs and photos [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/MapsDialog.test.tsx]
- a3: types and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a4: README documents the reset dialog [evidence: command: grep -qi 'saved maps are kept' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/map/MapsDialog.tsx`
- `nodes/web_ui/frontend/src/map/MapsDialog.test.tsx`
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
