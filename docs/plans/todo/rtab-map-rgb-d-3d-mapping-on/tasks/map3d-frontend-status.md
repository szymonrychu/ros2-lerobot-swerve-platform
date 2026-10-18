---
title: Map3d frontend status
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Map tab shows whether the 3D map is alive, aligned and how big it is, and the result of a 3D save or reset."
files: [nodes/web_ui/frontend/src/map/Map3dStatus.tsx, nodes/web_ui/frontend/src/map/Map3dStatus.test.tsx, nodes/web_ui/frontend/src/map/LayersPanel.tsx, nodes/web_ui/frontend/src/map/LayersPanel.test.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [map3d-frontend-layers, web-ui-map3d-save-reset, map3d-manager-status]
owns: []
uses: [map3d-frontend-layers, map3d-status-contract]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map3d frontend status

## Requirement

- `Map3dStatus.tsx`: renders the latest `/map3d/status` JSON (map3d status contract): state (`mapping`, `paused`, `waiting_for_inputs`, `error`), node count, database size MB, process memory MB, last update age, RTAB-Map graph correction (`graph_correction_m`, `graph_correction_deg`) with a warning colour above the status' `correction_warn_m`/`correction_warn_deg`, and the `missing_inputs` list when waiting. No status received yet shows "no 3D map status yet" (never zeros).
- `LayersPanel.tsx`: the `map3dCloud` row shows the status widget's one-line summary next to its checkbox.
- `MapNavTab.tsx`: the save/reset toolbar actions show the response's `map3d` result (status and message) after the 2D result.
- Tests first, seen failing: `Map3dStatus.test.tsx` (each state, warning colour over the correction threshold, no-status text), `LayersPanel.test.tsx` (summary next to the 3D row), `MapNavTab.toolbar.test.tsx` (save and reset show the 3D result, timeout shown as a warning).

## Acceptance conditions

- a1: status states, the alignment warning and the no-status case render correctly [evidence: test: nodes/web_ui/frontend/src/map/Map3dStatus.test.tsx]
- a2: save and reset show the 3D result after the 2D result [evidence: test: nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx]
- a3: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/map/Map3dStatus.tsx`
- `nodes/web_ui/frontend/src/map/Map3dStatus.test.tsx`
- `nodes/web_ui/frontend/src/map/LayersPanel.tsx`
- `nodes/web_ui/frontend/src/map/LayersPanel.test.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
