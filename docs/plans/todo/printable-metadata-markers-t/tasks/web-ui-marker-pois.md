---
title: Web UI marker POIs
status: todo
repo: ros2-lerobot-sverve-platform
value: "The person sees detected markers on the map with their facing and details, and can rename or delete them."
files: [nodes/web_ui/frontend/src/poi/types.ts, nodes/web_ui/frontend/src/poi/style.ts, nodes/web_ui/frontend/src/poi/style.test.ts, nodes/web_ui/frontend/src/poi/PoiLayer.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.tsx, nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx, nodes/web_ui/frontend/src/poi/PoiLegend.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [poi-marker-model]
owns: []
uses: [poi-marker-contract]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI marker POIs

## Requirement

- `types.ts`: `PoiKind` adds `'marker'`; `Poi` gains `z?`, `orientation?: [number, number, number, number] | null`, `marker?: MarkerInfo | null` (kind, id, key, size_mm, params, payload, version, pose_samples); `isPointLike('marker')` true.
- `style.ts`: `KIND_SHAPE.marker = 'hexagon'` (new MarkerShape with 6 sides), its colour, `facingYaw(poi)` (yaw of +z projected on the floor, null when within 45 deg of vertical; same rule as mcp_server); the kind filter at line ~153 accepts markers.
- `PoiLayer.tsx`: markers drawn with the hexagon and a facing arrow when facingYaw is not null; not draggable.
- `PoiPanels.tsx`: a marker's panel shows marker kind, id, size (mm), height z (m), times seen, last seen, confidence; name, note and status editable; delete allowed; no position or radius editor.
- `PoiLegend.tsx`: a Marker legend entry.
- Tests first (`style.test.ts`, `PoiPanels.test.tsx`), seen failing: KIND_SHAPE and sides for marker; facingYaw for a wall marker facing +y (90 deg) and null for a floor marker; the marker panel renders the marker fields and has no radius/position inputs; editing the name sends only name; the legend lists Marker.
- `nodes/web_ui/README.md`: marker POIs on the map and in the panels.

## Acceptance conditions

- a1: marker POIs have their own shape and a facing direction [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts]
- a2: the marker panel shows marker details, edits name, note and status only [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx]
- a3: the frontend typechecks, lints and builds [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]
- a4: the web_ui README mentions marker POIs [evidence: command: grep -qi 'marker POI' nodes/web_ui/README.md]

## Files

- `nodes/web_ui/frontend/src/poi/types.ts`
- `nodes/web_ui/frontend/src/poi/style.ts`
- `nodes/web_ui/frontend/src/poi/style.test.ts`
- `nodes/web_ui/frontend/src/poi/PoiLayer.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiLegend.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- `nodes/web_ui/frontend/src/poi/types.ts:Poi` marker fields; `nodes/web_ui/frontend/src/poi/style.ts:facingYaw`.

## Expected outcome

Markers show on the web UI map with facing arrows and details.

## Steps

- Write the tests; run `cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts src/poi/PoiPanels.test.tsx`; see them fail.
- Implement types, style, layer, panels, legend; run the verify command.
- Update the README; commit `feat(web_ui): show marker POIs`.

## Notes

- The UI never moves a marker: the detector owns its pose (design D13).
