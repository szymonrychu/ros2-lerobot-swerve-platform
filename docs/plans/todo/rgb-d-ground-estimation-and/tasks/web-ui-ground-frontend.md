---
title: Web UI ground frontend
status: todo
repo: ros2-lerobot-sverve-platform
value: "The map shows floor vs obstacle labels, the fitted plane and step/drop-off edges, so the user can validate the estimate by eye."
files: [nodes/web_ui/frontend/src/map3d/groundLayer.ts, nodes/web_ui/frontend/src/map3d/groundLayer.test.ts, nodes/web_ui/frontend/src/map3d/sceneLayers.tsx, nodes/web_ui/frontend/src/map3d/layers.ts, nodes/web_ui/frontend/src/map/LayersPanel.tsx, nodes/web_ui/frontend/src/map/LayersPanel.test.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/map3d/groundLayer.test.ts src/map/LayersPanel.test.tsx && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [web-ui-ground-backend]
owns: []
uses: [web-ground-roles, ground-state]
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

# Web UI ground frontend

## Requirement

- web_ui frontend (design D10): a "Ground" map layer, off by default, subscribed on demand (only while enabled) to the `ground_cloud` and `ground_state` roles.
- `src/map3d/groundLayer.ts` (pure, unit-tested): decode the ground cloud payload (reuse `cloudDecode.ts` for points/colours, decode `labels_b64`); `labelColour(label)` palette: floor green, obstacle red, drop blue, floor_colour_mismatch orange, unlabelled grey; `planePatch(state)` -> the corners of a 2 m x 2 m square around base_link on the state plane (from `plane.normal` and `plane.height_m`, in base_link) or null when `plane` is null; `regionEdges(state)` -> line segments (2 m long, centred on the edge point along the direction) per region with kind for colour.
- `src/map3d/sceneLayers.tsx`: render the labelled points (PointSet, coloured per label, placed by the cloud frame pose like the Head RGB-D layer), the translucent plane patch and the edge lines placed by the base_link pose; nothing drawn from a stale or absent state (state older than 2 s by its `stamp_ns` vs the client clock offset the map already uses, or plane null).
- `src/map/LayersPanel.tsx`: the "Ground" toggle with a legend of the five label colours and the slope / slope_level / plane height read-out from the state; `src/map3d/layers.ts` registers the layer id; `src/tabs/MapNavTab.tsx` wires the roles; the payload and state types live in `groundLayer.ts` (state keys equal STATE_KEYS of the ground-state contract).
- Tests first, seen failing: `src/map3d/groundLayer.test.ts` (label decode, palette, plane patch for a level and a 5 deg tilted plane, null plane, region edge segments, stale state); LayersPanel test for the toggle and legend (extend `src/map/LayersPanel.test.tsx`).
- `nodes/web_ui/README.md`: the Ground layer (what the colours mean, on demand, data source).

**Interfaces**: `groundLayer.ts` `decodeGroundCloud`, `labelColour`, `planePatch`, `regionEdges`; layer id `ground`; consumes part `web-ground-roles`, `ground-state`.

**Expected outcome**: `cd nodes/web_ui/frontend && npx vitest run src/map3d/groundLayer.test.ts src/map/LayersPanel.test.tsx && npx tsc --noEmit && npm run lint && npm run build` passes.

**Steps**: write groundLayer.test.ts and the LayersPanel test; run vitest, see FAIL; implement groundLayer.ts and the wiring until green; run the full frontend checks; update the README; commit `feat(web_ui): ground layer on the map`.

**Notes**: pattern: commit 044c6aae (Head RGB-D cloud layer: layers.ts, sceneLayers.tsx, LayersPanel.tsx, MapNavTab.tsx, obstaclePoints.ts, cloudDecode.ts).

## Acceptance conditions

- a1: ground cloud labels decode and map to the five legend colours; plane patch and region edges are computed from the state [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts]
- a2: the Ground toggle, legend and slope read-out are in the layers panel [evidence: test: nodes/web_ui/frontend/src/map/LayersPanel.test.tsx]
- a3: the frontend type-checks, lints and builds [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]
- a4: the layer draws nothing from a stale or plane-less state [evidence: judgement]

## Files

- `nodes/web_ui/frontend/src/map3d/groundLayer.ts`
- `nodes/web_ui/frontend/src/map3d/groundLayer.test.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map/LayersPanel.tsx`
- `nodes/web_ui/frontend/src/map/LayersPanel.test.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
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
