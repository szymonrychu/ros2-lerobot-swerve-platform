---
title: Web UI ground frontend
status: done
repo: ros2-lerobot-sverve-platform
value: "The map shows floor vs obstacle labels, the fitted plane and step/drop-off edges, so the user can validate the estimate by eye."
files: [nodes/web_ui/frontend/src/map3d/groundLayer.ts, nodes/web_ui/frontend/src/map3d/groundLayer.test.ts, nodes/web_ui/frontend/src/map3d/sceneLayers.tsx, nodes/web_ui/frontend/src/map3d/layers.ts, nodes/web_ui/frontend/src/map/LayersPanel.tsx, nodes/web_ui/frontend/src/map/LayersPanel.test.tsx, nodes/web_ui/frontend/src/tabs/MapNavTab.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx vitest run src/map3d/groundLayer.test.ts src/map/LayersPanel.test.tsx && npx tsc --noEmit && npm run lint && npm run build"
depends_on: [web-ui-ground-backend]
model: sonnet
review: ""
verified_head: 175066d79225a8e8f24bae4be246291a4a2bccbd
points: 8
expected_minutes: 75
expected_tokens: 6000000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Web UI ground frontend

## Requirement

- web_ui frontend (design D10): a "Ground" map layer, off by default, subscribed on demand (only while enabled) to the `ground_cloud` and `ground_state` roles.
- `src/map3d/groundLayer.ts` (pure, unit-tested): decode the ground cloud payload (reuse `cloudDecode.ts` for points/colours, decode `labels_b64`); `labelColour(label)` palette: floor green, obstacle red, drop blue, floor_colour_mismatch orange, unlabelled grey; `planePatch(state)` -> the corners of a 2 m x 2 m square around base_link on the state plane (from `plane.normal` and `plane.height_m`, in base_link) or null when `plane` is null; `groundVisible(state)` -> false for an absent, stale (`stale: true`) or plane-null state; `warningPolygons(state)` -> the warning area polygons (base_link) with kind and source; `regionEdges(state)` -> line segments (2 m long, centred on the edge point along the direction) per region with kind for colour.
- `src/map3d/sceneLayers.tsx`: render the labelled points (PointSet, coloured per label, placed by the cloud frame pose like the Head RGB-D layer), the translucent plane patch, the edge lines (red for class obstacle, yellow for warning) and the warning area polygons (translucent yellow, "slow down") placed by the base_link pose; nothing drawn from the plane, edges or read-out when the state is absent, carries `"stale": true` (set by the backend from its receive age, web-ui-ground-backend) or has `plane: null`; the labelled points are hidden with the state when it is stale.
- `src/map/LayersPanel.tsx`: the "Ground" toggle with a legend of the five label colours, the obstacle/warning edge colours and the warning area fill and the slope / slope_level / plane height read-out from the state; `src/map3d/layers.ts` registers the layer id; `src/tabs/MapNavTab.tsx` wires the roles; the payload and state types live in `groundLayer.ts` (state keys equal STATE_KEYS of the ground-state contract).
- Tests first, seen failing: `src/map3d/groundLayer.test.ts` (label decode, palette, plane patch for a level and a 5 deg tilted plane, null plane, region edge segments with class colours, warning polygons, `groundVisible(state)` false for absent, `stale: true` and plane-null states); LayersPanel test for the toggle and legend (extend `src/map/LayersPanel.test.tsx`).
- `nodes/web_ui/README.md`: the Ground layer (what the colours mean, on demand, data source).

## Acceptance conditions

- a1: ground cloud labels decode and map to the five legend colours; plane patch, class-coloured region edges and warning polygons are computed from the state [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts]
- a2: the Ground toggle, legend and slope read-out are in the layers panel [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map/LayersPanel.test.tsx]
- a3: the frontend type-checks, lints and builds [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && npm run build]
- a4: groundVisible is false for absent, backend-stale and plane-null states, so nothing is drawn [evidence: test: nodes/web_ui/frontend/src/map3d/groundLayer.test.ts]

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

`groundLayer.ts` `decodeGroundCloud`, `labelColour`, `planePatch`, `regionEdges`; layer id `ground`; consumes part `web-ground-roles`, `ground-state`.

## Expected outcome

`cd nodes/web_ui/frontend && npx vitest run src/map3d/groundLayer.test.ts src/map/LayersPanel.test.tsx && npx tsc --noEmit && npm run lint && npm run build` passes.

## Steps

- write groundLayer.test.ts and the LayersPanel test
- run vitest, see FAIL
- implement groundLayer.ts and the wiring until green
- run the full frontend checks
- update the README
- commit `feat(web_ui): ground layer on the map`.

## Notes

pattern: commit 044c6aae (Head RGB-D cloud layer: layers.ts, sceneLayers.tsx, LayersPanel.tsx, MapNavTab.tsx, obstaclePoints.ts, cloudDecode.ts).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on web-ui-ground-backend
