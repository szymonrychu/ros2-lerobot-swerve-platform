---
title: Map3d frontend layers
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The Map tab shows the measured 3D map as coloured points and lidar-only walls as clearly different extruded outlines where no depth was seen.
files: [nodes/web_ui/frontend/src/map3d/cloudDecode.ts, nodes/web_ui/frontend/src/map3d/cloudDecode.test.ts, nodes/web_ui/frontend/src/map3d/lidarWalls.ts, nodes/web_ui/frontend/src/map3d/lidarWalls.test.ts, nodes/web_ui/frontend/src/map3d/layers.ts, nodes/web_ui/frontend/src/map3d/layers.test.ts, nodes/web_ui/frontend/src/map3d/sceneLayers.tsx, nodes/web_ui/frontend/src/types.ts]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [web-ui-map3d-cloud-backend]
owns: [map3d-frontend-layers]
uses: [web-ui-map3d-roles]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--map3d-frontend-layers
branch: rtab-map-rgb-d-3d-mapping-on--map3d-frontend-layers
verified_head: ""
run: ""
provides: []
---

# Map3d frontend layers

## Requirement

- `cloudDecode.ts`: `decodeMap3dCloud(payload)` -> `{positions: Float32Array (metres, three.js order after the existing map->scene conversion in coords.ts), colors?: Float32Array, count, voxel}`; rejects (returns null and logs once) a payload whose `frame_id` is not the tab's `map3d_frame_id` (drawn in the map frame, D7) or whose byte lengths do not match `count`.
- `lidarWalls.ts` (U6, D13): `buildLidarWalls(grid, measuredColumns, heightM)` builds wall boxes from occupied cells of the SLAM `/map` payload already in the browser (same occupied threshold as the SLAM map layer), merged per row run, extruded from 0 to `lidar_wall_height_m`, skipping every cell whose column contains a measured 3D point (`measuredColumnsFromCloud(positions, resolution, origin)` hashes cloud x/y into map cells). Pure functions, no three.js in the geometry builder.
- `layers.ts`: new layer keys `map3dCloud` ('3D map (measured)') and `lidarWalls` ('Lidar walls (2D only)'), both visible by default; stored state of older browsers still parses.
- `sceneLayers.tsx`: a points layer (size = voxel, vertex colours when present, a single neutral colour otherwise) and a lidar-walls layer drawn translucent with a distinct hatch-free solid colour from the palette and no vertex colours, so lidar-derived structure is never mistaken for measured points; the cloud layer's visibility drives the on-demand `map3d_cloud` subscription (subscribed only while visible), walls recompute when /map or the cloud changes, at most once per second.
- `types.ts`: payload types for `map3d_cloud`, `map3d_status`, `map3d_result`, TabConfig fields `map3d_*` (including `map3d_frame_id`), `lidar_wall_height_m`.
- Tests first, seen failing: `cloudDecode.test.ts` (round-trip of a payload built in the test, frame mismatch rejected, length mismatch rejected), `lidarWalls.test.ts` (an occupied run becomes one box of the configured height; a column with a measured point is skipped; free and unknown cells give nothing), `layers.test.ts` (new keys, defaults, legacy stored state).

## Acceptance conditions

- a1: cloud payloads decode to scene positions and colours and wrong frames or lengths are rejected [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/cloudDecode.test.ts]
- a2: lidar walls extrude occupied /map cells to the configured height except where measured points exist [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/lidarWalls.test.ts]
- a3: the two new layers exist, default on, and legacy layer state parses [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts]
- a4: measured points and lidar walls are drawn with distinguishable styles [evidence: judgement]
- a5: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/map3d/cloudDecode.ts`
- `nodes/web_ui/frontend/src/map3d/cloudDecode.test.ts`
- `nodes/web_ui/frontend/src/map3d/lidarWalls.ts`
- `nodes/web_ui/frontend/src/map3d/lidarWalls.test.ts`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/types.ts`

## Interfaces

- `nodes/web_ui/frontend/src/map3d/cloudDecode.ts:decodeMap3dCloud`, `nodes/web_ui/frontend/src/map3d/lidarWalls.ts:buildLidarWalls`, `measuredColumnsFromCloud`; layer keys `map3dCloud`, `lidarWalls`.

## Expected outcome

The Map tab renders the room in 3D: coloured measured points, and translucent lidar walls only where the ToF never looked.

## Steps

- Write the three test files; run `cd nodes/web_ui/frontend && npx vitest run src/map3d/cloudDecode.test.ts src/map3d/lidarWalls.test.ts src/map3d/layers.test.ts`; see them fail.
- Implement; run `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`.
- Commit `feat(web_ui): 3D map and lidar wall layers`.

## Notes

- The LayersPanel lists layers from LAYER_LABELS, so no panel edit is needed; check `map/LayersPanel.test.tsx` still passes.
- Keep the point count from the backend cap; no client-side resampling beyond decoding.
