---
title: Head tab on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Head tab shows what the D435 sees: colour, depth and the RGB-D overlay."
files: [nodes/web_ui/frontend/src/types.ts, nodes/web_ui/frontend/src/tabs/HeadTab.tsx, nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx, nodes/web_ui/frontend/src/head/RgbdView.tsx, nodes/web_ui/frontend/src/camera/zoomApi.ts, nodes/web_ui/frontend/src/camera/zoomApi.test.ts]
verify: "cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx src/head/rgbd.test.ts && npx tsc --noEmit && npm run lint && cd .. && uv run --frozen pytest tests/test_stale_names_gone.py -q"
depends_on: [web-ui-head-tab-config-on-d4, web-ui-zoom-route-retired]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Head tab on D435

## Requirement

Goal G5: "... Head tab RGB-D view shows D435 data." Design D12.

- `src/types.ts`: head tab fields `depth_topic`, `depth_range_m` (replacing `tof_depth_topic`, `tof_range_m`); `tof_amplitude_topic` and the `zoom_*` fields removed.
- `src/tabs/HeadTab.tsx`: RGB preview of `rgb_camera_topic`, depth preview of `depth_topic` (16UC1 mm, colour range `depth_range_m`), RGB-D view toggle on the `/head_rgbd` pair (`head/RgbdView.tsx`, range `depth_range_m`). The amplitude preview and toggle are removed, and so are click-to-zoom with the zoom factor control and its error text. Doc comments say D435. Pan/tilt controls are unchanged.
- `src/camera/zoomApi.ts` and `src/camera/zoomApi.test.ts` deleted (feature retired, Q1).
- `src/head/RgbdView.tsx`: prop names follow `depth_range_m`.
- `src/tabs/HeadTab.test.tsx`: the amplitude and zoom cases are no longer accurate and are removed. New cases: the depth preview subscribes `depth_topic`; no amplitude or zoom control is rendered; the RGB-D view uses `depth_range_m`.
Keep `nodes/web_ui/tests/test_stale_names_gone.py` green: topic names come from config, never as literals in .ts/.tsx (it forbids `image_rect`).

## Acceptance conditions

- AC1: the Head tab previews the configured D435 depth topic with depth_range_m [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx -t "depth preview subscribes depth_topic"]
- AC2: no amplitude or zoom control is rendered [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx -t "no amplitude or zoom control"]
- AC3: the zoom client is no longer tracked [evidence: command:test -z "$(git ls-files nodes/web_ui/frontend/src/camera/zoomApi.ts)"]
- AC4: type check, lint and the stale-names guard pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint && cd .. && uv run --frozen pytest tests/test_stale_names_gone.py -q]

## Files

- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/head/RgbdView.tsx`
- `nodes/web_ui/frontend/src/camera/zoomApi.ts`
- `nodes/web_ui/frontend/src/camera/zoomApi.test.ts`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
