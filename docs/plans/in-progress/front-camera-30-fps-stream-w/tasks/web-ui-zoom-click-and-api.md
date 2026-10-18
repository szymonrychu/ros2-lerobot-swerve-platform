---
title: Web UI zoom click and API
status: escalated
repo: ros2-lerobot-sverve-platform
value: A click on a camera picture becomes an exact zoom point and request from the browser.
files: [nodes/web_ui/frontend/src/camera/zoomApi.ts, nodes/web_ui/frontend/src/camera/zoomApi.test.ts, nodes/web_ui/frontend/src/components/CameraStream.tsx, nodes/web_ui/frontend/src/components/CameraStream.test.tsx, nodes/web_ui/frontend/src/types.ts]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [web-ui-zoom-backend]
owns: [zoom-click-api]
uses: [web-zoom-endpoint]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--web-ui-zoom-click-and-api
branch: front-camera-30-fps-stream-w--web-ui-zoom-click-and-api
verified_head: ""
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Web UI zoom click and API

## Requirement

- `camera/zoomApi.ts`: `requestZoom(tab: string, u: number, v: number, zoom: 2 | 4 | 8): Promise<ZoomReply>` POSTing to `/api/camera/zoom?tab=` (the `poi/api.ts` fetch pattern); `ZoomReply` = the endpoint body; non-2xx rejects with an Error carrying the status and the result message. `clickToNormalised(clientX, clientY, rect, naturalWidth, naturalHeight): {u, v} | null` maps a click inside an `object-fit: contain` image (letterbox aware) to [0, 1] coordinates and returns null for clicks on the letterbox bars.
- `CameraStream.tsx`: optional prop `onPointSelect?: (u: number, v: number) => void`; when set the image shows a crosshair cursor and clicks call it with `clickToNormalised`; without it nothing changes.
- `types.ts`: head tab `zoom_command_topic?`, `zoom_result_topic?`, `zoom_image_topic?`, `zoom_timeout_s?`.
- Tests first, seen failing: `zoomApi.test.ts` (request body and URL, ok reply, 409/502/504 rejections, letterbox maths: centre, corners, bars return null, wide and tall containers); `CameraStream.test.tsx` (click calls onPointSelect with the mapped point; no handler, no cursor change).

## Acceptance conditions

- a1: requestZoom posts {u, v, zoom} and rejects on non-2xx with the status [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/camera/zoomApi.test.ts]
- a2: clicks map to normalised image coordinates, ignoring letterbox bars [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/camera/zoomApi.test.ts]
- a3: CameraStream reports clicks only when onPointSelect is set [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/components/CameraStream.test.tsx]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/camera/zoomApi.ts`
- `nodes/web_ui/frontend/src/camera/zoomApi.test.ts`
- `nodes/web_ui/frontend/src/components/CameraStream.tsx`
- `nodes/web_ui/frontend/src/components/CameraStream.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`

## Interfaces

- `zoom-click-api`: `requestZoom`, `ZoomReply`, `clickToNormalised` in `frontend/src/camera/zoomApi.ts`; `CameraStream` prop `onPointSelect`.

## Expected outcome

The browser can turn a click into a zoom request; the Head tab UI uses it next.

## Steps

- Write the two test files; run `cd nodes/web_ui/frontend && npm test -- zoomApi CameraStream`; see them fail.
- Implement; run the verify command; commit `feat(web_ui): zoom API client and click-to-point`.

## Notes

- Normalised (u, v) must refer to the full 16:9 frame: the preview is not cropped, so the displayed image's natural size is the reference.
