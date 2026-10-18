---
title: Web UI zoom click and API
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI zoom click and API

## Requirement

- `camera/zoomApi.ts`: `requestZoom(tab: string, u: number, v: number, zoom: 2 | 4 | 8): Promise<ZoomReply>` POSTing to `/api/camera/zoom?tab=` (the `poi/api.ts` fetch pattern); `ZoomReply` = the endpoint body; non-2xx rejects with an Error carrying the status and the result message. `clickToNormalised(clientX, clientY, rect, naturalWidth, naturalHeight): {u, v} | null` maps a click inside an `object-fit: contain` image (letterbox aware) to [0, 1] coordinates and returns null for clicks on the letterbox bars.
- `CameraStream.tsx`: optional prop `onPointSelect?: (u: number, v: number) => void`; when set the image shows a crosshair cursor and clicks call it with `clickToNormalised`; without it nothing changes.
- `types.ts`: head tab `zoom_command_topic?`, `zoom_result_topic?`, `zoom_image_topic?`, `zoom_timeout_s?`.
- Tests first, seen failing: `zoomApi.test.ts` (request body and URL, ok reply, 409/502/504 rejections, letterbox maths: centre, corners, bars return null, wide and tall containers); `CameraStream.test.tsx` (click calls onPointSelect with the mapped point; no handler, no cursor change).

## Acceptance conditions

- a1: requestZoom posts {u, v, zoom} and rejects on non-2xx with the status [evidence: test: nodes/web_ui/frontend/src/camera/zoomApi.test.ts]
- a2: clicks map to normalised image coordinates, ignoring letterbox bars [evidence: test: nodes/web_ui/frontend/src/camera/zoomApi.test.ts]
- a3: CameraStream reports clicks only when onPointSelect is set [evidence: test: nodes/web_ui/frontend/src/components/CameraStream.test.tsx]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/camera/zoomApi.ts`
- `nodes/web_ui/frontend/src/camera/zoomApi.test.ts`
- `nodes/web_ui/frontend/src/components/CameraStream.tsx`
- `nodes/web_ui/frontend/src/components/CameraStream.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
