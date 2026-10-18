---
title: Head tab click to zoom
status: done
repo: ros2-lerobot-sverve-platform
value: "The user clicks the Head tab RGB view and sees a sharp full-resolution crop, then the live view again."
files: [nodes/web_ui/frontend/src/tabs/HeadTab.tsx, nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build"
depends_on: [web-ui-zoom-click-and-api]
owns: []
uses: [zoom-click-api]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: front-camera-30-fps-stream-w--head-tab-click-to-zoom
verified_head: 41b73347b6df88feea5aa70b3a06e304cf3ccee9
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: [No red run recorded in tests.red or verification.evidence for the last commit]
---

# Head tab click to zoom

## Requirement

- `HeadTab.tsx` (D12, U5): when the tab has `zoom_command_topic`, the RGB view gets a zoom factor picker (2/4/8, default 4) and passes `onPointSelect` to its `CameraStream`; a click calls `requestZoom(tab.id, u, v, factor)` once, shows a pending state and ignores further clicks until it settles; on success the crop (`jpeg_b64`) replaces the live RGB view with its factor and AF state and a "Back to live" button that returns to the live stream; `af_failed` shows the crop with a "focus not confirmed" note; errors show one short message (busy, timeout, camera error) and the live view stays. Without zoom topics the tab is unchanged. Nothing is stored (no download, no local storage of crops; U6).
- README: Head tab section documents click-to-zoom.
- Tests first in `HeadTab.test.tsx` (mocked `requestZoom`), seen failing: picker default 4; click requests with the clicked point and chosen factor; second click while pending sends nothing; crop shown then "Back to live" restores the stream; af_failed note; error message keeps the live view; no picker without zoom topics; tab design rules still pass.

## Acceptance conditions

- a1: a click on the Head tab RGB view requests one zoom with the chosen factor and shows the crop until Back to live [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a2: while a zoom runs further clicks send nothing, and errors keep the live view [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a3: the web_ui README documents Head tab click-to-zoom [evidence: command: grep -qi 'click-to-zoom' nodes/web_ui/README.md]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

## Files

- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/README.md`

## Interfaces

- Head tab zoom UI in `frontend/src/tabs/HeadTab.tsx` (uses `requestZoom`, `onPointSelect`).

## Expected outcome

Head tab click-to-zoom works one crop at a time and returns to the live stream.

## Steps

- Extend `HeadTab.test.tsx`; run `cd nodes/web_ui/frontend && npm test -- HeadTab`; see it fail.
- Implement; update README; run the verify command; commit `feat(web_ui): Head tab click-to-zoom`.

## Notes

- Respect `layout/designRules` (the tab design tests).
- The ToF depth/amplitude part of the Head tab is untouched.

- 2026-10-18 run front-camera-30-fps-stream-w: dependency-failed on web-ui-zoom-click-and-api

## Ruling

Accepted by the orchestrator: verified and merged (frontend 931, web_ui 825, claude_agent 735, root 1296); head-tab-click-to-zoom no-progress was TDD-evidence only.
