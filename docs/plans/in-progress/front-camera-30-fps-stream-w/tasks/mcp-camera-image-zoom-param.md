---
title: MCP camera image zoom param
status: escalated
repo: ros2-lerobot-sverve-platform
value: The agent can zoom into any point of the head camera view with one optional parameter.
files: [nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/camera_tools.py, nodes/mcp_server/tests/test_camera_zoom_tool.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-zoom-request-client]
owns: []
uses: [mcp-camera-zoom]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--mcp-camera-image-zoom-param
branch: front-camera-30-fps-stream-w--mcp-camera-image-zoom-param
verified_head: not obtained
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# MCP camera image zoom param

## Requirement

- `get_camera_image` gains optional `zoom: CameraZoom | None = None` with `CameraZoom` = `{u: float [0, 1], v: float [0, 1], factor: Literal[2, 4, 8]}` (D10, U3, U4); u, v are fractions of the default picture (same 16:9 geometry as the stream, top-left origin). With zoom: only `camera='front'` (else a tool error saying zoom is only available on the front (head) camera, U7); `max_px` caps the crop (default 1024, <= 1024); calls `robot.camera_zoom`; returns the crop JPEG plus JSON metadata (status, crop, full, out, width, height, af_state, lens_position, phase_ms, stamp_s, the request, and a note on `af_failed` that focus did not converge and re-taking is allowed); `ZoomFailed`/`ZoomTimeout`/`ZoomNodeDown` become tool errors naming the status. A zoom crop is never recorded as a lookup (no `record_lookup`, no poi_store call; U6). Without zoom the tool behaves exactly as before (default 384 px, lookup recorded).
- Tool and `camera` field descriptions (tools.py and the camera-name wording in camera_tools.py): `front` is the head RGB camera (Camera Module 3 on the pan/tilt head, 16:9, default picture 384x216), the zoom parameter, its cost (the stream pauses about 1 s, 1-2.5 s per call), one crop per call, crop <= 1024 px. No "overhead"/640x480 wording remains.
- `nodes/mcp_server/README.md`: the `get_camera_image` row documents `zoom` (fields, front only, crop size per factor, metadata, errors, not recorded as a lookup, 8 s timeout).
- Tests first in new `tests/test_camera_zoom_tool.py` (fake robot), seen failing: zoom on front calls camera_zoom with u, v, factor and max_px 1024 by default and returns an image plus the metadata; max_px 512 is passed; zoom on gripper is a tool error and calls nothing; factor 3 and u 1.5 are rejected by validation; af_failed returns the image and the note; busy becomes a tool error naming busy; no lookup recorded for a zoom while a plain call still records one; the plain call is unchanged.

## Acceptance conditions

- a1: get_camera_image(camera='front', zoom={u, v, factor}) returns the crop and its metadata [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q]
- a2: zoom is refused on the gripper camera and invalid factors or points are rejected [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q]
- a3: zoom crops are never recorded as lookups [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q]
- a4: tool descriptions say head camera and no overhead or 640x480 front wording remains [evidence: command: ! git grep -n -i -E 'overhead camera|640x480' -- nodes/mcp_server/mcp_server/tools.py nodes/mcp_server/mcp_server/camera_tools.py]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/tests/test_camera_zoom_tool.py`
- `nodes/mcp_server/README.md`

## Interfaces

- `get_camera_image(camera, max_px=None, zoom: CameraZoom | None = None)` in `nodes/mcp_server/mcp_server/tools.py`; `CameraZoom` model (same file or models.py).

## Expected outcome

The agent-facing tool supports zoom with clear docs; nothing is persisted for zoom crops.

## Steps

- Write `tests/test_camera_zoom_tool.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_zoom_tool.py -q`; see it fail.
- Implement the parameter and descriptions; update README; run the verify command; commit `feat(mcp_server): optional zoom on get_camera_image`.

## Notes

- U4: a parameter, not a new tool (the decision's `zoom_camera_image` is rejected).
- Tool name unchanged, so claude_agent's sensor tool list needs no change.

## Ruling

Accepted by the orchestrator: verified and merged (head_camera_cpp 156 gtests, mcp_server 2533, web_ui 790, frontend 908, root 1296); escalations were verifier crashes, README tab-list nits and refuted parking flags.
