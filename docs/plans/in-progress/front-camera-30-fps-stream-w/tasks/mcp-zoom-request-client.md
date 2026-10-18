---
title: MCP zoom request client
status: done
repo: ros2-lerobot-sverve-platform
value: mcp_server can ask the head camera for a zoom crop and gets the image or a precise error.
files: [nodes/mcp_server/mcp_server/zoom_client.py, nodes/mcp_server/tests/test_zoom_client.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/tests/test_config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/models.py, nodes/mcp_server/mcp_server/tool_context.py, nodes/mcp_server/tests/fakes.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-head-camera-topic, zoom-request-json]
owns: [mcp-camera-zoom]
uses: [mcp-head-camera-topic, zoom-json-contract]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--mcp-zoom-request-client
branch: front-camera-30-fps-stream-w--mcp-zoom-request-client
verified_head: 4e161ce8c097cae65ea931228a1ac61627e9016e
run: front-camera-30-fps-stream-w
provides: [zoom-request-client]
acs_for_audit: []
process_gaps: []
---

# MCP zoom request client

## Requirement

- `zoom_client.py` (no ROS, the `poi_client.PoiRequests` pattern): `ZoomRequests(publish, node_running, timeout_s, image_grace_s, new_id)`; `request(u, v, zoom, max_px, af=True) -> ZoomReply` publishes the D4 command JSON with a fresh `request_id`, waits for the matching `/head_camera/zoom/result` (`on_result(payload)`) and, for `ok`/`af_failed`, for the image whose header stamp equals the result's `stamp_ns` (`on_image(stamp_ns, jpeg)`, keeps the last 4 images; the image may arrive before or after the result, waiting at most `image_grace_s` after the result). Errors (subclasses of `RobotError`): `ZoomNodeDown` (no subscriber on the command topic), `ZoomTimeout` (no result within timeout_s, or no image within the grace), `ZoomFailed(status, message)` for `busy`, `timeout`, `camera_error`, `invalid`. Malformed results and unknown ids are dropped.
- `config.py`: `topics.head_camera_zoom_command` `/head_camera/zoom/command`, `topics.head_camera_zoom_result` `/head_camera/zoom/result`, `topics.head_camera_zoom_image` `/head_camera/zoom/image`; `timeouts.zoom_timeout_s` 8.0 (> 0), `timeouts.zoom_image_grace_s` 1.0 (> 0).
- `models.py`: `CameraZoomFrame` (camera, topic, jpeg excluded, width, height, stamp_s, status, crop, full, out, af_state, lens_position, phase_ms, request: u, v, zoom).
- `ros_iface.py`: command publisher (COMMAND_QOS), result subscription (COMMAND_QOS) and image subscription (reliable, depth 2) created at startup; `camera_zoom(camera, u, v, zoom, max_px) -> CameraZoomFrame` (only `front`, else `RobotError`); the image JPEG is returned as published (already <= max_px) with its decoded width and height.
- `tool_context.py`: `camera_zoom` on the robot Protocol; `tests/fakes.py`: the fake robot implements it (records calls, returns a scripted frame or raises a scripted error).
- Tests first in `tests/test_zoom_client.py` (and config defaults in `tests/test_config.py`), seen failing: the published JSON has exactly the D4 request keys; result then image and image then result both resolve; af_failed returns the image; busy/timeout/camera_error/invalid raise `ZoomFailed` with the status; no result -> `ZoomTimeout`; ok result without image within the grace -> `ZoomTimeout`; no subscriber -> `ZoomNodeDown` without publishing; unknown ids and malformed payloads ignored; concurrent requests resolve independently.

## Acceptance conditions

- a1: a zoom request publishes the D4 JSON and pairs the result with its image by stamp_ns in either arrival order [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_zoom_client.py -q]
- a2: non-ok statuses, timeouts and a missing node raise distinct errors [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_zoom_client.py -q]
- a3: config defaults for the zoom topics and the 8 s timeout [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py -q]
- a4: the robot Protocol and the fake robot expose camera_zoom [evidence: symbol: camera_zoom]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/zoom_client.py`
- `nodes/mcp_server/tests/test_zoom_client.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/tests/fakes.py`

## Interfaces

- `mcp-camera-zoom`: `camera_zoom(camera: str, u: float, v: float, zoom: int, max_px: int) -> CameraZoomFrame` on the Protocol in `nodes/mcp_server/mcp_server/tool_context.py`, implemented in `ros_iface.py` and `tests/fakes.py`.
- `zoom_client.ZoomRequests`, `ZoomNodeDown`, `ZoomTimeout`, `ZoomFailed`; `models.CameraZoomFrame`.

## Expected outcome

mcp_server holds a tested zoom client wired to the head camera topics; the tool parameter is added next.

## Steps

- Write `tests/test_zoom_client.py` and the config tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_zoom_client.py tests/test_config.py -q`; see them fail.
- Implement zoom_client.py, config, models, ros_iface, Protocol and fake; run the verify command; commit `feat(mcp_server): head camera zoom request client`.

## Notes

- JSON contract: `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`.
- Reuse `poi_client.PoiRequests` structure (lock, pending dict, events).
- Never subscribe or destroy subscriptions from tool threads (README on-demand note); these subscriptions live for the node's lifetime.
