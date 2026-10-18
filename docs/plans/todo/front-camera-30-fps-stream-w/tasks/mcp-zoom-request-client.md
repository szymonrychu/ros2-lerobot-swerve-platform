---
title: MCP zoom request client
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

- a1: a zoom request publishes the D4 JSON and pairs the result with its image by stamp_ns in either arrival order [evidence: test: nodes/mcp_server/tests/test_zoom_client.py]
- a2: non-ok statuses, timeouts and a missing node raise distinct errors [evidence: test: nodes/mcp_server/tests/test_zoom_client.py]
- a3: config defaults for the zoom topics and the 8 s timeout [evidence: test: nodes/mcp_server/tests/test_config.py]
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
