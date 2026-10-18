---
title: Web UI zoom backend
status: todo
repo: ros2-lerobot-sverve-platform
value: The web UI backend can request a head camera zoom and hand the crop to the browser.
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/web_ui/server.py, nodes/web_ui/tests/test_camera_zoom.py, nodes/web_ui/tests/test_bridge.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q"
depends_on: [web-ui-head-camera-frontend, zoom-request-json]
owns: [web-zoom-endpoint]
uses: [web-head-camera-fields, zoom-json-contract]
model: sonnet
estimate: 80
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI zoom backend

## Requirement

- `config.py`: head tab optional `zoom_command_topic`, `zoom_result_topic`, `zoom_image_topic` (all or none, validated) and `zoom_timeout_s` (default 8.0, > 0); `default.yaml` and `ansible/group_vars/client.yml` head tabs set `/head_camera/zoom/command`, `/head_camera/zoom/result`, `/head_camera/zoom/image` (D12, U7: only the head tab).
- `bridge.py`: the type-hint entry `/front_camera/image_raw/compressed` becomes `/head_camera/image_raw/compressed`; for every tab with zoom topics a std_msgs/String command publisher, a String result subscription and a CompressedImage image subscription (reliable; created at startup); `zoom_request_async(tab_id, u, v, zoom) -> Future | None` publishes the D4 command with a fresh `request_id` (max_px 1024, af true) and resolves with `{result, jpeg}` once the matching result and (for `ok`/`af_failed`) the image with header stamp == `stamp_ns` arrived (either order, last 4 images kept); `None` when the tab has no zoom or nothing subscribes the command topic.
- `server.py`: `POST /api/camera/zoom?tab=<id>` with body `{u, v, zoom}` (pydantic: u, v in [0, 1], zoom in 2/4/8) -> 200 `{...result, jpeg_b64}` for ok/af_failed; 404 unknown tab or tab without zoom; 422 bad body; 503 bridge unavailable or head_camera_cpp not running; 409 `busy`; 502 for `timeout`/`camera_error`/`invalid` (body carries the result); 504 after `zoom_timeout_s`. Not blocked by the battery guard (no motion). Nothing is written to disk (U6).
- README: Head tab zoom (endpoint, config keys, statuses).
- Tests first in new `tests/test_camera_zoom.py` (and the hint change in `tests/test_bridge.py`), seen failing: config validation (all-or-none zoom topics); the published command JSON; result-then-image and image-then-result resolve; af_failed returns the image; endpoint status codes 200/404/422/503/409/502/504 with a fake bridge; no file written.

## Acceptance conditions

- a1: POST /api/camera/zoom returns the crop for ok and af_failed and the documented status codes otherwise [evidence: test: nodes/web_ui/tests/test_camera_zoom.py]
- a2: the bridge pairs the zoom result and image by stamp_ns in either order [evidence: test: nodes/web_ui/tests/test_camera_zoom.py]
- a3: only the head tab configures zoom topics in default.yaml and client.yml [evidence: command: grep -q 'zoom_command_topic: /head_camera/zoom/command' nodes/web_ui/config/default.yaml && grep -q 'zoom_command_topic: /head_camera/zoom/command' ansible/group_vars/client.yml]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_camera_zoom.py`
- `nodes/web_ui/tests/test_bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/README.md`

## Interfaces

- `web-zoom-endpoint`: `POST /api/camera/zoom?tab=<id>` body `{u, v, zoom}` -> `{request_id, status, message, af_state, lens_position, crop, full, out, stamp_ns, stamp_s, phase_ms, jpeg_b64}` in `nodes/web_ui/web_ui/server.py`.
- `BridgeNode.zoom_request_async` in bridge.py; head tab `zoom_*` fields in config.py.

## Expected outcome

The web_ui backend serves head camera zoom crops over REST; nothing is broadcast or saved.

## Steps

- Write `tests/test_camera_zoom.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_camera_zoom.py tests/test_bridge.py -q`; see them fail.
- Implement config, bridge, endpoint; edit default.yaml, client.yml, README; run the verify command; commit `feat(web_ui): head camera zoom endpoint`.

## Notes

- Pattern: `poi_request_async` (`bridge.py`) and `POST /api/poi` (`server.py:poi_command`), `await_ros_future`, `action_response`.
- JSON contract: `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`.
