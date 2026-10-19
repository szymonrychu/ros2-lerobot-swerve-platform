---
title: Web UI RGBD calib proxy
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Calibrate page reaches the robot calibration node through web_ui, and the IMU section is unaffected."
files: [nodes/web_ui/web_ui/rgbd_calibration.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/imu_calibration.py, nodes/web_ui/web_ui/server.py, nodes/web_ui/tests/test_rgbd_calibration_proxy.py, nodes/web_ui/tests/test_imu_calibration.py, nodes/web_ui/README.md]
verify: "(cd nodes/web_ui && uv run --frozen pytest tests/test_rgbd_calibration_proxy.py tests/test_imu_calibration.py tests/test_config.py -q && uv run --frozen poe lint)"
depends_on: [calib-node-http-api]
owns: [rgbd-calib-web-api]
uses: [rgbd-calib-node-http-api]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI RGBD calib proxy

## Requirement

web_ui forwards the RGB-D calibration calls to the node; the IMU section is unaffected (design D9).

- `nodes/web_ui/web_ui/config.py` `CalibrationConfig`: `rgbd_url: str | None = None` (an `http://` or `https://` URL; absent means no RGB-D section) and `rgbd_timeout_s: float = Field(10.0, gt=0)`.
- `nodes/web_ui/web_ui/imu_calibration.py` `get_config` answers `{"imu": True, "rgbd": <rgbd_url is set>, "arm": "deferred"}`.
- New `nodes/web_ui/web_ui/rgbd_calibration.py` `register_rgbd_calibration_routes(app, config)`: one route `/api/calibration/rgbd/{path:path}` for GET, POST, DELETE forwarding method, query and body to `<rgbd_url>/api/<path>` with `httpx.AsyncClient` (timeout `rgbd_timeout_s`; client lifecycle tied to the app); status code, `content-type`, `content-disposition` and body bytes pass through unchanged (JPEG thumbnails and zip downloads included); a path not matching `^[A-Za-z0-9_.\-/]+$` or holding a `..` segment answers 400; no `rgbd_url` answers 404 `{ok: false, message: "RGB-D calibration not configured"}`; connect errors and timeouts answer 503 `{ok: false, message: "calibration node unreachable"}`. Registered in `server.py:build_app` before the SPA mount. Pattern: `agent_proxy.py`.
- New `nodes/web_ui/tests/test_rgbd_calibration_proxy.py` (httpx `MockTransport` or a patched client): config flag on and off; GET, POST, DELETE forwarded with path, query and body; binary bodies and headers passed through; traversal rejected; unreachable gives 503; unconfigured gives 404. `tests/test_imu_calibration.py` asserts the new `rgbd` flag and every existing IMU test passes unchanged otherwise.
- `nodes/web_ui/README.md` Calibrate section documents `rgbd_url`, `rgbd_timeout_s` and the forwarded routes, and lists the new test (doc file).

## Acceptance conditions

- a1: /api/calibration/config reports the rgbd flag from rgbd_url [evidence: test:nodes/web_ui/tests/test_imu_calibration.py::test_config_reports_rgbd_flag]
- a2: GET, POST and DELETE are forwarded with path, query and body [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_forwards_method_path_query_body]
- a3: JPEG and zip bodies pass through with their headers [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_binary_bodies_and_headers_pass_through]
- a4: traversal paths are rejected [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_rejects_traversal_path]
- a5: an unreachable node answers 503 with a message [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_unreachable_node_is_503]
- a6: without rgbd_url the routes answer 404 [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_not_configured_is_404]
- a7: the whole web_ui backend suite and lint pass (IMU unchanged) [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/rgbd_calibration.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/imu_calibration.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_rgbd_calibration_proxy.py`
- `nodes/web_ui/tests/test_imu_calibration.py`
- `nodes/web_ui/README.md`

## Interfaces

- `nodes/web_ui/web_ui/rgbd_calibration.py::register_rgbd_calibration_routes(app: FastAPI, config: AppConfig) -> None`.
- `nodes/web_ui/web_ui/config.py::CalibrationConfig.rgbd_url`, `.rgbd_timeout_s`.
- Contract `rgbd-calib-web-api` (owned here): `/api/calibration/rgbd/<rest>` maps to node `/api/<rest>`; `/api/calibration/config` carries `rgbd`.

## Expected outcome

With `rgbd_url` set the page's RGB-D calls reach the node and binary bodies arrive intact; IMU calls behave exactly as before.

## Steps

- Write `tests/test_rgbd_calibration_proxy.py` and the `rgbd` flag test; run: red.
- Implement config, flag, proxy module, registration; update the README.
- Run the verify command: all pass, lint clean.

## Notes

- Pattern: `nodes/web_ui/web_ui/agent_proxy.py` (httpx, JSON error bodies). httpx is already a web_ui dependency.
- web_ui subscribes nothing for this section (design D1).
