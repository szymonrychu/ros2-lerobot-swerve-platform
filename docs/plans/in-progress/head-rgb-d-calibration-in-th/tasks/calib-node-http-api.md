---
title: Calib node HTTP API
status: todo
repo: ros2-lerobot-sverve-platform
value: "web_ui, and through it the operator, drives capture, sessions, compute, apply and download over one localhost API."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/app.py, nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py, nodes/head_rgbd_calibration/tests/test_app.py, nodes/head_rgbd_calibration/tests/test_main.py, nodes/head_rgbd_calibration/README.md]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run --frozen poe lint)"
depends_on: [calib-node-live-capture, calib-node-compute-job, calib-node-apply-download]
owns: [rgbd-calib-node-http-api]
uses: [rgbd-calib-session-store, rgbd-calib-result-files, rgbd-calib-applied-files, rgbd-calib-node-config]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-node-http-api
branch: head-rgb-d-calibration-in-th--calib-node-http-api
verified_head: ""
run: head-rgb-d-calibration-in-th
provides: []
acs_for_audit: []
process_gaps: []
---

# Calib node HTTP API

## Requirement

The node's localhost HTTP API and process entry point (design D1, D5, Contracts table "Node HTTP API").

- `head_rgbd_calibration/app.py` `build_app(capture, job, store, cfg, run) -> FastAPI` with exactly the routes of the design's Node HTTP API table; every error body is `{ok: false, message}`; `InvalidId`/`NotFound` give 404, `Busy`/`NotApplicable` give 409; capture start is refused while compute runs and compute while capture runs (409 naming the other job); compute is refused when the session has fewer poses than `limits.min_views` (409 naming both counts); thumbnails served as `image/jpeg`; download as `application/zip` with `Content-Disposition: attachment; filename="<sid>-<rid>.zip"`; `GET /api/applied` 404 when nothing is applied; `GET /api/sessions/{sid}` includes `coverage` from the store; `GET /api/state` carries `applied.json` only (no systemctl call), per-unit restart state appears only on `GET /api/applied`.
- `head_rgbd_calibration/__main__.py`: loads the config from `HEAD_RGBD_CALIBRATION_CONFIG`; `rclpy.init`; one node `head_rgbd_calibration`; `MultiThreadedExecutor` spinning in a daemon thread; uvicorn bound to `cfg.http_host:cfg.http_port` (127.0.0.1 by default, never 0.0.0.0); on shutdown stops any capture, kills a running compute child and destroys the node.
- Tests `tests/test_app.py` (TestClient with fake capture, job and a tmp store) and `tests/test_main.py` (bind host and port come from the config; shutdown stops capture).
- README `## HTTP API` section listing every route (doc file).

## Acceptance conditions

- a1: GET /api/state reports capture, compute and applied [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_state_shape -q]
- a2: capture and compute exclude each other with 409 [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_capture_and_compute_exclusive -q]
- a3: compute needs limits.min_views poses [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_compute_needs_min_views -q]
- a4: unknown and traversal ids answer 404 [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_bad_ids_are_404 -q]
- a5: thumbnails and the zip are served with their content types [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_binary_routes_content_types -q]
- a6: applying a failing result answers 409 [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_apply_failing_result_409 -q]
- a7: the server binds the configured localhost address and shutdown stops the capture [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_main.py -q]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]
- a9: the README lists the HTTP API [evidence: doc-section:nodes/head_rgbd_calibration/README.md#HTTP API]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/README.md`

## Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py::build_app(capture, job, store, cfg, run) -> FastAPI`; routes exactly as the design's Node HTTP API table.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py::main() -> None`.
- Contract `rgbd-calib-node-http-api` (owned here; consumed by web-ui-rgbd-calib-proxy and calib-node-ansible-deploy).

## Expected outcome

`python3 -m head_rgbd_calibration` serves the API on 127.0.0.1:18110 and spins the ROS node; the whole node suite passes.

## Steps

- Write `tests/test_app.py` and `tests/test_main.py`; run: red.
- Implement the app and the entry point; document the routes.
- Run the verify command: all pass, lint clean.

## Notes

- Error body shape `{ok: false, message}` matches web_ui's `imu_calibration.error`.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-live-capture, calib-node-compute-job, calib-node-apply-download
