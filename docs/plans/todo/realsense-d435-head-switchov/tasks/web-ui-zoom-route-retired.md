---
title: Web UI zoom route retired
status: todo
repo: ros2-lerobot-sverve-platform
value: The web UI drops a zoom path no camera serves any more.
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/web_ui/__main__.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_camera_zoom.py, nodes/web_ui/tests/test_server.py]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible"
depends_on: [web-ui-arm-tab-config-withou]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Web UI zoom route retired

## Requirement

Goal G9: head_camera_cpp "retired ... with their consumers". Design D11, Q1 (retire zoom everywhere; amended before it runs if the gate answers otherwise).

- `web_ui/server.py`: `POST /api/camera/zoom` and its request model are removed.
- `web_ui/bridge.py`: zoom publishers, pending futures and `zoom_request_async` are removed. `web_ui/__main__.py` stops passing `zoom_topics`.
- `web_ui/config.py`: head tab `zoom_command_topic`, `zoom_result_topic`, `zoom_image_topic`, `zoom_timeout_s`, the zoom constant and `zoom_topics()` are removed. `config/default.yaml` and the client.yml head tab drop the zoom lines.
- `tests/test_camera_zoom.py` deleted (the feature is retired). New `tests/test_server.py::test_camera_zoom_route_is_gone` (404/405) and `tests/test_server.py::test_head_tab_rejects_zoom_keys` (config validation through the app factory).

## Acceptance conditions

- AC1: POST /api/camera/zoom no longer exists [evidence: test:nodes/web_ui/tests/test_server.py::test_camera_zoom_route_is_gone]
- AC2: head tab zoom keys are rejected [evidence: test:nodes/web_ui/tests/test_server.py::test_head_tab_rejects_zoom_keys]
- AC3: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/__main__.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_camera_zoom.py`
- `nodes/web_ui/tests/test_server.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the two tests; run them; expect failures.
- Remove the route, bridge code, config fields and the zoom test file, update default.yaml and client.yml; rerun the web_ui suite; expect pass.
- Run the verify command; commit `refactor(web_ui): retire the head camera zoom route`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: zoom key test lives in test_server.py (in the task files)
