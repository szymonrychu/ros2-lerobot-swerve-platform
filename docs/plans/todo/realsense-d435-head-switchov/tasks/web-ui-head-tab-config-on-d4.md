---
title: Web UI head tab config on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "The Head tab backend serves D435 colour, depth and RGB-D data."
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/msg_serializer.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_config.py, nodes/web_ui/tests/test_msg_serializer.py, nodes/web_ui/tests/test_head_rgbd.py]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run poe test-ansible"
depends_on: [ground-recorder-on-d435-topi, d435-live-bring-up-and-topic]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Web UI head tab config on D435

## Requirement

Goal G5: "Every web_ui 3D view feature (... Head tab RGB-D view) shows D435 data." Design D12: "Head tab: D435 colour, a depth preview of /head/d435/depth/image_rect_raw (on demand, depth_topic/depth_range_m), and the RGB-D view on /head_rgbd/*. Amplitude and click-to-zoom are removed."

Backend half (frontend in head-tab-on-d435, zoom route in web-ui-zoom-route-retired):
- `web_ui/config.py` head tab model: `tof_depth_topic` renamed `depth_topic`, `tof_range_m` renamed `depth_range_m` (default 4.0), and `tof_amplitude_topic` with `HEAD_AMPLITUDE_ROLE` removed (no backward compatibility: old keys are rejected as unknown). Roles keep header-stamp gating for the RGB-D pair.
- `web_ui/msg_serializer.py`: the 16UC1 depth preview takes `depth_range_m`; the ToF wording in docstrings is gone.
- `web_ui/bridge.py`: `head_spec` without the amplitude role; depth role on the renamed field.
- `config/default.yaml` and the client.yml head tab: `rgb_camera_topic: /head/d435/color/image_raw/compressed`, `depth_topic: /head/d435/depth/image_rect_raw`, `rgbd_rgb_topic: /head_rgbd/rgb/image_raw/compressed`, `rgbd_depth_topic: /head_rgbd/depth/image_raw`, `depth_range_m: 4.0`. The comment block says D435.
- Tests: `tests/test_config.py` (renamed keys, rejected old keys), `tests/test_msg_serializer.py` (depth preview with depth_range_m), `tests/test_head_rgbd.py` (RGB-D pair roles unchanged; any ToF naming updated). Assertions on the old names are no longer accurate and are replaced.

## Acceptance conditions

- AC1: head tab config uses depth_topic and depth_range_m and rejects the ToF keys [evidence: test:nodes/web_ui/tests/test_config.py::test_head_tab_depth_keys_replace_tof_keys]
- AC2: the depth preview renders a 16UC1 frame with depth_range_m [evidence: test:nodes/web_ui/tests/test_msg_serializer.py::test_depth_preview_uses_depth_range_m]
- AC3: the deployed head tab reads the D435 colour, D435 depth and the /head_rgbd pair [evidence: test:nodes/web_ui/tests/test_config.py::test_client_head_tab_reads_the_d435]
- AC4: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/msg_serializer.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_msg_serializer.py`
- `nodes/web_ui/tests/test_head_rgbd.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the new tests and update stale ones; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py tests/test_msg_serializer.py tests/test_head_rgbd.py -q`; expect failures.
- Rename fields, drop amplitude, update default.yaml and client.yml; rerun the web_ui suite; expect pass.
- Run the verify command; commit `feat(web_ui): head tab backend on the D435`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
