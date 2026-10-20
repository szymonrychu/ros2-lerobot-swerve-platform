---
title: Web UI ground backend
status: todo
repo: ros2-lerobot-sverve-platform
value: The web UI backend streams the labelled ground cloud and the ground state to the browser on demand.
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/web_ui/cloud_serializer.py, nodes/web_ui/config/default.yaml, nodes/web_ui/tests/test_ground_layer.py, ansible/group_vars/client.yml, nodes/web_ui/README.md]
verify: "(cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && uv run poe test-ansible"
depends_on: [mcp-ground-summary]
owns: [web-ground-roles]
uses: [ground-state, ground-topics, ground-colour-labels]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rgb-d-ground-estimation-and--web-ui-ground-backend
branch: rgb-d-ground-estimation-and--web-ui-ground-backend
verified_head: ""
run: rgb-d-ground-estimation-and
provides: []
acs_for_audit: []
process_gaps: []
---

# Web UI ground backend

## Requirement

- web_ui backend (design D10): map_nav tab keys `ground_cloud_topic` (sensor_msgs/PointCloud2 XYZ+rgb+label of ground_estimator_cpp, on demand) and `ground_state_topic` (std_msgs/String JSON) in `web_ui/config.py` (optional, None = no layer), `config/default.yaml` and the client.yml web_ui map_nav block (`/ground/labelled_points`, `/ground/state`, commented like `rgbd_cloud_topic`).
- Roles `GROUND_CLOUD_ROLE = "ground_cloud"` and `GROUND_STATE_ROLE = "ground_state"` (config.py, part `web-ground-roles`) wired in `bridge.py` `ROLE_SPECS`: ground_cloud with SENSOR_SUB_QOS and `serialize_ground_cloud` (cloud_serializer: the map3d serializer plus a `labels_b64` uint8 array aligned with the kept points after voxel downsampling - the label of the voxel's first point - and `frame`), ground_state with a transient_local reliable QoS (like the tilt_guard state) and `serialize_json_object`. Point cap and voxel as the rgbd cloud (RGBD_CLOUD_MAX_POINTS / RGBD_CLOUD_VOXEL_M or own constants with a comment).
- Staleness by backend receive age, like the tilt guard state (`TILT_STALE_S_DEFAULT`, `tilt_guard_stale_s`, `Bridge.mark_tilt_stale`): config `ground_state_stale_s` (default `GROUND_STALE_S_DEFAULT` = 2.0, > 0) in the map_nav tab config; `Bridge.mark_ground_stale()` replaces the cached ground state by a copy with `"stale": true` once its last receive (monotonic) is older than `ground_state_stale_s`, broadcast like any update and cleared by the next real message; it runs on the same timer as `mark_tilt_stale`. No client clock is involved.
- A cloud without a `label` field is serialized without `labels_b64` (logged once), never with fabricated labels.
- Tests first in `nodes/web_ui/tests/test_ground_layer.py`, seen failing: config keys optional and in default.yaml and client.yml with the ground_estimator_cpp topics (consistency with `nodes/ground_estimator_cpp/config/ground_estimator.yaml` output_namespace); roles derive the message types; a synthetic labelled PointCloud2 (x, y, z, rgb, label uint8) serializes with labels aligned to the kept points after voxel downsampling; missing label field; the state fixture `nodes/ground_estimator_cpp/test/data/state_example.json` passes through serialize_json_object unchanged; mark_ground_stale sets `stale: true` after `ground_state_stale_s` without a message and a new message clears it.
- `nodes/web_ui/README.md`: the two keys and roles.

## Acceptance conditions

- a1: ground_cloud and ground_state roles and their tab keys exist in config, default.yaml and client.yml with the ground_estimator_cpp topics [evidence: test: nodes/web_ui/tests/test_ground_layer.py]
- a2: the labelled cloud serializes with labels aligned to the kept points, and without labels when the field is missing [evidence: test: nodes/web_ui/tests/test_ground_layer.py]
- a3: the full web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

- a4: the backend marks the cached ground state stale by receive age (ground_state_stale_s) and a new message clears it [evidence: test: nodes/web_ui/tests/test_ground_layer.py]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/cloud_serializer.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_ground_layer.py`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/README.md`

## Interfaces

roles `ground_cloud`, `ground_state` (part `web-ground-roles`); payload `{points_b64, colors_b64, labels_b64, frame, frame_id, ...}`; consumes parts `ground-state`, `ground-topics`, `ground-colour-labels` (label values 0..4).

## Expected outcome

`cd nodes/web_ui && uv run --frozen pytest tests/test_ground_layer.py -q` passes and the full web_ui suite stays green.

## Steps

- write tests/test_ground_layer.py
- run it, see FAIL
- implement config, bridge, cloud_serializer, default.yaml, client.yml
- run `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- update the README
- commit `feat(web_ui): ground cloud and state roles`.

## Notes

pattern: commit ba48647a (rgbd_cloud role: config.py:154/349, bridge.py:277-305, cloud_serializer.py:43-110). Other epics may edit client.yml concurrently: rebase.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on mcp-ground-summary
