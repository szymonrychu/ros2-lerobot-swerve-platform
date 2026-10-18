---
title: Web UI map3d cloud backend
status: todo
repo: ros2-lerobot-sverve-platform
value: "The browser gets the 3D map as a compact, capped point payload in the map frame, only while the Map tab shows it."
files: [nodes/web_ui/web_ui/cloud_serializer.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/config/default.yaml, nodes/web_ui/tests/test_map3d_cloud.py, ansible/group_vars/client.yml]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_map3d_topics.py -q"
depends_on: [map3d-topic-contract, map3d-uplink-node-type]
owns: [web-ui-map3d-roles]
uses: [map3d-topics]
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

# Web UI map3d cloud backend

## Requirement

- `cloud_serializer.py` (numpy only, D10): `serialize_map3d_cloud(msg, max_points: int, voxel_m: float) -> dict` decodes a sensor_msgs/PointCloud2 with float32 x, y, z and an optional packed `rgb`/`rgba` field (uint32 or float32 reinterpretation; any other layout raises ValueError naming the fields), drops non-finite points, voxel-downsamples (one point per `voxel_m` cell, first-seen colour) until at most `max_points` remain (increasing the voxel by x1.5 steps when needed), and returns `points_b64` (little-endian int16 centimetres x, y, z, clamped to +-327.67 m, out-of-range points dropped and counted), `colors_b64` (uint8 r, g, b per point; absent field -> key omitted), `count`, `source_count`, `voxel_m` (the one actually used), `dropped_out_of_range`, `frame_id`, `stamp`. An empty cloud returns `count: 0` (a real empty map, not placeholder) only when the message itself is empty; a malformed message is logged and not stored (no placeholder payload).
- Config (`config.py`, `default.yaml`, client.yml web_ui block): map_nav TabConfig fields `map3d_cloud_topic` (default `/map3d/cloud`), `map3d_status_topic` (default `/map3d/status`), `map3d_result_topic` (default `/map3d/result`), `map3d_command_topic` (default `/map3d/command`), `map3d_frame_id` (default `rtabmap_map`, shown in the map frame unchanged because D7 keeps RTAB-Map's map frame identical to slam_toolbox's `map`), `map3d_max_points` (default 60000, 1000..300000), `map3d_voxel_m` (default 0.05, 0.02..0.5), `lidar_wall_height_m` (default 1.0, 0.1..3.0) added to `MAP_NAV_DEFAULTS` and `MAP_NAV_TOPIC_ROLES` (`map3d_cloud`, `map3d_status`, `map3d_result`). Defaults equal the map3d topic contract's downlink dests (asserted by reading `nodes/rtabmap_mapping/config/map3d_topics.yaml` in the test).
- Bridge (`bridge.py`): role specs `map3d_cloud` (PointCloud2, best_effort keep_last 1, serializer above, on-demand: subscribed only while a client shows the layer, via `register_on_demand`, latest-wins), `map3d_status` (std_msgs/String JSON passed through as parsed JSON, reliable depth 1), `map3d_result` (String JSON, reliable depth 10). The cloud serializer runs lazily (the existing lazy-serialization path) so an unchanged cloud is not re-encoded per WS client.
- Tests first in `tests/test_map3d_cloud.py`, seen failing: xyz+rgb and xyz-only clouds; NaN dropped; cap respected with the voxel grown and reported; centimetre rounding; out-of-range counted; unsupported layout raises; config defaults and bounds; the defaults match the contract yaml; the role is on-demand and has the stated QoS.

## Acceptance conditions

- a1: PointCloud2 clouds serialize to capped int16-centimetre points with colours and metadata [evidence: test: nodes/web_ui/tests/test_map3d_cloud.py]
- a2: malformed clouds are rejected and never stored as placeholder payloads [evidence: test: nodes/web_ui/tests/test_map3d_cloud.py]
- a3: map3d topic defaults equal the contract's downlink dests [evidence: test: nodes/web_ui/tests/test_map3d_cloud.py]
- a4: the cloud subscription is on-demand with the stated QoS [evidence: test: nodes/web_ui/tests/test_map3d_cloud.py]
- a5: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/cloud_serializer.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_map3d_cloud.py`
- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
