---
title: Web UI obstacle points API
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The web UI backend can stream the ToF obstacle cloud and the lidar points into the 3D map view on demand.
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/web_ui/msg_serializer.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_obstacle_points.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run poe test-ansible"
depends_on: [collision-monitor-tof-stop]
owns: [obstacle-points-payload]
uses: [tof-points-filtered-topic]
model: sonnet
estimate: 70
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-3d-obstacle-detection-fo--web-ui-obstacle-points-api
branch: tof-3d-obstacle-detection-fo--web-ui-obstacle-points-api
verified_head: ""
run: ""
provides: []
---

# Web UI obstacle points API

## Requirement

Test first: `nodes/web_ui/tests/test_obstacle_points.py` fails before the code exists (D10, U5).

- `web_ui/config.py`: map_nav `TabConfig` fields `obstacle_cloud_topic` (default `/tof/points_filtered`,
  sensor_msgs/PointCloud2) and `scan_points_topic` (default `/scan_filtered`, sensor_msgs/LaserScan), added to the
  map_nav defaults and to `MAP_NAV_TOPIC_ROLES` as roles `cloud` and `scan_points`; not defaulted for other tab types.
- `web_ui/msg_serializer.py`: `serialize_point_cloud(msg, max_points=2000)` -> `{stamp, frame, points}` (flat xyz float
  list in the sensor frame, non-finite dropped, evenly downsampled to <= max_points, rounded to mm) and
  `serialize_scan_points(msg, max_points=720)` -> `{stamp, frame, points}` (flat xy list in the scan frame from finite
  ranges within range_min..range_max). Both use numpy (`sensor_msgs_py.point_cloud2` or a numpy view of `data`).
- `web_ui/bridge.py`: both roles subscribed on demand only (existing demand mechanism), latest-wins; each payload gains
  `frame_pose` = the pose of the sensor frame in `map` at the message stamp (x, y, z + quaternion) from the bridge TF
  buffer; when that TF is unknown the message is not sent (no placeholder).
- `config/default.yaml` and the client.yml web_ui map_nav tab set both topics explicitly with comments.
- `tests/test_obstacle_points.py`: serialisers (caps, downsampling, NaN/inf, empty cloud gives empty points, frame and
  stamp), config defaults/overrides and role mapping, a bridge-level test that a missing map TF sends nothing and a
  known TF attaches `frame_pose`.
- `nodes/web_ui/README.md`: map_nav "3D obstacles" layer (topics, payload, caps, on demand, ToF + lidar together).

## Acceptance conditions

- a1: serialize_point_cloud and serialize_scan_points cap, downsample and drop non-finite points [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q]
- a2: map_nav tab has obstacle_cloud_topic and scan_points_topic with roles cloud and scan_points [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q]
- a3: payloads carry frame_pose in map and nothing is sent without that TF [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q]
- a4: client.yml and default.yaml map_nav tabs name both topics [evidence: command: grep -q 'obstacle_cloud_topic: /tof/points_filtered' ansible/group_vars/client.yml && grep -q 'obstacle_cloud_topic: /tof/points_filtered' nodes/web_ui/config/default.yaml]
- a5: README documents the 3D obstacles layer [evidence: judgement]
- a6: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/msg_serializer.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_obstacle_points.py`
- `nodes/web_ui/README.md`

## Interfaces

- `obstacle-points-payload` (defined in `nodes/web_ui/web_ui/msg_serializer.py`): role `cloud` -> `{stamp: float,
  frame: str, frame_pose: {x, y, z, qx, qy, qz, qw}, points: number[] (x,y,z repeated)}`; role `scan_points` -> same
  with `points` (x,y repeated). Consumed by web-ui-obstacle-points-layer.

## Expected outcome

With the layer requested by a browser, the WS stream carries the ToF cloud (<= 2000 points) and lidar points (<= 720)
with their map poses; with no viewer, nothing is subscribed.

## Steps

- Write `tests/test_obstacle_points.py`; `cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q` fails.
- Add serialisers, config fields/roles, bridge wiring; rerun; run the verify command.
- README; commit `feat(web_ui): obstacle cloud and lidar points roles for the 3D map`.

## Notes

- Reuse `downsample_points`, `transform_to_pose_dict` and the existing costmap TF-to-map handling in `bridge.py`.
- web_ui stays Python (auto-memory); keep serialisation lazy like the other on-demand roles.
