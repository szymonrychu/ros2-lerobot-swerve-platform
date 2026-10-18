---
title: Laser filter ToF self filter
status: todo
repo: ros2-lerobot-sverve-platform
value: "The ToF cloud no longer contains the robot body, the arm or a carried object, so the robot never blocks itself."
files: [nodes/laser_filter/laser_filter/config.py, nodes/laser_filter/laser_filter/node.py, nodes/laser_filter/laser_filter/self_filter.py, nodes/laser_filter/config/footprint_filter.yaml, nodes/laser_filter/tests/test_cloud_filter.py, nodes/laser_filter/README.md, ansible/group_vars/client.yml, tests/test_nav_stack_config.py]
verify: "cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe test-ansible"
depends_on: [tof-obstacle-cloud-node]
owns: [tof-points-filtered-topic]
uses: [tof-points-topic]
model: opus
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Laser filter ToF self filter

## Requirement

Test first: `nodes/laser_filter/tests/test_cloud_filter.py` fails before the code exists (D2, D7, D8).

- `self_filter.py`: pure numpy function `cloud_self_mask(points_base: np.ndarray, polygon, max_height_m: float) ->
  np.ndarray` (True = keep): a point is removed when its (x, y) lies inside the current self polygon (shapely
  `contains_xy` on the prepared polygon, valid for the non-convex H) and its z <= `max_height_m`; non-finite points are
  removed; plus a helper that applies a 4x4 (or rotation + translation) transform to an (N, 3) array. Type hints and
  docstrings per CLAUDE.md.
- `config.py` (pydantic, unknown keys still rejected): `cloud_topic` (default `/tof/points`), `cloud_filtered_topic`
  (default `/tof/points_filtered`), `cloud_self_max_height_m` (1.0, > 0), `cloud_transform_timeout_s` (0.2, > 0);
  `cloud_topic: null` disables the cloud path. `config/footprint_filter.yaml` sets the four keys explicitly outside the
  generated region.
- `node.py`: subscribes `cloud_topic` (sensor-data QoS), reads xyz with `sensor_msgs_py.point_cloud2`, looks up
  `polygon_frame <- cloud frame` at the cloud's header stamp without blocking (timeout 0; one pending newest-wins slot
  retried on the next cloud or a short timer until `cloud_transform_timeout_s`, then dropped and counted in a log
  throttled warning), applies `cloud_self_mask` with the current runtime polygon, publishes the kept points as an xyz
  float32 cloud with the input header (same frame, same stamp) on `cloud_filtered_topic`. Scan filtering is never
  delayed by a cloud TF wait (separate callback handling or non-blocking lookups only). Nothing is published for a cloud
  without TF at its stamp; an input cloud with zero points is passed through as an empty cloud with its real header
  (D7); no timer ever publishes.
- `tests/test_cloud_filter.py`: points on the side frame / parked gripper region removed, points in the front and rear
  notch kept, points above `cloud_self_max_height_m` inside the polygon kept, below-floor points outside the polygon kept,
  non-finite removed, a replaced runtime polygon (dynamic footprint with a released arm shadow) removes arm points,
  transform helper round-trips a known yawed/tilted transform, config defaults and rejection of unknown keys.
- `ansible/group_vars/client.yml`: `laser_filter` type gains `apt_packages: [ros-jazzy-sensor-msgs-py]` (comment:
  point_cloud2 for the ToF cloud); `tests/test_nav_stack_config.py::test_laser_filter_ansible_wiring` asserts it
  instead of "no apt_packages".
- `nodes/laser_filter/README.md`: a "ToF cloud self filter" section (topics, frame/stamp kept, z limit, TF at stamp,
  non-blocking rule, drop counting, empty cloud rule, config rows), node purpose line updated.
- `scripts/gen_nav_footprint.py` still regenerates `footprint_filter.yaml` byte-identically
  (`tests/test_nav_footprint.py::test_laser_filter_equals_the_generator_output` passes).

## Acceptance conditions

- a1: points inside the self polygon below the height limit are removed, notch and high points kept [evidence: test: nodes/laser_filter/tests/test_cloud_filter.py]
- a2: a runtime polygon with the released arm shadow removes ToF arm points [evidence: test: nodes/laser_filter/tests/test_cloud_filter.py]
- a3: the node publishes kept points with the input frame and stamp only after TF at that stamp; no timer publishes [evidence: symbol: laser_filter.node.LaserFilterNode]
- a4: cloud TF lookups never block scan filtering (timeout 0 / pending slot) [evidence: judgement]
- a5: laser_filter type installs ros-jazzy-sensor-msgs-py and the wiring test pins it [evidence: test: tests/test_nav_stack_config.py::test_laser_filter_ansible_wiring]
- a6: the generated footprint_filter.yaml region is unchanged [evidence: test: tests/test_nav_footprint.py::test_laser_filter_equals_the_generator_output]
- a7: README documents the ToF cloud self filter [evidence: doc-section: nodes/laser_filter/README.md#ToF cloud self filter]
- a8: laser_filter suite and lint pass [evidence: command: cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/laser_filter/laser_filter/config.py`
- `nodes/laser_filter/laser_filter/node.py`
- `nodes/laser_filter/laser_filter/self_filter.py`
- `nodes/laser_filter/config/footprint_filter.yaml`
- `nodes/laser_filter/tests/test_cloud_filter.py`
- `nodes/laser_filter/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_nav_stack_config.py`

## Interfaces

- Topic `/tof/points_filtered` (PointCloud2 xyz float32, frame and stamp of `/tof/points`): contract
  `tof-points-filtered-topic`, defined by `cloud_filtered_topic` in `nodes/laser_filter/config/footprint_filter.yaml`.
- `laser_filter.self_filter.cloud_self_mask(points_base, polygon, max_height_m) -> np.ndarray[bool]`.
- Existing runtime parameter `polygon` (set by mcp_server `footprint.py`) is the only polygon source; unchanged.

## Expected outcome

`/tof/points_filtered` carries the ToF points minus the robot itself (body, parked or released arm, carried object),
stamped with the ToF capture time, while `/scan_filtered` keeps its rate.

## Steps

- Write `tests/test_cloud_filter.py`; `cd nodes/laser_filter && uv run --frozen pytest tests/test_cloud_filter.py -q`
  fails (missing function / config fields).
- Implement `cloud_self_mask` and the transform helper, then the config fields, then the node wiring; rerun tests.
- Update the client.yml apt list and the root wiring test; run the verify command.
- README section; commit `feat(laser_filter): self-filter the ToF cloud with the runtime self polygon`.

## Notes

- Reuse `parse_polygon`, `apply_polygon_param`, `quaternion_matrix` and the prepared polygon already in
  `self_filter.py`; do not add a second polygon parameter (mcp_server keeps setting only `polygon`).
- The head TF moves: never cache the cloud transform (the scan's static transform cache stays as it is).
- rclpy TransformListener already subscribes /tf and /tf_static; keep one Buffer.
