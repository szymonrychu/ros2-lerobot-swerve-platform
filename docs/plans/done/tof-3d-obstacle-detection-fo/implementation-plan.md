# ToF 3D obstacle detection for Nav2 implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step).

- Status: done
- Files: `.python-version`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: 271aae7d

#### Requirement

The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step).

Where: 

Why: Without the pin, every task's verification fails at uv sync regardless of the code.

#### Acceptance conditions

- AC1: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step). at the place the run named is resolved [evidence: judgement]

#### Files

- .python-version


#### Interfaces

Keep the interfaces around the finding unchanged unless resolving it requires otherwise: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step)..

#### Expected outcome

The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step). at the place the run named is resolved, and Without the pin, every task's verification fails at uv sync regardless of the code. no longer applies.

#### Steps

1. Write a failing test that shows: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step)..
2. Fix it at the place named above.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `tof-obstacle-cloud-node` during run `tof-3d-obstacle-detection-fo`.

- 2026-10-27 done: re-verified 2026-10-27: root suite green on main (2156 passed)

- 2026-10-27 actuals: no agent records

#### Ruling

Fixed on main by the orchestrator: root .python-version pins 3.12 (uv now resolves 3.12.13; root 1430 passed). Closed.

### S2: ToF obstacle cloud node

- Status: done
- Files: `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`, `nodes/tof_obstacle_cloud/README.md`, `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_tof_obstacle_cloud_config.py`, `tests/test_unit_cgroups.py`, `tests/README.md`, `nodes/README.md`
- Verify: `uv run pytest tests/test_tof_obstacle_cloud_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: eb4daa9d6fe41f1e2cbba45bb30a62186d177247

#### Requirement

Test first: write `tests/test_tof_obstacle_cloud_config.py` and see it fail before any config exists (D1, D7, D8, D9).

- New config-only node dir `nodes/tof_obstacle_cloud/` (no Python package, no pyproject): `config/point_cloud_xyz.yaml`
  holds the ROS params of `rtabmap_util/point_cloud_xyz` under `tof_obstacle_cloud: ros__parameters:` with
  `decimation: 4`, `min_depth: 0.2`, `max_depth: 3.0`, `voxel_size: 0.05`, `filter_nans: true`, `approx_sync: false`,
  `noise_filter_radius: 0.1`, `noise_filter_min_neighbors: 2`, a `qos` value compatible with the arducam_tof
  depth/camera_info publishers (read the QoS in `nodes/bridges/arducam_tof/arducam_tof/bridge.py` and comment the
  choice), each key commented with its reason (D1).
- `ansible/group_vars/client.yml`: node type `tof_obstacle_cloud` (`deploy_mode: native`, `node_launch_command:
  ros2 run rtabmap_util point_cloud_xyz --ros-args --params-file {{ ros2_repo_dest }}/nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml
  -r depth/image:=/tof/depth/image_raw -r depth/camera_info:=/tof/camera_info -r cloud:=/tof/points
  -r __node:=tof_obstacle_cloud`, `node_src_dir: ""`, `src_paths: [nodes/tof_obstacle_cloud]`, `apt_packages:
  [ros-jazzy-rtabmap-util]`, normal tier `cpu_weight 500`, `io_weight 500`, `oom_score_adjust -500`, `memory_low 64M`,
  `memory_max 192M` with the estimate comment used by other unmeasured types); a `ros2_nodes` entry `tof_obstacle_cloud`
  (`present: true`, `enabled: true`) right after `arducam_tof`, with a comment naming input and output topics.
- `ansible/playbooks/deploy_nodes_client.yml`: a "Deploy tof_obstacle_cloud" block after arducam_tof in the same shape
  as the rf2o / apt-only nodes (`_deploy_node_name: tof_obstacle_cloud`, tags).
- `tests/test_unit_cgroups.py`: `tof_obstacle_cloud` in the NORMAL tier set; every cgroup test still passes
  (memory_low sum within `MAX_MEMORY_LOW_SUM_MIB`).
- Static tests in `tests/test_tof_obstacle_cloud_config.py`: launch command, remaps and node name; apt package; params
  values above; `max_depth` <= the arducam_tof `range_mode` range from client.yml (far = 4.0 m, near = 2.0 m) and
  `min_depth` > 0; arducam_tof `frame_id` is `tof_optical_frame`; the type has no TF-publishing argument and no other
  client node type publishes `map -> odom` besides slam_toolbox (U4, D9); the playbook deploys it; entry present/enabled.
- `nodes/tof_obstacle_cloud/README.md` (audience: mid-level ROS2 dev): purpose, topics in/out (`/tof/points`,
  sensor frame `tof_optical_frame`, stamp = ToF capture stamp copied from the depth header), params table, why
  point_cloud_xyz (and not depth_image_proc / obstacles_detection, D1), the placeholder rule (publishes only from a
  synced real frame; an all-invalid frame gives an empty cloud with the real stamp, D7), CPU note. `nodes/README.md`
  lists the node; `tests/README.md` documents the new test file and each test.

#### Acceptance conditions

- a1: point_cloud_xyz node type, remaps, apt package and entry are configured as specified [evidence: test: tests/test_tof_obstacle_cloud_config.py]
- a2: max_depth stays within the configured ToF range and min_depth > 0 [evidence: test: tests/test_tof_obstacle_cloud_config.py]
- a3: no new map->odom or other TF publisher; slam_toolbox stays the map->odom owner [evidence: test: tests/test_tof_obstacle_cloud_config.py]
- a4: the new type is in the normal tier and the cgroup suite passes [evidence: command: uv run pytest tests/test_unit_cgroups.py -q]
- a5: the node README documents topics, params, stamp/frame and the empty-cloud rule [evidence: doc-section: nodes/tof_obstacle_cloud/README.md#Placeholder data]
- a6: ansible syntax and lint pass [evidence: command: uv run poe test-ansible]

#### Files

- `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`
- `nodes/tof_obstacle_cloud/README.md`
- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_tof_obstacle_cloud_config.py`
- `tests/test_unit_cgroups.py`
- `tests/README.md`
- `nodes/README.md`

#### Interfaces

- Topic `/tof/points` (sensor_msgs/PointCloud2, xyz float32, frame `tof_optical_frame`, stamp of the ToF frame):
  contract `tof-points-topic`, defined by the remap in `ansible/group_vars/client.yml` (tof_obstacle_cloud type).
- Inputs (existing): `/tof/depth/image_raw` 16UC1 mm (0 = invalid), `/tof/camera_info`, one shared header per frame
  (`nodes/bridges/arducam_tof/arducam_tof/bridge.py`).

#### Expected outcome

After deploy, `ros2-tof_obstacle_cloud.service` runs point_cloud_xyz and `/tof/points` carries at most 2700 points per
ToF frame at the ToF rate, only while real ToF frames arrive.

#### Steps

- Write `tests/test_tof_obstacle_cloud_config.py`; run `uv run pytest tests/test_tof_obstacle_cloud_config.py -q`;
  expect failures (no type, no params file).
- Add the params file, node type, entry, playbook block and cgroup tier; rerun until green; run the verify command.
- Write the README sections (heading `## Placeholder data` included), `nodes/README.md` row, `tests/README.md` rows.
- Commit `feat(tof_obstacle_cloud): decimated ToF point cloud via rtabmap_util point_cloud_xyz`.

#### Notes

- Verified 2026-10-18: `ros-jazzy-rtabmap-util 0.23.13` is in the Jazzy arm64 apt index; it pulls the rtabmap core
  libraries. point_cloud_xyz params and topic names are from rtabmap_ros 0.23.13 `point_cloud_xyz.cpp` (design,
  Verified facts).
- The node publishes only from its synced callback; do not add any republisher or timer.
- Do not add the node to any other host. Do not touch nav2_params here.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD (38 tests, lint, test-ansible); the escalation was a missing README heading name (now '## Placeholder data') and a macOS uv build of manifold3d in the verifier env.

### S3: ToF obstacle sector summary

- Status: done
- Files: `nodes/mcp_server/mcp_server/perception.py`, `nodes/mcp_server/mcp_server/perception_models.py`, `nodes/mcp_server/tests/test_tof_obstacles.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_tof_obstacles.py tests/test_perception.py -q && uv run --frozen poe lint`
- Verified head: c5b12f7f4d6f7a2263515df1e833688b61bad719

#### Requirement

Test first: `tests/test_tof_obstacles.py` fails before the function exists (D11).

- `perception.py`: `tof_obstacle_sectors(points_base: np.ndarray, obstacle_band: tuple[float, float], cliff_max_z:
  float, max_range_m: float) -> list[TofSector]` over the existing eight 45 deg `SECTOR_NAMES` (same sector maths as the
  scan summary): per sector the nearest obstacle point (horizontal distance from base_link, bearing, min and max z of
  obstacle points within 0.1 m of that distance) and the nearest cliff point distance; None fields when a sector has no
  such point; non-finite points ignored; points beyond `max_range_m` ignored.
- `perception_models.py`: pydantic `TofSector` and `Obstacles3d` (`sectors`, `cloud_age_s`, `frame`, `point_count`)
  with field descriptions written for the agent.
- `tests/test_tof_obstacles.py`: sector assignment at boundaries, nearest selection, height range, cliff detection,
  empty cloud gives all-None sectors, NaN handling, range cut.
- `tests/test_perception.py` (existing scan-sector suite) stays green.

#### Acceptance conditions

- a1: tof_obstacle_sectors returns per-sector nearest obstacle with height range and nearest cliff [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_tof_obstacles.py -q]
- a2: empty, non-finite and out-of-range points yield None fields, never zeros [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_tof_obstacles.py -q]
- a3: Obstacles3d model exists for get_map_summary [evidence: symbol: mcp_server.perception_models.Obstacles3d]
- a4: mcp_server lint passes [evidence: command: cd nodes/mcp_server && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/perception.py`
- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/tests/test_tof_obstacles.py`

#### Interfaces

- `tof-sector-summary`: `mcp_server.perception.tof_obstacle_sectors(...) -> list[TofSector]` and
  `perception_models.Obstacles3d`; consumed by map-summary-tof-obstacles.

#### Expected outcome

A pure, ROS-free function turns base_link points into the 8-sector 3D obstacle/cliff summary used by the agent tool.

#### Steps

- Write `tests/test_tof_obstacles.py`; run it; it fails on the missing import.
- Implement the models and the function with numpy; rerun; run the verify command.
- Commit `feat(mcp_server): ToF obstacle and cliff sector summary`.

#### Notes

- Reuse `SECTOR_NAMES` and the bearing/sector index maths in `perception.py` (scan summary) rather than duplicating it.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD (38 tests, lint, test-ansible); the escalation was a missing README heading name (now '## Placeholder data') and a macOS uv build of manifold3d in the verifier env.

### S4: Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'.

- Status: done
- Files: `ansible/group_vars/client.yml`
- Verify: `true  # delivered before the 2026-10-27 ticket migration`
- Verified head: 271aae7d

#### Requirement

Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'.

Where: ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540

Why: This is the same parking wording the review flagged, but it predates this task and belongs to other nodes' entries, so I left it alone.

#### Acceptance conditions

- AC1: Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'. at ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540 is resolved [evidence: judgement]

#### Files

- ansible/group_vars/client.yml


#### Interfaces

Keep the interfaces around ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540 unchanged unless resolving it requires otherwise: Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'..

#### Expected outcome

Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'. at ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540 is resolved, and This is the same parking wording the review flagged, but it predates this task and belongs to other nodes' entries, so I left it alone. no longer applies.

#### Steps

1. Write a failing test that shows: Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'..
2. Fix it at ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `laser-filter-tof-self-filter` during run `tof-3d-obstacle-detection-fo`.

- 2026-10-27 done: CORRECTION 2026-10-27: NOT delivered - grep still finds 2 'Not measured yet (C++ relay' comments in client.yml; carried into realsense-d435-head-switchov as client-yml-stale-memory-comm

- 2026-10-27 actuals: no agent records

#### Ruling

Folded into final-suite-and-runbook by the orchestrator (run 3 planText note).

### S5: Laser filter ToF self filter

- Status: done
- Files: `nodes/laser_filter/laser_filter/config.py`, `nodes/laser_filter/laser_filter/node.py`, `nodes/laser_filter/laser_filter/self_filter.py`, `nodes/laser_filter/config/footprint_filter.yaml`, `nodes/laser_filter/tests/test_cloud_filter.py`, `nodes/laser_filter/README.md`, `ansible/group_vars/client.yml`, `tests/test_nav_stack_config.py`
- Verify: `cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe test-ansible`
- Verified head: f8fde6c11babf46564892bf2cdaf7f25d10abf76

#### Requirement

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

#### Acceptance conditions

- a1: points inside the self polygon below the height limit are removed, notch and high points kept [evidence: command: cd nodes/laser_filter && uv run --frozen pytest tests/test_cloud_filter.py -q]
- a2: a runtime polygon with the released arm shadow removes ToF arm points [evidence: command: cd nodes/laser_filter && uv run --frozen pytest tests/test_cloud_filter.py -q]
- a3: the node publishes kept points with the input frame and stamp only after TF at that stamp; no timer publishes [evidence: symbol: laser_filter.node.LaserFilterNode]
- a4: cloud TF lookups never block scan filtering (timeout 0 / pending slot) [evidence: judgement]
- a5: laser_filter type installs ros-jazzy-sensor-msgs-py and the wiring test pins it [evidence: test: tests/test_nav_stack_config.py::test_laser_filter_ansible_wiring]
- a6: the generated footprint_filter.yaml region is unchanged [evidence: test: tests/test_nav_footprint.py::test_laser_filter_equals_the_generator_output]
- a7: README documents the ToF cloud self filter [evidence: doc-section: nodes/laser_filter/README.md#ToF cloud self filter]
- a8: laser_filter suite and lint pass [evidence: command: cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/laser_filter/laser_filter/config.py`
- `nodes/laser_filter/laser_filter/node.py`
- `nodes/laser_filter/laser_filter/self_filter.py`
- `nodes/laser_filter/config/footprint_filter.yaml`
- `nodes/laser_filter/tests/test_cloud_filter.py`
- `nodes/laser_filter/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_nav_stack_config.py`

#### Interfaces

- Topic `/tof/points_filtered` (PointCloud2 xyz float32, frame and stamp of `/tof/points`): contract
  `tof-points-filtered-topic`, defined by `cloud_filtered_topic` in `nodes/laser_filter/config/footprint_filter.yaml`.
- `laser_filter.self_filter.cloud_self_mask(points_base, polygon, max_height_m) -> np.ndarray[bool]`.
- Existing runtime parameter `polygon` (set by mcp_server `footprint.py`) is the only polygon source; unchanged.

#### Expected outcome

`/tof/points_filtered` carries the ToF points minus the robot itself (body, parked or released arm, carried object),
stamped with the ToF capture time, while `/scan_filtered` keeps its rate.

#### Steps

- Write `tests/test_cloud_filter.py`; `cd nodes/laser_filter && uv run --frozen pytest tests/test_cloud_filter.py -q`
  fails (missing function / config fields).
- Implement `cloud_self_mask` and the transform helper, then the config fields, then the node wiring; rerun tests.
- Update the client.yml apt list and the root wiring test; run the verify command.
- README section; commit `feat(laser_filter): self-filter the ToF cloud with the runtime self polygon`.

#### Notes

- Reuse `parse_polygon`, `apply_polygon_param`, `quaternion_matrix` and the prepared polygon already in
  `self_filter.py`; do not add a second polygon parameter (mcp_server keeps setting only `polygon`).
- The head TF moves: never cache the cloud transform (the scan's static transform cache stays as it is).
- rclpy TransformListener already subscribes /tf and /tf_static; keep one Buffer.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on tof-obstacle-cloud-node

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (root 1556 passed); nav2-tof-voxel-layer escalation was a debt-marker false positive on a tests/README row quoting the banned words (reworded).

### S6: Cliff stop polygon generator

- Status: done
- Files: `scripts/gen_nav_footprint.py`, `scripts/gen_nav_footprint.yaml`, `tests/test_nav_footprint.py`, `nodes/nav2_bringup/config/nav2_params.yaml`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint`
- Verified head: 81f876d02257de7810085e9543b7a436bec02178

#### Requirement

Test first: new tests in `tests/test_nav_footprint.py` fail before the generator change (D6).

- `scripts/gen_nav_footprint.yaml`: `cliff_margin_m: 0.25` with a comment (head ToF 0.35-0.40 m high sees the lower
  floor of a 15 cm step about 0.2 m beyond the edge, so the guard reaches past it); validated by the Settings model
  (> 0).
- `scripts/gen_nav_footprint.py`: `Generated` gains the cliff polygon `offset_polygon(footprint, cliff_margin_m,
  decimals)` (reuse the existing `offset_polygon`); a new marker region `cliffstop` rendered as the `points:` line of the
  collision monitor `CliffStop` polygon (polygon string format like the StopBox sub-polygons, >= 4 points).
- `nodes/nav2_bringup/config/nav2_params.yaml`: under `collision_monitor` add the `CliffStop` block (`type: polygon`,
  `action_type: approach`, `time_before_collision: 1.2`, `simulation_time_step: 0.05`, `min_points: 4`,
  `visualize: false`, `enabled: true`, `sources_names: ["tof_cliff"]`, the generated `cliffstop` region) with a comment
  that it is inert until listed in `polygons` (done by task collision-monitor-tof-stop); `polygons:` is NOT changed here.
- Tests: the cliff polygon equals the footprint grown by `cliff_margin_m` and contains the footprint;
  `test_nav2_params_equal_the_generator_output` covers the new region; `main` stays idempotent; the polygon has >= 4
  points.
- `nodes/nav2_bringup/README.md` "Footprint" section names the CliffStop polygon and its margin; `tests/README.md` rows.

#### Acceptance conditions

- a1: the generated CliffStop polygon is the footprint grown by cliff_margin_m with >= 4 points [evidence: test: tests/test_nav_footprint.py]
- a2: nav2_params.yaml equals the generator output including the cliffstop region [evidence: test: tests/test_nav_footprint.py::test_nav2_params_equal_the_generator_output]
- a3: CliffStop is not yet listed in collision_monitor polygons [evidence: command: uv run python -c "import yaml;d=yaml.safe_load(open('nodes/nav2_bringup/config/nav2_params.yaml'));assert 'CliffStop' not in d['collision_monitor']['ros__parameters']['polygons']"]
- a4: nav stack and footprint tests pass [evidence: command: uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]
- a5: README footprint section documents the cliff polygon [evidence: judgement]

#### Files

- `scripts/gen_nav_footprint.py`
- `scripts/gen_nav_footprint.yaml`
- `tests/test_nav_footprint.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

- `cliffstop-polygon`: the `cliffstop` marker region written by `scripts/gen_nav_footprint.py` into
  `collision_monitor.CliffStop.points`; consumed by collision-monitor-tof-stop, which lists `CliffStop` in `polygons`.

#### Expected outcome

`python -m scripts.gen_nav_footprint` (or the repo's documented invocation) rewrites the CliffStop points from the body
model; the polygon block exists but does not change runtime behaviour yet.

#### Steps

- Add failing tests to `tests/test_nav_footprint.py`; run `uv run pytest tests/test_nav_footprint.py -q`.
- Extend Settings, `generate`, `region_lines`/`render_nav2`; add the CliffStop block with empty markers; run the
  generator to fill it; rerun the verify command.
- README + tests/README; commit `feat(nav2): generated CliffStop polygon (footprint + cliff margin)`.

#### Notes

- Never hand-edit generated regions; the markers are `# >>> gen_nav_footprint: cliffstop` / `# <<< ...`.
- Jazzy rejects polygons with <= 3 points and one bad polygon aborts the whole nav2 bringup (existing comment).

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on laser-filter-tof-self-filter

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (root 1556 passed); nav2-tof-voxel-layer escalation was a debt-marker false positive on a tests/README row quoting the banned words (reworded).

### S7: Nav2 ToF voxel layer

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `ansible/group_vars/client.yml`, `tests/test_nav_tof_voxel_layer.py`, `tests/test_nav_stack_config.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: f1e210fb8532cc952db741b3e8940a7087b5612f

#### Requirement

Test first: `tests/test_nav_tof_voxel_layer.py` fails before the config exists (D3, D4, D8; U1, U2, U5).

- `nav2_params.yaml` local costmap: `plugins: ["obstacle_layer", "tof_voxel_layer", "keepout_filter",
  "inflation_layer"]`; `tof_voxel_layer` with exactly the D3 settings: `plugin:
  "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"`, `enabled true`, `voxel_decay 15.0`, `decay_model 0`,
  `voxel_size 0.05`, `track_unknown_space false`, `mark_threshold 0`, `update_footprint_enabled false` (comment: STVL
  clears the footprint with setConvexPolygonCost, convex like the ObstacleLayer, would erase obstacles in the H
  notches), `combination_method 1`, `publish_voxel_map false`, `transform_tolerance 0.3`, `mapping_mode false`,
  `observation_sources: tof_mark tof_cliff_mark tof_clear`; `tof_mark` (marking, not clearing, PointCloud2,
  `/tof/points_filtered`, `min_obstacle_height 0.06`, `max_obstacle_height 0.6`, `obstacle_range 2.0`, `filter voxel`,
  `voxel_min_points 0`, `observation_persistence 0.0`, `clear_after_reading true`, `expected_update_rate 0.0`,
  `inf_is_valid false`); `tof_cliff_mark` (same but `min_obstacle_height -1.0`, `max_obstacle_height -0.10`);
  `tof_clear` (clearing, not marking, `model_type 0`, `min_z 0.2`, `max_z 3.0`, `horizontal_fov_angle 1.0`,
  `vertical_fov_angle 0.78`, `decay_acceleration 1.0`). Each value commented with its reason (floor z = 0 in odom,
  ToF noise, robot height, FOV under the 70 deg diagonal).
- U5 kept: `obstacle_layer` with the `/scan_filtered` LaserScan source unchanged in both costmaps; the global costmap
  plugins unchanged (no STVL there); no STVL source uses `/scan_filtered` or a LaserScan (STVL is not for planar lidars).
- `ansible/group_vars/client.yml` nav2_bringup `apt_packages` gains `ros-jazzy-spatio-temporal-voxel-layer` (comment).
- `tests/test_nav_stack_config.py`: the plugin-order parametrisation of
  `test_nav2_keepout_filter_sits_right_before_inflation` uses the new local list; everything else unchanged and green.
- `tests/test_nav_tof_voxel_layer.py`: every value above; mark/clear split; heights: obstacle band above the floor and
  below robot height, cliff band strictly below `-0.05`; ranges: `max_z` and `obstacle_range` <= the
  tof_obstacle_cloud `max_depth` (read from `nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`); clearing FOV below
  the ToF FOV constants; lidar layer still present in both costmaps with the scan source; apt package listed.
- `nodes/nav2_bringup/README.md`: new section "ToF obstacle layer" (pipeline, layer settings table, U5 separation:
  ToF clearing only decays ToF voxels, lidar raytracing never clears ToF voxels, why local only, limitation: a cliff
  into a void with no returns is not seen); `tests/README.md` rows.

#### Acceptance conditions

- a1: local costmap plugins are obstacle_layer, tof_voxel_layer, keepout_filter, inflation_layer [evidence: test: tests/test_nav_stack_config.py::test_nav2_keepout_filter_sits_right_before_inflation]
- a2: tof_voxel_layer settings, mark/clear sources and height bands match D3 [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a3: STVL footprint clearing is disabled [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a4: the lidar obstacle_layer keeps /scan_filtered in both costmaps and no STVL source uses a LaserScan (U5) [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a5: nav2_bringup installs ros-jazzy-spatio-temporal-voxel-layer [evidence: test: tests/test_nav_tof_voxel_layer.py]
- a6: README documents the ToF obstacle layer and the lidar separation [evidence: doc-section: nodes/nav2_bringup/README.md#ToF obstacle layer]
- a7: nav, footprint and ansible checks pass [evidence: command: uv run pytest tests/test_nav_tof_voxel_layer.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe test-ansible]
- a8: ToF voxels live only in the local costmap; the global costmap has no STVL plugin or ToF source, decided by the user on 2026-10-18 [evidence: test: tests/test_nav_tof_voxel_layer.py]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_nav_stack_config.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

- Consumes `tof-points-filtered-topic` (`/tof/points_filtered`).
- Layer name `tof_voxel_layer` in `local_costmap` (mcp_server and web_ui read the merged `/local_costmap/costmap`; no
  change there).

#### Expected outcome

On the robot the local costmap marks ToF obstacles between 6 and 60 cm and below-floor cliff points, decaying 15 s after
they leave view, while lidar cells behave as before.

#### Steps

- Write `tests/test_nav_tof_voxel_layer.py` and update the parametrisation; `uv run pytest tests/test_nav_tof_voxel_layer.py -q` fails.
- Edit `nav2_params.yaml` (outside generated regions) and the apt list; rerun the verify command.
- README section and tests/README rows; commit `feat(nav2): ToF spatio-temporal voxel layer in the local costmap`.

#### Notes

- Verified: `ros-jazzy-spatio-temporal-voxel-layer 2.5.5` on Jazzy arm64; parameter names from the STVL jazzy source
  (design, Verified facts). Heights in STVL are in the costmap global frame (odom), where z = 0 is the floor.
- Do not touch the collision_monitor here (next task) or generated regions.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on cliff-stop-polygon-generator

#### Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (root 1556 passed); nav2-tof-voxel-layer escalation was a debt-marker false positive on a tests/README row quoting the banned words (reworded).

### S8: Collision monitor ToF stop

- Status: done
- Files: `nodes/nav2_bringup/config/nav2_params.yaml`, `tests/test_nav_tof_collision.py`, `tests/test_nav_stack_config.py`, `nodes/nav2_bringup/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint`
- Verified head: 3b27ad9559953651df82aed22e35d83f2fbbfcec

#### Requirement

Test first: `tests/test_nav_tof_collision.py` fails before the config exists (D4, D5; U1, U5).

- `nav2_params.yaml` collision_monitor: `observation_sources: ["scan", "tof_obstacles", "tof_cliff"]`;
  `tof_obstacles` (`type: "pointcloud"`, `topic: "/tof/points_filtered"`, `min_height: 0.06`, `max_height: 0.6`,
  `min_range: 0.0`, `enabled: true`) and `tof_cliff` (same topic, `min_height: -1.0`, `max_height: -0.10`); both keep
  the node `source_timeout` 1.0 (no per-source override), with a comment that a ToF or head-TF outage stops motion
  ("invalid source", collision_monitor_node.cpp 1.3.13) and how to disable a source at runtime (`ros2 param set
  /collision_monitor tof_cliff.enabled false`).
- `polygons: ["StopBox", "ArmStop", "CliffStop"]`; `StopBox.sources_names: ["scan", "tof_obstacles"]`;
  `ArmStop.sources_names: ["scan"]` (comment: ToF edge noise around the released arm must not freeze it; the costmap
  footprint includes the arm shadow); `CliffStop.sources_names: ["tof_cliff"]` (block from cliff-stop-polygon-generator,
  unchanged otherwise). Generated regions untouched.
- U5: the `scan` source stays `type: scan`, `/scan_filtered`, enabled, and stays in StopBox.
- `tests/test_nav_tof_collision.py`: sources and bands (obstacle band equals the STVL `tof_mark` band, cliff band equals
  `tof_cliff_mark`), all on `/tof/points_filtered`; every polygon's `sources_names` names only declared sources; StopBox
  / ArmStop / CliffStop source sets as above; CliffStop is `approach` with `time_before_collision` > 0 and its points
  equal the generator's cliff polygon; no ToF source sets `source_timeout: 0.0`. Existing
  `test_nav2_collision_monitor_uses_scan` stays green (CliffStop is a static polygon with >= 4 points).
- Docker param-load check, as in the README "Validation in Docker" practice: arm64 `ros:jazzy` with
  `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-spatio-temporal-voxel-layer`; load the repo
  `nav2_params.yaml` into a local costmap and the collision_monitor under a lifecycle manager with static TFs
  (base_link -> tof_optical_frame looking forward-down) and a synthetic `/tof/points_filtered` publisher: both configure
  and activate without errors; a box of points 0.6 m ahead at z 0.3 marks local costmap cells; points at z -0.3 inside
  the CliffStop polygon make the monitor stop a forward `cmd_vel_smoothed` and pass a backward one; stopping the
  synthetic publisher for > 1 s makes it stop ("invalid source"). Results recorded in the README section "ToF
  validation in Docker" (date, image, package versions, each outcome). If Docker is not available, report `blocked`
  with that reason (never claim it ran).
- `nodes/nav2_bringup/README.md`: "Collision monitor" section updated (sources table, polygon/source matrix, fail-safe
  stop on ToF outage, cliff limitation: only while the ToF looks at it), plus the Docker section; `tests/README.md` rows.

#### Acceptance conditions

- a1: tof_obstacles and tof_cliff pointcloud sources on /tof/points_filtered with the D5 bands [evidence: test: tests/test_nav_tof_collision.py]
- a2: StopBox uses scan + tof_obstacles, ArmStop scan only, CliffStop tof_cliff with approach [evidence: test: tests/test_nav_tof_collision.py]
- a3: CliffStop is listed in polygons and its points equal the generated cliff polygon [evidence: test: tests/test_nav_tof_collision.py]
- a4: the scan source and its StopBox use are unchanged (U5) [evidence: test: tests/test_nav_stack_config.py::test_nav2_collision_monitor_uses_scan]
- a5: Docker param-load run recorded with every outcome [evidence: doc-section: nodes/nav2_bringup/README.md#ToF validation in Docker]
- a6: nav and footprint tests pass [evidence: command: uv run pytest tests/test_nav_tof_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]
- a7: a ToF or head-TF outage stops the robot while Nav2 drives: both ToF sources keep a non-zero source_timeout (stale-source stop), decided by the user on 2026-10-18 [evidence: test: tests/test_nav_tof_collision.py]

#### Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_tof_collision.py`
- `tests/test_nav_stack_config.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

#### Interfaces

- Consumes `tof-points-filtered-topic` and `cliffstop-polygon`.
- collision_monitor source names `tof_obstacles`, `tof_cliff` (runtime-toggled via `<source>.enabled`).

#### Expected outcome

The collision monitor stops for ToF obstacles inside the swept StopBox, blocks motion toward below-floor points within
0.25 m of the footprint while allowing motion away, and stops Nav2 motion when the ToF cloud or its TF goes stale.

#### Steps

- Write `tests/test_nav_tof_collision.py`; `uv run pytest tests/test_nav_tof_collision.py -q` fails.
- Edit the collision_monitor block (outside generated regions); rerun the verify command.
- Run the Docker param-load check; record the outcomes in the README; tests/README rows.
- Commit `feat(nav2): collision monitor ToF obstacle and cliff sources`.

#### Notes

- Verified in navigation2 1.3.13 source: pointcloud heights compare in the base frame (negative bands work);
  `sources_names` per polygon; an enabled source with non-zero timeout and no fresh data or TF forces STOP.
- The approach model needs the ToF to see the lower floor; the polygon margin (0.25 m) covers the occlusion offset.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on nav2-tof-voxel-layer

### S9: Web UI obstacle points API

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/msg_serializer.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_obstacle_points.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run poe test-ansible`
- Verified head: 79b5cbbcd8a9406b233e1efca7e780904f203863

#### Requirement

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

#### Acceptance conditions

- a1: serialize_point_cloud and serialize_scan_points cap, downsample and drop non-finite points [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q]
- a2: map_nav tab has obstacle_cloud_topic and scan_points_topic with roles cloud and scan_points [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q]
- a3: payloads carry frame_pose in map and nothing is sent without that TF [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q]
- a4: client.yml and default.yaml map_nav tabs name both topics [evidence: command: grep -q 'obstacle_cloud_topic: /tof/points_filtered' ansible/group_vars/client.yml && grep -q 'obstacle_cloud_topic: /tof/points_filtered' nodes/web_ui/config/default.yaml]
- a5: README documents the 3D obstacles layer [evidence: judgement]
- a6: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/msg_serializer.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_obstacle_points.py`
- `nodes/web_ui/README.md`

#### Interfaces

- `obstacle-points-payload` (defined in `nodes/web_ui/web_ui/msg_serializer.py`): role `cloud` -> `{stamp: float,
  frame: str, frame_pose: {x, y, z, qx, qy, qz, qw}, points: number[] (x,y,z repeated)}`; role `scan_points` -> same
  with `points` (x,y repeated). Consumed by web-ui-obstacle-points-layer.

#### Expected outcome

With the layer requested by a browser, the WS stream carries the ToF cloud (<= 2000 points) and lidar points (<= 720)
with their map poses; with no viewer, nothing is subscribed.

#### Steps

- Write `tests/test_obstacle_points.py`; `cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py -q` fails.
- Add serialisers, config fields/roles, bridge wiring; rerun; run the verify command.
- README; commit `feat(web_ui): obstacle cloud and lidar points roles for the 3D map`.

#### Notes

- Reuse `downsample_points`, `transform_to_pose_dict` and the existing costmap TF-to-map handling in `bridge.py`.
- web_ui stays Python (auto-memory); keep serialisation lazy like the other on-demand roles.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on collision-monitor-tof-stop

### S10: Map summary ToF obstacles

- Status: done
- Files: `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/tool_context.py`, `nodes/mcp_server/mcp_server/perception_tools.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/fakes.py`, `nodes/mcp_server/tests/test_map_summary_tof.py`, `nodes/mcp_server/README.md`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q`
- Verified head: 9ba93ff46cbb186c97942fc1e1538e7d9ffb275b

#### Requirement

Test first: `tests/test_map_summary_tof.py` fails before the wiring exists (D11).

- `config.py` `TopicSettings.tof_points: str = "/tof/points_filtered"` (comment: subscribed lazily by
  get_map_summary, latest message only); a staleness bound for it (reuse an existing timeouts field or add
  `timeouts.tof_points_stale_s`, default 1.0).
- `ros_iface.py`: lazy, latest-only subscription to `tof_points` (same on-demand pattern as `local_costmap`), returning
  the points transformed to `base_frame` with the TF at the cloud stamp, plus stamp and frame; None when no cloud, a
  stale cloud or no TF.
- `tool_context.py` `RobotApi` protocol method for it; `tests/fakes.py` FakeRobot implements it.
- `perception_tools.py` `get_map_summary`: adds `obstacles_3d` (`Obstacles3d` from `tof_obstacle_sectors` with the
  STVL bands 0.06-0.6 m and cliff below -0.10 m, range 2.0 m) or null with a short reason when unavailable; the tool
  description mentions it (ToF on the head, only what the head currently sees).
- `ansible/group_vars/client.yml` mcp_server `topics:` sets `tof_points: /tof/points_filtered` with a comment.
- `tests/test_map_summary_tof.py`: summary with a fake cloud (obstacle ahead, cliff left), null when stale / missing,
  description text; root `tests/test_mcp_server_config.py` stays green (client.yml/README/source coupling).
- `nodes/mcp_server/README.md` tool table row and topics row updated.

#### Acceptance conditions

- a1: get_map_summary returns obstacles_3d from a fresh ToF cloud in base_link [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_tof.py -q]
- a2: obstacles_3d is null when the cloud is missing, stale or has no TF [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_map_summary_tof.py -q]
- a3: the ToF topic is subscribed lazily, latest only [evidence: judgement]
- a4: client.yml mcp_server topics names tof_points and the root coupling test passes [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/fakes.py`
- `nodes/mcp_server/tests/test_map_summary_tof.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`

#### Interfaces

- Consumes `tof-sector-summary` and `tof-points-filtered-topic`.
- New `RobotApi` method (name chosen by the builder) returning base_link points, stamp and frame or None.

#### Expected outcome

`get_map_summary` gives the agent a per-sector 3D obstacle and cliff view from the head ToF, or an explicit null.

#### Steps

- Write `tests/test_map_summary_tof.py`; run it; it fails.
- Add the topic setting, RobotApi method, ros_iface subscription, fake, tool output; rerun; run the verify command.
- README and client.yml; commit `feat(mcp_server): 3D obstacles in get_map_summary`.

#### Notes

- Deploying mcp_server kills a running agent session (auto-memory): the runbook checks first. mcp_server stays Python.
- No new tool, so the claude_agent tool lists stay unchanged.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on web-ui-obstacle-points-api

### S11: Web UI obstacle points layer

- Status: done
- Files: `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/map3d/obstaclePoints.ts`, `nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts`, `nodes/web_ui/frontend/src/map3d/layers.ts`, `nodes/web_ui/frontend/src/map3d/layers.test.ts`, `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`, `nodes/web_ui/frontend/src/hooks/subscription.ts`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 05be88c263ac186466242023076ecd1d2778d533

#### Requirement

Test first: `map3d/obstaclePoints.test.ts` and the new `layers.test.ts` cases fail before the code exists (D10, U5).

- `types.ts`: tab config fields `obstacle_cloud_topic`, `scan_points_topic`; payload types for roles `cloud` and
  `scan_points` matching the backend contract.
- `map3d/obstaclePoints.ts`: pure functions converting a payload into map-frame positions (apply `frame_pose`
  quaternion + translation to every point; scan points at z = 0 in the scan frame) and per-point colours (ToF by height
  band: cliff below floor, obstacle 0.06-0.6 m, other; lidar one colour); rejects payloads with an odd/mismatched length.
- `map3d/layers.ts`: one layer id `obstacles3d` labelled "3D obstacles (ToF + lidar)", default off; `layers.test.ts`
  asserts the label, default off, and that the layer depends on both topics.
- `map3d/sceneLayers.tsx`: draws both point sets as three.js `points` when the layer is on.
- `tabs/MapNavTab.tsx` / `hooks/subscription.ts`: both topics are requested only while the layer is on (on-demand), and
  released when it is turned off or the tab unmounts.
- `obstaclePoints.test.ts`: identity and 90 deg yaw + translation poses, scan conversion, colour bands, malformed
  payloads.

#### Acceptance conditions

- a1: payload points are transformed into map positions with frame_pose and coloured by band [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/obstaclePoints.test.ts]
- a2: one default-off layer 3D obstacles (ToF + lidar) drives both topics [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts]
- a3: topics are subscribed only while the layer is on [evidence: judgement]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.ts`
- `nodes/web_ui/frontend/src/map3d/obstaclePoints.test.ts`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/sceneLayers.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`

#### Interfaces

- Consumes `obstacle-points-payload` (backend `msg_serializer.py`).

#### Expected outcome

In the Map tab, toggling "3D obstacles (ToF + lidar)" shows the ToF points coloured by height and the lidar points in
the 3D scene; toggling it off stops both subscriptions.

#### Steps

- Write `obstaclePoints.test.ts` and the layer test; `cd nodes/web_ui/frontend && npm test` fails on them.
- Implement the pure module, the layer entry, the scene drawing and the on-demand wiring; run the verify command.
- Commit `feat(web_ui): 3D obstacles layer with ToF and lidar points`.

#### Notes

- Follow the existing layer and on-demand subscription patterns of the local costmap layer (`layers.ts`,
  `hooks/subscription.ts`, `MapNavTab.tsx`). If the LayersPanel needs a change to show the new layer, keep it minimal.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on web-ui-obstacle-points-api

### S12: Final suite and runbook

- Status: done
- Files: `tests/test_tof_obstacle_wiring.py`, `tests/README.md`, `scripts/node_cpu_compare.yaml`, `MEMORY.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`
- Verified head: 1f35ad68592fa438d2e1cd10f76606bad24846ac

#### Requirement

Test first: `tests/test_tof_obstacle_wiring.py` is written and seen failing on any break in the chain (D9, D12).

- `tests/test_tof_obstacle_wiring.py`: the topic chain is consistent across files - tof_obstacle_cloud remap output
  equals laser_filter `cloud_topic`; laser_filter `cloud_filtered_topic` equals every STVL source topic, both
  collision_monitor ToF sources, the web_ui map_nav `obstacle_cloud_topic` (client.yml and default.yaml) and the
  mcp_server `topics.tof_points` (client.yml and config default); arducam_tof `frame_id` is `tof_optical_frame`; the
  STVL and collision_monitor height bands agree; slam_toolbox still publishes map->odom (`transform_publish_period` > 0)
  and no other client node publishes it (U4); every new node/test is in `tests/README.md`.
- `scripts/node_cpu_compare.yaml`: entries `nav2_bringup`, `laser_filter`, `arducam_tof` (same unit before/after,
  `switch_time: 1` placeholder per the file's convention) with a comment that `ros2-tof_obstacle_cloud.service` is new
  and measured "after" only by the runbook query.
- `MEMORY.md`: one decision entry (ToF obstacles: point_cloud_xyz + laser_filter self filter + STVL local + collision
  monitor sources; height bands instead of obstacles_detection; lidar layers kept separate; fail-safe on ToF outage;
  CPU budget 0.15 core).
- The full verify command passes; the report quotes every summary line.
- The runbook below is complete and is executed by the orchestrator after merge (not by the implementer).

#### Acceptance conditions

- a1: the ToF topic chain, frames, bands and map->odom ownership are consistent [evidence: test: tests/test_tof_obstacle_wiring.py]
- a2: node_cpu_compare.yaml has the nav2_bringup, laser_filter and arducam_tof entries [evidence: command: grep -q 'name: nav2_bringup' scripts/node_cpu_compare.yaml && grep -q 'name: laser_filter' scripts/node_cpu_compare.yaml && grep -q 'name: arducam_tof' scripts/node_cpu_compare.yaml && uv run pytest tests/test_node_cpu_compare.py -q]
- a3: MEMORY.md records the ToF obstacle decision [evidence: command: grep -q 'spatio_temporal_voxel_layer' MEMORY.md]
- a4: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a5: the runbook covers ToF/TF preconditions, obstacle above and below the lidar plane, cliff stop at low speed, empty floor, lidar survival and CPU before/after [evidence: judgement]
- a6: added CPU <= 0.15 core on the Pi 5, measured in Grafana after deploy and recorded, decided by the user on 2026-10-18 [evidence: judgement]

#### Files

- `tests/test_tof_obstacle_wiring.py`
- `tests/README.md`
- `scripts/node_cpu_compare.yaml`
- `MEMORY.md`

#### Interfaces

- None new; `tests/test_tof_obstacle_wiring.py`.

#### Expected outcome

All suites green with an end-to-end wiring check; the orchestrator deploys and verifies on the robot with the runbook.

#### Steps

Implementer:
- Write `tests/test_tof_obstacle_wiring.py`; run `uv run pytest tests/test_tof_obstacle_wiring.py -q`; fix real breaks;
  add the cpu-compare entries and the MEMORY.md line; run the full verify command and quote each summary line; commit
  `test: ToF obstacle pipeline wiring`.

Orchestrator runbook (after merge, robot reachable; every deploy via the ansible-deploy skill, logged to `.logs/`):
1. Preconditions: pan/tilt epic deployed (`/head/joint_states` live, `tof_optical_frame` in TF: ansible ad-hoc
   `ros2 run tf2_ros tf2_echo base_link tof_optical_frame` for 3 s, no manual SSH); ToF streaming: Grafana MCP
   `rate(tof_frames_published_total{host="client"}[1m])` about 10. If either fails, stop and report the robot steps
   pending on it.
2. CPU before: note `date +%s` as switch time; record 30 min of `rate(container_cpu_usage_seconds_total{host="client",
   id="/system.slice/<unit>"}[1m])` for ros2-nav2_bringup, ros2-laser_filter, ros2-arducam_tof (robot idle, nodes up),
   and client total CPU from the "Robot resources" dashboard panel query.
3. Check no agent session is running (claude_agent `GET http://client.ros2.lan:18300/api/state`); if one is, ask before
   deploying mcp_server. Deploy: `./scripts/deploy-nodes.sh client tof_obstacle_cloud laser_filter nav2_bringup 2>&1 |
   tee .logs/deploy-client-tof-obstacles.log`, then `./scripts/deploy-nodes.sh client web_ui mcp_server 2>&1 | tee
   .logs/deploy-client-tof-consumers.log`. Units active; nav2 lifecycle active (no STVL plugin load error in journal via
   the deploy log / Loki).
4. Rates (topic_scraper_api enabled per CLAUDE.md, `scripts/topic_scraper_collect.py --source
   client=http://client.ros2.lan:18100 --select /tof/points_filtered:.header.stamp --select /scan_filtered:.header.stamp
   --interval 0.1`): `/tof/points_filtered` about 10 Hz, `/scan_filtered` unchanged rate.
5. FOV: read `/tof/camera_info` fx, fy (scraper); compute hfov/vfov; if below 1.0 / 0.78 rad, lower the STVL FOV, commit,
   redeploy nav2_bringup.
6. Empty floor: robot stationary, head home pose, 60 s: no ToF cells in `/local_costmap/costmap` on the floor ahead and
   no CliffStop/StopBox action in `/collision_monitor_state` (web UI 3D obstacles layer on, Map tab, Claude in Chrome
   screenshot).
7. Above the lidar plane: a box edge or table top at 0.3-0.5 m height, 1.0 m ahead (nothing at 0.20 m): cells appear in
   the local costmap within 1 s (agent `get_topdown_view` costmap layer and `get_map_summary.obstacles_3d`); remove it:
   cells gone within ~15 s. Below the lidar plane: an object 5-15 cm high: same check.
8. Lidar survival (U5): a lidar-visible obstacle outside the ToF view (behind the robot) stays marked while the ToF
   looks at empty space for 30 s.
9. Cliff at low speed: with a spotter and a physical stop, approach a 10-20 cm step down (or a platform edge) with the
   head in home pose at <= 0.1 m/s (Nav2 goal beyond the edge or `move_relative` with speed limit): the robot stops
   before the front reaches the edge (CliffStop in `/collision_monitor_state`); a backward command still moves it.
10. Outage: `ros2 param set /collision_monitor tof_obstacles.enabled false` and true again works; stopping arducam_tof
    stops Nav2 motion ("invalid source") and restarting it resumes.
11. CPU after: from switch + 10 min, 30 min windows as in step 2 plus ros2-tof_obstacle_cloud; run `uv run python -m
    scripts.node_cpu_compare` with the switch time set (local edit, not committed); added CPU (new unit + deltas) <= 0.15
    core mean and EKF `/odometry/filtered` still ~30 Hz; record the table in the record. If over budget, report with the
    numbers (do not tighten cgroups).
12. Disable topic_scraper_api again if it was disabled before.

#### Notes

- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- If the robot is offline, runbook steps are reported pending on the robot, with everything else done.

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on web-ui-obstacle-points-layer, map-summary-tof-obstacles

- 2026-10-27 done: superseded by realsense-d435-head-switchov/nav2-live-check-on-d435

- 2026-10-27 actuals: no agent records

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
