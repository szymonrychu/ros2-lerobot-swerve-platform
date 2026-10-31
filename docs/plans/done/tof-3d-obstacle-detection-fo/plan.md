---
title: ToF 3D obstacle detection for Nav2
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
owner: Szymon Richert
rung: 1
---

# ToF 3D obstacle detection for Nav2

## What and why

What: Head ToF depth becomes 3D obstacles and cliffs in the Nav2 local costmap and collision monitor
How: Depth to cloud with head TF, ground segmentation, STVL layer, collision_monitor sources, CPU budget
Why: The lidar alone misses overhangs, low obstacles and cliffs

Nav2 sees only the 2D lidar today. The head ToF (240x180 depth, ~10 Hz, up to 4 m) on the pan/tilt head adds 3D obstacle perception onboard the Pi 5: obstacles above and below the lidar plane (overhangs, low objects, cliffs) reach the local costmap and the collision monitor. The client CPU is already 80-90% busy, so the pipeline must be cheap.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: ToF depth -> point cloud in tof_optical_frame (head TF from the pan/tilt joint states) -> ground/obstacle segmentation (rtabmap_util point_cloud_xyz + obstacles_detection, or the cheapest equivalent proven on the Pi 5) -> a 3D obstacle layer (spatio_temporal_voxel_layer) in the Nav2 local costmap, with decay so stale voxels clear.
- 1.2: Nav2 collision_monitor gains a depth point-cloud source and a below-floor (cliff / negative obstacle) source.
- 1.3: Data published only when real ToF frames and TF exist (no placeholder data); obstacles marked only within the ToF's valid range and confidence.
- 1.4: Packages added to the nav2_bringup apt list (STVL, rtabmap_util or depth_image_proc as chosen); configs, nav-stack static tests, READMEs; CPU measured on the robot before/after (Grafana/Mimir) within an agreed budget.
- 1.5: Web UI: obstacle voxels/cloud optionally visible in the 3D view; agent can query nearby 3D obstacles if cheap.
- 1.6: 2.5D meaning: a ground robot that avoids 3D obstacles on the 2D map; no 3D path planning.

## Agreed scope

Rung 1, migrated from the epic format: the epic's goals were its agreed scope.

## Non-goals

- Building or storing a 3D map (next epic).
- Outdoor elevation mapping.
- Replacing slam_toolbox or the lidar pipeline.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Cloud = rtabmap_util point_cloud_xyz as an apt node; height bands instead of obstacles_detection

New node type and entry `tof_obstacle_cloud` (config-only dir `nodes/tof_obstacle_cloud/`): `ros2 run rtabmap_util
point_cloud_xyz` with `--params-file nodes/tof_obstacle_cloud/config/point_cloud_xyz.yaml`, remaps
`depth/image:=/tof/depth/image_raw`, `depth/camera_info:=/tof/camera_info`, `cloud:=/tof/points`,
`__node:=tof_obstacle_cloud`. Params: `decimation 4` (60x45 = 2700 pixels), `min_depth 0.2`, `max_depth 3.0` (inside
the 4 m far range; ToF error grows with range), `voxel_size 0.05`, `filter_nans true`, `approx_sync false` (depth and
camera_info share one header), `noise_filter_radius 0.1` / `noise_filter_min_neighbors 2` (flying pixels at depth
edges). It runs at the ToF's 10 Hz; STVL consumes the latest cloud per 5 Hz costmap update. Ground segmentation is a
height band in the consumers (floor z = 0 in base_link/odom), not `obstacles_detection` (normals + clustering) (my
call: the floor is flat indoors per U2, and the band costs nothing). Alternatives: depth_image_proc (no decimation,
43k-point organised cloud with NaNs, needs image_proc resize for decimation: two components, more points through DDS
and Python); obstacles_detection (more CPU, robust to small TF tilt errors; the fallback if the runbook shows floor
marks). Serves G1, G3, U1, U2.

### D2: The ToF self filter lives in laser_filter, sharing its runtime self polygon

`laser_filter` additionally subscribes `/tof/points`, transforms each point to `polygon_frame` (base_link) with the TF
at the cloud's stamp, drops points whose (x, y) is inside the current self polygon and whose z <= `cloud_self_max_height_m`
(1.0 m), and publishes the rest unchanged (same frame, same stamp, xyz float32) on `/tof/points_filtered`. The polygon
is the same `polygon` parameter mcp_server already sets (static footprint + 1 cm, or the dynamic footprint with the
released arm shadow and carried object), so mcp_server needs no change. TF lookups for clouds never block scan
filtering (timeout 0 with one pending newest-wins slot retried until `cloud_transform_timeout_s`, then dropped and
counted). Alternatives: a static pcl_ros CropBox (cannot follow the released arm), a new node (one more rclpy process
and a second mcp_server parameter target), a min-range crop (loses near obstacles, still sees the released arm). (my
call) Serves G1, G3.

### D3: STVL `tof_voxel_layer` in the local costmap only

Local costmap `plugins: [obstacle_layer, tof_voxel_layer, keepout_filter, inflation_layer]`. Layer: `plugin
spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`, `voxel_decay 15.0`, `decay_model 0`, `voxel_size 0.05`,
`track_unknown_space false`, `mark_threshold 0`, `update_footprint_enabled false`, `combination_method 1`,
`publish_voxel_map false`, `transform_tolerance 0.3`, `mapping_mode false`, `observation_sources: tof_mark
tof_cliff_mark tof_clear`, all on `/tof/points_filtered`:
- `tof_mark`: marking, `min_obstacle_height 0.06`, `max_obstacle_height 0.6` (robot height with the head; overhangs
  higher than the robot are not obstacles), `obstacle_range 2.0`, `filter voxel`, `voxel_min_points 0`,
  `observation_persistence 0.0`, `clear_after_reading true`, `expected_update_rate 0.0`.
- `tof_cliff_mark`: marking, `min_obstacle_height -1.0`, `max_obstacle_height -0.10`, `obstacle_range 2.0`, same
  buffer settings.
- `tof_clear`: clearing, `model_type 0`, `min_z 0.2`, `max_z 3.0`, `horizontal_fov_angle 1.0`, `vertical_fov_angle
  0.78` (just under the ToF's 70 deg diagonal FOV at 4:3: 58.5 x 45.6 deg, so clearing never reaches outside the real
  view), `decay_acceleration 1.0`.
`expected_update_rate 0.0`: a ToF dropout does not make the costmap "not current" (the collision monitor handles
outages, D5). Global costmap unchanged (my call: the goal names the local costmap; CPU). Serves G1, G6, U1, U2.

### D4: Lidar coverage kept and separated from ToF clearing (U5)

`obstacle_layer` with the `/scan_filtered` LaserScan source stays in both costmaps unchanged; the collision monitor
`scan` source stays and stays in StopBox. Costmap layers are separate grids combined by max, so STVL frustum clearing
(D3) only decays ToF voxels and never touches lidar-marked cells, and lidar raytracing never clears ToF voxels (they
decay in 15 s). The lidar covers 360 deg / beyond the 2-3 m ToF band; the ToF adds height detail inside its view.
Serves U5.

### D5: collision_monitor ToF sources, StopBox and a CliffStop approach polygon

Sources `tof_obstacles` (`type pointcloud`, `/tof/points_filtered`, `min_height 0.06`, `max_height 0.6`, `min_range
0.0`) and `tof_cliff` (`min_height -1.0`, `max_height -0.10`). `observation_sources: [scan, tof_obstacles,
tof_cliff]`; `StopBox.sources_names: [scan, tof_obstacles]`; `ArmStop.sources_names: [scan]` (my call: ToF edge noise
around the released arm must not freeze it; ToF obstacles near the arm still reach the costmap, whose footprint includes
the arm shadow); new `CliffStop` (`type polygon`, `action_type approach`, `time_before_collision 1.2`,
`simulation_time_step 0.05`, `min_points 4`, `sources_names [tof_cliff]`, points = footprint + `cliff_margin_m`, D6).
Approach lets the robot back away from a cliff while it stops motion toward it. ToF sources keep the node
`source_timeout 1.0`: a ToF or head-TF outage stops Nav2 motion (fail-safe, my call; question Q1). Limitation: the
monitor sees a cliff only while the ToF looks at it (head home pose looks forward-down); the costmap keeps cliff
voxels for 15 s. Serves G2, U1.

### D6: CliffStop polygon generated by gen_nav_footprint.py

`scripts/gen_nav_footprint.yaml` gains `cliff_margin_m: 0.25` (with a 0.35-0.40 m high ToF, the lower floor of a 15 cm
step 0.5 m ahead first becomes visible about 0.2 m beyond the edge, so the polygon reaches past it); the generator
writes `offset_polygon(footprint, cliff_margin_m)` into a new `cliffstop` marker region inside the `CliffStop` block of
`nav2_params.yaml`. The block is added inert (not listed in `polygons`) until D5's task enables it. Serves G2.

### D7: Placeholder-data rule for clouds

No node of this pipeline has a timer that publishes: point_cloud_xyz publishes only from a synced real ToF frame,
laser_filter only after a real cloud with TF at its stamp, never an unfiltered or untransformed cloud. A frame whose
every pixel is invalid gives an empty cloud with the real stamp; it is published, because it truthfully states "no
valid returns in this frame" and keeps the collision monitor's freshness check meaningful (a suppressed cloud would
read as a ToF outage and stop the robot) (my call, departs from the prior-art fix-as-ac "publish nothing"). No cloud
is published without a frame or without TF. Serves G3.

### D8: Packages

`tof_obstacle_cloud` type: `ros-jazzy-rtabmap-util`; `nav2_bringup` type: `ros-jazzy-spatio-temporal-voxel-layer`;
`laser_filter` type: `ros-jazzy-sensor-msgs-py` (point_cloud2 read/create). depth_image_proc is not used (D1). New type
tiered normal (cpu_weight 500, io_weight 500, oom_score_adjust -500), MemoryLow 64M estimate, MemoryMax 192M, listed in
`tests/test_unit_cgroups.py` NORMAL. Serves G4.

### D9: TF and map->odom ownership

No new TF publisher: the head chain comes from the pan/tilt epic's robot_state_publisher (U3); clouds keep the ToF
capture stamp and consumers look TF up at that stamp (STVL message filter, collision monitor `getTransform`, laser_filter
D2). slam_toolbox stays the only map->odom owner and EKF odom->base_link (U4). Serves U3, U4, G1.

### D10: Web UI "3D obstacles" layer: ToF cloud plus lidar points

map_nav tab fields `obstacle_cloud_topic` (`/tof/points_filtered`, role `cloud`) and `scan_points_topic`
(`/scan_filtered`, role `scan_points`), subscribed on demand only while the layer is on. Backend serialisers send
`{stamp, frame, frame_pose_in_map, points}` (points as a flat float list in the sensor frame, ToF <= 2000 points, scan
<= 720, downsampled evenly, non-finite dropped; no message when the TF to map is unknown). One frontend toggle "3D
obstacles (ToF + lidar)", default off, draws both as three.js points (ToF coloured by height, lidar one colour) (U5).
Serves G5, U5.

### D11: Agent: `get_map_summary` reports nearby 3D obstacles

mcp_server subscribes `/tof/points_filtered` lazily (latest only, like local_costmap), transforms to base_link at the
stamp, and `get_map_summary` gains `obstacles_3d`: per 45 deg sector the nearest ToF obstacle point (distance, min/max
height) and nearest cliff point, plus the cloud age; `obstacles_3d` is null when no cloud is fresher than
`timeouts`-style staleness or TF is missing (no placeholder). No new tool (my call: no claude_agent tool-list sync).
Serves G5.

### D12: CPU budget and robot verification

Budget: the ToF pipeline adds at most 0.15 core mean on the client (ros2-tof_obstacle_cloud.service plus the
before/after deltas of ros2-laser_filter.service and ros2-nav2_bringup.service), measured from Mimir
`rate(container_cpu_usage_seconds_total{host="client",id="/system.slice/<unit>"}[1m])` over 30 min windows (robot idle,
nodes up) before and after the deploy, and the EKF keeps its 30 Hz (Q2: decided by the user). Runbook in the
final task: obstacle above and below the lidar plane in the costmap, cliff stop at <= 0.1 m/s with a spotter, empty
floor stays clear, lidar cells survive ToF clearing, CPU before/after. Serves G4.

## Open questions

none

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| discovery (codebase) | ran: nav3d-codebase-brief.md | integration points nav2_params local costmap / collision_monitor, client.yml apt list, tests/test_nav_stack_config.py, web_ui roles, mcp lazy topics | offload to the Pi 4b (WiFi, server headroom ~1.1 GB) |
| prior art | ran: nav3d-prior-art.md | point_cloud_xyz -> STVL + collision monitor (rtabmap_demos turtlebot3_rgbd_scan.launch.py; STVL README), below-floor source + hard stop (answers.ros.org 289171, 403754) | nvblox, elevation_mapping_cupy (CUDA), Kimera/voxblox (ROS 1), ORB-SLAM3/OpenVINS (no Jazzy), Bonxai/EasyNav, depthimage_to_laserscan (loses height) |
| research-docs | ran by the planner (Verified facts above): packages.ros.org arm64 index, Nav2 1.3.13 collision_monitor source, STVL jazzy README/source, rtabmap_ros 0.23.13 point_cloud_xyz source | invalid-source stop semantics (D5), per-polygon sources_names (D5), negative height bands (D3, D5), STVL convex footprint clearing off (D3), empty-cloud behaviour (D7) | obstacles_detection (D1) |
| research-code-internal | skipped: the codebase brief plus planner reads found the reusable parts (laser_filter self polygon, gen_nav_footprint offset_polygon, msg_serializer downsample/transform helpers, mcp perception sectors) | reuse listed in Research inputs | - |
| research-papers | skipped: no algorithmic core beyond standard voxel decay / height bands | - | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| prior-art, reuse | arducam_tof `bridge.py:TofBridge` topics /tof/depth/image_raw, /tof/camera_info | tof-obstacle-cloud-node |
| prior-art, reuse | `nav2_params.yaml:local_costmap.obstacle_layer` | nav2-tof-voxel-layer (kept unchanged, U5) |
| prior-art, reuse | `nav2_params.yaml:collision_monitor.observation_sources` | collision-monitor-tof-stop |
| prior-art, reuse | slam_toolbox map->odom owner `slam_params.yaml:transform_publish_period` | tof-obstacle-cloud-node, final-suite-and-runbook |
| prior-art, reuse | `tests/test_nav_stack_config.py:test_nav2_keepout_filter_sits_right_before_inflation` | nav2-tof-voxel-layer |
| prior-art, do-not-build | custom 3D voxel costmap layer -> STVL | nav2-tof-voxel-layer |
| prior-art, do-not-build | custom depth-to-cloud + ground segmentation -> point_cloud_xyz (+ obstacles_detection) | tof-obstacle-cloud-node (point_cloud_xyz); obstacles_detection rejected: height bands suffice on a flat floor (D1) |
| prior-art, do-not-build | custom 3D SLAM -> rtabmap_ros | rejected: non-goal, next epic rtab-map-rgb-d-3d-mapping-on |
| prior-art, do-not-build | custom depth safety stop -> collision monitor pointcloud source | collision-monitor-tof-stop |
| prior-art, do-not-build | custom OctoMap server | rejected: non-goal (no 3D map in this epic) |
| prior-art, fix-as-ac | two map->odom publishers | tof-obstacle-cloud-node, final-suite-and-runbook |
| prior-art, fix-as-ac | STVL used for planar lidar | nav2-tof-voxel-layer |
| prior-art, fix-as-ac | stale head TF smearing voxels | laser-filter-tof-self-filter, tof-obstacle-cloud-node, nav2-tof-voxel-layer |
| prior-art, fix-as-ac | footprint clearing on the non-convex H | nav2-tof-voxel-layer |
| prior-art, fix-as-ac | placeholder data rule (nothing published when no valid points) | laser-filter-tof-self-filter, tof-obstacle-cloud-node; "publish nothing on an empty frame" rejected: an empty cloud from a real frame is real data and keeps the monitor's freshness check meaningful (D7) |
| prior-art, fix-as-ac | raw 30 fps RGB over WiFi to the Pi 4b | rejected: no offload in this epic (U1 onboard) |
| prior-art, fix-as-ac | autofocus changing intrinsics | rejected: the RGB camera is not used by this pipeline |
| prior-art, ideas | adopted: ToF -> point_cloud_xyz -> obstacles_detection -> STVL + collision monitor | tof-obstacle-cloud-node, nav2-tof-voxel-layer, collision-monitor-tof-stop (obstacles_detection replaced by height bands, D1) |
| prior-art, ideas | adopted: rtabmap RGB-D offload, publish_tf false | rejected here: next epic |
| prior-art, ideas | adopted: below-floor source + hard stop | cliff-stop-polygon-generator, collision-monitor-tof-stop, nav2-tof-voxel-layer |
| prior-art, ideas | adopted (later): grid_map / FastDEM 2.5D elevation | rejected: non-goal (outdoor elevation) |
| prior-art, ideas | rejected: nvblox, elevation_mapping_cupy | rejected: CUDA only |
| prior-art, ideas | rejected: Kimera, voxblox, ANYbotics elevation_mapping | rejected: ROS 1 / unmaintained |
| prior-art, ideas | rejected: ORB-SLAM3, OpenVINS, VINS-Fusion | rejected: no Jazzy / unsuited sensors |
| prior-art, ideas | rejected: Bonxai, EasyNav | rejected: no ROS 2 release / replaces Nav2 |
| prior-art, ideas | rejected: depthimage_to_laserscan as main path | rejected: loses height and cliffs |
| codebase brief, 3 | local costmap plugin order, client.yml nav2 apt list, test_nav_stack_config pins | nav2-tof-voxel-layer |
| codebase brief, 3 | web_ui new WS role + TabConfig field + map3d layer, on-demand | web-ui-obstacle-points-api, web-ui-obstacle-points-layer |
| codebase brief, 3 | mcp_server lazy topic + perception helper | tof-obstacle-sector-summary, map-summary-tof-obstacles |
| codebase brief, 4 | client CPU 80-90 %, Mimir per-unit CPU | final-suite-and-runbook |
| codebase brief, unknowns | ToF capture on the Pi 5, Jazzy apt availability | final-suite-and-runbook (runbook step), Verified facts (apt) |
| user U5 | lidar keeps its layers/source; ToF clearing never clears lidar marks; web UI shows lidar points with the cloud | nav2-tof-voxel-layer, collision-monitor-tof-stop, web-ui-obstacle-points-api, web-ui-obstacle-points-layer |

## Irreversible operations

none (config and code changes, apt installs, redeploys; all revertible by git revert and redeploy).

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-27)

Closed 2026-10-27 at the D435 switchover gate (Q3, user approved). Every delivered task was already merged to main under the earlier flow; the remaining runbook/final-suite task was superseded by a realsense-d435-head-switchov task (dated note on the task). No new code in this close. Root suite on main 2026-10-27: 2156 passed, 9 skipped; test-ansible 0 failures.

### design.md: Summary

The head ToF (240x180 depth, 10 Hz, `tof_optical_frame`, TF from the pan/tilt joint states) becomes a decimated point
cloud on the Pi 5 (`rtabmap_util/point_cloud_xyz`, apt), is cleaned of the robot's own body by the existing
`laser_filter` self polygon, and feeds two consumers: a `spatio_temporal_voxel_layer` (STVL, apt) in the Nav2 local
costmap (obstacles 6-60 cm above the floor and below-floor cliff points, 15 s decay, frustum clearing) and two new
`collision_monitor` pointcloud sources (3D obstacles into StopBox, cliff points into a new CliffStop approach polygon).
Ground segmentation is a height band on the flat floor (base_link z = 0 is the floor), not a normals segmenter. The lidar
keeps its obstacle layer in both costmaps and its collision source; ToF clearing only touches the ToF voxel grid. The
web UI 3D view gets an optional "3D obstacles" layer (ToF cloud plus lidar points) and the agent's `get_map_summary`
reports nearby 3D obstacles and cliffs. Added client CPU is measured in Mimir before/after against a 0.15 core budget.

### design.md: Intent and why

Nav2 sees only the RPLidar plane (z 0.20 m). Overhangs (table edges, shelves), low objects (below 0.20 m) and cliffs
(stairs down, platform edges) are invisible to the costmaps and the collision monitor. The ToF on the new pan/tilt head
looks forward and down in its home pose and covers exactly that gap within ~4 m. The client Pi 5 is 80-90 % busy, so the
pipeline must be the cheapest proven one and its cost must be measured, not assumed.

### design.md: Goals

- G1: ToF depth -> point cloud in `tof_optical_frame` (head TF from the pan/tilt joint states) -> height-band
  ground/obstacle split -> STVL layer in the local costmap with decay (D1, D2, D3, D9).
- G2: collision_monitor gains a ToF obstacle pointcloud source and a below-floor (cliff) source (D5, D6).
- G3: clouds only from real ToF frames with TF; points only within the configured valid depth range and the bridge's
  confidence threshold (D1, D7).
- G4: apt packages in the node types, configs, static tests, READMEs; CPU measured before/after in Mimir within the
  budget (D8, D12).
- G5: web UI 3D view can show the obstacle cloud (with lidar points); the agent can query nearby 3D obstacles (D10, D11).
- G6: 2.5D: a ground robot avoiding 3D obstacles on the 2D map, no 3D planning (D3).
- U1-U5: the user decisions in `approvals.json` (onboard Pi 5 ToF perception into local costmap + collision monitor +
  cliffs; 2.5D; head TF from joint states; slam_toolbox keeps map->odom; lidar extends the ToF).

### design.md: Non-goals

- Building or storing a 3D map (next epic `rtab-map-rgb-d-3d-mapping-on`), outdoor elevation mapping.
- Replacing slam_toolbox, the lidar pipeline or the lidar obstacle layers.
- ToF voxels in the global costmap (the goal names the local costmap; decided by the user, Q3).
- Detecting a cliff from missing floor returns (only below-floor points are a cliff signal; see Assumptions).
- Fixing Arducam ToF capture on the Pi 5 kernel (tracked by the Jetson-removal epic; verified in the runbook).

### design.md: Anti-goals

- The lidar loses coverage: any ToF clearing erasing lidar-marked cells, or the scan source leaving a costmap or the
  collision monitor (U5).
- The robot body, the parked/released arm or a carried object marked as an obstacle by the ToF (robot stuck in its own
  footprint, false stops).
- Floor noise marked as obstacles on an empty floor, or false cliff stops on a normal floor.
- Unmeasured CPU growth on the client; a second map->odom publisher; any synthetic cloud.

### design.md: Constraints

- CLAUDE.md: no placeholder data on topics; Ansible `ros2_nodes` schema kept in sync; deploy only via
  `scripts/deploy-nodes.sh` with the ansible-deploy skill, output to `.logs/`; established libraries first; config files
  over CLI; type hints, docstrings, constants at the top; tests for every new function; READMEs and `tests/README.md`.
- MEMORY.md / auto-memory: cgroups give robot nodes weights and protection, not tight caps (new node type: normal tier,
  cpu_weight 500, MemoryLow estimate, MemoryMax >= 2x); web_ui and mcp_server stay Python; deploying mcp_server kills a
  running agent session (check first).
- Pan/tilt epic end state: robot_state_publisher with the head URDF publishes base_link -> ... -> `tof_optical_frame`
  from `/head/joint_states`; arducam_tof publishes `/tof/depth/image_raw` (16UC1 mm, 0 = invalid: confidence below
  `confidence_threshold` 30, non-finite, <= 0), `/tof/camera_info` with one shared header per frame, 10 Hz, range mode
  far (4 m) (`nodes/bridges/arducam_tof/arducam_tof/bridge.py:1-13`, client.yml arducam_tof entry).
- base_link z = 0 is the floor (`nodes/web_ui/urdf/robot.urdf:3-4`); local costmap is odom-framed, rolling 3x3 m,
  2.5 cm, update 5 Hz, transform_tolerance 0.3 (`nav2_params.yaml:171-186`).
- Footprint-derived polygons come from `scripts/gen_nav_footprint.py` marker regions; never hand-edited.
- Robot offline during planning: no CPU baseline was readable (Mimir returned no client series on 2026-10-18).

#### Verified facts (research-docs lens, 2026-10-18)

- packages.ros.org `noble/main/binary-arm64/Packages` (Jazzy): `ros-jazzy-spatio-temporal-voxel-layer 2.5.5-1noble.20260921`
  (deps include libopenvdb10.0, PCL 1.14, nav2-costmap-2d), `ros-jazzy-rtabmap-util 0.23.13-1noble.20261003` (pulls
  `ros-jazzy-rtabmap` 0.23.13 core, ~22 MB plus Qt5/VTK/gtsam/libpointmatcher runtime libs), `ros-jazzy-rtabmap-ros
  0.23.13`, `ros-jazzy-depth-image-proc 5.0.13`, `ros-jazzy-nav2-collision-monitor 1.3.13`, `ros-jazzy-pcl-ros 2.6.5`,
  `ros-jazzy-robot-state-publisher 3.3.4`, `ros-jazzy-depthimage-to-laserscan 2.5.1`. All available on arm64.
- Nav2 collision_monitor 1.3.13 source (`nav2_collision_monitor/src/pointcloud.cpp:68-140`): `pointcloud` source params
  `min_height` (default 0.05), `max_height` (0.5), `min_range` (0.0, from the sensor origin), `use_global_height`
  (false); heights are compared in the base frame after the TF lookup, so negative bands work (below-floor source);
  non-finite points fall out of the band comparison. Subscribes with SensorDataQoS.
- `source.cpp:71-90` and `collision_monitor_node.cpp:436-446`: per-source `enabled` and `source_timeout` (defaults to
  the node `source_timeout`); when an enabled source with non-zero timeout has no data, stale data (> timeout) or no TF,
  the monitor publishes a STOP ("invalid source"). `source_timeout: 0.0` disables that stop for the source.
- `polygon.cpp:355-440`: per-polygon `sources_names` (default: all observation_sources), `min_points` (default 4),
  `action_type` stop/slowdown/limit/approach, `footprint_topic` (approach). A polygon not in `polygons` is not loaded.
- STVL jazzy README and `spatio_temporal_voxel_layer.cpp:88-211`: layer params `voxel_decay`, `decay_model`
  (0 linear), `voxel_size`, `track_unknown_space`, `mark_threshold`, `update_footprint_enabled` (clears with
  `setConvexPolygonCost`, i.e. convex, same problem as ObstacleLayer footprint clearing), `combination_method` (1 max),
  `publish_voxel_map`, `transform_tolerance`, `mapping_mode`; per source `topic`, `sensor_frame`, `marking`,
  `clearing`, `min_obstacle_height`/`max_obstacle_height` (global frame, i.e. odom z), `obstacle_range`,
  `observation_persistence`, `expected_update_rate`, `clear_after_reading`, `filter` (voxel/passthrough/none),
  `voxel_min_points`, `min_z`/`max_z`, `vertical_fov_angle`, `horizontal_fov_angle`, `decay_acceleration`,
  `model_type` (0 depth camera frustum). The README advises against STVL for planar lidars.
- rtabmap_util `point_cloud_xyz` (rtabmap_ros 0.23.13 `point_cloud_xyz.cpp:53-363`): params `decimation`, `min_depth`,
  `max_depth`, `voxel_size`, `noise_filter_radius`, `noise_filter_min_neighbors`, `filter_nans`, `approx_sync`,
  `qos`, `roi_ratios`; topics `depth/image`, `depth/camera_info` in, `cloud` out; accepts 16UC1; publishes only from
  the synced depth+camera_info callback (no timer), copying the depth header stamp and frame_id; it publishes an empty
  cloud when every pixel is invalid.

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 point_cloud_xyz node, height bands | tof-obstacle-cloud-node |
| D2 laser_filter ToF self filter | laser-filter-tof-self-filter |
| D3 STVL local layer | nav2-tof-voxel-layer |
| D4 lidar kept, separate clearing | nav2-tof-voxel-layer, collision-monitor-tof-stop |
| D5 collision monitor sources and CliffStop | collision-monitor-tof-stop |
| D6 generated CliffStop polygon | cliff-stop-polygon-generator |
| D7 placeholder rule | tof-obstacle-cloud-node, laser-filter-tof-self-filter |
| D8 packages and tier | tof-obstacle-cloud-node, laser-filter-tof-self-filter, nav2-tof-voxel-layer |
| D9 TF ownership | tof-obstacle-cloud-node, final-suite-and-runbook |
| D10 web UI layer | web-ui-obstacle-points-api, web-ui-obstacle-points-layer |
| D11 agent summary | tof-obstacle-sector-summary, map-summary-tof-obstacles |
| D12 CPU budget and runbook | final-suite-and-runbook |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `tof-points-topic` | tof-obstacle-cloud-node | laser-filter-tof-self-filter | `ansible/group_vars/client.yml` (tof_obstacle_cloud remap `cloud:=/tof/points`) |
| `tof-points-filtered-topic` | laser-filter-tof-self-filter | nav2-tof-voxel-layer, collision-monitor-tof-stop, web-ui-obstacle-points-api, map-summary-tof-obstacles | `nodes/laser_filter/config/footprint_filter.yaml` (`cloud_filtered_topic`) |
| `cliffstop-polygon` | cliff-stop-polygon-generator | collision-monitor-tof-stop | `scripts/gen_nav_footprint.py` (`cliffstop` region) |
| `obstacle-points-payload` | web-ui-obstacle-points-api | web-ui-obstacle-points-layer | `nodes/web_ui/web_ui/msg_serializer.py` (`serialize_point_cloud`, `serialize_scan_points`) |
| `tof-sector-summary` | tof-obstacle-sector-summary | map-summary-tof-obstacles | `nodes/mcp_server/mcp_server/perception.py` (`tof_obstacle_sectors`) |

### design.md: Review focus

- Height bands vs real ToF noise: floor marks (0.06 m) and false cliffs (-0.10 m); the runbook's empty-floor check.
- collision_monitor invalid-source STOP: the ToF sources make a ToF/TF outage stop motion; decided by the user (Q1).
- StopBox/ArmStop/CliffStop `sources_names` exactly as D5; CliffStop not in `polygons` until collision-monitor-tof-stop.
- laser_filter: cloud TF lookups never delay `/scan_filtered`; dropped clouds counted; output keeps frame and stamp.
- STVL `update_footprint_enabled false` and FOV under the true ToF FOV; lidar ObstacleLayer untouched (U5).
- No new TF publisher; no publisher without a real frame.

### design.md: Left to the builder

- laser_filter internals: callback groups or timer for the pending cloud, as long as scans are never delayed by a TF
  wait and the ACs hold.
- Web UI point styling (size, palette) and downsampling method, within the payload caps of D10.
- mcp_server staleness threshold for `obstacles_3d` (reuse an existing timeouts field or add one, default <= 1.0 s).
- Exact MemoryLow/MemoryMax estimates within the cgroup test rules (re-measured after deploy).

### design.md: Assumptions and unknowns

- Arducam ToF streams on the Pi 5 at ~10 Hz by the time this deploys (Jetson-removal epic tracks capture). Confirmed
  by the runbook (`tof_frames_published_total` rate) before any other step; if not, the robot steps are reported
  pending on the ToF.
- The pan/tilt epic delivers `tof_optical_frame` under base_link from joint states at >= 10 Hz; confirmed in the
  runbook with tf2_echo through an ansible ad-hoc command.
- ToF FOV 70 deg diagonal (Arducam spec); the runbook computes it from `/tof/camera_info` fx/fy and the clearing FOV is
  lowered if needed.
- Thresholds (0.06 / -0.10 m, 0.6 m robot height, 2.0 m range, 0.25 m cliff margin) are starting values tuned on the
  robot in the runbook.
- A cliff into a void with no returns (or a very dark lower floor) gives no below-floor points and is not detected;
  documented as a limitation in the nav2_bringup README.
- Decided by the user on 2026-10-18 (all as recommended): (Q1) a ToF or head-TF outage stops the robot while Nav2 drives
  (collision monitor stale-source stop); (Q2) CPU budget <= 0.15 core on the Pi 5, measured in Grafana after deploy;
  (Q3) ToF voxels only in the local costmap, never the global one.

### approvals.json

- G1: ToF depth -> point cloud in tof_optical_frame (head TF from the pan/tilt joint states) -> ground/obstacle segmentation (cheapest proven equivalent of point_cloud_xyz + obstacles_detection) -> a spatio_temporal_voxel_layer in the Nav2 local costmap, with decay so stale voxels clear. [decisions: D1, D2, D3, D9] [tasks: tof-obstacle-cloud-node, laser-filter-tof-self-filter, nav2-tof-voxel-layer] [status: approved]
- G2: Nav2 collision_monitor gains a depth point-cloud source and a below-floor (cliff / negative obstacle) source. [decisions: D5, D6] [tasks: cliff-stop-polygon-generator, collision-monitor-tof-stop] [status: approved]
- G3: Data published only when real ToF frames and TF exist (no placeholder data); obstacles marked only within the ToF's valid range and confidence. [decisions: D1, D7] [tasks: tof-obstacle-cloud-node, laser-filter-tof-self-filter] [status: approved]
- G4: Packages added to the node apt lists (STVL, rtabmap_util), configs, nav-stack static tests, READMEs; CPU measured on the robot before/after (Grafana/Mimir) within an agreed budget. [decisions: D8, D12] [tasks: tof-obstacle-cloud-node, laser-filter-tof-self-filter, nav2-tof-voxel-layer, final-suite-and-runbook] [status: approved]
- G5: Web UI: obstacle voxels/cloud optionally visible in the 3D view; the agent can query nearby 3D obstacles. [decisions: D10, D11] [tasks: web-ui-obstacle-points-api, web-ui-obstacle-points-layer, tof-obstacle-sector-summary, map-summary-tof-obstacles] [status: approved]
- G6: 2.5D: a ground robot that avoids 3D obstacles on the 2D map; no 3D path planning. [decisions: D3] [tasks: nav2-tof-voxel-layer] [status: approved]
- U1: Onboard ToF obstacle perception on the Pi 5 feeding the Nav2 local costmap (3D obstacle layer) plus the collision monitor and cliff detection. [decisions: D1, D3, D5] [tasks: tof-obstacle-cloud-node, nav2-tof-voxel-layer, collision-monitor-tof-stop] [status: approved]
- U2: 2.5D: ground robot avoiding 3D obstacles on the 2D map, no 3D path planning. [decisions: D1, D3] [tasks: tof-obstacle-cloud-node, nav2-tof-voxel-layer] [status: approved]
- U3: Head TF comes from the pan/tilt joint states. [decisions: D9] [tasks: tof-obstacle-cloud-node, final-suite-and-runbook] [status: approved]
- U4: slam_toolbox keeps map->odom. [decisions: D9] [tasks: tof-obstacle-cloud-node, final-suite-and-runbook] [status: approved]
- U5: The lidar map extends the ToF (and 3D SLAM) where possible: the lidar keeps its obstacle layer in both costmaps and its collision_monitor source alongside the new ToF sources (ToF adds 3D detail inside its 4 m view, lidar covers 360 deg / ~12 m beyond it); ToF clearing never clears lidar-marked obstacles outside the ToF frustum; when the 3D obstacle cloud is shown in the web UI 3D view, lidar points are shown with it. [decisions: D4, D10] [tasks: nav2-tof-voxel-layer, collision-monitor-tof-stop, web-ui-obstacle-points-api, web-ui-obstacle-points-layer] [status: approved]
