# RGB-D ground estimation and plane-relative obstacles design

## Summary

A new C++ node `ground_estimator_cpp` on the client fits the floor plane from the 10 Hz self-filtered head ToF cloud
(`/tof/points_filtered`) with an IMU gravity prior, and publishes obstacle and drop-off clouds (base_link coordinates, points selected by plane-relative height bands), a JSON
ground state (plane, slope, tilt_guard-limit cross-check, step and drop-off edges as surface regions) and an RGB-D
colour-labelled cloud (floor vs obstacle, from `/head_rgbd/points`). Nav2 gets a second STVL layer in the local
costmap fed by those clouds, alongside the existing fixed odom-z bands; the collision monitor keeps the
10 Hz ToF bands and the global costmap stays static + lidar. mcp_server and the web UI consume the estimate. Ansible
node type (normal tier), metrics, a Grafana panel, a CPU budget measured on the Pi 5 and a robot runbook close it.

## Intent and why

The user wants the head RGB-D (verified on the robot 2026-10-20) used for obstacle estimation, path planning and ground
level/shape estimation. Today every ToF consumer assumes a flat floor at odom z = 0: the STVL bands (6 cm..0.6 m
obstacles, -1.0..-0.10 m cliffs), the collision monitor and the mcp `obstacles_3d` summary. A slope, a floor tilt or
the ToF depth long bias (0-40 mm, pushes floor points below the floor) eat into those margins, and no component knows
the real floor height, slope, steps or drop-off edges. A fitted plane makes the bands relative to the measured floor
and gives the arm floor_guard and the agent real terrain instead of per-call guesses.

## Goals

- `ground_estimator_cpp` publishes, only from real synchronised data with valid TF, at the ToF rate: the floor plane
  (normal, height at base_link, inlier ratio, fit or held), obstacle and drop-off clouds selected by plane-relative bands, step and
  drop-off edges as `SurfaceRegion`-compatible half-planes, slope vs gravity with a tilt_guard-limit level, the plane vs
  base tilt cross-check, and (at the RGB-D cloud rate) a floor/obstacle colour-labelled cloud.
- Nav2 local costmap: a `ground_voxel_layer` marks from the band-selected ground clouds alongside `tof_voxel_layer`;
  switch-over criteria written in the nav2_bringup README; collision monitor, CliffStop and global costmap unchanged.
- mcp_server `get_map_summary` reports the ground estimate; floor_guard can build its Terrain from the measured regions.
- web UI map shows the labelled ground points, the fitted plane patch and the edges.
- Ansible node type with tier, metrics port, Grafana panel, CPU budget measured on the Pi 5, READMEs, tests/README,
  runbook with the switch-over validation.

## Non-goals

- Global costmap changes (stays static + lidar, ruling Q3).
- ToF depth-bias correction (after the head mount reprint and head-rgbd-fusion runbook step 5).
- Learning-based segmentation; multi-plane (stairs as several planes) fitting; switching the fixed bands off.
- Feeding the ground clouds to the collision monitor.

## Anti-goals

- Placeholder data on any ground topic (zero planes, empty "keep-alive" clouds when no plane is valid).
- Removing or weakening any existing ToF band, collision source or the fail-safe stop on a ToF outage.
- A CPU cost on the Pi 5 that takes headroom from Nav2/servos (the node runs at 10 Hz on ~2700 points).
- A custom ROS message package for one state topic.

## Constraints

- New nodes are C++ (MEMORY "Python nodes stay": only web_ui and mcp_server stay Python); layout of
  `nodes/head_rgbd_cpp` (core static lib + gtests per `test/core/test_*.cpp`, node lib, `main.cpp`,
  `shared/ros2_common_cpp` config/metrics, `colcon_source` with `-j2`), tested with `scripts/cpp-node-test.sh`.
- Test-first for every behaviour; CLAUDE.md no placeholder data; pydantic in Python; config files over CLI.
- Ansible `ros2_nodes` schema, cgroup tiers (client.yml:27-33), deploy only via `scripts/deploy-nodes.sh`.
- `scripts/gen_nav_footprint.py` owns the generated blocks of nav2_params.yaml; they are not hand-edited.
- Deploying mcp_server kills a running agent session (MEMORY): the runbook checks first.
- Each task about 60 min and at most 8 files.

## Decisions

### D1: New C++ node `ground_estimator_cpp` on the client, input `/tof/points_filtered`

The node subscribes to the laser_filter output (`tof_optical_frame`, ~10 Hz, reliable) so the robot's own points are
already removed, `/imu/data` (BNO055 orientation) and `/head_rgbd/points` (XYZRGB, <= 1 Hz). Every cloud is transformed
to `base_link` at its stamp (tf timeout `tf_timeout_s`); a cloud without TF is dropped and counted. Alternatives: fit
inside head_rgbd_cpp on the 1 Hz RGB-D cloud (too slow for the costmap, couples two concerns) or a Python node (house
rule: new nodes are C++). User decision D-A1. Serves A1.

### D2: Plane fit with PCL SAC perpendicular-plane model and two priors

`pcl::SACSegmentation` with `SACMODEL_PERPENDICULAR_PLANE` (axis = the prior up vector, `fit.max_normal_angle_deg`
10 deg, distance threshold `fit.distance_m` 0.02 m, fixed seed for tests) on the points of a candidate band
(`|z| <= fit.candidate_band_m` 0.15 m in base_link, horizontal range <= `fit.max_range_m` 2.5 m). The up prior is
gravity from the IMU orientation corrected by the level reference (0.7/1.3 deg, the tilt_guard values) rotated into
base_link; with a stale IMU (`imu.timeout_s` 0.5 s) the prior is base_link +z and the state says `prior: base_link`.
A plane is valid only if: inliers >= `fit.min_inliers` (150), inlier ratio of the band >= `fit.min_inlier_ratio`
(0.2), normal within the angle and plane height at the base_link origin within `fit.max_offset_m` (0.08 m; the robot
stands on that floor). Alternatives: own Eigen RANSAC (CLAUDE.md prefers established libraries; PCL gives the
constrained model and the clustering of D5), least squares on all points (no outlier rejection). (my call: PCL and
the numeric defaults.) Serves A1.

### D3: Tracking, hold and the tilt_guard cross-check

The valid plane is smoothed (EMA `fit.smoothing` 0.5) and stored in `odom`. Stamp rule (PlaneTracker::current): for any query stamp (ToF cloud, an older RGB-D cloud, the 2 Hz state at now) the newest fit at or before the stamp is used, moved into base_link at that stamp through odom; source `fit` when no failed cloud came after it, `held` when one did; age = stamp - fit stamp; nothing beyond `fit.hold_s` and never a later fit. When a cloud yields no valid plane, the
last one is held for `fit.hold_s` (1.0 s) transformed to the current base_link (`source: held`); after that nothing is
published but the state with `plane: null` and the reason. Derived: `slope_deg` and its roll/pitch components (plane
normal vs gravity), `slope_level` ok/warning/error against the tilt_guard limits (roll 8/15, pitch 12/20 deg, same
values as `tilt_guard` in client.yml, pinned by a consistency test), `base_tilt_deg` (IMU vs level) and
`plane_vs_base_deg` (plane normal vs base_link +z; large on a level robot means head TF/extrinsic error). This is the
tilt_guard cross-check: tilt_guard measures the tilt the robot has, the ground node the tilt it would get ahead.
(my call.) Serves A3.

### D4: Plane-relative candidate bands

Signed height above the plane `h`: above-floor candidates `fit.distance_m` < h <= `bands.obstacle_max_m` 0.6, below-floor
candidates `-bands.drop_max_m` (-1.0) <= h < `-fit.distance_m`, within `bands.max_range_m` 2.0 m horizontal (the fixed
bands' 2 m obstacle_range). Which candidates are lethal is decided per cluster by the traversability rule (D14), not by
a band. "Plane-relative" means the selection only: the published points keep their base_link x, y, z. Margins below
the fixed bands (0.06 / -0.10) are possible because the plane is fitted to the same biased points, so the ToF long
bias shifts the plane with the floor. Clouds are `sensor_msgs/PointCloud2` XYZ float32 in base_link with the input
stamp; with a valid plane an empty cloud is published (a real "nothing lethal" observation), with no valid plane none
is. (my call: values; confirmed by the runbook.) Serves A1, A2.

### D5: Steps and drop-offs as surface regions

`pcl::EuclideanClusterExtraction` (`edges.cluster_tolerance_m` 0.1, `edges.min_cluster_points` 10) on the above and
below candidates. Each cluster gets the features the rule (D14) needs: `edge_step_m` (p90 |h| within
`traversability.edge_band_m` 0.05 m of its robot-facing edge line: ~H for a box or a drop, ~0 at the foot of a ramp),
`slope_deg` (its PCA plane vs the floor normal), and a 2D hull. Drop clusters and flat low step clusters
(|median h| <= `edges.step_max_height_m` 0.2, spread <= `edges.step_flatness_m` 0.03) give one half-plane region in the mcp `SurfaceRegion` shape: `{name: "drop_<i>"|"step_<i>", kind:
"drop"|"step", height_m: <median h>, frame: "base_link", edge: {point, direction, side}}`, the edge line through the
cluster's boundary nearest the robot along its principal axis, the region side away from the robot; at most
`edges.max_regions` (8, mcp MAX_REGIONS is 16), nearest first. Alternative: a height-map grid (more CPU, the mcp model
is half-planes already). (my call.) Serves A3.

### D6: RGB-D colour labels

On each `/head_rgbd/points` cloud (transformed to base_link at its stamp, plane held at that stamp within
`fit.hold_s`): geometric floor points (|h| <= `fit.distance_m`) update a CIELAB Gaussian floor colour model (OpenCV
`cvtColor`, forgetting `colour.forgetting` 0.1, valid after `colour.min_floor_samples` 200). Each point gets a label:
0 floor, 1 above_floor, 2 below_floor (the candidate bands of D4), 3 floor_colour_mismatch (geometric floor,
Mahalanobis > `colour.max_mahalanobis` 3.0: flat objects, rugs, paper, liquid), 4 unlabelled (outside the bands or
range). Output `/ground/labelled_points` XYZRGB + uint8 `label` field, RGB-D stamp and frame base_link; the state
carries `colour {model_ok, floor_samples, mismatch_ratio, stamp_ns}`. Labels feed the web UI and mcp; floor_colour_mismatch points are clustered and classified by the same
obstacle/warning rule as the ToF clusters (D14), so a flat flagged object becomes a slow-down warning area. Alternative: per-pixel 2D segmentation on the image (needs the registered depth anyway, more
CPU). User decision D-A1 (colour labels floor vs obstacle), (my call: model). Serves A4.

### D7: Outputs: JSON state on std_msgs/String, clouds on PointCloud2

`/ground/state` is `std_msgs/String` JSON (reliable, transient_local depth 1, like `/tilt_guard/state`) at
`state.rate_hz` 2 Hz and on every validity change, keys fixed in `state.hpp` `STATE_KEYS` (contract `ground-state`):
`stamp_ns, frame, prior, imu_ok, plane {normal, height_m, inliers, inlier_ratio, source, age_s} | null, reason,
slope_deg, slope_roll_deg, slope_pitch_deg, slope_level, base_tilt_deg, plane_vs_base_deg, traversability, obstacle_points,
drop_points, regions (with class), warnings, colour`; `/ground/warning_areas` (std_msgs/String JSON in the map frame, contract
`ground-warning-areas`) carries the warning polygons for costmap_zones (D15). `/ground/obstacle_points`, `/ground/drop_points` (reliable depth 1, matching the laser_filter output STVL
already consumes) and `/ground/labelled_points` (best_effort depth 1).
nlohmann-json as in map_guard_cpp. Alternative: a custom msg package (none exists in the repo; one more build for one
topic). (my call.) Serves A1, A3, A4.

### D8: Nav2: a second STVL layer alongside the fixed bands

Local costmap plugins `[obstacle_layer, tof_voxel_layer, ground_voxel_layer, keepout_filter, ground_cost_filter,
inflation_layer]` (`ground_cost_filter`: D15).
`ground_voxel_layer` has the tof_voxel_layer settings (5 cm voxels, decay 15 s linear, combination_method 1, no
footprint clearing, transform_tolerance 0.3, expected_update_rate 0, publish_voxel_map false) and sources
`ground_obstacle_mark` (`/ground/obstacle_points`), `ground_drop_mark` (`/ground/drop_points`), both pass-through
height limits -1.5..1.5 m (the node already selected the obstacle-class points), obstacle_range 2.0, and `ground_clear`
(`/tof/points_filtered` frustum, same as tof_clear). A separate layer makes the switch-over a runtime `enabled` toggle
and keeps the fixed-band tests intact. `tof_voxel_layer`, the collision monitor sources (scan, tof_obstacles,
tof_cliff), CliffStop (fed by tof_cliff only) and the global costmap stay exactly as they are: the collision monitor
needs the 10 Hz source with its fail-safe stale stop (user decision D-A1). Switch-over criteria (README): the runbook
scenes pass with the ground layer alone; then a separate change disables `tof_mark`/`tof_cliff_mark`. Alternative:
add sources to tof_voxel_layer (shared decay grid, switch-over is a config edit and a redeploy). (my call: separate
layer.) Serves A2.

### D9: mcp_server consumers

`topics.ground_state` (default `/ground/state`, also in client.yml) is subscribed (latched JSON, latest only);
`get_map_summary` gains `ground` (plane height, slope, slope_level, plane_vs_base, regions, colour mismatch ratio, age)
or a note when absent or older than `ground_state_stale_s` 1.0 s. floor_guard: `FloorOverride.use_ground_estimate`
(default false, per call) builds the Terrain from the fresh state's regions (explicit `surfaces` listed after them,
so they win); requested but not fresh (> `floor_guard.ground_max_age_s` 1.0 s) or `plane: null` refuses the motion
with a reason naming /ground/state (fail-safe). User answer Q2: yes (D16). `ground_regions_to_surfaces` (mcp-ground-summary) converts GroundRegion to SurfaceRegion dropping `kind`, `distance_m`, `points` (SurfaceRegion is extra="forbid"), at most MAX_REGIONS 16. Serves A3.

### D10: Web UI ground layer

Backend: map_nav roles `ground_cloud` (PointCloud2 with the label field, voxel/point cap like rgbd_cloud) and
`ground_state` (JSON), tab keys `ground_cloud_topic` and `ground_state_topic` in default.yaml and client.yml. Frontend:
a "Ground" layer (off by default, on demand) drawing labelled points coloured per label, a translucent plane patch
(2 m around base_link from the state plane) and the region edges as lines, placed by the TF of base_link; legend in
the LayersPanel. Staleness is the backend receive age (`ground_state_stale_s` 2 s, like `tilt_guard_stale_s`): the bridge marks the cached state `stale: true`; the frontend draws nothing from a stale or plane-null state (no client clock). Serves A4.

### D11: Ansible node type, tier, metrics, dashboard, CPU budget

Node type `ground_estimator_cpp` (native colcon_source of `shared/ros2_common_cpp` + `nodes/ground_estimator_cpp`,
Release, `-j2`, apt: PCL `libpcl-dev`, `ros-jazzy-pcl-conversions`, OpenCV, Eigen, yaml-cpp, nlohmann-json,
prometheus-cpp, tf2), normal tier (cpu/io_weight 500, oom_score_adjust -500: it feeds the local costmap), memory_low
48M / memory_max 256M until the runbook measures p50 and peak, config `/etc/ros2/ground_estimator_cpp/config.yaml`
via `GROUND_ESTIMATOR_CONFIG`, metrics port 19119, a scrape entry. Metrics `ground_fits_total{result}`,
`ground_fit_seconds` (histogram), `ground_inlier_ratio`, `ground_slope_deg`, `ground_plane_vs_base_deg`,
`ground_regions`, `ground_colour_mismatch_ratio`, `ground_clouds_dropped_total{reason}` plus process metrics. Grafana
panels in robot-slam.json. CPU budget (my call): node <= 0.10 core p95, nav2_bringup increase <= 0.05 core, measured
on the Pi 5 with `rate(container_cpu_usage_seconds_total{host="client",id="/system.slice/ros2-ground_estimator_cpp.service"}[1m])`.
Serves A5.

### D12: Runbook, docs and robot verification

`docs/runbooks/ground-estimation.md`: deploy, topic rates, level-floor check (plane height within 1 cm, slope and
plane_vs_base < 1 deg), scenes (3 cm sill, 10 cm drop at 1 m, box, ramp, colour mismatch with a sheet of paper), fixed
vs ground marks compared in the web UI and via topic_scraper, CPU/memory measurement and the tier values written back,
the switch-over checklist. READMEs (node, nav2_bringup, mcp_server, web_ui, ansible, nodes), tests/README, MEMORY
decision entry, topic flow diagram. The orchestrator executes the runbook on the robot after merge. Serves A6.

### D13: Global costmap static + lidar; no depth-bias correction

Global costmap keeps `[static_layer, obstacle_layer, keepout_filter, inflation_layer]` (ruling Q3), pinned by the
existing test and the new layer test. No ToF depth-bias correction here (after the head mount reprint). User
decision. Serves A2.

### D14: Obstacle vs warning rule: 1/3 wheel diameter, max slope
Obstacle vs warning rule (replaces Q1): a step, threshold, drop or ramp cluster is an OBSTACLE (lethal: its points go to /ground/obstacle_points or /ground/drop_points and the ground_voxel_layer) when its edge step exceeds one third of the wheel diameter (traversability.wheel_radius_m 0.06 from swerve_drive_controller wheel_radius_m, max_step_fraction 1/3 -> 0.04 m, configurable, pinned equal by a consistency test) or its surface slope exceeds traversability.max_slope_deg (default 8 deg, my call: the lower of the tilt_guard warn limits, roll 8 / pitch 12, because a swerve base can meet a ramp at any heading, so driving onto a steeper ramp would at once raise a tilt_guard warning); otherwise it is a WARNING area, clear to ride through at reduced speed. Colour-flagged flat objects (floor_colour_mismatch clusters) are classified by the same rule. Tests cover both sides of each threshold (0.035/0.045 m sills and drops, 6/10 deg ramps, exactly 0.04 m and 8 deg are warnings). (user decision)

### D15: Slow-down via costmap_zones speed mask and local cost band
Slow-down mechanism for warning areas: ground_estimator_cpp publishes the warning polygons on /ground/warning_areas (JSON, map frame); costmap_zones, already the single writer of the Nav2 speed mask, keeps them for ground_warning_ttl_s 3 s and rasterises them into (1) the speed mask at ground_warning_speed_pct 40 (lowest wins with POI slow zones), which the existing local SpeedFilter turns into /speed_limit and controller_server (MPPI) honours, and (2) a new local-only ground cost mask (value 40 -> cost ~101, below INSCRIBED 253, so passable) read by a ground_cost_filter KeepoutFilter layer below inflation in the local costmap only. No second /speed_limit writer, the keepout mask and the global costmap stay untouched (ruling Q3). Alternatives rejected: a second SpeedFilter (two writers of /speed_limit), a custom costmap plugin (new C++ Nav2 plugin for one band), STVL (lethal only). (my call) (user decision)

### D16: floor_guard opt-in, refuse when stale
Q2 answered yes: floor_guard uses the measured ground regions only when a call sets use_ground_estimate (default off) and refuses the motion when requested but no fresh /ground/state with a plane exists. (user decision)

### D17: Run in parallel with the related epics
Q3 answered yes: this epic runs in parallel with tof-3d-obstacle-detection-fo and head-rgb-d-fusion-of-camera (no README blockers); the orchestrator queues their final tasks; tasks touching shared files (tests/test_tof_obstacle_wiring.py, client.yml) rebase on whatever lands first. The three graph --todo contract-order lines are an accepted ruling. (user decision)

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | ran (discovery brief rgbd-ground.md, 2026-10-20) | head_rgbd_cpp layout; laser_filter output as input; tilt_guard limits and level reference; SurfaceRegion shape (surfaces.py:46-118); rgbd_cloud role pattern (web_ui bridge.py:277-305) | fitting in head_rgbd_cpp (1 Hz) |
| research-code-external | skipped: PCL SAC perpendicular plane and Euclidean clustering are the standard tools, no fork left after D-A1 | PCL SACSegmentation, EuclideanClusterExtraction | own RANSAC |
| research-papers | skipped: plane fitting with a gravity prior is textbook RANSAC; no learning (non-goal) | none | learning segmentation |
| research-docs | skipped: STVL and collision monitor parameters already used and documented in nav2_params.yaml:242-329, 732-750 | STVL PointCloud2 sources, frustum clearing | feeding collision_monitor |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief: existing pipeline | /tof/points_filtered (laser_filter self filter, TF at stamp) is the input | ground-node-wiring, ground-node-type |
| brief: tof_voxel_layer bands in odom assume floor z=0 | plane-relative bands | ground-band-segmentation, nav2-ground-voxel-layer |
| brief: collision_monitor source_timeout 1 s, 1 Hz cloud must not feed it | collision monitor unchanged, pinned | nav2-ground-voxel-layer |
| brief: head_rgbd cloud 1 Hz, lazy, map3d_uplink subscribes | colour labels at the RGB-D rate | ground-colour-labels, ground-node-wiring |
| brief: no floor estimator exists; tilt_guard IMU levels | IMU prior and slope cross-check | ground-plane-fit, ground-plane-tracker |
| brief: mcp surfaces.py SurfaceRegion/Terrain | regions in SurfaceRegion shape, floor_guard opt-in | ground-step-edges, mcp-floor-guard-ground |
| brief: mcp perception tof_obstacle_sectors | ground field next to obstacles_3d | mcp-ground-summary |
| brief: web_ui obstaclePoints.ts / sceneLayers / rgbd_cloud role | ground layer pattern | web-ui-ground-backend, web-ui-ground-frontend |
| brief: cgroup tiers, metrics, node_cpu_compare.yaml | node type, tier, metrics, dashboard, CPU budget | ground-node-type, ground-dashboard-panels, ground-final-suite-runbook |
| brief: caveat ToF long bias pushes floor points below the floor | plane fitted to biased points; tighter drop margin | ground-band-segmentation |
| brief: gen_nav_footprint.py generated, do not hand-edit | generated blocks untouched | nav2-ground-voxel-layer |
| brief: tests/test_tof_obstacle_wiring.py, test_nav_tof_voxel_layer.py pin chain and bands | tests scoped to tof_voxel_layer, ground chain added | nav2-ground-voxel-layer |
| brief: unknown head joint_states rate, live CPU | measured in the runbook | ground-final-suite-runbook |
| brief: depth-bias correction | rejected: user decision, after the mount reprint | none |
| brief: global costmap | rejected: ruling Q3 keeps static + lidar | none |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 New C++ node on /tof/points_filtered | ground-package-config, ground-node-wiring |
| D2 Plane fit with priors | ground-plane-fit |
| D3 Tracking, hold, tilt_guard cross-check | ground-plane-tracker, ground-node-type |
| D4 Plane-relative bands | ground-band-segmentation |
| D5 Steps and drop-offs as regions | ground-step-edges |
| D6 RGB-D colour labels | ground-colour-labels, ground-node-wiring |
| D7 JSON state and clouds | ground-state-json, ground-node-wiring |
| D8 Nav2 second STVL layer | nav2-ground-voxel-layer |
| D9 mcp consumers | mcp-ground-summary, mcp-floor-guard-ground |
| D10 Web UI ground layer | web-ui-ground-backend, web-ui-ground-frontend |
| D11 Ansible, tier, metrics, dashboard, CPU | ground-node-type, ground-dashboard-panels, ground-final-suite-runbook |
| D12 Runbook, docs, robot verification | ground-final-suite-runbook, ground-node-readme |
| D13 Global costmap and bias out of scope | nav2-ground-voxel-layer |
| D14 Obstacle vs warning rule: 1/3 wheel diameter, max slope | ground-traversability-rule, ground-step-edges, ground-colour-labels, ground-node-wiring, ground-package-config, ground-node-type, ground-final-suite-runbook |
| D15 Slow-down via costmap_zones speed mask and local cost band | costmap-ground-warnings, nav2-ground-voxel-layer, ground-state-json, ground-node-wiring, mcp-ground-summary, web-ui-ground-frontend |
| D16 floor_guard opt-in, refuse when stale | mcp-floor-guard-ground |
| D17 Run in parallel with the related epics | nav2-ground-voxel-layer, ground-node-wiring |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `ground-config` | ground-package-config | ground-plane-fit, ground-plane-tracker, ground-band-segmentation, ground-step-edges, ground-colour-labels, ground-node-wiring, ground-node-type, ground-node-readme, ground-traversability-rule | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp` |
| `ground-plane-fit` | ground-plane-fit | ground-plane-tracker, ground-band-segmentation | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/plane_fit.hpp` |
| `ground-synthetic-scene` | ground-plane-fit | ground-plane-tracker, ground-band-segmentation, ground-step-edges, ground-colour-labels, ground-node-wiring, ground-traversability-rule | `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp` |
| `ground-tracker` | ground-plane-tracker | ground-state-json, ground-node-wiring | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/tracker.hpp` |
| `ground-segmentation` | ground-band-segmentation | ground-step-edges, ground-colour-labels, ground-node-wiring, ground-traversability-rule | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/segmentation.hpp` |
| `ground-edges` | ground-step-edges | ground-state-json, ground-traversability-rule, ground-node-wiring | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp` |
| `ground-colour-labels` | ground-colour-labels | ground-state-json, ground-node-wiring, web-ui-ground-backend | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/colour.hpp` |
| `ground-state` | ground-state-json | ground-node-wiring, mcp-ground-summary, mcp-floor-guard-ground, web-ui-ground-backend, web-ui-ground-frontend, ground-node-readme | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp` |
| `ground-topics` | ground-node-wiring | ground-node-type, nav2-ground-voxel-layer, mcp-ground-summary, web-ui-ground-backend, ground-node-readme, costmap-ground-warnings | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp` (topic accessors, documented in the node README) |
| `ground-metrics` | ground-node-wiring | ground-dashboard-panels, ground-node-readme | `nodes/ground_estimator_cpp/src/ground_node.cpp` |
| `ground-node-unit` | ground-node-type | nav2-ground-voxel-layer, mcp-ground-summary, ground-dashboard-panels, ground-final-suite-runbook, costmap-ground-warnings | `ansible/group_vars/client.yml` |
| `mcp-ground-state-client` | mcp-ground-summary | mcp-floor-guard-ground | `nodes/mcp_server/mcp_server/ros_iface.py` |
| `ground-traversability` | ground-traversability-rule | ground-state-json, ground-node-wiring | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp` |
| `ground-warning-areas` | ground-state-json | costmap-ground-warnings | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp` (`to_warning_areas_string`) |
| `ground-cost-mask` | costmap-ground-warnings | nav2-ground-voxel-layer | `nodes/costmap_zones/costmap_zones/config.py` |
| `web-ground-roles` | web-ui-ground-backend | web-ui-ground-frontend | `nodes/web_ui/web_ui/config.py` |

Cross-epic (registered in `docs/plans/contracts.json`): `head-rgbd-points` (owner head-rgb-d-fusion-of-camera,
consumed by ground-node-wiring), `tof-points-filtered` and `nav2-tof-voxel-bands` (owner tof-3d-obstacle-detection-fo,
consumed by ground-node-wiring, nav2-ground-voxel-layer). The tof epic's escalated `final-suite-and-runbook` also
lists `tests/test_tof_obstacle_wiring.py`; nav2-ground-voxel-layer only scopes its STVL-source check to
`tof_voxel_layer` and adds the ground chain, so whichever lands second rebases.

## Review focus

- No placeholder data: no cloud without a valid (fit or held) plane; state `plane: null` with a reason instead.
- TF at the cloud stamp, plane held in odom and moved to the current base_link correctly (sign conventions of the
  plane height, REP-103 roll/pitch).
- The existing fixed bands, collision monitor sources, CliffStop and global costmap are byte-for-byte unchanged.
- STATE_KEYS contract identical across C++, mcp and web_ui parsers.
- floor_guard: `use_ground_estimate` never weakens the default surface model; requested-but-stale refuses.
- CPU: the fit runs on the decimated band only; no per-point allocations in the 10 Hz path.

## Left to the builder

- Internal structure of the core library functions, as long as the headers named in the Contracts exist.
- The exact edge-line construction of D5 within: through the cluster boundary nearest the robot, region away from it.
- Grafana panel layout within robot-slam.json.

## Assumptions and unknowns

- PCL (`libpcl-dev`, `ros-jazzy-pcl-conversions`) resolves through rosdep in the cpp-node-test image and builds on the
  Pi 5 with `-j2` within memory; confirmed by ground-package-config (docker) and the runbook deploy.
- head joint_states rate is high enough for TF at the 10 Hz cloud stamp (laser_filter already relies on it, 0.2 s
  timeout); the runbook records the TF drop counter.
- `/head_rgbd/points` stays published continuously (map3d_uplink subscribes); the ground node subscribing is enough
  anyway (lazy publishing serves any subscriber).
- The numeric defaults (D2, D4, D5, D6, D11) are confirmed or corrected by the runbook scenes (runbook "Threshold validation"); the CPU budget, the container C++ suite, the frontend suite and the PCL -j2 build on the Pi are validated in CI and on the robot (runbook "Build and CI validation"), not by build-time ACs.

## Open questions

none
- Q1: Should points the RGB-D colour model labels as non-floor while geometrically flat (floor_colour_mismatch: rugs, paper, liquid) also mark the Nav2 local costmap in this epic? [recommended: no - labels go to the web UI and mcp only; geometry alone marks Nav2 until the runbook shows the colour model's false-positive rate]
- Q2: Should floor_guard use the measured ground regions only when a call opts in (use_ground_estimate, default off), refusing the motion when requested but no fresh estimate exists? [recommended: yes - opt-in per call, fail-safe refusal]
- Q3: Should this epic wait (README blockers) until tof-3d-obstacle-detection-fo and head-rgb-d-fusion-of-camera are done? graph --todo flags the three registered contracts (head-rgbd-points, tof-points-filtered, nav2-tof-voxel-bands) because no blocker orders them; both owners have only their robot runbook tasks left and the consumed topics are already on main. [recommended: no - run in parallel, rebase on whatever their runbooks change; the orchestrator records the graph lines as a ruling]

## Irreversible operations in scope

none (deploys via scripts/deploy-nodes.sh are reversible by redeploying; the runbook checks for a running agent
session before deploying mcp_server).
