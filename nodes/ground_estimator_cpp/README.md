# ground_estimator_cpp

Estimates the floor plane from the head ToF cloud, constrained by the IMU, and publishes obstacle and drop-off clouds
selected by plane-relative bands, a labelled RGB-D cloud, step/drop/ramp regions, warning areas and a JSON state. The
executable `ground_node` (ROS node name `ground_estimator`) wires the core library to ROS topics, TF and metrics.

## Layout

- `src/core/*.cpp` build into the static library `ground_estimator_core` (no ROS node code): plane fit, tracker,
  segmentation, edges, traversability, colour model, state JSON, config. `test/core/test_*.cpp` each become a gtest
  named after the file.
- `src/ground_node.cpp`, `include/ground_estimator_cpp/ground_node.hpp`: `GroundNode`; `src/main.cpp`: the executable.
  `test/test_node.cpp` runs the node in-process; `test/test_replay.cpp` replays the live recordings through it (see
  Replay tests).
- `config/ground_estimator.yaml`: every key with its default, meaning and unit (installed to
  `/etc/ros2/ground_estimator_cpp/config.yaml`, override path with `GROUND_ESTIMATOR_CONFIG`). Unknown keys and invalid
  values exit 78.
- `test/data/state_example.json`, `test/data/warning_areas_example.json`: documents the consumers are tested against.
- `test/data/replay/{home,box,ramp}.bin`, `test/replay_fixture.hpp`: the replay fixtures and their reader and metrics.

## Topics

Output topics are under `output_namespace` (default `/ground`). The output clouds keep base_link coordinates (the
points are not shifted onto the plane); which points land in them is decided by plane-relative bands (height above or
below the tracked floor plane), so they stay correct on a slope or a tilted base.

| Direction | Topic | Type | QoS | Frame | Notes |
|---|---|---|---|---|---|
| in | `/tof/points_filtered` | `sensor_msgs/PointCloud2` | reliable, depth 1 | ToF frame, transformed to base_link | 10 Hz, drives the plane fit and every output but `/ground/labelled_points` |
| in | `/imu/data` | `sensor_msgs/Imu` | sensor data | IMU frame | prior of the floor normal; older than `imu.timeout_s` it is not used |
| in | `/head_rgbd/points` | `sensor_msgs/PointCloud2` (XYZRGB) | best_effort, depth 1 | base_link or camera | coloured cloud for the floor colour model; only with `colour.enabled` |
| out | `/ground/obstacle_points` | `sensor_msgs/PointCloud2` | reliable, depth 1 | base_link | points above the plane up to `bands.obstacle_max_m`, within `bands.max_range_m` |
| out | `/ground/drop_points` | `sensor_msgs/PointCloud2` | reliable, depth 1 | base_link | points below the plane down to `bands.drop_max_m` |
| out | `/ground/labelled_points` | `sensor_msgs/PointCloud2` | best_effort, depth 1 | base_link | RGB-D cloud with a uint8 `label` field (see Labels); only with `colour.enabled` |
| out | `/ground/state` | `std_msgs/String` (JSON) | reliable, transient_local, depth 1 | base_link | see State; published at `state.rate_hz` |
| out | `/ground/warning_areas` | `std_msgs/String` (JSON) | reliable, depth 1 | `warnings.frame` | `{stamp_ns, frame, areas: [{kind, polygon, height_m, slope_deg, source}]}` for costmap_zones |

## State

`/ground/state` is one compact JSON object with these keys (contract `ground-state`, order fixed by `STATE_KEYS` in
`include/ground_estimator_cpp/state.hpp`, floats rounded to 4 decimals). Without a plane, `plane`, the slope keys and
`plane_vs_base_deg` are null, the counts are 0 and `regions` and `warnings` are empty.

| Key | Meaning | Unit |
|---|---|---|
| `stamp_ns` | stamp of the ToF cloud the state describes | nanoseconds |
| `frame` | frame of the plane, regions and warnings (`base_link`) | - |
| `prior` | source of the floor-normal prior, `imu`, or `base_link` (no fresh IMU: +z of base_link) | - |
| `imu_ok` | a fresh IMU sample was used | bool |
| `plane` | `{normal [x,y,z], height_m, inliers, inlier_ratio, source, age_s}`; `source` is `fit` or `held`; null without a plane | m, s |
| `reason` | why there is no plane: a fit reject (`too_few_points`, `no_plane`, `normal_off_prior`, `offset_off_prior`, `too_few_inliers`, `low_inlier_ratio`), `no_tf` or `no_cloud`; null with a plane | - |
| `slope_deg` | total floor slope against gravity | degrees |
| `slope_roll_deg` | roll part of the floor slope | degrees |
| `slope_pitch_deg` | pitch part of the floor slope | degrees |
| `slope_level` | `ok`, `warning` or `error` against the `slope.*` limits | - |
| `base_tilt_deg` | tilt of base_link against gravity | degrees |
| `plane_vs_base_deg` | angle between the floor normal and base_link +z | degrees |
| `traversability` | `{max_step_m, max_slope_deg}` in force | m, degrees |
| `obstacle_points` | points of the last obstacle cloud | count |
| `drop_points` | points of the last drop cloud | count |
| `regions` | step, threshold, drop and ramp regions: `name`, `kind`, `class` (`obstacle` or `warning`), `height_m`, `frame`, `edge {point, direction, side}`, `distance_m`, `points` | m |
| `warnings` | warning areas: `kind`, `polygon` (base_link, m), `height_m`, `slope_deg`, `source` (`tof` or `colour`) | m, degrees |
| `colour` | `{model_ok, floor_samples, mismatch_ratio, stamp_ns}` of the last labelled RGB-D cloud; null when colour is disabled or no cloud was labelled | ratio |

## Labels

The `label` field of `/ground/labelled_points`:

| Value | Name | Meaning |
|---|---|---|
| 0 | floor | in the floor band and matching the floor colour |
| 1 | above_floor | above the floor band (geometry wins over colour) |
| 2 | below_floor | below the floor band (geometry wins over colour) |
| 3 | floor_colour_mismatch | in the floor band but farther than `colour.max_mahalanobis` from the floor colour |
| 4 | unlabelled | no label: no plane, colour model not ready, or no valid colour |

## Obstacle or warning

A step, drop or ramp is an obstacle when its edge step exceeds one third of the wheel diameter (`traversability.max_step_fraction`
of `2 * traversability.wheel_radius_m`, 0.04 m by default) or its slope exceeds `traversability.max_slope_deg`.
Otherwise it is a warning area: the node publishes it on `/ground/warning_areas` and costmap_zones turns it into a
slow-down instead of a lethal cell. Colour-flagged flat objects (label 3) follow the same rule: a flat patch of a
different colour is a warning area with `source` `colour`, unless its height or slope crosses the limits above.

The edge step of a cluster is the median of |height| within `traversability.edge_band_m` of its robot-facing edge
line, so noise does not lift a 3 cm sill over the rule and a 5 cm box reads 5 cm. Its slope is a robust (Huber IRLS)
plane fit of the plane-relative height over one sample per level set of equal heights: after the temporal median
those are the cells, so the per-cell staircase does not flatten a ramp. A cluster covering fewer than
`edges.slope_min_cells` bins of `edges.slope_bin_m`, or whose bins lie in one line along or across its principal axis,
has no measurable slope (`slope_known` false, `slope_deg` 0).

## Temporal median

The ToF cloud is noisy (a few centimetres per point, more at long range), more than the 4 cm step rule. Before step,
drop and obstacle classification `CellMedianFilter` (`cell_median.hpp`) bins the points by their xy in the fixed frame
into `temporal.cell_size_m` cells (so the cells stay put while the robot moves), takes each frame's median plane-relative
height per cell and keeps the median of the last `temporal.window_frames` of those values, dropping values older than
`temporal.max_age_s`. The points of a cell with at least `temporal.min_frames` values are moved along the plane normal
to that height and segmentation, clusters, regions and the traversability rule run on them. The plane fit itself keeps
the raw cloud. The rule (0.04 m step, 15 degrees slope) is unchanged by the filter.

In-cell slope: a cell is not collapsed to one height. Per frame a cell with at least 12 points gets a robust plane
(least squares, refitted without residuals beyond 2.5 robust sigmas): the height at the cell centre and the gradient
in the fixed frame. The cell keeps the temporal median of both. A gradient component is applied only when it is 3
standard errors away from zero (the larger of the frame to frame scatter and the fit error of one frame, so 4 to 8
values cannot agree by chance), and a gradient steeper than tan 22 deg is dropped, because it is a step edge inside the
cell that a plane would smear into a ramp. The points are moved to height + gradient . (xy - centre). Without it a
25 cm ramp of 14 deg on 10 cm cells became a staircase and the cluster slope estimator saw 4-9 deg (measured 5.7 on the
robot); a cell with fewer than 12 points, or with noise too high to see the gradient, behaves as before (flat at its
median).

Step edges: a cell straddling a step (the foot of a sill or a box, a corner) fits a gradient as steady as a ramp
cell's, but its points lie on two levels. Such a cell keeps the median of its per-frame median heights, and its spread
gate uses those medians, as without in-cell slope. A cell gets its plane only when (1) the RMS of its points about the
plane is at most 1.5 times the smallest one among its 8 neighbours (on an inclined surface the plane leaves the sensor
noise, like the floor beside it; across a step it leaves about a quarter of the step height, measured 9 mm against
3 mm noise for a rendered 3 cm sill) and (2) a neighbour passing (1) has a gradient within 45 deg of its own (a ramp
spans cells, a lone edge or corner cell does not); its spread gate then uses the plane's centre heights. Without this
rule a 3 cm sill read as a 3.4 deg ramp and the 5 cm box's foot lost its step under the 4 cm rule
(`CellMedian.SillEdgeCellsKeepTheirMedianHeight`, `CellMedian.BoxEdgeCellsKeepTheStep`).

Decision (cell size): `temporal.cell_size_m` stays 0.10 because each cell keeps its in-cell slope. This task re-ran the
ramp test of `CellMedian.RobotRampSlopeSurvivesTheCells` (C++, 20 seeds, random cell offset and yaw) with both sizes:
0.10 m gives slope p50 14.3 deg (range 9.8-16.6; 13.7 deg with the step edge rule above), 0.05 m gives p50 13.0 deg
(range 10.3-15.5). Both are inside 14 +- 3 deg, so the smaller cell buys nothing, while it has 4x fewer points per
cell, which weakens the spread gate and the gradient fit. The earlier "8-10 deg with misaligned cells" figure came from the finding's Python simulation of
cells that collapse to one height; it was not measured by this task. Test `CellMedian.SlopeInsideACellIsKept` pins the
in-cell gradient on a noise-free 14 deg incline (points stay within 4 mm of the surface; a collapsed cell is 12 mm off).

Warm-up: a cell outputs nothing until it has `min_frames` values, so after start, after the plane is lost (the filter
is reset then) or when the robot looks at new floor, obstacles and drops appear after `min_frames` / 10 Hz = 0.4 s with
the defaults. Set `temporal.enabled: false` for the raw per-frame behaviour.

## Configuration

Every key of `config/ground_estimator.yaml` (nested keys dotted). Unknown keys are rejected at every level.

| Key | Default | Meaning |
|---|---|---|
| `cloud_topic` | `/tof/points_filtered` | PointCloud2 of the head ToF, 10 Hz, used for the plane fit |
| `imu_topic` | `/imu/data` | sensor_msgs/Imu, prior of the floor normal |
| `rgbd_cloud_topic` | `/head_rgbd/points` | coloured PointCloud2, used for the floor colour model |
| `output_namespace` | `/ground` | prefix of obstacle_points, drop_points, labelled_points, state, warning_areas |
| `base_frame` | `base_link` | robot frame the plane is expressed in |
| `fixed_frame` | `odom` | world-fixed frame for stamping and warning areas |
| `tf_timeout_s` | `0.1` | seconds to wait for a transform, 0..0.5 |
| `imu.timeout_s` | `0.5` | seconds after which an IMU sample is too old to use, > 0 |
| `imu.level_reference_roll_deg` | `0.7` | IMU roll reading on level ground, degrees, |v| <= 30 |
| `imu.level_reference_pitch_deg` | `1.3` | IMU pitch reading on level ground, degrees, |v| <= 30 |
| `fit.distance_m` | `0.02` | RANSAC inlier distance to the plane, metres, 0 < v <= 0.1 |
| `fit.max_normal_angle_deg` | `10.0` | max angle between fitted and IMU normal, degrees, 0 < v <= 30 |
| `fit.max_offset_m` | `0.08` | max plane offset from the nominal floor, metres, 0 < v <= 0.3 |
| `fit.min_inliers` | `150` | fewest inlier points for a valid plane, >= 10 |
| `fit.min_inlier_ratio` | `0.2` | fewest inliers as a fraction of candidates, 0 < v <= 1 |
| `fit.max_iterations` | `100` | RANSAC iterations, >= 10 |
| `fit.candidate_band_m` | `0.15` | half-width of the band around the IMU plane that feeds the fit, metres, > distance_m |
| `fit.max_range_m` | `2.5` | farthest point used for the fit, metres, > 0 |
| `fit.hold_s` | `1.0` | seconds the last plane is kept when a fit fails, >= 0 |
| `fit.smoothing` | `0.2` | weight of the new plane in the running estimate, 0 < v <= 1 (1 = no smoothing) |
| `bands.obstacle_max_m` | `0.6` | obstacle band top above the plane, metres, > traversability max step |
| `bands.drop_max_m` | `1.0` | drop band depth below the plane, metres, > traversability max step |
| `bands.max_range_m` | `2.0` | farthest point published in the bands, metres, > 0 |
| `bands.min_range_m` | `0.42` | nearest point (from base_link) in the bands: the robot's own parked gripper and its mixed pixels lie closer, metres, 0 <= v < max_range_m |
| `traversability.wheel_radius_m` | `0.06` | wheel radius, metres, same as swerve_drive_controller wheel_radius_m |
| `traversability.max_step_fraction` | `0.3333` | max rideable step as a fraction of the wheel diameter (0.04 m), 0 < v < 1 |
| `traversability.max_slope_deg` | `15.0` | steepest rideable surface, degrees, <= slope.roll_error_deg and slope.pitch_error_deg |
| `traversability.edge_band_m` | `0.05` | width of the strip on each side of an edge that sets its step height, metres, > 0 |
| `traversability.lethal_min_points` | `2` | fewest unclustered lethal points within edges.cluster_tolerance_m of each other for any to be lethal, 1..50 |
| `warnings.frame` | `map` | frame of the published warning areas |
| `edges.cluster_tolerance_m` | `0.1` | max gap inside one cluster, metres, > 0 |
| `edges.min_cluster_points` | `10` | fewest points in a cluster, >= 3 |
| `edges.max_regions` | `8` | most regions reported, 1..16 |
| `edges.step_max_height_m` | `0.2` | tallest step still treated as a step, metres, > max step, <= bands.obstacle_max_m |
| `edges.step_flatness_m` | `0.03` | max height spread for a flat top, metres, > 0 |
| `edges.slope_bin_m` | `0.1` | edge of the xy bins a cluster is sampled in for its slope fit, metres, 0.02 <= v <= 0.5 |
| `edges.slope_min_cells` | `3` | fewest slope bins a cluster must cover for a slope estimate (below it the slope is unknown), 2..50 |
| `edges.step_segment_m` | `0.1` | length of the boundary segment a step is measured along, metres, 0.02 <= v <= 0.5 |
| `edges.step_segment_min_points` | `3` | fewest boundary points in a segment for it to count, 1..50 |
| `edges.floor_reference_m` | `0.1` | ring around a cluster whose floor points set the floor level its step is measured from, metres, 0 (plane) .. 0.5 |
| `edges.floor_reference_min_points` | `4` | fewest ring points for the floor level to replace the plane, 1..50 |
| `occlusion.enabled` | `true` | do not read a cell hidden behind a nearer occluder as a drop or ramp |
| `occlusion.azimuth_bin_deg` | `4.0` | width of an azimuth bin of the occluder horizon, degrees, 0.1..10 |
| `occlusion.elevation_margin_deg` | `2.0` | slack on the shadow ray elevation, degrees, 0..5 |
| `occlusion.min_occluder_height_m` | `0.03` | lowest plane-relative height that occludes, metres, > 0 and < bands.obstacle_max_m |
| `occlusion.min_height_drop_m` | `0.005` | smallest height drop behind an occluder that counts as shadowed, metres, > 0 |
| `slope.roll_warn_deg` | `8` | tilt_guard roll warning limit, degrees |
| `slope.roll_error_deg` | `15` | tilt_guard roll error limit, degrees, > roll_warn_deg |
| `slope.pitch_warn_deg` | `12` | tilt_guard pitch warning limit, degrees |
| `slope.pitch_error_deg` | `20` | tilt_guard pitch error limit, degrees, > pitch_warn_deg |
| `colour.enabled` | `true` | use the floor colour model |
| `colour.max_mahalanobis` | `3.0` | distance from the floor colour above which a point is flagged, > 0 |
| `colour.forgetting` | `0.1` | weight of new samples in the colour model, 0 < v <= 1 |
| `colour.min_floor_samples` | `200` | floor samples needed before flagging, >= 10 |
| `state.rate_hz` | `2.0` | state topic rate, hertz, 0 < v <= 10 |
| `temporal.enabled` | `true` | take a per-cell temporal median of the plane-relative heights before segmentation |
| `temporal.cell_size_m` | `0.10` | edge of a fixed-frame cell, metres, 0.02 <= v <= 0.5 |
| `temporal.window_frames` | `8` | per-frame values a cell keeps, 1..30 |
| `temporal.min_frames` | `4` | per-frame values a cell needs before its points are output, 1..window_frames |
| `temporal.max_age_s` | `1.5` | per-frame values older than this are dropped, seconds, 0 < v <= 5 |
| `temporal.max_spread_m` | `0.025` | largest robust spread of a cell's per-frame medians before the cell is unknown, metres, 0 < v <= 0.2 |
| `hysteresis.enabled` | `true` | keep a cell's class until a different class holds enough of the recent frames |
| `hysteresis.window_frames` | `5` | recent frames a cell's class is judged over, 1..30 |
| `hysteresis.enter_frames` | `3` | frames of a class (within the window) needed to enter it, exit_frames..window_frames, a strict majority of the window |
| `hysteresis.exit_frames` | `1` | frames of the new class needed to leave the current one, 1..enter_frames |
| `metrics_port` | `19119` | Prometheus port, 1..65535, or null to disable |

The spread in `temporal.max_spread_m` is computed over the per-frame cell medians. With fewer than 5 values it is 1.4826 * MAD (1.4826 turns the MAD into a Gaussian sigma). From 5 values on it is the standard deviation after dropping the most deviant value, times 1.3: it is as robust to one outlier frame as the MAD but varies far less between windows (8 frames: a plain MAD slips under the limit by chance). Dropping a value shrinks the estimate to ~0.77 of sigma, so 1.3 is the consistent factor (1.35 was tried and failed the >= 95 % floor-visibility bound). Measured bounds (20+ seeds, 8-frame window): far-band cells of 43-90 mm spread leak in at most 5 % of cell-frames, floor cells of 4-20 mm stay visible in at least 95 % of cell-frames (a single 20 mm cell about 85 %); the class hysteresis and lethal point gates downstream absorb the leaks. A gated cell is unknown (no points, so never lethal, a step, a drop or part of a cluster), keeps its history, and is known again as soon as its spread is back under the limit. No range cap is used.

Measured bounds (24 fixed seeds, 8-frame window, `test_cell_median.cpp`): 8 values estimate a spread with ~25% scatter, so no limit separates 20 mm from 30 mm with certainty. Cells with 4..20 mm per-frame noise are visible in >= 95% of frames; cells with 30..42 mm temporal std are visible in <= 10% of cell-frames; cells with 43..90 mm are visible in <= 5% (not zero, the worst case is 43 mm). A single 20 mm cell is visible only ~85% of the time; the class hysteresis and lethal point gates downstream absorb those flickers.

## Robustness

How the node keeps noisy ToF returns from becoming marks (2026-10-21 capture: ~200 lethal points per frame on empty floor).

- **Rule**: a surface is an obstacle above 0.04 m or steeper than 15 deg (`traversability.max_slope_deg`, was 8 deg: the
  ~14 deg ramp is safe). `max_slope_deg` is bounded by the tilt_guard error limits (`slope.roll_error_deg` 15 and
  `slope.pitch_error_deg` 20), not the warn limits; the config rejects a larger value. Exactly 15 deg is a warning.
- **Temporal spread gate**: a cell whose robust spread of per-frame medians exceeds `temporal.max_spread_m` (0.025 m) is
  gated: unknown, never lethal, never part of an edge, a step or a drop. It is known again when its spread falls back.
  Gated cells are counted in `ground_temporal_gated_cells`.
- **Median edge step**: a region's step is the median of |height| in the `traversability.edge_band_m` strip, not the p90,
  so a 3 cm sill stays a warning and a 5 cm box stays an obstacle.
- **Robust slope**: a Huber plane fit over one sample per `edges.slope_bin_m` bin. Fewer than `edges.slope_min_cells`
  bins, or bins in a line, give `slope_known` false and slope 0 (never an obstacle by slope).
- **Ramp slope refit**: a cluster whose cell-median fit reads at least `SLOPE_REFIT_MIN_DEG` (5 deg) is refit from the
  raw heights of the last `SLOPE_HISTORY_WINDOWS` (2) x `temporal.window_frames` clouds in its cells and their
  neighbours (`surface_slope.hpp`), so a small ramp is judged on its whole surface. A new incline is fit over the
  clouds kept so far (a steep one is lethal from its first frame); a failed refit leaves the slope unknown.
- **Steady lethal points**: a lethal point counts only when its cell is steady (`CellMedian::steady_at`): output with a
  spread of at most 0.6 x `temporal.max_spread_m`, and neither it nor any of its 8 neighbours gated in the last
  2 x `temporal.window_frames` frames. This drops the floor/band mix at the edge of a noisy far band.
- **Colour clusters** (label 3) are warning-only: never lethal, whatever their slope.
- **Lethal points**: unclustered lethal points count only when at least `traversability.lethal_min_points` lie within
  `edges.cluster_tolerance_m` of each other.
- **Cell class hysteresis** (`hysteresis.*`): each fixed-frame cell keeps its class (free, warning, obstacle, drop) until
  another class holds `hysteresis.enter_frames` of the last `hysteresis.window_frames` frames; a cell stays in its
  class while that class still holds at least `hysteresis.exit_frames` of the window, so with `exit_frames` 1 a
  lethal cell drops only after a whole window with no lethal vote. At 10 Hz a new lethal cell therefore appears after 3 frames, a latency
  of 0.3 s. Stable cells are counted in `ground_stable_lethal_cells` and `ground_stable_warning_cells`.
- **Cell-aligned warning areas**: warning areas are built from the stable warning cells, so their polygons follow the
  cell grid and do not churn with single frames. costmap_zones keys them by cell and publishes the masks only on change.
- **Plane smoothing**: `fit.smoothing` is 0.2, so the plane follows the head slowly once the spread gate removed the
  noise.

## Metrics

Prometheus on `metrics_port`.

| Metric | Type | Meaning |
|---|---|---|
| `ground_fits_total` | counter, label `result` | plane fits, `ok` or the reject reason |
| `ground_fit_seconds` | histogram | plane fit and segmentation time per ToF cloud |
| `ground_inlier_ratio` | gauge | inlier ratio of the last valid fit |
| `ground_slope_deg` | gauge | floor slope against gravity of the last cloud with a plane and a fresh IMU |
| `ground_plane_vs_base_deg` | gauge | angle between the floor normal and base_link +z |
| `ground_regions` | gauge | step and drop regions of the last cloud with a plane |
| `ground_warning_areas` | gauge | warning areas (ToF and colour) of the last cloud with a plane |
| `ground_obstacle_points` | gauge | points of the last obstacle cloud |
| `ground_drop_points` | gauge | points of the last drop cloud |
| `ground_occluded_points` | gauge | points of the last cloud removed as lying in an occlusion shadow |
| `ground_temporal_cells` | gauge | cells the temporal median currently holds (0 when the plane is lost or temporal is disabled) |
| `ground_temporal_gated_cells` | gauge | cells the temporal spread gate currently holds unknown |
| `ground_stable_lethal_cells` | gauge | cells whose hysteresis class is lethal (obstacle or drop) |
| `ground_stable_warning_cells` | gauge | cells whose hysteresis class is warning |
| `ground_colour_mismatch_ratio` | gauge | floor-colour mismatch share of the floor band of the last labelled RGB-D cloud |
| `ground_clouds_dropped_total` | counter, label `reason` | clouds or warning-area messages dropped, `reason` is `tf`, `no_plane_rgbd`, `no_map_tf` or `bad_input` |
| `ground_labelled_clouds_total` | counter | labelled RGB-D clouds published |

## Plane tracking

- IMU prior: the gravity direction from `/imu/data`, rotated into base_link, is the expected floor normal. The level
  reference (`imu.level_reference_roll_deg`, `imu.level_reference_pitch_deg`) is the IMU reading on level ground and
  is removed so a level base gives +z. Without a fresh IMU (`imu.timeout_s`) the prior is +z of base_link and `prior`
  is `base_link`.
- A fit must stay within `fit.max_normal_angle_deg` of the prior and `fit.max_offset_m` of the nominal floor.
- `PlaneTracker` keeps the recent fits and answers by stamp: the newest fit at or before the queried stamp, with
  `source` `fit` when it is that cloud's own fit and `held` when an older one is reused. A fit older than `fit.hold_s`
  is not used: there is no estimate beyond `hold_s`, and the plane is null with a reason.
- `fit.smoothing` blends each new plane into the running estimate.

## No placeholder data

- Without a plane no obstacle, drop or labelled cloud is published; the node never publishes a cloud built on an
  assumed floor.
- The state then carries `plane: null` and a `reason`.
- There is no state before the first ToF cloud, and no warning areas are published without a plane.

## Replay tests

`test/test_replay.cpp` (gtest `test_replay`, part of `scripts/cpp-node-test.sh ground_estimator_cpp`) feeds the
2026-10-22 live recordings of the user's scene through the real `GroundNode` and prints one metrics line per scene.
The fixtures live inside the package because `cpp-node-test.sh` copies only the packages into the container.

### Fixtures

| File | Scene (robot static, head tilt 29.4 deg) |
|------|------------------------------------------|
| `test/data/replay/home.bin` | head pan 0: board near, box on the left, far floor |
| `test/data/replay/box.bin` | head pan +35 deg: the 5 cm box and the floor |
| `test/data/replay/ramp.bin` | head pan -30 deg: the black rubber ramp and the board edge |

Each holds 15 s (150 clouds, 375 IMU samples) of `/tof/points_filtered` as delivered (decimation 4, 5 cm voxel, self
filter applied; the ToF mixed pixels are in the data), recorded seconds 5-20 of the 40 s recordings. Regenerate a
fixture from its recording (numpy npz, keys and layout in the converter docstring) with:

```bash
uv run python scripts/ground_replay_export.py <recording>/ground_home.npz nodes/ground_estimator_cpp/test/data/replay/home.bin
```

The trim window is the constants `START_S` (5.0 s after the first cloud) and `DURATION_S` (15.0 s) at the top of
`scripts/ground_replay_export.py`; its `read_replay(path)` decodes the same layout in Python.

### Binary layout

Little-endian, version 1 (`REPLAY_VERSION`; the C++ reader `ground_replay::read_replay` rejects another magic or
version, a truncated file and trailing bytes):

| Field | Type |
|-------|------|
| magic | 8 bytes `GRNDRPLY` |
| version | uint32 `1` |
| cloud frame id | uint32 length, then that many UTF-8 bytes (`tof_optical_frame`) |
| cloud count | uint32 |
| per cloud | int64 `stamp_ns`; float64[7] `base_T_cloud` (base_link <- cloud frame); float64[7] `odom_T_base` (odom <- base_link), both `x y z qx qy qz qw` at the cloud stamp; uint32 point count; int16[count x 3] `x y z` in mm in the cloud frame |
| IMU count | uint32 |
| per IMU sample | int64 `stamp_ns`; float64[10] orientation `x y z w`, linear acceleration `x y z`, angular velocity `x y z` (frame `imu_link`) |
| `base_T_imu` | float64[7] `x y z qx qy qz qw` (base_link <- imu_link) |

### How a scene is replayed

- Config: the shipped `config/ground_estimator.yaml` through `validate` and `from_yaml`; only the cloud, IMU and
  RGB-D topics, the output namespace (unique per test) and `metrics_port` (off) are overridden.
- Time: the scene is restamped to the wall clock keeping its relative timing and replayed in real time: each cloud is
  published at its stamp, after the IMU samples up to that stamp, as a PointCloud2 (float32 x y z in metres in the
  cloud frame).
- TF: a `tf2_ros::Buffer` holds `base_link <- tof_optical_frame` and `odom <- base_link` at every cloud stamp, a
  static identity `map <- odom` and a static `base_link <- imu_link`.
- Outputs: the test waits for the cloud's own outputs before the next cloud (obstacle and drop clouds of its stamp,
  base_link, and the `/ground/warning_areas` document of its stamp, map frame). A cloud the node processed (counted in
  `ground_fits_total`) whose obstacle cloud does not arrive within 0.5 s was not answered with a plane.
- Every scene test asserts that the node answered with a plane in at least one frame and prints the metrics line;
  the metrics themselves are not asserted.

### Metrics and ROIs

Base_link coordinates, the D14 ground truth measured from the recordings. Every metric ignores points (and warning
areas whose centroid lies) at x < 0.37 m, the robot's own parked gripper. Counted frames are the frames at least 2 s
after the first cloud that the node answered with a plane. ROI edges are inclusive; a share with no counted frame is
`nan`.

| ROI | x (m) | y (m) | Meaning |
|-----|-------|-------|---------|
| `BOX_LETHAL_ROI` | 0.57 .. 0.81 | 0.02 .. 0.36 | the box top (x 0.60-0.78, y 0.05-0.33) dilated by 3 cm |
| `BOARD_ROI` | 0.37 .. 0.57 | -0.12 .. +0.14 | the flat ChArUco board, stopping 3 cm before the box front |
| `FAR_SIDE_ROI` | 0.78 .. 0.95 | 0.05 .. 0.33 | the occluded floor behind the box |
| `RAMP_ROI` | 0.75 .. 1.10 | -0.45 .. -0.15 | the black rubber ramp (no ToF returns) |
| stray zone | >= 0.37, hypot(x, y) <= 1.2 | | everything within 1.2 m outside `BOX_LETHAL_ROI` |

| Metric | Definition |
|--------|------------|
| `plane_share` | answered frames / frames after the 2 s warm-up |
| `box_lethal_share` | share of counted frames with an obstacle point in `BOX_LETHAL_ROI` |
| `board_lethal_frames` | counted frames with an obstacle or drop point in `BOARD_ROI` |
| `stray_lethal_share` | share of counted frames with an obstacle or drop point in the stray zone |
| `far_side_warning_frames` | counted frames with a `ramp` or `step` warning area whose polygon centroid (vertex mean, in base_link) lies in `FAR_SIDE_ROI` |
| `ramp_lethal_frames` | counted frames with an obstacle or drop point in `RAMP_ROI` |
| `churn_share` | share of consecutive counted frames whose warning set differs: kind plus polygon vertices rounded to 1 cm, as `scripts/ground_noise_report.py:warning_set` |

### Baseline

Measured by `scripts/cpp-node-test.sh ground_estimator_cpp` on the shipped config as committed with these fixtures;
two runs gave identical lines:

```text
replay home: plane_share=1.000 box_lethal_share=0.869 board_lethal_frames=116 stray_lethal_share=1.000 far_side_warning_frames=5 ramp_lethal_frames=0 churn_share=0.690
replay box: plane_share=1.000 box_lethal_share=0.519 board_lethal_frames=0 stray_lethal_share=0.512 far_side_warning_frames=21 ramp_lethal_frames=0 churn_share=0.391
replay ramp: plane_share=0.984 box_lethal_share=0.000 board_lethal_frames=22 stray_lethal_share=0.220 far_side_warning_frames=0 ramp_lethal_frames=4 churn_share=0.183
```
