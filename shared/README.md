# Shared libraries

Python code shared by multiple nodes. `shared/` is one installable uv package, `ros2-common` (import name
`ros2_common`, see [pyproject.toml](pyproject.toml)). Node code is in [nodes/](../nodes/).

See [CLAUDE.md](../CLAUDE.md): use Python 3 type hints and extend unit tests as the project grows.

A second, separate package lives in [ros2_metrics/](ros2_metrics/README.md): `ros2-metrics`, the Prometheus
exporter helper of our nodes (only `prometheus-client`).

The C++ nodes share [ros2_common_cpp/](ros2_common_cpp/), an ament_cmake package with config validation and metrics;
see [ros2_common_cpp (C++)](#ros2_common_cpp-c) below.

## Using it from a node

Add ros2-common as an editable path dependency, relative to the node directory:

```toml
[project]
dependencies = ["ros2-common"]

[tool.uv.sources]
ros2-common = { path = "../../shared", editable = true }
```

Then run `uv lock` once. On the Pi the whole repo is cloned to `ros2_repo_dest`, and the `ros2_node_deploy` role
runs `uv sync --frozen --no-dev` inside `<ros2_repo_dest>/nodes/<node>` into `/opt/ros2-nodes/<name>/venv`, so the
same `../../shared` path resolves there; nothing extra is needed in Ansible. A node one level deeper (e.g.
`nodes/bridges/<node>`) uses `../../../shared`.

## Modules

| Module | Contents |
|---|---|
| `ros2_common._utils` | `clamp(value, low, high)` |
| `ros2_common.battery` | `BatteryConfig` (pydantic: `topic` `/battery_state`, `cells` 3, `cutoff_cell_v` 2.8, `resume_cell_v` 2.9, `stale_s` 5.0; `resume_cell_v >= cutoff_cell_v`, `cells >= 1`) and `BatteryGuard` (thread-safe cut-off hysteresis: enter below `cells * cutoff_cell_v`, leave only above `cells * resume_cell_v`; no reading or older than `stale_s` = unknown = not blocked) |
| `ros2_common.camera_geometry` | Pixel <-> ground-plane maths and mount calibration (see below) |
| `ros2_common.lookups` | Camera lookup contract of mcp_server (writer) and web_ui (reader): `LookupRecord` (one `index.jsonl` line), `LookupQuery` + `NearFilter` (filters ANDed; `since` / `until` take unix seconds or ISO-8601, naive = UTC, via `parse_time`), `parse_line`, `record_summary` (record + ISO `time`) and `IndexCache` (parses the index only when its inode / mtime / size changed; corrupt lines warn once per distinct line; `appended()` lets the writer skip a re-parse) |
| `ros2_common.marker_payload` | Marker payload codec (see below): `MarkerPayload`, `encode_payload`, `decode_payload`, `PayloadError`, `MarkerInfo`, `marker_info` |
| `ros2_common.photos` | Photo POI contract (see below): `PhotoView`, `camera_view`, `effective_view`, `photo_as_poi`, `PhotoAdd`, `PhotoSelector`, `write_atomic`; also owns `PlanarPose` / `CameraPose` (re-exported by `lookups`) |
| `ros2_common.maps` | Named maps topic contract of map_manager (see below); no ROS imports |
| `ros2_common.poi_categories` | POI category tags in the POI `note` (see "POI categories" below): `CATEGORIES`, `PROBLEM_TYPES`, `PoiTags`, `category_tags`, `parse_tags`, `apply_tags`, `strip_tags`, `has_tag`, `check_category`; no ROS imports |
| `ros2_common.scan_agreement` | Scan-to-map agreement score (see below); numpy only |

Used by: `nodes/mcp_server` (motion tools refused in cut-off; lookup store) and `nodes/web_ui` (lookup routes; commands rejected in cut-off; uv path source `ros2-common = { path = "../../shared", editable = true }`).

## POI categories (`ros2_common.poi_categories`)

poi_store keeps `extra="ignore"` on `Poi`, so a new field would be dropped. A category is therefore a tag on the first line of the existing `note`; everything after the first line is free text. No new schema field.

| Category | kind | zone | Required fields | Tag |
|---|---|---|---|---|
| target | point, object or area | none | none | `#target` |
| slow | area | slow | `speed_limit_pct` | `#slow` |
| keepout | area | keepout | none | `#keepout` |
| problem | point or area | none or slow | problem type | `#problem/<type>` |
| detail | point or area | none | none | `#detail` |

Problem types: `window`, `mirror`, `glass`, `tight`, `overhang`, `low_obstacle` (overhang and low_obstacle are obstacles above or below the lidar plane), `drop`, `cable`, `soft_floor`, `other`.

Tag grammar: the first line holds space-separated tags, one category tag plus optional `#survey` and `#survey-spot`. Unknown `#words` are ignored; a first line with no known tag means no category.

```
#problem/window #survey
Floor-to-ceiling glass, lidar sees through it.
```

```
#slow
Hallway with a rug, 40 percent.
```

`apply_tags` replaces the category tag of an existing tag line, keeps `#survey` / `#survey-spot` already there, never duplicates a tag and keeps the free text; a result over 2000 characters (poi_store's limit) raises `ValueError`. `check_category` validates a category against kind, zone, problem type and `speed_limit_pct` with one message per rule.

Consent rule (D4): creating a keep-out is free; cancelling or deleting one needs the person's consent.

## Marker payload (`ros2_common.marker_payload`)

Pydantic, no ROS. One canonical, versioned text per marker, read the same way by the generator, the detector and poi_store.

Grammar (QR alphanumeric set `0-9 A-Z space $%*+-./:` only, so lower case is rejected):

```
P<major>[.<minor>]/K:<KIND>/N:<NAME>/S:<MM>[/<KEY>:<VAL>...]
```

- `P`: major must be 1 (otherwise `unsupported payload version N`); minor is optional and ignored.
- `K` kind `[A-Z0-9-]{1,16}`, `N` name `[A-Z0-9-]{1,20}`, `S` symbol edge in mm (20..1000, at most one decimal, printed without `.0`). All three are required.
- Other pairs are params: key `[A-Z0-9]{1,8}` (not P/K/N/S), value `[A-Z0-9 $%*+.-]{0,24}`. Duplicate keys and empty segments are rejected.
- Encoding writes P, K, N, S, then params sorted by key, so a payload has one text. Example `P1/K:CHARGER/N:DOCK-1/S:150`.
- `decode_payload` raises `PayloadError` (a `ValueError`) naming the problem; `decode_payload(encode_payload(p)) == p`.

| K | POI kind |
|---|---|
| `BASE` | `base` |
| `CHARGER` | `charger` |
| `DROPOFF` | `drop_off` |
| `BUTTON` | `button` |
| `SWITCH` | `switch` |
| `TOOLSTATION` | `tool_station` |
| any other valid K | its lower-case form (e.g. `LAMP` -> `lamp`) |

The marker key is `<poi_kind>/<N>`. `marker_info(payload, pose_samples=1)` builds `MarkerInfo` (`kind`, `id`, `key`, `size_mm`, `params`, `payload` = canonical text, `version`, `pose_samples`).

Size: `S` is the symbol edge from the outer edge of one finder pattern to the outer edge of the opposite side, excluding the quiet zone; module edge = S / (17 + 4 * version).

Marker frame: origin at the symbol centre, +x along the top edge (the edge with the two upper finders, left to right as read), +y along the left edge towards the top, +z out of the printed face towards a viewer.

## Named maps contract (`ros2_common.maps`)

Shared by `map_manager`, `web_ui` and `mcp_server`; pure pydantic v2 models, no ROS imports.

| Topic | Constant | Payload |
|---|---|---|
| `/map_manager/command` | `COMMAND_TOPIC` | `MapCommand` JSON |
| `/map_manager/result` | `RESULT_TOPIC` | `MapResult` JSON |
| `/map_manager/status` | `STATUS_TOPIC` | `MapStatus` JSON (latched) |
| `/map_manager/localised` | `LOCALISED_TOPIC` | Bool, published after a load verdict |

- `validate_map_name(name)`: `MAP_NAME_PATTERN` `^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$`, else `ValueError` naming the rule.
- `Pose2D`: `x`, `y`, `yaw`, all finite.
- `MapCommand` (`extra="forbid"`): `op` `save`/`load`/`delete`/`relocalise`/`clear_live`, `request_id`, `name` (required and validated for save/load/delete, empty for relocalise and clear_live), `overwrite`, `pose` (load only), `note` (<= 500 chars, save only), `since` and `discard_on_fail` (relocalise only). `clear_live` empties the live POIs and photos and takes no name, pose, note, overwrite, `since` or `discard_on_fail` (`NAMED_OPS` stays save/load/delete).
- `MapResult`: `request_id`, `ok`, `message`, `data`.
- `Localisation`: `state` (`unknown`/`loading`/`localised`/`not_localised`/`ambiguous`), `ratio` (0..1 or None), `pose`, `message`, `updated_at`.
- `MapSummary`: `name`, `saved_at`, `note`, `poi_count` (non-photo POIs), `photo_count` (photos of the bundle, default 0 for old bundles and status payloads), `last_pose`, `valid`, `error` (non-empty only for an invalid bundle).
- `MapStatus`: `maps`, `active_map`, `load_seq` (>= 0), `localisation`.
- Busy rule: one operation runs at a time; a second command is answered `ok: false` with `busy: <op> <name>`.
- Every model round-trips through `model_dump(mode="json")` / `model_validate`.

## Scan agreement (`ros2_common.scan_agreement`)

The scan-to-map agreement metric, shared by map_guard (mismatch detection) and map_manager (judging a relocalisation),
so both score a scan the same way. Pure numpy, no ROS imports. Moved unchanged from `map_guard.agreement`, which
re-exports it.

- `GridMap` (resolution, origin, origin yaw, `cells` int array, -1 unknown), `PreparedMap`, `AgreementResult` (`ratio`, `evaluated`, `agreeing`).
- `prepare_map(grid, occupied_threshold, hit_radius_cells)`: dilated hit/known masks, computed once per map.
- `scan_endpoints(ranges, angle_min, angle_increment, range_min, range_max, max_range_m)`: laser-frame endpoints of valid beams.
- `transform_points(points, translation, rotation)` with `yaw_quaternion` / `quaternion_matrix`: laser frame to map frame.
- `agreement(points_map, prepared, min_points)`: share of evaluated endpoints next to an occupied cell; `None` without a map or with fewer than `min_points` evaluated points. Unknown space is not evaluated.

Tests: `tests/test_shared_scan_agreement.py`.

## Photo contract (`ros2_common.photos`)

A photo is a POI: one `LookupRecord` (an `index.jsonl` line) plus `<id>.jpg`. `LookupRecord` gained defaulted fields `kind` (`"photo"`), `name` (<=60), `status` (`open`/`done`/`cancelled`), `created_by` (`agent`/`user`), `updated_at` (0 = never edited) and `view`, so legacy lines parse unchanged.

- `PhotoView`: camera pose in the map (`x`, `y`, `z`, `yaw`, `pitch` positive = looking down, `hfov`, `source` `camera_mount` or `base_heading`), all finite.
- `camera_view(base_pose_map, camera_pose_base, hfov)`: base pose composed with the mount; yaw = base yaw + heading of the mount body x axis (fixed-axis xyz rpy). Without a mount: `base_heading` (z, pitch, hfov None). Without a base pose: None.
- `effective_view(rec)`: stored view, else derived from the stored base pose and mount.
- `photo_as_poi(rec)`: POI dict; `x` / `y` are None (never 0) when there is no pose.
- `PhotoAdd`: add payload (32-hex `id` required, `file == "<id>.jpg"`, no `bytes`, unknown keys rejected).
- `PhotoSelector`: exactly one of `ids` (1..500), `older_than_s` (>0), `keep_newest` (>=0); optional `created_by`, `camera`, `dry_run`.
- `write_atomic(path, data)`: fsynced temp file (0644) then rename; `mcp_server.lookup_store` re-exports it.

## Camera geometry (`ros2_common.camera_geometry`)

Conventions (ROS REP-103):

- `MountPose` is the camera BODY frame (x forward, y left, z up) in `parent_frame`, in meters and radians.
- Orientation is fixed-axis RPY like URDF `rpy` (rotate about fixed x, then y, then z).
- The OPTICAL frame (z forward, x right, y down) is the body frame times the standard camera_link -> optical
  rotation (roll -pi/2, yaw -pi/2). `optical_from_mount(T_parent_mount)` applies it, so
  `T_frame_optical = optical_from_mount(T_frame_parent @ T_parent_mount)`.
- Pixels are (u right, v down). `T_a_b` maps points of frame b into frame a.

API: models `CameraIntrinsics` (`from_camera_info(width, height, k, d, distortion_model)` takes the plain fields of a
sensor_msgs/CameraInfo, keeps the 5 plumb_bob coefficients and raises `ValueError` naming the field for a zero,
non-finite or wrong-length `k` or a model other than `plumb_bob`/empty; `from_calibration_yaml(path)`, `from_hfov(w, h, hfov_deg)` - an approximation: square
pixels, centred principal point, no distortion), `MountPose` (`to_matrix()`), `CameraModel` (`calibrated`);
functions `undistort_pixel`, `pixel_to_ray_camera`, `camera_ray_in_frame`, `intersect_ground` (None if parallel or
behind), `pixel_to_ground(intr, T_frame_optical, u, v, ground_z)` (None if the ray misses the floor),
`project_point_to_pixel` (None if behind the camera or off-image), `optical_from_mount`, and
`solve_mount_pose(samples, intr, initial)` returning `(MountPose, rms_px)` (RMS Euclidean reprojection error per
sample; at least 3 samples, each `(T_frame_parent, (u, v), (x, y, z))`), and
`depth_to_points(depth, intrinsics, depth_scale_m=0.001, step=1, min_m=None, max_m=None)`: numpy-only deprojection of a
16UC1 (scaled by `depth_scale_m`, mm by default) or float32 (metres) depth image of the intrinsics' size into an
N x 3 optical-frame array (x right, y down, z forward); zero, non-finite and out-of-range pixels are dropped, `step`
subsamples, a size mismatch raises `ValueError`. Distortion is ignored (the D435 depth stream is rectified).

Calibration procedure:

1. Put a marker at measured floor points in front of the robot (known x, y, z in the ground frame).
2. For the gripper camera, capture the marker pixel plus the parent pose (`T_frame_parent`, e.g. from FK) at several
   arm poses; for the fixed overview camera, capture several marker positions (parent pose is identity).
3. Write a samples JSON (format in the script docstring) and run
   `python scripts/solve_camera_mount.py samples.json --camera gripper`.
4. Paste the printed YAML into the Ansible node config; the RMS should be around 1 px or less.

## head_rgbd_calibration

The head RGB-D calibration core. It never imports `scripts`, `rosbags`, `subprocess` or ssh code, so node venvs can use it.

- `config`: `StrictModel` (pydantic, unknown keys rejected) and the `BoardConfig`, `SelectionConfig`, `LimitsConfig` settings, plus the floor fit models `FloorPlaneConfig` (RANSAC distance, iterations, `min_inliers`, `min_inlier_ratio`, `max_range_m`, `candidate_band_m` > the RANSAC distance, `seed`), `FloorFitLimits` (observation, pose and pan span minimums; pose-mean height and normal RMS, sigma and delta-from-URDF maximums) and `FloorFitConfig` (`plane`, `limits`, the residual scales `sigma_height_mm` / `sigma_normal_deg` and `huber_scale`). The head-from-arm models are `BoardMountConfig`, `HeadArmFitLimits`, `BaseInBaseLink` and `HeadArmFitConfig` (see "Head from arm fit").
- `charuco`: `make_board`, `detect` and `Detection` (ChArUco detection for the RGB image and the normalised ToF amplitude).
- `frames`: `FrameSet`, `StampedImage`, `CameraInfoData`, `FramePair`, `normalise_amplitude`, `rgb_difference` (stillness measure with mains flicker removed), `tof_diff_fraction` and `still_pairs` (one RGB and ToF pair per still window).
- `poses`: `PoseCounter(board, sel, start_count=0)`, the live pose counter. `add(kind, frame, raw=None)` takes `RGB`, `AMPLITUDE` and `DEPTH` frames and returns a `PoseEvent(status, pose)` per judged RGB frame; `pose` is a `CapturedPose` (number, RGB frame and its compressed bytes, amplitude, the depth frame nearest the amplitude within `max_pair_dt_s` or None, and both detections) when a new still window has the board in both cameras. `status` is `WAITING` until frames arrived. Same stillness and pairing rules as `still_pairs`; history is kept `TOF_HISTORY_S`.
- `intrinsics`: `calibrate`, `calibrate_tof`, `compare_sdk`, `IntrinsicsResult` and `write_camera_calibration_yaml` (camera intrinsics from ChArUco detections).
- `extrinsics`: `stereo_calibrate`, `plane_depth_bias`, `ExtrinsicResult`, `PlaneCheck` and the extrinsics YAML read/write (ToF to RGB stereo extrinsic and board-plane depth check).
- `hints` (`hints.py`): `capture_hints(rgb, tof, board)` returns the capture-time operator hints of a pose (few RGB corners, few ToF corners, under 6 shared corners; `FEW_CORNERS_FRACTION` 0.5 of the board's inner corners).
- `calibrate` per-pose quality: `calibrate_detections(..., pose_ids=None)` fills `CalibrationRun.poses`, a tuple of `PoseErrors(pose_id, rgb_px, tof_px, stereo_px, outlier)`; `flag_outliers(errors, limit)` flags errors above `max(OUTLIER_MEDIAN_FACTOR x median, limit)`; `outlier_pose_ids(run)`, `pose_report_lines(run)` (report section) and `failed_limits` naming flagged poses. `extrinsics.ExtrinsicResult` gains `pair_indices` and `per_pair_errors` (stereo error per pair). Hints only: no view is dropped.
- `urdf_handoff`: `read_chain`, `tof_joint_origin`, `check_drift` and the matrix helpers; re-exports `rewrite_tof_joint`.
- `chain`: `UrdfChain` (numpy forward kinematics over URDF text: `from_text`, `transform(link, positions, origins, offsets)` with urdf_angle = measured + offset, `joint_names`, `origin`, `limits`), `pose_from_xyz_rpy`, `xyz_rpy_from_pose` and `arm_in_robot_base`. Shared by the head fit, the arm fit and the arm pose sequence.
- `views`: `ArmBoardView` (one still view of the gripper board: view id, arm pose index, tier, the seven measured joints, the board `Detection`, the head offset index) and `require_joints`; see "Head from arm fit" below.
- `head_arm_fit`: the head-from-arm head chain fit, `aim_head` and the report; see "Head from arm fit" below.
- `arm_pose_check`: the FK safety check of arm poses and joint paths with the board on the gripper (numpy, no ROS). `PoseChecker(...)` evaluates `urdf = measured + offset` FK placed in base_link by `arm_in_robot_base`; `check_pose(joints, gripper_rad, rolling)` returns named rule violations (body collisions, head envelope, board margin, roll rules, stretched arm, joint limits) and `check_path(start, target, gripper_rad)` samples the path between two poses. The body is `BodyConfig` (boxes and vertical cylinders, equal to the client.yml mcp_server `body` block), configured through `PoseCheckConfig`; `head_envelope(head_urdf, cfg)` gives the vertical cylinder containing the head at every pan/tilt. Distances are signed (negative = penetration). Tests: `tests/test_arm_pose_check.py`.
- `floor_fit`: the floor-plane head chain fit (numpy and scipy, no ROS). `predicted_floor(chain, pan, tilt, origins, sensor_frame)` gives the floor (base_link z = 0) in the depth sensor frame (`sensor_frame`, default `d435_depth_optical_frame`, the frame of `/head/d435/points`) as (unit normal toward the sensor, sensor height); `fit_plane(points, prior, cfg, rng)` gates finite points by range and by `candidate_band_m` around the prior plane, runs RANSAC and an SVD refit and returns a `PlaneFit(normal, height_m, inliers, candidates, rms_m)` or None; `fit_head_chain_floor(observations, urdf_text, cfg, sensor_frame)` fits `head_mount_joint` z, roll, pitch and the `head_tilt` origin pitch (tilt zero) and roll from `FloorObservation(pose, pan_rad, tilt_rad, plane)` with a Huber `least_squares`, every other origin component kept at the URDF value (d435_joint and the D435 optical joints included), and returns a `FloorFitResult` (`passed`, `failed_limits`, `params` value/sigma/urdf for `mount_z_m`, `mount_roll_deg`, `mount_pitch_deg`, `tilt_zero_deg`, `tilt_roll_deg`, the pose-mean `height_rms_mm` / `normal_rms_deg` the RMS limits apply to, the per-observation `observation_height_rms_mm` / `observation_normal_rms_deg`, per-pose pose-mean and per-observation residuals, the new `origins`, `to_json()` with non-finite numbers as null). `pose_mean_errors` gives one pose's mean height error and the angle between its mean observed and predicted normals. Fewer than `min_observations` skips the solver.
- `urdf_text`: `rewrite_joint_origin`, `rewrite_tof_joint`, `Origin`, `format_vec`, and the override pieces: the `head_chain.json` format `{"joints": {head_mount_joint|head_pan|head_tilt|d435_joint: {"xyz": [3], "rpy": [3]}}, "note", "session", "result"}` (`parse_head_chain`, `head_chain_json`) and `apply_overrides(urdf_text, applied_dir)`, the one applier that applies `head_chain.json` then `tof_joint.json` and returns the text plus one warning per unreadable or invalid file. A `head_chain.json` naming the pre-D435 `head_camera_joint` is skipped whole with a warning that the fit predates the D435 (re-run head-from-arm calibration). It must stay standard-library only (no numpy, scipy or package-relative imports): the head_state_publisher launch loads it by file path on the system Python.

Imported by the head_rgbd_calibration node.

## Head from arm fit

The head-from-arm fit calibrates the head chain against the trusted arm: the ChArUco board is on the gripper, the arm
(measured joints plus the trusted joint offsets) says where the board is, and the head camera sees it at different arm
poses and head pan/tilt angles. Everything is numpy, scipy and OpenCV, no ROS. The node side (capture, sequence, apply)
is in `nodes/head_rgbd_calibration/README.md`.

### Views (`views.py`)

`ArmBoardView(view_id, arm_pose, tier, joints, detection, head_offset)` is one still frame with the board detected.
`joints` holds the measured radians of all seven joints (`head_pan`, `head_tilt` and the five arm joints, never
defaults); `arm_pose` is the index of the scripted arm pose, `head_offset` the index of the (pan, tilt) offset the view
was taken at. `require_joints(view)` raises `ValueError` naming the view and every missing joint.

### Model and parameters (`head_arm_fit.py`)

For each detected board corner the model chains: board corner -> board pose in `gripper_link` (fitted, 6 DOF) -> arm FK
at the measured joints plus `joint_offsets_rad` (`urdf = measured + offset`) -> arm root in robot `base_link`
(`base_in_base_link`) -> the inverse of the `camera_frame` pose (default `d435_color_optical_frame`) of the head chain at
the measured pan/tilt -> `cv2.projectPoints` with the D435 colour intrinsics (the factory camera_info). The residual is the pixel distance to the detected corner,
under a Huber loss (`huber_f_scale_px`).

Free parameters (`fit_head_from_arm`):

- `PARAM_NAMES`: `mount_x_m`, `mount_y_m`, `mount_z_m`, `mount_roll_deg`, `mount_pitch_deg`, `mount_yaw_deg` (the
  `head_mount_joint` origin), `tilt_zero_deg` (the `head_tilt` origin pitch) and `tilt_roll_deg` (its roll).
- `CAMERA_MOUNT_PARAMS` with `fit_camera_mount: true` (the default): `camera_x_m`, `camera_z_m`, `camera_roll_deg`,
  `camera_yaw_deg`, the origin of the camera mount joint (`camera_mount_joint`: the joint after `head_tilt` on the path
  to `camera_frame`, `d435_joint` for the D435 colour), refined from its CAD start. Its y and pitch keep the URDF
  values: y runs along the tilt axis and is degenerate with the mount y (Jacobian condition number 8e10 on the
  synthetic scene), pitch commutes with the tilt and folds into `tilt_zero_deg` (correlation 0.9995). With the four
  free the synthetic scene's largest correlation is 0.905 (camera yaw / mount yaw), camera x / pan axis x 0.89. The
  real session (29 arm poses, camera mount free) gives camera z sigma 0.78 mm and passes every limit; the synthetic
  scene (24 poses) at 2.5 px per axis gives 2.74 mm, above `max_sigma_mm`. The camera parameters carry `start_delta`
  checks against `max_start_delta_mm` / `max_start_delta_deg`.
- `PAN_AXIS_PARAMS` with `fit_pan_axis: true` (the default): `pan_axis_x_m`, `pan_axis_y_m`, the `head_pan` origin x, y.
  The `head_tilt` origin is moved back by the same shift (`head_origins`), so the head stays where it is at pan 0 and
  only the axis it turns about moves. The real session `calib-20261023-074418` put that axis 38 mm behind the URDF
  value, which a fixed axis cannot absorb (about 13 px at the +-10 deg pan offsets).
- The board pose in `gripper_link` (6 values), solved from a start built by `initial_board_mount`: each view's
  `solvePnP` seeded from the configured `board_mount` and mapped through the start chain into `gripper_link`, median
  translation and mean rotation.

Fixed: every other head origin component (`head_pan` rpy and z, `head_tilt` z, the camera mount y and pitch, the D435
colour and optical joints), the arm URDF, the joint offsets, the arm gauge (`base_in_base_link`) and the intrinsics. The pan zero is folded
into `mount_yaw_deg` (the result's `PAN_ZERO_NOTE`): a pan scale or zero offset is degenerate with the mount yaw and
the axis shift, so it is not a parameter.

Start values are read from the head URDF text passed in. The node passes the head URDF with the applied overrides, so
after a floor fit has been applied the start is the floor fit's chain. `floor` is a
dict of the floor fit values per `floor_fit.PARAM_NAMES` entry (`mount_z_m`, `mount_roll_deg`, `mount_pitch_deg`,
`tilt_zero_deg`, `tilt_roll_deg`), or None.

### Floor cross-check and prior

The floor fit sees the ground plane and the arm fit sees the board; the two must agree on the five shared parameters.

- Cross-check: every shared parameter whose fitted value differs from the floor value by more than `max_floor_delta_mm`
  (10 mm) or `max_floor_delta_deg` (1.5 deg) fails `floor_delta:<name>`. Without floor values the fit fails
  `floor_cross_check` (it never silently skips).
- Prior: with `floor_prior_weight > 0` (default 10) and floor values, one extra residual per shared parameter,
  `weight * (value - floor) / unit` (1 mm or 0.1 deg), pulls the fit toward the floor values. Weight 0 turns it off.
  Without it the real session's arm views put the pan axis direction 2-3 deg off the floor fit.

### Held-out views, validation and limits

Every `holdout_every`-th distinct arm pose (sorted, default every 5th) is held out with all its views and is not in the
fit. `HeadArmFit` reports the fit-view corner RMS (`rms_px`), the held-out RMS (`holdout_rms_px`), the held-out board
error in mm (arm FK board centre against the view's `solvePnP` through the fitted chain, `holdout_board_mm`), one-sigma
per parameter from `(J^T J)^-1 * sum(r^2) / (m - n)` on the corner residuals, the largest parameter correlation, view
and arm pose counts, pan and tilt spans, the RMS per head offset and per view, and the new `origins`.
`limit_checks(fit, limits)` lists every check (name, value, limit, ok) in order and `failed_limits` names the failed
ones: `rms_px`, `holdout_rms_px`, `holdout_board_mm`, `max_sigma_mm`, `max_sigma_deg`, `max_correlation`, `min_views`,
`min_arm_poses`, `min_pan_span_deg`, `min_tilt_span_deg`, `floor_delta:*`, `start_delta:mount_x_m` /
`mount_y_m` / `mount_yaw_deg` (a start chain more than 30 mm / 5 deg off is refused) and `start_delta:pan_axis_*`
(`max_pan_axis_delta_mm`, 50). NaN fails. `passed` is true only with no failed limit.

Limits of the method: the fit trusts the arm to about 2 mm; it needs a pan span of at least 30 deg and a tilt span of
at least 10 deg (a result below them is refused); parameter pairs that are nearly degenerate
(mount y with mount yaw reach about 0.97) are caught by `max_correlation` 0.99; at least 3 views are required
(`ValueError` below that); the calibrated head camera intrinsics are taken as given. The limit values are evidence from
five leave-poses-out folds of the real session (`HeadArmFitLimits` docstring, `tests/test_head_arm_real_session.py`).
`head_arm_report_markdown(fit, cfg, session, date)` renders the report (parameter table, residuals per head offset,
coverage, the largest correlation, the limit table).

### `aim_head`

`aim_head(chain, point_base, camera_frame)` returns the (pan, tilt) in radians (+ tilt is down) that put a base_link
point on the optical axis of `camera_frame` (default `d435_color_optical_frame`), solved numerically on the given chain (the camera is off the pan and tilt
axes), clamped to the chain's `head_pan` and `head_tilt` limits (the tilt range of the head URDF is -30..+45 deg). The
node's sequence uses it with the applied head chain and the board centre from arm FK to aim the head at the board.

### Arm pose check (`arm_pose_check.py`)

`PoseChecker(arm_urdf, head_urdf, fit, cfg, board_margin_m=None)` decides whether an arm pose or a joint path is safe
with the board on the gripper. Poses are evaluated in base_link: arm URDF FK with `urdf = measured + offset`, placed by
`arm_in_robot_base`. `check_pose(joints, gripper_rad, rolling)` returns a list of violation strings, each starting with
the rule name; an empty list is safe. `check_path(start, target, gripper_rad)` samples the joint-linear path with no
joint step over `path_step_rad` and returns the violations of every sample, each ending "at path fraction <f>".
Distances are signed (negative = penetration). Rules (`PoseCheckConfig`, m and rad):

| Rule | Meaning |
|---|---|
| `joint_limits` | A joint (measured + offset, the gripper without offset) is outside the arm URDF limits. |
| `stretched_shoulder_lift_max_rad` | `shoulder_lift` above 1.85 rad with `elbow_flex` below 0 (stretched arm). |
| `min_tool_height_m` | The tool point (`gripper_frame_link`) is lower than 0.10 m. |
| `max_roll_gripper_rad`, `roll_min_tool_height_m` | While `wrist_roll` changes the gripper must be at most 0.8 rad open (half open) and the tool at least 0.15 m high. |
| `min_board_height_m` | A board point is lower than 0.05 m. |
| `board_clearance` | The board outline and grid points keep `board_margin_m` from every body obstacle and the head. |
| `link_clearance` | The upper arm, forearm and wrist capsules (`link_radius_m` 0.025) keep `margin_m` (0.03) from the body and the head; the upper arm is exempt from the crossbar it stands on. |
| `board_link_clearance` | The board keeps `board_margin_m` from the upper arm and forearm capsules. |

`board_margin_m` defaults to `margin_m`; the sequencer passes `margin_m + provisional_margin_m` while the board mount is
the hand-measured start value, because the mount uncertainty moves the board and not the arm links.

The body is `BodyConfig` (`BoxObstacle` and `CylinderObstacle` lists): a copy of the client.yml `mcp_server` `body`
block (frame rails, crossbar, four wheels, lidar), pinned equal by `tests/test_arm_pose_check.py`. It is a copy on
purpose: the shared package must not import mcp_server, and mcp_server keeps its own, stricter checks on top of this
one. The head is not in the body block: `head_envelope(head_urdf, cfg)` builds a vertical cylinder about the pan axis
(the `head_mount_joint` x, y of the given, applied head URDF) standing on the crossbar top, which contains every head
visual at every pan/tilt inside the URDF's limits (sampled at the limits and every 15 deg, grown by the largest arc sag
between samples). Pass the applied chain: a head that reaches further, or a wider tilt range, grows the envelope.

### Config models (`config.py`)

`HeadArmFitConfig` (`board`, `fit_pan_axis`, `fit_camera_mount`, `board_mount`, `limits`, `huber_f_scale_px`, `base_in_base_link`,
`joint_offsets_rad` (exactly the five arm joints), `floor_prior_weight`), `BoardMountConfig` (`xyz_init`, `rpy_init`,
`outline_margin_m`: the start of the board frame in `gripper_link` and the plate margin the pose check uses),
`HeadArmFitLimits` (every limit above, with `holdout_every` >= 2) and `BaseInBaseLink` (`x`, `y`, `z`, `yaw`, default
0.0592, -0.05, 0.100, 0). The defaults of the joint offsets and the base gauge equal the arm config in `client.yml`
(pinned by `tests/test_head_arm_calib_config.py`). `charuco.board_corners_3d` and `board_outline` give the board corner
geometry the fit and the pose check use. Unknown keys are rejected by every model.

Tests: `tests/test_head_arm_fit.py`, `tests/test_head_arm_real_session.py`, `tests/test_arm_pose_check.py`,
`tests/test_head_arm_calib_config.py`, `tests/test_head_arm_calib_chain.py`.

## ros2_common_cpp (C++)

An ament_cmake package with two plain library targets, `ros2_common_cpp::config` and `ros2_common_cpp::metrics`. None links rclcpp, so a node's pure-logic library can use them. Headers are in
[include/ros2_common_cpp/](ros2_common_cpp/include/ros2_common_cpp/); C++17; formatted with the root `.clang-format`
(ament style, 120 columns).

### config (`config.hpp`, yaml-cpp)

The C++ version of a pydantic model with `extra="forbid"`. A node writes one `Errors validate(const YAML::Node &)` that
returns every error (`Errors` = `std::vector<std::string>`, one `"<key>: <problem>"` per error):

| Helper | Behaviour |
|---|---|
| `require<T>(node, key, errors, mode)` | `std::optional<T>`. Missing key gives `"<key>: field required"`. A value that does not convert (including null) gives `"<key>: expected <type>"`. Types: `int`, `double`, `bool`, `std::string`, vectors of those, anything `YAML::convert` knows. |
| `get_or<T>(node, key, fallback, errors, mode)` | Optional field. Returns the fallback when the key is missing. A wrong type adds an error and returns the fallback. |
| `Coercion::LAX` (the optional `mode` of `convert`, `require`, `get_or`) | Reads a scalar as pydantic lax mode does over `yaml.safe_load`: float from bool, int, float, numeric string, sexagesimal, `.inf`/`.nan`; int from bool, int (bin, oct, hex), integral float, integer string; bool from `yes/no/on/off/true/false` words and 0/1; string only when PyYAML reads a string. Default `Coercion::STRICT` is yaml-cpp's own conversion. Helpers `lax_float`, `lax_int`, `lax_bool`, `lax_string`, `is_yaml_null` (PyYAML null words). map_guard_cpp uses LAX so it accepts exactly what its pydantic model accepts. |
| `forbid_unknown(node, allowed, errors, where = "")` | `"<where.>key: extra field not permitted"` for each key not in `allowed`. A non-mapping gives `"<where>: expected mapping"`. |
| `ge` / `gt` / `le(key, value, bound, errors)` | Range checks: `"<key>: must be >= <bound> (got <value>)"`. |
| `config_path_from_env(env_var, default_path)` | The env var when it is set and not empty, else the default (`<NAME>_CONFIG`, `/etc/ros2/<name>/config.yaml`). |
| `load_config(path, validate)` | `LoadResult{path, node, errors}`. Never exits. A missing or unreadable file, invalid YAML and a non-mapping root are errors too. An empty file counts as an empty mapping. |
| `load_config_or_exit(env_var, default_path, validate)` | Node entry point. Returns the validated root. On any error it logs every error to stderr (`config error: <path>: <error>`) and exits `EXIT_CONFIG` (78), which the unit's `RestartPreventExitStatus=78` honours. |

### metrics (`metrics.hpp`, prometheus-cpp)

`Metrics metrics("<node>")` registers `robot_node_info{node="<node>"} 1` and
`robot_node_start_time_seconds{node="<node>"}` (process start, epoch seconds). These are the same series as
`ros2_metrics.register_node_info`, so Grafana dashboards and alerts carry over.

- `counter(name, help, labels)`, `gauge(...)` and `histogram(name, help, buckets, labels)` return the prometheus-cpp
  child for those labels. The same name and labels return the same child.
- Every `Metrics` also exports the `process_*` series of prometheus_client's ProcessCollector
  (`process_cpu_seconds_total`, `process_virtual_memory_bytes`, `process_resident_memory_bytes`,
  `process_start_time_seconds`, `process_open_fds`, `process_max_fds`; unlabelled). `ProcessCollector`
  (`process_collector.hpp`) reads them from `/proc/self` on every collect, render and scrape, and returns no series
  where `/proc` cannot be read. So a C++ node and its Python counterpart have the same CPU and memory series.
- `collect()` returns every family (the registry's plus the `process_*` ones); `render()` returns them as the text
  exposition.
- `serve(port, host = "127.0.0.1")` serves `GET /metrics` in a background thread and returns the bound port. Port 0
  picks a free port.
- `resolve_metrics_port(config_port)` returns the config value, else `METRICS_PORT`, else `nullopt` (metrics off).
  A value outside 1..65535 throws `std::invalid_argument`.
- Keep each node's 191xx port from `tests/test_node_metrics_wiring.py`.

**Choice: prometheus-cpp, not a hand-written exporter over cpp-httplib.** Ubuntu 24.04 ships both as apt packages for
arm64: `prometheus-cpp-dev` 1.0.2 (core and pull, with civetweb) and `libcpp-httplib-dev` 0.14.3. This was checked
with `apt-cache policy` in the arm64 `ros:jazzy-ros-base` image. Note that the package is `prometheus-cpp-dev`, not
`libprometheus-cpp-dev`. We use prometheus-cpp because it is the established library: label escaping, histogram
buckets, the text format and the HTTP exposer are already implemented and tested upstream, so we write none of them
ourselves (CLAUDE.md, "Use the right libraries").

prometheus-cpp has no upstream rosdep rule. `package.xml` declares `<depend>prometheus-cpp</depend>`, and
`scripts/cpp-node-test.Dockerfile` maps that key to `prometheus-cpp-dev`. On the Pi, a C++ node type must list
`libyaml-cpp-dev` and `prometheus-cpp-dev` in its apt packages.

### Adding a C++ node

Layout: `nodes/<name>_cpp/`. The directory name must equal the `package.xml` `<name>`; the test script, the poe tasks
and CI address packages by it. The package contains `package.xml`, `CMakeLists.txt`, `include/<name>_cpp/`, `src/`,
`test/`, `config/` and `README.md`.

1. **Library target without rclcpp.** Put the pure logic (filters, state machines, scoring) and the config `validate`
   in it. It links `ros2_common_cpp::config` and, if it needs them, `ros2_common_cpp::metrics`. Its gtests need no ROS
   graph.
2. **Component plus standalone executable.** The rclcpp node class links the library. Register it with
   `rclcpp_components_register_node(<name>_component PLUGIN "<ns>::<Class>" EXECUTABLE <name>)`, so the node can be
   composed or run alone (`ros2 run <name>_cpp <name>`). Load the config in the constructor with
   `load_config_or_exit("<NAME>_CONFIG", "/etc/ros2/<name>/config.yaml", validate)`. Use the same env var and path as
   the Python node, so the `group_vars` config block stays the same.
3. **CMake.** In `CMakeLists.txt`:
   - `find_package(ros2_common_cpp REQUIRED)`, then link `ros2_common_cpp::config` / `ros2_common_cpp::metrics`.
   - Under `if(BUILD_TESTING)`: `set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")`,
     `find_package(ament_lint_auto REQUIRED)`, `ament_lint_auto_find_test_dependencies()`, and one `ament_add_gtest`
     per test file.
   - In `package.xml`, `test_depend` on `ament_cmake_gtest`, `ament_lint_auto`, `ament_cmake_clang_format`,
     `ament_cmake_cppcheck` and `ament_cmake_xmllint`. Do not add `ament_cmake_copyright`.
   - Copy [ros2_common_cpp/CMakeLists.txt](ros2_common_cpp/CMakeLists.txt) as the starting point.
4. **CI and tests.**
   - Add the package name to the `cpp` matrix in `.github/workflows/ci.yml`. `tests/test_ci_workflow.py` and
     `tests/test_node_lint_coverage.py` fail until you do.
   - The poe tasks `test-cpp` and `lint-cpp` find the package through their `nodes/*_cpp` glob.
   - Verify with `./scripts/cpp-node-test.sh <name>_cpp`.

### Tests (gtest, run by `./scripts/cpp-node-test.sh ros2_common_cpp`)

| File | Test | Covers |
|---|---|---|
| `test/test_config.cpp` | `ValidConfigHasNoErrors` | A valid config and a config that uses defaults give no errors. |
| | `UnknownKeyIsRejected`, `UnknownNestedKeyCarriesItsPath`, `NonMappingSectionIsRejected` | `forbid_unknown`, top level and nested (`camera.hight`). |
| | `WrongTypeIsRejected` | String for a float, float for an int, list for a string, null, bad bool. |
| | `OutOfRangeIsRejected`, `RangeBoundsAreInclusiveWhereStated` | `gt` / `ge` / `le` messages and bound inclusivity. |
| | `MissingRequiredIsRejected`, `EveryErrorIsReported` | `require` and collection of every error, not just the first. |
| | `PathFromEnvWithDefault` | Env var used when set and not empty, else the default path. |
| | `LoadConfigCollectsFileErrors`, `LoadConfigEmptyFileIsAnEmptyMapping` | Missing file, invalid YAML, non-mapping root, empty file. |
| | `LoadConfigOrExitReturnsTheValidRoot` | The success path returns the parsed root. |
| | `ConfigDeathTest.InvalidConfigLogsEveryErrorAndExits78`, `MissingFileFromDefaultPathExits78` | Exit code 78, every error logged (death tests). |
| `test/test_metrics.cpp` | `NodeInfoExactText`, `StartTimeIsNowInEpochSeconds` | The exact line `robot_node_info{node="x"} 1`, gauge types, start time close to now. |
| | `LabelValuesAreEscaped` | `"`, `\` and newline in label values are escaped. |
| | `CountersAndGaugesWithLabels` | Labelled children, the same child returned for the same labels. |
| | `HistogramBucketsAreCumulative` | `_bucket{le=...}` cumulative counts including `+Inf`, `_count` and `_sum`. |
| | `HttpGetOnLocalhostReturnsTheExposition` | `serve(0)` binds 127.0.0.1, a raw HTTP GET returns 200 with the series, and a second `serve` throws. |
| | `ResolvePortConfigThenEnvThenDisabled` | Config port, then `METRICS_PORT`, then disabled; invalid values throw. |

The ament linters (clang-format, cppcheck, xmllint) run as ctest tests of the same package.

## Tests

Run from the repo root with the root pytest (`uv run pytest tests -v`): `tests/test_shared_utils.py`,
`tests/test_shared_battery.py` (config and guard logic), `tests/test_shared_camera_geometry.py` and `tests/test_shared_package.py` (packaging and the node
path dependency). They import via `shared.ros2_common`. Documented in [tests/README.md](../tests/README.md).
