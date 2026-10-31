# head_rgbd_calibration

Robot-side capture, review and compute service for the head RealSense D435 calibration and the CSI camera intrinsics.
It runs five calibrations and
keeps their sessions, thumbnails, coverage and results on the robot, so a session survives restarts and deploys:

- **D435 intrinsics check**: ChArUco stills of the D435 colour stream with the depth aligned to it; the colour
  intrinsics fitted from them are compared with the factory camera_info, and the aligned depth is checked against the
  board plane. No calibration file is written: the D435 factory intrinsics stay authoritative (design D10).
- **Head floor fit**: a guided head sweep over a flat floor; floor planes in the D435 depth cloud fit the head mount,
  tilt zero and roll ("Head floor fit").
- **Head from arm**: the board on the gripper, the arm moves it through scripted poses and the head D435 colour views
  fit the head chain, the pan axis and the `d435_joint` mount ("Head from arm calibration").
- **Arm offsets**: the same head_arm sessions re-solve the arm joint offsets and the tool point ("Arm offsets compute
  and apply").
- **CSI camera intrinsics**: ChArUco views of the front and back Camera Module 3 fit K and plumb_bob D; apply writes
  the `camera_info_file` head_camera_cpp publishes `camera_info` from ("CSI camera intrinsics").

The calibration maths lives in `shared/ros2_common/head_rgbd_calibration`; this node owns the session state. After an
apply the head nodes in `restart_units` (`ros2-head_rgbd_cpp.service`, `ros2-head_state_publisher.service`) are
restarted by the Ansible apply path unit, never by this node.

## State directory

`state_dir` (default `/var/lib/ros2/head_rgbd_calibration`) is owned by the service user and never touched by deploys.
Only the stills the capture judged are stored.

```
sessions/<sid>/session.json                 id, kind (intrinsics | head_arm), created, rgb_size, next_pose
sessions/<sid>/poses/<pid>/rgb.jpg          colour JPEG bytes exactly as received
sessions/<sid>/poses/<pid>/depth.png        aligned depth, uint16 mm PNG (intrinsics sessions)
sessions/<sid>/poses/<pid>/pose.json        stamps, colour corner ids and points, factory camera_info, hints
sessions/<sid>/poses/<pid>/rgb_thumb.jpg    <= thumbnail_width_px wide, corners drawn
sessions/<sid>/poses/<pid>/depth_thumb.jpg  colourised aligned depth (turbo), corners drawn (intrinsics sessions)
sessions/<sid>/results/<rid>/               result files, result.json last
floor_fit/<sid>/                            floor sweep sessions ("Head floor fit")
applied/                                    applied.json and the applied head_chain.json
```

- Ids: session `calib-YYYYmmdd-HHMMSS` (`-N` when that second is taken), pose `001` (zero-padded from `next_pose`, never
  reused after a delete), result `r-YYYYmmdd-HHMMSS`. Every id is regex-checked and the resolved path must stay under
  `state_dir`: a bad id raises `InvalidId`, a missing item `NotFound` (both answer 404 in the HTTP layer).
- Writes are atomic: JSON files go through a temp file and `os.replace`; a pose directory is written under a
  `.tmp-<pid>` name and renamed into place. One lock serialises mutations.
- `session.json` records `rgb_size` from the first pose; a pose of another size is refused with `ValueError`.
  `SessionStore.add_pose` also refuses a depth that is not the colour size (not aligned), a camera_info for another
  image size and a depth with no valid pixel; nothing is stored then. `pose.json` records `plane_bias_samples`, the
  valid aligned depth pixels inside the detected board.
- A `session.json` without `kind` is a session of the retired RGB + ToF calibration (`rgbd`): it is listed by no route
  and nothing computes it.
- Coverage (`SessionStore.coverage`) counts the stored colour corners per cell of the `coverage` grid; it is never
  invented, and a session without poses gives zeros.

## Capture

`CaptureController(node, store, cfg, clock)` (`head_rgbd_calibration/capture.py`) subscribes the D435 topics only while
a capture runs (design D4) and publishes nothing.

- `start(session_id)` creates an intrinsics session (`None`) or reopens one to add poses (`NotFound` for an unknown id,
  `CaptureError` for a session of another kind). A second start while a capture or a still session runs raises `Busy`.
  The pose numbering continues from the stored pose count.
- Subscriptions:

  | Topic | Type | QoS |
  |---|---|---|
  | `topics.depth` (`/head/d435/aligned_depth_to_color/image_raw`) | `sensor_msgs/Image` 16UC1 mm | RELIABLE KEEP_LAST `qos.depth` (the driver default) |
  | `topics.rgb_info` (`/head/d435/color/camera_info`) | `sensor_msgs/CameraInfo` | RELIABLE depth 1 |
  | `topics.rgb` (`/head/d435/color/image_raw/compressed`) | `sensor_msgs/CompressedImage` | BEST_EFFORT KEEP_LAST `qos.rgb` |

- `stop()`, the `max_capture_s` timer and a start that fails part-way destroy every subscription and the timer: nothing
  outlives the capture.
- Pairing and stillness (`StillPairCounter`): the driver stamps the aligned depth with its colour frame's stamp, so a
  colour frame and a depth frame pair on the exact stamp; an unpaired frame more than 1 s older than the newest is
  dropped. A pair is still when every consecutive pair over `selection.still_window_s` differs by less than
  `selection.still_rgb_mad` (grey levels) and `selection.still_depth_mad` (mean depth change as a fraction of the
  median depth); pairs in the first `selection.skip_start_s` (auto exposure settling) are never captured. One still is
  stored per still window, once the board is detected on the colour and a factory colour camera_info has arrived.
- Frames: the depth decodes as `16UC1` honouring `step` and `is_bigendian`; the colour decodes with `cv2.imdecode` and
  its JPEG bytes are kept as `rgb.jpg`. A frame that does not decode is logged (throttled) and dropped. A pose the store
  refuses is logged and shown in the status.
- Hints: `pose.json` carries `hints`, operator texts with no re-detection: a still whose colour image shows fewer than
  half the board's inner corners gets `few corners: N RGB - move the board closer / fill the frame`. A good still has
  `hints: []`.
- `state()` returns `{running, session_id, status, pose_count, started_at, stops_at}` (plus `kind: "head_arm"` for a
  still session). `status` is the counter status (`waiting for frames` until a pair was judged, then `moving`,
  `still - no board in the colour image`, `still - no colour camera_info yet`, `pose N captured - move to the next
  pose`), `idle` before the first start, or why the capture stopped. `stops_at` is set only while running.
- Threads: the subscription and timer callbacks run on the executor thread; `start`, `stop` and `state` run on HTTP
  threads. One lock guards the shared state and is held only for field updates. Decoding, pairing, detection and pose
  writes run on the executor thread (serialised by a second lock no HTTP call takes). Callback errors are logged, never
  raised into the executor.

### Still capture (head_arm sessions)

`CaptureController.start_still_session(session_id)` opens a `head_arm` session and subscribes the D435 colour stream,
its camera_info `topics.rgb_info`, `topics.head_joint_states` and `topics.arm_joint_states` (refused while an
intrinsics capture runs). `still_camera_info()` is the latest colour camera_info of the running session (None before
the first). `capture_still(arm_pose, tier, head_offset)` blocks (sequencer thread) until the last `still.window_s` of
frames differ by at most `still.rgb_mad` and all seven joints moved less than `still.joint_still_rad`, detects
`fit.board` in the newest frame, and stores it with the head and arm samples nearest its stamp (within
`still.max_joint_dt_s`) and the latest factory colour camera_info. The outcome reason is `stored`, `no board`, `no
colour camera_info`, `joints stale`, `not still within <t> s` or `refused: <store message>`; a missing joint means no
pose, never a zero. `stop()` destroys every subscription. `fit.board` must equal `board`.

## Config keys

File named by `HEAD_RGBD_CALIBRATION_CONFIG`; every key has a default (`config/head_rgbd_calibration.yaml`). Unknown
keys are rejected.

| Key | Default | Meaning |
|---|---|---|
| `state_dir` | `/var/lib/ros2/head_rgbd_calibration` | Sessions, poses, results |
| `http_host` / `http_port` | `127.0.0.1` / `18110` | HTTP API bind |
| `topics.rgb` / `depth` / `rgb_info` | `/head/d435/color/image_raw/compressed`, `/head/d435/aligned_depth_to_color/image_raw`, `/head/d435/color/camera_info` | D435 capture topics (`nodes/bridges/realsense_d435/config/topics.yaml`) |
| `topics.head_joint_states` / `arm_joint_states` | `/head/joint_states`, `/follower/joint_states` | Measured joints of the head_arm still capture |
| `qos.depth` / `qos.rgb` | 10 / 5 | Subscription queue depths |
| `max_capture_s` | 900 | Longest single capture |
| `compute_timeout_s` / `compute_nice` | 600 / 10 | Compute child limits |
| `camera_frame` | `d435_color_optical_frame` | Optical frame of the D435 colour stream; the head-from-arm and arm fits solve the camera in it |
| `head_urdf_file`, `arm_urdf_file` | deployed checkout `head.urdf`, `so101_arm.urdf` | Start head chain (with the applied overrides) and the arm FK |
| `restart_units`, `restart_check_s` | `ros2-head_rgbd_cpp.service`, `ros2-head_state_publisher.service`; 60 | Units the apply path unit restarts after an apply, reported with their state |
| `thumbnail_width_px` | 320 | Thumbnail width (never enlarged) |
| `coverage.cols` / `coverage.rows` | 4 / 3 | Coverage grid |
| `board` | 6x5, 40 mm squares, 30 mm markers, `DICT_5X5_100`, 4 mm plate | ChArUco board; equals the charuco block of `scripts/markers_generate.yaml` |
| `selection` | window 0.5 s, colour MAD 2.0, depth MAD 0.01, skip 2.0 s | Still-pair selection of the intrinsics check |
| `limits` | plane bias 10 mm, plane spread 50 mm, 12 views, RMS 0.5 px, focal 1 %, principal point 5 px | Intrinsics check acceptance limits |
| `floor` | see "Head floor fit" | Floor sweep and fit |
| `still` | window 0.6 s, MAD 2.0, joints 0.002 rad, joint dt 0.1 s, timeout 6 s | head_arm still capture |
| `robot` | `http://127.0.0.1:18200/mcp`, `/etc/ros2/mcp_server/token` | mcp_server the sequencer drives the arm and head through |
| `fit.board_mount` | measured by the user 2026-10-23 | Start value of the board pose in `gripper_link`: the board on a stick in the closed gripper, plane across the tool axis 2 cm past the URDF tool point, print toward the arm, centre 0.12 m above the tool axis on the side that is up at the parked arm (FK of `scripts/gen_nav_footprint.yaml` `parked_arm_joints`), long edges level there. Hand-measured, not fit-solved (design D47); the fit solves the real mount |
| `fit.fit_pan_axis` | true | Head-from-arm fit: also fit the head pan axis position (`pan_axis_x_m`, `pan_axis_y_m`: the `head_pan` origin x, y in `head_pan_base`); the `head_tilt` origin moves back by the same shift, so the head is unchanged at pan 0 and only the axis it turns about moves. Session calib-20261023-074418 put the axis 37.9 mm behind and 11.2 mm right of the URDF value: the fixed axis left 13 px at the +-10 deg pan offsets (rms 12.2 px), the free axis fits every head offset at 2.6-2.8 px. The result.json `params` carry both entries (no floor value), `offset_rms_px` lists `{head_offset, views, rms_px}` per head offset (report.md: `## Residuals per head offset`), and a passing `head_chain.json` holds the `head_pan` origin for every consumer of `apply_overrides` (head_state_publisher, mcp_server HeadKinematics, web_ui) |
| `fit.fit_camera_mount` | true | Head-from-arm fit: also fit the D435 mount on the head (`camera_x_m`, `camera_z_m`, `camera_roll_deg`, `camera_yaw_deg`: the `d435_joint` origin x, z, roll, yaw, refined from the CAD start); its y and pitch are not observable (degenerate with the mount y and the tilt zero) and keep the head URDF values. A passing `head_chain.json` holds the fitted `d435_joint` origin |
| `fit.limits` | rms 3.0 px, held-out 4.0 px / 4.5 mm, sigma 2.0 mm / 0.2 deg, correlation 0.99, pan axis shift 50 mm, floor 10 mm / 1.5 deg | Head-from-arm acceptance limits. The residual limits are evidence from calib-20261023-074418 over five leave-poses-out folds (`tests/test_head_arm_real_session.py`): the arm is about 2 mm accurate (about 2.5 px at the board), the folds gave rms 2.51-2.72 px, held-out 2.56-3.59 px and 2.89-3.77 mm, sigma at most 0.72 mm / 0.153 deg and correlation 0.973-0.976; the fixed pan axis (12.2 px) and a pan scale factor instead of the axis (3.07-3.33 px) stay above `rms_px`. `max_correlation` refuses degenerate parameter sets (0.994 and more) |
| `fit.floor_prior_weight` | 10 | Weight of the floor-fit prior (one residual per floor parameter, 1 mm / 0.1 deg units). User decision 2026-10-23, replacing D49's prior off: without it calib-20261023-074418 failed the floor cross-check on mount roll, mount pitch and tilt zero (2.1-2.8 deg); with it every limit passes at rms 2.93 px, held-out 2.88 px / 2.86 mm |
| `fit.plate`, `fit.plate_mount` | 6x5 board, null | Arm offsets fit (shared `arm_offsets_fit`, design D9, Q2): the gripper-held plate and its mount in `gripper_link`: `xyz_init` / `rpy_init` start values, `rpy` the CAD rotation (null: the rotation is fitted and wrist_roll stays at `fit.wrist_roll_offset_rad`; set: wrist_roll is fitted), `tool_in_board_m` the CAD board-to-jaw-tip offset (a required key, null until measured, then the tool point is not fitted). The arm offsets fit refuses to run without `plate_mount` |
| `fit.wrist_roll_offset_rad`, `fit.tool_offset_m` | null, (0.0010, -0.0056, -0.0014) | Arm offsets fit: the wrist_roll zero from the recorded servo neutral (rad; required when `plate_mount.rpy` is null) and the current tool point in `gripper_frame_link` (client.yml mcp_server `arm.tool_offset_m`), reported unchanged when the tool point is not fitted |
| `fit.floor_fit`, `fit.arm_limits` | floor fit defaults; rms 3.0 px, held-out 4.0 px, tool 5.0 mm, 40 views, every 5th held out over 3 head settings | Arm offsets fit: the floor views' plane fit, limits and residual scales, and the acceptance limits; held-out plate views decide (reprojection px and the FK tool point against the view's solvePnP plate pose), the tool point is fitted only below `max_tool_holdout_mm` |
| `arm_sequence.base_pose`, `gripper_rad` | board ~0.34 m in front of the head camera, 0.4 rad | Base pose (measured joint angles, the `move_arm_joints` space: `urdf = measured + fit.joint_offsets_rad`) and the gripper sent once with it (at most `pose_check.max_roll_gripper_rad`) |
| `arm_sequence.tier0_poses` / `tier1_poses` / `tier2_poses` | 10 / 9 / 14 poses | Tier 0 moves only wrist_roll (within `max_roll_delta_rad` 1.2), tier 1 also wrist_flex (`max_flex_delta_rad` 0.4), tier 2 also elbow_flex, shoulder_lift, shoulder_pan (`max_big_delta_rad` 0.35). Load errors name the tier, pose and joint |
| `arm_sequence.speed_scale`, `settle_s`, `confirm_timeout_s` | 0.1 (<= 0.3), 1.0, 600 | Arm speed (slow for the fragile board mount; `move_arm_joints` has no acceleration option, its quintic peak acceleration scales with the speed squared), wait after a move, unanswered confirm timeout |
| `arm_sequence.head_offsets_deg` | `[[0, 0], [10, 0], [-10, 0], [0, -3]]` | (pan, tilt) offsets added to the aimed head angles, one view each. The tilt offset is -3 deg (head up): the board fills 250-340 px of the 360 px D435 colour image height, and the former +6 deg pushed it out on 26 of 33 poses |
| `arm_sequence.pose_check`, `provisional_margin_m` | core defaults, 0.05 | FK pose check settings; the board keeps `margin_m + provisional_margin_m` while the mount is the shipped hand-measured start; every view keeps each board corner `d435_min_z_m + d435_min_z_margin_m` (0.15 + 0.05 m) deep from `camera_frame` |
| `csi_cameras` | `front`: `/front_camera/image_raw/compressed`, `ros2-front_camera.service`; `back`: `/back_camera/image_raw/compressed`, `ros2-back_camera.service` | CSI cameras whose intrinsics are calibrated: name (lower-case letters and digits, it names `applied/<name>_camera_calibration.yaml`) -> `image_topic` (compressed) and `unit` (reads the applied file, reported after an apply) |
| `csi_min_view_shift_px` | 20 | A detected CSI view is kept only when its shared corners moved this far (mean, px) from the last kept view |
| `arm_sequence.park_pose`, `park_board_sag_margin_m`, `board_record_max_age_s` | the parked arm (`scripts/gen_nav_footprint.yaml` `parked_arm_joints`), 0.01, 7200 | Park with the board on (design D63): the arm moves to `park_pose` only after the path from the measured joints passes the pose check with the board outline and `park_board_sag_margin_m` added to the board floor, body and head clearances (the live run showed the board about 4 mm below FK under its 214 g) |

## Compute

`head_rgbd_calibration/compute.py` runs the D435 intrinsics check of one session's stored stills
(`SessionStore.load_stills`: colour, aligned depth and the recorded factory camera_info). `ComputeJob` in `jobs.py`
runs it as a child `nice -n <compute_nice> python -m head_rgbd_calibration.compute --config
<HEAD_RGBD_CALIBRATION_CONFIG> --session <sid> --result <rid>`, one at a time (`Busy` otherwise), so it never runs in
the web_ui loop or the capture process. The child is killed after `compute_timeout_s` (`error: timed out after N s`); a
non-zero exit without `result.json` reports the last 20 stderr lines as `error`. `state()` returns
`{running, session_id, result_id, started_at, error}`.

- Intrinsics: the board is detected on every colour still and the colour intrinsics are fitted from the detections
  (`check_intrinsics`, at least `limits.min_views` views); the fit RMS, the larger of the fx/fy differences from the
  factory camera_info (%) and the principal point distance (px) are judged against the `limits.intrinsics_max_*`
  limits.
- Board-plane depth bias: per still the board plane is solved by PnP with the factory K and D, and the aligned depth
  inside the board outline is compared with it; the median of the per-still medians must be within
  `limits.plane_max_bias_mm` and the median spread within `limits.plane_max_spread_mm`. A bias no still could measure
  fails `limits.plane_max_bias_mm`.
- Every still must carry the same factory camera_info (a still without one, or a changed one, is an input error: start
  a new session).

Output goes to `sessions/<sid>/results/<rid>/`, written under `.tmp-<rid>` and renamed when complete: `report.md` (the
check report) and `result.json` `{id, session, created, kind: "intrinsics", passed, failed_limits, rms_px,
focal_diff_pct, principal_diff_px, plane_bias_median_mm, plane_bias_spread_mm, views, pose_ids, poses, limits}`, where
`poses` lists `{id, error_px, plane_bias_mm, plane_spread_mm}` per still (null where the still has no detection or no
bias). Each failed limit in `failed_limits` names its `limits.*` key. An input error (not an intrinsics session, too few
views, no or differing factory camera_info) writes only `result.json` with `passed: false`, `error` and `poses: []`.

## Accept and download

`head_rgbd_calibration/apply.py` (design D6, D10). `apply_result(store, sid, rid, now)` refuses unless the result is an
intrinsics check (`kind: "intrinsics"`) with `passed: true` (`NotApplicable`). Accepting copies no file: it records
`{kind: "intrinsics", session, result, applied_at}` in `<state_dir>/applied/applied.json` (through a temp file and
`os.replace`), keeping the `head_chain` record the floor, head-from-arm and arm applies keep there. A head chain apply
needs this record (the Calibrate page's Accept check). The node never runs `systemctl restart`.

`applied_state(store, units, run)` returns `applied.json` plus per unit `{name, active_state, restarted}` from the
unprivileged `systemctl show --timestamp=unix -p ActiveEnterTimestamp -p ActiveState <unit>`; `restarted` is true when
the unit became active at or after `applied_at`. It is None when nothing is applied. The `csi_intrinsics` records of
the CSI cameras do not count as applied: with only them in `applied.json`, `GET /api/state` has `applied: null`,
`GET /api/applied` is 404 and a head chain apply is still refused (`has_check_record`).

`download_zip(store, sid, rid)` returns `<sid>-<rid>.zip` holding `report.md`, `result.json` and, for an arm result,
`joint_offsets_snippet.yaml`, so the result can be reviewed and committed.

## HTTP API

Bound to `http_host:http_port` (127.0.0.1 by default). Every error body is `{"ok": false, "message": ...}`; unknown or
malformed ids give 404, busy or not-applicable give 409.

| Route | Purpose |
|---|---|
| `GET /api/state` | capture status, compute status and `applied.json` (null when nothing applied; no systemctl call) |
| `POST /api/capture/start` | start an intrinsics capture (body optional `session_id`); answers the `GET /api/state` body; 409 while compute runs |
| `POST /api/capture/stop` | stop capture; answers the `GET /api/state` body |
| `GET /api/sessions` | list the intrinsics sessions `[{id, created, pose_count, result_count}]` |
| `GET /api/sessions/{sid}` | session detail: poses (`id`, `rgb_corners`, `plane_bias_samples`, `hints`), `coverage` and results; 400 for a session of another kind |
| `DELETE /api/sessions/{sid}/poses/{pid}` | delete one pose |
| `GET /api/sessions/{sid}/poses/{pid}/{rgb\|depth}.jpg` | thumbnail, `image/jpeg` |
| `POST /api/sessions/{sid}/compute` | start the check (202 `{result_id}`); 409 while capture runs or fewer than `limits.min_views` poses |
| `GET /api/sessions/{sid}/results/{rid}` | result with report |
| `GET /api/sessions/{sid}/results/{rid}/download` | `application/zip`, attachment `<sid>-<rid>.zip` |
| `POST /api/sessions/{sid}/results/{rid}/apply` | accept a passed check; answers the applied record plus `units`, the same body as `GET /api/applied`; 409 if it did not pass or compute runs |
| `GET /api/applied` | applied record plus per-unit restart state; 404 when nothing applied |

### Response contract

One body shape per route, pinned by the literal JSON examples in `tests/fixtures/api/` (`state.json`,
`capture_start.json`, `capture_stop.json`, `apply.json`, `applied.json`, `result.json`, `session.json`); `tests/contract.py` checks
that real responses have exactly the fixture's keys and value types.

- State (`GET /api/state`, `POST /api/capture/start`, `POST /api/capture/stop`): `{capture, compute, applied}`.
  `capture` is `{running, session_id, status, pose_count, started_at, stops_at}`, `compute` is
  `{running, session_id, result_id, started_at, error}`, `applied` is the applied record or null.
- Applied record (`applied.json`, embedded in every response): `{kind, session, result, applied_at}` plus the
  `head_chain` record once a head chain is applied. The keys are `session` and `result` everywhere, not
  `session_id`/`result_id`.
- Apply and `GET /api/applied`: the applied record plus `units`, a list of `{name, active_state, restarted}`.
- Session (`GET /api/sessions/{sid}`): every pose in `poses` has `hints`, a list of strings (empty for a good pose).
- Result: the `result.json` fields (including `poses`) plus `report` (markdown or null).

## Head floor fit

`head_rgbd_calibration/floor_sweep.py` (`FloorSweep`, design D5, D11) is the capture half of the floor-plane head chain fit
(mount z, roll, pitch and the head_tilt tilt zero and roll from the D435 depth floor plane in `floor.sensor_frame`; the maths is
`ros2_common.head_rgbd_calibration.floor_fit`). Clear a flat floor in front of the robot (a half-disc of about 1.8 m;
robot level, not on blocks), then start the sweep. Only the head moves.

- `start()` creates the `floor.joint_states_topic` and `floor.cloud_topic` subscriptions and the `floor.command_topic`
  publisher, and refuses (`SweepError`, nothing moved) when no head joint state arrives within `move_timeout_s`. It raises
  `Busy` while a sweep runs or a D435 intrinsics capture or compute runs (the HTTP layer passes that check as `blocked`).
  One sweep at a time, on a worker thread; a stop is `stop()`.
- Grid: `floor.tilt_deg` rows, the `floor.pan_deg` direction alternating per row (serpentine); defaults pan
  -30..45 by tilt 20/24/28 deg down, 18 poses, about a minute (pan -60 sees the parked arm; tilt 12 sees the floor
  to 1.8 m, where a room floor is often not one plane). Each pose is a `sensor_msgs/JointState` (`head_pan`,
  `head_tilt`, radians, `header.frame_id` = `command_frame_id`), the same command as the web UI head tab. A pose counts
  once both joints are within `reach_tol_deg` of the target and still (spread under `still_tol_deg` over
  `still_window_s`); not within `move_timeout_s` it is skipped with `did not reach pan/tilt`.
- After `settle_s` the next `clouds_per_pose` clouds stamped after the still time are fitted. A cloud whose `header.frame_id` is not `floor.sensor_frame` is dropped and counted (`cloud not in <frame>`). A cloud takes the head
  pan/tilt of the joint state nearest its header stamp within `max_stamp_dt_s` (no such state: the cloud is dropped and
  counted); points are read with `sensor_msgs_py.point_cloud2.read_points_numpy(skip_nans=True)` and fitted with
  `floor_fit.fit_plane` against the plane the URDF predicts. A pose without a valid plane is skipped with its reason; a
  pose with fewer than `min_clouds_per_pose` fitted clouds (an object in view of some clouds) is skipped with
  `partly blocked: k of n clouds` and none of its clouds are kept; nothing is filled in. Nothing is published but head
  commands.
- Every exit (done, `stop()`, error, exception) commands the head back to the pan/tilt it had at start and destroys the
  subscriptions and the publisher. Joint states that stop arriving for `move_timeout_s` abort the sweep with an error.
- `state()` returns `{running, session_id, pose, total, observations, skipped, status, error}`; `status` is `idle`,
  `starting`, `running`, `done`, `stopped` or `error`.

`floor` keys (all in the shipped YAML at their defaults): `joint_states_topic`, `command_topic`, `cloud_topic`,
`command_frame_id`, `pan_deg` (each -90..90), `sensor_frame`, `tilt_deg` (each -30..45, + is down, the head tilt range), `reach_tol_deg`, `still_tol_deg`,
`still_window_s`, `move_timeout_s`, `settle_s`, `clouds_per_pose` (>= 1, default 10), `min_clouds_per_pose`
(1..`clouds_per_pose`; absent or null: half of `clouds_per_pose`, rounded up, so 5 for 10 and 3 for 5),
`cloud_timeout_s`, `max_stamp_dt_s` and `fit` (the shared `FloorFitConfig`: plane, limits, residual scales).

Compute: `height_rms_mm` and `normal_rms_deg` are the RMS over the poses of the pose-mean residuals (the mean of a
pose's observations against the chain) and the `max_height_rms_mm` / `max_normal_rms_deg` limits apply to them; the
same pose gives plane normals about 1 deg apart cloud to cloud (depth noise no chain explains), so a
per-observation limit could never pass. `observation_height_rms_mm` and `observation_normal_rms_deg` report the
per-observation RMS; each pose has its signed pose-mean `height_residual_mm`, its pose-mean `normal_residual_deg` and
the per-observation `observation_height_rms_mm` / `observation_normal_rms_deg`. `params` holds `mount_z_m`,
`mount_roll_deg`, `mount_pitch_deg`, `tilt_zero_deg` and `tilt_roll_deg` (each value, sigma, urdf); the fitted
`head_tilt` roll and pitch go into head_chain.json with the mount origin, d435_joint stays as applied.

Files:

```
floor_fit/<sid>/observations.json   session, created, sensor_frame, grid, observations, skipped, dropped (cloud drop counts by reason)
floor_fit/<sid>/results/<rid>/      result.json of a compute; head_chain.json and head.urdf when it passed
applied/head_chain.json             the applied head chain override; applied.json gains head_chain {session, result, applied_at}
```

Session ids are `floor-YYYYMMDD-HHMMSS` (`-N` when that second is taken). `observations.json` is written atomically when the
sweep ends, also after a stop or an error (the poses done so far); each observation is `{pose, pan_rad, tilt_rad, normal,
height_m, inliers, candidates, rms_m, stamp_ns}` (plane in `floor.sensor_frame` = `d435_depth_optical_frame`, the frame `fit_head_chain_floor` predicts the floor in) and each skip `{pose, pan_deg, tilt_deg, reason}`.

Limits: the head must see the floor (tilt 20..28 deg down by default, flat empty floor 0.4..1.8 m out); `/head/joint_states` and the
`/head/d435/points` header stamps must share a clock (both are stamped by the robot); the sweep never drives anything but the head.

HTTP routes (proxied by the web UI under `/api/calibration/rgbd/`):

| Route | Body |
|---|---|
| `GET /api/floor/state` | `{sweep: {running, session_id, pose, total, observations, skipped, status, error}, applied: {session, result, note, joints} or null}` |
| `POST /api/floor/sweep/start`, `POST /api/floor/sweep/stop` | the state body; start 409 while a capture, compute or sweep runs |
| `GET /api/floor/sessions` | `[{id, created, observations, skipped, results}]` |
| `POST /api/floor/sessions/{sid}/compute` | result.json `{id, session, created, passed, failed_limits, params, height_rms_mm, normal_rms_deg, observation_height_rms_mm, observation_normal_rms_deg, poses, skipped}` |
| `GET /api/floor/sessions/{sid}/results/{rid}` | result.json |
| `POST /api/floor/sessions/{sid}/results/{rid}/apply` | `{head_chain: {session, result, applied_at}, units: [{name, active_state, restarted}]}` |

The routes are served by the node HTTP API alongside the capture routes.

## CSI camera intrinsics

`head_rgbd_calibration/csi_intrinsics.py` calibrates the intrinsics of each camera in `csi_cameras` (the front and back
Camera Module 3, head_camera_cpp at 1152x648). Until a camera is applied, head_camera_cpp publishes its images without
`camera_info` (no placeholder intrinsics) and marker_detector skips it.

- Capture: one at a time (any camera; 409 while one runs). It subscribes `csi_cameras.<camera>.image_topic`
  (BEST_EFFORT, `qos.rgb`) only while it runs and stops after `max_capture_s`. A frame becomes a view only when the
  ChArUco `board` is detected and the view differs from the last kept one (`csi_min_view_shift_px` mean shift of the
  shared corners, or fewer than 6 shared corners); a frame of another size than the session's is not kept. The status
  line says why a frame was not kept.
- Layout: `<state_dir>/csi/<camera>/<sid>/session.json` (`{id, camera, created, image_size}`, sid
  `csi-YYYYmmdd-HHMMSS[-N]`), `views/<NNN>/{image.jpg, view.json}` (the received JPEG; `{stamp_ns, ids, points}`) and
  `results/<rid>/result.json`.
- Compute (synchronous, refused while the session captures): `intrinsics.calibrate` of the shared core over the stored
  detections; fewer than `limits.min_views` views is 409 naming the count. `result.json` is `{kind: "csi_intrinsics",
  camera, session, result, created, image_size, k (3x3), d (5), rms, per_view_errors, view_count}`.
- Apply: writes `applied/<camera>_camera_calibration.yaml` atomically (ROS camera_calibration format: `image_width`,
  `image_height`, `camera_name` `<camera>_camera`, `camera_matrix`, `distortion_model` plumb_bob,
  `distortion_coefficients`, identity `rectification_matrix`, `projection_matrix` K | 0), then the
  `csi_intrinsics.<camera>` record `{kind: "csi_intrinsics", camera, session, result, applied_at}` in `applied.json`,
  keeping every other record. The apply path unit's script (`FILE:UNIT[,UNIT]` arguments) try-restarts only the units
  of a changed file, so a front apply restarts `ros2-front_camera.service` only. The answer is
  `{applied: record, units: [{name, active_state, restarted}]}` with the camera's `unit`.

| Route | Purpose |
|---|---|
| `GET /api/csi/{camera}/state` | `{camera, capture, session, applied}`: `capture` `{running, camera, session_id, status, view_count, started_at, stops_at}` (latest capture of any camera); `session` (the capture's when it is this camera's, else the newest, else null) `{id, created, view_count, image_size, coverage: {cols, rows, counts}, results}`; `applied` the camera's record or null |
| `POST /api/csi/{camera}/start` | start a capture (body optional `session_id` to add views); answers the state |
| `POST /api/csi/{camera}/stop` | stop the capture when it is this camera's; answers the state |
| `POST /api/csi/{camera}/sessions/{sid}/compute` | compute; answers `result.json` |
| `GET /api/csi/{camera}/sessions/{sid}/results` | the session's results, newest first |
| `GET /api/csi/{camera}/sessions/{sid}/results/{rid}` | one `result.json` |
| `POST /api/csi/{camera}/sessions/{sid}/results/{rid}/apply` | apply; answers `{applied, units}` |

An unknown camera, session or result is 404. web_ui reaches the routes through its `/api/calibration/rgbd/{path}` proxy.

## Process

`python -m head_rgbd_calibration` loads the config from `HEAD_RGBD_CALIBRATION_CONFIG`, runs one rclpy node
`head_rgbd_calibration` on a `MultiThreadedExecutor` in a daemon thread and serves the API with uvicorn. On shutdown it
stops capture (the D435 and the CSI one), kills a running compute child and destroys the node.

## Tests

`cd nodes/head_rgbd_calibration && uv run pytest tests/ -v`

- `tests/test_compute_arm.py` - the arm offsets compute and apply on a synthetic floor session and a head_arm session
  of plate views: result files and snippet, no floor session is an error, the named floor session wins, failed limits
  write only report and result, the tool point kept without `tool_in_board_m`, a missing plate mount or wrist_roll
  neutral is an error, apply writes the chain with `source: "arm"` and never the snippet.
- `tests/test_config.py` - config defaults equal the shipped YAML, unknown keys rejected, the `floor` defaults and the
  pan/tilt range checks, the `arm_sequence` defaults and its validation errors naming the tier, pose and joint.
- `tests/test_sequencer.py` - the sequence over a fake mcp_server: views inside the D435 min-Z or outside the colour FOV skipped with the reason and a pose with no usable view not visited (`test_poses_inside_min_z_are_skipped`), the wait for the colour camera_info; including Park with the board on (checked even when no load is reported, then the internal admin park; the board record offered only fresh and for a stored session, the stated-on Park, a stale record plus a non-board load refused): path-checked then moved at speed 0.1 and released with `park_calibration_board`, refused with the named rule when the sag margin eats the clearance, the fitted mount hook, another carried object and a load without this sequence's board still refused, a failed move keeps holding.
- `tests/test_arm_pose_set.py` - the shipped arm pose set on the shipped and the applied (floor fit 2026-10-22) head
  URDF: tier rules, short transits, every pose and transit (home -> base -> tiers -> base) safe by `PoseChecker` with
  the board margin `margin_m + provisional_margin_m` (`test_every_pose_and_transit_is_safe`) and with the mount
  shifted +-1 cm / turned
  +-5 deg (`test_safe_under_board_mount_uncertainty`), both also with a -30..+45 deg tilt URDF (`shipped-tilt-45`, `applied-tilt-45`), the base pose in front
  of the head, at least 12 board corners in view for 3 of 4 head offsets (`test_board_visible_from_aimed_head`), the
  aimed pan/tilt span and pose count (`test_coverage_limits`), no view inside the D435 min-Z and the kept views at 1.5
  times the coverage limits (`test_shipped_views_keep_the_board_in_the_d435_range`), at least 3 usable D435 views per
  pose (`test_every_pose_keeps_the_minimum_d435_views`; tier 1 pose 8 was wrist_flex -0.9 / wrist_roll -0.6 and lost
  all four, now -0.85 / -0.3) and each head offset usable on 3/4 of the poses
  (`test_every_head_offset_is_usable_on_most_poses`), a passing synthetic `fit_head_from_arm`
  (`test_synthetic_fit_on_shipped_set_passes`), and the measured-space phrase of `move_arm_joints`
  (`test_target_joint_convention`).
- `tests/test_floor_sweep.py` - the floor sweep against a fake node, fake clock and sleep and synthetic floor clouds:
  command order, frame_id and serpentine grid, session id and `observations.json`, clouds only after still plus settle,
  unreachable pose skipped with reason, a pose under `min_clouds_per_pose` skipped as partly blocked
  (`test_partly_blocked_pose_is_skipped`), clouds without a joint state dropped and counted, restore of the start pan/tilt
  and destroyed handles on done, stop, error and exception, `Busy`, refusal without a head joint state or while blocked.
- `tests/test_store.py` - ids and same-second suffix, layout of a stored D435 still (colour, aligned depth, thumbnails,
  camera_info), raw JPEG kept, pose ids never reused, refusals (another image size, unaligned depth, camera_info of
  another size, no valid depth), `load_stills`, a session without `kind` read as the ToF-era `rgbd`, head_arm still
  poses and their refusals, `load_arm_board_views`, `load_arm_camera_infos`, coverage counts, results listing, path
  safety (traversal ids, symlink escape), concurrent adds.
- Hints: `tests/test_store.py` (`test_add_pose_stores_hints`, `test_add_pose_of_a_good_pose_has_empty_hints`),
  `tests/test_app.py` (`test_session_poses_carry_hints`, `test_session_matches_contract_fixture` against
  `fixtures/api/session.json`), and `tests/test_compute.py` (`test_error_result_has_empty_pose_lists`).
- `tests/test_robot_client.py` - the tool helpers' names and arguments, structured data, the move timeout, tool errors, unreachable (also inside an exception group), a missing token file, the token never in any text, `release_arm(keep_position)`, and the board-park admin route (own transport and token file, bearer header, missing token file).
- `tests/test_head_arm_api.py` - the `/api/head-arm/*` routes over a fake sequencer: state and sessions bodies against the contract fixtures, the confirms (base, park, big joints), 409 for busy and wrong-phase requests, sequencer errors mapped to 400, only head_arm sessions listed, compute and apply, a failed result not applicable, an rgbd session 400 on the result routes, unknown ids 404.
- `tests/test_applied_chain_for_mcp.py` - the page's output consumed by mcp_server (2026-10-29): a synthetic session computed and applied, then the repo head URDF with `apply_overrides` on `applied/` through `mcp_server/head_kinematics.py` places the D435 colour optical frame within 5 mm of the truth chain at 5 head poses (`test_applied_files_place_the_camera_for_the_mcp_kinematics_within_the_target`), `head_chain.json` is in place before `applied.json` changes (mcp_server reloads on that mtime), an apply without an accepted check names the Accept check button, `state_dir/applied` is the directory mcp_server (`head.applied_dir`) and web_ui read, and the web_ui `rgbd_timeout_s` is not below `compute_timeout_s`. `test_head_arm_results.py` also pins the `residual` block of `head_chain.json`.
- `tests/test_head_arm_results.py` - compute writes `result.json`, `report.md` and (passed only) `head_chain.json`, fitted joints merged over the applied chain, the pan axis, the d435_joint fitted from the stills' camera_info in `camera_frame` with no `tof_joint.json` (`test_result_fits_the_d435_joint`), a still without camera_info, stills with different camera_info and a session without stills as error results, a failed limit writes no chain, floor values from the applied record (also the legacy record, and none failing the cross-check), apply writes the applied chain and the record with `source` and `floor`, `head_chain.json` lands in `applied/` before `applied.json` (`test_head_arm_apply_writes_the_head_chain_before_applied_json`, `tests/applied_writes.py` records the order), a failed result refused, `latest_board_mount`, `head_arm_session_exists`.
- `tests/test_floor_api.py`, `tests/test_floor_results.py` - the floor fit routes and results, including that a floor compute keeps the applied `head_pan` origin that apply writes `source: floor`, and that the floor apply lands `head_chain.json` before `applied.json` (`test_floor_apply_writes_the_head_chain_before_applied_json`: the apply path unit fires on `applied.json` and compares `head_chain.json`).
- `tests/test_runtime_deps.py` - `httpx`, imported at runtime by the robot client, is a runtime dependency in `pyproject.toml`.
- `tests/contract.py`, `tests/fixtures/api/*.json` - structural match of a real API body against a literal fixture (helpers and data, not tests).
- `tests/test_images.py` - RGB thumbnail width, corners drawn, input untouched, never enlarged; the colourised depth
  thumbnail, input untouched, `None` without valid depth.
- `tests/test_capture.py` - the D435 subscriptions and QoS (depth and camera_info RELIABLE, colour BEST_EFFORT), nothing
  subscribed before start, stop / timeout / failed start leave nothing subscribed, `Busy`, unknown session, a still
  stored with colour, aligned depth and camera_info, pairs only on the exact stamp, moving colour or depth not still,
  nothing stored before the camera_info or without a board, raw JPEG kept, reopened session numbering, undecodable
  frames dropped, frames after stop ignored, a refused still shown in the status, `decode_mono16` step and endianness,
  `state` polled while frames arrive; the head_arm still session (four topics, nearest joints, stale joints, no
  camera_info, stop and failed start) and the two capture kinds excluding each other.
- `tests/conftest.py` - stub `rclpy`, `rclpy.qos`, `rclpy.executors`, `sensor_msgs`, `sensor_msgs.msg` (incl. `JointState`, `PointCloud2`) and `sensor_msgs_py.point_cloud2` modules for the dev environment.
- `tests/pose_factory.py` - helper building poses from the core synthetic board renderer (not a test).
- `tests/test_apply.py` - accepting a passed check records it in applied.json and copies no file, the marker written atomically, a result of another kind refused, restart flags from a fake systemctl, zip members.
- Head pan axis: `tests/test_head_arm_results.py` (`test_pan_axis_written_to_head_chain_and_applied`: a synthetic session with the pan axis 30 mm back and 10 mm right gives both pan axis params, `offset_rms_px`, a head_chain.json `head_pan` origin with head_tilt moved back, and after apply the URDF from `apply_overrides` carries it), `tests/test_floor_results.py` (`test_compute_keeps_applied_pan_axis`: a floor compute keeps the applied `head_pan` origin), `tests/test_store.py` (`test_load_arm_board_views` reads `head_offset`).
- `tests/test_compute.py` - synthetic D435 session through the store: a passing check and its files, a focal offset and
  a plane bias over the limits fail, an unmeasured plane bias fails, a missing or changing factory camera_info, too few
  poses and a head_arm session are error results, `main`.
- `tests/test_jobs.py` - command and niceness, busy, timeout, stderr tail, unknown session (fake child commands).
- `tests/test_app.py` - every route with a fake capture, a fake job and a tmp store: error body shape, 404/409/400/422
  mapping, capture/compute mutual refusal, min_views refusal, coverage, thumbnails `image/jpeg`, zip headers, apply,
  `/api/state` without systemctl, `/api/applied` units.
- `tests/test_main.py` - bind host and port from the config (loopback by default), shutdown stops capture, the CSI
  capture and job and destroys the node, the CSI router is built with the CSI controller
  (`test_csi_router_is_built_and_its_capture_stopped_at_shutdown`), missing config env gives exit 78.
- `tests/test_csi_intrinsics.py` - the CSI intrinsics over synthetic 1152x648 ChArUco views fed to a fake node: the
  `csi_cameras` defaults and their validation, `view_differs`, the image subscription only while capturing and `Busy`,
  a view kept only with the board detected and different from the last kept one (`NO_BOARD`, `SAME_VIEW`), coverage,
  a reopened session, expiry, undecodable frames dropped, an unknown camera; compute returns K, D and RMS near the
  truth and too few views name the count, refused while capturing; apply writes
  `applied/front_camera_calibration.yaml` (head_camera_cpp's loader keys, the C++ test fixture's key set) before the
  `csi_intrinsics` record in `applied.json`, keeps the other records and reports the camera unit; a CSI-only
  `applied.json` leaves the D435 check unaccepted (`test_d435_records_ignore_a_csi_only_applied_json`); the
  `/api/csi/{camera}` routes, an unknown camera 404 on each, too few views 409.

### Floor fit HTTP API

`floor_results.py` computes and applies the fit, `floor_api.py` serves it (`__main__` includes the router; shutdown stops the
sweep first so the head is restored). Errors use the common `{ok: false, message}` body (404 unknown or malformed ids, 409 busy
or not applicable).

| Route | Body |
|---|---|
| `GET /api/floor/state` | `{sweep, applied}` (applied = `applied/head_chain.json` content, null when absent) |
| `POST /api/floor/sweep/start`, `/stop` | the sweep state; start gives 409 while a capture, a compute or a sweep runs |
| `GET /api/floor/sessions` | `[{id, created, observations, skipped, results}]` (result ids) |
| `POST /api/floor/sessions/{sid}/compute` | the result; synchronous; 409 while a sweep runs |
| `GET /api/floor/sessions/{sid}/results/{rid}` | result.json |
| `POST /api/floor/sessions/{sid}/results/{rid}/apply` | `{head_chain: {session, result, applied_at}, units}`; 409 for a failed result or when no D435 intrinsics check is accepted |

A passing compute also writes `head_chain.json` (the applied joints kept, `head_mount_joint` and `head_tilt` replaced; a
`head_pan` origin from a head-from-arm fit is kept, and so is the `head_tilt` xyz it moved). Apply
copies it to `applied/` and adds `head_chain` to `applied/applied.json`, which starts the apply path unit.

## Head from arm calibration

The head-from-arm calibration fits the head chain against the trusted arm: the ChArUco board (6x5, 40 mm, see
`docs/markers.md`) is on the gripper, the arm moves it through scripted poses, the node aims the head at it, and the
fit (`shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`, see "Head from arm fit" in `shared/README.md`) solves
the head mount, the tilt zero and roll and the head pan axis position from the head camera views. It writes the same
`head_chain.json` as the floor fit, so every consumer of the applied chain (head_state_publisher, mcp_server, web_ui) is
served by either fit. This fit trusts the arm offsets in `fit.joint_offsets_rad`; the arm's own zero errors are solved
from the same sessions by "Arm offsets compute and apply" below.

Order of use: the floor fit first (it gives the start chain and the cross-check values), then the Calibrate page
section "Head from arm" (`nodes/web_ui/README.md`) or the routes below.

### Session kind `head_arm`

A session has a `kind` in `session.json`: `intrinsics` (the D435 intrinsics check) or `head_arm` (`rgbd`, the retired RGB +
ToF capture, for sessions without the key).
A `head_arm` session holds still RGB views only. Each pose directory has `rgb.jpg` (as received), `rgb_thumb.jpg` (corners
drawn) and `pose.json` with the stamps (`rgb`, `joints`), the detected corner ids and points, the measured `joints` (all
seven: `head_pan`, `head_tilt` and the five arm joints, never defaults), `arm_pose` (index into the whole scripted set),
`tier`, `head_offset` and the factory colour `camera_info` (width, height, distortion_model, d, k, p).
`SessionStore.add_still_pose` refuses a session of the other kind, a missing joint, a tier other than 0/1/2, a negative
index, a camera_info for another image size and an image size other than the session's; `load_arm_board_views` turns
the poses into `ArmBoardView`s (view id = pose id) and `load_arm_camera_infos` returns the camera_info per pose id. Results live in `results/<rid>/` as for the other kinds.

### Still capture

`CaptureController.start_still_session` and `capture_still` are described under "Still capture (head_arm sessions)"
above. Topics: the D435 colour stream `topics.rgb` (BEST_EFFORT, depth `qos.rgb_depth`), its camera_info
`topics.rgb_info` (RELIABLE, depth 1) and the measured joints
`topics.head_joint_states` (`/head/joint_states`) and `topics.arm_joint_states` (`/follower/joint_states`), both with
the same BEST_EFFORT profile. The still rule: a frame is stored only when the last `still.window_s` (0.6 s) of frames
differ by at most `still.rgb_mad` (2.0 grey levels, flicker-free difference) and every one of the seven joints moved
less than `still.joint_still_rad` (0.002 rad) over the window, with the head and arm samples within
`still.max_joint_dt_s` (0.1 s) of the frame stamp; after `still.timeout_s` (6 s) the view is skipped. A frame without a
detected board is skipped (`no board`), a frame before the first colour camera_info is skipped (`no colour camera_info`),
and a view whose joints are missing or stale is skipped (`joints stale`). Nothing is filled with zeros: no joint sample
means no view.

### Robot client and token

The node never drives the robot itself. `robot_client.RobotClient` calls the guarded tools of mcp_server over
Streamable HTTP (`robot.mcp_url`): `take_control` (arm), `move_arm_joints`, `set_head_angles`, `get_arm_state`,
`get_control_state` and `release_control`. The bearer token is read per call from `robot.token_file`
(`/etc/ros2/mcp_server/token`, a `MCP_SERVER_TOKEN=<token>` line or the bare token); the node's service user joins the
`mcp-token` group for it (Ansible, `ansible/README.md`). A call never raises: it returns a `ToolResult` with `is_error`,
the text (the token replaced by `<token>`), `unreachable` when nothing answers at the URL, and the structured `data`.
Calls have `robot.call_timeout_s` (60 s), arm moves `robot.move_timeout_s` (120 s). The park with the board on uses a
separate plain HTTP admin route and its own token (`robot.board_park_token_file`, group `board-park`, see "Park with the
board on" below).

### The sequence

`sequencer.Sequencer` runs one worker thread; HTTP threads only request steps and read the state. Phases (`phase` in
the state): `idle`, `moving_to_base`, `waiting_board`, `capturing`, `waiting_big_joints`, `returning_to_base`,
`holding`, `parking`.

1. `POST /api/head-arm/sequence/base` (confirm): builds the `PoseChecker` on the arm URDF and the applied head URDF,
   takes the arm lease and moves, without the board, to `arm_sequence.base_pose` (the gripper `gripper_rad` is sent once,
   with this move).
2. Phase `waiting_board`: the user clips the board to the gripper and presses Board attached
   (`POST .../sequence/board-attached`). The board record is written (see "The board record").
3. Phase `capturing`: a still session is started and the sequence waits up to `still.timeout_s` for its first colour
   camera_info (none fails the sequence, holding); tier 0 then tier 1 run.
4. Phase `waiting_big_joints`: tier 2 moves `elbow_flex`, `shoulder_lift` and `shoulder_pan`, so the user must allow it
   (`POST .../sequence/big-joints {"confirm": true}`) or skip it (`false`, or no body).
5. Tier 2 (if allowed), then phase `returning_to_base` back to the base pose, and the worker ends.

Tiers (`arm_sequence`, validated at load: a pose that moves a joint its tier may not, or beyond its delta, names the tier,
pose and joint): tier 0 moves `wrist_roll` only (10 poses, within `max_roll_delta_rad` 1.2), tier 1 also `wrist_flex`
(9 poses, `max_flex_delta_rad` 0.4), tier 2 also the big joints (14 poses, `max_big_delta_rad` 0.35). Poses are measured
joint angles, the `move_arm_joints` space (`urdf = measured + fit.joint_offsets_rad`).

For every pose: the board centre from arm FK is passed to `aim_head` on the applied head chain (in `camera_frame`) and
one view is planned per `arm_sequence.head_offsets_deg` entry (default `[0,0]`, `[10,0]`, `[-10,0]`, `[0,-3]` deg added
to the aimed pan/tilt). A view is skipped and counted with its reason when its head target is outside the head URDF
limits, when a ChArUco corner is nearer than 0.20 m along the optical axis of `camera_frame` (`inside D435 min-Z`: the
D435 min-Z of 0.15 m at 640x360 plus 0.05 m, `pose_check.d435_min_z_m` and `d435_min_z_margin_m`), or when a corner
projects outside the colour image of the session's camera_info (`outside colour FOV`; `PoseChecker.check_view`). A pose
whose every view is skipped at its target joints is not visited. Otherwise the path from the measured joints
(`get_arm_state`) to the pose is checked by `PoseChecker.check_path` and a refusal stops before anything moves (the
named rules are quoted); then `move_arm_joints` at `arm_sequence.speed_scale` (0.1, at most 0.3); then the views are
planned again at the measured joints and each one that is not skipped is taken. A head move that does not converge is
skipped; each view waits `settle_s` and then the still rule above. The state carries `captured` and `skipped`
(reason -> count). The tier and big-joint confirm structure is unchanged by these skips.

Safety and the holding exits: nothing moves without its confirm, enforced in the routes and again in the sequencer.
Every exit of the sequence (done, stop, tool error, exception, unanswered confirm after `confirm_timeout_s` 600 s)
stops the still capture and releases the arm with `keep_position=True`: the arm and the head stay where they are, at a
pose that was path-checked. With the board attached the phase is then `holding` (the board is still on the gripper);
otherwise `idle`. The sequence never parks on its own and never sends a carried object radius. A new base move is
refused while `holding`: press Park first.

### Pose set and its tests

`tests/test_arm_pose_set.py` proves the shipped set before it can reach the robot: tier rules, short transits between
poses, every pose and every transit (home -> base -> tiers -> base) safe by `PoseChecker` with the provisional board
margin and with the mount shifted +-1 cm / turned +-5 deg, on the shipped and the applied head URDF and with the widened
-30..+45 deg tilt range; the board in view of the nominal D435 colour camera (at least 12 corners for 3 of 4 head
offsets); no view inside the D435 min-Z, and the views the colour FOV rule keeps meeting 1.5 times the fit's coverage
limits in every tier; every pose keeping at least 3 views the min-Z and colour FOV rule passes (nominal 640x360
intrinsics), so the sequencer visits every pose, and each head offset usable on at least 3/4 of the poses; enough pan and tilt span for the limits; and a passing synthetic fit on the set. The sequencer re-checks every move from the measured joints at run
time, so a robot that is not where the test assumed is still refused. The tests of the sequence over a fake mcp_server
are `tests/test_sequencer.py`.

### Compute and apply

`POST /api/head-arm/sessions/{sid}/compute` (`head_arm_results.compute_head_arm`, synchronous) fits the session in
`camera_frame` (`d435_color_optical_frame`; d435_joint x, z, roll and yaw are fitted, its y and pitch kept). Inputs: the
stored views, the D435 factory colour intrinsics from the camera_info stored with the stills
(`CameraIntrinsics.from_camera_info`; a still without one, or stills with different ones, give an error result), the
head URDF with the applied overrides as the start chain
(`floor_results.base_urdf`), the arm URDF, `fit` and, as the cross-check, the floor values of the floor result named by
the `head_chain.floor` reference in `applied/applied.json` (none gives a failed `floor_cross_check`). The result
directory `results/<rid>/` holds:

- `result.json`: `passed`, `failed_limits`, `error`, rms, held-out rms and board error, views, arm poses, spans,
  `max_correlation`, `params` (per fitted parameter: `value`, `sigma`, `start`, `floor`, `floor_delta`), `board_mount`,
  `per_view`, `offset_rms_px`, `limits` (name, value, limit, ok) and `floor` (the session/result referenced). An input
  error gives `passed: false` and `error` only.
- `report.md`: the human-readable report.
- `head_chain.json`: only when the result passed; the applied chain's joints with the fitted `head_mount_joint`,
  `head_tilt` and (with the pan axis) `head_pan` origins merged over them.
  It also carries `residual {kind: "head_arm", holdout_board_mm, holdout_rms_px, rms_px}`: the camera-to-arm accuracy
  the fit measured on held-out views. mcp_server reports it as `calibration.residual_mm` in `pixel_to_point` /
  `camera_point_to_arm` and gates camera-derived direct grasps on it (`camera_arm.max_residual_mm`, 5 mm; a passed fit is
  at most `fit.limits.holdout_board_mm`, 4.5). A floor-fit chain carries no residual.

`POST .../sessions/{sid}/results/{rid}/apply` goes through the writer shared with the floor fit,
`floor_results.apply_head_chain_file`: it copies `head_chain.json` into `applied/` and records it in `applied/applied.json`
as `head_chain: {session, result, applied_at, source, floor}`, which starts the apply path unit that restarts the head
nodes (`restart_units`); the answer lists each unit's state. `source` is `head_arm` for this fit and `floor` for a floor
fit; `floor` is the floor result the applied chain was checked against, carried forward from the current record. A
result that did not pass, or an apply while no D435 intrinsics check is accepted, is refused with 409 (the message names the RGB-D section's Accept check, which creates `applied.json`). The web_ui proxy waits `calibration.rgbd_timeout_s` (600 in client.yml) for the synchronous compute.

### Arm offsets compute and apply (design D9, D17)

The arm joint offsets and the tool point are re-solved from the same head_arm sessions (the board on the gripper, all
seven joints per still); there is no separate session kind. `POST /api/head-arm/sessions/{sid}/compute-arm` with body
`{"floor_session": "floor-..." | null}` (`arm_results.compute_arm`, synchronous) runs `arm_offsets_fit.fit_head_arm` on:

- the plate views of the session (`store.load_arm_board_views`) and the D435 factory colour intrinsics stored with the
  stills (as the head-from-arm compute);
- the floor planes of the floor session named in the body, else of the floor reference of the applied head chain
  (`floor_results.current_floor_ref`); with neither the result is an error, `apply a head calibration first`;
- the head URDF with the applied overrides (`floor_results.base_urdf`), the arm URDF and `fit`, starting from the
  current offsets `fit.joint_offsets_rad`.

`fit.plate_mount` must be set (the plate start pose), and `fit.wrist_roll_offset_rad` too while `plate_mount.rpy` is
null; otherwise the result is an error naming the key and no fit runs. `plate_mount.tool_in_board_m` is required with no
default: while it is null the tool point is not fitted, the current `tool_offset_m` is kept and the report says so.

The result directory `results/<rid>/` holds `result.json` (`{id, session, kind: "arm", floor_session, floor, created,
passed, failed_limits, error, rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, tool_fitted, tool_offset_m,
tool_offset_current_m, offsets: {joint: {value, sigma, current}}, head: {name: {value, sigma, urdf}}, limits,
snippet}`; `sigma` of a fixed offset is null, `tool_offset_current_m` is the tool point the fit started from), `report.md` and, only when it passed, `head_chain.json` (every head joint, d435_joint included),
`head.urdf` and `joint_offsets_snippet.yaml` (the `joint_offsets_rad` block, plus `tool_offset_m` when the tool point
was fitted, for the client.yml mcp_server `arm:` block). `floor` is `{session, result}` of the floor session used (its
newest result when the session was named in the request).

`POST .../results/{rid}/apply` dispatches on the result's `kind`: an arm result applies only its `head_chain.json`
through `floor_results.apply_head_chain_file` with the record `source: "arm"`; applied.json's top-level intrinsics
check record is left as it is. The snippet never goes to `applied/`: the offsets are committed to client.yml by hand
after review. `GET /api/sessions/{sid}/results/{rid}/download` zips `joint_offsets_snippet.yaml` with the report.

### Routes

All under `/api/head-arm` (web_ui proxies them as `/api/calibration/rgbd/head-arm/*`); errors use the common
`{ok: false, message}` body: 400 for a refused step (a missing confirm, a refusal of the path check, a session of the
wrong kind), 404 for an unknown or malformed id, 409 for a busy or wrong-phase request.

| Route | Body |
|---|---|
| `GET /state` | `{sequence, applied}`: the sequencer state (`running`, `phase`, `session_id`, `tier`, `arm_pose` (0-based), `total_poses`, `captured`, `skipped`, `status`, `error`, `big_joints`, `waiting_since`, `board_record`) and the applied head chain (`{session, result, note, source, floor, joints}`, null when none) |
| `POST /sequence/base {"confirm": true}` | state; 409 while a sequence, an intrinsics capture, a floor sweep or a compute runs |
| `POST /sequence/board-attached` | state; 409 outside `waiting_board` |
| `POST /sequence/big-joints {"confirm": bool}` | state; 409 outside `waiting_big_joints` |
| `POST /sequence/stop` | state (also when idle) |
| `POST /park {"confirm": true, "board_on": bool}` | state; see "Park with the board on" |
| `GET /sessions` | `[{id, created, views, arm_poses, results}]` (head_arm sessions only) |
| `POST /sessions/{sid}/compute` | the result; 409 while a sequence runs |
| `POST /sessions/{sid}/compute-arm {"floor_session": str \| null}` | the arm offsets result (`kind: "arm"`); 409 while a sequence runs |
| `GET /sessions/{sid}/results/{rid}` | result.json (either kind) |
| `POST /sessions/{sid}/results/{rid}/apply` | `{head_chain: {session, result, applied_at, source, floor}, units}`; `source` `arm` for an arm result |

The bodies are pinned by the contract fixtures `tests/fixtures/api/head_arm_state.json`, `head_arm_sessions.json` and
`head_arm_result.json` (the web UI validators in `frontend/src/calibrate/headArmApi.ts` are tested against the same
files), checked in `tests/test_head_arm_api.py` with `tests/contract.py`.

### Config keys of the head-from-arm calibration

| Key | Default | Meaning |
|---|---|---|
| `topics.head_joint_states`, `topics.arm_joint_states` | `/head/joint_states`, `/follower/joint_states` | Measured joints of the still capture |
| `still.window_s`, `rgb_mad`, `joint_still_rad`, `max_joint_dt_s`, `timeout_s` | 0.6, 2.0, 0.002, 0.1, 6.0 | The still rule |
| `arm_urdf_file` | deployed checkout `so101_arm.urdf` | Arm URDF for the fit and the pose check |
| `fit.*` | see "Config keys" | Board, board mount start, pan axis, limits, Huber scale, arm gauge, joint offsets, floor prior weight |
| `robot.mcp_url`, `token_file`, `board_park_token_file`, `call_timeout_s`, `move_timeout_s` | `http://127.0.0.1:18200/mcp`, `/etc/ros2/mcp_server/token`, `/etc/ros2/mcp_server/board_park_token`, 60, 120 | The mcp_server the node moves the robot through |
| `arm_sequence.*` | see "Config keys" | Base pose, gripper, the three tiers, speeds, confirm timeout, head offsets, pose check, margins, park |

`fit.board` must equal `board` (checked at load). Unknown keys are rejected.

## Park with the board on (design D63)

`POST /api/head-arm/park {"confirm": true, "board_on": false}` works with the calibration board still on the gripper, so
a retry after a refusal needs no board removal. The board counts as on when this sequence clipped it, or the user
states it now (`board_on: true`, the Calibrate page button "Park (board is on the gripper)" in the idle state). The
sequencer reads `get_control_state`:

- Board not on: the plain `release_control` park; a holding load is refused as ever, with a hint to state the board.
- Board on (whatever the load reads): the path from the measured joints to `arm_sequence.park_pose` is checked with
  `PoseChecker` and the board outline of the newest fitted `board_mount` of the session
  (`head_arm_results.latest_board_mount`), else the recorded mount, else `fit.board_mount`, with
  `park_board_sag_margin_m` added to the board floor (`min_board_height_m`), body and head clearances. A violation
  refuses with the named rules ("Park refused by the path check with the board on ...") and the arm keeps holding. A pass
  moves to `park_pose` through the guarded `move_arm_joints` at `arm_sequence.speed_scale` (0.1), then calls mcp_server's
  internal `POST /admin/park_calibration_board` with the board mount as the only content (`robot.board_park_token_file`,
  its own token). mcp_server owns the outline and margins, refuses a mount over 30 mm / 10 deg from its configured one,
  requires the measured arm at its rest pose (the stored home, which `park_pose` equals), FK-checks the board on every
  home leg from the measured joints against the floor and the body, and parks at 0.1 (see its README). The
  agent-facing `release_control` has no such argument and claude_agent cannot read the board-park token.
- Any carry reason other than the holding load (a held grasp, a carried radius) refuses with mcp_server's reasons quoted.

### The board record

When the board is attached (and again when the sequence ends, with the capture session id) the sequencer writes
`<state_dir>/head_arm_board.json` (`session_id`, `mount` used, `clipped_at`). A completed park or a new base move removes
it. A node start only offers a record: it is shown as `sequence.board_record` (the Park dialog mentions it) when it is
younger than `arm_sequence.board_record_max_age_s` (7200 s) and its session is a stored head_arm session; older, unknown
or unreadable records are ignored and logged. The record never switches the board on: after a restart the user must
state that the board is on (`board_on: true`, the idle "Park (board is on the gripper)" button) before a board park, and
the path checks still run. Without that statement a clamped board is refused.
