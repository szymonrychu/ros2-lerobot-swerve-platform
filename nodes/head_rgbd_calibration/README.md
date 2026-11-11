# head_rgbd_calibration

Robot-side capture, review and compute service for the head RealSense D435 calibration and the CSI camera intrinsics.
It runs six calibrations and
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
- **Ground board**: the FULL board lying flat on the floor, the head sweeping around it at 1-3 placements; its
  colour views fit the camera to ground head chain, depth floor planes cross-check it, the user accepts or declines on
  the estimated accuracy and a verification reads /ground/state afterwards ("Ground board calibration").
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
ground_board/<sid>/                         ground board sessions ("Ground board calibration")
applied/                                    applied.json, the applied head_chain.json and the accepted
                                            arm_calibration.json (arm to camera)
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
pose, never a zero. `stop()` destroys every subscription. `fit.board` is the ARM board (see "Board roles").

## Board roles

The node works with two printed ChArUco boards (`docs/markers.md`, "Calibration boards"); both are generated by
`scripts/markers_generate.py` and their `BoardConfig` constants are `FULL_BOARD` and `ARM_BOARD` in
`shared/ros2_common/head_rgbd_calibration/config.py`:

| Role | Config key | Board | Marker ids | Used by |
|---|---|---|---|---|
| FULL (`intrinsics`) | `board` | `charuco-6x5-40mm`: 6x5, 40 mm squares, 30 mm markers, 250 x 210 mm plate | 0-14 | D435 intrinsics check, CSI camera intrinsics |
| ARM (`head-arm and arm offsets`) | `fit.board`, `fit.plate` | `charuco-arm-6x5-28mm`: 6x5, 28 mm squares, 21 mm markers, 178 x 150 mm plate with the edge grip handle | 15-29 | Head from arm, arm offsets (the board held in the gripper) |

Both use `DICT_5X5_100`. The id overlap rule: `board` must not share a marker id with `fit.board` or `fit.plate`
(`NodeConfig.check_board_ids`, "board <name> ids 0-14 overlap fit.board <name> ids ..."), so a view that shows both
boards (the FULL board on a wall and the ARM board in the gripper) never mixes their corners. `GET /api/state` lists
both boards with their role in `boards` (`app.board_roles`), and the Calibrate page shows the board each step needs
with its PDF and 3MF download (`nodes/web_ui/README.md`).

ARM board mount (CAD seed): `fit.board_mount` and `fit.plate_mount` start from the handle CAD, the `board_mount` and
`handle_in_board` blocks of `markers/charuco-arm-6x5-28mm.json` (rounded to 6 decimals in `config.py`: `CAD_BOARD_XYZ`,
`CAD_BOARD_RPY`, `CAD_TOOL_IN_BOARD_M`, `CAD_HANDLE_BOXES_MM`; node yaml and the client.yml block equal them, tests
pin both to the JSON). The board frame (origin the first inner corner, pattern at z = 0, back at z = 4 mm) sits in
`gripper_link` at xyz (-0.0794, 0.187949, -0.118113) m, rpy (pi, 0, 0): the pattern plane lies across the tool axis
about 20 mm past the URDF tool point, the pattern facing the wrist and the head camera, the plate beside the jaws
along `gripper_link` +y. It is a seed, not fit-solved: the fit refines the real mount from it, and the pose check adds
`provisional_margin_m` (0, user decision 2026-11-10) while the mount is still the seed. `fit.board_mount.handle_boxes_mm` (the JSON
`handle_envelope_mm`) are three boxes in the board frame (mm) around the material beyond the plate, all on the pattern
side (z < 0) past the plate's bottom edge: the post `[[71.5, 175.7, -42], [100.05, 200.3, -6]]`, the gusset
`[[80, 145.7, -42], [88, 175.7, -6]]` and the tab `[[34, 145, -6], [134, 200.3, 4]]`; the pose check and the park
check sample them as board points (`handle_box_points_local`), so the post top 42 mm above the pattern face is kept
off the floor, the body and the head.

Board lattice: the plate faces and each handle box are sampled by the shared module
`shared/ros2_common/head_rgbd_calibration/board_lattice.py` (`axis_samples`, `box_lattice`, `plate_lattice`,
`handle_box_lattice`), laid out from a maximum spacing per axis, `arm_sequence.pose_check.board_spacing_m` (0.005 m,
refused below 1 mm and above `margin_m`), not from a fixed point count (the former 5 x 5 plate lattice and the 3-point
axes of the 100 mm tab left gaps of 50 mm that a body part could slip between). A point check of a solid box can miss at
most half a cell diagonal of clearance, `lattice_miss_bound(spacing)` = spacing * sqrt(3) / 2 (4.33 mm at 5 mm), so
`PoseChecker.board_threshold_m` = the board margin plus that bound: a pose that passes keeps the full margin from the
true board surface. mcp_server's `BoardPark` uses the same module and the same threshold (its `lattice_spacing_m`,
pinned equal to `board_spacing_m` by `tests/test_board_park_config.py`).

Grip keys (`arm_sequence`): `insert_gripper_rad` 0.4 (the jaws half open while the handle goes in),
`handle_contact_rad` 0.0951 (measured on the robot 2026-11-09; CAD 0.070279, the JSON `contact_rad`, see below),
`handle_squeeze_rad` 0.02 (the JSON `squeeze_rad`; > 0 and below the contact and below `grip_converge_tolerance_rad`)
and the derived `grip_rad` = contact - squeeze = 0.0751 rad, the commanded grip. `grip_converge_tolerance_rad` 0.03 is the mcp_server gripper converge tolerance
(`limits.converge_tolerance_for("gripper")`, pinned equal by `tests/test_board_park_config.py`). `board_mass_g`
211.72779 is the ARM board mass from the CAD volume (PLA 1.24 g/cm3).

Nominal versus printed: the grip faces carry a 0.15 mm print allowance per clamp face (`clamp_clearance_mm`, user
decisions 282 and 283). `handle_contact_rad` (JSON `contact_rad`) is the contact solved on the NOMINAL surfaces (the
handle as printed after FDM growth), not a measured contact; the JSON `modelled_contact_rad` (0.066202) is the
contact on the printed solid, 0.3 mm thinner, pressed on the fixed jaw. The board prints back down, so both clamp faces
(the "bed face" is only the handle-frame name of the fixed-jaw face) are vertical post walls that grow under FDM and a
real print lands near nominal; the true contact still lies between
`modelled_contact_rad` and `contact_rad`; `grip_rad` closes past both, so the grip still clamps. The printed ARM board
handle measured thicker than that on the robot (contact 0.0951 rad, 2026-11-09: the slack check stopped the sequence
at the CAD contact), so `handle_contact_rad` is the measured contact; the CAD JSON stays the model.

## Config keys

File named by `HEAD_RGBD_CALIBRATION_CONFIG`; every key has a default (`config/head_rgbd_calibration.yaml`). Unknown
keys are rejected.

| Key | Default | Meaning |
|---|---|---|
| `state_dir` | `/var/lib/ros2/head_rgbd_calibration` | Sessions, poses, results |
| `arm_stale_marker` | `/var/lib/ros2/arm/calibration_stale.json` | Follower bridge arm re-zero stale marker: the arm to camera accept is refused while it lists joint_offsets for a re-zero at or after the session's first still; null: no check |
| `arm_home_file` | `/var/lib/ros2/arm/home.yaml` | mcp_server's stored arm home (`mcp_server.arm.home_file`, `{joints: {name: rad}}`, written by Set home): read at each Park with the board on as its target (the five arm joints; the gripper entry is ignored). Missing or invalid refuses the park naming the file; nothing moves |
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
| `board` | `charuco-6x5-40mm`: 6x5, 40 mm squares, 30 mm markers, `DICT_5X5_100`, ids from 0, 4 mm plate | FULL board (intrinsics check, CSI intrinsics); equals the `charuco-6x5-40mm` entry of `boards` in `scripts/markers_generate.yaml` (`name`, `ids_offset` included). See "Board roles" |
| `selection` | window 0.5 s, colour MAD 2.0, depth MAD 0.01, skip 2.0 s | Still-pair selection of the intrinsics check |
| `limits` | plane bias 10 mm, plane spread 50 mm, 12 views, RMS 0.5 px, focal 1 %, principal point 5 px | Intrinsics check acceptance limits |
| `floor` | see "Head floor fit" | Floor sweep and fit; its motion and plane settings also drive the ground board session and verification |
| `ground_board` | see "Ground board calibration" | Ground board session grid, detection, placements, fit limits and bounds, verification |
| `still` | window 0.6 s, MAD 2.0, joints 0.002 rad, joint dt 0.1 s, timeout 6 s | head_arm still capture |
| `robot` | `http://127.0.0.1:18200/mcp`, `/etc/ros2/mcp_server/token` | mcp_server the sequencer drives the arm and head through |
| `fit.board` | `charuco-arm-6x5-28mm`: 6x5, 28 mm squares, 21 mm markers, ids from 15 | ARM board of the head-from-arm fit and the still capture; its marker ids must not overlap `board` |
| `fit.board_mount` | CAD seed: xyz (-0.0794, 0.187949, -0.118113), rpy (3.141593, 0, 0), `outline_margin_m` 0.01, `handle_boxes_mm` post, gusset, tab | Start value of the ARM board pose in `gripper_link` from the handle CAD (`markers/charuco-arm-6x5-28mm.json`), not fit-solved (design D47); the fit solves the real mount. `handle_boxes_mm`: the handle material beyond the plate, checked as board points by the pose and park checks. See "Board roles" |
| `fit.fit_pan_axis` | true | Head-from-arm fit: also fit the head pan axis position (`pan_axis_x_m`, `pan_axis_y_m`: the `head_pan` origin x, y in `head_pan_base`); the `head_tilt` origin moves back by the same shift, so the head is unchanged at pan 0 and only the axis it turns about moves. Session calib-20261023-074418 put the axis 37.9 mm behind and 11.2 mm right of the URDF value: the fixed axis left 13 px at the +-10 deg pan offsets (rms 12.2 px), the free axis fits every head offset at 2.6-2.8 px. The result.json `params` carry both entries (no floor value), `offset_rms_px` lists `{head_offset, views, rms_px}` per head offset (report.md: `## Residuals per head offset`), and a passing `head_chain.json` holds the `head_pan` origin for every consumer of `apply_overrides` (head_state_publisher, mcp_server HeadKinematics, web_ui) |
| `fit.fit_camera_mount` | true | Head-from-arm fit: also fit the D435 mount on the head (`camera_x_m`, `camera_z_m`, `camera_roll_deg`, `camera_yaw_deg`: the `d435_joint` origin x, z, roll, yaw, refined from the CAD start); its y and pitch are not observable (degenerate with the mount y and the tilt zero) and keep the head URDF values. A passing `head_chain.json` holds the fitted `d435_joint` origin |
| `fit.limits` | rms 3.0 px, held-out 4.0 px / 4.5 mm, sigma 2.0 mm / 0.2 deg, correlation 0.99, pan axis shift 50 mm, floor 10 mm / 1.5 deg | Head-from-arm acceptance limits. The residual limits are evidence from calib-20261023-074418 over five leave-poses-out folds (`tests/test_head_arm_real_session.py`): the arm is about 2 mm accurate (about 2.5 px at the board), the folds gave rms 2.51-2.72 px, held-out 2.56-3.59 px and 2.89-3.77 mm, sigma at most 0.72 mm / 0.153 deg and correlation 0.973-0.976; the fixed pan axis (12.2 px) and a pan scale factor instead of the axis (3.07-3.33 px) stay above `rms_px`. `max_correlation` refuses degenerate parameter sets (0.994 and more) |
| `fit.floor_prior_weight` | 10 | Weight of the floor-fit prior (one residual per floor parameter, 1 mm / 0.1 deg units). User decision 2026-10-23, replacing D49's prior off: without it calib-20261023-074418 failed the floor cross-check on mount roll, mount pitch and tilt zero (2.1-2.8 deg); with it every limit passes at rms 2.93 px, held-out 2.88 px / 2.86 mm |
| `fit.plate`, `fit.plate_mount` | the ARM board; CAD seed: xyz/rpy as `fit.board_mount`, `rpy` (3.141593, 0, 0), `tool_in_board_m` (0.0705, 0.193767, -0.021386) | Arm offsets fit (shared `arm_offsets_fit`, design D9, Q2): the gripper-held plate and its mount in `gripper_link`: `xyz_init` / `rpy_init` start values, `rpy` the CAD rotation (null: the rotation is fitted and wrist_roll stays at `fit.wrist_roll_offset_rad`; set: wrist_roll is fitted), `tool_in_board_m` the CAD board-to-jaw-tip offset (the JSON `handle_in_board.tool_point_m`; null: the tool point is not fitted). The arm offsets fit refuses to run without `plate_mount` |
| `fit.wrist_roll_offset_rad`, `fit.tool_offset_m` | null, (0.0010, -0.0056, -0.0014) | Arm offsets fit: the wrist_roll zero from the recorded servo neutral (rad; required when `plate_mount.rpy` is null) and the current tool point in `gripper_frame_link` (client.yml mcp_server `arm.tool_offset_m`), reported unchanged when the tool point is not fitted |
| `fit.floor_fit`, `fit.arm_limits` | floor fit defaults; rms 3.0 px, held-out 4.0 px, tool 5.0 mm, 40 views, every 5th held out over 3 head settings | Arm offsets fit: the floor views' plane fit, limits and residual scales, and the acceptance limits; held-out plate views decide (reprojection px and the FK tool point against the view's solvePnP plate pose), the tool point is fitted only below `max_tool_holdout_mm` |
| `arm_sequence.base_pose`, `insert_gripper_rad` | board ~0.34 m in front of the head camera, 0.4 rad | Base pose (measured joint angles, the `move_arm_joints` space: `urdf = measured + fit.joint_offsets_rad`) and the gripper opening sent once with it while the handle is inserted (at most `pose_check.max_roll_gripper_rad`) |
| `arm_sequence.handle_contact_rad`, `handle_squeeze_rad`, `board_mass_g` | 0.0951, 0.02, 211.72779 | ARM board handle grip: the contact angle measured on the robot 2026-11-09 (CAD 0.070279), the squeeze past it from the JSON `grip` block (> 0, below the contact); `grip_rad` = contact - squeeze (0.0751) is the commanded grip; the board mass from the CAD. See "Board roles" and "The sequence" |
| `arm_sequence.grip_converge_tolerance_rad` | 0.03 | The mcp_server gripper converge tolerance (pinned equal); a measured jaw at or above `grip_rad` + it after the grip close is a slack grip and stops the sequence. The squeeze must be below it |
| `arm_sequence.tier0_poses` / `tier1_poses` / `tier2_poses` | 10 / 9 / 14 poses | Tier 0 moves only wrist_roll (within `max_roll_delta_rad` 1.2), tier 1 also wrist_flex (`max_flex_delta_rad` 0.4), tier 2 also elbow_flex, shoulder_lift, shoulder_pan (`max_big_delta_rad` 0.35). Load errors name the tier, pose and joint |
| `arm_sequence.speed_scale`, `settle_s`, `confirm_timeout_s` | 0.1 (<= 0.3), 1.0, 600 | Arm speed (slow for the fragile board mount; `move_arm_joints` has no acceleration option, its quintic peak acceleration scales with the speed squared), wait after a move, unanswered confirm timeout |
| `arm_sequence.head_offsets_deg` | `[[0, 0], [10, 0], [-10, 0], [0, -3]]` | (pan, tilt) offsets added to the aimed head angles, one view each. The tilt offset is -3 deg (head up): the board fills 250-340 px of the 360 px D435 colour image height, and the former +6 deg pushed it out on 26 of 33 poses |
| `arm_sequence.pose_check`, `provisional_margin_m` | core defaults, 0.0 | FK pose check settings (`board_spacing_m` 0.005: the largest gap of the board lattice per axis, at most `margin_m`; see "Board lattice"); the board (plate and handle boxes) keeps `margin_m + provisional_margin_m` while the mount is the shipped CAD seed (`provisional_margin_m` 0 by user decision 2026-11-10: no 50 mm allowance); every view keeps each board corner `d435_min_z_m + d435_min_z_margin_m` (0.15 + 0.05 m) deep from `camera_frame` |
| `csi_cameras` | `front`: `/front_camera/image_raw/compressed`, `ros2-front_camera.service`, enabled; `back`: `/back_camera/image_raw/compressed`, `ros2-back_camera.service`, disabled | CSI cameras whose intrinsics are calibrated: name (lower-case letters and digits, it names `applied/<name>_camera_calibration.yaml`) -> `image_topic` (compressed), `unit` (reads the applied file, reported after an apply) and `enabled` (default true; mirrors the camera node's ros2_nodes `enabled`: only enabled cameras are listed by `GET /api/csi` and a disabled one is unknown, 404, to every camera route) |
| `csi_min_view_shift_px` | 20 | A detected CSI view is kept only when its shared corners moved this far (mean, px) from the last kept view |
| `csi_frame_timeout_s` | 10 | A CSI capture that received no frame this long after its start stops as camera not detected (`capture.no_frames`) |
| `arm_sequence.park_board_sag_margin_m`, `board_record_max_age_s` | 0.01, 7200 | Park with the board on (design D63): the arm moves to the stored home (`arm_home_file`) only after the path from the measured joints passes the pose check with the board outline and `park_board_sag_margin_m` added to the board floor, body and head clearances (the live run showed the stick-mounted 214 g board about 4 mm below FK; the 211.7 g ARM board on its handle is lighter and its tab deflects about 0.25 mm by the CAD estimate, so the margin is conservative) |

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
| `GET /api/state` | capture status, compute status, `applied.json` (null when nothing applied; no systemctl call) and the two boards with their roles |
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

- State (`GET /api/state`, `POST /api/capture/start`, `POST /api/capture/stop`): `{capture, compute, applied,
  boards}`. `capture` is `{running, session_id, status, pose_count, started_at, stops_at}`, `compute` is
  `{running, session_id, result_id, started_at, error}`, `applied` is the applied record or null. `boards` is
  `{full, arm}`, each the board's `BoardConfig` fields (`name`, `squares_x`, `squares_y`, `square_mm`, `marker_mm`,
  `dictionary`, `ids_offset`, `thickness_mm`) plus `role` (`intrinsics` for `full`, `head-arm and arm offsets` for
  `arm`), from `board` and `fit.board`.
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
`camera_info` (no placeholder intrinsics) and marker_detector skips it. Only the cameras with `enabled` true are offered
(`GET /api/csi`; the shipped config disables `back` while back_camera is disabled).

- Capture: one at a time (any camera; 409 while one runs). It subscribes `csi_cameras.<camera>.image_topic`
  (BEST_EFFORT, `qos.rgb`) only while it runs and stops after `max_capture_s`. A frame becomes a view only when the
  ChArUco `board` is detected and the view differs from the last kept one (`csi_min_view_shift_px` mean shift of the
  shared corners, or fewer than 6 shared corners); a frame of another size than the session's is not kept. The status
  line says why a frame was not kept. A capture that receives no frame at all within `csi_frame_timeout_s` (head_camera_cpp
  publishes nothing when libcamera finds no sensor, e.g. "camera ... not found" after a failed imx708 probe) stops with
  the status "camera not detected: no frame on <topic> within <n> s of the start; check the ribbon cable and reboot" and
  `capture.no_frames` true (logged on the node logger and as the capture's `stopped` calib_event), instead of waiting
  for frames until `max_capture_s`.
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
| `GET /api/csi` | `{cameras: [names]}`: the enabled cameras in config order (fixture `csi_cameras.json`) |
| `GET /api/csi/{camera}/state` | `{camera, capture, session, applied}` (fixtures `csi_state.json`, `csi_state_running.json`): `capture` `{running, camera, session_id, status, view_count, started_at, stops_at, no_frames}` (latest capture of any camera); `session` (the capture's when it is this camera's, else the newest, else null) `{id, created, view_count, image_size, coverage: {cols, rows, counts}, results}`; `applied` the camera's record or null |
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
stops the head-arm sequence, the ground board session and verification (each returns the head), capture (the D435 and
the CSI one), the floor sweep, kills the running compute children (RGB-D, arm to camera, ground) and destroys the node.

## Telemetry

Every calibration flow logs structured telemetry to the system journal, so a debugging agent can reconstruct where and
why a run went wrong from `journalctl` alone (T-99 decisions 308 and 309: the journal only, no files, routes or UI).
`head_rgbd_calibration/telemetry.py` writes one stdlib logging line per event:

```
calib_event {"flow":"head_arm","run_id":"seq-20261109-101500-3fa2c1","step":"path_check","outcome":"refused","reason":"2 violated entries","data":{...}}
```

- Keys: `ts` (local ISO time the event happened), `flow`, `run_id`, `step`, `outcome`, `reason` (why it failed, was
  refused or skipped), `data` (inputs and measured values) and, for a timed step, `duration_s`. Errors log at ERROR,
  refusals, skips and stops at WARNING. `calib_event` never raises: an event it cannot encode is logged with
  `data.encode_error` and the names of its data keys.
- `run_id`: the session id (floor sweep, D435 and CSI capture), the result id (computes, applies), else a generated
  `<prefix>-YYYYmmdd-HHMMSS-<hex>` (`seq-` a head-arm sequence, `park-` a park, `req-` an HTTP request).
- Values are JSON-safe (numpy, paths, dataclasses, pydantic models, NaN as `"nan"`) and bounded: a numeric list longer
  than 12 (per-view corner counts, residuals) becomes `{count, head, min, max, mean}`, a numeric array
  `{shape, count, min, max, mean}`, any other list longer than 40 `{count, head}`, a string is cut at 400 characters,
  and a line over 16000 characters keeps only the names of its data keys (`truncated_chars`, `keys`; journald splits
  longer lines). The value of a key naming a token, secret, password or authorization is `<redacted>` (also in a request
  query and JSON body; a body that is not JSON is logged as its size only); the MCP bearer token and the board park token
  are never passed to the log.
- `step_timer` logs a step's `start` and its end with `duration_s`: `ok` (or the outcome the step sets), `stopped`
  (a Stop), or `error` at ERROR with `error_type`, the exception message and the step's inputs.
- The compute child (`compute.py`) logs to its stderr in `CHILD_LOG_FORMAT`; when it ends, `jobs.py` re-logs each such
  line under its own logger and level (`forward_child_log`), so its events reach the journal with its run id; other
  stderr lines (a traceback) are logged at WARNING and still form the job's error tail. The forwarded lines reach the
  journal when the child ends, so their journal time is the end; `ts` in the event is when it happened.
- The CSI flow logs through the same helper (stdlib logging); the rclpy node logger keeps only the throttled frame-drop
  warnings and the executor-callback errors of the capture and CSI callbacks.

Events per flow:

| Flow | run_id | Steps (outcomes) |
|---|---|---|
| `head_arm` | `seq-...` | `sequence` (start with every gating config value; done, stopped or error with `failed_step`, session, counts), timed `take_control`, `base_move`, `wait_board`, `handle_grip`, `still_session`, `camera_info` (intrinsics), `pose` (target, captured, skipped; `not_visited` when every view is skipped), `wait_big_joints` (`allowed`), `return_to_base`; `get_arm_state` (measured joints), `path_check` (ok or refused with start, target, gripper and every violated entry), `move_arm_joints` and `set_head_angles` (MCP status, message, result), `tool_error`, `gripper_measured` (seated, unseated, slack against the limits), `view` skipped (head command, rule), `still` stored or skipped (pose id, corners, board ids, still-window stats, joint stamp gaps), `board_attached`, `big_joints`, `hold`, `board_record` (offered, ignored, written) |
| `head_arm` | `park-...` | `park` (start with mount, refused, done or error with the MCP `reset`), `control_state` (carry reasons), `park_path_check` (mount source, margins, entries), timed `park_take_control`, `park_move`, `park_release`, `hold` |
| `head_arm_still` | session id | `still_session` start and stopped |
| `floor_sweep` | session id | `sweep` start (grid, config, start pan/tilt), refused start (error), `pose` observed (reached pan/tilt, plane heights, RMS, inliers, dropped clouds) or skipped (reason, last measured pan/tilt), `restore`, `release`, `sweep` done, stopped or error with counts |
| `floor_compute` | result id | `inputs` (observations, poses, skipped, dropped), timed `fit`, `result` passed or failed (params, RMS, residuals per pose, failed limits, limits) |
| `d435_capture` | session id | `capture` start (selection and board config), `pose` stored (corners), `capture` stopped (reason, pose count, last status) |
| `d435_compute` | result id | `child` start and end (exit code, duration, error) in the node; `inputs`, `detections` (corners per still, stills without a board), `result` passed, failed or error in the child |
| `d435_accept` | result id | `accept` accepted or refused (reason, failed limits) |
| `head_arm_compute` | result id | `inputs` (views per tier and head offset, corners, intrinsics, floor reference), `result` passed, failed or error (params value and sigma, RMS, held-out, every limit check, worst views) |
| `arm_compute` | result id | `inputs` (plate views, floor reference, current offsets), `result` passed, failed or error (offsets, head params, tool point, limit checks) |
| `arm_camera_compute` | result id | `child` start and end in the node (the job); in the child: `inputs` (views per tier and head offset, corners, intrinsics, the fixed head chain source, applied offsets and base the fit starts from with `applied_source` ({session, result} of the accepted calibration, null for the configured values), tool point, `fit.arm_camera`), `solver` (status, message, evaluations, cost, start and end RMS, free parameters, largest correlation, condition number), one `refusal` (refused, WARNING) per refusal and one `warning` per warning (name, value, limit, unit, message), `result` plausible, implausible or error (score, params, skew, sag, accuracy examples, the applied calibration's score, refusal and warning names) |
| `arm_camera_accept` | result id | `accept` accepted (record, offsets, base, `captured_from`, unit states) or refused (reason, in check order: not done, declined, implausible, arm re-zero after the capture or unknown, invalid calibration, head chain changed); `decline` declined (plausible, score) or refused |
| `ground_sweep` | session id | `sweep` start (config, home, start pan/tilt) or refused start (error); `placement` start, `located` (located board, aim, grid, dropped poses, home corners, upscale) or `failed` (WARNING, reason: home not reached, no camera_info, board not detected at home, not located), `awaiting`, `next`, `wait_ended`, `done` (poses, skipped, views), `home_unconfirmed`; `pose` observed (target, reached, corners, upscale and depth bias per frame, dropped frames, planes, plane heights, dropped clouds) or skipped (WARNING, reason); `restore`, `release`; `sweep` done, stopped or error (placements, failures, views, planes, skipped, reasons) |
| `ground_compute` | result id | `child` start and end in the node (the job); in the child: `inputs` (views, poses, placements, planes, skipped poses, placement failures, depth bias summary, camera_info, the head chain the fit starts from, `ground_board.fit`, frames), `solver` (status, message, evaluations, cost, start and end RMS, free parameters, largest correlation, condition number), one `refusal` per refusal and one `warning` per warning, `result` plausible, implausible or error (score, params, placements, cross-check, accuracy examples, the applied chain's score, refusal and warning names) |
| `ground_accept` | result id | `accept` accepted (record, score, joints, unit states) or refused (reason: not done, declined, already accepted, implausible; a refusal of the apply itself, no accepted D435 intrinsics check or head chain changed, is logged once by `head_chain_apply`); `decline` declined (plausible, score) or refused |
| `ground_verify` | `verify-YYYYmmdd-HHMMSS` | `verify` start (head chain, result, poses, tolerances, units) or refused start; `units` restarted, not_restarted (WARNING) or skipped; `pose` passed or failed (WARNING; medians, maxima, samples, every check, reached angles); `record` error; `restore`, `release`; `verify` passed, failed, stopped or error |
| `head_chain_apply` | result id | `apply` applied (record, files written, unit states, units not restarted yet) or refused |
| `applied` | none | `write` (every applied/ write: files with sizes, applied.json keys before and after) |
| `csi_capture` | session id | `capture` start, `view` kept (corners), `capture` stopped |
| `csi_compute`, `csi_apply` | result id (session id for a refusal) | `compute` ok (K, D, RMS, per-view errors) or refused; `apply` applied (file, unit state) |
| `api` | `req-...` | `<METHOD> <path>` for every POST, PUT, PATCH, DELETE: query, body, status; `refused` (4xx) with the reply message, `error` (5xx or exception); a request cancelled by a client disconnect is not logged, and the `req-` id is not the flow's run id (the flow logs its own) |

Reading it on the robot (through the deploy tooling):

```bash
journalctl -u ros2-head_rgbd_calibration -o cat | grep -o 'calib_event {.*' | cut -d' ' -f2- | jq
# one run, in order
journalctl -u ros2-head_rgbd_calibration -o cat | grep -o 'calib_event {.*' | cut -d' ' -f2- \
  | jq -c 'select(.run_id=="seq-20261109-101500-3fa2c1")'
# every failure of today
journalctl -u ros2-head_rgbd_calibration -o cat --since today | grep -o 'calib_event {.*' | cut -d' ' -f2- \
  | jq -c 'select(.outcome=="error" or .outcome=="refused")'
```

`docs/runbooks/head-arm-calibration.md` ("Reading a failed run from the journal") walks through a failed head-arm run.

## Tests

`cd nodes/head_rgbd_calibration && uv run pytest tests/ -v`

- `tests/test_telemetry.py` - the `calib_event` line (prefix, compact JSON, level, reason, rounded duration), JSON-safe
  conversion of numpy scalars and arrays, paths, dataclasses, pydantic models, datetimes and NaN/inf, the bounds (long
  numeric lists and arrays summarised, long lists cut to their head, long strings cut), redacted secret keys, odd
  values never raising, `step_timer` (start and ok with duration and fields, an outcome set by the body, an exception
  logged at ERROR with the step inputs and re-raised, a quiet exception logged as stopped), `new_run_id`, `events_in`,
  and `RequestTelemetry` (GET not logged, POST ok with body and query, 4xx refused with the reply message, an exception
  as error).
- `tests/test_sequencer_telemetry.py` - the head-arm journal over the sequencer rig: a full run under one `seq-` run id
  with every timed step, the measured joints, path checks, MCP results, the gripper reading against its limits, still
  outcomes with their detail and the end counts; each UI event is exactly one journal line; an unseated handle names
  `handle_grip` and the reason; a missing measured joint in the base move; a path-check refusal logs the start joints,
  the target and the violated entries; a move that does not converge logs the MCP result; a stop logs `stopped`; a
  board park logs its `park-` run, the carry reasons, the mount source and the MCP reset; a refused board park logs its
  entries; no token in any log line.
- `tests/test_capture_telemetry.py` - an intrinsics capture logs its start, each stored pose with its corner count and
  its end; a stored still carries its corners, board ids, window stats and joint gaps; a still that never settles names
  the moving joint; stale joints report the gap; a still session logs its start and stop.
- `tests/test_compute_telemetry.py` - child stderr lines in `CHILD_LOG_FORMAT` re-logged under their logger and level,
  other lines at WARNING and kept as the error tail; a failed and a passing child log their end with the exit code; the
  compute logs inputs, detections and the check, a failing check its failed limits, an input error its reason.
- `tests/test_floor_telemetry.py` - a sweep logs its config, each observed and skipped pose (reason, last measured
  pan/tilt), the restore and its end; a refused start and lost joint states log the reason; the floor compute logs its
  inputs and the fit.
- `tests/test_head_arm_results.py`, `tests/test_compute_arm.py`, `tests/test_csi_intrinsics.py`, `tests/test_app.py`
  also check the telemetry of their flow: the compute inputs, values and limit checks of a pass, a failed limit and an
  input error, the applied files and unit states, the CSI capture, compute and apply events and a refused compute, and
  the `api` events of mutating requests.

- `tests/test_compute_arm.py` - the arm offsets compute and apply on a synthetic floor session and a head_arm session
  of plate views: result files and snippet, no floor session is an error, the named floor session wins, failed limits
  write only report and result, the tool point kept without `tool_in_board_m`, a missing plate mount or wrist_roll
  neutral is an error, apply writes the chain with `source: "arm"` and never the snippet.
- `tests/test_config.py` - config defaults equal the shipped YAML, unknown keys rejected, the `floor` defaults and the
  pan/tilt range checks, the `ground_board` defaults (home equal to mcp_server's head home, the runbook verification
  tolerances) and its range checks (the home pose included), the `arm_sequence` defaults and its validation errors naming the tier, pose and joint; the
  board roles (`board` FULL, `fit.board` / `fit.plate` ARM) and the id overlap refusal, the CAD seed mounts, handle
  boxes and grip keys equal to `markers/charuco-arm-6x5-28mm.json`, a squeeze at or past the contact refused, the
  `arm_home_file` default (mcp_server's `home.yaml`) and `arm_sequence.park_pose` refused as an unknown key.
- `tests/test_board_facing.py` - the ARM board on its edge handle faces the wrist and the head camera: the CAD mount's
  pattern normal points against the tool axis, and at the shipped base pose with the head aimed at the board the
  pattern faces the D435 (facing cos 0.971 > `MIN_FACING_COS`).
- `tests/test_app.py` - also `GET /api/state` `boards` (both boards with their roles, `board_roles`).
- `tests/test_sequencer.py` - the sequence over a fake mcp_server: every step failure logged at WARNING or above with
  its message (`test_step_failure_is_logged_with_its_message`), the recent events of a full run newest first with
  start, step outcomes and `sequence done`, a failed sequence's `sequence error` event in the phase it failed in, a
  stop's warning event, park start / done / failed events (logged) and a refused park or start as error events, the
  list bounded to `EVENT_LIMIT` (`test_events_are_bounded_to_the_newest`); the gripper opens to `insert_gripper_rad` with the base
  move and closes alone to `grip_rad` after Board attached, a jaw closed past `contact - squeeze / 2` (unseated) or no
  reported gripper angle stops the sequence holding the arm, the threshold boundary passes; the resume with the board on (refused unless holding with the board on, not running, not blocked and with usable URDFs; a new run whose only gripper target is the measured jaw angle, never `insert_gripper_rad` and no handle grip, no `waiting_board`, a new still session and the tiers, ending holding with the board on; a path check refusal or a missing gripper angle moves nothing; a stop keeps holding; from idle: refused without `board_gripped`, while running or blocked, and with open or unreported jaws (nothing moved, no board record), after a restart a run with the jaw kept, no `waiting_board`, the `board_confirmed` event and the restored mount, a path refusal moves nothing and ends holding); views inside the D435 min-Z or outside the colour FOV skipped with the reason and a pose with no usable view not visited (`test_poses_inside_min_z_are_skipped`), the wait for the colour camera_info; including Park with the board on (checked even when no load is reported, then the internal admin park; the board record shown at any age with `offered` true only fresh and for a stored session and `problem` naming why not, the stated-on Park, a stale record plus a non-board load refused; `clear_board_record` needs the confirm, removes an old record with a `board_record_clear` calib_event and no mcp_server call, returns a holding sequence to idle so Park is the plain park, says when there was no record, and is refused while a sequence runs): path-checked then moved at speed 0.1 to the stored home read from `arm_home_file` at each press (its gripper entry no target; a re-stored home is the next target; a missing or invalid home file refused naming it, nothing moved) and released with `park_calibration_board`, refused with the named rule when the sag margin eats the clearance, the fitted mount hook, another carried object and a load without this sequence's board still refused, a failed move keeps holding.
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
- `tests/test_board_park.py` - the park with the 211.7 g edge handle board: the board is no heavier than the 214 g board
  the 4 mm park sag was measured with (`test_board_is_no_heavier_than_the_board_the_park_sag_was_measured_with`), and the
  path from the base pose and every tier pose to the stored home passes the node's park check (board margin plus
  `park_board_sag_margin_m`, `min_board_height_m` plus the sag, plate and handle boxes) at the CAD seed mount and at a
  fitted mount, for the footprint's parked arm and for the home the servo calibration stored on 2026-11-11
  (0, -pi/2, pi/2, 0, 0) (`test_park_path_with_the_board_is_clear_from_every_pose`).
- `tests/test_arm_home.py` - `load_arm_home` reads mcp_server's home file: the five arm joints with the gripper dropped,
  integer angles as floats, and a refusal naming the file for a missing file (asks to store a home), unparsable YAML, no
  `joints` mapping, a missing arm joint and a non-finite or non-number angle.
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
- `tests/test_head_arm_api.py` - the `/api/head-arm/*` routes over a fake sequencer: state and sessions bodies against the contract fixtures, the confirms (base, resume with the board on, from idle only with `board_gripped`, park, big joints), 409 for busy and wrong-phase requests, sequencer errors mapped to 400, only head_arm sessions listed, compute and apply, a failed result not applicable, an rgbd session 400 on the result routes, unknown ids 404.
- `tests/test_applied_chain_for_mcp.py` - the page's output consumed by mcp_server (2026-10-29): a synthetic session computed and applied, then the repo head URDF with `apply_overrides` on `applied/` through `mcp_server/head_kinematics.py` places the D435 colour optical frame within 5 mm of the truth chain at 5 head poses (`test_applied_files_place_the_camera_for_the_mcp_kinematics_within_the_target`), `head_chain.json` is in place before `applied.json` changes (mcp_server reloads on that mtime), an apply without an accepted check names the Accept check button, `state_dir/applied` is the directory mcp_server (`head.applied_dir`) and web_ui read, and the web_ui `rgbd_timeout_s` is not below `compute_timeout_s`. `test_head_arm_results.py` also pins the `residual` block of `head_chain.json`.
- `tests/test_head_arm_results.py` - compute writes `result.json`, `report.md` and (passed only) `head_chain.json`, fitted joints merged over the applied chain, the pan axis, the d435_joint fitted from the stills' camera_info in `camera_frame` with no `tof_joint.json` (`test_result_fits_the_d435_joint`), a still without camera_info, stills with different camera_info and a session without stills as error results, a failed limit writes no chain, floor values from the applied record (also the legacy record, and none failing the cross-check), apply writes the applied chain and the record with `source` and `floor`, `head_chain.json` lands in `applied/` before `applied.json` (`test_head_arm_apply_writes_the_head_chain_before_applied_json`, `tests/applied_writes.py` records the order), the floor reference carried forward is the one applied at the write (a floor apply starting right after its read lands after it), a failed result refused, `latest_board_mount`, `head_arm_session_exists`.
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
- `tests/test_jobs.py` - command and niceness, busy, timeout, stderr tail, unknown session (fake child commands), a job
  with its own child module and telemetry flow (the arm to camera job), a job on a `GroundStore` (the ground compute,
  results under ground_board/<sid>/results/), a result id whose temp directory exists is skipped.
- `tests/test_ground_sweep.py` - the ground board session against the floor sweep's fake head world plus fake colour
  frames (the detection is the board projected through the head chain at the frame's head angles), aligned depth
  (rendered floor) and camera_info: `plan_grid` keeps poses with the board in view and clamps the tilt; `detect_frame`
  retries upscaled and gives None without enough corners; a placement goes home first, locates the board within
  2 cm / 2 deg, sweeps the grid, writes views (keys, frames, corners), planes and camera_info, matches the session
  state fixture and awaits; a next placement locates the moved board with unique pose indices; the session ends after
  `max_placements` or after `placement_wait_s` without a next placement; a board not visible at home fails the placement with its message and a later placement works; too
  few corners skip poses with the reason; the depth bias against the board plane is under 3 mm; a stop during a
  placement keeps the views and restores the head; blocked and double starts are refused; no joint state refuses the
  start; the telemetry steps.
- `tests/test_ground_results.py` - the ground compute, accept, decline and state on the synthetic ground scene
  (`tests/ground_board_synthetic.py`) written as observations.json: a done plausible result with head_chain.json and
  head.urdf recovering the true mount z; unknown and malformed ids; too few views and a missing camera_info are failed
  results; the compute telemetry (inputs, solver, refusal, result); running and crashed results from the job state;
  accept applies the head chain (`source: "ground_board"`, `floor: null`), stamps `accepted_at`, shows in the state
  and refuses a decline; accept after the applied chain changed, of an implausible, declined, unchecked or already accepted
  result is refused, and a refused apply (head chain changed) is one `head_chain_apply` event, not repeated by
  `ground_accept`; refusals come in check order (implausible, then a missing D435 check, before a changed head
  chain), like the arm to camera accept; a head chain apply that starts right after the accept's applied chain check lands after the accept,
  never in between (`tests/applied_writes.py` `interleave_apply`); accept and decline telemetry; the session list; the
  arm to camera source name.
- `tests/test_ground_compute.py` - the ground compute child `main` writes the result of its session and result id.
- `tests/test_ground_verify.py` - `evaluate_pose` against the runbook tolerances (medians, a plane height, a tilted
  plane, clutter, no plane), and the verification against the fake head world plus a fake ground estimator: every pose
  passes and the summary is stored with the ground result (state fixture), a pose off the floor fails, samples while
  the head moves do not count, a missing obstacle count fails its check, a head chain applied during the verification
  ends it with an error and nothing stored, units not restarted end it with an error before the head moves, no applied chain and a
  blocked start are refused, a floor fit chain is verified without a result record, a stop restores the head.
- `tests/test_ground_api.py` - the `/api/ground-board/*` routes with fake session and verification objects and an
  inline job: the compute, result, sessions, running result, accept and state bodies against the
  `ground_board_*.json` fixtures (a plausible result without refusals against `ground_board_result.json`, an
  implausible one with its refusals against `ground_board_result_implausible.json`, and each fixture is plausible
  exactly without refusals); the sweep and verify routes and their 409 / 400 refusals; 404 for unknown sessions,
  results and a result of another kind; 409 while blocked or busy, for accept or decline of a running result, accept of
  a declined or implausible result, accept while a ground verification or session runs and decline of an accepted
  one.
- `tests/test_arm_camera_results.py` - the arm to camera compute, accept, decline and state on the synthetic arm to
  camera scene (`arm_camera_scene` of `tests/head_arm_synthetic.py`) stored as a head_arm session with an applied
  floor-fit head chain: a done, plausible result recovers the true offsets and writes the snippet; no applied head chain
  is a failed result; tightened bounds give an implausible result whose accept is refused; the compute telemetry
  (inputs, solver, one refusal event per refusal with value and limit, result); accept writes arm_calibration.json and
  the `arm_calibration` record and keeps the head chain, its record and head_chain.json unchanged (what mcp_server's
  head reader reads), stamps `accepted_at` and refuses a later decline, writes `captured_from` and a body the shared
  `read_arm_calibration` (what mcp_server applies) returns with the accepted offsets and base; the next result compares
  with the accepted one and starts from it (`applied_offsets_rad`, `applied_base`, `applied_source` in the inputs
  event), without one from the config, and an unreadable accepted file fails the compute; the accept is refused for a
  re-zero after the capture, a zero restore recorded after the capture (marker deleted) or an unreadable / timeless
  stale marker and goes through for a re-zero before the capture or a marker without joint_offsets, never writing the
  marker; a session without stills is refused; `captured_from` is the earliest stamp of any source; decline records `declined` and changes nothing applied, a declined result cannot be accepted; a failed result
  is refused by both; a running and a crashed compute read from the job state; `/state` without a head chain and the
  head chain source names; the shipped bounds; an accept after another head chain was applied is refused and the
  accepted calibration reads `head_chain_changed`; a head chain apply that starts right after the accept's applied
  chain check lands after the arm calibration, never in between; refusals come in check order (a re-zero refusal
  before a changed head chain, a declined result before anything else), like the ground board accept.
- `tests/test_arm_camera_compute.py` - the arm to camera child `main` writes the result of its session and result id,
  and a failed result without an applied head chain.
- `tests/test_arm_camera_api.py` - the `/api/arm-camera/*` routes with an inline job: 202 compute, the result, accept
  and state bodies against the contract fixtures `arm_camera_result.json`, `arm_camera_result_running.json`,
  `arm_camera_accept.json`, `arm_camera_state.json`; 404 for unknown sessions, results and a result of another kind,
  400 for an intrinsics session, 409 while a sequence or a compute runs, when not ready, for accept or decline of a
  running or failed result, accept of an implausible or declined result and decline of an accepted one; the head-arm
  routes neither list nor serve arm to camera results; `POST /board-record/clear` needs `confirm: true` (400), answers
  the sequence state, is refused (409) while an arm to camera compute runs and maps the sequencer's Busy (409) and
  SequenceError (400).
- `tests/test_app.py` - every route with a fake capture, a fake job and a tmp store: error body shape, 404/409/400/422
  mapping, capture/compute mutual refusal, min_views refusal, coverage, thumbnails `image/jpeg`, zip headers, apply,
  `/api/state` without systemctl, `/api/applied` units.
- `tests/test_main.py` - bind host and port from the config (loopback by default), shutdown stops capture, the CSI
  capture, the job and the arm to camera job and destroys the node, the arm to camera router runs its own compute job
  (`test_arm_camera_router_runs_its_own_compute_job`), a running arm to camera compute blocks a sequence and a sweep,
  the ground router runs its own compute job on the ground store, the ground session, verification and compute and the
  other head motions and computes block each other (`test_ground_parts_and_the_other_head_motions_block_each_other`),
  the CSI router is built with the CSI controller
  (`test_csi_router_is_built_and_its_capture_stopped_at_shutdown`), missing config env gives exit 78.
- `tests/test_csi_intrinsics.py` - the CSI intrinsics over synthetic 1152x648 ChArUco views fed to a fake node: the
  `csi_cameras` defaults and their validation, `view_differs`, the image subscription only while capturing and `Busy`,
  a view kept only with the board detected and different from the last kept one (`NO_BOARD`, `SAME_VIEW`), coverage,
  a reopened session, expiry, undecodable frames dropped, an unknown camera; the frame watchdog (no frame within
  `csi_frame_timeout_s` stops the capture with `NO_FRAMES` and `no_frames` true, one frame keeps it running), the
  shipped back camera disabled (not listed by `GET /api/csi`, 404 on its routes), the camera list and the state bodies
  against `csi_cameras.json`, `csi_state_running.json` and `csi_state.json` (the `cfg` fixture enables both cameras);
  compute returns K, D and RMS near the
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

The head-from-arm calibration fits the head chain against the trusted arm: the ARM ChArUco board (6x5, 28 mm, ids
15-29, see "Board roles" and `docs/markers.md`) is held in the gripper by its edge handle, the arm moves it through scripted poses, the node aims the head at it, and the
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
   takes the arm lease and moves, without the board, to `arm_sequence.base_pose` (the gripper opens to
   `insert_gripper_rad` 0.4 rad, sent once with this move).
2. Phase `waiting_board` (prompt "Seat the arm board handle in the gripper jaws up to the shoulder, then press Board
   attached"): the user slides the ARM board's grip post between the open jaws, the moving finger in the channel, until
   the finger tip stops at the shoulder, and presses Board attached (`POST .../sequence/board-attached`). The board
   record is written (see "The board record").
3. Grip close (`Sequencer.close_on_handle`, phase `moving_to_base`, status "closing the gripper on the handle"): the
   gripper alone moves to `grip_rad` (`handle_contact_rad - handle_squeeze_rad`), through the same checked move as every
   arm move. The squeeze (0.02 rad) is below the mcp_server gripper converge tolerance, so the move converges while the
   jaws still press on the handle and the target stays commanded. The effort limit is the servo `Torque_Limit` of the
   default grip profile (`move_arm_joints` has no per-call effort limit). Then the seat check: the measured gripper
   angle must be at least `handle_contact_rad - handle_squeeze_rad / 2` (0.0851 rad); a jaw that closed further
   closed past the handle (not seated up to the shoulder, or no board), and the sequence stops with "the gripper closed
   past the handle contact: seat the handle up to the shoulder and start again" (measured and limit quoted), holding the
   arm. The slack check refuses the other side: mcp_server reports a jaw that stopped short of `grip_rad` by less than
   its settle tolerance as converged too, holds that target for `arm_settle_hold_s` and then relaxes the hold to the
   measured pose, which would leave the board hanging with no squeeze. So a measured gripper angle at or above
   `grip_rad + grip_converge_tolerance_rad` (0.1051 rad) stops the sequence the same way, saying the handle is thicker
   than the CAD contact and that `arm_sequence.handle_contact_rad` must be set from the measured contact. A grip move
   that fails or does not converge, or a `get_arm_state` without a gripper angle, stops it the same way.
4. Phase `capturing`: a still session is started and the sequence waits up to `still.timeout_s` for its first colour
   camera_info (none fails the sequence, holding); tier 0 then tier 1 run.
5. Phase `waiting_big_joints`: tier 2 moves `elbow_flex`, `shoulder_lift` and `shoulder_pan`, so the user must allow it
   (`POST .../sequence/big-joints {"confirm": true}`) or skip it (`false`, or no body).
6. Tier 2 (if allowed), then phase `returning_to_base` back to the base pose, and the worker ends.

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
refused while `holding`: press Park first, or resume with the board on.

Resume with the board on (`POST .../sequence/resume-board-on {"confirm": true}`, `Sequencer.resume_with_board`):
accepted only when no sequence runs, the phase is `holding` and the board is on (`board_on` in the state); otherwise
409 naming the phase. It is for a board taped into the jaws (or simply still gripped) after a stop or an end: the jaws
must never open again before the park. A new run (new `seq-...` run id, `sequence start` event "sequence resumed with
the board on") reads the measured joints, then moves to the base pose through the same checked move (path check from
the measured joints with the board on; a refusal moves nothing and the arm keeps holding) with the gripper commanded
to its measured angle (never `insert_gripper_rad`, never more open than measured; no measured gripper angle stops it
before any move). It skips `waiting_board` and the handle grip and seat checks, then runs steps 4 to 6 above unchanged
(a new still session, tier 0 and 1, the tier-2 consent, tier 2, the return) and ends like any sequence: `holding` with
the board on, then Park. The previous session's fitted mount (or the recorded one) is kept for the board park check
until the new session has its own.

Resume from idle (`POST .../sequence/resume-board-on {"confirm": true, "board_gripped": true}`,
`Sequencer.resume_with_board(confirmed, board_gripped=True)`): the phase and `board_on` live in memory only, so a node
restart (a deploy) returns to `idle` with `board_on` false while the board is still taped or seated in the jaws. From
`idle` the resume is also accepted when the user states the board is gripped in the jaws: without `board_gripped` the
route answers 400 (the sequencer raises the same refusal); while a sequence, an intrinsics capture, a floor sweep or a
compute runs, or `blocked` names a reason, 409. Before anything moves the node reads the measured gripper angle
(`get_arm_state`) and refuses (400, nothing moved, still `idle` without the board) when it is not reported or above
`handle_contact_rad + grip_converge_tolerance_rad` (0.0951 + 0.03 = 0.1251 rad by default: the handle contact angle
plus the mcp_server gripper converge tolerance, the widest a jaw closed on the handle can read; above it the jaws are
clearly open, e.g. the half-open `insert_gripper_rad` 0.4). A pass sets `board_on`, writes the board record and logs a
`board_confirmed` calib_event ("board confirmed gripped by the user (from idle, jaw ... rad)", `from_phase` idle,
`by` user, the jaw and the limit, the restored session and mount); then the run is exactly the resume above (jaws kept
at the measured angle, no `waiting_board`, no handle grip, a new still session, the tiers and the tier-2 consent),
ending `holding` with the board on. The park check uses the same mount fallback: the fit of the session restored from a
fresh board record, else that record's mount, else the configured `fit.board_mount`. A refusal inside the run (path
check, no gripper angle) moves nothing and ends `holding` with the board on, as the user stated.

Events and errors: every sequence start, step outcome (base pose reached, board attached, handle gripped with the jaw
angle, still session started, each pose with its captured and skipped views or "not visited", the big-joint answer,
back at the base pose), sequence end (`sequence done: <status>`, `sequence stopped`, `sequence error: <error>`), a
refused start, park start, `park done: <status>`, `park failed: <error>` and a refused park is logged to the node's
journal as one `calib_event` line of flow `head_arm` carrying its message, step, outcome and phase (section Telemetry;
errors at ERROR, stops and unvisited poses at WARNING, the rest at INFO; an unexpected exception also logs its
traceback) and kept in the state's `events`: the newest `EVENT_LIMIT = 50` `{time (ISO 8601, UTC), phase (the phase the
event happened in), level (info, warning, error), message}`, newest first. One call (`Sequencer.append_event`) writes
both, so the journal never repeats a UI event. A `StepFailed` (a tool error, a path check refusal, an unseated handle, a
confirm timeout) is never only "sequence error" in the journal: the end event names the `failed_step` and why, and the
timed step's own `error` event carries its inputs. The web UI Calibrate page shows the
`error` in a banner and the events as a scrollable log (`nodes/web_ui/README.md`, "Head from arm section").

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
| `GET /state` | `{sequence, applied}`: the sequencer state (`running`, `phase`, `session_id`, `tier`, `arm_pose` (0-based), `total_poses`, `captured`, `skipped`, `status`, `error`, `big_joints`, `waiting_since`, `board_on` (the board is on the gripper: attached or stated), `board_record`, `events` (the newest 50 `{time, phase, level, message}`, newest first)) and the applied head chain (`{session, result, note, source, floor, joints}`, null when none) |
| `POST /sequence/base {"confirm": true}` | state; 409 while a sequence, an intrinsics capture, a floor sweep or a compute runs |
| `POST /sequence/board-attached` | state; 409 outside `waiting_board` |
| `POST /sequence/resume-board-on {"confirm": true, "board_gripped": false}` | state; resume the capture with the board on (gripper kept at its measured angle); 400 without the confirm, from `idle` without `board_gripped: true`, or from `idle` with open jaws (measured gripper above `handle_contact_rad + grip_converge_tolerance_rad`); 409 unless `holding` with the board on or `idle`, or while a sequence, an intrinsics capture, a floor sweep or a compute runs |
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

### Arm to camera calibration (T-99)

The arm calibrated in the head camera's frame with the head chain held FIXED (user decisions 315, 324-326, m577,
m578). The head chain is the applied one (`floor_results.base_urdf`: today the depth floor fit or a head-from-arm
result, decision 325); no head parameter is fitted and no floor session is read. Capture is the head-arm sequence
(`/api/head-arm/sequence/*`): a session here is a `head_arm` session, and its results live next to the head-arm
results in `results/<rid>/` with `kind: "arm_camera"`.

Model (`shared/ros2_common/head_rgbd_calibration/arm_camera_fit.py`, details in `shared/README.md`): board corner ->
board seat in gripper_link (the CAD `fit.board_mount` moved by a free 6-DOF seat with a soft prior) turned about the
grip axis by the gravity sag -> arm FK with the fitted offsets -> the fitted arm base in base_link -> the fixed head
camera -> projectPoints. Free: the shoulder_pan, shoulder_lift, elbow_flex and wrist_flex offsets, the arm base x, y
and z (`fit.arm_camera.fit_base_z`), the seat and the sag (and, with `fit_droop`, a droop along gravity). Gauges
(exact degeneracies, held at the applied value): the arm base yaw (a yaw turn is a shoulder_pan offset change plus a
base x, y shift: the pan axis is vertical and sits 38.8 mm off the arm root) and the wrist_roll offset unless
`fit_wrist_roll` (a wrist_roll zero change is a board turn about that axis, only the seat prior can pin it). The
tool point is the applied `fit.tool_offset_m`. The applied calibration the fit starts from and compares with is the
one mcp_server runs with (`effective_fit`): the accepted `applied/arm_calibration.json` when there is one, else
`fit.joint_offsets_rad` / `fit.base_in_base_link` (pinned equal to mcp_server's `arm:` block, the baseline). An
accepted file that cannot be read makes the compute a failed result naming it.

The compute runs in its own ComputeJob (`python -m head_rgbd_calibration.arm_camera_compute`, niced, killed after
`compute_timeout_s`; about 4 s on a laptop for 144 views), so the request returns at once and the page polls.

#### API contract

All under `/api/arm-camera` (the web UI builds against this contract; fixtures `tests/fixtures/api/arm_camera_*.json`
checked with `tests/contract.py`). Errors use the common `{ok: false, message}` body.

| Route | Answer |
|---|---|
| `POST /sessions/{sid}/compute` | 202 `{result_id}`; 404 unknown session, 400 not a head_arm session, 409 while a head-arm sequence runs, while an arm to camera compute runs, or when not ready (no applied head chain; the message is the `reason` of `/state`) |
| `GET /sessions/{sid}/results/{rid}` | the result (below); 404 unknown or malformed ids or a result of another kind |
| `POST /sessions/{sid}/results/{rid}/accept` | `{arm_calibration: {session, result, accepted_at, score}, units: [{name, active_state, restarted}]}` (`units`: the `restart_units` states as every apply reports them; no unit reads arm_calibration.json, so they show `restarted: false`; mcp_server applies it without a restart); 409 when the result is not done (running or failed), declined, not plausible (the message lists the refusals), captured before the last arm re-zero (or the re-zero time is unknown), not a valid arm calibration, or fitted against a head chain that is no longer the applied one (checked in this order, like the ground board accept); 404 as above |
| `POST /sessions/{sid}/results/{rid}/decline` | the result with `declined: true`, `declined_at`; nothing applied changes; 409 when not done or already accepted |
| `GET /state` | `{applied: {session, result, accepted_at, score, accuracy_examples, head_chain_changed} \| null, head_chain_source, ready, reason}` (`head_chain_changed`: the applied head chain is no longer the one the accepted calibration was fitted against); `head_chain_source` is `floor_fit`, `head_arm`, `arm_offsets` (the applied.json head_chain record `source` floor, head_arm, arm), null for an applied chain without a matching record, `urdf` without an applied chain (then `ready` false and `reason` "apply a head calibration first ...") |
| `POST /board-record/clear` | body `{confirm: true}`: the user states the board is already off the gripper; removes the board record, moves nothing, answers the head-arm sequence state (`sequence.board_record` then null); 400 without the confirm, 409 while a head-arm sequence or park or an arm to camera compute runs (see "The board record") |

Result body (result.json; while the job runs and after a child that wrote nothing it is built from the job state,
which keeps only the last compute: a crashed result reads 404 once the next compute starts):

```
{id, session, kind: "arm_camera", state: "running" | "done" | "failed", error, created_at,
 declined, declined_at, accepted_at,
 head_chain: {source, session, result} | null,     # the fixed head chain the fit used
 report: {...} | null,                              # done results only
 calibration: {joint_offsets_rad: {5 joints}, base_in_base_link: {x, y, z, yaw}, tool_offset_m, snippet} | null}
```

Report fields (`arm_camera_fit.arm_camera_report`):

- `plausible` (bool) and `refusals`: physically implausible results are refused outright and cannot be accepted
  (decision 324). Each entry `{name, value, limit, unit, message}`; names `offset_change:<joint>` (deg, any joint offset
  change over `bounds.max_offset_change_deg` 15), `base_move_mm` (100), `base_turn_deg` (10), `skew_mm` (10),
  `skew_deg` (10), `sag_deg` (the sag at the reference torque, 5), `droop_mm` (5, with `fit_droop`), `non_finite`.
- `warnings`: today's pass/fail limits as advice only (m578), same entry shape: `rms_px`, `holdout_rms_px` and
  `holdout_tool_mm` (`fit.arm_limits`), `min_views` (`fit.arm_limits`), `min_arm_poses` (`fit.limits`), `no_holdout`.
- `score`: `{rms_px, heldout_rms_px, heldout_tool_mm, views, poses}` (held-out: every `fit.limits.holdout_every`-th
  arm pose with all its views; the tool error is the arm FK tool point against the view's solvePnP board pose).
- `params`: `[{name, value, sigma, held, unit, applied_value, change}]` in plain units: `<joint>_offset` (deg), `base_x`,
  `base_y`, `base_z` (mm), `base_yaw` (deg), `board_x`, `board_y`, `board_z` (mm, the seat in gripper_link; the applied
  value is the CAD mount, the change equals `skew.xyz_mm`), `board_skew_roll`, `board_skew_pitch`, `board_skew_yaw`
  (deg, the seat turn against the CAD mount in the board frame, equal to `skew.rpy_deg`; applied value 0), `sag`
  (deg/(N m)), `droop` (mm/(N m), only with `fit_droop`).
  `held` is true and `sigma` null for a fixed parameter (a gauge); a fitted parameter whose sigma is not finite (a
  singular covariance) also has `sigma` null, with `held` false. Non-finite numbers are null everywhere in the result,
  including `calibration` (a `non_finite` fit stays strict JSON, so its result can be read and declined).
- `accuracy_examples`: `[{label, value, unit: "mm"}]`, two per example (`<example>, 1 sigma` and `<example>, 95 %`) for
  "tool point at 30 cm reach", "tool point at full reach" and "gripper position seen by the head camera at 50 cm",
  the parameter covariance propagated to the tool point in base_link (1 sigma: RMS 3D error; 95 %: chi-square radius
  of the largest principal axis, an upper bound).
- `comparison`: `{applied, current_score, shift_examples}`: `applied` the accepted arm to camera calibration in effect
  (`{session, result, accepted_at, score, accuracy_examples}`, null when none), `current_score` the configured
  (applied) offsets and base scored on this session with only the seat and sag refitted, `shift_examples`
  `[{label, value, unit}]` how far the new calibration moves the tool point at each example pose (mm).
- `skew`: `{xyz_mm, rpy_deg}` the fitted board seat against the CAD mount (printing skew, seating tolerance).
- `sag`: `{deg_per_nm, sigma_deg_per_nm, reference_torque_nm, at_reference_deg, range_deg, droop_mm_per_nm,
  sigma_droop_mm_per_nm, droop_at_reference_mm}` (droop keys null without `fit_droop`).
- `notes`: the gauge and model notes (head fixed, yaw gauge, wrist_roll gauge or prior, base z fixed, tool point).

Accept applies the calibration to the robot; Decline keeps the one in use. Accept writes
`applied/arm_calibration.json` (`{session, result, accepted_at, score, joint_offsets_rad, base_in_base_link,
tool_offset_m, snippet, captured_from, head_chain, accuracy_examples}`; `captured_from` is the session's earliest still,
the body is validated as ros2_common `ArmCalibrationFile` before the write) and then the `arm_calibration` record of
`applied/applied.json` through `apply.update_applied`, the writer of every apply (the data file first, applied.json
last: the commit point); the check that the head chain the result was fitted against is still applied runs under the
same applied lock as the write. The head chain, its record and the D435 check record stay as they are, and `accepted_at` is
stamped into result.json. mcp_server (`arm.applied_dir`, the same `applied/`) sees the applied.json change on its
0.25 s monitor tick and swaps the accepted joint offsets, arm base and mount height (`arm_base_height_m` = base z)
into the running arm model at its next idle point (no arm motion running); no unit restarts (the apply path unit
restarts the head nodes only when a file they read changed) and no client.yml edit is needed. The client.yml `arm:`
values stay the baseline mcp_server falls back to when nothing is accepted. The tool point is not fitted and stays
the configured one. See `nodes/mcp_server/README.md` ("Accepted arm calibration").

Arm zero: the accept is refused (409) when the servo zero may have changed at or after `captured_from` (the earliest
stamp of any source of any still: colour, head and arm joints), read by ros2_common `zero_change_refusal` from the
follower bridge's stale marker (`arm_stale_marker`) and its zero change record (`zero_changed.json` next to it, the
time of the last neutral save, zero restore or feetech_servos `calibrate_servos.py` zero change, never cleared): the marker lists `joint_offsets` for a re-zero at or
after the capture, the record names a zero change at or after it (a zero restore deletes the marker but not the
record), either is unreadable, or the marker lists `joint_offsets` without a timezone-aware `saved_at`. Times count
within one second of the capture as after it (the bridge writes whole seconds). Capture a new session then. The accept
only reads these files; mcp_server clears `joint_offsets` from the marker when it applies a calibration that passes the
same check (one writer).

`snippet` / `joint_offsets_snippet.yaml` (`joint_offsets_rad`, `arm_base_height_m`, `base_in_base_link`) stay for
review and for moving an accepted calibration into the client.yml baseline (with `fit.joint_offsets_rad` /
`fit.base_in_base_link` of this node, filter_node's `self_collision_guard.arm` and web_ui's `arm_calibration`, the
baselines of the runtime consumers, pinned equal to mcp_server's by `tests/test_arm_guard_config.py`); the runtime override
sets exactly these keys (pinned by `tests/test_arm_calibration_file.py`). The head-arm routes
(`/api/head-arm/sessions`, `.../results/{rid}`) neither list nor serve arm to camera results. While an arm to camera
compute runs, a head-arm sequence and a floor sweep do not start.

Telemetry: see the `arm_camera_compute` and `arm_camera_accept` rows of the Telemetry table.

Config `fit.arm_camera` (`ArmCameraFitConfig`, every key in the shipped YAML): `fit_base_z` (true), `fit_wrist_roll`
(false), `fit_droop` (false), `pixel_sigma_px` (1.0), priors `board_prior_mm` 3.0, `board_prior_deg` 2.0,
`sag_prior_deg` 1.0, `droop_prior_mm` 1.0, the ARM board mass model `board_mass_kg`, `board_com_m`, `grip_point_m`,
`grip_axis` (from `markers/charuco-arm-6x5-28mm.json`, pinned by test), the accuracy example settings `reach_m` 0.3
and `camera_distance_m` 0.5, and `bounds` (the refusal bounds above).

### Config keys of the head-from-arm calibration

| Key | Default | Meaning |
|---|---|---|
| `topics.head_joint_states`, `topics.arm_joint_states` | `/head/joint_states`, `/follower/joint_states` | Measured joints of the still capture |
| `still.window_s`, `rgb_mad`, `joint_still_rad`, `max_joint_dt_s`, `timeout_s` | 0.6, 2.0, 0.002, 0.1, 6.0 | The still rule |
| `arm_urdf_file` | deployed checkout `so101_arm.urdf` | Arm URDF for the fit and the pose check |
| `fit.*` | see "Config keys" | Board, board mount start, pan axis, limits, Huber scale, arm gauge, joint offsets, floor prior weight |
| `robot.mcp_url`, `token_file`, `board_park_token_file`, `call_timeout_s`, `move_timeout_s` | `http://127.0.0.1:18200/mcp`, `/etc/ros2/mcp_server/token`, `/etc/ros2/mcp_server/board_park_token`, 60, 120 | The mcp_server the node moves the robot through |
| `arm_sequence.*` | see "Config keys" | Base pose, insert opening, handle grip, board mass, the three tiers, speeds, confirm timeout, head offsets, pose check, margins, park |

`board` and `fit.board` / `fit.plate` must use disjoint marker ids (checked at load, see "Board roles"). Unknown keys
are rejected.

## Ground board calibration (camera to ground, T-99)

The head chain solved from the big FULL ChArUco board (`charuco-6x5-40mm`, 40 mm squares) lying flat on the floor
(user decisions 312, 313, 324, 326). The board may lie anywhere close to the robot, as long as it is flat on the
floor and fully visible in the camera preview with the head homed; nothing else about its position is assumed. The
user may move it between 1 and `ground_board.max_placements` (3) placements in one session; each placement has its own
unknown x, y and yaw. The runbook is `docs/runbooks/head-ground-board.md`.

### Capture (ground session)

`GroundSweep` (`head_rgbd_calibration/ground_sweep.py`, a `FloorSweep` subclass: the head moves only through the
floor sweep's command path, `floor.command_topic`, with its reach, stillness and timeout settings; nothing else is
published). Per placement:

1. The head goes home (`ground_board.fit.home_pan_deg` 0, `home_tilt_deg` 30: mcp_server's head home) and waits for
   the colour camera_info. The board must show at least `home_min_corners` (16 of 20) corners there, else the placement
   fails with a message (`board not detected at home ...`) and the session awaits the next placement.
2. The board is located on the floor from the home view: every corner's ray through the APPLIED head chain meets the
   board plane (z = board thickness), then a 2D rigid fit of the board corners to those points
   (`ground_board_fit.locate_board`; no PnP, so no planar flip at the oblique views).
3. The head aims at the board centre (`aim_head`) and `plan_grid` builds the grid of `pan_offsets_deg` x
   `tilt_offsets_deg` around the aim (serpentine, tilt clamped to `min_tilt_deg`..`max_tilt_deg`, at most the 45 deg
   head_tilt limit), keeping only poses whose predicted board stays `image_margin_px` inside the image; the dropped
   count is logged.
4. At home and at each grid pose, once the head is reached and still and `floor.settle_s` passed, up to
   `frames_per_pose` colour frames with at least `min_corners` corners are kept (`detect_frame`: the detection is
   retried at `detect_upscale` times the size when too few corners come back; oblique 30 mm markers are about 13 px
   tall at 640x360). Each view keeps the measured head angles nearest its stamp (within `floor.max_stamp_dt_s`, at the
   target within `floor.reach_tol_deg`), the received JPEG (`frames/<view_id>.jpg`) and the aligned depth bias against
   the board plane (`calibrate.board_plane_bias_mm`, the PnP seeded from the located board: no flip). Then the depth
   floor planes of `floor.clouds_per_pose` clouds are fitted as the floor sweep does (the fit's cross-check). A pose
   with fewer than `min_frames_per_pose` frames is skipped with the frame drop reasons.
5. The head returns home and the session awaits the next placement (`POST /sweep/next-placement` after the user moved
   the board) or its end (`POST /sweep/stop`, or `placement_wait_s` without the next one). After
   `max_placements` placements it ends by itself. Every exit commands the head back to where it was at start.

State directory:

```
ground_board/<sid>/observations.json        session, camera_info, home, placements (located board, aim, grid,
                                            counts), placement_failures, views, observations (depth planes),
                                            skipped poses, dropped clouds
ground_board/<sid>/frames/<view_id>.jpg      colour JPEG of each kept view
ground_board/<sid>/results/<rid>/            result.json, head_chain.json and head.urdf (done results),
                                            verification.json (after a verification of the applied result)
```

Session id `ground-YYYYmmdd-HHMMSS` (`-N` when that second is taken); view id `p<placement>-<pose>-<frame>`, pose
indices unique over the session.

### Fit

`shared/ros2_common/head_rgbd_calibration/ground_board_fit.py` (details in `shared/README.md`): board on the floor at
z = thickness, face up, per placement free x, y, yaw; free head chain: the mount z, roll, pitch, the tilt zero and tilt
roll, the D435 mount x, z, roll, yaw and the pan axis x, y. Gauge (decision 326): the head mount x, y and yaw stay at
their applied values (a free board cannot tell a head shift or turn in the floor plane from a board shift or turn).
Huber least squares on the corner pixels; every `limits.holdout_every`-th head pose is held out. The depth floor
planes are a separate cross-check (pose-mean height and normal RMS under the fitted and the applied chain), not
residuals: the depth carries its own structured bias that would leak into the colour chain.

The compute runs in its own ComputeJob (`python -m head_rgbd_calibration.ground_compute` on a `GroundStore`, niced,
killed after `compute_timeout_s`; about 5-15 s on a laptop for 69 views; a full session of 3 placements at 3 frames per pose has some 200+ views and takes several times that, more on the Pi 5, under `compute_timeout_s`), so the request returns at once and the page
polls.

### Accept, decline and verification

- Accept (only a done, plausible, not declined result fitted against the head chain still applied, checked under the
  applied lock with the write, so no other apply lands in between) copies the
  result's head_chain.json (the applied origins merged with the fitted `head_mount_joint`, `head_tilt`, `head_pan` and
  `d435_joint`) into applied/ through `floor_results.apply_head_chain_file`, the one writer of the applied head chain:
  the applied.json `head_chain` record is `{session, result, applied_at, source: "ground_board", floor: null}` and the
  apply path unit restarts `restart_units`. `floor: null`: no floor fit result checked this chain, so a later
  head-from-arm fit has no floor cross-check until a floor fit is applied. The arm to camera API names the source
  `ground_board`.
- Decline records `declined: true` and `declined_at` in result.json; nothing applied changes.
- Verification (decision 313, `GroundVerify` in `ground_verify.py`, `POST /verify/start` after an accept): waits until
  every `restart_units` unit is active and restarted since the head chain's `applied_at` (up to `restart_check_s`),
  then visits `ground_board.verify.poses_deg` (home, pan 40 / tilt 20, pan -40 / tilt 25: the runbook poses) and at
  each, once the head is still and `settle_s` passed, collects /ground/state (ground_estimator_cpp, RELIABLE
  TRANSIENT_LOCAL) documents with `head_moving` false for `window_s` (up to `sample_timeout_s` until `min_samples`).
  A pose passes when the median plane height is within `max_height_m` (0.01 m) of 0, the median `plane_vs_base_deg`
  under `max_plane_vs_base_deg` (0.6), the median `obstacle_points` under `max_obstacle_points` (10) and the median
  `drop_points` at most `max_drop_points` (0) (`docs/runbooks/head-floor-fit.md` acceptance checks; a sample without
  `obstacle_points` or `drop_points` makes that check fail, a missing count is not a zero). When the applied head
  chain changes during the verification it ends with an error and stores nothing. Every exit
  commands the head back. When the applied chain is a ground board result, the summary is stored with it
  (verification.json and result.json `verification`).

Mutual exclusion: a ground session or verification does not start while a capture, a D435 compute, an arm to camera
compute, a ground compute, a head-arm sequence, a floor sweep or the other one runs; a ground compute does not start
while a ground session, a verification, a floor sweep, a head-arm sequence or an arm to camera compute runs; a floor
sweep and a head-arm sequence do not start while a ground session, verification or compute runs; an arm to camera
compute does not start while a ground session, verification or compute runs (two heavy fit children never run at
once).

### API contract

All under `/api/ground-board` (the web UI builds against this contract; fixtures `tests/fixtures/api/ground_board_*.json`
checked with `tests/contract.py`; the session and verification state fixtures against the real objects). Errors use
the common `{ok: false, message}` body.

| Route | Answer |
|---|---|
| `GET /state` | `{sweep, verify, compute, head_chain, accepted}`: `sweep` as below, `verify` as below, `compute` the ground job state `{running, session_id, result_id, started_at, error}`, `head_chain` the head chain in effect `{source, session, result}` (`source` `floor_fit`, `head_arm`, `arm_offsets`, `ground_board`, null, or `urdf` without one), `accepted` the ground board result whose chain is applied `{session, result, accepted_at, score, accuracy_examples, verification}` or null (`verification` null until a verification of it ran, then the verification state without `running`) |
| `POST /sweep/start` | the session state; 409 while a session or anything above that blocks it runs, 400 when it cannot start (no head joint state: nothing moved; unreadable URDF) |
| `POST /sweep/next-placement` | the session state; 409 when no placement is awaited |
| `POST /sweep/stop` | the session state (a running placement stops and keeps its views; an awaited one is not taken) |
| `GET /sessions` | `[{id, created, placements, views, planes, skipped, results}]`, newest first |
| `POST /sessions/{sid}/compute` | 202 `{result_id}`; 404 unknown or malformed session, 409 while blocked (message names what runs) or while a ground compute runs |
| `GET /sessions/{sid}/results/{rid}` | the result (below); 404 unknown or malformed ids or a result of another kind |
| `POST /sessions/{sid}/results/{rid}/accept` | `{head_chain: {session, result, applied_at, source: "ground_board", floor: null}, units: [{name, active_state, restarted}]}`; 409 when not done, declined, already accepted, not plausible (the message lists the refusals), without an accepted D435 intrinsics check, or fitted against a head chain that is no longer applied (checked in this order, like the arm to camera accept), or while a ground verification or ground session runs |
| `POST /sessions/{sid}/results/{rid}/decline` | the result with `declined: true`, `declined_at`; 409 when not done or already accepted |
| `POST /verify/start` | the verification state; 400 without an applied head chain, 409 while blocked |
| `POST /verify/stop` | the verification state (the head returns) |
| `GET /verify/state` | the verification state |

Session state: `{running, status, session_id, placement, placements_done, max_placements, awaiting_placement, pose,
total, views, planes, skipped, board, message, error}`; `status` is `idle`, `starting`, `homing`, `locating`,
`sweeping`, `awaiting_placement`, `done`, `stopped` or `error`; `placement` the 0-based placement being captured (the
next one while awaiting), `pose` / `total` the 1-based pose of its grid (home included), `board` the located board
`{x_m, y_m, yaw_deg}` of the current placement (null before), `message` the operator hint (the failure reason of a
placement, or "move the board ...").

Verification state: `{running, status, id, head_chain, result, pose, total, poses, passed, error}`; `status` is
`idle`, `starting`, `waiting_for_restarts`, `measuring`, `done`, `stopped` or `error`; `result` the ground board result
`{session, result}` the summary is stored with (null for another chain); `passed` null until every pose was judged;
each `poses` entry `{pose, pan_deg, tilt_deg, reached {pan_deg, tilt_deg} | null, samples, plane_samples, height_m,
plane_vs_base_deg, obstacle_points, obstacle_points_max, drop_points, drop_points_max, checks [{name, value, limit,
ok}], passed, error}` (medians; `checks` names `samples`, `plane`, `height_m`, `plane_vs_base_deg`, `obstacle_points`,
`drop_points`; `error` "did not reach pan/tilt ..." or null).

Result body (result.json; while the job runs and after a child that wrote nothing it is built from the job state):

```
{id, session, kind: "ground_board", state: "running" | "done" | "failed", error, created_at,
 declined, declined_at, accepted_at,
 head_chain: {source, session, result} | null,     # the applied chain the fit started from (the gauge)
 report: {...} | null,                              # done results only
 calibration: {joints: {head_mount_joint, head_tilt, d435_joint, head_pan: {xyz, rpy}}, note} | null,
 verification: {...} | null}                        # the verification state of this result once verified
```

Report fields (`ground_board_fit.ground_board_report`):

- `plausible` and `refusals` (decision 324: refused outright, cannot be accepted), each `{name, value, limit, unit,
  message}`: `mount_move_mm` (head mount z change over `bounds.max_mount_move_mm` 100), `mount_turn_deg` (mount roll or
  pitch over 10), `tilt_change_deg` (tilt zero or roll over 10), `camera_move_mm` (D435 mount x, z over 30),
  `camera_turn_deg` (D435 mount roll or yaw over 10), `pan_axis_move_mm` (50), `board_distance_m:<placement>` (a board
  farther than 2.5 m from the head mount), `non_finite`, `accuracy_unknown` (a sigma or accuracy example is not
  finite: the column-scaled Jacobian condition is over 1e8, the parameters are not all observable from the views).
- `warnings` (advisory, the user decides), same shape: `rms_px` (2.0), `holdout_rms_px` (3.0), `max_sigma_mm` (2.0),
  `max_sigma_deg` (0.2), `min_views` (30), `min_poses` (12), `min_placements` (2), `min_pan_span_deg` (20),
  `min_tilt_span_deg` (10), `floor_height_rms_mm` (8) and `floor_normal_rms_deg` (1.0) (the depth cross-check under the
  fitted chain), `no_depth_planes`, `no_holdout`.
- `score`: `{rms_px, heldout_rms_px, views, poses, placements, floor_height_rms_mm, floor_normal_rms_deg}`.
- `params`: `[{name, value, sigma, unit, applied_value, change}]` in mm and deg for every head parameter of the model:
  `mount_x_m`, `mount_y_m`, `mount_yaw_deg` (the gauge: sigma null, change 0), `mount_z_m`, `mount_roll_deg`,
  `mount_pitch_deg`, `tilt_zero_deg`, `tilt_roll_deg`, `camera_x_m`, `camera_z_m`, `camera_roll_deg`,
  `camera_yaw_deg`, `pan_axis_x_m`, `pan_axis_y_m` (the names keep the head_arm_fit suffixes; the unit field is the
  reported unit).
- `placements`: `[{placement, x_m, y_m, yaw_deg, sigma_x_mm, sigma_y_mm, sigma_yaw_deg, views, rms_px}]`.
- `accuracy_examples`: `[{label, value, unit}]`, two per example (`, 1 sigma` and `, 95 %`, 1.96 sigma): "floor height
  1 m ahead of the head" (mm), "floor height 2 m ahead of the head" (mm), "obstacle position 2 m ahead, lateral (head heading as applied, not calibrated here)" (mm),
  "obstacle position 2 m ahead, range" (mm), "head pitch (optical axis to the floor)" (deg), "head roll (image horizon
  to the floor)" (deg): the head parameter covariance propagated to a floor measurement in the depth sensor frame at
  the home pose. They leave out the gauge: the head x, y and heading are not calibrated by this fit (kept as applied), so an error in the applied heading (0.5 deg is 17 mm lateral at 2 m) is not in the lateral example; its label says so.
- `comparison`: `{applied, current_score {rms_px, heldout_rms_px}, shift_examples [{label, value, unit}]}`: `applied`
  the head chain the fit started from (`{source, session, result}`, null for the repo URDF), `current_score` that
  chain scored on this session (placements refitted), `shift_examples` how far the new chain moves each example
  quantity.
- `cross_check`: `{planes, poses, fitted {height_rms_mm, normal_rms_deg}, applied {...}}`.
- `poses`: `[{pose, placement, pan_deg, tilt_deg, views, rms_px, held_out}]`.
- `notes`: gauge, camera mount, pan axis, cross-check and example notes.

Telemetry: the `ground_sweep`, `ground_compute`, `ground_accept` and `ground_verify` rows of the Telemetry table.

Config `ground_board` (every key in the shipped YAML and in client.yml): `pan_offsets_deg` [-24, -12, 0, 12, 24],
`tilt_offsets_deg` [-12, -6, 0, 6, 12], `min_tilt_deg` 0, `max_tilt_deg` 45, `image_margin_px` 10,
`frames_per_pose` 3, `min_frames_per_pose` 2, `min_corners` 12, `home_min_corners` 16, `detect_upscale` 2,
`frame_timeout_s` 3, `max_placements` 3, `placement_wait_s` 1800; `fit` (`GroundBoardFitConfig`: the FULL board,
`fit_camera_mount`, `fit_pan_axis`, `huber_f_scale_px`, `near_m` 1, `far_m` 2, the home pose, `limits` and `bounds`
above); `verify` (`state_topic` /ground/state, `poses_deg`, `settle_s` 3, `window_s` 15, `min_samples` 10,
`sample_timeout_s` 60 and the four tolerances above).

## Park with the board on (design D63)

`POST /api/head-arm/park {"confirm": true, "board_on": false}` works with the calibration board still on the gripper, so
a retry after a refusal needs no board removal. The board counts as on when this sequence attached it, or the user
states it now (`board_on: true`, the Calibrate page button "Park (board is on the gripper)" in the idle state). The
sequencer reads `get_control_state`:

- Board not on: the plain `release_control` park; a holding load is refused as ever, with a hint to state the board.
- Board on (whatever the load reads): the stored home is read from `arm_home_file` (mcp_server's `home.yaml`, fresh at
  each press, so a re-stored home is the next target; the five arm joints, the gripper entry ignored). A missing or
  invalid file refuses naming the file ("store the arm home (Calibrate > General, Set home) first") and nothing moves.
  The path from the measured joints to that home is checked with `PoseChecker` and the board outline of the newest fitted `board_mount` of the session
  (`head_arm_results.latest_board_mount`), else the recorded mount, else `fit.board_mount`, with
  `park_board_sag_margin_m` added to the board floor (`min_board_height_m`), body and head clearances. A violation
  refuses with the named rules ("Park refused by the path check with the board on ...") and the arm keeps holding. A pass
  moves to the stored home through the guarded `move_arm_joints` at `arm_sequence.speed_scale` (0.1), then calls mcp_server's
  internal `POST /admin/park_calibration_board` with the board mount as the only content (`robot.board_park_token_file`,
  its own token). mcp_server owns the outline and margins, refuses a mount over 30 mm / 10 deg from its configured one,
  requires the measured arm at its rest pose (the same stored home), FK-checks the board on every
  home leg from the measured joints against the floor and the body, and parks at 0.1 (see its README). The
  agent-facing `release_control` has no such argument and claude_agent cannot read the board-park token.
- Any carry reason other than the holding load (a held grasp, a carried radius) refuses with mcp_server's reasons quoted.

### The board record

When the board is attached (and again when the sequence ends, with the capture session id) the sequencer writes
`<state_dir>/head_arm_board.json` (`session_id`, `mount` used, `clipped_at`: the time the board was attached; the key
keeps its name). A completed park, a new base move or a clear (below) removes it. mcp_server refuses `/arm/home` while
the file exists, at any age, so `sequence.board_record` always shows it while it exists (None without a file):
`{file, session_id, clipped_at, age_s, offered, problem}`. `offered` is true when the record would pre-fill a board
park: younger than `arm_sequence.board_record_max_age_s` (7200 s) and of a stored head_arm session; else `problem` says
why ("too old", "not a stored head_arm session", or "unreadable: <error>" with the other fields null). A node start
restores the session and mount of an offered record only, and logs an ignored one. The record never switches the board
on: after a restart the user must state that the board is on (`board_on: true`, the idle "Park (board is on the
gripper)" button) before a board park, and the path checks still run. Without that statement a gripped board is
refused.

When the board is already off the gripper (taken off by hand without a park), `POST /api/arm-camera/board-record/clear`
with `{confirm: true}` removes the record (`Sequencer.clear_board_record`): nothing moves, a `holding` phase returns to
idle (the board is off, so Park is the plain park), the status and a recent event say "board record cleared: the user
stated the calibration board is off the gripper (nothing moved)", and the journal gets a calib_event of step
`board_record_clear` (outcome `cleared`, or `none` without a record) under a `board-...` run id with the removed record.
It answers the head-arm sequence state; 400 without the confirm, 409 while a head-arm sequence or park (or its worker
thread) or an arm to camera compute runs. The Calibrate page offers it as "The board is already off the gripper: clear
the record" behind a confirm, in the Arm to camera flow and in the General section.
