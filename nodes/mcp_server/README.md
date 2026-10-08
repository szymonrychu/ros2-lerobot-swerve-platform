# mcp_server

Robot MCP server for LLM agents (Claude Code and other MCP clients). One rclpy node (`mcp_server`) spun by a
`MultiThreadedExecutor` in a background thread, plus an MCP **Streamable HTTP** app (official MCP Python SDK 2.x,
`mcp.server.mcpserver.MCPServer`) served by uvicorn. Runs natively on the client RPi 5 (`ros2-mcp_server.service`).

- Endpoint: `http://client.ros2.lan:18200/mcp` (host/port/path from config)
- Auth: static bearer token from the environment variable `MCP_SERVER_TOKEN` (the server refuses to start without a
  token of at least 24 characters); verified in constant time by `StaticTokenVerifier` (`Authorization: Bearer ...`)
- Admin route: `POST /admin/clear_agent_pois` (plain HTTP, same bearer token, NOT an MCP tool so the model cannot call
  it). It sends the poi_store `clear` command for `created_by: "agent"` and returns poi_store's answer:
  `200 {"ok": true, "removed": <count>}`; `401` without or with a wrong token; `503` when poi_store is not running;
  `504` when it does not answer within `poi.request_timeout_s`; `502` when it rejects the command. claude_agent calls it
  on "New session" because it runs as its own Linux user and DDS data from that user does not reach the robot-user
  nodes (poi_store), while this node does.
- Config: YAML at `MCP_SERVER_CONFIG` (default `/etc/ros2/mcp_server/config.yaml`), validated with pydantic
  (`mcp_server/config.py`; unknown keys are rejected, motion limits have hard caps)

## Tools

| Tool | What it does |
|---|---|
| `get_robot_state` | map -> base_link pose (TF), odometry twist, latest Nav2 goal status, collision monitor action (if published), arm joints/efforts, gripper effort, filter_node active source, lease state, data age per source. Stale data is omitted and listed in `notes`. |
| `get_camera_image(camera, max_px<=1024)` | Waits for the next frame on a persistent per-camera subscription (created at startup, never destroyed: per-call subscriptions raced the executor and killed it), with timeout. `gripper`: `/camera_0/image_raw/compressed` (JPEG passed through or downscaled once); `front`: `topics.front_camera` (default `/overview_camera/image_raw/compressed`, 640x480 overhead Camera Module 3 looking down at the front of the robot, the arm and the floor in front; JPEG passed through or downscaled once, best view for judging gripper-to-object position). Returns MCP image content (JPEG) + capture stamp; error if no frame within `image_timeout_s` or the frame is older than 1 s. |
| `get_body_state` | Body vitals (sensor, always allowed): per-servo latest temperature / load / current / voltage / status flags with data age, hottest servo, battery V, per-cell V, margin to cut-off and cut-off state, IMU roll/pitch/tilt and last bump, wheel slip residual, commanded vs measured base speed, CPU temperature and firmware throttling flag, active source + lease, last 10 events. Missing data is `null` with a reason in `notes`. |
| `get_map_summary(include_png, radius_m)` | Nearest `/scan_filtered` obstacle in 8 sectors around base_link, `/map` size and known/occupied/free cells, robot pose, optional small PNG of the map around the robot. |
| `navigate_to_pose(x, y, yaw, frame='map', timeout_s)` | Nav2 `NavigateToPose`; blocks until result/timeout (goal cancelled on timeout or stop); returns result and final pose. The description states the goal precision from `nav.goal_xy_tolerance_m` / `nav.goal_yaw_tolerance_deg` (default 1 cm / 2 deg, keep equal to the Nav2 goal checker) and that a sideways goal first turns the robot toward the path (front leading) and turns back to the goal heading at the end. |
| `move_relative(dx, dy, dyaw)` | Same, goal given in base_link (converted to a map goal via TF). Small moves (a few cm) really move. |
| `drive(vx, vy, wz, duration_s<=2)` | 20 Hz on `/cmd_vel_nav` (through velocity smoother + collision monitor), clamped to 0.25 m/s / 0.5 rad/s, then zero. |
| `stop` | Always available: cancels all NavigateToPose goals and publishes a zero twist. Aborts any arm motion and holds the arm at its measured pose only if this server holds arm control (or a motion is running); otherwise the arm is not touched (`arm_held: false`). |
| `get_arm_state` | Joint positions (measured follower values)/efforts, gripper effort, tool point pose (x, y, z, pitch; forward kinematics with `arm.joint_offsets_rad` applied), `floor_z_m` (floor height in the base_link frame), active source, lease, home stored. |
| `acquire_control` / `release_control` | Start the autonomy lease (publish the measured pose on `/filter/autonomy_joint_commands`) / end it (`std_msgs/Bool` true on `/filter/autonomy_release`). The lease is sticky: release it explicitly when done. |
| `move_arm_joints(targets, speed_scale<=0.5)` | Interpolated motion to joint targets (follower joint radians, as in `get_arm_state`). Unnamed joints keep their last commanded target. `speed_scale` 0.5 is the maximum, `limits.arm_max_joint_velocity_rps` (default 1.0 rad/s); the description states the configured value. A `wrist_roll` change above `limits.roll_guard_min_change_rad` is refused while the gripper is open wider than `limits.roll_max_gripper_open_rad` (see Wrist roll guard). `converged` results may carry `residual_error` (see Safety model). |
| `move_arm_cartesian(x, y, z, pitch=None, frame='base_link', speed_scale, wrist_roll=None, object_width_m=None)` | ikpy IK on `nodes/web_ui/urdf/so101_arm.urdf` (5-DOF: position + approach pitch); `wrist_roll` (rad, measured space, clamped to the limits; clamping is listed in `clamped`) is the roll the IK keeps for this target and the motion rolls to, omitted = the current roll is kept; `object_width_m` (m, 0 < w <= 0.08) makes (x, y, z) the OBJECT CENTRE (see Grasp shift); `unreachable` is reported, never guessed. `base_link` here is the arm URDF root (arm mount, z = 0). The floor is at `z = -arm.arm_base_height_m` (default 0.165, measured 16.5 cm); the tool descriptions and `get_arm_state.floor_z_m` state it. No motion restriction is derived from it. |
| `set_gripper(open_fraction | close_until_effort, effort_threshold)` | Open to a fraction (0 closed, 1 open) or close slowly until `abs(effort) >= threshold` (then hold: `grasped`, else `closed_no_contact`). A stall or effort contact only counts as `grasped` when the jaw closed at least `limits.gripper_grasp_min_travel_rad` (0.15 rad) from where it started AND stopped no more open than `limits.gripper_grasp_max_open_rad` (1.2 rad; a nearly open jaw that stalls is pushing on something); otherwise the status is `blocked` with "jaw stopped at X rad after Y rad travel - likely pressing on an object rather than holding it", and the measured jaw position is held (no squeeze). `arm.gripper_closed_rad` / `gripper_open_rad` are follower gripper joint positions (defaults -0.165 / 1.5 rad; measured fully closed is -0.172 rad, URDF lower limit -0.1745, so the jaws close fully). With `close_until_effort` the load is ignored for `gripper_effort_ignore_s` (0.3 s, motor start-up spike) and then counts only once the jaw moved `gripper_contact_travel_rad` (0.03) or stalled. The arm joints keep being held at their intended targets while the gripper moves (see Safety). |
| `arm_home` / `arm_set_home` | Move to / store the home pose. `arm_home` keeps arm control afterwards only if it was already held before the call; otherwise it releases it. |
| `pixel_to_ground(camera, u, v, surface_height_m=0.0)` | Point seen at a pixel (sensor): `surface_height_m`, `ground_base_link`, `ground_map` (when the map pose is known), `distance_from_base_m`, `bearing_deg`, `method`, `uncertainty_note`. `surface_height_m` (-0.5..0.5) is the height of the surface the pixel lies on relative to the robot's floor (positive above, negative below; top of a 3 cm box 0.03, a floor 10 cm lower -0.10): the ray is intersected with the plane at floor + that height and the returned z is on it. Error `camera <name> not calibrated: ...` until intrinsics and mount are configured. See [Camera tools and calibration](#camera-tools-and-calibration). |
| `get_annotated_camera_image(camera, overlays=['grid'], planned_gripper, grid_step_m=0.1)` | JPEG with metric overlays (`grid`, `reach`, `gripper`, `planned_gripper`, `lidar`) plus metadata. |
| `mark_candidate_points(camera, region, spacing_px=40, max_points=40, surface_height_m=0.0)` | Image with numbered dots on a pixel grid plus a table `{set_id, surface_height_m, points:[{n, u, v, ground_base_link{x,y,z}, ground_map}]}`; the points are intersected with the plane at floor + `surface_height_m` (as in `pixel_to_ground`); the last 10 sets are kept. |
| `resolve_candidate(set_id, n)` | Stored coordinates of one numbered point, with its age and whether the base/arm moved since. |
| `capture_calibration_sample(camera, u, v, ground_x, ground_y, ground_z=0.0)` | Store one marker sample (pixel + measured floor point + parent-link pose + the raw measured arm `joints` when fresh joint states exist) in `cameras.calibration_dir`. |
| `solve_camera_calibration(camera, initial)` | Fit the mount pose to the stored samples; returns `rms_px` and a YAML snippet for `client.yml` (never edits the config). |
| `clear_calibration_samples(camera)` | Delete the stored samples of a camera. |
| `get_topdown_view(radius_m=2.5, layers=all, px=480)` | Robot-up PNG centred on the robot (see Perception and memory) plus metadata `pose`, `scale_m_per_px`, `layers_present`, `layers_missing` (reason each), `data_ages`. Sensor. |
| `remember_object` / `list_objects` / `forget_object` | Object memory in map coordinates, stored as object POIs in poi_store (merge, distance/bearing from the robot). Sensor. |
| `look_around(captures=4, camera='front')` | Effector, ONE motion call: full in-place turn in equal steps with a camera frame and lidar summary per stop. |
| `list_pois` / `add_poi` / `update_poi` / `delete_poi` | Points and areas of interest through `poi_store` (`/poi/*`). Sensor. |

ROS services (`std_srvs/Trigger`): `/arm/home` (move to the stored home pose, then always release arm control, also
after a failed motion; used by the web UI "Arm home" button) and `/arm/set_home` (store the measured pose). The home pose is YAML (`joints: {name: rad}`) at `arm.home_file` (default `/var/lib/ros2/arm/home.yaml`,
directory created by Ansible, owned by the node user), written atomically.

## Camera tools and calibration

All camera tools are read-only (sensor class: allowed in battery cut-off, never command the robot). They use
`ros2_common.camera_geometry` (see `shared/README.md`): pinhole intrinsics plus a mount pose give the pixel ray, which
is intersected with the flat floor. Pixel coordinates `(u right, v down)` are in the **calibrated image size**
(the intrinsics `width x height`; every image these tools return has that size, resized if the camera delivers another).

Both cameras are **not calibrated by default** (`intrinsics: null`, `mount: null`): the tools then fail with
`camera <name> not calibrated: set intrinsics and mount in mcp_server config (see README calibration)`.
`intrinsics` is `{calibration_file: <camera_calibration yaml>}` or an approximation `{hfov_deg, width, height}` (square
pixels, centred principal point, no distortion): results then say `approximate intrinsics`. `capture_calibration_sample`
works before calibration; `solve_camera_calibration` needs the intrinsics.

### Frames

| Camera | Reference frame of results | Floor | Mount `parent_frame` |
|---|---|---|---|
| `front` (fixed overhead camera) | `base_link` (x forward, y left, z up) | `z = 0`: base_link is the ground-projected centre between the wheels (the static TF puts the lidar 0.20 m above it) | `base_link` (fixed) |
| `gripper` (on the wrist) | arm base frame (URDF `base_link` of `so101_arm.urdf`, `z = 0` on the arm mount plane) | `z = -arm_base_height_m` (`floor_z_m`, -0.165 m) | URDF link `gripper_link` (child of `wrist_roll`, the rigid gripper body carrying the fixed jaw; `gripper_frame_link` is only the tool point at the jaw tips and `moving_jaw_so101_v1_link` moves with the gripper joint, so neither is a valid mount) |

For the gripper camera `T_arm_base_gripper_link` is computed from the CURRENT measured joints with the same ikpy chain
as IK/FK (`ArmKinematics.link_frame`), so the tools need fresh `/follower/joint_states`. Mount orientation convention:
REP-103 camera body frame (x forward, y left, z up), fixed-axis RPY in `parent_frame`.

`arm.base_in_base_link` (`x`, `y`, `z`, `yaw` of the arm base frame in base_link; z = `arm_base_height_m`) is unset until
measured. Without it, gripper results are in the arm base frame only (`ground_arm_base`; no `ground_base_link`, no
`ground_map`; distance and bearing are measured from the arm base) and the front camera cannot place arm overlays.

### Tools

- `pixel_to_ground`: `front` gives `ground_base_link`; `gripper` gives `ground_arm_base` (plus `ground_base_link` and
  `ground_map` when `arm.base_in_base_link` is set). `ground_map` comes from the live map -> base_link TF; it is omitted
  when the pose is unknown. Errors for pixels outside the image or rays that miss the floor (sky, behind).
- `get_annotated_camera_image`: overlays are drawn with `project_point_to_pixel` maths in the camera's reference frame:
  `grid` (floor grid, `(x,y)` metre labels, `grid_step_m` 0.05 to 1), `reach` (floor annulus `arm.reach_inner_m` to
  `arm.reach_outer_m` around the shoulder pan axis; approximate, measure and set), `gripper` (tool point circle and its
  straight-down floor foot cross), `planned_gripper` (a `{x,y,z}` target in the arm base frame, as for
  `move_arm_cartesian`, shown before moving), `lidar` (latest `/scan_filtered`, base_link <- laser TF, range-coloured
  dots: red near, blue far). Overlays that cannot be drawn are listed in the metadata `notes` (for example arm overlays on
  the front camera without `arm.base_in_base_link`). A short legend is drawn in the image.
- `mark_candidate_points` / `resolve_candidate`: dots whose pixel does not see the surface plane (floor, or the one at `surface_height_m`), or whose floor point is farther
  than 6 m, are skipped (`skipped_no_ground`). Sets live in memory (last 10, lost on restart). `resolve_candidate` returns
  the STORED values with `age_s`, `robot_moved_since` and `arm_moved_since`: after the base moved `ground_base_link` is stale
  (`ground_map` stays valid); after the arm moved the pixel no longer matches the live image. `resolve_candidate` also
  returns the `surface_height_m` the set was made with.
- Uncertainty: flat floor assumed; error grows with distance (about 1 px of pixel error is several cm far away); the
  gripper camera pose comes from measured joints (servo sag shifts it by millimetres); hfov intrinsics are approximate.

### Calibration procedure

1. Set intrinsics: `cameras.<camera>.intrinsics` (a `camera_calibration` yaml, or an approximate `hfov_deg` with the
   image size) and redeploy `mcp_server`. Start the solver from a rough guess of the mount (measure it with a ruler).
2. Place a marker (a coloured dot or tape cross) on the floor at points whose position you measured with a ruler from
   the robot: for `front` in `base_link` (x forward, y left, floor `z = 0`), for `gripper` in the arm base frame
   (`z = -0.165` for the floor). Spread at least 6 points across the field of view and over distance.
3. Take a photo (`get_camera_image` or `get_annotated_camera_image`; the annotated image works once an approximate mount is
   set and shows how far off it is), read the marker pixel `(u, v)` and call
   `capture_calibration_sample(camera, u, v, ground_x, ground_y, ground_z)`. For the `gripper` camera repeat this at
   several arm poses (move the arm to look at the markers from different joint configurations): each sample stores the
   parent-link pose of that moment (computed with `arm.joint_offsets_rad`) and the raw measured joint positions as
   `joints` (optional field of the sample JSON), so the joint offsets can be re-solved from the same samples.
4. `solve_camera_calibration(camera, initial)` (`initial` = `{x, y, z, roll, pitch, yaw}`, or the configured mount) returns
   `rms_px` (aim for about 1 px or less) and a YAML snippet.
5. Paste the snippet under `cameras:` in the `mcp_server` section of `ansible/group_vars/client.yml`, redeploy
   `mcp_server`, and check with `get_annotated_camera_image(camera)`: the grid lines must meet the markers.
   `clear_calibration_samples` starts a new run. Samples are JSON files in `/var/lib/ros2/camera_calibration/<camera>.json`
   (directory created by Ansible, owned by the node user).

### Joint zero offsets

The measured follower joint angles (`/follower/joint_states`, as in `get_arm_state` and `move_arm_joints`) differ from
the URDF model angles by a constant per joint. `arm.joint_offsets_rad` (`shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`; default all 0.0; the gripper has none) defines `urdf_angle = measured_angle + offset`.

- Applied in ONE place, `ArmKinematics` (`ik.py`, `to_urdf` / `to_measured`): forward kinematics (`tool_pose`, any link
  frame such as the gripper camera's `gripper_link`, the calibration `T_frame_parent`), IK (the seed is converted to
  URDF space, the solution back: command = urdf_angle - offset) and URDF joint-limit clamping (limits are URDF values:
  `move_arm_joints` targets are converted to URDF space, clamped, converted back).
- NOT changed: joint targets of `move_arm_joints`, reported joint positions, `arm_home` poses and every command stay in
  MEASURED space; only kinematics use URDF space.
- How offsets are obtained: capture gripper-camera calibration samples at several arm poses (each stores the raw measured
  `joints`). Per-pose reprojection errors that are consistent with wrong joint zeros indicate the offsets; the offsets
  are then re-solved from the stored samples (jointly with the mount, minimising reprojection error over the samples'
  `joints`), written to `arm.joint_offsets_rad` in `ansible/group_vars/client.yml` and `mcp_server` redeployed. After
  changing them, redo the camera mount solve (the stored `t_frame_parent` of old samples used the old offsets).

### Tool centre point

The tool point is where the jaws actually close, not the URDF frame `gripper_frame_link`. `arm.tool_offset_m` (`x`, `y`,
`z` in m, default all 0.0) is that point expressed IN the `gripper_frame_link` frame (measured on the robot by closing
the jaws on ruler marks).

- Applied in ONE place, `ArmKinematics` (`ik.py`): `forward` / `tool_pose` report `T_base_tool @ [offset, 1]`; the pitch is
  that of the tool frame, unchanged.
- `move_arm_cartesian` (IK) places that point on the target: the `gripper_frame_link` target is
  `target - R_tool @ offset`, iterated (R_tool depends on the solution) until the tool point is within 0.5 mm. The
  pitch handling, floor/approach logic and unreachable errors are unchanged; a zero offset takes the old code path.
- Camera tools (gripper overlay, planned gripper marker) use `forward`, so they draw the corrected point.
- Re-measure after any gripper or jaw change; the repeatability of the current measurement is about 3 mm.

### Grasp shift, wrist roll guard and limit overrides

The gripper has one FIXED and one MOVING jaw; the tool point is (about) the fixed jaw's inner face, so aiming it at an object's
centre pushes the fixed jaw into the object.

- **Grasp shift.** `move_arm_cartesian(object_width_m=w)` treats (x, y, z) as the object centre: for that solve the tool offset
  is extended by `(w / 2) * arm.jaw_open_axis` (a unit vector in `gripper_frame_link`, default `[-1, 0, 0]`: the moving jaw opens
  toward -x; normalised on load). The tool point (fixed jaw face) therefore ends w/2 from the centre against the opening
  direction, the object centred between the jaws. `ArmKinematics.forward` / `inverse` take a per-call `extra_offset` (the shared
  `tool_offset` is never modified; `ik.grasp_offset` builds it). The result carries `grasp_shift`
  `{object_width_m, shift_m, jaw_open_axis, tool_point}` (`tool_point` = the fixed jaw point reached, arm base frame);
  `expected_tool_pose` / `achieved_tool_pose` refer to the object centre. Widths outside (0, 0.08] m are an error.
- **Wrist roll guard.** A motion (`move_arm_joints`, or `move_arm_cartesian` with `wrist_roll`; also `arm_home`) that changes
  `wrist_roll` by more than `limits.roll_guard_min_change_rad` (0.1) is refused with an `ArmError`/tool error and NO motion
  (the lease is not even taken) when the gripper is more open than `limits.roll_max_gripper_open_rad` (0.8 rad, about half of
  the -0.17..1.75 range): the larger of the measured gripper position and any gripper target of the same call counts. The
  message tells the agent to set the gripper about half open (`open_fraction` about 0.5), lift the arm clear of the robot body
  and objects, then roll, and open wider only for the grasp.
- **Joint limit overrides.** `arm.joint_limit_overrides_rad` (`{joint: [lower, upper]}`, URDF-space rad, default empty;
  names must be a chain joint or `gripper`, lower < upper) replaces the URDF limits of the named joints in the IK bounds,
  `within_limits`/`clamp_seed` and every target clamp in `ArmController` (the margins still apply). Example for reaching
  below the floor: `{shoulder_lift: [-1.74533, 2.6]}`. Check on the robot that the wider range is mechanically safe.
- **Speed.** `limits.arm_max_joint_velocity_rps` now defaults to 1.0 rad/s (`le` 1.5); tool descriptions derive the number
  from the config. `gripper_velocity_rps` is unchanged (0.5).

## Body awareness: monitor, events, digest, early return

`RobotMonitor` (`monitor.py`, pure logic, unit tested) is fed by thin ROS callbacks in `ros_iface.py`: `/follower/servo_registers`
(JSON dump of every servo incl. swerve, about every 10 s: `present_temperature` C, `present_load`, `present_current` raw,
`present_voltage` x 0.1 V, `status` bits), `/imu/data`, the swerve `/odom` twist covariance, `/odometry/filtered`,
`/odom_rf2o_twist`, `/cmd_vel_nav`, `/collision_monitor_state`, `/filter/active_source`, the battery guard and the CPU
temperature (`/sys/class/thermal/thermal_zone0/temp`, throttling from the firmware sysfs `get_throttled`, null if
missing). A 4 Hz timer runs the stall, latched-collision and CPU checks. Thresholds are the `monitor` config section
(`MonitorSettings`, also in `ansible/group_vars/client.yml`).

| Event `type` | Severity | Condition (default threshold) |
|---|---|---|
| `overheat` (source = joint) | warning / critical | servo temperature >= `servo_temp_warn_c` 60 / `servo_temp_critical_c` 70 C |
| `servo_error` | critical | servo `status` register != 0; bits decoded: voltage, sensor, overheat, overcurrent, overload |
| `battery_low` | warning | pack < cells x (cutoff_cell_v + `battery_warn_margin_cell_v` 0.2 V) |
| `battery_cutoff` | critical | the shared `BatteryGuard` cut-off (hysteresis as in the gate) |
| `collision_stop` | critical | collision monitor STOP while a base motion runs. The monitor publishes its state only on change, so a STOP from before the motion counts only if no fresher state arrives within `collision_latch_grace_s` 0.5 s: a drive away from the obstacle gets a new state and is not cut short |
| `stall` | critical | base: commanded speed (> 0.05 m/s or 0.1 rad/s) but measured odometry/rf2o ~0 for > `stall_s` 1.0 s while a base motion runs (the arm's tracking abort is its own `arm_tracking_abort` event) |
| `arm_tracking_abort` | warning | arm: setpoint vs measured tracking error above `limits.arm_tracking_error_rad` + `limits.arm_tracking_lag_s` x the motion's velocity; never interrupts the model's turn or a base motion (not in `BASE_INTERRUPTS`) |
| `wheel_slip` | warning | swerve residual (decoded from the twist covariance `var_xy = 0.002 + r^2`; parked fixed value = none) > `slip_residual_warn_mps` 0.1 |
| `bump` | warning / critical | horizontal acceleration spike after baseline removal >= `bump_warn_mps2` 4 / `bump_critical_mps2` 9 m/s^2 for `bump_min_samples` (2) consecutive IMU samples; one glitchy sample is ignored and the severity follows the weakest sample of the run |
| `tilt` | warning | tilt from the IMU orientation > `tilt_warn_deg` 10 deg |
| `human_takeover` | critical | active source leaves `autonomy` while this server holds the lease |
| `cpu_overheat` | warning / critical | CPU >= 75 / 82 C |
| `<type>_cleared` | info | a level condition (overheat, servo_error, battery_*, tilt, wheel_slip, stall, cpu_overheat) ended |

Events are debounced per (type, source) (`debounce_default_s` 30 s, shorter for bump 2 s / collision_stop 1.5 s / stall 1.5 s /
human_takeover 5 s, so a repeat during a retried motion is not hidden; an escalation always passes). Each event is published on **`/robot_events`** (`std_msgs/String`, reliable,
depth 50) as JSON `{"seq": int, "ts": float (epoch s), "type": str, "severity": "info"|"warning"|"critical",
"source": str, "message": str, "data": object}`; `seq` starts at 1 per process.

**Digest on every tool result.** `RobotMCPServer.call_tool` (one override, so every current and future tool gets it)
adds `robot_events_since_last_call` (events since the previous tool call of the same MCP session: one cursor per `Mcp-Session-Id`, so
claude_agent's own `robot_stop` session never consumes the model's events; a new session sees the retained history, at most
16 sessions are tracked (LRU), calls without a session id share one cursor; newest `digest_max_events` 20 kept) and `vitals` (`battery 11.40 V, hottest servo 41 C (elbow_flex), CPU 52 C`,
`n/a` when unknown) as a text block and, when the result is structured, as structured-content keys (image tools stay
unstructured: text block plus `_meta`). Tool errors get the same JSON appended to the message. The tool output schemas do
not list these two keys.

**Early return.** `navigate_to_pose`, `move_relative`, `drive` and the arm tools (`move_arm_joints`, `move_arm_cartesian`,
`set_gripper`, `arm_home`) end early with status `interrupted` and `interrupted_by` = the critical event type fired during
the call: base `collision_stop`, `stall`, `battery_cutoff`, `overheat`, `cpu_overheat`, `servo_error`, `human_takeover`,
`bump`; arm `overheat`, `cpu_overheat`, `servo_error`, `battery_cutoff`, `human_takeover` (the lease is dropped, nothing is
published against the human) and `arm_tracking_abort` (a warning event, not a critical one: the arm result keeps status `aborted_tracking` and carries
`interrupted_by: arm_tracking_abort`). The base cancels the Nav2 goal and publishes a zero twist; the arm holds its measured pose.
The battery cut-off is also re-read live from the guard during every motion. Warnings never interrupt, and only events
raised after the motion started count. Every motion result carries `expected` (goal) vs `achieved` (final measured
pose / joints; for `drive` the integrated command vs the measured displacement in the start frame; for
`move_arm_cartesian` also `expected_tool_pose` / `achieved_tool_pose`) and `duration_s`; the older `goal` / `final_pose`
/ `target` / `positions` fields stay.

New tool modules (e.g. perception) are registered in one place: a function taking a `ToolContext` added to
`tools.TOOL_MODULES`.

## Perception and memory

Module `perception_tools.py` (one entry in `tools.TOOL_MODULES`); the logic is in rclpy-free modules (`topdown.py`,
`object_memory.py`, `look_around.py`, `poi_client.py`) that only use OpenCV/numpy and render on demand (the service has
a 25 % CPU quota).

**`get_topdown_view`.** PNG, `px` x `px` (default 480), centred on the robot and **robot-up**: base_link +x (the
heading) is the top of the image, +y (left) is the left; one pixel is `2 * radius_m / px` metres (`scale_m_per_px` in
the metadata). A red arrow marks the heading, the blue rectangle is the footprint (`footprint.length_m` x `width_m`,
470 x 386 mm), a 0.5 m scale bar sits bottom left and a legend top left lists the drawn layers. Layers (`layers`,
default all): `map` (SLAM map crop sampled through the map -> base_link pose: white free, black wall, grey unknown),
`costmap` (`/local_costmap/costmap`, subscribed lazily on the first call, latest message only, placed through the TF of
its frame, orange tint), `lidar` (`/scan_filtered` points), `footprint`, `reach` (circle of `topdown.arm_reach_m` about
`arm_mount_x_m`/`arm_mount_y_m` in base_link), `path` (latest `/plan`, only when younger than
`topdown.plan_max_age_s`), `pois` (points as circles with their radius, areas as polygons, names; orange open, green
done, grey cancelled) and `objects` (remembered objects as diamonds with labels). A layer without data is **never
fabricated**: it is listed in `layers_missing` with the reason (no TF, stale, poi_store down, ...); layers that live in
the map frame also need the robot pose.

**Object memory.** Remembered objects ARE POIs: `remember_object(label, x, y, frame='map', note='', confidence=0.7)`
adds (or updates) a poi_store POI of `kind: "object"`, `created_by: "agent"`, `name` = label (<= 60 chars) plus
`times_seen`, `first_seen`, `last_seen` and `confidence`, through the same `/poi/command` client as `add_poi`. They show
on the person's web UI map and are removed with the other agent-made POIs when the person starts a new session (the
claude_agent "New session" button sends `{"op": "clear", "created_by": "agent"}`); POIs the person made stay. A sighting
with the same label (case insensitive) within `objects.merge_radius_m` (0.25 m) of a remembered object updates the
nearest one instead: position = average weighted by `times_seen x confidence` (old) and `confidence` (new),
`times_seen + 1`, `last_seen`, the higher confidence, and the note only if a new one is given. Records returned to the
agent: `{id, label, x, y, note, confidence, first_seen, last_seen, times_seen}` (the id is the POI id).
`list_objects(label_contains, near_x, near_y, radius_m)` returns only object POIs, sorted by distance to the robot, with
`distance_m` and `bearing_deg` (0 ahead, + left); `near_x`/`near_y` go together (radius default 1 m). `list_pois` lists
every POI and states each one's `kind` (`point`, `area` or `object`). `forget_object(id)` deletes one object POI (other
POIs: `delete_poi`). The tools need poi_store (RobotError "poi_store is not running" otherwise). `get_topdown_view`
reads both layers from the same `/poi/list`: `pois` draws the non-object POIs, `objects` the object POIs, so nothing is
drawn twice.

*Migration.* `objects.store_path` (`/var/lib/ros2/objects/objects.json`) is now only the legacy file of the former
separate object memory. On the first object tool call that reaches poi_store, its entries are imported once as object
POIs (`created_by: "agent"`, sighting fields and confidence kept) and the file is renamed `objects.json.migrated`; while
poi_store is down the file is left untouched and imported later. An unreadable file is moved to
`objects.json.corrupt-<ts>`.

**`look_around(captures=4, camera='front')`.** Counts as ONE motion call and is in `MOTION_TOOLS` (battery gate;
early return). It checks the lidar first and is **refused without moving** when the nearest return is closer than the
footprint circumscribed radius plus `look_around.clearance_margin_m` (10 cm), or when there is no fresh scan. Then it
rotates the base in place in `captures` equal steps (3 to 12; steps of 360 / captures degrees through
`move_relative(0, 0, step)`), at each stop it grabs a camera frame and the lidar sector summary, and a last step
returns to the start heading. One event mark covers the whole run and is checked before every rotation, so a critical
body event raised while capturing between steps also ends it: `interrupted` + `interrupted_by`, the frames captured so
far are kept, no further rotation and **no return to the start** (the message says so). A step that ends `interrupted`
or failed, a `stop` call between steps, or an obstacle inside the rotation circle likewise ends the sequence at once.
A `RobotError` from a step (stale map pose, ...) no longer escapes: status `failed` with the message and the frames so
far, and one corrective rotation back to the start heading is attempted unless the error was that another base motion is
running. `returned_to_start` is true only when the closing rotation succeeded and the measured final heading is within
`nav.goal_yaw_tolerance_deg` (2 deg) of the start heading; otherwise a note reports the heading error.
Result: a montage JPEG (tiles labelled
with the heading in degrees counter-clockwise from the start; a missing frame is a grey "no frame" tile), a top-down PNG,
and structured `headings` (nearest obstacle overall and per sector at each stop), `steps`, `expected` (360 deg, stops)
vs `achieved` (`rotation_deg`, `final_pose`, `heading_error_deg`), `returned_to_start`, `notes`.

**POI tools** (`poi_store`, see `nodes/poi_store/README.md`). `list_pois(status, near)` reads the latched `/poi/list`
and adds `distance_m`, `bearing_deg` (areas: centroid, plus `inside`); `near=true` keeps POIs within
`poi.near_radius_m` (5 m), nearest first. `add_poi(kind, name, note, x, y, polygon, radius_m)` publishes
`/poi/command` `{op: "add", request_id, poi}` with `created_by: "agent"`; a point without x/y is placed at the robot's
current position, an area needs `polygon` (>= 3 vertices). `update_poi(id, name, note, status, x, y, polygon)` sends only
the given fields; `delete_poi(id)`. Each call waits up to `poi.request_timeout_s` (3 s) for the `/poi/result` with the
same `request_id`; the tool errors when poi_store is not running (no subscriber on `/poi/command`), does not answer
or rejects the change.

## Battery cut-off gate

Optional `battery` config section (the shared `ros2_common.battery.BatteryConfig`: `topic`, `cells`, `cutoff_cell_v`,
`resume_cell_v`, `stale_s`). When present, the node subscribes to the `sensor_msgs/BatteryState` topic and feeds a
`ros2_common.battery.BatteryGuard`. Cut-off starts below `cells * cutoff_cell_v` and ends only above
`cells * resume_cell_v` (hysteresis); no reading, or one older than `stale_s`, means unknown and nothing is refused.
While in cut-off every motion tool raises an MCP tool error without touching the robot (and logs a warning), e.g.
`battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); motion refused`. Without a `battery` section the gate is off.

The classification lives in one place, `mcp_server/tools.py` (`MOTION_TOOLS`, `ALWAYS_ALLOWED_TOOLS`; a test checks they
partition every tool):

| Class | Tools |
|---|---|
| `MOTION_TOOLS` (refused in cut-off) | `navigate_to_pose`, `move_relative`, `drive`, `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `arm_set_home`, `look_around` |
| `ALWAYS_ALLOWED_TOOLS` | `stop`, `get_robot_state`, `get_body_state`, `get_camera_image`, `get_map_summary`, `get_arm_state`, `acquire_control`, `release_control`, `get_topdown_view`, `remember_object`, `list_objects`, `forget_object`, `list_pois`, `add_poi`, `update_poi`, `delete_poi` |

`arm_set_home` is classed as motion because it rewrites the pose a later `arm_home` drives to.

## Safety model

- **Base**: navigation goes through Nav2 (planner, controller, velocity smoother, collision monitor). `drive` publishes
  on `/cmd_vel_nav` (upstream of the smoother and collision monitor), clamped and limited to 2 s, and always ends with
  a zero twist; the swerve controller also stops on its own 0.5 s cmd_vel timeout. One base motion at a time.
- **Arm**: the server never writes `/follower/joint_commands`. It publishes setpoints on filter_node's autonomy input
  (`/filter/autonomy_joint_commands`); filter_node arbitrates with the leader arm and the web UI and reports the active
  source on `/filter/active_source`. Motion tools acquire the lease implicitly; if filter_node switches to another
  source (after a 0.5 s grace) the lease is dropped and the motion stops without fighting the new source. While the
  lease is held and idle, the last setpoint is republished at `hold_republish_hz` (5 Hz) as a keepalive.
- **Lease is sticky**: filter_node ignores the leader arm and the web UI while the autonomy lease is held, so the lease
  is only ended by an explicit `release_control` (the MCP client is told so in the tool descriptions), by `/arm/home`
  (always releases), by `arm_home` when control was not held before it, or on shutdown.
- **Release is final**: every autonomy publish (setpoint, hold, keepalive, release) runs under one lock and setpoints
  are published only while the lease is held. `release_control` aborts the stream, stops the keepalive and then
  publishes the release, so no setpoint can follow the release and silently re-take the lease.
- **Stop does not grab the arm**: `stop` always cancels Nav2 goals and zeroes the base, but holds the arm only when
  this server holds arm control or an arm motion is running. A base-only stop leaves a human teleoperating with the
  leader arm or web UI in control.
- **Orphaned lease after a crash**: on startup the server watches `/filter/active_source` for
  `ORPHAN_LEASE_WINDOW_S` (3 s). If filter_node reports `autonomy` while this process does not hold control (a
  previous run crashed or was OOM-killed holding the lease), it publishes the autonomy release and logs a warning,
  returning the arm to normal arbitration.
- **Arm motions**: targets clamped to URDF limits minus `arm_limit_margin_rad` (0.05; `arm_limit_margin_overrides`
  replaces it per joint, default `{gripper: 0.005}` so the gripper reaches -0.165 rad, just above the measured stop at
  -0.172 rad); synchronised quintic trajectory
  with per-joint velocity <= `arm_max_joint_velocity_rps` (default 1.0 rad/s; `speed_scale` 0.5 = that maximum, lower is proportionally slower) streamed at
  25 Hz; blocks until converged (`arm_converge_tolerance_rad`) or `arm_converge_timeout_s` after the trajectory.
  Aborts and holds the measured pose when `/follower/joint_states` is older than 0.3 s, the tracking error (setpoint vs
  measured, gripper excluded) exceeds `arm_tracking_error_rad` (0.25) + `arm_tracking_lag_s` (0.25 s) x the motion's joint velocity cap (0.5 rad at full speed 1.0 rad/s, 0.3 rad at 0.2 rad/s: servos lag more at speed), or `stop` is called. One arm motion at a time.
- **No sag ratchet**: while the lease is held, a motion starts from the last commanded pose and joints it does not name
  keep their last commanded target (the measured pose is used only right after acquiring). Re-commanding the measured
  pose would lock gravity sag in and let it accumulate over calls.
- **Settled residual error**: the position servos stop short of the target under gravity load (0.05-0.06 rad when
  lifting the shoulder slowly). When every moved joint is within `arm_settle_tolerance_rad` (0.08, must lie between
  the converge tolerance and the tracking abort) and moved less than `arm_settle_motion_rad` (0.005) over
  `arm_settle_window_s` (0.5 s), the motion returns `converged` with message `settled with residual error (...)` and
  `residual_error` = target - measured (rad) per joint outside the converge tolerance; the target stays commanded (no
  hold at the sagged pose). A joint still moving or further off ends in `timeout` and holds the measured pose as
  before.
- **Bounded residual hold**: a settled target is held for `arm_settle_hold_s` (2.0 s). Then the keepalive relaxes the
  hold setpoint of joints still outside the converge tolerance to their measured pose, so a joint stalled against an
  obstacle or the floor is not pushed at the torque limit indefinitely (only wrist_roll and gripper publish effort).
  The intended target is kept separately and still seeds the next motion (unnamed joints, IK seed), so the sag
  ratchet stays fixed. Any new setpoint, release, lease loss or stop cancels the pending relax.
- **Gripper motions keep the arm held**: `set_gripper` (and any gripper-only motion) streams and holds the arm joints
  at their intended targets, also when the residual hold had already relaxed (the first gripper setpoint restores
  it), and a grasp stops with the arm intent plus the measured gripper, not the sagged arm pose. Joints that were
  held with a residual error get their relax re-armed `arm_settle_hold_s` after the gripper motion finished.
- **Effort contact gate**: `close_until_effort` ignores the gripper load for `gripper_effort_ignore_s` (0.3 s) after
  the close starts (the load spikes when the motor starts) and afterwards counts it only when the jaw moved at least
  `gripper_contact_travel_rad` (0.03) from its start or stalled (less than `arm_settle_motion_rad` over
  `arm_settle_window_s`).
- **Follower gripper command range**: direct (autonomy / web_ui) gripper commands are clamped by the feetech
  follower bridge to the servo command range (`command_min_steps: 1900` in `client.yml`, max read from the servo
  register; gripper not inverted). The closed target -0.165 rad is 2048 + (-0.165 / 0.001534) = 1940 steps, inside
  that range (the measured stop -0.172 rad is 1936 steps). The servo EEPROM angle limits are not changed by this node.
- **Grasp from stall**: when closing (`close_until_effort` or `open_fraction=0`) the jaw settles before the closed
  position (residual outside the converge tolerance), the result is `grasped` with "contact inferred: jaw stalled
  ..." and the gripper holds the stall position plus `gripper_grasp_squeeze_rad` (0.03) toward closed (never past
  closed) instead of squeezing to the full closed target. Both this and an effort contact must pass the closure check
  (`gripper_grasp_min_travel_rad` from the start, at most `gripper_grasp_max_open_rad` open), else `blocked`.
- **No gravity lead**: a feed-forward offset (target + k in the lift direction) is not applied: whether a joint
  works against gravity depends on the whole arm pose (needs a mass model), a motion-direction lead overshoots when
  lowering, and the result cannot be checked without the robot.
- **No placeholder data**: nothing is published without fresh measured joint states; state tools omit stale sources.

## Code layout

| Module | ROS? | Purpose |
|---|---|---|
| `config.py` | no | pydantic config, `MCP_SERVER_CONFIG`, `MCP_SERVER_TOKEN` |
| `trajectory.py` | no | quintic interpolation, limit clamping, tracking error |
| `ik.py` | no | URDF limits, ikpy FK/IK with verification |
| `arm.py` | no | `ArmController`: lease, streaming, aborts, gripper, home |
| `base_motion.py` | no | timed clamped drive loop, stop sequence (`run_stop`) |
| `perception.py` | no | scan sectors/points, map stats/PNG, image encoding |
| `topdown.py`, `perception_models.py` | no | robot-up top-down renderer (layers, transforms); perception result models |
| `object_memory.py`, `look_around.py`, `poi_client.py` | no | object POI memory + merge, look_around plan/loop/montage, POI request matching |
| `perception_tools.py` | no | the perception, memory, look_around and POI tools (one `TOOL_MODULES` entry) |
| `home_store.py`, `staleness.py`, `geometry.py`, `models.py` | no | home YAML, data age, pose math, result models |
| `monitor.py` | no | `RobotMonitor`: vitals, events, digest, `MotionWatch` |
| `tool_context.py`, `body_tools.py` | no | `ToolContext` / `RobotApi`; the `get_body_state` module |
| `camera_tools.py` | no | the camera tool module (registered in `tools.TOOL_MODULES`) |
| `camera_scene.py`, `camera_overlay.py`, `camera_candidates.py`, `camera_calib.py` | no | calibrated scene and frames, overlay drawing and candidate grids, candidate sets, sample store and solver wiring |
| `tools.py` | no | MCP tools, `StaticTokenVerifier`, Streamable HTTP app |
| `ros_iface.py` | yes | `RosRobot`: subscriptions, TF, Nav2 action, publishers, services |
| `spin.py` | no | `spin_forever` (executor loop that survives rclpy stale-handle errors), `run_or_exit` (exits the process when the executor thread dies, so systemd restarts the node instead of serving stale data), `FrameCache` (latest camera frame) |
| `__main__.py` | yes | entry point: executor thread + uvicorn |

## Configuration

All keys are optional (defaults in `config.py`). Example (`/etc/ros2/mcp_server/config.yaml`, deployed from the
`ros2_nodes` entry in `ansible/group_vars/client.yml`):

```yaml
server:
  host: "0.0.0.0"
  port: 18200
  path: /mcp
arm:
  urdf_path: nodes/web_ui/urdf/so101_arm.urdf   # relative paths resolve against the repo root
  home_file: /var/lib/ros2/arm/home.yaml
  arm_base_height_m: 0.165   # arm mount plane height above the floor (m); floor_z_m = -this
  gripper_open_rad: 1.5       # follower gripper joint positions (rad)
  gripper_closed_rad: -0.165
  reach_outer_m: 0.25        # floor reach around the shoulder axis (annotated image annulus)
  reach_inner_m: 0.05
  joint_offsets_rad: {shoulder_pan: 0.0, shoulder_lift: 0.0, elbow_flex: 0.0, wrist_flex: 0.0, wrist_roll: 0.0}   # urdf = measured + offset
  tool_offset_m: {x: 0.0, y: 0.0, z: 0.0}   # jaw closing point in the gripper_frame_link frame
  jaw_open_axis: [-1.0, 0.0, 0.0]           # direction the moving jaw opens, gripper_frame_link (normalised)
  joint_limit_overrides_rad: {}             # e.g. {shoulder_lift: [-1.74533, 2.6]} replaces URDF limits
  # base_in_base_link: {x: 0.0, y: 0.0, z: 0.165, yaw: 0.0}   # optional, once measured
cameras:                  # default: not calibrated (see Camera tools and calibration)
  calibration_dir: /var/lib/ros2/camera_calibration
  gripper: {parent_frame: gripper_link, intrinsics: null, mount: null}
  front: {parent_frame: base_link, intrinsics: {hfov_deg: 66.0, width: 640, height: 480}, mount: null}
limits:
  arm_max_joint_velocity_rps: 1.0
  roll_guard_min_change_rad: 0.1     # wrist_roll changes above this are refused ...
  roll_max_gripper_open_rad: 0.8     # ... while the gripper is open wider than this (about half open)
  arm_settle_tolerance_rad: 0.08   # steady-state error reported as residual_error instead of a timeout
  gripper_effort_threshold: 300.0
  gripper_effort_ignore_s: 0.3      # ignore the load spike when the motor starts
  gripper_contact_travel_rad: 0.03  # jaw travel (or a stall) required before effort counts as contact
  gripper_grasp_min_travel_rad: 0.15  # closure from the start required for 'grasped' (else 'blocked')
  gripper_grasp_max_open_rad: 1.2     # a stall more open than this is pushing on something ('blocked')
  arm_limit_margin_rad: 0.05
  arm_limit_margin_overrides: {gripper: 0.005}   # per-joint margins; the gripper may close to its physical stop
timeouts:
  follower_stale_s: 0.3
  image_max_age_s: 1.0
battery:                  # optional; absent = battery gate off
  topic: /battery_state
  cells: 3
  cutoff_cell_v: 2.8
  resume_cell_v: 2.9
  stale_s: 5.0
topics:                   # perception additions (defaults shown)
  local_costmap: /local_costmap/costmap
  plan: /plan
  poi_list: /poi/list
  poi_command: /poi/command
  poi_result: /poi/result
footprint: {length_m: 0.47, width_m: 0.386}
topdown: {default_radius_m: 2.5, default_px: 480, arm_reach_m: 0.41, plan_max_age_s: 30}
objects: {store_path: /var/lib/ros2/objects/objects.json, merge_radius_m: 0.25}  # store_path: legacy file, imported once into poi_store
look_around: {default_captures: 4, clearance_margin_m: 0.10, step_timeout_s: 30}
poi: {request_timeout_s: 3.0, near_radius_m: 5.0}
monitor:                  # all optional; thresholds of the body monitor (see Body awareness)
  servo_temp_warn_c: 60
  servo_temp_critical_c: 70
  cpu_temp_warn_c: 75
  cpu_temp_critical_c: 82
  stall_s: 1.0
  bump_warn_mps2: 4.0
  bump_critical_mps2: 9.0
  bump_min_samples: 2
  tilt_warn_deg: 10
```

## Claude Code setup

The token is generated once on the robot by Ansible (`/etc/ros2/mcp_server/token`, mode 0600) and never stored in git.

```bash
# 1. Export the token from the robot (ssh to client.ros2.lan; override with ROBOT_SSH_TARGET=user@host)
eval "$(./scripts/robot_mcp_token.sh)"

# 2a. Project scope: the repo-root .mcp.json already registers server "robot" and expands ${ROBOT_MCP_TOKEN};
#     start Claude Code from this repo in the same shell and approve the project server.
claude

# 2b. Or add it explicitly (user scope)
claude mcp add --transport http robot http://client.ros2.lan:18200/mcp \
  --header "Authorization: Bearer ${ROBOT_MCP_TOKEN}"
```

## Development

```bash
cd nodes/mcp_server
poetry install    # also installs the shared `ros2-common` package (path dependency ../../shared, develop mode)
poetry run pytest tests -q   # rclpy-free unit tests (rclpy is only imported by ros_iface.py / __main__.py)
poetry run poe lint          # ruff check, ruff format --check, vulture
```

Deploy: `./scripts/deploy-nodes.sh client mcp_server` (a `--tags mcp_server` run of `ansible/playbooks/deploy_nodes_client.yml`).
