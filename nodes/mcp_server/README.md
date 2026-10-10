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
| `get_camera_image(camera, max_px<=1024)` | Default size `limits.default_camera_px` (384 px on the longest side; ask for more only to read detail, images fill the context: 640x480 is about 400 tokens). Waits for the next frame on a persistent per-camera subscription (created at startup, never destroyed: per-call subscriptions raced the executor and killed it), with timeout. `gripper`: `/camera_0/image_raw/compressed` (JPEG passed through or downscaled once); `front`: `topics.front_camera` (default `/overview_camera/image_raw/compressed`, 640x480 overhead Camera Module 3 looking down at the front of the robot, the arm and the floor in front; JPEG passed through or downscaled once, best view for judging gripper-to-object position). Returns MCP image content (JPEG) + capture stamp; error if no frame within `image_timeout_s` or the frame is older than 1 s. |
| `get_body_state` | Body vitals (sensor, always allowed): per-servo latest temperature / load / current / voltage / status flags with data age, hottest servo, battery V, per-cell V, margin to cut-off and cut-off state, IMU roll/pitch/tilt and last bump, wheel slip residual, commanded vs measured base speed, CPU temperature and firmware throttling flag, active source + lease, last 10 events. Missing data is `null` with a reason in `notes`. |
| `get_map_summary(include_png, radius_m)` | Nearest `/scan_filtered` obstacle in 8 sectors around base_link, `/map` size and known/occupied/free cells, robot pose, optional small PNG of the map around the robot. |
| `navigate_to_pose(x, y, yaw, frame='map', timeout_s, precise=false)` | Nav2 `NavigateToPose`; by default the goal ends early (cancelled, base zeroed, status `succeeded`) as soon as the measured pose is within `nav.intermediate_xy_tolerance_m` / `nav.intermediate_yaw_tolerance_deg` (3 cm / 5 deg); `precise=true` waits for Nav2's own checker (`nav.goal_xy_tolerance_m` / `goal_yaw_tolerance_deg`, 1 cm / 2 deg, slower). Blocks until result/timeout (goal cancelled on timeout or stop); returns result and final pose. The description states the goal precision from `nav.goal_xy_tolerance_m` / `nav.goal_yaw_tolerance_deg` (default 1 cm / 2 deg, keep equal to the Nav2 goal checker) and that a sideways goal first turns the robot toward the path (front leading) and turns back to the goal heading at the end. |
| `move_relative(dx, dy, dyaw, timeout_s, precise=false)` | Same (incl. `precise`), goal given in base_link (converted to a map goal via TF). Small moves (a few cm) really move. |
| `drive(vx, vy, wz, duration_s<=2)` | 20 Hz on `/cmd_vel_nav` (through velocity smoother + collision monitor), clamped to 0.25 m/s / 0.5 rad/s, then zero. |
| `stop` | Always available: clears the motion queue first (dropped job ids in `motion_queue_dropped`), cancels all NavigateToPose goals and publishes a zero twist. Aborts any arm motion and holds the arm at its measured pose only if this server holds arm control (or a motion is running); otherwise the arm is not touched (`arm_held: false`). |
| `get_arm_state` | Joint positions (measured follower values)/efforts, gripper effort, tool point pose (x, y, z, pitch; forward kinematics with `arm.joint_offsets_rad` applied), `floor_z_m` (floor height in the base_link frame), active source, lease, home stored. |
| `acquire_control` / `release_control` | Start the autonomy lease (publish the measured pose on `/filter/autonomy_joint_commands`) / end it (`std_msgs/Bool` true on `/filter/autonomy_release`). The lease is sticky: release it explicitly when done. |
| `move_arm_joints(targets, speed_scale<=0.5, settle=None)` | `settle`: `trajectory_end` (default; `final` when the call moves the gripper joint, `limits.arm_default_settle`) or `final`, see the safety model.  Interpolated motion to joint targets (follower joint radians, as in `get_arm_state`). Unnamed joints keep their last commanded target. `speed_scale` 0.5 is the maximum, `limits.arm_max_joint_velocity_rps` (default 1.0 rad/s); the description states the configured value. A `wrist_roll` change above `limits.roll_guard_min_change_rad` is refused while the gripper is open wider than `limits.roll_max_gripper_open_rad` (see Wrist roll guard). `converged` results may carry `residual_error` (see Safety model). |
| `move_arm_cartesian(x, y, z, pitch=None, frame='base_link', speed_scale, wrist_roll=None, object_width_m=None, settle=None)` | `settle` as for `move_arm_joints`.  ikpy IK on `nodes/web_ui/urdf/so101_arm.urdf` (5-DOF: position + approach pitch); `wrist_roll` (rad, measured space, clamped to the limits; clamping is listed in `clamped`) is the roll the IK keeps for this target and the motion rolls to, omitted = the current roll is kept; `object_width_m` (m, 0 < w <= 0.08) makes (x, y, z) the OBJECT CENTRE (see Grasp shift); `unreachable` is reported, never guessed. `base_link` here is the arm URDF root (arm mount, z = 0). The floor is at `z = -arm.arm_base_height_m` (default 0.104, measured 2026-10-10; see Arm mount); the tool descriptions and `get_arm_state.floor_z_m` state it. Motions near or below the effective surface are slowed (never blocked) by the floor slow zone; `surface_z_m`, `tilt_override_deg` and `surfaces` (also on `move_arm_joints`, `set_gripper` and `arm_home`; see Surface regions) shift it per call and results report `slow_zone`. |
| `set_gripper(open_fraction | close_until_effort, effort_threshold, grip_profile, surfaces)` | Open to a fraction (0 closed, 1 open) or close with a grip profile (see Grip profiles) until `abs(effort) >= threshold` (default: the profile's `contact_effort_threshold`) or the closing load reaches the profile's `target_load` (then hold: `grasped`, else `closed_no_contact`). A stall or effort contact only counts as `grasped` when the jaw closed at least `limits.gripper_grasp_min_travel_rad` (0.15 rad) from where it started AND stopped no more open than `limits.gripper_grasp_max_open_rad` (1.2 rad; a nearly open jaw that stalls is pushing on something); otherwise the status is `blocked` with "jaw stopped at X rad after Y rad travel - likely pressing on an object rather than holding it", and the measured jaw position is held (no squeeze). `arm.gripper_closed_rad` / `gripper_open_rad` are follower gripper joint positions (defaults -0.165 / 1.5 rad; measured fully closed is -0.172 rad, URDF lower limit -0.1745, so the jaws close fully). With `close_until_effort` the load is ignored for `gripper_effort_ignore_s` (0.3 s, motor start-up spike) and then counts only once the jaw moved `gripper_contact_travel_rad` (0.03) or stalled with the command at least `gripper_stall_lead_rad` (0.15) ahead of it. The arm joints keep being held at their intended targets while the gripper moves (see Safety). |
| `plan_grasp(object, strategy='auto', params, approach_pitch_deg, surface_z_m, tilt_override_deg, grip_profile, surfaces)` | Dry-run grasp plan (sensor, no motion): outcome `planned` / `infeasible` with reasons, the resolved `grip_profile` (an unknown profile is refused), chosen strategy, pitch, wrist roll, jaw opening, waypoints and slow-zone annotations. See Grasp macros. |
| `grasp_object(object, strategy='auto', params, approach_pitch_deg, surface_z_m, tilt_override_deg, grip_profile, surfaces)` | Plan and execute (motion): outcome `grasped` / `missed` / `aborted` / `infeasible`, with the executed steps, the final gripper position/load and the grip report (`grip_profile`, `holding_load`, `slipping`, `crush_risk`; see Grip profiles). See Grasp macros. |
| `release_object(params, surface_z_m, tilt_override_deg, surfaces)` | Open to `release_open_fraction` and lift `release_lift_m` straight up (motion): outcome `released` / `aborted`. |
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
| `look_around(captures=4, camera='front', mode=None, return_to_start=None)` | Effector, ONE motion call: in-place turn through equal headings with a camera frame and lidar summary per heading. `mode` `spin` (default, `look_around.mode`): one continuous slow rotation; `steps`: one precise Nav2 rotation per heading. Ends facing the last heading unless `return_to_start=true` (default `look_around.return_to_start`, false). |
| `enqueue_motions(steps, replace=false)` | Effector: queue motion steps and return at once (job ids, queue length, blend groups). See [Motion queue](#motion-queue). |
| `get_motion_status` | Queue snapshot (sensor): running, current step (elapsed, progress, blend group), pending steps, last events. |
| `cancel_motions` | Uncapped control: drop the pending steps and abort the running one (base zeroed, arm held when a motion step runs). |
| `wait_for_event(timeout_s, until='queue_empty', since_seq)` | Sensor: block until the queue drains or something relevant happens; returns the new events, the queue status and a compact state digest. |
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
| `gripper` (on the wrist) | arm base frame (URDF `base_link` of `so101_arm.urdf`, `z = 0` on the arm mount plane) | `z = -arm_base_height_m` (`floor_z_m`, -0.100 m with the measured mount) | URDF link `gripper_link` (child of `wrist_roll`, the rigid gripper body carrying the fixed jaw; `gripper_frame_link` is only the tool point at the jaw tips and `moving_jaw_so101_v1_link` moves with the gripper joint, so neither is a valid mount) |

For the gripper camera `T_arm_base_gripper_link` is computed from the CURRENT measured joints with the same ikpy chain
as IK/FK (`ArmKinematics.link_frame`), so the tools need fresh `/follower/joint_states`. Mount orientation convention:
REP-103 camera body frame (x forward, y left, z up), fixed-axis RPY in `parent_frame`.

`arm.base_in_base_link` (`x`, `y`, `z`, `yaw` of the arm base frame in base_link; z must equal `arm_base_height_m`)
defaults to the mount measured on the robot 2026-10-10 `{x: 0.0592, y: -0.05, z: 0.100, yaw: 0.0}` (5.92 cm forward,
5 cm right, 10 cm above the robot plane). Set it to `null` to disable the arm <-> base_link conversions. Without it, gripper results are in the arm base frame only (`ground_arm_base`; no `ground_base_link`, no
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
   (`z = -arm_base_height_m`, -0.100 with the measured mount, for the floor). Spread at least 6 points across the field of view and over distance.
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
- Grid calibration 2026-10-10 (current): joint offsets, gripper camera mount and intrinsics and the tool point were
  solved jointly on a 50 mm grid sheet (234 camera points from 20 views, 6 tip touch-downs, floor fixed at the measured
  `z = -0.104`): pan 0.0182, lift -0.0345, elbow -0.1070, wrist_flex 0.0988 rad (wrist_roll -0.0710 kept: not
  identifiable from these data); camera mount (-0.0046, 0.0364, -0.0364) m rpy (-1.5622, 1.1950, -1.4466) on
  `gripper_link`; intrinsics f 410.6, k1 -0.1666 (`calibration/gripper_camera.yaml`). Held out: floor error 4.1 mm
  (2026-10-08 set: 44.9 mm), tip 6 mm (35 mm), home tip height 243.6 mm vs 243 mm measured. Data and results:
  `docs/calibration/2026-10-10/session/results.yaml`. The earlier 2026-10-10 floor re-check of the 2026-10-08 set
  (`docs/calibration/2026-10-10/README.md`) found no improvement from re-solving with the old samples.

### Tool centre point

The tool point is where the jaws actually close, not the URDF frame `gripper_frame_link`. `arm.tool_offset_m` (`x`, `y`,
`z` in m, default all 0.0) is that point expressed IN the `gripper_frame_link` frame. Since 2026-10-10 it is the physical
fixed-jaw tip (0.0010, -0.0056, -0.0014), fitted with the joint offsets (grid sheet tip touches); the 2026-10-08 value
was an effective TCP (0.0104, -0.0282, -0.0017) that absorbed kinematic errors.

- Applied in ONE place, `ArmKinematics` (`ik.py`): `forward` / `tool_pose` report `T_base_tool @ [offset, 1]`; the pitch is
  that of the tool frame, unchanged.
- `move_arm_cartesian` (IK) places that point on the target: the `gripper_frame_link` target is
  `target - R_tool @ offset`, iterated (R_tool depends on the solution) until the tool point is within 0.5 mm. The
  pitch handling, floor/approach logic and unreachable errors are unchanged; a zero offset takes the old code path.
- Camera tools (gripper overlay, planned gripper marker) use `forward`, so they draw the corrected point.
- Re-fit it with the joint offsets after any gripper or jaw change; the held-out tip error of the current fit is 6 mm.

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
the shoulder_pan axis in base_link: `arm.base_in_base_link` plus the URDF pan joint origin, 38.8 mm ahead of the mount;
about base_link `0` without a mount), `path` (latest `/plan`, only when younger than
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
`move_relative(0, 0, step)`) in `mode='steps'`, or in ONE continuous slow rotation in `mode='spin'` (the default, see
below); at each heading it grabs a camera frame and the lidar sector summary. A last step returns to the start heading
only with `return_to_start=true` (tool default `look_around.return_to_start`, false: the turn ends facing the last
heading, 270 deg for 4 captures). One event mark covers the whole run and is checked before every rotation, so a critical
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

**Spin mode** (`look_around.mode: spin`, default). The steps mode stopped at every heading for a precise Nav2 goal
(median 35 s for 4 captures plus the return). The spin mode captures heading 0, then turns once at
`look_around.spin_speed_rps` (0.4 rad/s) through `RobotApi.spin` (`base_motion.run_spin`: `/cmd_vel_nav` at
`spin_rate_hz` through the velocity smoother and collision monitor, the base motion lock, the stop flag, critical body
events, a lost pose and a timeout of angle / speed + `spin_timeout_margin_s` all end it, a zero twist always follows).
The rotation is integrated from the measured heading, and a frame plus the lidar summary is taken each time it passes
the next heading (frames are taken while turning slowly). An obstacle inside the rotation circle at a heading ends the
spin (`aborted_obstacle`). With `return_to_start` the spin covers the full circle and one precise corrective
`move_relative` removes a final heading error beyond the tolerance (never after an interruption, a stop or a failure).
Tests check that both modes give the same tiles, labels and heading summaries (`test_look_around.py`).

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
| `MOTION_TOOLS` (refused in cut-off) | `navigate_to_pose`, `move_relative`, `drive`, `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `arm_set_home`, `look_around`, `grasp_object`, `release_object`, `enqueue_motions` |
| `ALWAYS_ALLOWED_TOOLS` | `stop`, `cancel_motions`, `get_motion_status`, `wait_for_event`, `plan_grasp`, `get_robot_state`, `get_body_state`, `get_camera_image`, `get_map_summary`, `get_arm_state`, `acquire_control`, `release_control`, `get_topdown_view`, `remember_object`, `list_objects`, `forget_object`, `list_pois`, `add_poi`, `update_poi`, `delete_poi` |

`arm_set_home` is classed as motion because it rewrites the pose a later `arm_home` drives to.

## Arm mount and floor slow zone

### Arm mount (measured)

The arm base frame (URDF `base_link` of `so101_arm.urdf`) sits at `arm.base_in_base_link` in the robot `base_link`:
x 0.0592 m, y -0.05 m (5 cm right of the centre line), z 0.100 m, yaw 0, measured on the robot 2026-10-10: the
shoulder_pan axis is 98 mm forward and 50 mm right of the centre between the wheels (CAD), the URDF origin is 38.8 mm
behind and 62.4 mm below it, and the height comes from two tip touch-downs on the floor (contact at tool-point
`z = -0.104`, corrected 2026-10-10 to 0.100 after a deployed-tip floor check: tip FK height 6.9 mm, physically 3 mm above the floor; the floor is arm frame `z = -0.100`). Earlier estimates were 0.165 m and 0.15 m high
(15 cm forward, 4 cm right). claude_agent states the same height in its prompt (`arm_base_height_m` in its config). The pure helpers
`floor_guard.arm_to_base_link` / `base_link_to_arm` convert points (yaw supported); the grasp tools accept objects in
either frame.

### Floor slow zone (`floor_guard.py`)

'Below ground' is relative to the robot: `base_link` z = 0 is the plane under the wheels. Every arm motion through
`ArmController` (`move_joints`, `move_cartesian`, `move_path`, `set_gripper`, `home`, the grasp macros) is checked
once when its trajectory is planned: forward kinematics of every 25 Hz sample gives the fixed jaw tip
(`gripper_frame_link` origin), the tool point (fixed jaw inner face), the moving jaw tip (jaw model: the closed tip at
the tool point, rotating about the URDF `gripper` pivot), the wrist (`wrist_link` origin) and the elbow
(`lower_arm_link` origin). The effective surface at a point is the HIGHER of

- (a) the robot plane shifted to the expected surface: `base_link z = surface_z_m` (config `floor_guard.surface_z_m`,
  default 0), and
- (b) the gravity-level plane through `base_link (0, 0, surface_z_m)` from the robot tilt: the IMU `/imu/data`
  orientation (roll/pitch only, from the body monitor), ignored when older than `floor_guard.imu_max_age_s` (1 s; then
  plane (a) only, logged once per outage), or the per-call `tilt_override_deg {roll, pitch}`. With pitch p (nose down
  > 0) and roll r (left side up > 0) the level plane is `z = x tan(p) / cos(r) - y tan(r)`: a robot tilted nose down
  raises the zone in front of it.

A trajectory step whose either end has a checked point closer than `floor_guard.margin_m` (0.02 m) to that surface
(or below it) is time-scaled to `floor_guard.slow_speed_scale` (0.2) of its normal speed: it is split into `1/scale`
linear sub-steps at the same 25 Hz rate. The guard NEVER blocks a motion; the tracking-error and gripper effort/stall
detection stay the contact safety net. Results carry `slow_zone` (`slowed_samples`, `samples`, `speed_scale`,
`margin_m`, `min_clearance_m`, `lowest_point`, `surface_z_m`, `tilt_source` imu|override|none, `tilt_deg`; with
surfaces also `lowest_feature` and `surfaces`), null when the whole motion ran at normal speed.

Per-call overrides (arm motion tools, grasp tools and queue steps; nothing else changes the zone, there is no off
switch for agents): `surface_z_m` = expected surface height relative to the robot plane (e.g. -0.18 for an object on a
stair below or in a hole: normal speed down to that surface, slow below it), `tilt_override_deg` replacing the IMU tilt
and `surfaces` (below). All of them travel in `floor_guard.FloorOverride`.

### Surface regions (`surfaces.py`, pure)

One `surface_z_m` cannot describe a robot standing on the floor and an object on a stair below: the arm passes the
upper floor's edge on the way down. `surfaces` is a list (at most 16) of planar regions:

```json
{"name": "stair", "height_m": -0.10, "frame": "base_link",
 "edge": {"point": [0.28, 0.0], "direction": [0.0, -1.0], "side": "left"}}
{"name": "table", "height_m": 0.15, "frame": "arm", "polygon": [[0.2, -0.1], [0.4, -0.1], [0.4, 0.1], [0.2, 0.1]]}
```

- `height_m`: surface height relative to the robot floor (base_link z = 0), -1..1 m.
- `frame`: `arm` (default, the arm base frame) or `base_link`; regions are converted to `base_link` with the arm mount.
- Exactly one shape: `edge` = a half-plane bounded by the line through `point` along `direction`, the surface on its
  `side` (`left` default, looking along the direction; the example is the stair beyond x 0.28 m), or `polygon` = a
  convex polygon (either winding; split a concave area into several regions). A hole is a polygon below the floor.
- Outside every region `surface_z_m` applies (default the robot floor); where regions overlap the later one wins.
- Between regions of different height the boundary is a vertical step face from the lower to the higher surface. The
  clearance of a point is the smaller of its height above the local surface and its distance to the nearest step face
  (`Terrain.clearance`, `clearance_many`, `capsule_clearance`).

Slow zone: the effective surface uses the local region height plus the same tilt term, and a checked point near a step
face (within `margin_m`) is slowed like one near the surface (`lowest_feature` says which, e.g. "step edge of 'stair'
(0.10 m step)").

Planner (only with surfaces; without them plans are unchanged, the golden plans prove it):

- Every waypoint and straight-line sample is checked: the jaw points (fixed jaw tip, tool point, moving jaw tip) and the
  gripper body hull (`grasp.GRIPPER_BODY_POINTS`: housing and fixed finger collision geometry of the sim model, in
  `gripper_link`) need `surface_jaw_clearance_m` (0.003), the forearm (elbow to wrist) and wrist link (wrist to
  gripper_link) capsules (radius 0.02) need `surface_link_clearance_m` (0.015, from the sim matrix: 1.1 cm still
  touched, 1.9 cm did not). A violation rejects the candidate with a reason naming the part and the feature, e.g.
  "grasp: wrist link clearance -1.4 cm to the step edge of 'stair' (0.10 m step), needs 1.5 cm".
- The object's `support_z - gap_below_m` must match the region height under its centre within
  `surface_mismatch_tolerance_m` (0.01), otherwise the plan is infeasible with "does not match the surface under the
  object".
- An object beyond a step (a higher surface between the shoulder pan axis and the object): after the strategy's own
  candidates the planner tries each again with the pre-grasp and the lift at least `step_pre_grasp_clearance_m` (0.05)
  above the upper surface, then (angled) the steeper `step_pitches_deg` (55, 65, 75, 90) over the step, and takes the
  first that clears. A feasible plan lists the rejected tries in `rejected_candidates`. The IK budget of the strategy
  is tripled when these candidates exist.
- `ObjectSpec.surfaces` (regions described with the object) are appended to the call's `surfaces` for the plan and the
  whole grasp execution.

## Grasp macros

### Planner (`grasp.py`, pure)

Input: `ObjectSpec` `{frame: 'arm'|'base_link', x, y (centre), support_z (bottom of the object = the surface it rests
on), width_m (across the jaws), depth_m (along the approach), height_m, yaw (rad, width axis; omitted = across the
approach), gap_below_m (clear height under the object's bottom, e.g. an overhang or a raised object; default 0 = flat on
its support), surfaces (optional regions around the object, see Surface regions)}`, a strategy and `GraspParams` (the `grasp` config section with per-call `params` overrides). The 5-DOF arm
can only approach in the vertical plane of `shoulder_pan`, so every strategy approaches radially from the arm base.

| Strategy | Geometry |
|---|---|
| `scoop` | Only for an object with room under it: `gap_below_m` must be at least `jaw_thickness_m` + `scoop_gap_margin_m` (0.008 + 0.004 m), otherwise the plan is infeasible with "no gap under object for the fixed jaw" (in the sim a scoop against a box resting flat pushes or tips it instead of getting under it). Wrist roll about 0 (fixed jaw underneath, moving jaw closes from above), horizontal radial slide at the object's bottom height until the fixed jaw tip is under the object centre. The fixed jaw top goes `below_object_offset_m` under the object bottom, clamped so the jaw (`jaw_thickness_m`) stays `skim_clearance_m` above the effective surface when the object rests on it (`skim`). Opening = height + `jaw_open_margin_m` (+ how far a skimming jaw sits above the object bottom). Approach pitch starts at `scoop_pitch_deg` (0, horizontal) and steepens in `scoop_pitch_step_deg` up to `scoop_max_pitch_deg` (40; sim: every gap scoop up to 40 deg lifted, steeper ones swing the moving jaw into the object top and slip): a near-horizontal gripper cannot reach low near the base. |
| `angled` | radial approach pitched down by `approach_pitch_deg` (default `angled_pitch_deg` 45), jaws across the object width (roll from the object yaw relative to the pan direction), object centred between the jaws (grasp shift), tool point at mid-height (tall narrow objects: see below). |
| `top_down` | pitch 90 deg (straight down), jaws across the width, opening = width + margin, vertical approach. |
| `auto` | `auto_order` (default top_down, angled 45, scoop): the first feasible plan wins; `attempts` lists each try (a flat object's scoop attempt carries the no-gap reason). |

Tall narrow objects (`height_m / width_m` above `tall_ratio`, 1.5) pivot out of the jaws when gripped at mid-height:
`angled` and `top_down` put the tool point at `tall_grasp_height_fraction` (0.3) of the height from the bottom (never
lower than a jaw thickness plus `skim_clearance_m` above the surface) and the `lift` waypoint carries `lift_speed_scale`
(0.05) instead of `slide_speed_scale`; the executor runs every straight-line step at its waypoint's speed scale. Objects
narrower across the jaws (the scoop: taller) than `min_object_width_m` (0.01) are rejected: "the jaws cannot hold it".
The strategies and defaults are validated in the MuJoCo sim with the jaws calibrated to `arm.tool_offset_m`
(`sim/README.md`, "Planner scenario matrix").

New strategies plug into `grasp.STRATEGIES` (name -> function returning candidate `GraspGeometry`s) and the
`GraspStrategyName` literal in `config.py`.

Output `GraspPlan`: ordered waypoints `pre_grasp` (lifted by `pre_grasp_clearance_m` above the approach start; the
wrist roll changes here, gripper at most `limits.roll_max_gripper_open_rad`), `open` (opening for the object),
`approach` (straight line to `approach_distance_m` before the object), `grasp` (straight slide), `close`, `lift`
(`lift_height_m` up), `retreat` (`retreat_distance_m` radially back), each with the tool point target, pitch, roll,
gripper command, speed scale and IK joints; the straight segments carry joint samples every `interpolation_step_m`
(IK seeded from the previous sample). Feasibility: reachable (IK verified by FK), joints within limits minus margin,
no joint jump above `max_joint_jump_rad` between samples, no stretched-arm stall pose (shoulder_lift above
`stall_shoulder_lift_rad` 1.85 with elbow_flex at or below `stretched_elbow_max_rad` 0; negative elbow_flex stretches),
opening within `max_object_width_m` and the gripper's reach, roll guard ordering, with surfaces the surface and step
edge clearance (see Surface regions), plus slow-zone annotations per step.
Infeasible plans return human-readable `reasons`; the planner never raises for an infeasible object.

Planner speed (the RPi 5 is several times slower than a Mac; the web UI `/api/grasp` plan times out at 30 s). Every
ikpy solve costs about 3 ms and a feasible straight-line sample needs 3 (top_down) to 30 (angled, where the arm plane
heading has to be iterated) of them, so the planner avoids them where the result cannot change:

- `ArmKinematics.inverse` rejects a target beyond the summed link lengths without any search (`max_reach_m`).
- Identical ikpy queries are served from a per-instance cache (`ArmKinematics.solutions`), so re-planning the same
  object (preview, then execute) is near instant.
- `realize` solves the key waypoints (`grasp`, `retreat`, `lift`, hardest first) before it interpolates any 5 mm line,
  so an unreachable plan fails after one to three solves instead of after walking the whole path.
- Effort caps: a candidate may spend `MAX_IK_SOLVES_PER_CANDIDATE` (3200) ikpy runs and a strategy
  `MAX_IK_SOLVES_PER_STRATEGY` (8000) over all its pitches (scoop offers up to 9); beyond that the plan is infeasible
  with an "IK budget" reason instead of searching until the caller times out. The largest feasible plan in the matrix
  (angled 45 deg, 4 cm cube on a 0 m ledge at x 0.25) uses about 2700 runs.

Feasible plans keep their feasibility and strategy: `tests/test_grasp_speed.py` compares 13 of them (top_down, angled,
scoop, auto) with golden joints within 1e-6 rad (`tests/data/grasp_golden.json`, regenerated 2026-10-10 with the grid
calibration; the stair case moved from -0.25 to -0.204 and the baseline of that regeneration equals the current plans) and each case keeps its `baseline` (the plan before that change), which the current plan must match within 2e-3
rad (0.1 deg, far below the arm's backlash). The speed tests (feasible angled and auto golden scenarios and auto on the
reference 4 cm cube under 1 s, scoop 3 s, unreachable 0.2 s) are skipped with `GRASP_SPEED_SKIP=1`.

The angled cost was the arm plane heading: the approach vector needs the heading the solution ends up with (link
offsets, wrist_roll), and the fixed-point iteration converged at about 0.9 per step, so each sample needed 10 to 16
ikpy runs. `inverse_flange` now extrapolates the heading with a secant step after the first iteration and the planner
starts each straight-line sample from the previous sample's converged heading (`last_heading_bias`, reset for every
candidate so a plan never depends on an earlier one): 3 to 4 runs per sample, 150 to 180 runs for a feasible angled
plan (was 1500 to 2700). The sim matrix replays the new plans with the same results (240 scenarios, 80 feasible,
auto 34/34, top_down 19/19, angled45 18/18, scoop_gap 9/9 lifted).

Measured on a MacBook M4 with the deployed config, before -> after (4 cm cube on the floor at x 0.25 unless noted; timings
vary +-30% with machine load): top_down 0.29 -> 0.30 s, angled 45 infeasible 0.28 -> 0.08 s, angled 45 feasible (raised
2 cm at x 0.30) 4.0 -> 0.33 s, angled 45 feasible on a 0 m ledge at x 0.25 7.2 -> 0.35 s, scoop raised 2 cm at x 0.30
(feasible) 1.2 -> 0.9 s, auto 0.43 -> 0.34 s, auto falling to angled (x 0.30, ledge -0.08) 4.1 -> 0.5 s, object beyond
reach 0.00 -> 0.00 s.

### Executor (`grasp_tools.GraspExecutor`)

Runs a plan through `ArmController` (lease, stop, roll guard, limit clamping, tracking/stale aborts, slow zone): if the
roll changes and the gripper is open wider than half, it first half-opens; moves to the lifted pre-grasp keeping the
current roll, rolls there, opens to the planned opening, approaches and slides (`move_path` at the waypoint speed,
`slide_speed_scale`),
closes with `close_until_effort` using `params.grip_profile` (see Grip profiles; `close_effort_threshold` overrides the
profile's contact threshold when set; never a full squeeze: the effort/stall detection and the
gripper limits apply), verifies the grasp (jaw stopped at least `min_hold_gap_rad` short of closed and a load of at
least `hold_effort_min` at the contact or after it), then lifts (`lift_speed_scale` for tall narrow objects) and
retreats: `grasped`. A close on nothing
(`closed_no_contact`, `blocked` or no load) opens again, lifts and retreats: `missed`. Any other motion status, a
refusal or a stop (stop tool, or the service `stop`) stops and holds: `aborted` with the reason, and the default
gripper torque limit is restored. Results carry the close's `grip_profile`, `holding_load`, `slipping` and
`crush_risk`.

### Web-UI contract (`/grasp/command` -> `/grasp/result`)

The web UI (or any ROS client) publishes a JSON request as `std_msgs/String` on `topics.grasp_command`
(`/grasp/command`, reliable, depth 10) and receives the JSON answer on `topics.grasp_result` (`/grasp/result`). `stop`
and invalid requests are answered at once; `plan`, `execute` and `release` run in a worker thread (one execute/release
at a time).

Request:

```json
{"action": "plan" | "execute" | "release" | "stop",
 "request_id": "optional string, echoed back",
 "object": {"frame": "arm" | "base_link", "x": 0.22, "y": 0.0, "support_z": -0.100,
            "width_m": 0.03, "depth_m": 0.03, "height_m": 0.04, "yaw": null, "gap_below_m": 0.0},
 "strategy": "auto" | "scoop" | "angled" | "top_down",
 "params": {"lift_height_m": 0.04},
 "approach_pitch_deg": null,
 "surface_z_m": null,
 "tilt_override_deg": {"roll": 0.0, "pitch": 0.0},
 "grip_profile": "gentle" | "normal" | "firm" | {"base": "gentle", "squeeze_rad": 0.01},
 "surfaces": [{"name": "stair", "height_m": -0.10, "frame": "arm",
               "edge": {"point": [0.16, 0.0], "direction": [0.0, -1.0]}}]}
```

`object` is required for `plan` and `execute`; everything else is optional (`strategy` defaults to `auto`, `params`
keys are `grasp` config fields; `grip_profile` defaults to `grip_profiles.default_grip_profile` and may also be given as
`params.grip_profile`, an unknown profile is an error). Response:

```json
{"ok": true, "request_id": "...", "action": "execute",
 "result": {"outcome": "planned" | "infeasible" | "grasped" | "missed" | "aborted" | "released",
            "reasons": ["..."], "plan": {"strategy": "top_down", "feasible": true, "waypoints": [...], ...},
            "steps": [{"label": "pre_grasp", "status": "converged", "message": "...", "slow_zone": null}],
            "gripper_position_rad": 0.21, "gripper_effort": 350.0,
            "grip_profile": {"name": "gentle", "squeeze_rad": 0.02, "torque_limit": 250, "close_speed_rps": 0.25,
                             "target_load": 120.0, "contact_effort_threshold": 150.0, "crush_load": 220.0,
                             "capped": []},
            "holding_load": 130.0, "slipping": false, "crush_risk": false}}
```

`stop` answers `{"ok": true, ..., "result": {"arm_held": bool, "message": str}}` and aborts a running execute/release.
Errors answer `{"ok": false, "request_id": ..., "action": ..., "error": "..."}` (invalid JSON or fields, missing
object, unknown params, another action running, battery cut-off for execute/release).

The camera overlay `planned_gripper` of `get_annotated_camera_image` takes one point: pass a plan waypoint (x, y, z)
to draw it.

## Grip profiles

How hard the gripper grips is set per object (`grip.py`, config section `grip_profiles`). A profile is
`{squeeze_rad, torque_limit, close_speed_rps, target_load, contact_effort_threshold, crush_load}` in servo units
(loads and torque limit in 0.1 % of max torque; tune on the real gripper). Presets:

| Preset | torque_limit | squeeze_rad | close_speed_rps | target_load | contact threshold | crush_load | For |
|---|---|---|---|---|---|---|---|
| `gentle` | 250 | 0.02 | 0.25 | 120 | 150 | 220 | fragile, soft or light objects |
| `normal` (default) | 500 | 0.03 | 0.5 | - | 300 | 450 | ordinary objects (the squeeze, speed and threshold used before profiles) |
| `firm` | 700 | 0.06 | 0.5 | - | 400 | 650 | heavy or slippery objects, tools |

`grip_profile` is a preset name or inline overrides `{base?, <field>?...}` (base defaults to `default_grip_profile`;
the result names it `<base>+custom`). It is accepted by `set_gripper` (close_until_effort only), `plan_grasp` /
`grasp_object` (argument or `params.grip_profile`), the motion queue `gripper` step (and `grasp` steps via
`params.grip_profile`) and the `/grasp/command` contract.

A close with a profile:

1. writes the profile's `torque_limit` to the gripper servo's `torque_limit` RAM register (feetech bridge, JSON on
   `topics.follower_set_register`, `/follower/set_register`) before the jaw moves;
2. closes at `close_speed_rps` (capped to `limits.gripper_velocity_rps`);
3. stops on contact: the unchanged stall logic, `|load| >= contact_effort_threshold`, or (when `target_load` is set)
   once the load in the closing direction (`closing_load_sign` times the decoded effort) reaches `target_load`;
4. holds the stall position plus `squeeze_rad` toward closed (effort contacts hold the measured position);
5. measures the hold: two samples `hold_check_delay_s` apart; `holding_load` is the second sample's effort, `slipping`
   when the jaw closed more than `slip_threshold_rad` between them, `crush_risk` when `|holding_load| > crush_load`;
6. reports `grip_profile` (name, applied values, `capped` fields) and `torque_limit_readback`: `verified` /
   `mismatch ...` when the bridge's `/follower/servo_registers` dump (about every 10 s) arrived after the write,
   else `unverified ...` (logged).

**Hard caps**, enforced server-side whatever the profile says: `torque_limit <= torque_limit_max` (default 800 of 1000;
the register range is checked too), `squeeze_rad <= squeeze_max_rad` (default 0.08), `close_speed_rps <=
limits.gripper_velocity_rps`. Presets above a cap fail config validation; inline overrides are clamped and listed in
`capped`.

**Torque restore**: a grasp keeps its profile's torque limit while holding. The default preset's `torque_limit` is
written back after every open (after the jaw moved, so the jaw never squeezes harder before it opens), after a close
that did not grasp (`closed_no_contact`, `blocked`, any abort, an exception), when a grasp run aborts, on
`release_control`, when the lease is lost to another source, and on mcp_server startup (as soon as the bridge
subscribes `/follower/set_register`), so a crash or an unexpected end never leaves the gripper weakened or strong.

```yaml
grip_profiles:
  default_grip_profile: normal
  torque_limit_max: 800       # hard cap of every profile (0..1000)
  squeeze_max_rad: 0.08       # hard cap of every profile
  closing_load_sign: 1        # sign of the decoded gripper effort while closing on an object
  hold_check_delay_s: 0.25
  slip_threshold_rad: 0.01
  presets:
    gentle: {squeeze_rad: 0.02, torque_limit: 250, close_speed_rps: 0.25, target_load: 120, contact_effort_threshold: 150, crush_load: 220}
    normal: {squeeze_rad: 0.03, torque_limit: 500, close_speed_rps: 0.5, contact_effort_threshold: 300, crush_load: 450}
    firm: {squeeze_rad: 0.06, torque_limit: 700, close_speed_rps: 0.5, contact_effort_threshold: 400, crush_load: 650}
```

## Motion queue

Every blocking motion tool returns only when its motion is done, and each arm move used to end at zero velocity, so the
robot stood still while the model thought between steps (measured: 47 % of wall time model-only, median 3.3 s gaps).
The motion queue (`motion_queue.py`, pure scheduling logic; tools in `motion_tools.py`) keeps it moving.

**Steps.** `enqueue_motions(steps, replace)` validates every step and returns at once with `job_ids`, `queue_length`
(pending steps), `replaced` and `blend_groups`. Kinds: `arm_joints {targets}`, `arm_cartesian {x, y, z, pitch,
wrist_roll, object_width_m}` (both with `speed_scale`, `settle`, `surface_z_m`, `tilt_override_deg`, `surfaces`),
`gripper {open_fraction | close_until_effort, effort_threshold}` (also with the slow-zone fields), `base_relative {dx, dy, dyaw, precise, timeout_s}`,
`navigate_to_pose {x, y, yaw, frame, precise, timeout_s}`, `wait_s {seconds}` and `grasp {object, strategy, params,
approach_pitch_deg, surface_z_m, tilt_override_deg, surfaces}` (the `grasp_object` plan + execute through `GraspExecutor`; `grasped` succeeds, `missed` /
`aborted` / `infeasible` fail). The call is all or nothing: an unknown joint, a bad speed, a timeout above
`timeouts.nav_max_timeout_s` or an unreachable cartesian target (IK is solved at enqueue time, seeded with the previous
queued arm target, else the commanded pose) refuses the whole call with every reason (`step i (kind): reason`). At most
`motion_queue.max_steps` (32) pending steps. `replace=true` drops the pending steps first (the running one finishes).

**Execution.** One background worker thread runs the FIFO through the normal code paths: `ArmController.move_blend` /
`set_gripper` and `RobotApi.navigate` / `move_relative`, so the lease, the stop flag, the stale/tracking/effort guards,
the roll guard, critical-event interrupts and the floor slow zone apply exactly as for the blocking tools. Before every
step the worker checks for a stop issued outside the queue (`stop_count`), re-checks the battery cut-off and evaluates
the step's precondition.

**Blending.** Consecutive arm steps (`arm_joints` / `arm_cartesian`) form a blend group run as ONE continuous trajectory
(`trajectory.blend_trajectory`): quintic segments with zero acceleration at their ends and velocity continuity at the via
points (a joint's via velocity is the mean of its adjacent average slopes, zero where it changes direction, so there is
no overshoot), durations grown until every sampled per-joint velocity and acceleration stays within the velocity cap and
`limits.arm_max_joint_accel_rps2` (8 rad/s^2), sampled at the 25 Hz streaming rate. The floor slow zone time-scales every
streamed step of the spline like any other motion, and convergence is judged at the last target with its settle policy.
A step with `settle='final'` (also the default of a step naming the gripper joint), any non-arm step, a precondition
other than `none`, or a different `speed_scale` / slow-zone override ends a group: the arm comes to rest there. The
intermediate steps of a group report `step_done` when the stream passes their via point.

**Preconditions and failure policy.** `precondition {type, joints, tol, min_fraction}` is evaluated on live state when
its step is dispatched: `none`, `gripper_holding` (jaw at least `motion_queue.holding_min_gap_rad` short of closed, no
more open than `limits.gripper_grasp_max_open_rad`, gripper load at least `holding_min_effort`), `gripper_open` (open
fraction >= `min_fraction`), `arm_near` (every named joint within `tol`), `base_still` (odometry below
`base_still_linear_mps` / `base_still_angular_rps`) and `battery_ok` (fresh reading, not in cut-off). Unknown state never
passes. `on_fail`: `stop_queue` (default) drops the rest of the queue (`queue_stopped`), `skip` continues with the next
step. A failed precondition affects only its own step; steps blended behind it go back to the queue front.

**Events.** `step_started`, `step_done` (data: status, trajectory/settle seconds, tracking error, settling, residual,
slow zone), `step_failed` (guard aborts such as `aborted_tracking`, `aborted_stale`, `interrupted`, `timeout`,
`unreachable`, `blocked`, a refused call, `battery_cutoff`), `precondition_failed`, `step_skipped`, `contact` (gripper
`grasped` / `blocked`, a `grasped` grasp step), `queue_stopped`, `cancelled`, `replaced`, `stopped`, `step_aborted` (the
running step ended by cancel/stop) and `queue_empty`. The slow zone is not a failure (reported in the data). Events
carry `seq`, wall time, job id, kind and label, are logged on `mcp_server.motion_queue`, and the last
`motion_queue.event_history` (200) are kept.

**Waiting.** `wait_for_event(timeout_s, until, since_seq)` blocks only until something relevant happens: `queue_empty`
(default: drained, or a failure / precondition failure / contact / aborted step; a failure the queue skips past with
`on_fail: skip` does not end it), `step_done` (also any finished or
skipped step), `failure` (failures only) or `any`. It returns at once with reason `idle` when nothing is queued,
returns the events not returned before (`since_seq` overrides the cursor) plus the queue status and a state digest
(arm joints, gripper position / open fraction / effort, base pose, battery V, lease; unknown values null), so a separate
`get_robot_state` is rarely needed. Timeout default `wait_default_s` (30 s), capped at `wait_max_s` (120 s).
`get_motion_status` gives the same status without waiting.

**Single motion owner.** While the queue is busy (a step runs or steps are pending) the worker owns the robot's motion:
`RobotMCPServer.call_tool` refuses every tool in `QUEUE_EXCLUSIVE_TOOLS` (`MOTION_TOOLS` except `enqueue_motions` and
`arm_set_home`) with "the motion queue is running". Conversely `enqueue_motions` is refused while one of those blocking
tools runs, or while another arm motion holds the arm (a web-UI grasp or `/arm/home`). Should a blocking motion still
start in between, the arm motion lock and the base motion lock make the later one fail (`step_failed`, status
`refused`) instead of moving concurrently. `stop` calls `halt_for_stop` first (pending steps dropped at once, event
`stopped`) and then the normal stop, which ends the running step; `cancel_motions` drops the pending steps and stops the
robot when a motion step runs. The blocking tools stay available for simple single moves when nothing is queued.
When the process shuts down with a busy queue (`__main__` calls `tools.stop_queued_motion`), the queue is cleared and the
robot stopped first, so no queued Nav2 goal outlives the server.

## Gravity sag compensation

The STS3215 position servos are proportional controllers with gearbox friction: under the arm's weight a joint settles
short of its target. Measured 2026-10-10: after lifting, shoulder_lift stops 0.06-0.10 rad below the target
(`settled with residual error (shoulder_lift -0.061 rad)`), and after a top-down descent the tool point lands 9-16 mm
short in reach and lower (elbow_flex 0.03-0.07 rad). Forward kinematics reports the true (short) pose; the
compensation makes the arm land ON the target.

**Model (`sag.py`, pure).** `GravityModel` reads the link masses and centres of mass (`<inertial>`) of
`so101_arm.urdf` and computes the static gravity torque tau about shoulder_lift, elbow_flex and wrist_flex (the arm base
level, gravity -z; shoulder_pan and wrist_roll carry none). The predicted deflection is d = k * tau, saturated at
`max_rad`, in the direction gravity turns the joint. The friction band makes k depend on the final approach of the
joint: `k` after a lifting approach (the last planned motion went against gravity: full load deflection) and
`k_lowering` after a lowering one (the joint came down with gravity and stops early, about no deflection). A hold at a
measured pose (lease acquire, stop, abort, timeout) uses the middle of the band, (k + k_lowering) / 2, where the arm
neither rises nor sags.

**Controller (`ArmController`).** With `enabled: true` every published setpoint (streamed moves of `move_joints`,
`move_cartesian`, `move_path`, blended queue moves, `home`, the end-of-motion hold, the keepalive and the gripper
streams' arm joints) is `commanded = target - d(target)`:

- The approach mode of each joint comes from the planned final approach to the goal; a joint the motion does not move
  (e.g. during a gripper motion) keeps its mode. The gains ramp from the current to the goal's values along the
  motion, so a mode change never steps the command.
- The compensation is applied in URDF space inside the joint limits minus margin (`arm_limit_margin_rad` and its
  overrides): it never commands past a limit, and a target already inside the margin (a measured hold) is never pushed
  further out. shoulder_pan, wrist_roll and the gripper are never changed, so the roll guard is unaffected.
- `_last_setpoint` / `_last_target`, the tracking-error check, convergence and the settled-residual detection all use
  the UNcompensated target: a compensated move that lands on its target converges with `reached target` instead of
  reporting an overshoot, and `target` / `positions` / `residual_error` in results stay in target terms.
- The bounded residual hold keeps the compensation: after `arm_settle_hold_s` a stalled joint is held at its measured
  pose plus its compensation (not the bare measured pose), so the arm does not sag back.
- Floor slow zone: the planned samples are checked as targets (where the arm should settle) AND as over-commanded
  poses (where an arm that does not sag, e.g. load-free or resting on something, would go); each sample keeps the lower
  clearance of the two (`lowest_point` then ends in `(sag over-command)`). The grasp planner and other FK floor checks
  keep using the targets.
- Off (`enabled: false`, the code default) the controller behaves exactly as before (tested).

**Settle records.** Every arm move logs one structured line on the `mcp_server.sag` logger:
`arm_settle {"status", "settled", "compensation_enabled", "target", "commanded", "measured", "residual", "modes",
"gains"}` (rad, measured space): immediately for `settle: final` moves (converged or timeout), and for `trajectory_end`
moves by the keepalive `SETTLE_PROBE_S` (1 s) after the move returned, unless another motion started first. Gripper-only
motions and aborts log nothing. `measured - commanded` is the servo deflection under load whether or not compensation
was on, so the records keep accumulating fit data.

**Re-fit from logs.**

1. Save the journal: `ssh client.ros2.lan 'sudo journalctl -u ros2-mcp_server --since <date> --no-pager' >
   docs/calibration/2026-10-10/settle_<date>.log` (or a new dated calibration folder with a copy of the script).
2. Add the file to `journals:` in `docs/calibration/2026-10-10/sag_fit.yaml` (and update `joint_offsets_rad` if the
   offsets changed).
3. `cd nodes/mcp_server && uv run python ../../docs/calibration/2026-10-10/sag_fit.py`: fits `k` (lifting records) and
   `k_lowering` (lowering records) per joint with `sag.fit_gains` (least squares through the origin, never negative),
   scores them leave-one-group-out (`sag.cross_validate`, `sag.pool_reports`) and writes `sag_fit_results.json`.
4. Put the gains in `client.yml` (`arm.sag_compensation`) only for joints whose held-out reduction is clearly above 50%
   and whose data cover the torques the arm uses; redeploy mcp_server.

Fit 2026-10-10 (`docs/calibration/2026-10-10/sag_fit.md`): k shoulder_lift 0.141, elbow_flex 0.169, k_lowering
shoulder_lift 0.004 rad per N m; held-out residual shoulder_lift 0.067 -> 0.005 rad, elbow_flex 0.042 -> 0.007 rad, hover
tool point 12.8 -> 3.4 mm RMS. wrist_flex is not compensated (data only at 0.009-0.012 N m of a range up to 0.117 N m).
Not modelled: a payload in the gripper (adds torque the model does not know), a tilted base, temperature.

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
- **Settle policy** (`settle` on `move_arm_joints` / `move_arm_cartesian`): `final` waits for convergence (above);
  `trajectory_end` returns as soon as the last setpoint is streamed (after one more stale/tracking check) with status
  `converged`, `settling=true` while joints are still outside their converge tolerance, and `tracking_error_rad` (the
  current largest goal error). The default (`limits.arm_default_settle`, `trajectory_end`) applies to intermediate
  moves; a call that moves the gripper joint, `set_gripper`, `arm_home` and the grasp macros use `final`, and the agent
  passes `settle='final'` before a camera aiming check. Semantics: the motion lock is released on return; the goal
  stays commanded by the keepalive; joints still off target are remembered, so the next motion starts the joints it
  names from their measured state (no jump) while all other joints keep their intended targets; a stalled joint is
  relaxed to its measured pose after `arm_settle_hold_s` like a settled residual; the marker is cleared by release or
  a lost lease. Expected saving per move: the measured settle phase (about 3.3 s median of 5.5 s per converged move)
  minus the short tail the servos need anyway; the model runs while the arm finishes closing in.
- **Convergence timeouts** (54 of the measured moves ran into the 3 s timeout, median 9.6 s): two causes in the code.
  The gravity-loaded `shoulder_lift` / `elbow_flex` sag by more than the global 0.03 rad converge tolerance (and past
  0.08 rad when stretched), so they never counted as converged or settled; and the gripper joint was judged like an arm
  joint, so a jaw holding an object (stalled short of its target) timed out every arm move that named it. Fixes:
  per-joint tolerances (`arm_converge_tolerance_overrides` default 0.05 and `arm_settle_tolerance_overrides` default
  0.12 for those two joints, validated to stay between the converge tolerance and the tracking-error abort), and the
  gripper is not judged when other joints move too (the result message names the unjudged error). Stale-feedback and
  tracking-error aborts are unchanged. Every result carries `trajectory_s`, `settle_s` and `settle`.
- **Timing log**: `call_tool` logs one line per call on the `mcp_server.timing` logger,
  `tool_call {"tool", "started_at", "ended_at", "duration_s", "ok", ...}`; arm moves add `status`, `trajectory_s`,
  `settle_s`, `settle`, `settling`, `tracking_error_rad`, so trajectory and settle seconds are separable in the journal.
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
  `arm_settle_window_s` while the commanded jaw stayed at least `gripper_stall_lead_rad` (0.15) ahead of it). The
  real jaw needs up to about 0.9 s to break away from rest under the slowly ramping close command and reports a high
  load meanwhile; without the lead condition that start-up stillness counted as a stall and an empty close returned
  `blocked` after 0.000 rad travel.
- **Follower gripper command range**: direct (autonomy / web_ui) gripper commands are clamped by the feetech
  follower bridge to the servo command range (`command_min_steps: 1900` in `client.yml`, max read from the servo
  register; gripper not inverted). The closed target -0.165 rad is 2048 + (-0.165 / 0.001534) = 1940 steps, inside
  that range (the measured stop -0.172 rad is 1936 steps). The servo EEPROM angle limits are not changed by this node.
- **Grasp from stall**: when closing (`close_until_effort` or `open_fraction=0`) the jaw settles before the closed
  position (residual outside the converge tolerance), the result is `grasped` with "contact inferred: jaw stalled
  ..." and the gripper holds the stall position plus the grip profile's `squeeze_rad` (normal 0.03) toward closed (never past
  closed) instead of squeezing to the full closed target. Both this and an effort contact must pass the closure check
  (`gripper_grasp_min_travel_rad` from the start, at most `gripper_grasp_max_open_rad` open), else `blocked`.
- **Gravity sag compensation** (optional, `arm.sag_compensation`): with a URDF mass model and per-approach gains the
  published setpoints lead the target against gravity so the arm settles on it; see "Gravity sag compensation".
- **No placeholder data**: nothing is published without fresh measured joint states; state tools omit stale sources.
- **Floor slow zone**: every arm motion is checked once against the effective surface (robot plane, IMU level plane,
  per-call `surface_z_m` / `tilt_override_deg`); steps near or below it run at `floor_guard.slow_speed_scale`. It never
  blocks (see Arm mount and floor slow zone); the tracking-error and effort/stall checks remain the contact safety net.

## Code layout

| Module | ROS? | Purpose |
|---|---|---|
| `config.py` | no | pydantic config, `MCP_SERVER_CONFIG`, `MCP_SERVER_TOKEN` |
| `trajectory.py` | no | quintic interpolation, limit clamping, tracking error, `blend_trajectory` (velocity-continuous spline through via points) |
| `ik.py` | no | URDF limits, ikpy FK/IK with verification |
| `arm.py` | no | `ArmController`: lease, streaming, aborts, gripper, home, `move_path`, `move_blend`, `solve_cartesian`, slow-zone time scaling |
| `sag.py` | no | gravity sag model: URDF gravity torques, deflection k * tau per approach mode, limit-band compensation, gain fit, leave-one-out validation, `arm_settle` record parsing |
| `floor_guard.py` | no | arm mount conversions, effective surface (robot plane, IMU level plane, surface regions, overrides), jaw model, per-sample slow zone, `retime` |
| `surfaces.py` | no | surface regions (half-plane / convex polygon), local height, step faces, point / capsule clearance |
| `grasp.py` | no | grasp planner: strategies registry, waypoints, straight-line IK samples, feasibility reasons, surface / step-edge checks |
| `grasp_tools.py` | no | `GraspExecutor`, the grasp MCP tools, `GraspService` (web-UI JSON) |
| `grip.py` | no | grip profiles: resolve a preset name / inline overrides to a capped `GripProfile` |
| `base_motion.py` | no | timed clamped drive loop, continuous spin with marks (`run_spin`), stop sequence (`run_stop`) |
| `motion_queue.py` | no | motion queue: step models, blend groups, preconditions, worker thread, events, waits |
| `motion_tools.py` | no | `enqueue_motions`, `get_motion_status`, `cancel_motions`, `wait_for_event` (one `TOOL_MODULES` entry) |
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
  arm_base_height_m: 0.100   # arm mount plane height above the floor (m, corrected 2026-10-10); floor_z_m = -this
  gripper_open_rad: 1.5       # follower gripper joint positions (rad)
  gripper_closed_rad: -0.165
  reach_outer_m: 0.25        # floor reach around the shoulder axis (annotated image annulus)
  reach_inner_m: 0.05
  joint_offsets_rad: {shoulder_pan: 0.0, shoulder_lift: 0.0, elbow_flex: 0.0, wrist_flex: 0.0, wrist_roll: 0.0}   # urdf = measured + offset
  tool_offset_m: {x: 0.0, y: 0.0, z: 0.0}   # jaw closing point in the gripper_frame_link frame
  jaw_open_axis: [-1.0, 0.0, 0.0]           # direction the moving jaw opens, gripper_frame_link (normalised)
  joint_limit_overrides_rad: {}             # e.g. {shoulder_lift: [-1.74533, 2.6]} replaces URDF limits
  base_in_base_link: {x: 0.0592, y: -0.05, z: 0.100, yaw: 0.0}   # measured (default); z = arm_base_height_m; null disables
  sag_compensation:                         # gravity sag compensation (default off; see "Gravity sag compensation")
    enabled: false
    k: {}                                   # rad per N m after a lifting approach, e.g. {shoulder_lift: 0.141}
    k_lowering: {}                          # rad per N m after a lowering approach
    max_rad: 0.12                           # saturation per joint (at most 0.2)
floor_guard:              # below-surface slow zone (see Arm mount and floor slow zone)
  enabled: true
  margin_m: 0.02
  slow_speed_scale: 0.2
  surface_z_m: 0.0
  imu_max_age_s: 1.0
grasp:                    # grasp planner defaults, overridable per call via params (see Grasp macros)
  approach_distance_m: 0.04
  pre_grasp_clearance_m: 0.05
  slide_speed_scale: 0.15
  lift_height_m: 0.05
  retreat_distance_m: 0.05
  jaw_thickness_m: 0.008
  jaw_open_margin_m: 0.015
  below_object_offset_m: 0.005
  skim_clearance_m: 0.003
  max_object_width_m: 0.08
  min_object_width_m: 0.01      # narrower objects are rejected (the jaws cannot hold them)
  close_effort_threshold: null  # null = the grip profile's contact_effort_threshold
  grip_profile: null            # null = grip_profiles.default_grip_profile
  hold_effort_min: 100
  min_hold_gap_rad: 0.08
  interpolation_step_m: 0.005
  max_joint_jump_rad: 0.25
  scoop_pitch_deg: 0
  scoop_max_pitch_deg: 40
  scoop_pitch_step_deg: 5
  scoop_gap_margin_m: 0.004     # scoop only when gap_below_m >= jaw_thickness_m + this
  tall_ratio: 1.5               # height / width above this: tall narrow object
  tall_grasp_height_fraction: 0.3
  lift_speed_scale: 0.05        # lift speed of tall narrow objects
  angled_pitch_deg: 45
  stall_shoulder_lift_rad: 1.85
  stretched_elbow_max_rad: 0.0
  surface_jaw_clearance_m: 0.003     # with surfaces: jaw points and gripper body vs surfaces / step faces
  surface_link_clearance_m: 0.015    # with surfaces: forearm and wrist link capsules vs surfaces / step faces
  surface_mismatch_tolerance_m: 0.01 # support_z - gap_below_m vs the region under the object
  step_pitches_deg: [55, 65, 75, 90] # angled pitches tried for an object beyond a step
  step_pre_grasp_clearance_m: 0.05   # pre-grasp and lift above the upper surface of a step
  release_open_fraction: 0.6
  release_lift_m: 0.05
  auto_order: [{strategy: top_down}, {strategy: angled, approach_pitch_deg: 45}, {strategy: scoop}]
cameras:                  # default: not calibrated (see Camera tools and calibration)
  calibration_dir: /var/lib/ros2/camera_calibration
  gripper: {parent_frame: gripper_link, intrinsics: null, mount: null}
  front: {parent_frame: base_link, intrinsics: {hfov_deg: 66.0, width: 640, height: 480}, mount: null}
limits:
  arm_max_joint_velocity_rps: 1.0
  arm_max_joint_accel_rps2: 8.0      # acceleration cap of blended (queued) arm trajectories
  roll_guard_min_change_rad: 0.1     # wrist_roll changes above this are refused ...
  roll_max_gripper_open_rad: 0.8     # ... while the gripper is open wider than this (about half open)
  arm_settle_tolerance_rad: 0.08   # steady-state error reported as residual_error instead of a timeout
  gripper_velocity_rps: 0.5         # fastest gripper close (caps grip profile close_speed_rps)
  gripper_effort_ignore_s: 0.3      # ignore the load spike when the motor starts
  gripper_contact_travel_rad: 0.03  # jaw travel (or a stall) required before effort counts as contact
  gripper_stall_lead_rad: 0.15      # a stall only counts while the command leads the jaw by this much
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
look_around: {default_captures: 4, clearance_margin_m: 0.10, step_timeout_s: 30, mode: spin, spin_speed_rps: 0.4,
              spin_rate_hz: 20, spin_timeout_margin_s: 10, return_to_start: false}
motion_queue: {max_steps: 32, event_history: 200, status_events: 10, wait_events: 40, wait_default_s: 30,
               wait_max_s: 120, poll_s: 0.05, base_still_linear_mps: 0.02, base_still_angular_rps: 0.05,
               holding_min_effort: 100.0, holding_min_gap_rad: 0.05}
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
uv sync    # also installs the shared `ros2-common` package (path dependency ../../shared, develop mode)
uv run pytest tests -q   # rclpy-free unit tests (rclpy is only imported by ros_iface.py / __main__.py)
uv run poe lint          # ruff check, ruff format --check, vulture
```

Deploy: `./scripts/deploy-nodes.sh client mcp_server` (a `--tags mcp_server` run of `ansible/playbooks/deploy_nodes_client.yml`).
