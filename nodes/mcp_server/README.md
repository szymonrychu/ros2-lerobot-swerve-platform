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
| `get_camera_image(camera, max_px<=1024)` | Default size `limits.default_camera_px` (384 px on the longest side; ask for more only to read detail, images fill the context: 640x480 is about 400 tokens). Waits for the next frame on a persistent per-camera subscription (created at startup, never destroyed: per-call subscriptions raced the executor and killed it), with timeout. `gripper`: `/camera_0/image_raw/compressed` (JPEG passed through or downscaled once); `front`: `topics.front_camera` (default `/overview_camera/image_raw/compressed`, 640x480 overhead Camera Module 3 looking down at the front of the robot, the arm and the floor in front; JPEG passed through or downscaled once, best view for judging gripper-to-object position). Returns MCP image content (JPEG) + metadata: capture stamp, the picture's `width` x `height`, `calibrated_image` (the size the pixel tools work in, 640x480 here), `scale_to_calibrated` and a `pixel_note` saying to pass `image_width` / `image_height` with pixels read from this picture, and `lookup_id` (the picture as recorded in the lookup store, null when the write failed; see [Camera lookups](#camera-lookups)); error if no frame within `image_timeout_s` or the frame is older than 1 s. |
| `list_lookups(since, until, near{x,y,radius_m}, poi_id, text, camera, mcp_session, limit<=20)` | Past camera images, metadata only, newest first (default 10). Sensor. See [Camera lookups](#camera-lookups). |
| `get_lookup_image(lookup_id, max_px)` | One recorded image (JPEG, exactly as the model received it; `max_px` only downscales) plus its metadata. Sensor. |
| `annotate_lookup(lookup_id, note, poi_ids)` | Set a lookup's note and/or POI links (replacing them; omitted fields stay); rewrites its index line. Sensor. |
| `get_body_state` | Body vitals (sensor, always allowed): per-servo latest temperature / load / current / voltage / status flags with data age, hottest servo, battery V, per-cell V, margin to cut-off and cut-off state, IMU roll/pitch/tilt and last bump, wheel slip residual, commanded vs measured base speed, CPU temperature and firmware throttling flag, active source + lease, last 10 events. Missing data is `null` with a reason in `notes`. |
| `get_map_summary(include_png, radius_m)` | Nearest `/scan_filtered` obstacle in 8 sectors around base_link, `/map` size and known/occupied/free cells, robot pose, optional small PNG of the map around the robot. |
| `navigate_to_pose(x, y, yaw, frame='map', timeout_s, precise=false)` | Nav2 `NavigateToPose`; by default the goal ends early (cancelled, base zeroed, status `succeeded`) as soon as the measured pose is within `nav.intermediate_xy_tolerance_m` / `nav.intermediate_yaw_tolerance_deg` (3 cm / 5 deg); `precise=true` waits for Nav2's own checker (`nav.goal_xy_tolerance_m` / `goal_yaw_tolerance_deg`, 1 cm / 2 deg, slower). Blocks until result/timeout (goal cancelled on timeout or stop); returns result and final pose. The description states the goal precision from `nav.goal_xy_tolerance_m` / `nav.goal_yaw_tolerance_deg` (default 1 cm / 2 deg, keep equal to the Nav2 goal checker) and that a sideways goal first turns the robot toward the path (front leading) and turns back to the goal heading at the end. |
| `move_relative(dx, dy, dyaw, timeout_s, precise=false)` | Same (incl. `precise`), goal given in base_link (converted to a map goal via TF). Small moves (a few cm) really move. |
| `drive(vx, vy, wz, duration_s<=2)` | 20 Hz on `/cmd_vel_nav` (through velocity smoother + collision monitor), clamped to 0.25 m/s / 0.5 rad/s, then zero. |
| `stop` | Always available: clears the motion queue first (dropped job ids in `motion_queue_dropped`), cancels all NavigateToPose goals and publishes a zero twist. Aborts any arm motion and holds the arm at its measured pose only if this server holds arm control (or a motion is running); otherwise the arm is not touched (`arm_held: false`). |
| `get_arm_state` | Joint positions (measured follower values)/efforts, gripper effort, tool point pose (x, y, z, pitch; forward kinematics with `arm.joint_offsets_rad` applied), `floor_z_m` (floor height in the base_link frame), active source, lease, home stored. |
| `take_control(subsystem: arm\|base)` | Take a control lease (see Control leases); moves nothing, a no-op when already held. `arm` starts the autonomy lease (publishes the measured pose on `/filter/autonomy_joint_commands`; needs fresh joint states); `base` sets the mcp_server-internal base lease. Returns `ControlState`. Allowed under battery cut-off and at any tilt level. Replaces `acquire_control`. |
| `release_control(subsystem: arm\|base\|all = all, keep_position=false, carried_object_radius_m: float\|None = None)` | Release leases, parking first by default: arm drives home (the `arm_home` routes) then publishes the release (`std_msgs/Bool` true on `/filter/autonomy_release`); base cancels Nav2, zeroes the twist and waits for neutral steering. Always releases; returns `ReleaseResult {released, reset, state}`. Refused while the motion queue runs, and parking the arm is refused while the gripper carries an object. An arm release with `keep_position=true` publishes the dynamic navigation footprint (static outline + arm shadow + optional carried circle of `carried_object_radius_m`, 0..0.3 m, keep_position only; it then follows the released arm), see [Navigation footprint](#navigation-footprint). |
| `get_control_state` | Both leases (sensor): `arm {held, held_since, at_home, home_stored, active_source, lost}`, `base {held, held_since, steering_neutral}`. |
| `move_arm_joints(targets, speed_scale<=0.5, settle=None)` | `settle`: `trajectory_end` (default; `final` when the call moves the gripper joint, `limits.arm_default_settle`) or `final`, see the safety model.  Interpolated motion to joint targets (follower joint radians, as in `get_arm_state`). Unnamed joints keep their last commanded target. `speed_scale` 0.5 is the maximum, `limits.arm_max_joint_velocity_rps` (default 1.0 rad/s); the description states the configured value. A `wrist_roll` change above `limits.roll_guard_min_change_rad` is refused while the gripper is open wider than `limits.roll_max_gripper_open_rad` (see Wrist roll guard). `converged` results may carry `residual_error` (see Safety model). |
| `move_arm_cartesian(x, y, z, pitch=None, frame='base_link', speed_scale, wrist_roll=None, object_width_m=None, settle=None)` | `settle` as for `move_arm_joints`.  ikpy IK on `nodes/web_ui/urdf/so101_arm.urdf` (5-DOF: position + approach pitch); `wrist_roll` (rad, measured space, clamped to the limits; clamping is listed in `clamped`) is the roll the IK keeps for this target and the motion rolls to, omitted = the current roll is kept; `object_width_m` (m, 0 < w <= 0.08) makes (x, y, z) the OBJECT CENTRE (see Grasp shift); `unreachable` is reported, never guessed. `base_link` here is the arm URDF root (arm mount, z = 0). The floor is at `z = -arm.arm_base_height_m` (default 0.104, measured 2026-10-10; see Arm mount); the tool descriptions and `get_arm_state.floor_z_m` state it. Motions near or below the effective surface are slowed (never blocked) by the floor slow zone; `surface_z_m`, `tilt_override_deg` and `surfaces` (also on `move_arm_joints`, `set_gripper` and `arm_home`; see Surface regions) shift it per call and results report `slow_zone`. |
| `set_gripper(open_fraction | close_until_effort, effort_threshold, grip_profile, surfaces)` | Open to a fraction (0 closed, 1 open) or close with a grip profile (see Grip profiles) until `abs(effort) >= threshold` (default: the profile's `contact_effort_threshold`) or the closing load reaches the profile's `target_load` (then hold: `grasped`, else `closed_no_contact`). A stall or effort contact only counts as `grasped` when the jaw closed at least `limits.gripper_grasp_min_travel_rad` (0.10 rad; was 0.15 until a 39 mm jar was refused at 0.147 rad on 2026-10-10) from where it started AND stopped no more open than `limits.gripper_grasp_max_open_rad` (1.2 rad; a nearly open jaw that stalls is pushing on something); otherwise the status is `blocked` with "jaw stopped at X rad after Y rad travel - likely pressing on an object rather than holding it", and the measured jaw position is held (no squeeze). A stall or contact within `limits.gripper_empty_stall_rad` (0.08 rad, about 6 mm) of closed is an empty jaw: `closed_no_contact` with "closed without contact: the jaw stopped X rad before closed ...", never `grasped` (2026-10-10: the empty firm close stalled 0.055 rad before closed at load 148). `arm.gripper_closed_rad` / `gripper_open_rad` are follower gripper joint positions (defaults -0.165 / 1.5 rad; measured fully closed is -0.172 rad, URDF lower limit -0.1745, so the jaws close fully). With `close_until_effort` the load is ignored for `gripper_effort_ignore_s` (0.3 s, motor start-up spike) and then counts only once the jaw moved `gripper_contact_travel_rad` (0.03) or stalled with the command at least `gripper_stall_lead_rad` (0.15) ahead of it (or already at the close target: a jaw stopped less than 0.15 rad before closed can never see a larger lead). Stalls are classified by position: a jaw that stalls (that stall criterion: it moved less than `arm_settle_motion_rad` over `arm_settle_window_s`) at least `gripper_empty_stall_rad` from closed is a contact even when its load stays below `target_load` and its error exceeds the settle tolerance (an object loading the jaw lightly never settles within it): the close ends at once and the stall becomes `grasped` (hold = the stalled position plus the profile's `squeeze_rad` toward closed) or `blocked` by the travel/open rules above; a stall inside the empty-jaw band stays `closed_no_contact`. Until 2026-10-11 such a stall ended `timeout` and `grasp_object` aborted. A gripper-only motion near the floor or the robot body is slowed only by what moves (see Floor slow zone and Body collision): the 2026-10-11 10:27 close (jaw tip 12.5 mm above the floor, the stationary upper arm 14.9 mm from the lidar) ran 325 ticks instead of 64. The arm joints keep being held at their intended targets while the gripper moves (see Safety). |
| `plan_grasp(object, strategy='auto', params, approach_pitch_deg, surface_z_m, tilt_override_deg, grip_profile, surfaces)` | Dry-run grasp plan (sensor, no motion): outcome `planned` / `infeasible` with reasons, the resolved `grip_profile` (an unknown profile is refused), chosen strategy, pitch, wrist roll, jaw opening, waypoints and slow-zone annotations. See Grasp macros. |
| `grasp_object(object, strategy='auto', params, approach_pitch_deg, surface_z_m, tilt_override_deg, grip_profile, surfaces)` | Plan and execute (motion): outcome `grasped` / `unverified` / `missed` / `aborted` / `infeasible` (`unverified` also attaches a fresh gripper camera image, max 640 px, as an image content item), with the executed steps (each with the `residual_error` of arm joints off their intended target), the final gripper position/load, the grip report (`grip_profile`, `holding_load`, `slipping`, `crush_risk`, `retightened`; see Grip profiles), `residual_error` and `warnings` (joints off their target beyond their converge tolerance) and `held_pose` (measured jaw centre and tool point against the last waypoint). See Grasp macros. |
| `release_object(params, surface_z_m, tilt_override_deg, surfaces)` | Open to `release_open_fraction` and lift `release_lift_m` straight up (motion): outcome `released` / `aborted`. The way out of an `unverified` grasp whose image shows empty jaws (then retry). |
| `arm_home` / `arm_set_home` | Move to / store the home pose. `arm_home` needs the arm lease and keeps it afterwards (also after a failed motion); the `/arm/home` service (web UI Home button) still always releases. |
| `pixel_to_ground(camera, u, v, surface_height_m=0.0, image_width, image_height)` | Point seen at a pixel (sensor). **u, v are pixels of the calibrated image size** (640x480; `get_annotated_camera_image` and `mark_candidate_points` images) unless `image_width` / `image_height` give the size of the image the pixel came from: `get_camera_image` returns 384x288 by default, and an unscaled small-image pixel lands at the wrong place (2026-10-10). Both sizes are needed and must keep the calibrated aspect (a crop is refused); the result then also reports `pixel_input` (what was passed and the scale). Returns `pixel` (calibrated size), `image` (calibrated size), `surface_height_m`, `ground_base_link`, `ground_map` (when the map pose is known), `distance_from_base_m`, `bearing_deg`, `method`, `uncertainty_note`. `surface_height_m` (-0.5..0.5) is the height of the surface the pixel lies on relative to the robot's floor (positive above, negative below; top of a 3 cm box 0.03, a floor 10 cm lower -0.10): the ray is intersected with the plane at floor + that height and the returned z is on it. Error `camera <name> not calibrated: ...` until intrinsics and mount are configured. See [Camera tools and calibration](#camera-tools-and-calibration). |
| `get_annotated_camera_image(camera, overlays=['grid'], planned_gripper, grid_step_m=0.1)` | JPEG with metric overlays (`grid`, `reach`, `gripper`, `planned_gripper`, `lidar`) plus metadata. |
| `mark_candidate_points(camera, region, spacing_px=40, max_points=40, surface_height_m=0.0, image_width, image_height)` | Image (calibrated size) with numbered dots on a pixel grid plus a table `{set_id, surface_height_m, image {width, height}, points:[{n, u, v, ground_base_link{x,y,z}, ground_map}]}`; the points are intersected with the plane at floor + `surface_height_m` (as in `pixel_to_ground`); the last 10 sets are kept. `region` is in calibrated pixels unless `image_width` / `image_height` give the size it was read in. |
| `resolve_candidate(set_id, n)` | Stored coordinates of one numbered point (its u, v in the calibrated `image` size), with its age and whether the base/arm moved since. |
| `capture_calibration_sample(camera, u, v, ground_x, ground_y, ground_z=0.0, image_width, image_height)` | Store one marker sample (u, v scaled from `image_width` x `image_height` to the intrinsics' size when given; needs the intrinsics) (pixel + measured floor point + parent-link pose + the raw measured arm `joints` when fresh joint states exist) in `cameras.calibration_dir`. |
| `solve_camera_calibration(camera, initial)` | Fit the mount pose to the stored samples; returns `rms_px` and a YAML snippet for `client.yml` (never edits the config). |
| `clear_calibration_samples(camera)` | Delete the stored samples of a camera. |
| `get_topdown_view(radius_m=2.5, layers=all, px=480)` | Robot-up PNG centred on the robot (see Perception and memory) plus metadata `pose`, `scale_m_per_px`, `layers_present`, `layers_missing` (reason each), `data_ages`. Sensor. |
| `remember_object` / `list_objects` / `forget_object` | Object memory in map coordinates, stored as object POIs in poi_store (merge, distance/bearing from the robot). Sensor. |
| `look_around(captures=4, camera='front', mode=None, return_to_start=None)` | Effector, ONE motion call: in-place turn through equal headings with a camera frame and lidar summary per heading. `mode` `spin` (default, `look_around.mode`): one continuous slow rotation; `steps`: one precise Nav2 rotation per heading. Ends facing the last heading unless `return_to_start=true` (default `look_around.return_to_start`, false). |
| `enqueue_motions(steps, replace=false)` | Effector: queue motion steps and return at once (job ids, queue length, blend groups). See [Motion queue](#motion-queue). |
| `get_motion_status` | Queue snapshot (sensor): running, current step (elapsed, phase, planning_s, progress, blend group), pending steps, last events. |
| `cancel_motions` | Uncapped control: drop the pending steps and abort the running one (base zeroed, arm held when a motion step runs). |
| `wait_for_event(timeout_s, until='queue_empty', since_seq)` | Sensor: block until the queue drains or something relevant happens; returns the new events, the queue status and a compact state digest. |
| `list_pois` / `add_poi` / `update_poi` / `delete_poi` | Points and areas of interest through `poi_store` (`/poi/*`). Sensor. Areas can carry a Nav2 zone: `add_poi` / `update_poi` take `zone` (`none` / `keepout` / `slow`, `none` removes it) and `speed_limit_pct` (int 10..100, required for `slow`, refused for any other zone); `list_pois` returns both on every view. `keepout`: Nav2 will not plan or drive into the area (the robot can leave if it is inside; never set a goal inside). `slow`: MPPI speed cap while the robot centre is inside. A zone is active only while the POI status is `open`. A zone on a point, `slow` without a percentage, or a percentage with another zone is refused before the request. Agent zones are session-scoped (agent POIs are cleared at each new session). See [POI zones](#poi-zones). |

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
- Image sizes: the pixel tools work in the calibrated image size (the intrinsics' width x height, 640x480 for both
  cameras). `get_camera_image` defaults to 384x288 (`limits.default_camera_px`), so a pixel read from it must be passed
  with `image_width` / `image_height` (scaled by `camera_scene.pixel_scale`; both or neither, same aspect within 2 %).
  Without them the pixel is taken as a calibrated-size pixel (the behaviour before 2026-10-10, kept for pixels from
  `get_annotated_camera_image` / `mark_candidate_points`). The claude_agent prompt tells the agent to always pass the
  size or to pick pixels on the annotated image.
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
  is extended by `(w / 2 + c) * arm.jaw_open_axis` (a unit vector in `gripper_frame_link`, default `[-1, 0, 0]`: the moving jaw
  opens toward -x; normalised on load), with `c = max(grasp.jaw_open_margin_m / 2, grasp.fixed_jaw_clearance_m)` (7.5 mm by
  default), the same clearance a planned centred grasp uses. The tool point (fixed jaw face) therefore ends w/2 + c from the
  centre against the opening direction, clear of the object side face, the object centred between the jaws (before
  2026-10-10 JC1 it was w/2: the fixed jaw on the side face, 0 mm). `ArmKinematics.forward` / `inverse` take a per-call `extra_offset` (the shared
  `tool_offset` is never modified; `ik.grasp_offset` builds it). The result carries `grasp_shift`
  `{object_width_m, shift_m, fixed_jaw_clearance_m, jaw_open_axis, tool_point}` (`tool_point` = the fixed jaw point reached, arm base frame);
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
`/odom_rf2o_twist`, `/cmd_vel_nav`, `/collision_monitor_state`, `/filter/active_source`, `/tilt_guard/state`, the battery guard and the CPU
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
| `arm_tracking_abort` | warning | arm: setpoint vs measured tracking error of a joint (named in the event) above `limits.arm_tracking_error_rad` + `limits.arm_tracking_lag_s` x the motion's velocity, measured against the setpoint commanded `limits.arm_tracking_delay_s` earlier; never interrupts the model's turn or a base motion (not in `BASE_INTERRUPTS`) |
| `wheel_slip` | warning | swerve residual (decoded from the twist covariance `var_xy = 0.002 + r^2`; parked fixed value = none) > `slip_residual_warn_mps` 0.1 |
| `bump` | warning / critical | horizontal acceleration spike after baseline removal >= `bump_warn_mps2` 4 / `bump_critical_mps2` 9 m/s^2 for `bump_min_samples` (2) consecutive IMU samples; one glitchy sample is ignored and the severity follows the weakest sample of the run |
| `tilt_warning` (source `tilt_guard`) | critical | `/tilt_guard/state` level entered `warning` (also on the way back down from `error`); see [Tilt guard](#tilt-guard) |
| `tilt_error` (source `tilt_guard`) | critical | `/tilt_guard/state` level entered `error` (base drive inhibited by swerve_drive_controller) |
| `tilt_drive_enabled` (source `tilt_guard`) | info | `base_drive_inhibited` released while the level is still `warning` (manual `drive` works again) |
| `tilt_clear` (source `tilt_guard`) | info | `/tilt_guard/state` level back to `ok` |
| `tilt_nav_cancel_failed` (source `tilt_guard`) | critical | the Nav2 goal cancel at a warning/error entry was not answered after `tilt_cancel_attempts` (3) tries `tilt_cancel_retry_s` (1 s) apart: a goal may still drive the tilted robot, call `stop` |
| `tilt` | warning | IMU fallback only while the tilt guard is silent (no `/tilt_guard/state` for `tilt_guard_stale_s` 3 s): tilt from the IMU orientation > `tilt_warn_deg` 10 deg |
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
16 sessions are tracked (LRU), calls without a session id share one cursor; newest `digest_max_events` 20 kept) and `vitals` (`battery 11.40 V, hottest servo 41 C (elbow_flex), CPU 52 C, ctl arm=H@home base=-`,
`n/a` when unknown; the tilt guard when known; `ctl` is the control lease summary: `H` held, `-` not held, `@home` /
`@away` when the arm's home position is known, see Control leases) as a text block and, when the result is structured, as structured-content keys (image tools stay
unstructured: text block plus `_meta`). Tool errors get the same JSON appended to the message. The tool output schemas do
not list these two keys.

**Early return.** `navigate_to_pose`, `move_relative`, `drive` and the arm tools (`move_arm_joints`, `move_arm_cartesian`,
`set_gripper`, `arm_home`) end early with status `interrupted` and `interrupted_by` = the critical event type fired during
the call: base `collision_stop`, `stall`, `battery_cutoff`, `overheat`, `cpu_overheat`, `servo_error`, `human_takeover`,
`bump`, `tilt_warning`, `tilt_error`; arm `overheat`, `cpu_overheat`, `servo_error`, `battery_cutoff`, `human_takeover` (the lease is dropped, nothing is
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
no CPU cap but runs in the agent-facing `CPUWeight=300` tier, so it yields CPU to the robot nodes).

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

## Camera lookups

Every successful `get_camera_image` and `get_annotated_camera_image` call is recorded automatically
(`lookup_store.py`, `lookup_tools.py`): the exact JPEG the model received (the annotated image for the annotated tool)
goes to `<lookups.store_dir>/<id>.jpg` and one JSON line (`LookupRecord`) is appended to `index.jsonl` in the same
directory (default `/var/lib/ros2/poi/lookups`, next to poi_store's `poi.json`, created by Ansible; poi_store itself
never reads it). The camera tool's metadata gains `lookup_id`, so the agent can link the picture to a POI right away.

| Field | Meaning |
|---|---|
| `id`, `ts`, `tool` | uuid hex, unix time of the record, `get_camera_image` / `get_annotated_camera_image` |
| `camera`, `width`, `height`, `file`, `bytes` | camera, image size (px), JPEG file name relative to the store, JPEG size |
| `base_pose_map` | `{x, y, yaw}` in map at capture, null when the pose is unknown |
| `arm_joints` | measured arm joints (rad), null when stale |
| `camera_pose_base` | `{xyz, rpy}` of the camera body in base_link (fixed-axis rpy), when the camera has a mount (the gripper camera also needs `arm.base_in_base_link` and fresh joints); null otherwise |
| `mcp_session` | Mcp-Session-Id of the call (a contextvar set in `RobotMCPServer.call_tool_with_digest`; empty without one). mcp_server cannot see claude_agent's instruction id: time range plus session stand in for it |
| `note`, `poi_ids` | empty until `annotate_lookup` sets them |

Retention is oldest first against three limits: `lookups.max_records` (3000), `lookups.max_bytes` (300 MB of JPEG) and
`lookups.max_age_days` (14). Pruning rewrites the whole index, so it only starts once a limit is exceeded by 5% and then
drops back to the limit itself (no rewrite per camera call at the limit). The record just written is never pruned. JPEGs
without an index line (crash between the image write and the index append) are removed on the first write of a process
and with every prune rewrite, unless younger than 60 s. A
session reset does not clear lookups (history is the point). Index lines that do not parse are logged once per distinct line and skipped;
`annotate_lookup` rewrites only its own line (fsynced temp file, mode 0644, + rename) and not at all when nothing changes.
An append first terminates a trailing line a crash left without newline, then fsyncs. The record, query and index reader
types are shared with web_ui in `shared/ros2_common/lookups.py`; the parsed index is cached until the file changes. A write failure (full disk, missing
directory, permissions) is logged on `mcp_server.lookups` and counted in `mcp_lookup_write_failures_total`; it never
fails the camera tool, whose `lookup_id` is then null.

`list_lookups` filters combine with AND: `since` / `until` (inclusive; unix seconds or an ISO-8601 time like
`2025-10-09T08:53:20Z`, naive means UTC), `near` (map distance of
`base_pose_map`; records without a pose never match), `poi_id` (member of `poi_ids`), `text` (case-insensitive
substring of `note` only; notes are empty until `annotate_lookup` sets them, so an empty result of a `text` search carries a
`hint`), `camera`, `mcp_session`; `limit` 1-20 (default 10). Every returned field is named in the tool description.
`annotate_lookup` rejects `poi_ids` that are not existing POIs (checked against poi_store via `/poi/list`); when poi_store
is not running the ids cannot be checked and are accepted as given.

## Tilt guard

swerve_drive_controller owns the tilt guard (roll/pitch from the IMU relative to the level reference, persistence and
hysteresis, the base drive inhibit at error) and latches its state on **`/tilt_guard/state`** (`std_msgs/String` JSON,
reliable + transient_local depth 1, on every level change and at 1 Hz):
`{"level": "ok"|"warning"|"error", "roll_deg", "pitch_deg", "limits": {"roll_warn_deg", "roll_error_deg",
"pitch_warn_deg", "pitch_error_deg"}, "since", "base_drive_inhibited", "imu_ok"}`. Roll (tipping sideways, short lever)
has the smaller limits (default 8 / 15 deg), pitch (tipping forward/back along the 430 mm axis) the bigger ones
(12 / 20 deg). mcp_server consumes it (`RobotMonitor.on_tilt_guard`, the topic is `topics.tilt_guard_state`); a message
that does not match the contract is logged and dropped.

| Level change | Robot event | Nav2 goals | Tools |
|---|---|---|---|
| -> `warning` | `tilt_warning`, critical (claude_agent interrupts the running instruction at once): roll/pitch vs limits and "automatic navigation is disabled; drive back to level ground manually with short base moves (the drive tool ...), then wait for the guard to clear". From `error` the message says the base drive is enabled again, or, when `base_drive_inhibited` stays true (the inhibit can outlast the error), that the base drive is still inhibited and no base motion tools work yet | every goal cancelled (`cancel_all_goals` from a worker thread, retried `monitor.tilt_cancel_attempts` (3) times `monitor.tilt_cancel_retry_s` (1 s) apart; never answered: a critical `tilt_nav_cancel_failed` event telling the agent to call `stop`), a running `navigate_to_pose` / `move_relative` / `drive` / `look_around` ends `interrupted` (`BASE_INTERRUPTS`) | `navigate_to_pose`, `move_relative`, `look_around` and `enqueue_motions` with a base step refused; queued base steps fail at dispatch (status `tilt_guard`); `drive` allowed unless `base_drive_inhibited` is still true (then refused like at `error`) |
| -> `error` | `tilt_error`, critical: "base drive is inhibited until the IMU is back within limits; no base motion tools work" | every goal cancelled | every base motion tool refused (`base inhibited by the tilt guard`) |
| `warning`, `base_drive_inhibited` true -> false | `tilt_drive_enabled`, info: "base drive enabled again" plus the manual drive-back instruction | - | `drive` allowed again (navigation still refused) |
| -> `ok` | `tilt_clear`, info | - | everything allowed again |

The classification lives in `mcp_server/tool_context.py` (`TILT_NAV_TOOLS`, `TILT_MANUAL_BASE_TOOLS`,
`ToolContext.tilt_gate`). Manual drive follows `base_drive_inhibited` as well as the level: the controller drops base
commands while inhibited, so `drive` is refused then instead of being silently ignored. The queue re-checks before every base step (`MotionQueue(tilt_gate=...)`). Arm tools are never
affected. Refusals are MCP tool errors, e.g. `drive refused: base inhibited by the tilt guard (level error, roll 16.0 deg
(limits 8/15), pitch 1.0 deg (limits 12/20)): no base motion until the IMU is back within limits`. A state older than
`monitor.tilt_guard_stale_s` (3 s) is stale: its last level stays enforced (the refusal and `get_body_state` say so) and
the IMU fallback `tilt` warning runs again. A dead guard is never trusted as level: at level `ok`, a stale state or
`imu_ok: false` refuses navigation (`navigate_to_pose`, `move_relative`, `look_around`, queued base steps: "automatic
navigation is refused until the tilt guard reports a fresh state with its IMU"), while short manual `drive` moves stay
allowed; `navigation_allowed` is false then too. `get_body_state` reports `tilt_guard` (`level`, `roll_deg`, `pitch_deg`,
`limits`, `since`, `base_drive_inhibited`, `imu_ok`, `age_s`, `stale`, `navigation_allowed`, `manual_drive_allowed`;
null with a note before the first message; both allowed flags are false while `base_drive_inhibited`) and the vitals
line ends with e.g. `tilt guard warning: roll 9.0 deg (limits 8/15), pitch 2.5 deg (limits 12/20)` once a state has
arrived (`tilt guard warning, base inhibited: ...` while the inhibit outlasts the error).

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
| `ALWAYS_ALLOWED_TOOLS` | `stop`, `cancel_motions`, `get_motion_status`, `wait_for_event`, `plan_grasp`, `get_robot_state`, `get_body_state`, `get_camera_image`, `get_map_summary`, `get_arm_state`, `take_control`, `release_control`, `get_control_state`, `get_topdown_view`, `remember_object`, `list_objects`, `forget_object`, `list_pois`, `add_poi`, `update_poi`, `delete_poi`, `list_lookups`, `get_lookup_image`, `annotate_lookup` |

`arm_set_home` is classed as motion because it rewrites the pose a later `arm_home` drives to.

## Control leases

Every arm and base tool needs a control lease (`control.py`, `ControlManager`). The table `TOOL_SUBSYSTEM` in
`tools.py` (next to `MOTION_TOOLS`) is checked in one place, `RobotMCPServer.call_tool_with_digest`, before the tool
body: so before the battery and tilt gates, and a tool without its lease is refused before anything is published.

| Lease | Tools |
|---|---|
| arm | `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `grasp_object`, `release_object` |
| base | `navigate_to_pose`, `move_relative`, `drive`, `look_around` (base-only spin) |
| per step | `enqueue_motions`: `arm_joints` / `arm_cartesian` / `gripper` / `grasp` steps need arm, `base_relative` / `navigate_to_pose` (`motion_queue.BASE_KINDS`) need base; checked first inside the tool, naming every missing lease |
| none | `stop`, `cancel_motions`, `wait_for_event`, `get_motion_status`, `take_control` / `release_control` / `get_control_state`, `plan_grasp`, `arm_set_home`, camera, perception and POI tools |
| any (placeholder) | a future head tool: `LeaseNeed` `"any"` is satisfied by either lease, so a base-to-arm hand-over needs no head reset. No tool uses it yet; `ControlManager.refusal(name, "any")` implements it |

- **Arm lease** = the filter_node autonomy lease (the hardware arbitration). `take_control('arm')` is
  `ArmController.acquire()` (or adopts an arm mcp_server already holds implicitly) and records the lease generation
  (`ArmController.lease_generation`, bumped whenever a lease starts or ends). The agent holds the arm only while that
  generation is current (`ControlManager.arm_held`): an implicit hold by the web UI grasp service or by a `stop` during
  a foreign motion is not the agent's lease (`get_control_state` `arm.held_by`: `agent`, `mcp_server` for such an
  implicit hold, or filter_node's active source such as `leader` / `web_ui`). A human can end it: the web UI Home button
  calls `/arm/home` (always releases), and a filter_node switch to another source drops it. The next arm tool is then
  refused saying how it was lost (`ArmController.lost_note`: `taken over by web_ui`, `released by the /arm/home service
  (web UI Home button)`, `filter_node reports no active source (restarted?)`, `mcp_server restarted (the orphaned
  autonomy lease was released)`); the agent never silently re-takes it, also not inside a running tool: every arm tool
  body and queued step runs bound to the generation it was checked against (`ControlManager.lease_scope` sets
  `arm.EXPECTED_LEASE`), so a grasp still planning when the human pressed Home fails its next motion (`aborted`, reason
  `arm control not held (...) - call take_control(subsystem='arm')`) instead of re-acquiring the arm. GraspService (web
  UI grasp) and the `/arm/home` service pass no generation and still take the arm implicitly.
- Leases are global to the server, not per MCP session (two clients of one server share them).
- **Base lease** = a flag inside mcp_server (nothing downstream arbitrates `/cmd_vel_nav`; humans can still drive).
- Both may be held at once. `stop` does not release leases.
- **Refusal text**: `move_arm_joints refused: arm control not held - call take_control(subsystem='arm')`; with a lost
  lease `arm control not held (taken over by web_ui) - ...`. At tilt warning without the base lease the lease refusal
  comes first; with the lease held the tilt refusal follows as before.
- **Queue dispatch**: the motion queue re-checks the lease before every step (`lease_gate`, step status `lease`), so a
  lease lost mid-queue fails the next step instead of re-taking it.

**release_control(subsystem, keep_position=false)** always releases; the reset outcome is reported per subsystem:

| Subsystem | Default reset | `reset` values |
|---|---|---|
| arm | `ArmController.home(keep_prior_control=True)` (the self-collision-safe home routes), then `release()` | `homed`, `kept` (keep_position), `failed: no home pose stored`, `failed: <status>: <message>`, `failed: <error>`, `skipped: battery_cutoff` (battery below cut-off: release only), `skipped: not held`, `stopped: ...` (a `stop` during the park: the arm holds where it stopped and the lease is KEPT, not handed to the leader arm / web UI), `refused: carrying (...)` (`all` while carrying, see below) |
| base | `RosRobot.reset_base()` (the base half of `stop`: abort base calls, zero twist, cancel the Nav2 goals THIS server sent (`control.NavGoalRegistry`, filled by `run_nav`; a human's web UI goal is left alone; `stop` still cancels every goal), zero twist), then wait until every `control.steer_joint_names` joint on `/swerve_drive/joint_states` is within `control.steer_neutral_tolerance_rad` (0.05) of 0, at most `control.base_reset_timeout_s` (6 s = swerve `idle_recenter_s` 3 s + 3 s; the controller straightens the wheels itself after 3 s of zero twist) | `neutral`, `kept`, `failed: steering not neutral within 6 s (...)`, `failed: no fresh steer joint states ...`, `failed: Nav2 cancel service did not answer (zero twist sent)`, `skipped: not held` |

`all` resets the base first (wheels stopped before the arm moves), then the arm. A lease not held is not released
(`skipped: not held`). The base lease is cleared before its reset starts: base tools are refused during the up to 6 s
steering wait, and a `take_control('base')` meanwhile is a new lease that stays held. The call is refused while the
motion queue runs (`cancel_motions or wait_for_event first`), while another blocking motion tool runs, and (arm
targeted) while an arm motion runs (`release_control refused: an arm motion is running - call stop first or wait for it
to finish`), so a release never aborts a motion mid-trajectory.

**Carrying an object: one signal.** `get_control_state` / `take_control` / `release_control` report
`arm.carrying = {carrying, reasons, radius_m}` (`ControlManager.carrying_state`), the only carrying signal of
mcp_server. Reasons: `carried_object_radius_m` of the last keep_position arm release (`radius_m`; 0 means none); the
last grasp is held (`ArmController.grasp_held`: the last close ended `grasped`, or grasp_object's `unverified`
empty-band hold kept gripping, and no open converged since); a holding load (the motion queue's `gripper_holding`
precondition: jaw short of closed, not wide open, load >= `motion_queue.holding_min_effort`). The first two live in
the carry record (`carry.py`, `<home_file dir>/carry.yaml`, written atomically on every change), so they survive
`take_control` and an mcp_server restart; a converged open clears it, and so does a measured jaw more open than
`limits.gripper_grasp_max_open_rad` (a human emptied the gripper with the leader arm). Parking the arm
(`keep_position=false`) is refused while carrying: `release_control refused: the arm cannot park while the gripper
carries an object (<reasons>) - put the object down first (release_object), or release with keep_position=true`;
`subsystem='all'` still releases (and resets) the base and reports `reset.arm = "refused: carrying (<reasons>)"` with
the arm lease kept. A release with `keep_position=true` stays allowed (that is how the robot drives while carrying
something) and, while carrying, keeps the grip profile's torque limit on the gripper (`ArmController.release(
restore_torque=False)`); the default torque comes back with the next open or a parked release. `carried_object_radius_m`
is refused without `keep_position=true`, with `subsystem='base'`, outside 0..0.3 m, or without the arm lease held.

**get_control_state** (sensor): `arm.at_home` is true when the five non-gripper joints are within
`limits.settle_tolerance_for(joint)` (0.08 rad default) of the stored home pose; the gripper is excluded so a job may
end at home holding an object; `null` without a stored home or fresh joint states. `base.steering_neutral` is `null`
without fresh swerve joint states. `/swerve_drive/joint_states` (50 Hz) is subscribed lazily, not for the process
lifetime: the first steer read (`get_control_state`, the `release_control` base reset) subscribes it and waits up to
`control.steer_states_wait_s` (0.5 s) for a first message (none: `steering_neutral` is `null`, the base reset keeps
polling until `failed: no fresh steer joint states ...`); the subscription is destroyed (and its sample dropped) once
unread for `control.steer_states_idle_s` (30 s; checked every quarter of it). `held_since` is wall-clock epoch seconds (the agent's lease). `arm.held_by` and
`arm.carrying` are described above. A converged `/arm/home` service call (web UI Home) runs the footprint tick, which
publishes the static footprint again. The stored home pose is cached by file stat (mtime, size, inode) and re-read
only after it changed. `footprint` reports the published
navigation footprint: `kind` (`static` / `dynamic` / `conservative`), `vertices` (costmap footprint vertex count),
`carried_object_radius_m` (`null` without one) and `consumers` (`local_costmap`, `global_costmap`, `laser_filter`,
`collision_monitor`: `ok`, `pending`, or `unavailable: ...` / `refused: ...` / `timeout: ...` / `republished: ...`);
`footprint` is `null` without a `nav_footprint` config block.

## Navigation footprint

`footprint.py` (`NavFootprint`, no ROS) decides what Nav2, the collision monitor and the lidar self filter treat as
the robot; `ros_iface.RosRobot` is its `FootprintSink` (publishers, parameter clients, the republish timer, the
`published_footprint` subscriptions, the consumer readiness poll and the measured arm pose).

| Kind | When | Footprint |
|---|---|---|
| static | autonomy holds the arm (`take_control(subsystem='arm')`: the arm may move from then on), a parked release, a converged `/arm/home` (its success runs the footprint tick at once, which sees the parked arm), or the measured released arm lies inside the static outline | the H body outline (`footprint_geometry.body_outline` over `body:`) united with the ground shadow of the arm parked at its stored home (`nav_footprint.parked_arm_shadow`), identical to the generated `nav2_params.yaml` footprint |
| dynamic | the arm is released out of the static outline (`release_control(subsystem='arm', keep_position=true)`, a failed park, a human moving the released arm, an mcp_server start with the arm out) | union of the static outline, the ground shadow of the measured arm pose and, with `carried_object_radius_m`, a circle of that radius around the tool point |
| conservative | the arm is released but its pose is unknown: mcp_server start without fresh joint states, or no fresh joint states since an unverified static footprint (a non-homed release, e.g. `keep_position`, without them publishes it at once with the carried circle) | the static outline united with a disc of `conservative_reach_m` (0.5 m; the farthest arm part is ~0.445 m from the mount) plus the carried radius around the arm mount |

The arm shadow is the convex hull of every arm part point of `SelfCollisionChecker.part_points` (link capsule ends
widened by the 2 cm capsule radius, the grasp_sim collision box faces, the gripper hull, the jaw points) projected to
the floor in `base_link`. Shadow and carried circle are grown by `nav_footprint.simplify_m` (5 mm) and then simplified
by it (so the simplification never trims the shadow), and unioned with the exact static outline (holes filled, one
polygon). An arm shadow within `simplify_m` of the static outline publishes the static footprint (ArmStop off).

**Republish tick** (every `republish_s`, 5 s; the timer runs at the period `NavFootprint` asks for). While autonomy
does not hold the arm it recomputes the footprint from the fresh measured pose (2.6-2.8 ms) and publishes it when it
moved by more than `refresh_threshold_m` (1 cm, so joint noise does not make the costmaps re-inflate); with stale
joint states it keeps the last footprint, unless that one never saw the released pose (static from `take_control`):
then it goes conservative. It also republishes the footprint to a costmap whose `<ns>/published_footprint` (what the
costmap uses, in its global frame; compared up to a rigid motion) differs, e.g. after a Nav2 restart, or never
arrived; an unchanged footprint is not re-sent (each footprint change re-inflates the whole costmap). The laser_filter
`polygon` and `ArmStop.enabled` are re-sent every tick. Parameter sets still unanswered after `param_timeout_s` (2 s)
are cancelled and reported as `timeout`. A consumer whose `set_parameters` service comes up (polled every
`consumer_poll_s`, 0.5 s; a restarted node starts from its yaml values) gets the current values at once. `/arm/home`
runs the tick right after a converged home. One re-entrant lock covers a state change and its sends, so a tick never
finishes sending a footprint a tool call has already replaced.

Consumers:

| Consumer | Input | Value |
|---|---|---|
| Nav2 local / global costmap | `geometry_msgs/Polygon` on `/local_costmap/footprint`, `/global_costmap/footprint` (`Costmap2DROS` subscribes its relative `footprint` topic) | the footprint (`footprint_padding` is 0) |
| Nav2 local / global costmap inflation | parameters `inflation_layer.inflation_radius` and `inflation_layer.cost_scaling_factor` of `/local_costmap/local_costmap`, `/global_costmap/global_costmap` (dynamic InflationLayer parameters) | the footprint's circumscribed radius rounded up to a cell, and the steepest factor up to `max_cost_scaling_factor` (25) with cost >= 1 there (`k <= ln(252) / (circumscribed - inscribed)`), so MPPI / Smac keep their circumscribed-radius shortcut; sent only when changed (or the costmap restarted) |
| laser_filter (`nodes/laser_filter`) | parameter `polygon` of `/laser_filter` through its `set_parameters` service | footprint + `laser_filter_margin_m` (1 cm), so the lidar does not see the robot's own arm or carried object |
| collision_monitor | `PolygonStamped` (transient local) on `/collision_monitor/arm_stop_polygon` + parameter `ArmStop.enabled` | dynamic / conservative: the part of the arm extension outside the static outline (the whole extension when that part falls apart, e.g. the arm also spans a notch) grown by `stopbox_margin_m` (2 cm), enabled; static: `ArmStop.enabled` false |

The StopBox velocity sub-polygons are static parameters (Jazzy collision_monitor reads them once at configure, and
`velocity_polygon` has no footprint topic), so they keep covering the static footprint and the separate `ArmStop`
polygon covers the released arm. ArmStop uses `action_type: approach` (simulates the commanded motion for
`time_before_collision`, 1 s): it slows down toward an obstacle next to the arm but lets the robot back away from it.

The geometry numbers (`circle_segments`, `close_gap_m`, `laser_filter_margin_m`, `stopbox_margin_m`, `simplify_m`,
`decimals`, `costmap_resolution`, `max_cost_scaling_factor`, `parked_arm_shadow`) are generated into the
`nav_footprint:` block of `client.yml` by `scripts/gen_nav_footprint.py` from `scripts/gen_nav_footprint.yaml`
(`parked_arm_joints` is the stored home pose; the script computes its shadow with `python -m mcp_server.footprint` in
this uv environment). The outline / circle / offset / inflation code (`footprint_geometry.py`) is the one the
generator imports: the static footprint and inflation mcp_server publishes equal the static Nav2 / laser_filter
configs (a test checks it). Without the block nothing is published.

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
  orientation (roll/pitch only, from the body monitor) minus `floor_guard.imu_level_reference {roll_deg, pitch_deg}`,
  ignored when older than `floor_guard.imu_max_age_s` (1 s; then plane (a) only, logged once per outage), or the
  per-call `tilt_override_deg {roll, pitch}` (used as given, not shifted by the reference). With pitch p (nose down
  > 0) and roll r (left side up > 0) the level plane is `z = x tan(p) / cos(r) - y tan(r)`: a robot tilted nose down
  raises the zone in front of it.

IMU level reference: the BNO055 is not mounted exactly level; on the floor the robot height (`arm.arm_base_height_m`
0.100) was fitted on it reads a constant roll 0.5-0.8 / pitch 1.2-1.4 deg. That floor IS the robot plane, so the raw
tilt counted the mounting bias twice: at base_link x 0.29-0.32 it lifted the surface 5.3-8.2 mm, and top_down tips
ended 16.3 mm above the floor on a 20 mm leg (3.7 mm jaw overlap). The slow zone, the grasp surface and the reported
`tilt_deg` therefore use the IMU tilt minus `imu_level_reference` (default roll 0.7 / pitch 1.3 deg, equal to the
swerve tilt guard's code default: a parity test keeps them equal); an IMU-sourced `slow_zone` / plan
`surface` also reports the raw `imu_tilt_deg` and `imu_level_reference_deg`. client.yml sets roll 0.7 / pitch 1.3 deg
(the 2026-10-11 investigation values, not a fresh capture). Re-capture it (mean IMU roll/pitch while parked level)
whenever the floor height is recalibrated; a real slope beyond the reference still lifts the surface.

A trajectory step whose either end has a checked point closer than `floor_guard.margin_m` (0.02 m) to that surface
(or below it) is time-scaled to `floor_guard.slow_speed_scale` (0.2) of its normal speed: it is split into `1/scale`
linear sub-steps at the same 25 Hz rate. The guard NEVER blocks a motion; the tracking-error and gripper effort/stall
detection stay the contact safety net. Results carry `slow_zone` (`slowed_samples`, `samples`, `speed_scale`,
`margin_m`, `min_clearance_m`, `lowest_point`, `surface_z_m`, `tilt_source` imu|override|none, `tilt_deg` (relative to
the level reference; with an IMU source also `imu_tilt_deg` and `imu_level_reference_deg`); with
surfaces also `lowest_feature` and `surfaces`), null when the whole motion ran at normal speed.

Gripper-only motions (only the gripper joint changes over the planned samples: `set_gripper`, a grasp's close or
open) leave every checked point but the moving jaw tip where it is, so a stationary point near the surface cannot slow
them. Only the moving jaw tip counts: a sample is slowed when its clearance is below `GRIPPER_ONLY_SLOW_MARGIN_M`
(`floor_guard.py`, 0.005 m) AND lower than at the previous sample (the jaw descends toward the surface). A jaw swinging
closed that stays above 5 mm, or does not descend, runs at full speed; `margin_m` does not apply to these motions.

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

## Body collision

The arm can hit its own robot. Seen from above the robot is an H, and those are the only parts the arm can reach
(`body_model.py`, `self_collision.py`, pure geometry, base_link: x forward, y left, z up from the floor):

| Obstacle | Model | Default (m) |
|---|---|---|
| `wheel_fl`, `wheel_fr`, `wheel_rl`, `wheel_rr` | vertical cylinder around the steer axis (the wheels steer, so the whole steering circle counts: the half-diagonal of the 120 x 35 mm wheel) | axes (+-0.1525, +-0.1333), radius 0.0625, z 0 .. 0.12 |
| `frame_left`, `frame_right` | box along x over both wheels of a side (the H strokes joining front and rear wheel, carrying the steering servos) | x -0.2125 .. 0.2125, y centred +-0.1333, width 0.035, z 0.12 .. 0.18 |
| `crossbar` | box between the strokes up to their inner faces (the H bar, carries the arm base and the lidar) | x -0.15 .. 0.10, y -0.1158 .. 0.1158, z 0.06 .. 0.10 |
| `lidar` | vertical cylinder from the crossbar top to the RPLidar A1 dome top | axis (0.150, 0.040), radius 0.037, z 0.10 .. 0.21 |

Measured (2026-10-10, by the user): the lidar dome diameter 74 mm (radius 37 mm), centred 0.150 m ahead of and 0.040 m left of base_link, and its top face 210 mm above the floor; the wheels
35 mm wide (with the 0.06 m wheel radius their steering circle is 0.0625); the side bars joining front and rear wheels
35 mm wide, their top face about 180 mm above the floor. Still ASSUMED: the steer axes (`swerve_controller`
half_length_m / half_width_m), the wheel radius and top (0.12), the side bar bottom (0.12) and length, the crossbar
extent and height, the lidar column bottom (0.10); the dome centre is measured and is independent of the `laser_frame` TF (0.158, 0.06), which is the calibrated scan origin. The numbers live in
`ansible/group_vars/client.yml` (`body:` in the mcp_server config, pydantic `BodySettings`: positive sizes, cylinder
z_max above z_min, unique names); `scripts/gen_nav_footprint.py` derives the Nav2 footprint from the same block.

The dome first entered the model with radius 74 mm (that was its diameter) at the `laser_frame` position, which put it
22 mm into touch-down t4 that the arm reached on the robot. With the measured centre and radius 37 mm t2 and the other
touch-downs are clear; t4 stands 0.3 mm from the dome with the 2.5 mm box points (the 1 cm points had shown 8.3 mm: their
spacing hid 8 mm), under the 2 mm hard margin, so the check refuses it. This is accepted: the real robot reached t4
without contact, so the model (dome position measured to about a millimetre, the simplified sim upper arm box, about
6 mm FK tip error) is a few mm conservative there, and refusing is the safe side.
`test_touch_t4_is_refused_by_the_lidar` pins the refusal; every other touch keeps the strict clearance check. The
folded home is clear of the hard margin (24 mm).

How to measure and correct each number (tape, robot on a flat floor, base_link = centre between the four steer axes on
the floor; x forward, y left):

- Wheels: `x`, `y` = the steer axis (the vertical axis the wheel turns about) from the centre; `radius` = the largest
  distance from that axis to anything that turns with the wheel (tyre edge, steering bracket) seen from above, plus a few
  mm; `z_max` = the top of the tyre / bracket.
- Side frames: `center` x = middle of the frame bar, y = middle of its width; `size` = full length along x, width along
  y and height (bottom of the frame to the top of the servo housings); `center` z = (bottom + top) / 2.
- Crossbar: the deck between the side frames: front and back edge, its width (inner edge to inner edge of the side
  frames), its bottom and top heights.
- Lidar: `x`, `y` = the centre of the dome; `radius` = half the widest diameter (dome or base, whichever is wider; the measured 74 mm is the diameter);
  `z_min` = where it stands (crossbar top), `z_max` = the top of the dome.

Checked arm parts (one FK pass per joint sample, the arm mount `arm.base_in_base_link` converts to base_link): the upper
arm (shoulder_lift axis to elbow), forearm (elbow to wrist_flex axis) and wrist link (wrist_flex axis to gripper_link)
as capsules of radius 0.02 (half the STS3215 width, shared with the grasp planner in `arm_geometry.py`) and as points on
the faces of the arm collision boxes of the grasp_sim MuJoCo model (`arm_geometry.LINK_COLLISION_BOXES`, the six boxes
of `sim/grasp_sim/assets/so101/so101.xml` on 2.5 mm grids (`BOX_POINT_SPACING_M`, spacing / sqrt(2) = 1.8 mm stays within the 2 mm hard margin; a test pins that); the forearm box reaches 4.8 cm off the
capsule axis, where the sim arm hit the lidar dome while the capsules alone saw 18 mm), the gripper body / fixed jaw
hull points and the jaw tip, tool point and moving jaw tip. The two upper arm boxes are contact-only: the hard check
counts their distance plus `hard_margin_m` (it rejects exactly their contact) and the slow band leaves them out. Held
to the margins they would reject the folded home or slow nearly every forward motion
(the slab passes close over the dome top edge); in the plan matrix they touched the dome only in the low stair
reaches, which the contact check rejects. `tests/test_arm_link_boxes.py` pins the boxes to the MJCF and bounds the
checker's clearance by the sim boxes' over random poses. The arm base and the shoulder link stand on the crossbar and
only turn in place about the pan axis: they are not checked at all, in particular not against the crossbar. Distances
are exact signed distances (negative = penetration): points to boxes and finite vertical cylinders in closed form,
capsules by a golden-section minimum along the axis (the distance to a convex obstacle is convex along it).

Semantics (`body_check:`, `BodyCheckSettings`; hard_margin_m must be below soft_margin_m):

- Every streamed arm trajectory is checked sample by sample (the same samples the floor slow zone checks, start pose
  first): `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home` and the `/arm/home` service, the motion
  queue's blended arm groups, every grasp macro step (`grasp_object`, `release_object`, the web UI `/grasp/command`).
  `look_around` and the base tools do not move arm joints.
- A sample with a clearance below `soft_margin_m` (default 0.03) runs at `slow_speed_scale` (0.3); the floor slow
  zone's scale applies too and the slower one wins per sample. The result reports `self_collision` {obstacle, link,
  min_clearance_m, speed_scale, slowed_samples, ...} next to `slow_zone` (slow-band clearances: without the
  contact-only upper arm boxes).
- Gripper-only motions: only the parts that move (`self_collision.GRIPPER_MOVING_PARTS`, the moving jaw tip) count
  for the soft band; a stationary link's clearance cannot change during the motion, so it never slows it, and
  `self_collision` then reports the moving jaw. The hard reject below still covers every part.
- A sample below `hard_margin_m` (0.002) or inside an obstacle rejects the whole motion before anything is published:
  the tool fails (`ok` false) with "refused (self-collision): the <link> would come <clearance> m from the robot body
  obstacle <obstacle> ... Hint: ...". A start that already is inside the hard margin may move out (no clearance may
  drop more than 2 mm below its start value).
- Motion queue: arm groups are checked at enqueue (from the current commanded pose plus the pending targets, as the
  blend will run) and again when they start (the pose can differ): a refused group ends with status `self_collision`,
  ok false and `data.self_collision` {obstacle, link, clearance_m, hard_margin_m, sample, hint}.
- Grasp planner: a candidate whose waypoint, straight-line segment or free joint move (from the start pose to the
  pre-grasp) comes within the hard margin is infeasible with the reason `body_collision:<obstacle>: <label>: ...`.
- Home: when the straight joint path home would hit the body, `home` goes leg by leg via the first safe intermediate
  of `ArmController.home_routes` (raise shoulder_lift and elbow_flex to their home values first, fold the elbow first,
  raise the shoulder first, straighten the wrist first, lift the tool 5 / 10 / 15 cm straight up first, then the
  rest); the result message names it. Only when no route clears the body does it fail, naming every route tried.
- `body_check.enabled: false` switches all of it off (no slowdown, no rejection, no planner reasons).
- Metrics: `mcp_self_collision_rejections_total{obstacle}`, `mcp_self_collision_slowdowns_total{obstacle}`,
  `mcp_self_collision_min_clearance_m` (last executed motion, slow-band clearance).

Check cost (bounding-sphere pruning, `SelfCollisionChecker.bounded_clearances`): the 2.5 mm box points are about 9
million point distances for an 83-sample path, so every check that needs only verdicts and the nearest clearance
prunes: the guarded trajectory and joint_line checks (`path`: `ArmController.guarded_trajectory`, `check_body`, home
routes, queue checks), the grasp plan body check (`GraspPlanner.body_problem`), `point_reach` of `get_arm_reach`
(`closest`, one batch for all pitches) and the reach envelope (`clear_of`). The box and hull points sit in blocks of
32 on their link (`LINK_BLOCKS`) with a bounding sphere; per sample only the sphere centres are placed in base_link,
and a block (or capsule) is placed point by point and evaluated only when its sphere comes within the sample's
nearest clearance (bounded from above by the nearest sphere) or the hard limit of an obstacle. Every reported value
is unchanged, bit for bit: per-sample scales, clearances, obstacles and links (also beyond the soft margin, which the
min-clearance gauge and the plan annotations report), `slow_zone` / `self_collision` summaries and the rejection
(tests compare the pruned reports with `path(..., prune=False)` over seeded random and scenario trajectories; points
are placed element by element so a point gets the same bits in any batch). The three capsules of an obstacle share one
golden-section search. Dev Mac (MacBook M4), deployed default config, before -> after (best of 7; +-30% with load):

| Case | Before | After |
|---|---|---|
| guarded move, clear pose -> home (78 samples, slowed near the lidar) | 136 ms | 21 ms |
| joint_line clear pose -> home (83 samples) | 144 ms | 21 ms |
| grasp plan top_down feasible (4 cm cube, x 0.25): body check / whole plan | 342 / 503 ms | 58 / 193 ms |
| grasp plan angled 45 infeasible: body check / whole plan | 38 / 102 ms | 9 / 49 ms |
| `point_reach` (all 9 pitches) | 71 ms | 9 ms |
| reach envelope (full grid, deployed headings) | 17.4 s | 5.4 s |

The rest of a cold plan is ikpy (cached per instance, see Planner speed). `JawModel.inner_gap` now turns the whole jaw
silhouette in one vectorised Rodrigues rotation (it ran one rotation per profile point, about 15% of a plan). On the
robot (busy RPi5, then capped at a quarter of one core; the unit has no CPU cap now) plan_grasp took 52.6 s and grasp_object 205-259 s before this; expect the body check
share to drop by the same factor.

`get_arm_reach(points?)` (in `body_tools.py`, never refused): the obstacle list with dimensions, the arm mount and pan
axis, the margins and a collision-free reach envelope; with up to `reach_max_points` (20) base_link points
{x, y, z, pitch?} each point reports `reachable` (IK within the joint limits), `collision_free`, the working
`pitch_range_deg` (pitches -30 .. 90 deg every 15 deg, or the given one), the `limiting` obstacle / link / clearance and
a `suggestion`. The envelope (`arm_reach.py`) is computed once in a background thread in batches of `ENVELOPE_BATCH_SAMPLES` (32) poses, about 30 MB of arrays and 70 MB RSS (a whole heading at once peaked at about 2.4 GB and OOM-looped the 256M unit on 2026-10-10) (about 5.4 s on the dev Mac with the 2.5 mm box points, 17 s before the link-frame blocks of Check cost, 3.6 s with the 1 cm points; on the busy RPi5 it took about 95 s with the 1 cm points
while the unit still had a CPU limit (measured before 2026-10-11, when robot units moved to CPU weights with no limit), measure it again after deploying;
`SelfCollisionChecker.clear_of` places and evaluates only the blocks of 32 box points whose bounding sphere comes within the hard margin; a call waits up to `reach_wait_s` and otherwise reports `computing`).
With `reach_cache_file` set (the robot: `/var/lib/ros2/arm/reach_envelope.json`) it starts when the server is built
and is saved to that file under a hash of every input (body, arm link model, mount, joint limits and offsets, grid,
hard margin), so
a restart loads it instead of recomputing and any changed input recomputes; without it the envelope is computed on
the first call. Per
shoulder_pan heading every `reach_heading_step_deg` (15) across the pan limits and tool height 0 .. `reach_height_max_m`
(0.35) every `reach_height_step_m` (0.05) in base_link, the min / max horizontal tool distance from the pan axis over a
13 x 13 x 7 grid of shoulder_lift x elbow_flex x wrist_flex (wrist_roll 0, gripper closed) whose body clearance is at
least the hard margin (2 mm). It is a sampled approximation (about 1 cm at full reach).

Sample spacing and the margin (accepted limitation): the 2 mm margin is exact between box points (spacing / sqrt(2) <=
2 mm) but not between trajectory samples. A link point moves up to 17.4 mm between two trajectory samples (25 Hz, 1 rad/s
cap) and up to 8.7 mm between two `joint_line` samples (0.02 rad: planner free moves, home routes). That cannot
cross any obstacle (the thinnest, a side frame, is 35 mm) and a dip along a straight chord past a round surface is below
1 mm, but a path skimming a sharp obstacle edge between two samples can come closer than 2 mm undetected.

## Unmeasured efforts

feetech_servos publishes a NaN effort for a joint without an effort reading (a non-effort joint, a missing load).
mcp_server treats a non-finite or missing effort as unmeasured, never as a load of 0 and never as a number in a
comparison (`arm.finite_effort` is the one reader): the ROS joint sample (`RosRobot.joint_sample`) drops non-finite
efforts, `ArmState.efforts` omits them and `gripper_effort` is `null` in the arm state, motion results, grasp results
and the `wait_for_event` digest. Every gripper load check says so instead of guessing: grasp verification reports
`gripper load unmeasured` (not grasped), the hold report gives `holding_load` / `crush_risk` `null` and never
re-tightens, the `gripper_holding` precondition and the live holding-load reason of the carrying signal are not holding,
and a close-until-effort cycle without a load takes no effort contact (the stall check still ends it).

## Grasp macros

### Planner (`grasp.py`, pure)

Input: `ObjectSpec` `{frame: 'arm'|'base_link', x, y (centre), support_z (bottom of the object = the surface it rests
on), width_m (across the jaws), depth_m (along the approach), height_m, yaw (rad, width axis; omitted = across the
approach), gap_below_m (clear height under the object's bottom, e.g. an overhang or a raised object; default 0 = flat on
its support), surfaces (optional regions around the object, see Surface regions)}`, a strategy and `GraspParams` (the `grasp` config section with per-call `params` overrides). The 5-DOF arm
can only approach in the vertical plane of `shoulder_pan`, so every strategy approaches radially from the arm base.

| Strategy | Geometry |
|---|---|
| `scoop` | Only for an object with room under it: `gap_below_m` must be at least `jaw_thickness_m` + `scoop_gap_margin_m` (0.008 + 0.004 m), otherwise the plan is infeasible with "no gap under object for the fixed jaw" (in the sim a scoop against a box resting flat pushes or tips it instead of getting under it). Wrist roll about 0 (fixed jaw underneath, moving jaw closes from above), horizontal radial slide at the object's bottom height until the fixed jaw tip is under the object centre. The fixed jaw top goes `below_object_offset_m` under the object bottom, clamped so the jaw (`jaw_thickness_m`) stays `skim_clearance_m` above the effective surface when the object rests on it (`skim`). Opening = height + `jaw_open_margin_m` (+ how far a skimming jaw sits above the object bottom), held over the object's depth in the jaws (see Jaw clearance). Approach pitch starts at `scoop_pitch_deg` (0, horizontal) and steepens in `scoop_pitch_step_deg` up to `scoop_max_pitch_deg` (40; sim: every gap scoop up to 40 deg lifted, steeper ones swing the moving jaw into the object top and slip): a near-horizontal gripper cannot reach low near the base. |
| `angled` | radial approach pitched down by `approach_pitch_deg` (default `angled_pitch_deg` 45), jaws across the object width (roll from the object yaw relative to the pan direction), jaw opening centred on the object (grasp shift, see Jaw clearance), tool point at mid-height (tall narrow objects: see below). |
| `top_down` | pitch 90 deg (straight down), jaws across the width, opening centred on the object (see Jaw clearance), vertical approach, minimum jaw overlap (below). |
| `auto` | `auto_order` (default top_down, angled 45, scoop): the first feasible plan wins; `attempts` lists each try (a flat object's scoop attempt carries the no-gap reason). |

Tall narrow objects (`height_m / width_m` above `tall_ratio`, 1.5) pivot out of the jaws when gripped at mid-height:
`angled` and `top_down` put the tool point at `tall_grasp_height_fraction` (0.3) of the height from the bottom (never
lower than a jaw thickness plus `skim_clearance_m` above the surface) and the `lift` waypoint carries `lift_speed_scale`
(0.05) instead of `slide_speed_scale`; the executor runs every straight-line step at its waypoint's speed scale. Objects
narrower across the jaws (the scoop: taller) than `min_object_width_m` (0.01) are rejected: "the jaws cannot hold it".

Minimum jaw overlap (vertical approach: `top_down`, or `angled` at 90 deg): the fixed-jaw tip (the tool point) goes at
or below object top - max(`min_jaw_overlap_m` 0.008, `min_jaw_overlap_fraction` 0.4 x height), so a short object is
gripped below mid-height. The floor clamp (surface + `skim_clearance_m` + `jaw_thickness_m`, 11 mm above a flat floor)
wins over it: when it leaves less overlap, the plan stays feasible with `warnings: ["shallow_overlap: expected jaw
overlap X mm ..."]` (also in the summary) as long as the overlap is at least `shallow_overlap_min_m` (0.004), and is
infeasible ("jaw overlap X mm below shallow_overlap_min_m ...") below that. On the robot floor 19 mm and taller get the
full 8 mm, 15-19 mm are shallow, under 15 mm infeasible for a vertical grasp. Angled grasps below 90 deg keep the
mid-height rule.
The strategies and defaults are validated in the MuJoCo sim with the jaws calibrated to `arm.tool_offset_m`
(`sim/README.md`, "Planner scenario matrix").

New strategies plug into `grasp.STRATEGIES` (name -> function returning candidate `GraspGeometry`s) and the
`GraspStrategyName` literal in `config.py`.

Output `GraspPlan`: ordered waypoints `pre_grasp` (lifted by `pre_grasp_clearance_m` above the approach start; the
wrist roll changes here, gripper at most `limits.roll_max_gripper_open_rad`), `open` (opening for the object),
`approach` (straight line to `approach_distance_m` before the object), `grasp` (straight slide), `close`, `lift`
(`lift_height_m` up), `retreat` (`retreat_distance_m` radially back), each with the target x, y, z, pitch, roll,
gripper command, speed scale (`pre_grasp` / `open`: `approach_speed_scale`, default 0.3, capped to
`limits.arm_max_speed_scale`; the straight lines `slide_speed_scale` or `lift_speed_scale`), IK joints and
`tool_point` (FK of the fixed jaw inner face at those joints). `angled` and `top_down` centre the jaw opening on the
object: the plan's `grasp_shift` `{object_width_m, shift_m, fixed_jaw_clearance_m, moving_jaw_clearance_m, jaw_depth_m,
jaw_open_axis}` says so (`center_shift_m` = `shift_m` unrounded), the waypoint x, y, z is then the **jaw centre** (the
object centre) and `tool_point` lies `shift_m` (half the width plus the fixed jaw clearance) beside it against the
opening direction. The arm therefore stands sideways of a plain `move_arm_cartesian` to the same x, y, z (6.5 cm cube
at 0.37 m reach: the tool point 4.0 cm off the object centre; before the jaw clearance 3.3 cm, shoulder_pan -0.732
centred vs -0.636 plain). `scoop` has no shift
(x, y, z is the tool point); the straight segments carry joint samples every `interpolation_step_m`
(IK seeded from the previous sample). Feasibility: reachable (IK verified by FK), joints within limits minus margin,
no joint jump above `max_joint_jump_rad` between samples, no stretched-arm stall pose (shoulder_lift above
`stall_shoulder_lift_rad` 1.85 with elbow_flex at or below `stretched_elbow_max_rad` 0; negative elbow_flex stretches),
opening within `max_object_width_m` and the gripper's reach, roll guard ordering, with surfaces the surface and step
edge clearance (see Surface regions), plus slow-zone annotations per step.
Infeasible plans return human-readable `reasons`; the planner never raises for an infeasible object.

#### Jaw clearance (2026-10-10)

Until 2026-10-10 a centred grasp put the tool point (the fixed jaw inner face, `arm.tool_offset_m`) half the object
width from its centre (`ik.grasp_offset`): the fixed jaw came down exactly on the object's side face (0 mm) and the
moving jaw got the whole `jaw_open_margin_m` (15 mm). On the robot this tipped a 39 mm wooden jar over on the approach
and made firm grasps miss. Now the opening is centred on the object:

- Each jaw clears its side face by `max(jaw_open_margin_m / 2, fixed_jaw_clearance_m)` (default 7.5 mm both): the tool
  point goes `width / 2 + clearance` from the object centre (`ik.grasp_offset(width, axis, clearance)`), the opening
  `opening_m` is `width + 2 * clearance`, then the moving jaw closes the object against the fixed jaw. A
  `fixed_jaw_clearance_m` above half the margin widens the opening on both sides.
- The moving jaw is not a straight parallel jaw: it swings about the gripper pivot and its inner face flares away
  from the fixed jaw toward the palm, so once open the opening narrows into the jaws (`JawModel.inner_gap`, from the
  jaw silhouette `mcp_server/jaw_profile.py` taken from the vendored MuJoCo jaw meshes and checked against them by
  `sim/grasp_sim/tests/test_tcp.py`; at 0.534 rad 54 mm at the tips, 42 mm 2.75 cm in, 35 mm 4 cm in). The tip
  opening alone (`JawModel.gap`, which matched the 39 mm jar contact at 0.336 rad on the robot) left a tall object
  touching the moving jaw: in the sim its mesh hit the jar's top edge 2.75 cm into the jaws. Every strategy now opens
  until `opening_m` holds over the object's depth in the jaws (`jaw_depth_m`: the near top corner along the jaws,
  `cos(pitch) * depth / 2 + sin(pitch) * (top - grasp z)`): the 39 mm x 6 cm jar top_down opens to 0.97 rad instead
  of 0.53, a 4 cm cube top_down 0.68 instead of 0.55, angled 45 0.75; scoops keep their geometry and open wider too.
  An object too deep for the gripper's `arm.gripper_open_rad` is infeasible ("the object reaches ... cm into the
  jaws, where the tilted moving jaw needs ...").
- `moving_jaw_clearance_m` in `grasp_shift` is the narrowest moving jaw clearance over that depth.
- Scoop keeps its semantics: the fixed jaw already goes `below_object_offset_m` under the object, so it had no
  zero-clearance fixed jaw; only its opening now accounts for the depth.

Validated in the sim (`sim/README.md`, "Jaw clearance re-run"): the matrix lifts what it lifted before (104 feasible,
100 lifted, 0 contacts) and the tip-over jar scenario lifts without tipping (descent tilt 18.5 deg before with the jar
3 mm toward the fixed jaw, 0 now).

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

Feasible plans keep their feasibility and strategy: `tests/test_grasp_speed.py` compares 6 of them (top_down, angled,
scoop, auto) with golden joints within 1e-6 rad. The robot body check dropped the cases its plans now hit (assumed body:
13 -> 8; first measured body, 2026-10-10: the lidar dome (then radius 74 mm, really its diameter) rejects all but 4x4x4_r25_floor_auto; with the measured dome (radius 37 mm at (0.150, 0.040)) all 6 cases plan again and the two r30 ledge 0.07 scoop_gap plans moved, regenerated),
and five feasible cases of the current matrix were added (floor -0.10: top_down; ledge -0.03: angled 45, tall-box auto;
ledge 0.05 with a 2 cm gap: two scoops; baseline = their plan). Before that, `tests/data/grasp_golden.json` was
regenerated 2026-10-10 with the grid calibration (the stair case moved from -0.25 to -0.204, baseline = the current
plans) and again for the jaw clearance, which moved every centred plan by design (shoulder_pan 0.03 to 0.05 rad, the
open angle of every case, scoop arm joints unchanged). Each case keeps its `baseline`, which the current plan must
match within 2e-3 rad (0.1 deg, far below the arm's backlash). The speed tests (feasible angled and auto golden scenarios and auto on the
reference 4 cm cube under 1 s, scoop 3 s, unreachable 0.2 s) are skipped with `GRASP_SPEED_SKIP=1`.

Regenerating the golden plans after a planner change that moves them by design (check the sim matrix first):

```bash
cd nodes/mcp_server
uv run python -m tests.regenerate_grasp_golden   # re-plans every case, sets baseline = the new plan
uv run python -m tests.regenerate_grasp_golden --drop-body-collisions   # also drops cases the body check rejects
uv run pytest tests/test_grasp_speed.py -q
```

It keeps each case's scenario, refuses to write anything when a case became infeasible, and leaves unchanged plans
(the scoop arm joints of the jaw clearance change) byte for byte.

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
current roll (at the planned `approach_speed_scale`, per-call via `params`, default 0.3, never above
`limits.arm_max_speed_scale`; before 2026-10-10 these moves ran at the maximum 0.5 with no override), rolls there at
the same speed, opens to the planned opening, approaches and slides (`move_path` at the waypoint speed,
`slide_speed_scale`),
closes with `close_until_effort` using `params.grip_profile` (see Grip profiles; `close_effort_threshold` overrides the
profile's contact threshold when set; never a full squeeze: the effort/stall detection and the
gripper limits apply), verifies the grasp (jaw stopped at least `min_hold_gap_rad` short of closed and a load of at
least `hold_effort_min` at the contact or after it; an unmeasured load, see Unmeasured efforts, never verifies a
grasp: `gripper load unmeasured ...`), then lifts (`lift_speed_scale` for tall narrow objects) and
retreats: `grasped`. A close that ends `closed_no_contact` (the jaw reached the empty band, within
`limits.gripper_empty_stall_rad` of closed) is `unverified`, not a miss: a soft object (2026-10-11: the plush croc's
legs squashed until the jaw stopped 0.05 rad short of closed at load 140, or closed fully at low load) cannot be told
from empty jaws by the load, and opening dropped an object the jaws actually held. The executor keeps the jaw closed
(step `hold`, `ArmController.hold_empty_close`: the grip profile's capped `torque_limit` is written again and the jaw
holds the measured position plus the profile's `squeeze_rad` toward closed, never past it), lifts and retreats as for a
grasp, and the reasons say the jaw reached the empty band and the object may be soft/squashed or absent. The
`grasp_object` tool then attaches a fresh gripper camera image (`robot.camera_image("gripper", 640)`, capped by
`limits.max_image_px`; a camera failure adds a warning instead) so the agent can look: object in the jaws, treat it as
held; otherwise `release_object` (opens and lifts) and retry. Only the `grasp_object` tool attaches the image, so the
reason text there says "look at the attached gripper camera image"; everywhere else (queued grasp steps, the web UI's
GraspService, which returns the same outcome without an image) it says "call get_camera_image 'gripper' and look at
it". An unverified hold counts as carrying (`get_control_state` `arm.carrying`), and its profile torque limit is
relaxed to the default after `grip_profiles.unverified_hold_relax_s` (60 s; the servo heats while the agent looks). A contact that fails verification (`blocked`, jaw within `min_hold_gap_rad` of closed or load below
`hold_effort_min`) opens again, lifts and retreats: `missed`. Any other motion status, a
refusal or a stop (stop tool, or the service `stop`) stops and holds: `aborted` with the reason, and the default
gripper torque limit is restored. Results carry the close's `grip_profile`, `holding_load`, `slipping` and
`crush_risk`.

Hold at the intent: every arm joint stays commanded at its planned target through approach, close, lift and hold
(gripper-only steps re-publish the arm intent, never the sagged measured pose; sag compensation never touches
shoulder_pan, wrist_roll or the gripper). After each step the executor compares every arm joint (also those a gripper
step does not move, e.g. a pan pushed while squeezing) with the step's planned pose: joints off by more than
`limits.converge_tolerance_for(joint)` are the step's `residual_error` (target - measured, rad, merged with the motion's
own settled residual) and a `warnings` line (`close: shoulder_pan -0.060 rad off its target (tolerance 0.03 rad)`). The
result's `residual_error` is the same check at the end against the last executed waypoint (the hold), and `held_pose`
gives `{waypoint, expected_jaw_centre, jaw_centre, tool_point, error_m}` from the measured joints.

2026-10-10 "sideways drift": during the holds of the grip tuning session the measured tool point was 1.5 cm (jar,
3.9 cm wide) to 3.4 cm (cube, 6.5 cm) beside the object, shoulder_pan -0.735 against the -0.636 a plain move to the
same spot needs, and nothing reported it. The cause is the centring above, not a hold error: the mcp_server journal's
`arm_settle` records of those grasps show shoulder_pan within 0.03 rad of its commanded target at every step (for
example cube lift: target -0.7317, commanded -0.7317, measured -0.7348), commanded equal to target for pan (no
compensation), and IK at the cube's lift point gives pan -0.7317 with the half-width shift and -0.6364 without it;
the measured jaw centre was 1.4 mm from the object centre. The tool description called the waypoints "tool point", and
the result had neither the shift nor the tool point, so the agent read the centring as drift. The descriptions, the
plan (`grasp_shift`, `tool_point`), `held_pose` and the residual checks now make both visible. Tests:
`tests/test_grasp_hold.py` (full grasp sequence on a follower that sags with sag compensation on, with and without a
pan push while squeezing).

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
 "grip_profile": "gentle" | "normal" | "firm" | "soft" | {"base": "gentle", "squeeze_rad": 0.01},
 "surfaces": [{"name": "stair", "height_m": -0.10, "frame": "arm",
               "edge": {"point": [0.16, 0.0], "direction": [0.0, -1.0]}}]}
```

`object` is required for `plan` and `execute`; everything else is optional (`strategy` defaults to `auto`, `params`
keys are `grasp` config fields; `grip_profile` defaults to `grip_profiles.default_grip_profile` and may also be given as
`params.grip_profile`, an unknown profile is an error). Response:

```json
{"ok": true, "request_id": "...", "action": "execute",
 "result": {"outcome": "planned" | "infeasible" | "grasped" | "unverified" | "missed" | "aborted" | "released",
            "reasons": ["..."], "plan": {"strategy": "top_down", "feasible": true, "waypoints": [...], ...},
            "steps": [{"label": "pre_grasp", "status": "converged", "message": "...", "slow_zone": null,
                       "residual_error": {}}],
            "gripper_position_rad": 0.21, "gripper_effort": 350.0,
            "grip_profile": {"name": "gentle", "squeeze_rad": 0.01, "torque_limit": 250, "close_speed_rps": 0.2,
                             "target_load": 80.0, "contact_effort_threshold": 120.0, "crush_load": 200.0,
                             "capped": []},
            "holding_load": 130.0, "slipping": false, "crush_risk": false, "retightened": false,
            "residual_error": {}, "warnings": [],
            "held_pose": {"waypoint": "retreat", "expected_jaw_centre": {"x": 0.17, "y": 0.0, "z": -0.03},
                          "jaw_centre": {"x": 0.1702, "y": 0.0001, "z": -0.0303},
                          "tool_point": {"x": 0.1701, "y": 0.0151, "z": -0.0304}, "error_m": 0.0004}}}
```

`stop` answers `{"ok": true, ..., "result": {"arm_held": bool, "message": str}}` and aborts a running execute/release.
Errors answer `{"ok": false, "request_id": ..., "action": ..., "error": "..."}` (invalid JSON or fields, missing
object, unknown params, another action running, battery cut-off for execute/release).

The camera overlay `planned_gripper` of `get_annotated_camera_image` takes one point: pass a plan waypoint (x, y, z)
to draw it.

## Grip profiles

How hard the gripper grips is set per object (`grip.py`, config section `grip_profiles`). A profile is
`{squeeze_rad, torque_limit, close_speed_rps, target_load, contact_effort_threshold, crush_load}` in servo units
(loads and torque limit in 0.1 % of max torque). Presets, tuned on the real gripper in the 2026-10-10 grip tuning
session (config defaults and `client.yml`):

| Preset | torque_limit | squeeze_rad | close_speed_rps | target_load | contact threshold | crush_load | For |
|---|---|---|---|---|---|---|---|
| `gentle` | 250 | 0.01 | 0.2 | 80 | 120 | 200 | fragile or light objects |
| `normal` (default) | 500 | 0.03 | 0.5 | 200 | 300 | 450 | ordinary objects (the squeeze, speed and threshold used before profiles, plus a closing-load stop) |
| `firm` | 650 | 0.05 | 0.4 | 300 | 350 | 600 | heavy or slippery objects, tools |
| `soft` | 600 | 0.08 | 0.3 | 150 | 250 | 550 | plush / soft toys, foam, fabric |

**soft** (added 2026-10-11, values chosen within the caps, not yet validated on a held object): at 12:18 `grasp_object` angled on a plush croc head (5.5 cm) with `normal` reported
`grasped` (jaw stalled at 0.35 rad, load 204 at the close, the head visibly in the jaws), but `holding_load` right after
read 0 and the croc slipped out during the lift: a plush compresses under the hold and the jaw loses its force. soft
therefore squeezes the full `squeeze_max_rad` (0.08) past the contact, so the compressed object keeps pushing back,
and closes a little slower (0.3 rad/s) with a lower stop load (150) so it meets the plush before squashing it.
`torque_limit` 600 is above normal (the jaw must still press a compressed plush) and well below the 800 cap.
Burn-out risk and its mitigation: the SO-101 gripper servo overheats when it squeezes a blocked object at high torque
for long. The soft hold target lies inside the object, so the servo stalls on the compressed object rather than at
closed, pushing with at most the 600 torque limit (never more: the hold uses the profile's torque limit, which stays
below the 800 cap); the squeeze, and with it the position error the servo works against, is capped at 0.08 rad from
the contact and never past closed; `crush_load` 550 flags a hold close to the limit (`crush_risk`), and every open,
release, abort and lost lease restores the default torque limit (500), so the soft limit only lasts while an object
is held. Release soft objects as soon as the task allows.
Session findings behind them: the closing load is positive (+36 holding a plush, +148 on an empty firm jaw; opening
-24 to -36), so `closing_load_sign` 1 is right. Only gentle has a successful hold behind it (plush croc tail); the
earlier gentle (target_load 120) compressed the 4 cm tail to about 7 mm before the load reached the target and the hold
relaxed to 36. **A load-based `crush_load` cannot detect squashing a soft object**: plush gives way without the load
rising, so `crush_risk` stays false while the object is flattened; look at a picture. normal refused the jar at
0.147 rad travel (hence `gripper_grasp_min_travel_rad` 0.10), firm is unvalidated on a held object (both firm attempts
missed). The empty firm close stalled 0.055 rad before closed at load 148, above `hold_effort_min` 100: the
`gripper_empty_stall_rad` check (0.08) in the arm controller reports it as `closed_no_contact`. Since 2026-10-11 (plush
croc squashed into that same band) `grasp_object` reports such a close as `unverified`: it keeps holding, lifts and
attaches a gripper camera image. For plush and other soft objects use `soft` (see above); a plush squashed into the
empty band still reports `unverified`.

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
   when the jaw closed more than `slip_threshold_rad` between them, `crush_risk` when `|holding_load| > crush_load`
   (`holding_load` and `crush_risk` are `null` when the load is unmeasured, and an unmeasured load never re-tightens);
   **re-tighten** (`ArmController.retighten_hold`, profiles with `retighten: true`: normal, firm, soft; not gentle,
   whose 0.01 rad squeeze would grow 8x on a light rigid object that reads about 0 load): when the closing-direction
   holding load is below
   `retighten_load_below` (30) and the measured jaw is still outside the empty band (not within
   `gripper_empty_stall_rad` of closed), the hold steps once further toward closed, to `squeeze_max_rad` past the
   contact position in total (normal: 0.03 + 0.05; an effort contact, which holds the measured position: + 0.08;
   soft already squeezes the cap, so nothing is left), never past closed and with the torque limit unchanged; the
   hold is then measured again and the result reports `retightened: true` (`false` when the check found nothing to
   do, null without a grasp). The step is bounded by the same caps a preset may use and fires only when the servo is
   not pushing at all, so it never adds squeeze to a loaded hold; a re-tightened hold whose new measurement shows
   `crush_risk` goes back to the hold before it (`retighten_reverted: true`; `crush_risk` reports that measurement).
   An inline override can set `retighten` too. `retighten_load_below: null` disables it for every profile;
6. reports `grip_profile` (name, applied values, `capped` fields) and `torque_limit_readback`: `verified` /
   `mismatch ...` when the bridge's `/follower/servo_registers` dump (about every 10 s) arrived after the write,
   else `unverified ...` (logged).

**Hard caps**, enforced server-side whatever the profile says: `torque_limit <= torque_limit_max` (default 800 of 1000;
the register range is checked too), `squeeze_rad <= squeeze_max_rad` (default 0.08), `close_speed_rps <=
limits.gripper_velocity_rps`. Presets above a cap fail config validation; inline overrides are clamped and listed in
`capped`.

**Torque restore**: a grasp keeps its profile's torque limit while holding. The default preset's `torque_limit` is
written back after every open (after the jaw moved, so the jaw never squeezes harder before it opens), after a close
that did not grasp (`closed_no_contact`, `blocked`, any abort, an exception; an `unverified` grasp_object writes the
profile's limit again for its hold and keeps it until the release or `unverified_hold_relax_s`), when a grasp run
aborts, on `release_control` (except a keep_position release while carrying: the grip stays), when the lease is lost
to another source, and on mcp_server startup (as soon as the bridge
subscribes `/follower/set_register`), so a crash or an unexpected end never leaves the gripper weakened or strong.

```yaml
grip_profiles:
  default_grip_profile: normal
  torque_limit_max: 800       # hard cap of every profile (0..1000)
  squeeze_max_rad: 0.08       # hard cap of every profile
  closing_load_sign: 1        # sign of the decoded gripper effort while closing on an object
  hold_check_delay_s: 0.25
  slip_threshold_rad: 0.01
  retighten_load_below: 30    # re-tighten a retighten profile's hold that relaxed below this load (null = never)
  unverified_hold_relax_s: 60 # unverified hold: profile torque this long, then the default (null = never)
  presets:
    gentle: {squeeze_rad: 0.01, torque_limit: 250, close_speed_rps: 0.2, target_load: 80, contact_effort_threshold: 120, crush_load: 200, retighten: false}
    normal: {squeeze_rad: 0.03, torque_limit: 500, close_speed_rps: 0.5, target_load: 200, contact_effort_threshold: 300, crush_load: 450, retighten: true}
    firm: {squeeze_rad: 0.05, torque_limit: 650, close_speed_rps: 0.4, target_load: 300, contact_effort_threshold: 350, crush_load: 600, retighten: true}
    soft: {squeeze_rad: 0.08, torque_limit: 600, close_speed_rps: 0.3, target_load: 150, contact_effort_threshold: 250, crush_load: 550, retighten: true}
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
approach_pitch_deg, surface_z_m, tilt_override_deg, surfaces}` (the `grasp_object` plan + execute through `GraspExecutor`; `grasped` succeeds, `unverified` (still holding, needs a
look: `get_camera_image` 'gripper'; it always stops the queue, also with `on_fail: skip`, since the object may be in the
jaws) / `missed` / `aborted` / `infeasible` fail). The call is all or nothing: an unknown joint, a bad speed, a timeout above
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

**Planning phase and progress.** An arm group first plans: the body / floor / sag checks of the whole trajectory run
before the first setpoint is published and can take tens of seconds. From `step_started` until that first setpoint,
`current.phase` is `planning` (progress 0, `planning_s` = seconds so far, growing); then `executing` (`planning_s`
null; non-arm steps are `executing` from the start). When planning ends the queue emits one `planned` event per job of
the group (`data.planning_s`; it never wakes `wait_for_event`, `until='any'` excepted). Progress of an arm group is the
fraction of the published trajectory time (setpoints published / total, so it advances smoothly for single moves and
blend groups and includes the slow-zone time scaling); it reaches 1.0 when the trajectory ends, before convergence
settling. `wait_s` progress is the elapsed fraction. The tool descriptions tell the agent that a growing `planning_s`
is normal work, not a stall.

**Preconditions and failure policy.** `precondition {type, joints, tol, min_fraction}` is evaluated on live state when
its step is dispatched: `none`, `gripper_holding` (jaw at least `motion_queue.holding_min_gap_rad` short of closed, no
more open than `limits.gripper_grasp_max_open_rad`, gripper load at least `holding_min_effort`; an unmeasured load is
not holding: `gripper load unmeasured`), `gripper_open` (open
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
  source on `/filter/active_source`. MCP arm tools need the lease taken with `take_control` (see Control leases); the
  arm controller itself (GraspService, `/arm/home`, queued steps) still takes it implicitly when it moves. If
  filter_node switches to another source (after a 0.5 s grace) the lease is dropped and the motion stops without
  fighting the new source. While the
  lease is held and idle, the last setpoint is republished at `hold_republish_hz` (5 Hz) as a keepalive.
- **Lease is sticky**: filter_node ignores the leader arm and the web UI while the autonomy lease is held, so the lease
  is only ended by an explicit `release_control` (the MCP client is told so in the tool descriptions), by `/arm/home`
  (always releases), or on shutdown.
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
  Aborts and holds the measured pose when `/follower/joint_states` is older than 0.3 s, the tracking error (gripper
  excluded: the measured joint against the setpoint commanded `arm_tracking_delay_s` (0.35 s) earlier, by publish time;
  a joint between that setpoint and the newest one has error 0) of a joint exceeds `arm_tracking_error_rad` (0.25) +
  `arm_tracking_lag_s` x the motion's joint velocity cap (`arm_tracking_lag_s` is 0: the delay window already carries
  the servo lag, and stacking both let a jammed joint at 1 rad/s run 0.85 rad behind the newest setpoint before the
  abort; now about 0.6 rad), or `stop` is called. The abort message and the `arm_tracking_abort` event name the joint. One arm motion at a time.
- **Tracking limit and the follower delay**: the follower chain reaches a setpoint about 0.2 s after it is published
  (clean servo lag 0.20-0.24 s on every joint, the unloaded wrist_flex as much as the gravity-loaded ones: least-squares
  fit of the follower's `joint:value` journal lines against the actual setpoint publish times, 2026-10-11 with P=16).
  The tracking check therefore compares each joint against the setpoint commanded `arm_tracking_delay_s` (0.35 s, margin
  over the lag) earlier, measured on the backend clock from the publish times (the motion's start pose before any
  setpoint is that old), and counts a joint anywhere between that setpoint and the newest one as following (error 0).
  A jammed joint aborts once the delayed setpoint moved 0.25 + 0.25 x vmax away from it; a follower stalled far beyond
  the window (1.2 s in the tests) aborts too. History: H1 (2026-10-11 12:19:55, `arm_home` aborted at 0.504 rad over
  0.500) raised the window to 0.65 s from a fit against index-timed setpoints (0.53-0.65 s), which included the 25%
  tick stretch of the old fixed-sleep stream (see Motion timing) and the follower register-dump stall; with ticks on a
  deadline and time-stamped setpoints 0.35 s suffices. `arm_tracking_delay_s: 0` compares against the newest setpoint.
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
  `settle_s`, `settle`, `settling`, `tracking_error_rad`, `time_to_first_setpoint_s` and `tick_rate_hz`, so trajectory
  and settle seconds are separable in the journal.
- **Motion timing**: `ArmController.stream` schedules its ticks on a deadline (`next += period`, then sleeps
  `max(0, next - now)`) and every tick publishes the planned trajectory at the actual elapsed time (interpolated between
  the 25 Hz plan samples, clamped to the goal; the slow-zone and body-margin time scaling stretch that plan and are kept).
  A motion therefore finishes at its planned `trajectory_s` whatever the tick jitter; a tick that overruns past the next
  deadline publishes the plan at its real time once (one larger step) and drops the missed ticks instead of bursting to
  catch up. Before 2026-10-11 each tick did its work and then slept a fixed period, so ticks averaged 49.8 ms for the
  planned 40 ms (Mimir `mcp_motion_tick_period_seconds`: 31% 40-45, 34% 45-50, 25% 50-60, 9% 60-80 ms) and every
  trajectory played about 25% slower than planned. Vias of a blend (`on_via`) fire when the stream reaches their
  setpoint (each one in order when an overrun skips past several), `on_tick` reports (planned setpoints reached,
  total). Every arm/gripper motion result carries `time_to_first_setpoint_s` (motion start to the first
  published trajectory setpoint: planning, slow zone and body check), `ticks` (trajectory setpoints published) and
  `tick_rate_hz` (`ticks / trajectory_s`; the plan streams at 25 Hz, a lower value means ticks were dropped because the
  host could not keep up, e.g. a CPU-throttled robot; the motion still ends on time). Queued motion events carry the same two timing fields.
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
  (`gripper_grasp_min_travel_rad` from the start, at most `gripper_grasp_max_open_rad` open), else `blocked`; and a
  jaw within `gripper_empty_stall_rad` of closed is empty (`closed_no_contact`, never `grasped`).
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
| `base_motion.py` | no | timed clamped drive loop, continuous spin with marks (`run_spin`), stop sequence (`run_stop`) and its base half (`run_base_stop`) |
| `control.py` | no | control leases: `ControlManager` (take / release with the arm home and base steering resets / carried-object park refusal / footprint triggers / state / lease refusals / vitals digest), `wait_steering_neutral` |
| `footprint.py`, `footprint_geometry.py` | no | navigation footprint: `NavFootprint` (static / dynamic / conservative, arm shadow, carried circle, republish tick following the released arm, consumers and their outcomes via `FootprintSink`), `python -m mcp_server.footprint` (parked-arm shadow for the generator); shared outline / parked-arm union / circle / offset / inflation geometry (also imported by `scripts/gen_nav_footprint.py`) |
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
| `lookup_store.py`, `lookup_tools.py` | no | camera lookup store (JPEG + `index.jsonl`, slack-based retention, orphan sweep; models shared via `ros2_common.lookups`); the recorder hook of the camera tools and the `list_lookups` / `get_lookup_image` / `annotate_lookup` module |
| `camera_scene.py`, `camera_overlay.py`, `camera_candidates.py`, `camera_calib.py` | no | calibrated scene and frames, overlay drawing and candidate grids, candidate sets, sample store and solver wiring |
| `tools.py` | no | MCP tools, `StaticTokenVerifier`, Streamable HTTP app |
| `ros_iface.py` | yes | `RosRobot`: subscriptions, TF, Nav2 action, publishers, services; the footprint sink (costmap / ArmStop polygon publishers, `published_footprint` subscriptions, `set_parameters` clients with outcome reports and timeouts, the republish timer, the consumer readiness poll, the arm pose) |
| `spin.py` | no | `spin_forever` (executor loop that survives rclpy stale-handle errors), `run_or_exit` (exits the process when the executor thread dies while ROS is up, so systemd restarts the node instead of serving stale data; an error after ROS was shut down is the normal end), `FrameCache` (latest camera frame) |
| `metrics.py` | no | Prometheus `mcp_*` metric objects, the per-feed sample age collector |
| `serve.py` | no | uvicorn config; `timeout_graceful_shutdown` 3 s so the claude_agent's open MCP stream cannot stall `systemctl stop` until the 30 s SIGKILL |
| `__main__.py` | yes | entry point: executor thread + uvicorn |

## Metrics

`GET /metrics` on the MCP port (18200, Prometheus text format) is scraped by Grafana Alloy on the robot. It needs no
bearer token but answers loopback clients only (127.0.0.1, ::1; anything else gets 403), so the 0.0.0.0 bind does not
expose it on the LAN. The MCP path and the admin route stay behind the token. `robot_node_info{node="mcp_server"}` and
`robot_node_start_time_seconds` come from `ros2_metrics.register_node_info` at start-up.

| Metric | Where it is updated |
|---|---|
| `mcp_tool_calls_total{tool,outcome}` (ok/error), `mcp_tool_duration_seconds{tool}` | `McpServer.call_tool` in `tools.py`, the same wrapper that writes the `tool_call` timing log line (also for failing calls) |
| `mcp_motion_queue_depth` | steps waiting in the motion queue (the running group is not counted); set on enqueue, when the worker takes a group, on cancel/stop and when a group ends |
| `mcp_motion_steps_total{kind,status}` | `MotionQueue.finish_locked`: every step that ends, with the outcome status (`converged`, `stopped`, `aborted_tracking`, `unreachable`, `precondition_failed`, `refused`, `grasped`, `succeeded`, ...); a blended arm step that is passed through without a stop counts as `blended` |
| `mcp_motion_tracking_error_rad{kind}` | worst joint tracking error of a finished queued motion (the group's last step kind) |
| `mcp_grasp_plans_total{mode,outcome}` | `plan_grasp`; mode is the requested strategy (auto/scoop/angled/top_down), outcome feasible/infeasible |
| `mcp_grasp_attempts_total{mode,result}` | `GraspExecutor.grasp`, which `grasp_object`, the queue's `grasp` step and the web UI `execute` all use; see the mapping below |
| `mcp_gripper_effort` | last gripper effort seen in an arm state or an arm motion result |
| `mcp_grip_profile_uses_total{profile}` | every `close_until_effort` close, by resolved profile name |
| `mcp_floor_guard_slowdowns_total{reason}` | arm motions the slow zone slowed; reason is the checked point that came closest to the surface (`elbow`, `wrist`, `jaw_tip`, `tool_point`, `moving_jaw_tip`) |
| `mcp_self_collision_rejections_total{obstacle}` | arm motions refused before they started because a link would hit the robot body (`ArmController.check_body`: executed motions, queue enqueue checks, home routes with no clear route) |
| `mcp_self_collision_slowdowns_total{obstacle}` | executed arm motions slowed inside the body soft margin, by the obstacle that came closest |
| `mcp_motion_first_setpoint_seconds{kind}` | `ArmController.stream` (every arm/gripper motion, queued ones too): motion start to the first published trajectory setpoint; kind `arm` or `gripper` (gripper-only motion) |
| `mcp_motion_tick_period_seconds{kind}` | `ArmController.stream`: interval between consecutive trajectory setpoints (0.04 s at 25 Hz; larger = ticks the host could not keep up with) |
| `mcp_self_collision_min_clearance_m` | smallest arm-to-body clearance of the last executed arm motion (m) |
| `mcp_robot_events_total{severity}` | `RobotMonitor.emit` (debounced repeats are not counted) |
| `mcp_sample_age_seconds{feed}` | age of the latest cached sample per ROS feed (`RosRobot.latest` keys, via `staleness.sample_ages`), read at scrape time; a feed without a sample has no series |
| `mcp_lookup_writes_total{tool}`, `mcp_lookup_write_failures_total` | `LookupRecorder.record` (`lookup_tools.py`): camera images recorded in the lookup store, and the ones it failed to record (the camera tool still answered) |
| `mcp_nav_goals_total{result}`, `mcp_nav_goal_duration_seconds` | `nav_result` in `base_motion.py`, so every Nav2 goal that was sent (`succeeded`, `canceled`, `timeout`, `interrupted`, `rejected`, ...); errors before a goal exists (Nav2 not available) raise and are not counted |

### Grasp result mapping

`mcp_grasp_attempts_total` counts one attempt per `GraspExecutor.grasp` call that got past grip profile validation.
The `result` label is derived from `GraspResult.outcome` (`metrics.GRASP_RESULT_BY_OUTCOME`):

| `GraspResult.outcome` | `result` | Meaning |
|---|---|---|
| `grasped` | `lifted` | closed on the object, verified, lifted and retreated while holding it |
| `unverified` | `unverified` | jaw closed into the empty band (`closed_no_contact`): kept closed with the profile squeeze, lifted and retreated; soft/squashed object or nothing, the agent looks at the attached image |
| `missed` | `missed` | contact that failed verification (or a blocked close): opened again, lifted and retreated |
| `aborted` | `aborted` | stop, refusal or unexpected motion status: stopped and held |
| `infeasible` | `infeasible` | no feasible plan, nothing moved |

`mode` is the strategy that was requested (`auto` stays `auto`).

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
  imu_level_reference:    # IMU roll/pitch (deg) on the calibration floor, subtracted from the IMU tilt
    roll_deg: 0.7         # BNO055 mounting bias (= swerve tilt guard default); re-capture after a floor height recalibration
    pitch_deg: 1.3
grasp:                    # grasp planner defaults, overridable per call via params (see Grasp macros)
  approach_distance_m: 0.04
  pre_grasp_clearance_m: 0.05
  approach_speed_scale: 0.3     # free moves to the lifted pre-grasp and its roll (capped to arm_max_speed_scale)
  slide_speed_scale: 0.15
  lift_height_m: 0.05
  retreat_distance_m: 0.05
  jaw_thickness_m: 0.008
  jaw_open_margin_m: 0.015
  fixed_jaw_clearance_m: 0.0075  # centred grasps: each jaw at least this far off the object's side face
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
  min_jaw_overlap_m: 0.008      # vertical approach: fixed-jaw tip at least max(this, fraction x height) below the top
  min_jaw_overlap_fraction: 0.4
  shallow_overlap_min_m: 0.004  # floor clamp leaves less overlap: shallow_overlap warning, infeasible below this
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
  gripper_grasp_min_travel_rad: 0.10  # closure from the start required for 'grasped' (else 'blocked')
  gripper_empty_stall_rad: 0.08       # a stall/contact this close to closed is an empty jaw ('closed_no_contact')
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
  swerve_joint_states: /swerve_drive/joint_states   # steer joints for the release_control base reset
footprint: {length_m: 0.47, width_m: 0.386}
topdown: {default_radius_m: 2.5, default_px: 480, arm_reach_m: 0.41, plan_max_age_s: 30}
objects: {store_path: /var/lib/ros2/objects/objects.json, merge_radius_m: 0.25}  # store_path: legacy file, imported once into poi_store
look_around: {default_captures: 4, clearance_margin_m: 0.10, step_timeout_s: 30, mode: spin, spin_speed_rps: 0.4,
              spin_rate_hz: 20, spin_timeout_margin_s: 10, return_to_start: false}
motion_queue: {max_steps: 32, event_history: 200, status_events: 10, wait_events: 40, wait_default_s: 30,
               wait_max_s: 120, poll_s: 0.05, base_still_linear_mps: 0.02, base_still_angular_rps: 0.05,
               holding_min_effort: 100.0, holding_min_gap_rad: 0.05}
poi: {request_timeout_s: 3.0, near_radius_m: 5.0}
lookups:                  # camera lookup store (see Camera lookups); pruned oldest first on every write
  store_dir: /var/lib/ros2/poi/lookups
  max_records: 3000
  max_bytes: 300000000    # total JPEG bytes (300 MB)
  max_age_days: 14
control:                  # control leases: release_control base reset (see Control leases)
  steer_joint_names: [fl_steer, fr_steer, rl_steer, rr_steer]
  steer_neutral_tolerance_rad: 0.05
  base_reset_timeout_s: 6.0   # swerve idle_recenter_s (3.0) + 3 s
  steer_poll_s: 0.1
  steer_states_wait_s: 0.5    # lazy /swerve_drive/joint_states: first-message wait after subscribing
  steer_states_idle_s: 30.0   # ... destroyed once unread this long
nav_footprint:            # optional; published navigation footprint (see Navigation footprint)
  circle_segments: 16       # generated by scripts/gen_nav_footprint.py (do not edit by hand) ...
  close_gap_m: 0.01
  laser_filter_margin_m: 0.01
  stopbox_margin_m: 0.02
  simplify_m: 0.005
  decimals: 4
  costmap_resolution: 0.025
  max_cost_scaling_factor: 25.0
  parked_arm_shadow: [[0.3225, -0.0674], ...]   # ... down to here
  costmap_footprint_topics: [/local_costmap/footprint, /global_costmap/footprint]
  inflation_layer: inflation_layer
  arm_stop_topic: /collision_monitor/arm_stop_polygon
  collision_monitor_node: /collision_monitor
  arm_stop_polygon: ArmStop
  laser_filter_node: /laser_filter
  laser_polygon_param: polygon
  frame_id: base_link
  republish_s: 5.0
  refresh_threshold_m: 0.01
  conservative_reach_m: 0.5
  param_timeout_s: 2.0
  consumer_poll_s: 0.5
monitor:                  # all optional; thresholds of the body monitor (see Body awareness)
  servo_temp_warn_c: 60
  servo_temp_critical_c: 70
  cpu_temp_warn_c: 75
  cpu_temp_critical_c: 82
  stall_s: 1.0
  bump_warn_mps2: 4.0
  bump_critical_mps2: 9.0
  bump_min_samples: 2
  tilt_warn_deg: 10          # IMU fallback warning, only while /tilt_guard/state is silent
  tilt_guard_stale_s: 3.0    # /tilt_guard/state older than this is stale (last level still enforced; nav refused at ok)
  tilt_cancel_attempts: 3    # Nav2 cancel tries at a tilt warning/error entry, then tilt_nav_cancel_failed
  tilt_cancel_retry_s: 1.0
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

## POI zones

`add_poi(kind='area', zone=..., speed_limit_pct=...)` and `update_poi(id, zone=..., speed_limit_pct=...)` write the zone fields of the area POI in `poi_store`; the `costmap_zones` node turns active zones into Nav2 keepout and speed masks.

- `zone='keepout'`: Nav2 does not plan or drive into the area. A robot already inside can leave. Never send a navigation goal inside one.
- `zone='slow'` with `speed_limit_pct` (10..100): MPPI is capped to that percentage while the robot centre is inside.
- `zone='none'` (default) removes a zone. Switching to `slow` needs `speed_limit_pct` in the same `update_poi` call.
- Active only while `status` is `open`; `done` / `cancelled` switch the zone off and keep the record.
- Zones created by the agent are session-scoped: agent POIs are removed at the next session start.
- Client-side checks mirror `poi_store` and raise `ValueError` before any request: zone on a point, `slow` without a percentage, percentage with a zone other than `slow`.
