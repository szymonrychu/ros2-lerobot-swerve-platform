# mcp_server

Robot MCP server for LLM agents (Claude Code and other MCP clients). One rclpy node (`mcp_server`) spun by a
`MultiThreadedExecutor` in a background thread, plus an MCP **Streamable HTTP** app (official MCP Python SDK 2.x,
`mcp.server.mcpserver.MCPServer`) served by uvicorn. Runs natively on the client RPi 5 (`ros2-mcp_server.service`).

- Endpoint: `http://client.ros2.lan:18200/mcp` (host/port/path from config)
- Auth: static bearer token from the environment variable `MCP_SERVER_TOKEN` (the server refuses to start without a
  token of at least 24 characters); verified in constant time by `StaticTokenVerifier` (`Authorization: Bearer ...`)
- Admin route: `POST /admin/clear_agent_pois` (plain HTTP, same bearer token, NOT an MCP tool so the model cannot call
  it). It sends the poi_store `clear` command for `created_by: "agent"` and returns poi_store's answer.
  Optional JSON body `{"statuses": ["done", "cancelled"]}` (any of `open`, `done`, `cancelled`) clears only agent POIs
  in those statuses (`[]` clears nothing); no body clears every agent POI. An invalid body or status is `400` with an
  `error` and nothing is cleared:
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
| `get_camera_image(camera, max_px<=1024, zoom)` | Default size `limits.default_camera_px` (384 px on the longest side; ask for more only to read detail, images fill the context: a 384x216 picture is about 110 tokens). Waits for the next frame on an on-demand per-camera subscription (created by the first call, destroyed by the executor's reaper timer after `on_demand.linger_s` unused, never from the tool thread: per-call subscriptions destroyed from tool threads raced the executor and killed it), with timeout. `gripper`: `/camera_0/image_raw/compressed` (JPEG passed through or downscaled once); `front`: `topics.head_camera` (default `/head_camera/image_raw/compressed`, the head RGB camera: Camera Module 3 on the pan/tilt head with the ToF, head_camera_cpp, 1152x648 at 30 fps, 16:9, autofocus, default picture 384x216; JPEG passed through or downscaled once, best view for judging gripper-to-object position). Returns MCP image content (JPEG) + metadata: capture stamp, the picture's `width` x `height`, `calibrated_image` (the size the pixel tools work in: the camera's intrinsics size, `null` for `front` when no intrinsics are configured), `scale_to_calibrated` and a `pixel_note` saying to pass `image_width` / `image_height` with pixels read from this picture, and `lookup_id` (the picture's photo POI id, null when it was not recorded; see [Photos (photo POIs)](#photos-photo-pois)); error if no frame within `image_timeout_s` or the frame is older than 1 s. **`zoom`** (optional `{u, v, factor}`; `u`, `v` in 0..1 are fractions of the default 16:9 picture, top-left origin; `factor` 2, 4 or 8): an autofocused full-resolution crop from the head camera via `robot.camera_zoom` (`head_camera_cpp` zoom command/result/image topics). `front` only (the gripper gives a tool error: zoom is only available on the front (head) camera). `max_px` caps the crop (default 1024, at most 1024; the crop covers 1/factor of the picture width, about 1152x648 at factor 4 of the 4608x2592 frame, then downscaled to `max_px`). Cost: the stream pauses about 1 s, 1-2.5 s per call, one crop per call, 8 s timeout (`timeouts.zoom_timeout_s`). Returns the crop JPEG plus metadata `status` (`ok` or `af_failed`: focus did not converge, the crop is still returned with a `note` and re-taking is allowed), `crop`, `full`, `out`, `width`, `height`, `af_state`, `lens_position`, `phase_ms`, `stamp_s` and `request`. `busy`, `timeout`, `camera_error`, `invalid`, an unanswered request (`ZoomTimeout`) and a head camera that is not running (`ZoomNodeDown`) are tool errors naming the status. A zoom crop is not recorded as a lookup (no photo POI, no `lookup_id`); without `zoom` the tool behaves as above. |
| `list_lookups(since, until, near{x,y,radius_m}, poi_id, text, camera, mcp_session, limit<=20)` | Past camera images (photo POIs), metadata only, newest first (default 10). Sensor. See [Photos (photo POIs)](#photos-photo-pois). |
| `get_lookup_image(lookup_id, max_px)` | One recorded image (JPEG, exactly as the model received it; `max_px` only downscales) plus its metadata. Sensor. |
| `annotate_lookup(lookup_id, note, poi_ids)` | Set a photo's note and/or POI links (replacing them; omitted fields stay) through a poi_store `update`; errors when poi_store is down. Sensor. |
| `delete_photos(ids, older_than_days, keep_newest, dry_run=false)` | Delete photos (index lines and JPEGs) through a poi_store `delete_photos` with exactly one selector; `dry_run` only reports. Sensor, no motion. |
| `get_body_state` | Body vitals (sensor, always allowed): per-servo latest temperature / load / current / voltage / status flags with data age, hottest servo, battery V, per-cell V, margin to cut-off and cut-off state, IMU roll/pitch/tilt and last bump, wheel slip residual, commanded vs measured base speed, CPU temperature and firmware throttling flag, active source + lease, last 10 events. Missing data is `null` with a reason in `notes`. |
| `get_map_summary(include_png, radius_m)` | Nearest `/scan_filtered` obstacle in 8 sectors around base_link, `/map` size and known/occupied/free cells, robot pose, `obstacles_3d` (head ToF cloud `topics.tof_points`, `/tof/points_filtered`, subscribed lazily, latest message only, placed in base_link with the TF at the cloud stamp: per 45 deg sector the nearest obstacle 0.06-0.6 m above the floor with its height range and the nearest cliff below -0.10 m, within 2 m; only what the head currently sees; `null` plus a note when the cloud is missing, older than `timeouts.tof_points_stale_s` (1 s) or has no TF), optional small PNG of the map around the robot. |
| `navigate_to_pose(x, y, yaw, frame='map', timeout_s, precise=false)` | Nav2 `NavigateToPose`; by default the goal ends early (cancelled, base zeroed, status `succeeded`) as soon as the measured pose is within `nav.intermediate_xy_tolerance_m` / `nav.intermediate_yaw_tolerance_deg` (3 cm / 5 deg); `precise=true` waits for Nav2's own checker (`nav.goal_xy_tolerance_m` / `goal_yaw_tolerance_deg`, 1 cm / 2 deg, slower). Blocks until result/timeout (goal cancelled on timeout or stop); returns result and final pose. The description states the goal precision from `nav.goal_xy_tolerance_m` / `nav.goal_yaw_tolerance_deg` (default 1 cm / 2 deg, keep equal to the Nav2 goal checker) and that a sideways goal first turns the robot toward the path (front leading) and turns back to the goal heading at the end. |
| `move_relative(dx, dy, dyaw, timeout_s, precise=false)` | Same (incl. `precise`), goal given in base_link (converted to a map goal via TF). Small moves (a few cm) really move. |
| `drive(vx, vy, wz, duration_s<=2)` | 20 Hz on `/cmd_vel_nav` (through velocity smoother + collision monitor), clamped to 0.25 m/s / 0.5 rad/s, then zero. |
| `stop` | Always available: clears the motion queue first (dropped job ids in `motion_queue_dropped`), cancels all NavigateToPose goals and publishes a zero twist. Aborts any arm motion and holds the arm at its measured pose only if this server holds arm control (or a motion is running); otherwise the arm is not touched (`arm_held: false`). |
| `get_arm_state` | Joint positions (measured follower values)/efforts, gripper effort, tool point pose (x, y, z, pitch; forward kinematics with `arm.joint_offsets_rad` applied), `floor_z_m` (floor height in the base_link frame), active source, lease, home stored. |
| `take_control(subsystem: arm\|base)` | Take a control lease (see Control leases); moves nothing, a no-op when already held. `arm` starts the autonomy lease (publishes the measured pose on `/filter/autonomy_joint_commands`; needs fresh joint states); `base` sets the mcp_server-internal base lease. Returns `ControlState`. Allowed under battery cut-off and at any tilt level. Replaces `acquire_control`. |
| `release_control(subsystem: arm\|base\|all = all, keep_position=false, carried_object_radius_m: float\|None = None)` | Release leases, parking first by default: arm drives home (the `arm_home` routes) then publishes the release (`std_msgs/Bool` true on `/filter/autonomy_release`); base cancels Nav2, zeroes the twist and waits for neutral steering; the head goes home once the call leaves no lease held (`reset.head`, see Control leases). Always releases; returns `ReleaseResult {released, reset, state}`. Refused while the motion queue runs, and parking the arm is refused while the gripper carries an object. An arm release with `keep_position=true` publishes the dynamic navigation footprint (static outline + arm shadow + optional carried circle of `carried_object_radius_m`, 0..0.3 m, keep_position only; it then follows the released arm), see [Navigation footprint](#navigation-footprint). |
| `get_control_state` | Both leases (sensor): `arm {held, held_since, at_home, home_stored, active_source, lost}`, `base {held, held_since, steering_neutral}`. |
| `move_arm_joints(targets, speed_scale<=0.5, settle=None)` | `settle`: `trajectory_end` (default; `final` when the call moves the gripper joint, `limits.arm_default_settle`) or `final`, see the safety model.  Interpolated motion to joint targets (follower joint radians, as in `get_arm_state`). Unnamed joints keep their last commanded target. `speed_scale` 0.5 is the maximum, `limits.arm_max_joint_velocity_rps` (default 1.0 rad/s); the description states the configured value. A `wrist_roll` change above `limits.roll_guard_min_change_rad` is refused while the gripper is open wider than `limits.roll_max_gripper_open_rad` (see Wrist roll guard). `converged` results may carry `residual_error` (see Safety model). |
| `move_arm_cartesian(x, y, z, pitch=None, frame='base_link', speed_scale, wrist_roll=None, object_width_m=None, settle=None)` | `settle` as for `move_arm_joints`.  ikpy IK on `nodes/web_ui/urdf/so101_arm.urdf` (5-DOF: position + approach pitch); `wrist_roll` (rad, measured space, clamped to the limits; clamping is listed in `clamped`) is the roll the IK keeps for this target and the motion rolls to, omitted = the current roll is kept; `object_width_m` (m, 0 < w <= 0.08) makes (x, y, z) the OBJECT CENTRE (see Grasp shift); `unreachable` is reported, never guessed. `base_link` here is the arm URDF root (arm mount, z = 0). The floor is at `z = -arm.arm_base_height_m` (default 0.104, measured 2026-10-10; see Arm mount); the tool descriptions and `get_arm_state.floor_z_m` state it. Motions near or below the effective surface are slowed (never blocked) by the floor slow zone; `surface_z_m`, `tilt_override_deg` and `surfaces` (also on `move_arm_joints`, `set_gripper` and `arm_home`; see Surface regions) shift it per call and results report `slow_zone`. |
| `set_gripper(open_fraction | close_until_effort, effort_threshold, grip_profile, surfaces)` | Open to a fraction (0 closed, 1 open) or close with a grip profile (see Grip profiles) until `abs(effort) >= threshold` (default: the profile's `contact_effort_threshold`) or the closing load reaches the profile's `target_load` (then hold: `grasped`, else `closed_no_contact`). A stall or effort contact only counts as `grasped` when the jaw closed at least `limits.gripper_grasp_min_travel_rad` (0.10 rad; was 0.15 until a 39 mm jar was refused at 0.147 rad on 2026-10-10) from where it started AND stopped no more open than `limits.gripper_grasp_max_open_rad` (1.2 rad; a nearly open jaw that stalls is pushing on something); otherwise the status is `blocked` with "jaw stopped at X rad after Y rad travel - likely pressing on an object rather than holding it", and the measured jaw position is held (no squeeze). A stall or contact within `limits.gripper_empty_stall_rad` (0.08 rad, about 6 mm) of closed is an empty jaw: `closed_no_contact` with "closed without contact: the jaw stopped X rad before closed ...", never `grasped` (2026-10-10: the empty firm close stalled 0.055 rad before closed at load 148). `arm.gripper_closed_rad` / `gripper_open_rad` are follower gripper joint positions (defaults -0.165 / 1.5 rad; measured fully closed is -0.172 rad, URDF lower limit -0.1745, so the jaws close fully). With `close_until_effort` the load is ignored for `gripper_effort_ignore_s` (0.3 s, motor start-up spike) and then counts only once the jaw moved `gripper_contact_travel_rad` (0.03) or stalled with the command at least `gripper_stall_lead_rad` (0.15) ahead of it (or already at the close target: a jaw stopped less than 0.15 rad before closed can never see a larger lead). Stalls are classified by position: a jaw that stalls (that stall criterion: it moved less than `arm_settle_motion_rad` over `arm_settle_window_s`) at least `gripper_empty_stall_rad` from closed is a contact even when its load stays below `target_load` and its error exceeds the settle tolerance (an object loading the jaw lightly never settles within it): the close ends at once and the stall becomes `grasped` (hold = the stalled position plus the profile's `squeeze_rad` toward closed) or `blocked` by the travel/open rules above; a stall inside the empty-jaw band stays `closed_no_contact`. Until 2026-10-11 such a stall ended `timeout` and `grasp_object` aborted. A gripper-only motion near the floor or the robot body is slowed only by what moves (see Floor slow zone and Body collision): the 2026-10-11 10:27 close (jaw tip 12.5 mm above the floor, the stationary upper arm 14.9 mm from the lidar) ran 325 ticks instead of 64. The arm joints keep being held at their intended targets while the gripper moves (see Safety). A `close_until_effort` that ends `grasped` is not trusted on the sensors alone (see Grasp verification): the result carries `grasp_check` (`grasp_id`, sensor `band`, `evidence`, `image`, `crop`, `fused`) and a fresh gripper camera image cropped to the jaws (the `status` stays the sensor's `grasped`, the message says `awaiting confirm_grasp(grasp_id=...)`); the hold counts as `unverified` until `confirm_grasp` fuses your look with the band. |
| `plan_grasp(object, strategy='auto', params, approach_pitch_deg, surface_z_m, tilt_override_deg, grip_profile, surfaces)` | Dry-run grasp plan (sensor, no motion): outcome `planned` / `infeasible` with reasons, the resolved `grip_profile` (an unknown profile is refused), chosen strategy, pitch, wrist roll, jaw opening, waypoints and slow-zone annotations. See Grasp macros. |
| `grasp_object(object, strategy='auto', params, approach_pitch_deg, surface_z_m, tilt_override_deg, grip_profile, surfaces)` | Plan and execute (motion): outcome `unverified` for every close that holds something (verified contact or empty-band hold; it attaches a fresh gripper camera frame cropped to the jaws, at most `grasp_check.image_px` (640) px, as an image content item, and `grasp_check.grasp_id` for `confirm_grasp`) / `missed` / `aborted` / `infeasible` (these attach no image; `grasped` is only the band-only outcome of the web UI and the motion queue, which attach no image either), with the executed steps (each with the `residual_error` of arm joints off their intended target), the final gripper position/load, the grip report (`grip_profile`, `holding_load`, `slipping`, `crush_risk`, `retightened`; see Grip profiles), `residual_error` and `warnings` (joints off their target beyond their converge tolerance) and `held_pose` (measured jaw centre and tool point against the last waypoint). See Grasp macros. |
| `release_object(params, surface_z_m, tilt_override_deg, surfaces)` | Open to `release_open_fraction` and lift `release_lift_m` straight up (motion): outcome `released` / `aborted`. The way out of an `unverified` grasp whose image shows empty jaws, or of a `confirm_grasp` that came back `missed` (then retry). |
| `confirm_grasp(grasp_id, object_between_fingers: yes\|no\|unsure, evidence: 1..500 chars)` | Record your look at the jaw crop of the last held close (sensor class: moves nothing, needs no lease, not a motion tool) and get `{grasp_id, band, image, fused, evidence, carrying, next}`. Fuse table in Grasp verification. An unknown or superseded `grasp_id` (a later close or an open) is a tool error and changes nothing; calling again overwrites the earlier verdict. |
| `suggest_approach_pose(x, y, z \| object_id, z_m \| poi_id, z_m \| camera, u, v, surface_height_m, pitch_deg, max_results<=10, check_path=true)` | Where to park the base for a grasp (sensor, no lease, no motion). Exactly one target form; `z` / `z_m` is the grasp height above the floor (stored objects and point/area POIs have none, so it is required: nothing is invented; a marker `poi_id` takes the marker's own `z` when `z_m` is omitted, and `z_m` still overrides it). Inverts the committed reach map (`data/reach_map.npz`, loaded at start-up: a stale or missing file is logged as an error and both approach tools then refuse with the regenerate command) over every base yaw, scores the poses on the global costmap with the CURRENT `NavFootprint` points (dock pose, staging pose `approach.staging_offset_m` behind it, corridor between), verifies the top `approach.verify_top_k` with `point_reach` and plans the top `approach.path_check_k` with `ComputePathToPose` (planning only). Returns ranked `candidates` (`pose`, `staging_pose`, `reach` with margins, `costmap`, `verified`, `path`, `score`), `rejected` counts (`out_of_reach`, `collision`, `unknown_space`, `staging_blocked`, `corridor_blocked`, `below_margin`, `not_verified`, `no_path`), `costmap_age_s`, `reach_map {key, created_at}`, `docking {server_ready, action}` and `notes`. Refuses without a global costmap newer than `approach.costmap_stale_s`. |
| `approach_for_grasp(target, pose, pitch_deg, prefer_mode='auto', timeout_s, dry_run=false)` | Drive to the best approach pose (motion, needs the base lease, battery gate, tilt gate `nav`, refused while the motion queue runs). Runs the suggest pipeline (or re-validates the given candidate `pose`; one that is no longer valid is refused), then picks the mode: `servoed` only when a calibrated camera (front, or the gripper camera at the current measured joints) sees the target from both the staging and the dock pose (`edge_margin_px` from every border), else `navigate` (a precise Nav2 goal) with the reason in `mode_reason`; `prefer_mode='servoed'` is refused when it is not possible. Servoed: a `DetectionLoop` (`approach.tracker`) publishes real detections on `/grasp/detected_dock_pose` while `RobotApi.dock` runs the DockRobot goal (`dock_type grasp_servoed`, `navigate_to_staging_pose`), and is stopped in a `finally`. The base motion lock, `monitor.base_motion()` and the `BASE_INTERRUPTS` watch are those of `navigate_to_pose` (stop, a critical event or the timeout cancel the goal and zero the base). After a success the target is re-checked with `point_reach` at the measured pose (`reach_after`); unreachable is `arrived_out_of_reach`, never `succeeded`. Result: `status` (`succeeded`, `arrived_out_of_reach`, `failed`, `rejected`, `interrupted`, `canceled`, `timeout`, `planned` for a dry run), `mode`, `mode_reason`, `error_code` / `error_name` / `num_retries` (DockRobot 90x; a failed servoed dock suggests `prefer_mode='navigate'`), `goal`, `final_pose`, `reach_after` (with the `point_reach` `suggestion` when out of reach), `detections_published`, `detections_rejected` (tracker jumps beyond `max_correction_m`), `duration_s`. `dry_run=true` sends no goal and publishes no detection or twist: it returns `plan`, `mode`, `would_send` (the DockRobot goal or the navigate goal), `docking` readiness and, in servoed mode, `tracker` (one step on a fresh frame: the detection pose it would publish, or why none). |
| `arm_home` / `arm_set_home` | Move to / store the home pose. `arm_home` needs the arm lease and keeps it afterwards (also after a failed motion); the `/arm/home` service (web UI Home button) still always releases. |
| `pixel_to_ground(camera, u, v, surface_height_m=0.0, image_width, image_height)` | Point seen at a pixel (sensor). **u, v are pixels of the calibrated image size** (the intrinsics' size: gripper 640x480; `get_annotated_camera_image` and `mark_candidate_points` images) unless `image_width` / `image_height` give the size of the image the pixel came from: `get_camera_image` returns 384x216 (front) or 384x288 (gripper) by default, and an unscaled small-image pixel lands at the wrong place (2026-10-10). Both sizes are needed and must keep the calibrated aspect (a crop is refused); the result then also reports `pixel_input` (what was passed and the scale). Returns `pixel` (calibrated size), `image` (calibrated size), `surface_height_m`, `ground_base_link`, `ground_map` (when the map pose is known), `distance_from_base_m`, `bearing_deg`, `method`, `uncertainty_note`. `surface_height_m` (-0.5..0.5) is the height of the surface the pixel lies on relative to the robot's floor (positive above, negative below; top of a 3 cm box 0.03, a floor 10 cm lower -0.10): the ray is intersected with the plane at floor + that height and the returned z is on it. Error `camera <name> not calibrated: ...` until intrinsics and mount are configured. See [Camera tools and calibration](#camera-tools-and-calibration). |
| `get_annotated_camera_image(camera, overlays=['grid'], planned_gripper, grid_step_m=0.1)` | JPEG with metric overlays (`grid`, `reach`, `gripper`, `planned_gripper`, `lidar`) plus metadata. |
| `mark_candidate_points(camera, region, spacing_px=40, max_points=40, surface_height_m=0.0, image_width, image_height)` | Image (calibrated size) with numbered dots on a pixel grid plus a table `{set_id, surface_height_m, image {width, height}, points:[{n, u, v, ground_base_link{x,y,z}, ground_map}]}`; the points are intersected with the plane at floor + `surface_height_m` (as in `pixel_to_ground`); the last 10 sets are kept. `region` is in calibrated pixels unless `image_width` / `image_height` give the size it was read in. |
| `resolve_candidate(set_id, n)` | Stored coordinates of one numbered point (its u, v in the calibrated `image` size), with its age and whether the base/arm moved since. |
| `capture_calibration_sample(camera, u, v, ground_x, ground_y, ground_z=0.0, image_width, image_height)` | Store one marker sample (u, v scaled from `image_width` x `image_height` to the intrinsics' size when given; needs the intrinsics) (pixel + measured floor point + parent-link pose + the raw measured arm `joints` when fresh joint states exist) in `cameras.calibration_dir`. |
| `solve_camera_calibration(camera, initial)` | Fit the mount pose to the stored samples; returns `rms_px` and a YAML snippet for `client.yml` (never edits the config). |
| `clear_calibration_samples(camera)` | Delete the stored samples of a camera. |
| `get_topdown_view(radius_m=2.5, layers=all, px=480)` | Robot-up PNG centred on the robot (see Perception and memory) plus metadata `pose`, `scale_m_per_px`, `layers_present`, `layers_missing` (reason each), `data_ages`. Sensor. |
| `list_frontiers(max_count, reachable_only)` / `plan_frontier_visit(id)` | Frontiers of the SLAM map (numbered, stable ids) with geodesic distance, bearing, observation pose, nearest POI and last facing lookup; plan one visit with a Nav2 check. Sensors, no lease. See [Frontiers](#frontiers). |
| `get_exploration_status()` / `mark_survey_spot(note, x, y)` | Is the surrounding area mapped and surveyed, and is the battery above the survey stop limit; mark where you looked around. Sensors, no lease, no motion. See [Exploration status](#exploration-status). |
| `go_to_frontier(id, timeout_s)` / `look_toward_frontier(id, camera)` | Effectors (base lease, battery and tilt gates): Nav2 drive to a frontier's observation pose; in-place turn plus one recorded camera image toward it (no travel). See [Frontiers](#frontiers). |
| `set_head_angles(pan_deg, tilt_deg)` | Move the pan/tilt head (degrees; pan + left, tilt + down, 0/0 level ahead). Lease `any` (arm or base), a motion tool (refused under battery cut-off), not in `RATE_SUBSYSTEMS`. Outside the URDF limits it is refused naming the limits in degrees. Returns status (`converged`/`timeout`/`no_feedback`), commanded and measured pan/tilt in degrees. |
| `head_look_at(x, y, z, frame='base_link'\|'map')` | Aim the RGB camera axis once at a point (`HeadKinematics.look_at`, no tracking) and move there. `map` points are converted with the current robot map pose (refused when unknown). Unreachable points are refused with the pan/tilt they would need and nothing is published. Same lease/battery rules; also returns the target in base_link. |
| `head_home()` | Move to the configured head home (`head.home_pan_rad` / `head.home_tilt_rad`, forward and down). Same lease/battery rules. |
| `remember_object` / `list_objects` / `forget_object` | Object memory in map coordinates, stored as object POIs in poi_store (merge, distance/bearing from the robot). Sensor. |
| `look_around(captures=4, camera='front', mode=None, return_to_start=None)` | Effector, ONE motion call: in-place turn through equal headings with a camera frame and lidar summary per heading. `mode` `spin` (default, `look_around.mode`): one continuous slow rotation; `steps`: one precise Nav2 rotation per heading. Ends facing the last heading unless `return_to_start=true` (default `look_around.return_to_start`, false). |
| `enqueue_motions(steps, replace=false, watch_constraints=None)` | Effector: queue motion steps and return at once (job ids, queue length, blend groups); `watch_constraints` abort or pause the queue on a sensor condition. See [Motion queue](#motion-queue). |
| `resume_motions` | Effector (battery gate): continue a queue paused by a watch constraint; refused while a watch of the pending steps still violates; a no-op message when not paused. |
| `check_guard_rules(tool, arguments={})` | Sensor, read-only: the tool-phase guard rule verdicts (`pass` / `reject` / `skip`, basis, message, observed values) a call would get now; no counters, no logs, nothing moves. Arm-target rules are evaluated at motion start, so they are not predicted. |
| `get_motion_status` | Queue snapshot (sensor): running, current step (elapsed, phase, planning_s, progress, blend group), pending steps, last events. |
| `cancel_motions` | Uncapped control: drop the pending steps and abort the running one (base zeroed, arm held when a motion step runs). |
| `wait_for_event(timeout_s, until='queue_empty', since_seq)` | Sensor: block until the queue drains or something relevant happens; returns the new events, the queue status and a compact state digest. |
| `clear_pois(creator='agent', kinds=None, include_zones=False)` | Clears POIs in one call through `poi_store` `clear`: `creator` agent (default) / user / all (all sends no `created_by`), `kinds` subset of point/area/object/photo (default every kind, so your own photos go too; the person's photos only with creator user/all), zones kept unless `include_zones=true` (`keep_zones`). The person's POIs or zones only when the person asked. Photos removed lose their images. Returns `{"removed": n, "pois": a, "photos": b}`. Sensor, no motion. |
| `save_map(name, overwrite=false, note='')` | Saves the current map, its POIs and the last robot pose as the named map `name` through `map_manager`. Refuses an existing name unless `overwrite`. Errors when map_manager is down or the name breaks the rule. No motion. |
| `list_maps` | Saved maps (name, saved_at, note, poi_count, last_pose, valid), the active map and the localisation (state, ratio, pose). Sensor, read-only; errors until map_manager has published a status. |
| `load_map(name, x=None, y=None, yaw=None)` | Loads the named map (the current map and POIs are first kept as `autosave`) and relocalises; returns the verdict (state, ratio, pose, reason). Calls stop first (halts the motion queue and zeroes the base); that is the only motion, no servo moves. Pose hint needs x, y and yaw together. Not navigable unless the state is `localised`. Consent default: none needed, but the load replaces the live map and POIs, so do it only when asked. |
| `delete_map(name, confirm=false)` | Deletes a saved map and its POIs for good. Consent default: `confirm=false`, nothing is sent until the person agreed and the call repeats with `confirm=true`. No motion. |
| `list_pois` / `add_poi` / `update_poi` / `delete_poi` | Points and areas of interest through `poi_store` (`/poi/*`). Sensor. Areas can carry a Nav2 zone: `add_poi` / `update_poi` take `zone` (`none` / `keepout` / `slow`, `none` removes it) and `speed_limit_pct` (int 10..100, required for `slow`, refused for any other zone); `list_pois` returns both on every view and never lists photos (`list_lookups` does); `list_pois(marker_kind=...)` keeps only marker POIs of that kind. **Marker POIs** (kind `marker`: fixed places from printed markers - base, charger, drop-off, button, switch, tool station; the pose is owned by the marker detector, `add_poi` stays point/area only) are listed with the others; their views carry `z` (height, m), `marker` (`kind`, `id`, `size_mm`, `params`, `times_seen`, `last_seen`, `confidence`), `facing_yaw_deg` (map yaw of the marker's +z axis projected on the floor; null when that axis is within 45 deg of vertical, a marker lying flat or on a ceiling) and `approach_pose` (`x`, `y`, `yaw_deg`: the marker position plus `markers.standoff_m` (default 0.6, 0.2..2.0) along the facing, yaw facing the marker; null without facing); all null for other kinds. As approach targets (`suggest_approach_pose`, `approach_for_grasp`) a marker `poi_id` takes its own `z` when `z_m` is omitted (`z_m` still overrides); point/area `poi_id` still requires `z_m`. `delete_poi` with a photo id deletes that photo and its image, deleting a POI keeps the photos linked to it. `keepout`: Nav2 will not plan or drive into the area (the robot can leave if it is inside; never set a goal inside). `slow`: MPPI speed cap while the robot centre is inside. A zone is active only while the POI status is `open`. A zone on a point, `slow` without a percentage, or a percentage with another zone is refused before the request. Agent zones are session-scoped (agent POIs are cleared at each new session). See [POI zones](#poi-zones). |
| `get_3d_map_status` | Sensor, read-only: the latest `/map3d/status` JSON (mapping state, node count, alignment) plus the local cloud's `voxel_count`, `bounds` (map frame, m), `age_s` and `alignment_agreement` (0..1, the share of measured walls at the lidar plane that lie on the slam_toolbox `/map` walls; low means the 3D map is shifted). Fields are null with a note when nothing was received, never a guess; a result older than `map3d.max_age_s` carries a stale note. |
| `query_3d_map_at(x, y, radius_m=0.3)` | Sensor, read-only. Map-frame x, y and radius in m (0.05..2). The highest measured obstacle within the radius (`height_m` above the floor in m, `highest_point`, median colour), floor and below-floor voxel counts and the lidar `/map` cells. `source`: `depth_3d`, `lidar_2d` (a `/map` wall depth never measured: `height_m` is null, `lidar_min_height_m` is only a lower bound), `both`, or `unknown` (nothing observed there, which is not free). Errors when no 3D map was received yet. |
| `get_3d_free_space(x, y, radius_m=1.0, z_min_m=0.02, z_max_m=None)` | Sensor, read-only. Map-frame x, y; `radius_m` 0.2..5 m; the height band `z_min_m`..`z_max_m` in m above the floor (default top = `map3d.robot_height_m`). For 8 bearing sectors (`bearing_deg` 0 = map +x, counter-clockwise) the distance `free_m` to the nearest obstacle inside the band, plus `observed` (0..1). `free_m` is null for a sector nobody observed (unknown is never free); lidar-only walls block every height. Errors when no 3D map was received yet. |

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
| `front` (head RGB camera) | `base_link` (x forward, y left, z up) | `z = 0`: base_link is the ground-projected centre between the wheels (the static TF puts the lidar 0.20 m above it) | a link of the head URDF `head.urdf_path` (default and deployed `head_link`; moves with the pan/tilt head) |
| `gripper` (on the wrist) | arm base frame (URDF `base_link` of `so101_arm.urdf`, `z = 0` on the arm mount plane) | `z = -arm_base_height_m` (`floor_z_m`, -0.100 m with the measured mount) | URDF link `gripper_link` (child of `wrist_roll`, the rigid gripper body carrying the fixed jaw; `gripper_frame_link` is only the tool point at the jaw tips and `moving_jaw_so101_v1_link` moves with the gripper joint, so neither is a valid mount) |

For the gripper camera `T_arm_base_gripper_link` is computed from the CURRENT measured joints with the same ikpy chain
as IK/FK (`ArmKinematics.link_frame`), so the tools need fresh `/follower/joint_states`. The front camera works the
same way on the head: `T_base_link_head_link` comes from the CURRENT measured `head_pan` / `head_tilt`
(`HeadKinematics.link_pose`, `camera_scene.parent_transform(..., head=...)`), so the front camera tools need fresh
`/head/joint_states` (an error naming them otherwise, never a fixed-pose fallback). Lookups record the head-composed
`camera_pose_base` the same way (null when the head joints are stale), and the approach tools' camera view check
(`camera_view_problem`) builds the front scene with the current measured head pose, assuming the head keeps it while
the base moves; stale head joints give the reason `head joint states stale`. The deployed front mount is the
`head.urdf` `head_link -> head_camera_link` origin (x 0.02, y -0.025, z 0, rpy 0). Mount orientation convention:
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
- Image sizes: the pixel tools work in the calibrated image size (the intrinsics' width x height: 640x480 for the
  gripper camera; the front camera, the head RGB camera streaming 1152x648 at 30 fps, has no intrinsics
  configured yet). `get_camera_image` defaults to 384 px on the longest side (`limits.default_camera_px`; 384x216 for the head camera, 384x288 for the gripper), so a pixel read from it must be passed
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
   several arm poses (move the arm to look at the markers from different joint configurations); for `front` the sample
   stores the `head_link` pose at the measured head angles, so views at several head poses can be mixed. Each sample
   stores the parent-link pose of that moment (computed with `arm.joint_offsets_rad`) and the raw measured joint positions as
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
and `/swerve_drive/servo_registers` (JSON dumps of the arm servos and the swerve servos, about every 10 s: `present_temperature` C, `present_load`, `present_current` raw,
`present_voltage` x 0.1 V, `status` bits), `/imu/data`, the swerve `/odom` twist covariance, `/odometry/filtered`,
`/odom_rf2o_twist`, `/cmd_vel_nav`, `/collision_monitor_state`, `/filter/active_source`, `/tilt_guard/state`, the battery guard and the CPU
temperature (`/sys/class/thermal/thermal_zone0/temp`, throttling from the firmware sysfs `get_throttled`, null if
missing). A 4 Hz timer runs the stall, latched-collision and CPU checks.

State cache (raw, latest-only): the 95 Hz `/follower/joint_states` and the odometry topics (`/odometry/filtered`, swerve `/odom`,
`/odom_rf2o_twist`) are raw (serialized) subscriptions that keep only the latest bytes and the receive time. A message is deserialized
only when a reader asks for it (`RosRobot.get`, the tools), at most once per received message. The monitor hooks of these topics are fed
from the existing 4 Hz monitor tick (no timer of their own) with the latest message only; a topic with no new message since the
previous tick feeds nothing. Messages are never logged individually. `mcp_sample_age_seconds` is unchanged (it reads the receive time, never the message). Thresholds are the `monitor` config section
(`MonitorSettings`, also in `ansible/group_vars/client.yml`).

On-demand subscriptions (`on_demand` config section, `ros_iface.OnDemandSubscription`): profiling showed most of the
idle CPU was rclpy executor wake-ups for messages no tool read. The two cameras (`topics.gripper_camera`,
`topics.head_camera`), `topics.scan` (`/scan_filtered`) and `topics.tf` (`/tf`) are subscribed only while a tool uses
them. The first use subscribes and waits a bounded time for a first message: a camera waits `timeouts.image_timeout_s`
for the next frame; the scan waits up to `timeouts.state_stale_s`; a TF lookup waits `max(timeouts.tf_timeout_s, the
rest of a timeouts.state_stale_s window)` right after `/tf` was subscribed, then `tf_timeout_s`. Every read restarts
the linger. A reaper timer (every quarter of `on_demand.linger_s`, default 30 s) destroys a subscription unused for
`linger_s`. A reaped scan drops its cached sample. A reaped `/tf` clears the TF buffer and re-inserts the stored static
transforms, so a later lookup waits for fresh dynamic transforms instead of returning the old pose. `/tf_static` stays
subscribed (latched, about 0 msg/s). The node subscribes `/tf` and `/tf_static` itself (into a `tf2_ros.Buffer`), not
through a `TransformListener`.

Never on demand (`ros_iface.MOTION_CRITICAL_TOPICS`, subscribed at start at full rate and never destroyed): the
follower joint states and currents (arm settle and convergence, the grasp band), `/filter/active_source` (human takeover),
`/odometry/filtered`, swerve `/odom` and `/odom_rf2o_twist` (stall and slip), `/imu/data` (bump and tilt), `/cmd_vel_nav`,
`/follower/servo_registers` and `/swerve_drive/servo_registers` (temperatures, servo errors), `/tilt_guard/state`, `/collision_monitor_state`,
`/filter/input_joint_updates` (take_control leader check), `/grasp/command`, `/tf_static`, and the battery topic.
The follower joint states are always subscribed, at the rate below.

### Dual-rate joint states

`/follower/joint_states` and `/swerve_drive/joint_states` arrive at 95 Hz, and each message wakes the rclpy executor
(about 0.3-0.4 core while nothing moves, measured 2026-10-13). The
`joint_state_throttle_follower` / `joint_state_throttle_swerve` nodes (`ros-jazzy-topic-tools` `throttle messages
<topic> 10.0 <topic>/throttled`, see `ansible/README.md`) publish 10 Hz copies. `ros_iface.DualRateTopic`
(`joint_state_rate` config section) reads them like this:

- **Idle:** only `<topic><throttled_suffix>` (default `/throttled`) is subscribed. It is a raw subscription for the
  follower and best effort, which matches the source and the copy. The cache key and every reader stay the same.
- **Motion:** `RosRobot.begin_motion(subsystem)` runs before a motion of that subsystem. It subscribes the full-rate
  topic and waits, at most `timeouts.state_stale_s`, until a full-rate sample is cached. The motion's start pose,
  planning and first setpoint come after that wait. The arm switches in `ArmController`'s `MotionGuard`, which every
  arm motion enters: move, cartesian, blend, path, gripper open and close (before the rest baseline), home, grasp,
  release and jogs, from the MCP tools, the queue, the web UI `/grasp/command`, `/arm/home` and the `release_control`
  park. The base (only the swerve feed) switches in `RosRobot.navigate`, `move_relative`, `drive`, `spin` and `dock`.
  The MCP server also switches once per tool for the whole tool body (`tools.RATE_SUBSYSTEMS`: every lease-checked
  motion tool for its subsystem, plus `release_control` for both).
- **Back to idle:** full rate stays until `joint_state_rate.linger_s` (default 5 s) after the last motion of the
  subsystem ended (overlapping motions are counted). Then the throttled copy is subscribed again.
- **No gap, no placeholder:** until the newly selected subscription delivers, the old one keeps feeding the cache.
  After that, only the selected one does. Subscriptions are destroyed only from the executor's 4 Hz monitor tick,
  never from a tool thread. A throttled sample is subject to the same freshness limits as before
  (`timeouts.follower_stale_s` for the arm, `timeouts.state_stale_s` for the steering), so a stale one is never acted
  on. Nothing is ever synthesised.
- **Fallback:** the copy may have no publisher (throttle node down), or it may be silent for `timeouts.state_stale_s`.
  Either way the feed reads the full-rate topic and logs one warning per outage. It retries the copy one
  `linger_s` later, once it has a publisher again.
- `joint_state_rate.enabled: false` reads only the full-rate topics (rollback). A topic field in
  `ros_iface.FULL_RATE_AT_REST` (a consumer that needs full rate at rest, per the audit below) is also read at full
  rate permanently. The list is empty: no consumer needs full rate at rest.
- `mcp_sample_age_seconds{key="joint_states"}` reads up to about 0.1 s while idle, instead of about 0.01 s.

### Joint-state consumer audit

Every reader of the two dual-rate topics in this node (`ros_iface`, `arm`, `jog`, `tools`, `models`, `monitor`,
guards, grasp, settle, effort/contact and stall/collision detection) and whether it needs the full rate while its
subsystem is idle. A `yes` row would put its topic field in `ros_iface.FULL_RATE_AT_REST` (checked by
`tests/test_joint_state_dual_rate.py`).

| Consumer | Topic | Full rate when idle | Why |
|---|---|---|---|
| `ArmController.stream` (all arm motions): stale-feedback abort, tracking-error abort, convergence, settled residual, close-until-effort contact (load delta, effort, target load), jaw stall contact | `follower_joint_states` | no | Runs only inside a motion, and `MotionGuard` switches to full rate before it reads the start pose |
| `ArmController.rest_baseline` (gripper close baseline, `contact_baseline_samples` distinct samples) | `follower_joint_states` | no | Runs inside `set_gripper`'s `MotionGuard`, after the switch |
| `hold_report`, `retighten_hold`, `grasp_check` (slip, crush risk, hold current and jaw) | `follower_joint_states` | no | Run inside the grasp's `MotionGuard`, or within the linger right after the close |
| `keepalive_tick` hold republish, `relax_due_locked` (`arm_settle_hold_s` 2 s), settle probe (1 s after a trajectory end) | `follower_joint_states` | no | The republish sends the last setpoint and reads no sample. Relax and probe read one fresh sample, inside the 5 s linger (at 10 Hz still well within `follower_stale_s` 0.3 s) |
| `acquire` / `take_lease` / `ensure_lease` (take_control holds the measured pose), `stop_hold`, `finish` holds | `follower_joint_states` | no | One fresh sample (at most about 0.1 s old at 10 Hz, limit 0.3 s); a hold is not a motion |
| `ArmController.state`, `at_home`, `measured`, `get_arm_state`, `get_robot_state`, `get_control_state`, `ControlManager.carrying_state` | `follower_joint_states` | no | Point-in-time reads of the measured pose and load |
| Guards: `GuardState.fresh`, `measured_pose`, the watch-constraint sampler (`motion_queue.watch_rate_hz` 10 Hz) | `follower_joint_states` | no | Point reads at tool entry and dispatch; the sampler runs while queued motions run (full rate) |
| Motion queue: `read_live_state`, `state_digest`, `body_reasons`, `solve` | `follower_joint_states` | no | Point reads before dispatch; each step enters its own `MotionGuard` |
| `jog.py` (web UI jogs: base pose, opening, preview) | `follower_joint_states` | no | Point reads; the jog itself is a `move_joints` / `move_cartesian` / `set_gripper` motion (`MotionGuard`) |
| Grasp and approach planning: `grasp_tools` plan, `intent_errors`, `held_pose`, `jaw_crop_jpeg`; `approach_tools` IK seed; `lookup_tools.arm_joints`; `body_tools.get_arm_reach`; `camera_tools` arm pose | `follower_joint_states` | no | Point-in-time reads of the measured pose |
| Navigation footprint (`RosRobot.arm_parts`, `ControlManager.publish_footprint`) and self-collision preview | `follower_joint_states` | no | The measured pose of an arm at rest, one sample per footprint tick |
| `RobotMonitor` (overheat, servo error, stall, slip, bump, collision stop, tilt, human takeover) | `follower_joint_states` | no | Gets no joint-state feed: its arm interrupts come from servo registers, active source and the arm controller |
| `models` (`ArmState`, `JointSample` results) and `metrics` (`mcp_sample_age_seconds`) | `follower_joint_states` | no | Built from the samples above or from the receive time; the age gauge reads about 0.1 s while idle |
| `RosRobot.steer_positions` (release_control steering check, `get_control_state`) | `swerve_joint_states` | no | One fresh sample within `timeouts.state_stale_s` 1 s; still lazy (subscribed on first read, reaped after `control.steer_states_idle_s`) |
| Base motions (`navigate`, `move_relative`, `drive`, `spin`, `dock`) and the base monitor | `swerve_joint_states` | no | Read odometry, TF, IMU and cmd_vel, never the swerve joint states; they switch it to full rate anyway (contract) |
| `LeaderActivity` (take_control refused while the leader arm moves) | `leader_joint_updates` | no | Not a follower or swerve state and not dual-rate: the leader relay publishes only while the leader arm streams (under 1 Hz in the 2026-10-13 idle profile), so its subscription is unchanged |

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
| `nav_abort` (source `nav2`) | warning | Nav2 aborted the goal of `navigate_to_pose`, `move_relative` or `go_to_frontier` (the tool result has status `aborted`); `data` = `{tool, message}`; never debounced; raised by `call_tool_with_digest` before the digest is taken |
| `<type>_cleared` | info | a level condition (overheat, servo_error, battery_*, tilt, wheel_slip, stall, cpu_overheat) ended |

Events are debounced per (type, source) (`debounce_default_s` 30 s, shorter for bump 2 s / collision_stop 1.5 s / stall 1.5 s /
human_takeover 5 s, so a repeat during a retried motion is not hidden; an escalation always passes). Each event is published on **`/robot_events`** (`std_msgs/String`, reliable,
depth 50) as JSON `{"seq": int, "ts": float (epoch s), "type": str, "severity": "info"|"warning"|"critical",
"source": str, "message": str, "data": object}`; `seq` starts at 1 per process. The monitor fans every event out to
all its sinks in order (`set_sink` replaces them, `add_sink` appends; a failing sink is logged and the next one still
runs): the `/robot_events` publisher first, then the auto stuck-point marker.

**Auto stuck points (`auto_poi.py`).** `AutoPoiMarker.on_event(event, pose)` turns a stuck event (`auto_poi.event_types`,
default `collision_stop` and `nav_abort`) at a fresh map pose (`robot_pose()`, at most `auto_poi.max_pose_age_s` 2 s old)
into a point POI: kind `point`, `created_by: agent`, name `stuck: collision stop` / `stuck: Nav2 abort`, note
`stuck <n> times, last <local time> (<polygon or Nav2 message>)`. A repeat within `auto_poi.merge_radius_m` (0.5 m) of an
open agent `stuck:` point, or inside an open agent `stuck:` area, updates only that POI's note (count and time) instead of
adding one. No pose, a stale pose or a non-map pose means no POI (never a placeholder). It never creates an area or a
keepout (a keepout would block Nav2): the agent upgrades the point itself. The added or updated id is put on the digest
entry of that event as **`auto_poi_id`** in `robot_events_since_last_call` (only on events that got one; `/robot_events`
JSON is unchanged). `ros_iface` wires it with `attach_auto_poi`; the marker runs on the delivering thread and waits at
most `poi.request_timeout_s` for poi_store, and the digest waits (up to 5 s) for events still being delivered, so a
collision_stop that ends a motion has its `auto_poi_id` in that motion's result. A poi_store failure is logged
(`mcp_server.auto_poi`) and never breaks the event path or the tool result. `auto_poi.enabled: false` turns it off.

**`poi_hint` nudge.** The server counts `get_camera_image` and `look_around` results since the last agent POI write
(`tool_context.PoiWriteCounter`, one counter per server). From the `auto_poi.hint_after_images`-th result on (default 3,
at least 1), that result carries a **`poi_hint`** string next to `robot_events_since_last_call` and `vitals` (in the digest
text block and in `structured_content`, or `_meta` for image results) telling the agent to mark what it saw with
`add_poi` / `remember_object` and link the photo with `annotate_lookup`. It repeats on every further image result until a
successful `add_poi`, `update_poi`, `remember_object` or `annotate_lookup` resets the count; the next image result after
that write has no `poi_hint`. Other tools never carry it, auto stuck points and the photo POIs the camera records do not
reset it, and a failed write does not either.

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
done, grey cancelled), `objects` (remembered objects as diamonds with labels) and `frontiers` (cyan frontier cells with a numbered marker at each
frontier target, the `list_frontiers` id; see [Frontiers](#frontiers)). A layer without data is **never
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

## 3D map

Module `map3d_tools.py` (one entry in `tools.TOOL_MODULES`; the voxel model is in the rclpy-free `map3d.py`). The three
tools `get_3d_map_status`, `query_3d_map_at` and `get_3d_free_space` read the RTAB-Map point cloud that the server's
`map3d_manager` publishes and the client receives as `/map3d/cloud` (`topics.map3d_cloud`, PointCloud2, best effort) and
`/map3d/status` (`topics.map3d_status`, JSON). Both are subscribed on demand like the scan: the first call subscribes and
waits up to `map3d.cloud_wait_s` for a cloud, the subscription ends after the linger time without a use.

The cloud is a snapshot of everything RTAB-Map has built, in `map3d.frame_id` (`rtabmap_map`, identical to the `map`
frame by the alignment to slam_toolbox). It is refreshed about every 10 s (the downlink is throttled to 0.1 Hz) and only
contains what the head RGB-D camera has seen, so an area the head never looked at is absent. The tools voxelise it at
`map3d.voxel_m` (5 cm).

Measured versus lidar-only: a cell is **measured** when depth saw points there (`source: depth_3d`, a height in m above
the floor). A wall that only the slam_toolbox `/map` shows is **lidar-only** (`lidar_2d`): its height is unknown (null),
only a lower bound (`lidar_plane_height_m`, the laser plane) is given, and it blocks every height band in
`get_3d_free_space`. Cells nobody observed are `unknown`, never free. A cloud older than `map3d.max_age_s` still answers
but every result carries a stale note.

Config (`map3d:` block in `ansible/group_vars/client.yml`, defaults shown): `frame_id: rtabmap_map`, `voxel_m: 0.05`,
`lidar_plane_height_m: 0.20`, `robot_height_m: 0.55`, `max_age_s: 120.0`, `cloud_wait_s: 12.0`.

## Photos (photo POIs)

A photo is a POI of kind `photo` owned by `poi_store` (see `nodes/poi_store/README.md`): a camera image plus the camera
pose at capture. Every successful camera tool image (`get_camera_image`, `get_annotated_camera_image`,
`look_toward_frontier`, the grasp jaw crop) is recorded automatically by `LookupRecorder.record` (`lookup_tools.py`):

1. it gathers the robot context (map pose, arm joints, camera pose in base_link) and the camera `view`;
2. it writes exactly the JPEG the model received to `<lookups.store_dir>/<id>.jpg` (`LookupStore.write_image`, atomic,
   mode 0644; default `/var/lib/ros2/poi/lookups`, next to poi_store's `poi.json`);
3. it sends a poi_store `add` with `{kind: "photo", ...PhotoAdd}` (the shared `ros2_common.photos.PhotoAdd` payload;
   poi_store sets `bytes` from the file and appends the `index.jsonl` line) and waits up to `poi.request_timeout_s`;
4. it returns the id, which the camera tool reports as `lookup_id`, so the agent can link the picture to a POI right away.

When poi_store is not running or refuses the add, the JPEG is removed again; when it does not answer in time the JPEG
is kept (poi_store may still index it, otherwise its orphan sweep removes it). Both, like a failed JPEG write, are
logged on `mcp_server.lookups` and counted in `mcp_lookup_write_failures_total`, and never fail the camera tool, whose
`lookup_id` is then null.

mcp_server never writes `index.jsonl`: `LookupStore` (`lookup_store.py`) is a read-only view (`records`, `get`,
`query`, `image`) over the shared stat-keyed `IndexCache` (`shared/ros2_common/lookups.py`), plus `write_image` /
`remove_image`. Index lines that do not parse are logged once per distinct line and skipped. Retention, annotation
writes and deletes all happen in poi_store.

| Field | Meaning |
|---|---|
| `id`, `ts`, `tool` | uuid hex (also the POI id), unix time of the capture, the camera tool that took it |
| `camera`, `width`, `height`, `file`, `bytes` | camera, image size (px), JPEG file name relative to the store, JPEG size |
| `base_pose_map` | `{x, y, yaw}` in map at capture, null when the pose is unknown |
| `arm_joints` | measured arm joints (rad), null when stale |
| `camera_pose_base` | `{xyz, rpy}` of the camera body in base_link (fixed-axis rpy), when the camera has a mount (the gripper camera also needs `arm.base_in_base_link` and fresh joints; the front camera is composed with the measured head angles and needs fresh head joint states); null otherwise |
| `view` | camera pose in map from `ros2_common.photos.camera_view`: `x`, `y`, `z`, `yaw` (camera forward axis), `pitch` (positive looking down), `hfov`, `source` (`camera_mount`, or `base_heading` without a mount: base position and yaw, z / pitch / hfov null); null without a map pose |
| `mcp_session` | Mcp-Session-Id of the call (a contextvar set in `RobotMCPServer.call_tool_with_digest`; empty without one). mcp_server cannot see claude_agent's instruction id: time range plus session stand in for it |
| `kind`, `name`, `status`, `created_by`, `updated_at` | `photo`, label, `open` / `done` / `cancelled`, `agent` / `user`, unix time of the last edit (0 = never) |
| `note`, `poi_ids` | empty until `annotate_lookup` (or `update_poi` for the note) sets them |

`view.hfov` is `2*atan(width / (2*fx))` from the camera's configured intrinsics (`cameras.<camera>.intrinsics`), only
for a full frame: `record_lookup(ctx, tool, camera, jpeg, w, h, full_frame=True)` keeps its positional contract, and the
grasp jaw crop passes `full_frame=False` (a crop's field of view is not the camera's), so its hfov is null. Without
intrinsics hfov is null too; nothing is invented.

Photos survive the agent session reset (poi_store's `clear` never touches them). Legacy index lines (written before
photos were POIs) read as photos with the defaults above; `list_lookups`, `get_lookup_image` and `annotate_lookup`
return their `view` derived on read from the stored pose (`photo_summary` in `lookup_store.py`, via
`ros2_common.photos.effective_view`), the same view poi_store lists for them.

`list_lookups` filters combine with AND: `since` / `until` (inclusive; unix seconds or an ISO-8601 time like
`2025-10-09T08:53:20Z`, naive means UTC), `near` (map distance of
`base_pose_map`; records without a pose never match), `poi_id` (member of `poi_ids`), `text` (case-insensitive
substring of `note` only; notes are empty until `annotate_lookup` sets them, so an empty result of a `text` search carries a
`hint`), `camera`, `mcp_session`; `limit` 1-20 (default 10). Every returned field is named in the tool description.
`list_pois` does not list photos; its description points at `list_lookups`.

`annotate_lookup(lookup_id, note, poi_ids)` refuses ids that are not photos in the index, rejects `poi_ids` that are not
existing POIs (checked against `/poi/list`), deduplicates them and sends a poi_store `update {id, note?, poi_ids?}`;
the result is the re-read record in the `list_lookups` shape. It errors when poi_store is not running.

`delete_photos(ids, older_than_days, keep_newest, dry_run=false)` takes exactly one selector (a ToolError otherwise) and
sends a poi_store `delete_photos` command whose `selector` field is a shared `PhotoSelector`
(`older_than_days` becomes `older_than_s`). The result is `{removed, bytes_freed, missing}` or, with `dry_run`,
`{would_remove, bytes}`. `PoiRequests.request` puts the payload of this op in the command's `selector` field instead of
`poi`. `delete_poi(id)` and `update_poi(id, name, note, status)` accept photo ids as well; their descriptions say so.

## Frontiers

Module `frontier_tools.py` (one `tools.TOOL_MODULES` entry) on top of the rclpy-free `frontiers.py`. A frontier is the
boundary between known free space and unknown space on the SLAM `/map`: where the map ends and exploring can go on. The
server only supplies geometry (reachability, distance, bearing, nearest POI, whether a camera already looked that way);
the agent ranks the frontiers itself. No ROS topic is published (nothing to fake: a value that cannot be computed is
`null` with a line in `notes`).

**Algorithm (`frontiers.py`).** Computed lazily, only when a tool or the topdown layer asks, and cached per `/map`
version (`MapGrid.version` = stamp sec, nanosec, width, height, origin x, y; slam_toolbox publishes every 10 s).

1. *Detection, a vectorised Wavefront Frontier Detector.* Frontier cells = unknown cells with an 8-neighbour known-free
   cell (`binary_dilation`); clusters = `scipy.ndimage.label` with 8-connectivity; size, centroid and bbox from
   `ndimage`. The work is cropped to the bbox of known cells plus one cell, so unknown padding costs nothing. A map
   border cell counts only when it touches free space. Clusters shorter than `frontiers.min_length_m` are dropped. Each
   frontier has `centroid` (mean of its cells, may lie off the frontier) and `target` (the frontier cell nearest the
   centroid, always on the frontier). This is the output of the classic two-level-BFS WFD without a Python loop.
2. *Traversability.* Known free cells whose distance to the nearest non-free cell (unknown counts as an obstacle,
   `distance_transform_edt`) is at least `footprint.width_m / 2 + pass_clearance_margin_m`.
3. *Geodesic ranking.* One `scipy.sparse.csgraph.dijkstra` from the robot over the traversable grid (8 neighbours,
   costs 1 and sqrt 2 times the cell size, on a grid `geodesic_downsample` times coarser than the map). One pass gives
   the distance to every candidate; cached per (map version, robot cell). A robot cell that is not traversable (hugging
   a wall) snaps to the nearest traversable cell within 0.3 m, else every frontier is `reachable: false` with a note.
4. *Observation pose.* Candidates are known free cells with clearance of at least the current circumscribed radius of
   the navigation footprint (`NavFootprint.circumscribed_radius()`, the static half-diagonal when `nav_footprint` is
   off) plus `view_clearance_margin_m`, at a distance from the nearest frontier cell inside
   `[standoff_min_m, standoff_max_m]`. Score = geodesic distance + abs(distance - mid standoff). The pose is the first
   of the best `los_candidates` whose straight line to the frontier crosses no occupied and no unknown cell; yaw faces
   the target. **Never a pose in unknown space**: no pose, no goal.
5. *Nav2 check, chosen pose only.* `plan_frontier_visit` asks Nav2 (`ComputePathToPose`, `use_start: false`,
   `topics.planner_id`) for one path to the observation pose. It is not run per frontier: SmacPlannerLattice may spend
   up to 2 s per plan on the busy Pi, and an external plan request can preempt the planner while a navigation runs, so
   the check is skipped with a note while a navigation is active (`nav_active()`: the base_motion lock is held or
   Nav2 reports an accepted/executing goal).

**Stable ids, epochs, retired successors.** `FrontierTracker` (one per server, thread-safe, `ToolContext.frontiers`)
matches each new snapshot to the previous one with `scipy.optimize.linear_sum_assignment` on centroid distance, gated by
`match_radius_m`. A matched cluster keeps its integer id (matching is in map coordinates, so map growth and origin
shifts do not renumber); new clusters get the next integer. An id that disappears is *retired* with a `reason`:
`explored` (no successor), `merged into N` or `split into N` (`successor` = the nearest current id within twice the
match radius). The last `retired_keep` retired ids are remembered, and `list_frontiers.retired_since_last_call` reports
the ones retired since the previous call. A retired or unknown id in a tool gives a `ToolError` such as `frontier 7 is
gone (explored); nearest current frontier: 9`. When known cells fall below `epoch_reset_known_fraction` of the previous
map (a SLAM reset) or the resolution changes, a new **epoch** starts: everything is cleared, ids restart at 1 and the
list notes `map reset: frontier ids restarted`. Ids are only comparable inside one `epoch`.

**Tools.** `list_frontiers` / `plan_frontier_visit` are in `ALWAYS_ALLOWED_TOOLS`; the two motion tools are in
`MOTION_TOOLS`, `TOOL_SUBSYSTEM` (`base`) and `TILT_NAV_TOOLS`, and are also refused (like `look_around`) by the lease,
battery and tilt gates.

| Tool | Class | Fields |
|---|---|---|
| `list_frontiers(max_count=None, reachable_only=false)` | sensor, no lease, no motion | `frontiers[]`: `id`, `centroid {x,y}`, `target {x,y}`, `length_m`, `cells`, `reachable`, `path_distance_m` (geodesic to the observation pose, null when unreachable), `straight_distance_m`, `bearing_deg` (from the robot: 0 ahead, + left), `observation_pose {x,y,yaw}` or null, `nearest_poi {id,name,kind,distance_m}` within `poi_radius_m` or null, `last_lookup_facing` (below) or null. Top level: `map_version`, `map_age_s`, `epoch`, `retired_since_last_call [{id,reason,successor}]`, `compute_ms` (per stage; 0 = served from the cache), `robot_pose`, `notes`. Reachable first, by path distance; capped at `max_listed`. Errors when `/map` is missing or older than `map_max_age_s`; without the robot pose the distance, bearing and reachable fields are null plus a note. |
| `plan_frontier_visit(id)` | sensor, no lease, no motion | `frontier` (the list entry), `observation_pose`, `reachable`, `path_distance_m`, `rotation_deg` (signed in-place turn from the current heading to face the target, + left), `rotation_clearance` (the `look_around` lidar check of the rotation circle), `nav2_check {status: succeeded/failed/timeout/unavailable, length_m, planning_time_s, error_code, error_msg}` (null or skipped with a note while navigating), `notes`. |
| `go_to_frontier(id, timeout_s=None)` | motion, base lease, battery gate, tilt gate | Recomputes the snapshot, resolves the id, then `navigate` to the observation pose with the published dynamic footprint. Returns the `navigate_to_pose` fields plus `frontier_id`, `observation_pose`, `frontier_still_present` (checked after the move; null when the map did not update). Refused with nothing moving for a gone id or when no reachable observation pose exists. A frontier explored on the way is not an error: the move ends at the pose. |
| `look_toward_frontier(id, camera=frontiers.look_camera)` | motion (base only, the arm never moves), base lease, battery gate, tilt gate | Lidar clearance check like `look_around` (refused without moving), then a precise in-place rotation to face the target (skipped within `nav.goal_yaw_tolerance_deg`), then one camera image recorded as a lookup with tool `look_toward_frontier`. Returns the image plus `frontier_id`, `status`, `rotation_deg`, `heading_error_deg`, `camera`, `image_captured`, `lookup_id`, `nav`, `notes`. No capture when the rotation failed or was interrupted (status passed through). The gripper camera is captured at its current pose; head pan/tilt aiming is not built. |

**Topdown layer.** `get_topdown_view(layers=[..., "frontiers"])` (part of the default `all`) draws the frontier cells in
cyan and a filled marker at each `target` with the `list_frontiers` id in black on white (legend: `frontier (number =
list_frontiers id)`); `frontier_ids_drawn` in the metadata lists the markers that are inside the view. The layer uses
the cached snapshot (the tracker is only called when the layer is requested); when `/map` or the robot pose is missing
it is listed in `layers_missing` with the reason and nothing is drawn.

**Facing lookup (`last_lookup_facing`).** The newest lookup (from the lookup store `index.jsonl`) whose camera axis
points at the frontier `target` within `facing_half_angle_deg`, no further than `facing_max_distance_m` from the lookup
position. Result: `{lookup_id, age_s, camera, distance_m, off_axis_deg}`; prefer frontiers where it is null (nobody has
looked there). The camera axis in the map frame = the record's `base_pose_map` yaw plus the yaw of the body x axis of
`camera_pose_base`. **Head-mounted front camera:** the front camera sits on the pan/tilt head, so its record's
`camera_pose_base` is composed from the measured head angles at capture (pan +45 deg turns the axis 45 deg left of the
base heading). Only legacy records without a pose (taken before the camera was on the head) fall back to the base
heading; a front lookup taken with stale head joint states records no pose (never a base_link guess), and its facing
then also falls back to the base heading, the same as a legacy record. Gripper records without a pose are skipped,
never guessed.

**CPU budget and `mcp_frontier_compute_seconds`.** The Pi 5 runs near 90% CPU, so there is no timer: work happens only
inside tool calls and the topdown layer, once per `/map` version (detection stages) and once per (version, robot cell)
(geodesic). Measured on an M4 (synthetic map with walls): 400x400 cells extract 7 ms, geodesic 3 ms; 1200x1200 extract
114 ms, geodesic 115 ms; expect 3-5x on the Pi. Budget: extract + geodesic p95 <= 300 ms on the real map. Read the
histogram `mcp_frontier_compute_seconds{stage}` from `/metrics` (stages `extract`, `traversability`, `field`, `match`
once per map version; `geodesic` once per map version and robot cell). Every `list_frontiers` result also carries the
per-call `compute_ms`; a repeated call on the same map version shows ~0 for the detection stages. If the sum of the
stage p95 values exceeds 300 ms, raise `frontiers.geodesic_downsample` (2 -> 3). `mcp_frontier_count` is the number of
frontiers of the latest computed version. A frontier that is `reachable: true` here can still fail the Nav2 check (the
geodesic uses half-width clearance, Nav2 the real footprint on a lattice): `plan_frontier_visit` is the authority for
the chosen pose; tune `pass_clearance_margin_m` if the two disagree often.

## Exploration status

`get_exploration_status()` and `mark_survey_spot(note="", x=None, y=None)` (in `frontier_tools.py`, both in `ALWAYS_ALLOWED_TOOLS`, no lease, no motion) tell the agent when the surroundings are completely mapped and surveyed and when to stop for the battery. Coverage maths is `exploration.py`; the survey spot tags are `ros2_common.poi_categories`.

**`mark_survey_spot`** adds an agent point POI named `survey spot` at the robot pose (or at `x`, `y`, both required together) with the note `#survey #survey-spot` on the first line and your note text after it. Error without a robot pose and without `x`/`y`. Result: the poi_store answer.

**`get_exploration_status`** refreshes the frontier snapshot and returns:

- `frontiers[]`: the reachable frontiers (`reachable: true`) with `id`, `path_distance_m` and `looked` (a lookup already faced it, `last_lookup_facing` not null); counts `reachable_frontiers`, `unreachable_frontiers`, `unlooked_frontiers`.
- `survey_spots[]`: open `agent` POIs whose first note line has `#survey-spot` (a `#survey` sighting alone, another creator or a done/cancelled POI does not count).
- `coverage`: share of the reachable free cells (the robot's connected traversable region) that lie within `exploration.survey_radius_m` line of sight of a survey spot, plus the `unsurveyed_areas` (centroid, `area_m2`, largest first).
- `battery`: `{pack_v, stop_v, limit_reached}` from the `BatteryGuard` the motion gates use; `stop_v = cells * (cutoff_cell_v + exploration.stop_margin_cell_v)`, `limit_reached` when `pack_v < stop_v`. All null plus a note without a guard or a fresh reading.
- `complete` and `reasons[]`.

**Completion rule.** `complete` is true only when the robot pose is known AND no reachable frontier is left AND no reachable area is unsurveyed. `reasons` names every unmet condition (the remaining frontier ids, the unsurveyed areas with size and place). With no `/map`, no robot pose or no poi_store the result is null (`complete`, `coverage`, `frontiers`) with a note, never a guess; the battery is still reported.

`exploration:` settings (`ExplorationSettings`, all optional, defaults shown):

| Key | Default | Meaning |
|---|---|---|
| `survey_radius_m` | 2.5 | A survey spot counts free cells it sees within this range |
| `rays` | 72 | Minimum rays per spot (raised to one per cell on the radius arc) |
| `min_unsurveyed_area_m2` | 0.5 | Smaller unsurveyed pockets are ignored |
| `downsample` | 2 | Coverage grid cell = map resolution x this |
| `stop_margin_cell_v` | 0.3 | Survey stop voltage per cell = `battery.cutoff_cell_v` + this |
| `max_unsurveyed_listed` | 10 | Largest unsurveyed areas listed |

Tests: `tests/test_exploration_tools.py`.

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
| -> `warning` | `tilt_warning`, critical (claude_agent interrupts the running instruction at once): roll/pitch vs limits and "automatic navigation is disabled; drive back to level ground manually with short base moves (the drive tool ...), then wait for the guard to clear". From `error` the message says the base drive is enabled again, or, when `base_drive_inhibited` stays true (the inhibit can outlast the error), that the base drive is still inhibited and no base motion tools work yet | every goal cancelled (`cancel_all_goals` from a worker thread, retried `monitor.tilt_cancel_attempts` (3) times `monitor.tilt_cancel_retry_s` (1 s) apart; never answered: a critical `tilt_nav_cancel_failed` event telling the agent to call `stop`), a running `navigate_to_pose` / `move_relative` / `drive` / `look_around` / `approach_for_grasp` ends `interrupted` (`BASE_INTERRUPTS`) | `navigate_to_pose`, `move_relative`, `look_around`, `approach_for_grasp` and `enqueue_motions` with a base step refused; queued base steps fail at dispatch (status `tilt_guard`); `drive` allowed unless `base_drive_inhibited` is still true (then refused like at `error`) |
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
| `MOTION_TOOLS` (refused in cut-off) | `navigate_to_pose`, `move_relative`, `drive`, `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `arm_set_home`, `look_around`, `grasp_object`, `release_object`, `enqueue_motions`, `resume_motions`, `approach_for_grasp` |
| `ALWAYS_ALLOWED_TOOLS` | `stop`, `cancel_motions`, `get_motion_status`, `wait_for_event`, `check_guard_rules`, `plan_grasp`, `suggest_approach_pose`, `get_robot_state`, `get_body_state`, `get_camera_image`, `get_map_summary`, `get_arm_state`, `take_control`, `release_control`, `get_control_state`, `get_topdown_view`, `remember_object`, `list_objects`, `forget_object`, `list_pois`, `add_poi`, `update_poi`, `delete_poi`, `clear_pois`, `list_lookups`, `get_lookup_image`, `annotate_lookup`, `delete_photos` |

`arm_set_home` is classed as motion because it rewrites the pose a later `arm_home` drives to.

## Servo torque-off gate

The web UI Calibrate page can release (torque off) the arm, base steering or head servo group. The feetech bridges
publish each group's state as JSON (`released`, reliable + transient_local, repeated periodically) and mcp_server
reads it so the agent never commands a limp group.

Topics (`topics.*`): `follower_torque_state` (`/follower/torque_state`, arm), `base_torque_state`
(`/swerve_drive/torque_state`, base), `head_torque_state` (`/head/torque_state`, head). `timeouts.servo_torque_stale_s`
(default 3.0, > 0): a state with no sample or older than this is unknown, and unknown refuses nothing (the bridge gate
is the backstop). `RosRobot.servo_released(group)` returns `True`, `False` or `None`.

Tool mapping (`tools.TOOL_SERVO_GROUPS`, one entry per `MOTION_TOOLS` tool): arm tools (`move_arm_joints`,
`move_arm_cartesian`, `set_gripper`, `arm_home`, `arm_set_home`, `grasp_object`, `release_object`) -> arm; base
tools (`navigate_to_pose`, `move_relative`, `drive`, `look_around`, `go_to_frontier`, `look_toward_frontier`,
`approach_for_grasp`) -> base; head tools -> head; `enqueue_motions` -> the groups of its step kinds;
`resume_motions` -> arm and base.

Behaviour while a group is released:

- `call_tool_with_digest` (after the lease check, before the battery and tilt gates) refuses a tool of that group with
  `<group> servo torque is off (released on the web UI Calibrate page); motion refused`; nothing is published.
- `publish_command` (arm) and `publish_head_command` (head) publish nothing and log one debug line, so the 5 Hz
  keepalive and the head home-on-lease-release never reach a limp group.
- A held arm lease is dropped without publishing (`lost_note` "arm torque off", seen in `get_control_state`
  `arm.lost`); filter_node's active source `released` is treated as this case, not a takeover. `take_control('arm')`
  (and the web UI take) is refused with the same message; `stop` does not take the lease.
- `get_robot_state` and `get_control_state` carry `servos_released` (groups with a fresh released state).

Tests: `tests/test_torque_gate.py`.

## Head

`head_tools.py` (`set_head_angles`, `head_look_at`, `head_home`) drives the pan/tilt head through `head.HeadController`
(`RosRobot.head`). Config section `head` (`HeadSettings`): `urdf_path`, `joint_names` (`head_pan`, `head_tilt`),
`command_topic` (`/head/joint_commands`), `state_topic` (`/head/joint_states`), `home_pan_rad` / `home_tilt_rad` (home
looks forward and down, must lie inside the URDF limits), `tolerance_rad`, `move_timeout_s`, `state_max_age_s`. The tools
need either the arm or the base lease (`TOOL_SUBSYSTEM` `"any"`), are in `MOTION_TOOLS` and are deliberately absent from
`RATE_SUBSYSTEMS` (the head is on neither joint-state stream). `get_robot_state` carries `head: {pan_deg, tilt_deg}` only
from a fresh `/head/joint_states` sample.

## Control leases

Every arm and base tool needs a control lease (`control.py`, `ControlManager`). The table `TOOL_SUBSYSTEM` in
`tools.py` (next to `MOTION_TOOLS`) is checked in one place, `RobotMCPServer.call_tool_with_digest`, before the tool
body: so before the battery and tilt gates, and a tool without its lease is refused before anything is published.

| Lease | Tools |
|---|---|
| arm | `move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `arm_home`, `grasp_object`, `release_object` |
| base | `navigate_to_pose`, `move_relative`, `drive`, `look_around` (base-only spin), `approach_for_grasp` |
| per step | `enqueue_motions`: `arm_joints` / `arm_cartesian` / `gripper` / `grasp` steps need arm, `base_relative` / `navigate_to_pose` (`motion_queue.BASE_KINDS`) need base; checked first inside the tool, naming every missing lease |
| none | `stop`, `cancel_motions`, `wait_for_event`, `get_motion_status`, `take_control` / `release_control` / `get_control_state`, `plan_grasp`, `suggest_approach_pose`, `arm_set_home`, camera, perception and POI tools |
| any | `set_head_angles`, `head_look_at`, `head_home`: `LeaseNeed` `"any"` is satisfied by either lease (`ControlManager.refusal(name, "any")`), so a hand-over from one lease to the other never moves the head |

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
| head | Decided last, after the base and arm resets (`ControlManager.release_head`, only when a head is configured: `tools.py` passes `robot.head`). `keep_position=true`: kept. The agent still holds the arm or the base lease after this call (releasing base while holding arm, or the reverse): not moved. The call released no lease (everything `skipped: not held`): no `head` entry, nothing published. Otherwise (the call released a lease and none remains): `HeadController.home()` (`head.home_pan_rad` / `head.home_tilt_rad`), waiting for convergence; skipped under battery cut-off. A head failure never blocks the arm/base release (the releases already happened). | `kept`, `kept: arm lease held`, `kept: base lease held`, `homed`, `failed: <status>` (`timeout`, `no_feedback`), `failed: <error>`, `skipped: battery_cutoff` |

`all` resets the base first (wheels stopped before the arm moves), then the arm, then decides the head. A lease lost
involuntarily (web UI take, `/arm/home`, a lease generation change) does not move the head, and `stop` never touches it.
Releasing `arm` and then `base` in two calls leaves the head in place on the arm release (`kept: base lease held`) and
homes it on the base release. A lease not held is not released (`skipped: not held`). The base lease is cleared before its reset starts: base tools are refused during the up to 6 s
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

### Web UI take / release (`/arm/take_control`, `/arm/release_control`)

Two `std_srvs/Trigger` services let the web UI Arm tab take the arm from the leader (or the agent's idle hold) and hand
it back with one button. Both go through the same `ControlManager` and arm lease as the agent's `take_control`
(`topics.take_control_service`, `topics.release_control_service`).

- `/arm/take_control` acquires the lease for the holder `web_ui` (`ControlManager.take_web_ui`): filter_node's active
  source becomes `autonomy` and the measured pose is commanded, so the arm does not move. Idempotent. Refused (success
  false, message says why) while the agent holds the arm, under battery cut-off, and while the leader arm is actively
  moving (a leader joint moved more than `control.leader_motion_threshold_rad`, 0.02 rad, within
  `control.leader_motion_window_s`, 0.5 s, measured on `topics.leader_joint_updates`, `/filter/input_joint_updates`).
- `/arm/release_control` (`ControlManager.release_web_ui`) publishes the autonomy release and clears the holder; the
  arm stays put and filter_node's proximity rule (0.15 rad) hands it back to the leader once the leader arm is
  close. Refused when the web UI does not hold the arm (the agent's lease is never released by it).
- `/arm/home` is unchanged: it moves home, then releases (which also clears the `web_ui` holder).
- `get_control_state` reports `arm.held_by: "web_ui"` (`arm.held` stays the agent's lease). The startup orphan check
  never drops a `web_ui` lease (this process holds it). An agent `take_control` while the web UI holds the arm adopts
  the lease (the holder becomes `agent`).

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
Interpreter exit stops a still-running computation at its next batch (`stop_running_envelopes`, atexit, up to
`EXIT_STOP_TIMEOUT_S` = 10 s), so the daemon thread does not hold the C++ body kernel through shutdown (nanobind would
report the `BodyChecker` as leaked); nothing partial is stored or cached.
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

## Reach map

`mcp_server/reach_map.py` is an offline inverse reachability map of the arm with the robot body: for every cell of a
grid around the shoulder_pan axis it stores which approach pitches put the tool point there without touching the body.
It is built on the dev Mac and is meant to be loaded at runtime (the approach planner ranks base poses with it); the
robot never computes it, so it costs no robot CPU.

Grid (`reach_map:` config, `ReachMapSettings`; values are the defaults):

| Axis | Values |
|---|---|
| heading | tool bearing from the pan axis in base_link, every `heading_step_deg` (5) inside the bearings the pan range minus the limit margin covers (43 headings, -105 .. 105 deg) |
| r | horizontal distance from the pan axis, `r_step_m` (0.02) .. `r_max_m` (0.48) every `r_step_m` (24) |
| z | tool height in base_link (floor = 0), `z_step_m` (0.025) .. `z_max_m` (0.45) every `z_step_m` (18) |
| pitch | `POINT_PITCHES_DEG` (-30 .. 90 every 15 deg, + down), one bit each |

Build (forward sampling, RM4D-style, no numeric IK): forward kinematics of an even `planar_samples` (72 x 72 x 37)
grid of shoulder_lift x elbow_flex x wrist_flex inside their limits (minus the margin) at pan 0, wrist_roll 0, gripper
closed. The pan axis is vertical, so turning the pan only rotates a sample about it: its r, z and pitch stay and its
bearing moves by the pan angle (`pan_geometry` checks that FK at a pan angle equals the pan 0 FK rotated about the
pan joint axis within 1e-6 m, and that the axis is vertical within 1e-4 rad; the URDF's `rpy="3.14159 ..."` tilts it
2.65e-6 rad, under 2 um at full reach). Each sample belongs to the cell nearest its (r, z) and to the nearest pitch
within 7.5 deg. Per heading every sample whose required pan lies inside the pan range is a candidate; the
`samples_per_cell` (2) nearest each cell centre and pitch are body checked at that pan in batches of
`ENVELOPE_BATCH_SAMPLES` (32). The verdict is `SelfCollisionChecker.clear_of`'s at `body_check.hard_margin_m` (one
pruned pass bounded at the soft margin gives it and the clearance; a test pins the verdict to `clear_of`). Stored in
a compressed `.npz` (written to a temporary file and renamed):

- `pitch_mask` uint16 [H, R, Z]: bit k set = pitch k reachable and collision-free in that cell;
- `clearance` float16 [H, R, Z]: best body clearance of the cell's clear samples (m, saturated at the 3 cm soft
  margin), NaN where no pitch works;
- `meta` JSON: key, version, grid (headings, radii, heights, pitches, pan axis), planar_samples, samples_per_cell,
  compute_s, created_at and counts (samples, checked, clear, reachable cells and cell-pitches).

A cell is reachable when some sample within half a cell of its centre works, so its edge cells are about a centimetre
optimistic; on the 30 deg test grid `point_reach` (IK at the exact centre) confirms at least 90 % of sampled reachable
cells and refutes at least 90 % of the unreachable ones.

Key: sha256 of `arm_reach.reach_inputs(checker)` (body, arm link model, mount, joint limits, joint offsets, limit
margin: the same inputs as the envelope's key, which is unchanged), the tool point offset, the jaw (closed angle, open
axis), the hard and soft margins, the grid fields (everything in `ReachMapSettings` but `file`), the pitches and
`REACH_MAP_VERSION`. `ReachMap.load(path, checker, settings)` raises `ReachMapMissing` (no file or unreadable) or
`ReachMapStale(expected, found)` (built for other inputs); both messages name the regenerate command, so a stale map
is never used silently. The file defaults to the package `mcp_server/data/reach_map.npz` (`reach_map.file`), so it
deploys with the git checkout.

Query: `ReachMap.cells(z, pitch_deg | None)` returns the reachable cells of the height slice nearest z (within half a
step; outside the grid nothing): heading_deg, r_m, the working pitches, clearance and `margin_m`, the horizontal
distance from the cell centre to the nearest unreachable cell centre in that slice (cells outside the grid count as
unreachable: the ring one r step beyond `r_max_m` and the heading rows one step beyond each end of the pan range).
`ReachMap.describe()` gives key, version, created_at, compute_s, grid and counts.

CLI (from `nodes/mcp_server`):

```bash
uv run python -m mcp_server.reach_map build --client-yml ../../ansible/group_vars/client.yml --out mcp_server/data/reach_map.npz
uv run python -m mcp_server.reach_map check --client-yml ../../ansible/group_vars/client.yml  # exit 1 when missing or stale
```

Regenerate after any change to the body, arm mount, joint limits or offsets, tool point, jaw, margins or the
`reach_map` grid. Measured on the M4 dev Mac 2026-10-11 with the deployed client.yml config and the default grid:
82 s (191,808 FK samples, 147,798 body-checked poses, 101,436 clear; 12,838 of 18,576 cells and 52,738 cell-pitches
reachable), 360 MB peak RSS, a 17 KB file. The committed artifact is this build: `uv run python scripts/gen_reach_map.py`
(repo root; `--check` exits 1 when stale) wrote it in 93 s CPU / 2 min 26 s wall, 12,838 reachable of 18,576 cells, 52,738
reachable cell-pitches, 17,188 bytes. `tests/test_mcp_server_config.py` fails (naming the script) when its key no longer
matches client.yml's mcp_server config or its grid differs from the `reach_map:` block there. The hatchling wheel
includes `mcp_server/data/` (package files), and the robot runs from the git checkout, which carries the committed file. Doubling the planar density on the default r / z cells (30 deg headings)
adds 1.9 % reachable cell-pitches.

## Approach planner

`mcp_server/approach.py` (pure, no ROS) ranks base poses from which the arm reaches a target, by inverting the reach
map against the global costmap. Settings: `approach:` (`ApproachSettings`, defaults in parentheses).

Target (`ApproachTarget`): exactly one of `xyz` (map frame, z = height above the floor), `object_id` (ObjectMemory),
`poi_id` (point POI or area centroid) or `pixel` `{camera, u, v, surface_height_m}` (ground point at that height,
z = the surface height). Objects and POIs store no height, so `z_m` is required for them and refused for the other
forms. `resolve_target` resolves it with injected lookups; an unknown object, POI or a pixel that misses the surface
raises `ApproachUnavailable`.

Candidates (`generate_seeds`, `invert`): for every base yaw 0..360 deg every `yaw_step_deg` (10; yaws wrapped to
[-180, 180)) and every reachable cell of `ReachMap.cells(z, pitch)` (heading h from the pan axis in base_link, r):

- pan axis P = T - r (cos(yaw + h), sin(yaw + h)); base B = P - R(yaw) a, a = the pan axis in base_link (from the arm
  mount config);
- `expected_margin_m` = `margin_m` - (`pose_error_xy_m` (0.03) + rad(`pose_error_yaw_deg` (3)) * |T - B|): the cell
  margin left when the base stops off by the pose error (the yaw error swings the target by its lever arm);
- cells below `min_expected_margin_m` (0.0) are dropped (`below_margin`), the best `cells_per_yaw` (30) per yaw kept.
  No cell at the height (or pitch) gives no candidate and an `out_of_reach` note: no fallback pose is invented.

Costmap check (`score_candidates`) on `global_costmap_topic` (`/global_costmap/costmap`, OccupancyGrid translation:
100 lethal, 99 inscribed, -1 unknown). The grid must be fresh: none received or older than `costmap_stale_s` (5)
raises `ApproachUnavailable` with the reason. The current footprint polygon (`NavFootprint.current.points`, passed in)
is rasterised once per yaw with `cv2.fillPoly` (`footprint_mask`) on a 16x finer grid; a costmap cell belongs to the
footprint when more than half of it is covered, which keeps the raster area within 5 % of the polygon at the 2.5 cm
costmap resolution (the base origin is taken at its cell centre, up to half a cell off). A pose is blocked when a
footprint cell is lethal, unknown or outside the grid, or the centre cell is at least inscribed. Each candidate is
checked at the dock pose, the staging pose (`staging_offset_m` (0.30) back along the yaw, where the docking server
starts) and `corridor_samples` (5) poses evenly between them, and counted once, in this order: `collision` (dock
lethal or inscribed centre), `unknown_space`, `staging_blocked`, `corridor_blocked`. The result reports the dock
`max_cost` / `mean_cost` and the staging and corridor maxima.

Score (`score_value`, higher is better; `weights`):

```text
score = margin (1.0) * min(expected_margin_m, margin_cap_m) / margin_cap_m     # margin_cap_m 0.08
      - cost (0.5) * mean_cost / 100
      - travel (0.2) * travel_m          # path length + staging leg when planned, else straight line from the robot
      - heading (0.1) * |h| / 180        # prefer the arm straight ahead
```

`rank` sorts best first. Hooks (injected callables, so the module stays ROS-free):

- `verify_top_k`: the best `verify_top_k` (5) are re-checked with `point_reach` (the grasp planner's IK and body
  check) on the target in the candidate's base_link; failures are dropped (`not_verified`), the rest of the list
  stays unverified (`verified: null`).
- `path_check`: the best `path_check_k` (3) are planned to their staging pose (ComputePathToPose, planning only);
  `no_path` drops the candidate (`no_path`), `ok` replaces travel by the path length plus `staging_offset_m` and
  re-ranks, `unavailable` is kept and reported. Without a robot pose travel is not scored and paths are
  `unavailable: no robot pose`.

`suggest` runs the whole pipeline and returns `ApproachSuggestion` (target, ranked candidates with pose, staging_pose,
reach {heading_deg, r_m, z_m, pitches_deg, clearance_m, margin_m, expected_margin_m}, costmap, verified, path,
travel_m, score; `rejected` counters; `costmap_age_s`; notes). The rest of `approach:` (`docking_threshold_m`,
`planner_action`, `dock_action`, `detection_topic`, `dock_timeout_s`, `tracker`) configures the grasp docking server
interfaces and the camera target tracker; `staging_offset_m` and `docking_threshold_m` must match the grasp docking
server params.

### ROS adapters of the approach tools

`mcp_server/ros_dock.py` (imports rclpy, like `ros_iface.py`) holds the ROS side; `RosRobot` creates the clients at
startup and exposes `dock()`, `plan_path()`, `global_costmap()` and `publish_detection()` on `RobotApi`.

| Interface | Name (`approach:` setting) | Use |
|---|---|---|
| action `nav2_msgs/action/DockRobot` (client) | `/grasp/dock_robot` (`dock_action`) | `RosDockPort`: goal by pose (`use_dock_id` false, `dock_pose` PoseStamped in `map`, `dock_type`, `navigate_to_staging_pose`, `max_staging_time`); feedback state tracked by name (`NAV_TO_STAGING_POSE` .. `RETRY`); result = status, `error_code`, `num_retries`. Own goals are registered like the NavigateToPose goals. |
| action `nav2_msgs/action/ComputePathToPose` (client) | `/compute_path_to_pose` (`planner_action`) | `RosPlannerPort`: plans from the robot's current pose (`use_start` false), planning only. `ok` with the path length, `no_path` (aborted or empty path, with the error code), `unavailable` (server down, no answer, rejected, no result within 5 s). |
| topic `nav_msgs/OccupancyGrid` (subscriber, reliable transient local) | `/global_costmap/costmap` (`global_costmap_topic`) | `GlobalCostmapCache`: subscribed on the first `global_costmap()` call (waits `topdown.costmap_wait_s` for the first message), then the newest grid as a `CostGrid` with `age_s` since it was received. A stale grid is returned with its age; `approach.check_grid` refuses it against `costmap_stale_s`. |
| topic `geometry_msgs/PoseStamped` (publisher) | `/grasp/detected_dock_pose` (`detection_topic`) | `DetectionPublisher`: frame `map` (another frame is refused), header stamp = the image capture stamp passed in. It publishes only when `publish_detection()` is called, never on its own. |

`cancel_all_goals` (the `stop` tool, the monitor's tilt-warning canceller) cancels every NavigateToPose goal and then
every `/grasp/dock_robot` goal with the zero goal id; the docking server's internal staging goal is a NavigateToPose
goal and is covered by the first cancel. A docking server that is not running is not waited for (its cancel service is
only asked when it is up), so a stop is never delayed by it. `reset_base` (the `release_control` base reset) cancels
the NavigateToPose and the docking goals this server sent. `dock()` takes the same base motion lock and stop flag as
`navigate_to_pose` and ends on critical events (`run_dock`).

### Camera target tracker (servoed mode)

`mcp_server/target_tracker.py` (pure: frames, `Scene`s, base poses and the surface height are passed in) is the
detection source of the servoed grasp approach. Settings: `approach.tracker` (`TrackerSettings`).

- `visible(scene, point_base_link, edge_margin_px)`: the precheck that picks the mode. True only when the base_link
  point projects into the image (`Scene.project`) at least `edge_margin_px` (20) from every border; False behind the
  camera, off the image, inside the margin, or when the scene cannot reach base_link (gripper camera without
  `arm.base_in_base_link`).
- `dock_pose_from_target(target_map, dock_yaw, t_rel)`: dock pose = target - R(dock_yaw) t_rel, yaw = dock_yaw, frame
  `map` (`t_rel` = the target in the planned dock frame).
- `TargetTracker(planned_target, dock_yaw, t_rel, settings)`: seeded with a grayscale `template_px` (48) patch either
  around the agent's pixel (`seed_from_pixel`, pixel targets; the estimate becomes that pixel's surface point) or, on
  the first `step` where the projected planned target is in view with room for the patch, around that projection (the
  seed frame returns no detection: matching a template against its own frame would only echo the plan). Each
  `step(frame_bgr, stamp, scene, base_pose_at_stamp, surface_z)` predicts the target pixel from the estimate, rescales
  the template by the predicted distance ratio (seed camera distance / current), runs `cv2.matchTemplate`
  TM_CCOEFF_NORMED in a `search_px` (96, grown with the template) window with a sub-pixel peak, accepts a score >=
  `min_score` (0.6), cuts the pixel's ray with the surface plane (`Scene.ground_point`), takes it to the map with the
  base pose at the capture stamp, and rejects it when it is farther than `max_correction_m` (0.10) from the planned
  target. Counters: `frames`, `accepted`, `rejected` (jumps), `low_score`. The `Detection` carries the dock pose, the
  capture stamp unchanged, the target, the pixel and the score.
- `DetectionLoop(tracker, source, publish, rate_hz)`: a thread that pulls the newest `TrackerInput` from `source` at
  `rate_hz` (5, at least twice the docking plugin's `external_detection_timeout` rate), skips frames whose stamp is not
  newer than the last one, and calls `publish` (`publish_detection`) only for accepted detections. `stop()` (or leaving
  the `with` block) sets the stop event and joins the thread; source, tracking and publish errors are logged and
  counted in `errors` without ending the loop.

No accepted match means nothing is published for that frame, and a stored target position is never re-stamped as a
detection: when detections stop, the docking plugin times out (FAILED_TO_DETECT_DOCK, 904) and stops the base.

Limits today:

- The front camera is not calibrated (no intrinsics and mount in the client config), so it cannot see a target in the
  tracker's sense; the gripper camera is calibrated but, with the arm parked, often does not see the target from the
  staging and dock poses. Until a camera sees the target, approaches run in blind mode (`navigate_to_pose(precise)`)
  and the servoed loop stays unexercised on the robot; the tool's `mode` / `mode_reason` say which mode ran and why.
- The template is rescaled isotropically by distance only: perspective foreshortening from a changing view angle and
  in-plane rotation of the base are not compensated, so large heading changes or steep view-angle changes during the
  approach lower the score (frames below `min_score` publish nothing).
- The target must lie on a flat surface of known height (`surface_z`) and carry visible texture (a featureless patch is
  not seeded); repeated textures can produce a confident wrong match, which `max_correction_m` bounds.

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
`grasp_object` tool then takes a fresh gripper frame after the lift and retreat, crops it to the jaws and attaches it
(see Grasp verification): every held close, a verified contact as well as an empty-band hold, is reported `unverified`
until the agent calls `confirm_grasp`; a camera failure adds a warning and `grasp_check.image` `not_attached` instead.
The executor itself (queued grasp steps, the web UI's GraspService) keeps the band-only outcome (`grasped` for a verified
contact, `unverified` for the empty-band hold) with `grasp_check.image` `not_requested` and no image, and the reason
text of its `unverified` says "call get_camera_image 'gripper' and look at it". An unverified hold counts as carrying (`get_control_state` `arm.carrying`), and its profile torque limit is
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
and invalid requests are answered at once; `plan`, `execute`, `release` and `jog` run in a worker thread (one
execute/release at a time; jogs coalesce, see "Jog" below).

Request:

```json
{"action": "plan" | "execute" | "release" | "stop" | "jog",
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
               "edge": {"point": [0.16, 0.0], "direction": [0.0, -1.0]}}],
 "jog": {"kind": "cartesian", "direction": "up", "step": 0.01}}
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
object or jog spec, unknown params, another action running, battery cut-off for execute/release/jog).

**Jog** (`jog.py`, the Arm tab jog panel). `{"action": "jog", "request_id": "...", "jog": {"kind", "direction",
"step"?}}` moves the arm one small step; `jog` is required and rejects unknown keys:

| kind | directions | step (default and cap) | how |
|---|---|---|---|
| `cartesian` | `up`, `down`, `forward`, `back`, `left`, `right` | m, `jog.max_cartesian_step_m` 0.01 | tool point along the gripper frame, approach pitch kept: `solve_cartesian` then `move_cartesian` |
| `shoulder_pan` | `left`, `right` | rad, `jog.max_joint_step_rad` 0.05 | `move_joints`; left turns the gripper heading counter-clockwise (seen from above) |
| `wrist_pitch` | `up`, `down` | rad, `jog.max_joint_step_rad` 0.05 | `move_joints` on `wrist_flex`; up tilts the gripper up (lowers the approach pitch) |
| `wrist_roll` | `left`, `right` | rad, `jog.max_joint_step_rad` 0.05 | `move_joints`; left is + roll |
| `gripper` | `open`, `close` | open fraction, `jog.max_gripper_step` 0.1 | `set_gripper(open_fraction)` |

The gripper frame: `forward`/`back` along the approach axis (the way the gripper points), `left`/`right` horizontal and
perpendicular to the approach heading (roll ignored), `up`/`down` perpendicular to both (world up when the gripper points
horizontally, away from the robot when it points straight down). A step starts from the commanded pose while the lease
holds one, else from the measured pose (`ArmState.tool_pose`), so sag is never locked in. A larger `step` is clamped to
the cap; an omitted one uses the cap. The joint signs come from the kinematics, never from URDF axis conventions.

A jog is refused with `{"ok": false, "error": "jog refused: <reason>"}` (nothing moved, no lease taken) under the
battery cut-off, without fresh `/follower/joint_states`, while the agent holds the arm (`take_control`), while
mcp_server holds no lease and filter_node's fresh active source is another one than `autonomy`/`none` (e.g. `leader`:
take control first, `/arm/take_control`), while a grasp execute/release runs, while another (non-jog) arm motion runs, when the IK target is unreachable, when the step would hit the robot body (self-collision, checked
before the lease is taken), when a cartesian step would end below `arm.floor_z_m + jog.floor_clearance_m` (a step that
rises is never refused for the floor), when the joint is at its limit, when the gripper is already fully open/closed,
and for a `wrist_roll` step while the gripper is open past `limits.roll_max_gripper_open_rad` (the roll guard; a jog step
is below `roll_guard_min_change_rad`, so the jog checks the opening itself). The floor slow zone applies to the motion.

A done step answers `{"ok": true, ..., "result": {"kind", "direction", "step", "step_capped", "status", "message",
"tool_pose": {"x", "y", "z", "pitch"}, "motion": <ArmMotionResult>}}` (`status` is the motion status, e.g.
`converged`).

Coalescing: while a jog runs, a newer jog becomes the single pending one and replaces any earlier pending jog, which is
answered at once with `{"ok": false, "error": "superseded by a newer jog ..."}`; the pending jog runs when the current
one ends (the last request wins, never an unbounded queue). `stop` drops the pending jog (answered `dropped by stop
before it ran`). Every request gets exactly one answer on `/grasp/result`.

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
  contact_baseline_samples: 10   # load/current baseline = mean of this many samples (>= 3), arXiv 2608.23983 N_warm
  contact_persist_samples: 3     # consecutive samples at or above contact_load_delta (arXiv 2608.23983 N_persist)
  current_contact_delta_a: null  # gripper current rise (A) voting 'object' in the band; null = recorded, no vote
  presets:
    gentle: {squeeze_rad: 0.01, torque_limit: 250, close_speed_rps: 0.2, target_load: 80, contact_effort_threshold: 120, crush_load: 200, retighten: false}   # optional per profile: contact_load_delta (load units above the baseline, null = delta contact off)
    normal: {squeeze_rad: 0.03, torque_limit: 500, close_speed_rps: 0.5, target_load: 200, contact_effort_threshold: 300, crush_load: 450, retighten: true}
    firm: {squeeze_rad: 0.05, torque_limit: 650, close_speed_rps: 0.4, target_load: 300, contact_effort_threshold: 350, crush_load: 600, retighten: true}
    soft: {squeeze_rad: 0.08, torque_limit: 600, close_speed_rps: 0.3, target_load: 150, contact_effort_threshold: 250, crush_load: 550, retighten: true}
```

## Grasp verification (load-delta contact, current, jaw crop, confirm_grasp)

A close is judged by two independent witnesses: the sensors (load rise above a measured baseline, gripper current, jaw
gap) give a **band**, the agent looks at a jaw crop and gives a **verdict**, and `fuse` combines them. mcp_server has no
LLM access, so the agent is the judge; the image never decides alone (visual judges are weakest on contact-only
outcomes, FailBench arXiv 2609.03611).

**Contact model (`ArmController.set_gripper`, `DeltaContact`).** The main contact signal is a load rise above a baseline,
not a fixed load, after arXiv 2608.23983 (SO-ARM101 / STS3215 gripper at 50 Hz: baseline = mean of 10 samples, contact =
load minus baseline >= 20 load units for 3 samples in a row, detection off during warm-up, free-closing noise +-3-5
units). A delta survives pads or TPU tips shifting the absolute load. Our closes run at 0.2-0.5 rad/s, far faster than
the paper's 5 mm/s, and the jaw needs up to 0.9 s to break away from rest, so:

1. every `close_until_effort` mints a `grasp_id` (uuid4 hex, 12 digits);
2. rest baseline: the mean of the last `grip_profiles.contact_baseline_samples` (10) finite closing-direction gripper loads
   (and currents) before the stream starts (`baseline_source: rest`, the fallback);
3. free-closing baseline: the same number of samples collected after `contact_allowed` first opens (the load ignore
   window, travel or stall), `baseline_source: free_closing`; delta contact is off until it is collected;
4. contact fires when `closing_load_sign * load - baseline >= contact_load_delta` (profile field) for
   `grip_profiles.contact_persist_samples` (3) consecutive samples (`evidence.contact: load_delta`);
5. the absolute `contact_effort_threshold`, `target_load` and the stall check stay as backstops (`contact: absolute /
   target_load / stall`); a profile without `contact_load_delta` behaves exactly as before. A close that starts already
   touching the object has a high baseline, so the delta never fires and a backstop ends it (band `ambiguous`);
6. the paper's bounded extra closure is the existing `squeeze_rad` hold (contact jaw plus `squeeze_rad`, capped by
   `squeeze_max_rad` and the torque limit); `hold_report` also samples the gripper current.

An unmeasured (NaN) load gives no baseline, no delta contact and a band that can never be `object`.

**Current ingestion.** The feetech bridge reads Present_Current in the state sync read and publishes
`/follower/joint_currents` (`sensor_msgs/JointState`, effort = A, same stamp as `joint_states`, only when every listed
joint was read). `topics.follower_joint_currents` names it; `RosRobot.joint_sample` fills `JointSample.currents` from the
latest message received within `timeouts.follower_stale_s` of the joint states, else `{}` (a stale or missing current
never votes; disabling `publish_current_joints` in the bridge is the rollback). `grip_profiles.current_contact_delta_a`
(null by default: the current is recorded but does not vote) is the current rise that counts as an object.

**Evidence and band (`grasp_check.py`).** `ArmMotionResult.grasp_check` and `GraspResult.grasp_check` carry
`{grasp_id, band, evidence, image, crop, fused}`. `evidence` (`GripEvidence`): `baseline_load`, `baseline_source`,
`contact`, `peak_load_delta`, `hold_load_delta`, `rest_current_a`, `hold_current_a`, `current_delta_a`, `jaw_gap_rad`
(measured jaw minus `gripper_closed_rad`), `in_empty_band` (`jaw_gap_rad < limits.gripper_empty_stall_rad`), `band`.
`band_verdict`: `object` = not in the empty band, a contact was detected, the hold load delta reaches
`contact_load_delta` (`hold_effort_min` on the load for a profile without it) and the current does not vote against;
`empty` = in the empty band, the load does not say object and the current does not vote for it; anything else
(including an unmeasured load) is `ambiguous`.

**Fuse table (`fuse`, `confirm_grasp`).**

| band | image yes | image no | image unsure |
|---|---|---|---|
| object | grasped | unverified | unverified |
| ambiguous | unverified | unverified | unverified |
| empty | unverified | missed | unverified |

**Jaw crop (`jaw_crop.py`, `grasp_tools.attach_jaw_crop`).** After the lift and retreat of a held `grasp_object` close
(or right after a `set_gripper` close that grasped) the tool takes a fresh gripper frame (`camera_image` waits for a
frame newer than the end of the motion), projects the jaw points (`jaw_tip`, `tool_point`, `moving_jaw_tip`, with the
measured gripper angle) plus the same points moved `grasp_check.crop_depth_m` (0.045) toward `gripper_link` into the
camera, takes their bounding box plus `crop_margin_px` (24), clamps it to the frame and crops the JPEG
(`crop_jpeg`, longest side `grasp_check.image_px` 640, capped by `limits.max_image_px`). `crop_min_px` (64): a smaller
box, an uncalibrated camera, a jaw region behind the camera or no fresh joint states give the full frame with
`grasp_check.crop.full_frame_reason` (a real image, never a placeholder). The image is stored as a photo POI
(`record_lookup`, tool `grasp_object` / `set_gripper`, id in `crop.lookup_id`; `full_frame=False` for a crop, so no hfov) and `crop.box_px` is the box in the frame.
`grasp_check.image` is then `pending`; with no frame (camera failure) it is `not_attached`, the result has a warning and
the outcome is still `unverified` (use `get_camera_image` and then `confirm_grasp`). `mark_visual_pending` sets the carry
record to an unverified hold awaiting `confirm_grasp` (and starts the `unverified_hold_relax_s` timer).

**confirm_grasp.** `ArmController.record_visual_verdict` applies `fuse` to the stored band and the verdict: `grasped` sets
`grasp_kind: grasped` and stops the relax timer (when it already fired the reply says the default torque was restored and
suggests re-gripping), `unverified` keeps `grasp_kind: unverified`, `missed` clears the carry record (the jaws stay
closed: the reply says to call `release_object`). The reply is `{grasp_id, band, image, fused, evidence (null after an
mcp_server restart: the carry file keeps only the band), carrying, next}`; `mcp_grasp_verdicts_total{band,image,fused}`
counts it and the evidence text is logged. `control.carrying_state` reasons say "awaiting confirm_grasp" while pending
and name the fused state afterwards.

**Carry record (`carry.yaml`)** gains optional `grasp_id`, `band`, `image` and `fused` (None defaults: an old file
without them still loads), so a pending confirm still works after an mcp_server restart.

**Tuning notes.** `contact_load_delta` (20 from the paper, per preset in `client.yml`) is retuned on the robot: take the
free-closing noise sigma of the empty close (5 closes per profile) and set it to about 3-4 sigma; set
`current_contact_delta_a` from the same runs. **Pads or TPU tips** change the contact geometry, not the delta, but the
position test `limits.gripper_empty_stall_rad` (0.08 rad, about 6 mm of jaw gap) decides `in_empty_band`: thicker pads
stop the empty jaw earlier, so retune it (measure the empty close's jaw gap at the hold: `evidence.jaw_gap_rad`) or every
empty close reads as `ambiguous` and every object as in the empty band. Rollout order on the robot: bridge current read
(95 Hz gate), then gripper-only closes (empty, rigid block, plush) checking `band` and the crop, then a full
`grasp_object`.

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
running step ended by cancel/stop or a watch), `watch_triggered`, `paused`, `resumed` (see Watch constraints) and
`queue_empty`. The slow zone is not a failure (reported in the data). Events
carry `seq`, wall time, job id, kind and label, are logged on `mcp_server.motion_queue`, and the last
`motion_queue.event_history` (200) are kept.

**Waiting.** `wait_for_event(timeout_s, until, since_seq)` blocks only until something relevant happens: `queue_empty`
(default: drained, or a failure / precondition failure / contact / aborted step / watch trip; a failure the queue skips past with
`on_fail: skip` does not end it), `step_done` (also any finished or
skipped step), `failure` (failures only) or `any`. It returns at once with reason `idle` when nothing is queued,
returns the events not returned before (`since_seq` overrides the cursor) plus the queue status and a state digest
(arm joints, gripper position / open fraction / effort, base pose, battery V, lease; unknown values null), so a separate
`get_robot_state` is rarely needed. Timeout default `wait_default_s` (30 s), capped at `wait_max_s` (120 s).
`get_motion_status` gives the same status without waiting.

**Single motion owner.** While the queue is busy (a step runs, steps are pending or a watch paused it) the worker owns
the robot's motion:
`RobotMCPServer.call_tool` refuses every tool in `QUEUE_EXCLUSIVE_TOOLS` (`MOTION_TOOLS` except `enqueue_motions`, `resume_motions` and
`arm_set_home`) with "the motion queue is running". Conversely `enqueue_motions` is refused while one of those blocking
tools runs, or while another arm motion holds the arm (a web-UI grasp or `/arm/home`). Should a blocking motion still
start in between, the arm motion lock and the base motion lock make the later one fail (`step_failed`, status
`refused`) instead of moving concurrently. `stop` calls `halt_for_stop` first (pending steps dropped at once, event
`stopped`) and then the normal stop, which ends the running step; `cancel_motions` drops the pending steps and stops the
robot when a motion step runs. The blocking tools stay available for simple single moves when nothing is queued.
When the process shuts down with a busy queue (`__main__` calls `tools.stop_queued_motion`), the queue is cleared and the
robot stopped first, so no queued Nav2 goal outlives the server.

**Watch constraints.** `MotionQueue.enqueue(steps, replace, watch_constraints)` takes up to
`motion_queue.watch_max_per_call` (8) `WatchConstraint`s that are watched *while the steps of that call run*:

| field | meaning |
|---|---|
| `name` | optional label (<= 40 chars); default `<metric>[<joint>] <op> <value>` |
| `metric` | `gripper_effort`, `gripper_position`, `joint_position`, `joint_effort`, `tilt_deg`, `roll_deg`, `pitch_deg`, `base_speed_mps`, `distance_travelled_m`, `battery_v`, `tool_height_m` |
| `joint` | arm joint, required for (and only allowed with) `joint_position` / `joint_effort` |
| `op`, `value` | `lt` / `le` / `gt` / `ge` against a finite threshold |
| `for_s` | 0..5 s (default 0): the violation must hold continuously this long (debounce) |
| `action` | `abort` (default) or `pause` |
| `on_unknown` | `ignore` (default: an unknown value neither trips nor resets the debounce; NaN efforts are routine) or `trigger` (unknown counts as a violation) |

Values come from `guard_state.WatchSampler` over the caches mcp_server already keeps (no new subscription): efforts
are absolute values, `tool_height_m` is FK of the measured pose, and `distance_travelled_m` is integrated by the watcher
from fresh `base_speed_mps` since the watch became active (a stale-odometry gap is not integrated and reads unknown).
A watch set binds to the job ids of its enqueue call and is active while the running group contains one of them (from
dispatch through the step end); appended calls have their own sets, and sets none of whose steps is pending or running
any more are dropped (e.g. by `replace=true`). The `motion-watch` daemon thread (started with the worker) sleeps on the
queue condition while no set is active and otherwise ticks at `motion_queue.watch_rate_hz` (10 Hz, at most 20). It
never runs inside the 25 Hz arm setpoint stream (`on_tick`); the overhead test runs a fake 25 Hz stream beside the
watcher at 20 Hz and bounds what the watcher can add to its tick period (the GIL hold of one tick, measured as the
tick's thread CPU time) at p95 under 1 ms.

A trip is decided under the queue lock and generation-checked: when the step ended (or was cancelled) while the caches
were sampled, the late trip is dropped. It emits `watch_triggered` {watch, metric, value, op, threshold, action}, counts
`mcp_motion_watch_triggers_total{metric,action}` and reports a warning RobotEvent `watch_triggered` (digest,
`/robot_events`; a warning, so it interrupts neither motions nor claude_agent). Then:

- `abort`: the cancel path - pending steps dropped (`cancelled`), the running step ends as `step_aborted`, `robot.stop`
  outside the lock (once) and the stop baseline reset.
- `pause`: the running group is stopped (`step_aborted`, not re-run), the pending steps are kept, `paused` is emitted
  and the worker does not dispatch. `get_motion_status` shows `paused: {by, at}`; steps enqueued while paused are
  appended and wait (`replace=true` drops pending steps, the pause stays). `MotionQueue.resume()` emits `resumed` and
  re-arms the watches of the pending steps; it is refused (`QueueError`, the queue stays paused) in battery cut-off or
  while a watch of the pending steps still violates, and is a no-op message when not paused. `stop` and
  `cancel_motions` clear the pause together with the queue.

`watch_triggered` and `paused` are failure events, so `wait_for_event` (`queue_empty`, `step_done`, `failure`) wakes
with reason `watch_triggered`. `get_motion_status` lists `watches: [{name, metric, joint, op, value, last_value,
job_ids, active, triggered}]`. Tool surface: `enqueue_motions(..., watch_constraints=[{metric, op, value, for_s, action, joint, name, on_unknown}])` (at most `watch_max_per_call`; the description lists metrics, ops, `for_s`, abort vs pause and the scope = the steps of that call) and `resume_motions` (battery gate, returns `ResumeResult`; refusals are tool errors). Without a sampler (`MotionQueue(sampler=None)`) watch constraints are refused.

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

## Guard rules

Declarative safety rules checked outside the model (`guard_rules.py`, pure Python, no ROS). They live in the
`guard_rules:` section of `config.yaml` and are validated by pydantic in `load_config`, so a bad rule stops the node
with `EX_CONFIG` (78) instead of starting half-configured. The code default is enabled with no rules.

### Schema

```yaml
guard_rules:
  enabled: true                       # master switch; false = the engine is a no-op (counters untouched)
  rules:
    - name: base_motion_arm_outside_footprint   # ^[a-z][a-z0-9_]{2,63}$, unique
      mode: dry_run                   # enforce | dry_run (default) | off
      message: "..."                  # 1..300 chars, shown to the model verbatim
      applies_to:
        tools: [navigate_to_pose, move_relative, drive, look_around]   # subset of GUARDABLE_TOOLS
        steps: [base_relative, navigate_to_pose]                       # subset of STEP_KINDS
        arm_motion_from: []           # tool names / step kinds whose arm motions are target-checked (ARM_ORIGINS)
      all:                            # every condition must hold (optional)
        - {field: footprint.kind, op: eq, value: static}
        - {field: arm.outside_footprint_m, op: gt, value: 0.03}
      any: []                         # at least one must hold (optional); a rule needs all or any
      when_unknown: reject            # reject (default) | allow: verdict when the result is unknown
```

Load-time checks (each a `ValueError`): the field must be in the registry below; the op must fit the field type
(`lt`/`le`/`gt`/`ge` with a finite number on float fields; `eq`/`ne`/`in`/`not_in` with a matching value on str and bool
fields, enum values checked; a non-empty list for `in`/`not_in`, a single value otherwise; `args.*` takes any op);
every field must be readable in every phase the rule runs in (`args.*` only with `tools`/`steps`; `target.*`,
`start.*`, `motion.*` only with `arm_motion_from`); `tools` only from `GUARDABLE_TOOLS` (the motion tools without
`arm_set_home`, plus `take_control`, minus `NEVER_GATED`); `NEVER_GATED` (`stop`, `cancel_motions`, `release_control`,
`arm_home`, `resume_motions`) and every read-only tool can never be named, so recovery and inspection stay open;
`steps` only from `STEP_KINDS` (the motion queue step kinds); `arm_motion_from` only from `ARM_ORIGINS`
(`move_arm_joints`, `move_arm_cartesian`, `set_gripper`, `grasp_object`, `release_object`, `arm_joints`,
`arm_cartesian`, `gripper`, `grasp`); a rule must apply to something and have `all` or `any`; names are unique.

### Fields

Closed registry (`guard_rules.FIELDS`, plus the patterns `arm.joint.<j>`, `target.joint.<j>`, `motion.abs_delta.<j>`
with `<j>` in `ARM_JOINTS`, and `args.<name>`). Unknown is `None`, never a substitute value.

| Field | Type | Phases | Source (cached, no new subscription; unknown = `None`) |
|---|---|---|---|
| `tool.name` / `step.kind` | str | tool / step | the call / step (engine) |
| `args.<name>` | any | tool, step | raw tool arguments / step fields (engine) |
| `arm.lease_held` | bool | all | `ControlManager.arm_held` |
| `arm.active_source` | str (`leader`, `web_ui`, `autonomy`, `none`) | all | `arm.backend.active_source()`; unknown when older than `timeouts.state_stale_s` |
| `arm.carrying` | bool | all | `ControlManager.carrying_state().carrying` |
| `arm.at_home` | bool | all | `ArmController.at_home()`; unknown without a stored home or with stale joint states |
| `arm.joint.<j>` | float | all | measured position of a fresh joint sample; stale or NaN is unknown |
| `arm.tool_height_m` | float | all | `ik.forward(measured).z` (base_link z = height above the floor); stale joint states unknown |
| `arm.outside_footprint_m` | float | all | `NavFootprint.arm_outside_static_m(part_points(measured))`: largest distance of the arm shadow outside the static outline, 0 inside; unknown without `nav_footprint` or with stale joint states |
| `footprint.kind` | str (`static`, `dynamic`, `conservative`) | all | `NavFootprint.current.kind`; unknown without `nav_footprint` |
| `footprint.carried_radius_m` | float | all | `NavFootprint.current.carried_radius_m` or 0; unknown without `nav_footprint` |
| `base.lease_held` | bool | all | `ControlManager.base_held` |
| `base.speed_mps` | float | all | `RobotMonitor.odom_speed()` (filtered odometry twist, fresh within `monitor.odom_fresh_s`) |
| `tilt.level` | str (`ok`, `warning`, `error`) | all | `RobotMonitor.tilt_snapshot().level`; a stale state keeps its last level (as `tilt_refusal`), unknown before the first state |
| `tilt.deg` | float | all | `RobotMonitor.tilt_snapshot().tilt_deg`, fresh within `floor_guard.imu_max_age_s` |
| `battery.voltage_v`, `battery.cutoff` | float, bool | all | `BatteryGuard.state()`; stale or no guard is unknown |
| `events.human_takeover_age_s` | float | all | `RobotMonitor.last_event_age_s("human_takeover")`; `+inf` when no `human_takeover` is in the retained history |
| `queue.busy` | bool | all | `MotionQueue.busy()`; unknown when no queue is bound |
| `start.tool_height_m`, `target.tool_height_m` | float | arm | forward kinematics of the motion start (command base) / clamped target (joints the motion does not name keep the start value) |
| `target.joint.<j>`, `motion.abs_delta.<j>` | float | arm | clamped target, abs(target - start) (engine) |

`guard_state.py` implements these (`GuardState`, a `FieldSource`) and also `WatchSampler.sample(metric, joint)` for the
motion watches: `gripper_effort` and `joint_effort` (absolute, finite only), `gripper_position`, `joint_position`,
`tilt_deg`, `roll_deg`, `pitch_deg` (IMU fresh within `floor_guard.imu_max_age_s`), `base_speed_mps`, `battery_v`,
`tool_height_m`; `None` for anything stale, NaN or not configured, never a substitute value.

The engine resolves `tool.name`, `step.kind`, `args.*`, `target.joint.*` and `motion.abs_delta.*` (abs(target - start))
itself; every other field comes from a `FieldSource` (`get(field, ctx)`, ctx = phase, key, args, start, target).

### Semantics

- Kleene three-valued logic: a condition is True, False or Unknown (field None or NaN, or a value of a type the op
  cannot compare). `all`: False if any False, else Unknown if any Unknown, else True. `any`: True if any True, else
  Unknown if any Unknown, else False. Both present blocks are combined with AND. True = hit (basis `match`), False =
  pass, Unknown = `when_unknown` (basis `unknown`). A known False passes even when other fields are unknown.
- Lazy and memoized: only rules applicable to (phase, tool / step kind / arm origin) are evaluated, each field is read
  at most once per check, and a rule stops reading at its deciding condition (the first False in `all`, the first True
  in `any`). A tool without an applicable rule costs one dict lookup (`has_rules`).
- Every applicable rule is evaluated. Enforce hits raise one `GuardRejection` naming all of them; dry_run hits log
  `guard_rule_would_refuse {json}` at WARNING (rule, phase, target, action, basis, message, values) and are counted,
  never raised; `off` rules are skipped (`explain_tool` lists them as `skip`); `enabled: false` disables everything.
- Phases: `check_tool(tool, args)` at tool entry, `check_step(kind, step_fields)` at queue dispatch,
  `check_arm(start, target)` at arm motion start for the origin in the `GUARD_ORIGIN` context variable (None = not an
  agent motion: web UI, park, stop hold, `arm_home` are never checked). `explain_tool(tool, args)` returns the
  hypothetical tool-phase verdicts (`Verdict`: rule, mode, outcome pass/reject/skip, basis, message, values) without
  counters or logs.

### Hooks and ordering

`tools.register_tools` builds ONE `GuardEngine` (`config.guard_rules` over `guard_state.GuardState`) shared by three
hooks (`RobotMCPServer.guard_engine` holds it):

1. **Tool entry** (`RobotMCPServer.call_tool_with_digest`): lease refusal -> queue-exclusive refusal -> **guard tool
   phase** -> blocking-motion counter -> tool body (battery and tilt gates inside). Only when
   `has_rules("tool", name)`: `check_tool(name, arguments)` runs in the threadpool (the field providers take the
   footprint / monitor locks, never on the event loop). A `GuardRejection` becomes a `ToolError` with the rejection text
   (the digest is appended by the usual error path) and nothing reaches the robot. The tool body runs with
   `GUARD_ORIGIN` = the tool name (next to the lease scope; it propagates into the threadpool like the expected lease).
2. **Queue dispatch** (`MotionQueue.dispatch`, `guard_gate`): stop -> battery -> lease -> tilt (base steps) -> **guard
   step phase** (`check_step(kind, step fields)` for every step of the blend group) -> precondition -> run. A rejection
   fails the step with `status: guard_rule`, `data.rules` = the refusing rule names, and ALWAYS stops the queue
   (`queue_stopped`), also with `on_fail='skip'`: the later steps were planned assuming this one ran. The group runs
   with `GUARD_ORIGIN` = the head step's kind.
3. **Arm motion start** (`ArmController.check_motion_guards`, all of `stream_to`, `move_path`, `move_blend`): the
   wrist roll guard first, then `motion_guard(start, full target pose)` -> `check_arm` for every leg (every blend
   target, every path sample) once, before the first setpoint and never inside the 25 Hz streaming loop. A rejection
   is an `ArmError` carrying the rejection text, so tools report it like any refused arm motion (the MCP framework
   prefixes "Error executing tool ...") and the queue fails the step with `status: refused`. Only origins named in a
   rule's `arm_motion_from` are checked: `arm_home`, the `release_control` park, `/arm/home`, web UI grasps and stop
   holds (no `GUARD_ORIGIN`, or an origin no rule may name) never are. This is where the IK-solved targets of
   `move_arm_cartesian` and grasp sub-motions become visible.

### Rejection text

Stable, parsed by claude_agent (`guard_rules.GUARD_REFUSAL_PATTERN`, one match per refused rule):

```
guard rule '<name>' refused <tool | queued <kind> step | arm motion of <origin>>: <message> (<field>=<value>, ...)
```

The values are the conditions' observed values (floats rounded to 3 decimals, `true`/`false`, `inf`, `unknown` for
None); several enforce hits are joined by `; `.

### Metrics

`mcp_guard_rule_hits_total{rule,mode,phase,basis}` (mode `enforce` = refused, `dry_run` = would-be refusal; phase
`tool`/`step`/`arm`; basis `match`/`unknown`), `mcp_guard_eval_seconds{phase}` (buckets 10 us .. 50 ms, observed for
checks that had applicable rules) and `mcp_motion_watch_triggers_total{metric,action}` (motion queue watches). Label
cardinality is bounded by the configured rule names. Latency in the unit tests (dev Mac): `check_tool` p99 < 20 us on
an unruled tool and < 500 us with five rules over fake providers.

### Deployed rule set

`ansible/group_vars/client.yml` deploys five rules, all `mode: dry_run` (`test_guard_rules_deployed.py` validates them
with `GuardRuleSettings`): `base_motion_arm_outside_footprint` (static footprint and the arm more than 0.03 m outside
it), `base_motion_carrying_without_object_footprint` (carrying with no carried-object radius), `no_arm_retake_from_human`
(take_control arm while the web UI holds the arm or a human took it under 30 s ago), `wrist_roll_needs_lifted_arm`
(wrist roll over 0.1 rad with the tool under 0.12 m) and `no_stretched_shoulder_overreach` (shoulder_lift over 1.85 rad
with a negative elbow_flex). Each message names its way out. `motion_queue.watch_rate_hz: 10` and `watch_max_per_call: 8`
sit beside them. Flip a rule to `enforce` only after about a week of `mcp_guard_rule_hits_total` counters: see
`ansible/README.md` "mcp_server guard rules".

## Safety model

- **Guard rules**: configured `guard_rules` (see Guard rules) are checked at tool entry (after the lease and
  queue-exclusive refusals, before the tool body), at queue dispatch (after the tilt gate, before the precondition;
  a refusal always stops the queue) and once per arm motion leg before its first setpoint (after the wrist roll guard).
  Recovery (`stop`, `cancel_motions`, `release_control`, `arm_home`) and read-only tools are never gated.
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
| `guard_state.py` | no | guard rule field source over cached robot state (`GuardState`) and the watch metric sampler (`WatchSampler`) |
| `guard_rules.py` | no | guard rules: schema (`GuardRuleSettings`), field registry, Kleene evaluation (`GuardEngine`), rejection text and `GUARD_REFUSAL_PATTERN` |
| `trajectory.py` | no | quintic interpolation, limit clamping, tracking error, `blend_trajectory` (velocity-continuous spline through via points) |
| `ik.py` | no | URDF limits, ikpy FK/IK with verification |
| `arm.py` | no | `ArmController`: lease, streaming, aborts, gripper, home, `move_path`, `move_blend`, `solve_cartesian`, slow-zone time scaling |
| `sag.py` | no | gravity sag model: URDF gravity torques, deflection k * tau per approach mode, limit-band compensation, gain fit, leave-one-out validation, `arm_settle` record parsing |
| `floor_guard.py` | no | arm mount conversions, effective surface (robot plane, IMU level plane, surface regions, overrides), jaw model, per-sample slow zone, `retime` |
| `surfaces.py` | no | surface regions (half-plane / convex polygon), local height, step faces, point / capsule clearance |
| `grasp.py` | no | grasp planner: strategies registry, waypoints, straight-line IK samples, feasibility reasons, surface / step-edge checks |
| `grasp_tools.py` | no | `GraspExecutor`, the grasp MCP tools, `GraspService` (web-UI JSON) |
| `grip.py` | no | grip profiles: resolve a preset name / inline overrides to a capped `GripProfile` |
| `grasp_check.py` | no | grasp verification models (`GripEvidence`, `CropInfo`, `GraspCheck`), `band_verdict`, `fuse` |
| `jaw_crop.py` | no | jaw crop: project the jaw region into the gripper camera, `crop_jpeg` |
| `base_motion.py` | no | timed clamped drive loop, continuous spin with marks (`run_spin`), stop sequence (`run_stop`) and its base half (`run_base_stop`) |
| `base_dock.py` | no | `DockGoal` / `DockResult` / `DockPort` and the blocking `run_dock` loop over `nav2_msgs/action/DockRobot` (mirrors `run_nav`: server down or no answer raises `RobotError`; rejected, critical event = cancel + zero twist, stop, timeout, result and error-code mapping 0/901..906/999). The loop publishes nothing but a zero twist after a critical event; the docking server drives. The ROS adapter is `ros_dock.py`. |
| `control.py` | no | control leases: `ControlManager` (take / release with the arm home and base steering resets / carried-object park refusal / footprint triggers / state / lease refusals / vitals digest), `wait_steering_neutral` |
| `footprint.py`, `footprint_geometry.py` | no | navigation footprint: `NavFootprint` (static / dynamic / conservative, arm shadow, carried circle, republish tick following the released arm, consumers and their outcomes via `FootprintSink`), `python -m mcp_server.footprint` (parked-arm shadow for the generator); shared outline / parked-arm union / circle / offset / inflation geometry (also imported by `scripts/gen_nav_footprint.py`) |
| `motion_queue.py` | no | motion queue: step models, blend groups, preconditions, worker thread, watch constraints (watcher thread, abort / pause / resume), events, waits |
| `motion_tools.py` | no | `enqueue_motions`, `resume_motions`, `get_motion_status`, `cancel_motions`, `wait_for_event` (one `TOOL_MODULES` entry) |
| `guard_tools.py` | no | `check_guard_rules` (read-only `GuardEngine.explain_tool`; one `TOOL_MODULES` entry) |
| `perception.py` | no | scan sectors/points, map stats/PNG, image encoding |
| `topdown.py`, `perception_models.py` | no | robot-up top-down renderer (layers, transforms); perception result models |
| `object_memory.py`, `look_around.py`, `poi_client.py` | no | object POI memory + merge, look_around plan/loop/montage, POI request matching |
| `jog.py` | no | web-UI arm jog: `JogSpec`, `Jogger` (gripper-frame cartesian steps via IK, joint and gripper steps, step caps, refusals, last-request-wins coalescing), `gripper_axes` |
| `auto_poi.py` | no | `AutoPoiMarker` (stuck events to point POIs, merge by radius), `attach_auto_poi` (monitor sink + `auto_poi_id` annotation) |
| `frontiers.py` | no | frontier core: `MapGrid`, vectorised WFD `extract_frontiers`, `traversability`, `geodesic_from`, `observation_pose`, `FrontierTracker` (stable ids, epochs, retired successors), `facing_lookup`, `nearest_poi`, cell/map conversion |
| `frontier_tools.py` | no | `list_frontiers`, `plan_frontier_visit`, `get_exploration_status`, `mark_survey_spot`, `go_to_frontier`, `look_toward_frontier` (one `TOOL_MODULES` entry) |
| `perception_tools.py` | no | the perception, memory, look_around and POI tools (one `TOOL_MODULES` entry) |
| `home_store.py`, `staleness.py`, `geometry.py`, `models.py` | no | home YAML, data age, pose math, result models |
| `monitor.py` | no | `RobotMonitor`: vitals, events, digest, `MotionWatch` |
| `tool_context.py`, `body_tools.py` | no | `ToolContext` / `RobotApi`; the `get_body_state` module |
| `camera_tools.py` | no | the camera tool module (registered in `tools.TOOL_MODULES`) |
| `approach_tools.py` | no | the `suggest_approach_pose` / `approach_for_grasp` module (reach map load at registration, mode selection, dry run, `reach_after`); planner in `approach.py`, dock loop in `base_dock.py`, tracker in `target_tracker.py` |
| `lookup_store.py`, `lookup_tools.py` | no | photo store facade: read-only `LookupStore` over poi_store's `index.jsonl` (`records`, `get`, `query`, `image`) plus `write_image` / `remove_image` of the JPEG, and `photo_summary` (record metadata with the view derived on read for legacy lines); models shared via `ros2_common.lookups` / `ros2_common.photos`. The recorder hook of the camera tools (JPEG write, then a poi_store photo `add`) and the `list_lookups` / `get_lookup_image` / `annotate_lookup` / `delete_photos` module (see [Photos (photo POIs)](#photos-photo-pois)) |
| `camera_scene.py`, `camera_overlay.py`, `camera_candidates.py`, `camera_calib.py` | no | calibrated scene and frames, overlay drawing and candidate grids, candidate sets, sample store and solver wiring |
| `tools.py` | no | MCP tools, `StaticTokenVerifier`, Streamable HTTP app |
| `ros_iface.py` | yes | `RosRobot`: subscriptions, TF, Nav2 action, publishers, services; the footprint sink (costmap / ArmStop polygon publishers, `published_footprint` subscriptions, `set_parameters` clients with outcome reports and timeouts, the republish timer, the consumer readiness poll, the arm pose) |
| `spin.py` | no | `spin_forever` (executor loop that survives rclpy stale-handle errors), `run_or_exit` (exits the process when the executor thread dies while ROS is up, so systemd restarts the node instead of serving stale data; an error after ROS was shut down is the normal end), `FrameCache` (latest camera frame) |
| `metrics.py` | no | Prometheus `mcp_*` metric objects, the per-feed sample age collector |
| `serve.py` | no | uvicorn config; `timeout_graceful_shutdown` 3 s so the claude_agent's open MCP stream cannot stall `systemctl stop` until the 30 s SIGKILL |
| `__main__.py` | yes | entry point: executor thread + uvicorn |
| `reference/<family>.py` | no | Python references of the kernels ported to `mcp_core_cpp` (`KERNELS` registry per family: reference, bench input function, tolerances) |

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
| `mcp_grasp_verdicts_total{band,image,fused}` | `confirm_grasp`: the sensor band, the agent's image verdict and the fused outcome (`grasped`, `unverified`, `missed`) |
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
| `mcp_lookup_writes_total{tool}`, `mcp_lookup_write_failures_total` | `LookupRecorder.record` (`lookup_tools.py`): camera images recorded as photo POIs, and the ones it failed to record (JPEG write failed, poi_store down, refused or timed out) (the camera tool still answered) |
| `mcp_guard_rule_hits_total{rule,mode,phase,basis}` | `GuardEngine.run` (`guard_rules.py`): a rule hit at a tool / step / arm check; mode `enforce` (refused) or `dry_run` (would-be refusal), basis `match` or `unknown` (resolved to reject by `when_unknown`) |
| `mcp_guard_eval_seconds{phase}` | `GuardEngine.run`: evaluation time of a check that had applicable rules (buckets 10 us .. 50 ms); unruled tools are not observed |
| `mcp_motion_watch_triggers_total{metric,action}` | motion queue watch constraints that triggered, by metric and action (`abort`/`pause`) |
| `mcp_frontier_compute_seconds{stage}`, `mcp_frontier_count` | `frontier_tools.refresh` / `geodesic_for`: per-stage frontier compute time (`extract`, `traversability`, `field`, `match` once per `/map` version, `geodesic` once per version and robot cell; a cache hit observes nothing) and the frontier count of the latest version; see [Frontiers](#frontiers) |
| `mcp_nav_goals_total{result}`, `mcp_nav_goal_duration_seconds` | `nav_result` in `base_motion.py`, so every Nav2 goal that was sent (`succeeded`, `canceled`, `timeout`, `interrupted`, `rejected`, ...); errors before a goal exists (Nav2 not available) raise and are not counted |
| `mcp_dock_goals_total{result}`, `mcp_dock_goal_duration_seconds` | `dock_result` in `base_dock.py`, one count per docking goal that was sent (`succeeded`, `failed`, `rejected`, `interrupted`, `canceled`, `timeout`); server unavailable or no answer raise and are not counted |

### Grasp result mapping

`mcp_grasp_attempts_total` counts one attempt per `GraspExecutor.grasp` call that got past grip profile validation.
The `result` label is derived from `GraspResult.outcome` (`metrics.GRASP_RESULT_BY_OUTCOME`):

| `GraspResult.outcome` | `result` | Meaning |
|---|---|---|
| `grasped` | `lifted` | closed on the object, verified, lifted and retreated while holding it |
| `unverified` | `unverified` | a held close the sensors could not settle (jaw closed into the empty band: kept closed with the profile squeeze, lifted and retreated; soft/squashed object or nothing). The `grasp_object` tool turns every held close, `grasped` included, into `unverified` until `confirm_grasp`, but the attempt is counted at the executor, so it stays `lifted` here; the fused result is `mcp_grasp_verdicts_total` |
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
approach:                 # approach pose planner and grasp docking interfaces (see "Approach planner")
  yaw_step_deg: 10
  cells_per_yaw: 30
  min_expected_margin_m: 0.0
  pose_error_xy_m: 0.03
  pose_error_yaw_deg: 3
  margin_cap_m: 0.08
  weights: {margin: 1.0, cost: 0.5, travel: 0.2, heading: 0.1}
  staging_offset_m: 0.30          # = -staging_x_offset of the grasp docking server
  docking_threshold_m: 0.03       # = its docking_threshold
  corridor_samples: 5
  verify_top_k: 5
  path_check_k: 3
  global_costmap_topic: /global_costmap/costmap
  costmap_stale_s: 5
  planner_action: /compute_path_to_pose
  dock_action: /grasp/dock_robot
  detection_topic: /grasp/detected_dock_pose
  dock_timeout_s: 120
  tracker: {rate_hz: 5, min_score: 0.6, template_px: 48, search_px: 96, max_correction_m: 0.10, edge_margin_px: 20}
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
  front: {parent_frame: head_link, intrinsics: {hfov_deg: 66.0, width: 640, height: 480}, mount: null}
grasp_check:               # jaw crop of a held close (see Grasp verification)
  crop_depth_m: 0.045      # how far into the jaws the crop reaches
  crop_margin_px: 24
  crop_min_px: 64          # a smaller crop falls back to the full frame
  image_px: 640            # longest side of the attached image (capped by limits.max_image_px)
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
  tof_points: /tof/points_filtered   # get_map_summary obstacles_3d (lazy, latest only)
  plan: /plan
  poi_list: /poi/list
  poi_command: /poi/command
  poi_result: /poi/result
  swerve_joint_states: /swerve_drive/joint_states   # steer joints for the release_control base reset
  compute_path_action: /compute_path_to_pose        # Nav2 ComputePathToPose, the plan_frontier_visit check
  planner_id: GridBased                             # planner_server plugin id used by compute_path_action
  follower_joint_currents: /follower/joint_currents # gripper Present_Current (A) for the grasp band; stale or absent = no vote
footprint: {length_m: 0.47, width_m: 0.386}
topdown: {default_radius_m: 2.5, default_px: 480, arm_reach_m: 0.41, plan_max_age_s: 30}
objects: {store_path: /var/lib/ros2/objects/objects.json, merge_radius_m: 0.25}  # store_path: legacy file, imported once into poi_store
look_around: {default_captures: 4, clearance_margin_m: 0.10, step_timeout_s: 30, mode: spin, spin_speed_rps: 0.4,
              spin_rate_hz: 20, spin_timeout_margin_s: 10, return_to_start: false}
frontiers:                # frontier tools (see Frontiers); all optional, defaults shown
  min_length_m: 0.30             # smaller clusters are dropped (12 cells at 2.5 cm)
  map_max_age_s: 60.0            # older /map is unavailable
  geodesic_downsample: 2         # geodesic grid = map resolution x this (raise to 3 if over the CPU budget)
  pass_clearance_margin_m: 0.03  # traversable = clearance >= footprint.width_m / 2 + this
  view_clearance_margin_m: 0.05  # observation pose clearance >= circumscribed radius + this
  standoff_min_m: 0.4            # observation pose distance window to the nearest frontier cell
  standoff_max_m: 1.2
  los_candidates: 5              # best-scored poses checked for line of sight
  match_radius_m: 0.6            # centroid distance that keeps an id across map versions
  epoch_reset_known_fraction: 0.5   # known cells below this fraction of the previous map: new epoch, ids restart
  retired_keep: 50               # retired ids remembered (gone, replaced by)
  max_listed: 15                 # list_frontiers cap
  poi_radius_m: 2.0              # nearest_poi search radius
  facing_half_angle_deg: 30.0    # a lookup faces a frontier within this angle of the camera axis
  facing_max_distance_m: 6.0     # ... and within this distance of the lookup position
  compute_path_timeout_s: 5.0    # plan_frontier_visit Nav2 check
  look_timeout_s: 20.0           # look_toward_frontier rotation timeout
  look_camera: front             # default camera (front | gripper)
exploration: {survey_radius_m: 2.5, rays: 72, min_unsurveyed_area_m2: 0.5, downsample: 2,
              stop_margin_cell_v: 0.3, max_unsurveyed_listed: 10}  # see Exploration status
motion_queue: {max_steps: 32, event_history: 200, status_events: 10, wait_events: 40, wait_default_s: 30,
               wait_max_s: 120, poll_s: 0.05, base_still_linear_mps: 0.02, base_still_angular_rps: 0.05,
               holding_min_effort: 100.0, holding_min_gap_rad: 0.05,
               watch_rate_hz: 10.0, watch_max_per_call: 8}  # watcher rate (gt 0, le 20) and watches per call (1..16)
poi: {request_timeout_s: 3.0, near_radius_m: 5.0}
auto_poi:                 # auto stuck points (see Auto stuck points); defaults shown
  enabled: true                  # false: the marker never adds or updates a POI
  merge_radius_m: 0.5            # a repeat within this distance updates the same stuck POI
  max_pose_age_s: 2.0            # an older map pose is stale: no POI
  event_types: [collision_stop, nav_abort]
  hint_after_images: 3           # camera / look_around results since the last POI write before poi_hint appears
jog:                      # web-UI arm jog (action 'jog' on /grasp/command, see Web-UI contract); defaults shown
  max_cartesian_step_m: 0.01     # tool point step cap (gt 0, le 0.05); a larger request is clamped
  max_joint_step_rad: 0.05       # shoulder_pan / wrist_pitch / wrist_roll step cap (gt 0, le 0.3)
  max_gripper_step: 0.1          # gripper open-fraction step cap (gt 0, le 1)
  speed_scale: 0.3               # jog speed, capped to limits.arm_max_speed_scale
  floor_clearance_m: 0.01        # a cartesian step ending below arm.floor_z_m + this is refused
lookups:                  # photo JPEG directory (see Photos (photo POIs)); poi_store owns the index and retention
  store_dir: /var/lib/ros2/poi/lookups
control:                  # control leases: release_control base reset (see Control leases)
  steer_joint_names: [fl_steer, fr_steer, rl_steer, rr_steer]
  steer_neutral_tolerance_rad: 0.05
  base_reset_timeout_s: 6.0   # swerve idle_recenter_s (3.0) + 3 s
  steer_poll_s: 0.1
  steer_states_wait_s: 0.5    # lazy /swerve_drive/joint_states: first-message wait after subscribing
  steer_states_idle_s: 30.0   # ... destroyed once unread this long
on_demand:                # cameras, /scan_filtered and /tf subscribed only while used (see Body awareness)
  linger_s: 30.0            # destroyed after this long unused (checked every quarter of it), must be > 0
joint_state_rate:         # 10 Hz throttled joint states while idle, full rate around motions (see Dual-rate joint states)
  enabled: true             # false: always the full-rate topics
  throttled_suffix: /throttled  # copy topic = <topic><suffix> (joint_state_throttle_* topic_tools nodes)
  linger_s: 5.0             # full rate kept this long after the last motion of the subsystem, must be > 0
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
guard_rules:              # optional; default enabled with no rules (schema in Guard rules)
  enabled: true
  rules: []
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

C++ kernels: `mcp_core_cpp/` is a nanobind extension (scikit-build-core) that `uv sync` compiles into this venv as a
uv path dependency; it needs a C++17 compiler and the Python headers. Layout, how to port a kernel, the golden parity
workflow (`tests/core_parity.py`, `tests/regenerate_core_golden.py`, `tests/data/core_golden/`) and the version-bump
rule for C++ changes are in [mcp_core_cpp/README.md](mcp_core_cpp/README.md).

Frontier tests: `tests/test_frontiers.py` (pure core on synthetic grids: detection, geodesic, observation pose, tracker ids
/ epochs / successors, facing lookup, nearest POI, a 1200x1200 timing guard), `tests/test_ros_iface_frontiers.py`
(`map_grid` version and staleness, `compute_path` outcomes, `nav_active`), `tests/test_frontier_tools.py` (the four
tools with `FakeRobot`), plus the frontier cases in `test_config.py`, `test_footprint.py`, `test_topdown.py`,
`test_perception_tools.py`, `test_tools.py` and `test_metrics.py`.

Deploy: `./scripts/deploy-nodes.sh client mcp_server` (a `--tags mcp_server` run of `ansible/playbooks/deploy_nodes_client.yml`).

## POI zones

`add_poi(kind='area', zone=..., speed_limit_pct=...)` and `update_poi(id, zone=..., speed_limit_pct=...)` write the zone fields of the area POI in `poi_store`; the `costmap_zones` node turns active zones into Nav2 keepout and speed masks.

- `zone='keepout'`: Nav2 does not plan or drive into the area. A robot already inside can leave. Never send a navigation goal inside one.
- `zone='slow'` with `speed_limit_pct` (10..100): the planner prefers routes around the area (non-lethal path cost) and MPPI is capped to that percentage while the robot centre is inside.
- `zone='none'` (default) removes a zone. Switching to `slow` needs `speed_limit_pct` in the same `update_poi` call.
- Active only while `status` is `open`; `done` / `cancelled` switch the zone off and keep the record.
- Zones created by the agent are session-scoped: agent POIs are removed at the next session start.
- Client-side checks mirror `poi_store` and raise `ValueError` before any request: zone on a point, `slow` without a percentage, percentage with a zone other than `slow`.

### POI categories

`add_poi` and `update_poi` take optional `category` (`target`, `slow`, `keepout`, `problem`, `detail`), `problem_type` (`window`, `mirror`, `glass`, `tight`, `overhang`, `low_obstacle`, `drop`, `cable`, `soft_floor`, `other`) and `survey` (bool, default false). They map onto the existing kind/zone/`speed_limit_pct` fields and write a tag on the first line of the note (no new schema field); the rest of the note is free text. Tag format, mapping table and parsers: [shared/README.md](../../shared/README.md) (`ros2_common.poi_categories`).

- Validation (`check_category`, shared) runs on the resulting kind, zone and speed cap before any request; for `update_poi` on the stored POI's current values merged with the change. A broken rule is a tool error naming it: keepout/slow need an area with zone `keepout`/`slow` (and `speed_limit_pct` for slow, passed explicitly: no implicit zone), `problem` needs a `problem_type`, `problem_type` is refused for other categories.
- `update_poi` with a category replaces the category tag and keeps the free text and an existing `#survey` tag. `survey=true` adds `#survey` (with or without a category).
- `remember_object(survey=true)` writes `#target #survey` into the object's note.
- `list_pois` views carry `category`, `problem_type` and `survey` parsed from the note (null / false when untagged).
- Creating a keep-out is free; cancelling or deleting one needs the person's consent.

## Named maps

`save_map`, `list_maps`, `load_map` and `delete_map` talk to the `map_manager` node (`nodes/map_manager`) over three
topics, configured under `maps:` and equal to the map_manager entry in `ansible/group_vars/client.yml`
(`tests/test_mcp_server_config.py` checks it): `command` (`/map_manager/command`, std_msgs/String JSON), `result`
(`/map_manager/result`) and `status` (`/map_manager/status`, latched, the saved maps, active map and localisation).

```yaml
maps:
  command: /map_manager/command
  result: /map_manager/result
  status: /map_manager/status
  save_timeout_s: 30.0     # wait for the save result
  load_timeout_s: 120.0    # wait for the load result (after the relocalisation verdict)
  delete_timeout_s: 10.0
```

- A named map bundles the slam_toolbox graph, the POIs and the last robot pose. `clear_pois` (above) empties the POI set
  alone, by default only the agent's own POIs and without zones.
- None of the map tools moves a servo or drives the base. `load_map` stops the robot first (motion queue halted, base
  zeroed) because the map under it is replaced; the stop is its only motion.
- Consent defaults: `save_map` refuses an existing name unless `overwrite=true`; `delete_map` sends nothing without
  `confirm=true`, which the agent sets only after the person agreed; `clear_pois` clears only creator `agent` unless
  the person asked for more.
- After `load_map`, navigate only when the verdict state is `localised`; otherwise retry with a pose hint or tell the
  person.
