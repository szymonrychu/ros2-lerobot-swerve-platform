# Drivable ground, ToF filtering and head-from-arm calibration design


## Summary

Umbrella epic, user decision 2026-10-22 (verbatim): "Go for all 3 within one epic". It holds:

1. Ground detection that passes live on the user's scene, then Nav2's `ground_voxel_layer` back on (approval G1,
   decisions D21-D25). The scene is recorded through the web_ui recorder, committed as replay fixtures with truth from
   the ChArUco board pose and the user's tape measurements (D22), the estimator is fixed on the replay first with the
   guidance off (D23), the live acceptance moved from ground-detection-robustness is measured through web_ui (D24),
   and only then the ground layer goes on and Nav2 drives a short goal across the scene with the user present (D25).
2. The ToF flying-pixel filter retuned on the recorded raw depth and turned back on (approval G2, D26, D27).
3. The head-from-arm calibration of epic head-and-arm-calibration-fro, moved verbatim (approvals H1-H9 and HQ1-HQ4,
   decisions D31-D51: its D<n> is D<n+30> here), run before the ground recordings so they carry the calibrated head
   chain (D21).

Parts 1 and 2 reuse the RGB and amplitude guidance plan (approvals A1-A8 and Q1-Q4, decisions D1-D19, its own summary
below) instead of duplicating it. Ledger ids G1-G3 are approval items; the design goals G1-G9 and HG1-HG6 below are
separate. The run plan (D29) alternates code runs with live steps that need the user.

RGB and amplitude guidance (D1-D19): ground estimation keeps depth as its only height source. Two image streams that
already exist get used for more: the ToF amplitude image and the head RGB stream registered to the ToF by
head_rgbd_cpp. They decide which depth pixels to trust, where the boundaries of lethal regions are, and where depth is
missing although something is there. Headline decisions: an amplitude gate in arducam_tof (D7); RGB edges through the
existing guided-filter upsampling in head_rgbd_cpp, which is switched on (D2); "edge points" that sharpen lethal
obstacle and drop boundaries in ground_estimator_cpp, only next to regions that depth already found lethal (D3); a
"no_return" caution warning area where the RGB sees non-floor and the ToF returns nothing (D5). Before anything
changes, new recordings with RGB + amplitude are captured through web_ui and give a baseline (D8, D14). Ground truth
comes from the ChArUco board pose (D10). Every step is judged on replay against that truth.

Head from arm (D31-D51): re-planned 2026-10-22. A "Head from arm" guided step on the web UI Calibrate page calibrates
the pan/tilt head chain against the follower arm, which the user trusts as calibrated. The existing 6x5 ChArUco board
is clipped to the fixed jaw; the head_rgbd_calibration node moves the arm through a scripted pose set (wrist roll, then
wrist flex, then elbow and shoulder only after a confirm), every pose and transit FK-checked against the floor, the
robot body and the head, and points the head at the board from a few pan/tilt offsets per arm pose. One robust
least-squares fit solves mount x/y/z, mount roll/pitch/yaw (the pan zero folded into the yaw), tilt zero and tilt roll,
with the applied floor-fit values as start values and cross-check, sigma, held-out views and limits that refuse Apply.
Apply reuses the floor fit's `applied/head_chain.json` path; mcp_server HeadKinematics and the front camera (reloaded
without a restart) and the web UI head URDF read it. The arm joint offset solve moved to epic
arm-joint-offsets-from-the-g.

## Intent and why

User 2026-10-22, the three open items, decided "Go for all 3 within one epic":

1. Ground detection on the live scene. Live after every ground-detection-robustness fix, on the new layout (box, ramp
   and board ahead and to the right of the robot): the ChArUco board about 0.4 m ahead is lethal in 36-45 of 80 frames
   at 2.3 cm; the 5 cm box flips obstacle/warning at pan -30 (its height reads 3.4-3.5 cm); drop warnings appear
   2.5 cm behind the box; the plane reads 1.9-2.2 deg off base_link. The replay fixtures of the old layout pass, so
   they do not show this scene. Nav2's `ground_voxel_layer` is off by default (`nodes/nav2_bringup/config/nav2_params.yaml`,
   commit a04b7df1): its 15 s decay piled the flickering lethal points into dozens of false lethal cells and Nav2
   found no path. The robot cannot drive on the ground layer until this passes live.
2. The flying-pixel filter (`nodes/bridges/arducam_tof/arducam_tof/bridge.py:flying_pixel_filter`, 3x3 jump >
   max(30 mm, 5 % of depth)) removed about 70 % of the live pixels (`tof_flying_pixels_removed_total` about 300k/s at
   10 Hz on 43,200 px), starved the ground fit, and is off in `ansible/group_vars/client.yml` (commit 1e07a728). The
   new `min_range_m` 0.25 cut already drops the arm (commit bd091d28).
3. The head chain still uses CAD yaw and mount x/y: the floor fit cannot observe them (part 3 below).

RGB and amplitude guidance (D1-D19), user request 2026-10-22: "keep depth computation from depth stream, but empower
edges and areas detection with RGB stream, so it's more precise". Today the ground estimator only sees the decimated
ToF cloud (240x180 decimated by 4, 5 cm voxels, 10 cm temporal cells), so:
- lethal edges land about 5 cm off;
- mixed ToF pixels make fake ramps behind objects;
- the printed ChArUco board reads -4..+3 cm (albedo bias);
- the black rubber ramp gives no ToF returns at all and looks like free floor.

The robot navigates on that costmap (2.5 cm resolution), so a 5 cm edge error and an invisible black obstacle are both
real driving risks. The approved order is:
1. a baseline after ground-detection-robustness run 4;
2. amplitude confidence;
3. RGB-guided edges and no-return areas.

Head from arm (D31-D51): the floor-plane fit (epic head-chain-floor-plane-fit-a, applied 2026-10-22 result
r-20261022-065211: mount z 0.246 m, tilt zero -6.50 deg, mount pitch 1.50 deg, mount roll -0.32 deg, tilt roll 0.92
deg; runbook `docs/runbooks/head-floor-fit.md`) fixed the floor for ground estimation, but a plane is invariant to head
yaw, mount x/y and the pan zero, so `head_look_at`, grasping from head images and the 3D head view still use the CAD
values (`head_mount_joint` x 0.15, y 0.10, yaw 0). The user (2026-10-22): "calibrate head position/angles based on the
arm position/angles (it should be well calibrated now)". The arm is the yaw and x/y reference: its base pose
(`arm.base_in_base_link`) and joint offsets (`arm.joint_offsets_rad`, `client.yml` mcp_server) stay as they are.

## Goals

- G1 Baseline: the box, board, ramp and empty scenes are recorded with RGB + amplitude after run 4 is deployed. Their
  replay metrics (guidance off, gate off) are in `nodes/ground_estimator_cpp/README.md` before any guided change is
  judged.
- G2 Amplitude: a range-normalised amplitude gate in arducam_tof. On the recorded frames it brings the board's pixel
  heights to p95 |h| <= 0.02 m and keeps >= 95 % of the floor pixels between 0.3 and 2.0 m.
- G3 Edges: on the gated replay with guidance on, the error of the box front edge in the lethal cloud is <= 0.02 m p95
  and its width error is <= 0.02 m. Both must be strictly better than with guidance off. Lethal points more than 3 cm
  outside the box appear in <= 5 % of frames.
- G4 No-return: on the ramp replay, a `no_return` warning area covers >= 70 % of the ramp footprint in >= 90 % of
  frames after warm-up. A no_return area cell more than 0.10 m (one cell, my call) outside the ramp appears in <= 5 % of frames, in every scene.
- G5 Safety: RGB never creates a lethal point by itself. The board, tile seams and shadows never add lethal points.
  `board_lethal_frames(frames, truth)` stays 0 (run 4's metric name, truth overload from guidance-replay-metrics), and every ground-detection-robustness replay assertion still passes.
- G6 Ops: Pi 5 CPU of head_rgbd_cpp (upsampling), ground_estimator_cpp (guidance) and arducam_tof (gate) is measured
  before/after. A live check through the web_ui websocket is recorded in `docs/runbooks/ground-guidance.md`. READMEs
  and tests/README are up to date.
- G7 Live ground acceptance (approval G1): on the replay of the recorded scene with the guidance off, and then live
  through web_ui captures (2 x 60 s per scene): the box lethal in >= 95 % of states, the board never lethal, no
  lethal point on empty floor at p95, warning churn < 10 % in every scene, the ramp never lethal. The
  ground-robustness runbook's Results hold the live numbers.
- G8 Nav2 (approval G1): `ground_voxel_layer` enabled again after G7 passes, and Nav2 reaches a goal across the scene
  (around the box) and back with the user present, without contact and without lethal cells on the board or the empty
  floor.
- G9 Flying-pixel filter (approval G2): on the recorded raw depth the retuned filter keeps >= 95 % of the floor pixels
  and removes the mixed pixels at the box edges, it is on in client.yml, and live it removes <= 10 % of the pixels
  with the ground plane valid in >= 95 % of states.

Head from arm:

- HG1: A guided "Head from arm" section on the Calibrate page runs base pose (confirm), board clip, tiers, Compute,
  result (applied, fitted, delta, sigma, floor cross-check, held-out numbers, failed limits) and Apply.
- HG2: The fit recovers mount x/y/z, roll/pitch/yaw, tilt zero and tilt roll from synthetic scenes (0.3 px noise) within
  1.5 mm / 0.15 deg and refuses results over the RMS, held-out, sigma, coverage and floor cross-check limits.
- HG3: No arm motion is commanded before its pose and transit pass the FK check (tool, board and arm links clear of the
  floor, the body and the head; roll rules; stretched-arm rule), and every arm motion goes through mcp_server.
- HG4: Apply writes `applied/head_chain.json`; head_state_publisher (TF), mcp_server HeadKinematics and front camera,
  and the web UI head URDF use it; mcp_server without a restart.
- HG5: The head tilt range is widened to the reprinted mount's recorded limit everywhere it is pinned.
- HG6: Ansible in sync, READMEs and tests/README.md, and a robot runbook with real numbers.

## Non-goals

- Learning-based segmentation.
- Heights from RGB (no monocular depth).
- Graded traversability cost (declined by the user 2026-10-22).
- A higher head mapping stream rate (stays 2 Hz, contract head-camera-stream-rate, Q2).
- Refining the polygons of non-lethal warning areas (they stay cell-aligned, D4, Q3).
- Driving beyond the Nav2 check of D25 (two short goals, at most 1.5 m, with the user present); every other live step
  keeps the robot static.
- Loosening any acceptance threshold, or changing the free / warning / lethal tiers or the rule, to pass.
- Recording through rclpy probes, the `ros2` CLI, rosbag2 or SSH on the client.

Head from arm:

- Fitting arm joint offsets, link lengths, the arm base pose, or the tool point (epic arm-joint-offsets-from-the-g
  holds the moved offset solve).
- The camera intrinsics, the ToF-to-camera extrinsic and the head_camera_joint lever (applied RGB-D calibration), and
  the 0.04 m tilt offset (degenerate with mount z).
- Driving the base (the robot is charging; the old drive-straight yaw check is struck).
- The head as an obstacle in mcp_server's own self-collision body model (noted, not doing now).

## Anti-goals

- A lethal mark that only RGB supports, or an obstacle on a texture edge (board squares, tile seams, shadows).
- Lethal output that is removed or delayed by RGB: the lethal latency of ground-detection-robustness
  (`hysteresis.enter_frames` frames) is unchanged.
- Placeholder data on any topic: no guessed height on a no_return area, no guessed pose on a recorded sample, no
  depth invented inside a large hole.
- Warning churn above 10 % because of the new areas.
- An rclpy probe or SSH session on the client to get live data.
- A new node.
- `ground_voxel_layer` turned on before every live acceptance row passes, or a Nav2 goal sent without the user at the
  robot.
- The flying-pixel filter switched off again silently, or retuned so that it starves the ground fit (floor kept < 95 %
  alone, < 90 % with the amplitude gate).
- Fixtures that do not reproduce the live failures being used to declare them fixed.
- Recordings made on a head chain that is changed afterwards without re-recording (D21).

Head from arm:

- An arm motion with the board attached that was not FK-checked, or a parking move with the board on the gripper.
- Any elbow_flex, shoulder_lift or shoulder_pan motion without the user's OK in the UI.
- Placeholder data: a view without a fresh joint sample for all seven joints or without a board detection is not
  stored; nothing is zero-filled.
- An applied result that failed a limit, or a consumer that silently ignores the applied chain.
- Restarting mcp_server on apply (it kills a running agent session, MEMORY), rclpy probes on the client (MEMORY).

## Constraints

- Umbrella (D20): ground-detection-robustness is closed (done 2026-10-22, review note: live acceptance moved here),
  so its run 4 artifacts exist on main and its blocker is dropped. The README blockers are the three in-progress
  epics whose open runbook tasks still list `ansible/group_vars/client.yml`, `docs/markers.md` or
  `nodes/web_ui/urdf/head.urdf` (head-rgb-d-fusion-of-camera/rgbd-final-suite-and-runbook,
  printable-metadata-markers-t/markers-final-runbook, rtab-map-rgb-d-3d-mapping-on/final-suite-and-runbook);
  `ticket.py` orders cross-epic file overlaps through blockers.
- Live steps (D28) need the user: the scene in place, the light on, the robot static (docked and charging), head moves
  allowed; arm moves only through the head-from-arm sequence with the user's confirms; driving only in
  nav2-goal-across-the-scene, short slow goals with the user present at the robot.
- The current live scene (new layout, box ahead-right of the robot, board about 0.4 m ahead) is the one recorded; the
  box and ramp placements are measured by the user against the board (D22), not assumed against its edges.
- The flying-pixel filter stays off on the robot until tof-flying-pixel-retune is deployed, so the baseline recordings
  hold raw depth (D26).

RGB and amplitude guidance:

- CLAUDE.md: C++ for robot nodes except web_ui and mcp_server; Python type hints, docstrings with types, constants on
  top, pydantic configs with unknown keys rejected; config files over CLI; Ansible mirrors every node config change
  (`ros2_nodes` schema); READMEs and `tests/README.md` updated; no placeholder data on topics.
- No new nodes (user). Live data only through the web_ui websocket/API (user). Recordings: robot static and charging,
  head moves allowed (user).
- The free / warning / lethal tiers and the rule (step > 0.04 m or slope > 15 deg) are unchanged (user).
- ground-detection-robustness is merged and closed (done 2026-10-22); its run 4 artifacts exist on main. They come
  from these run 4 tasks of that epic:
  - `replay-fixtures-and-test`: `nodes/ground_estimator_cpp/test/replay_fixture.hpp` (part replay-harness: reader,
    `ReplayFrame`, the metrics `box_lethal_share`, `board_lethal_frames`, `stray_lethal_share`,
    `far_side_warning_frames`, `ramp_lethal_frames`, `churn_share`, `plane_share`), `test/test_replay.cpp`,
    `scripts/ground_replay_export.py`, `tests/test_ground_replay_export.py` and `test/data/replay/{home,box,ramp}.bin`;
  - `tof-flying-pixel-filter`: `arducam_tof/bridge.py:flying_pixel_filter` (contract tof-depth-flying-pixel-filter);
  - `warning-area-stability`: the area stability in `CellClassFilter`;
  - `replay-acceptance-and-docs`: the replay assertions and the latency pin;
  - `wire-shadow-gate-on-replay` and `shadow-gate-and-segment-step`: the occlusion gate.

  This epic extends them; it does not duplicate them.
- Contracts used from three other in-progress epics are already delivered on main, so they are not added as blockers:
  - `head-rgbd-points` (head-rgb-d-fusion-of-camera): `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp` publishes
    `points()` and the `depth_upsampled` topics;
  - `tof-points-filtered` (tof-3d-obstacle-detection-fo): the laser_filter cloud path to `/tof/points_filtered` in
    `ansible/group_vars/client.yml`;
  - `tof-depth-z` (tof-depth-fix-and-ground-noi): `arducam_tof/bridge.py:radial_to_z`.

  `graph --todo` reports them as "not blocked by owner"; that is expected.
- Tests: C++ through `scripts/cpp-node-test.sh <pkg>` (no test ref runner for .cpp files, so C++ ACs are `command:`
  refs); Python per package with `uv run --frozen pytest`.
- Fixtures go in plain git: at most 3 MB per scene file without pairs and 6 MB with pairs (D9).
- Run plan (D11): three code runs separated by live steps that the orchestrator triggers.
  - Run 1: every task without a live precondition.
  - Live step 1, orchestrator: deploy web_ui and head_rgbd_cpp; the user sets the scene (Q1); then
    capture-baseline-recordings runs.
  - Run 2: tof-amplitude-gate and live-report-guidance-metrics (both need the committed baseline truth and frames).
  - Live step 2, orchestrator: deploy arducam_tof; then capture-gated-recordings runs.
  - Run 3: guidance-replay-acceptance.
  - Then the orchestrator deploys ground_estimator_cpp, costmap_zones, mcp_server and web_ui, and runs
    guidance-live-check-and-cpu.

Head from arm:

- House rules (CLAUDE.md): Python with type hints, pydantic config with every key in the shipped YAML and the
  client.yml block (`tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults` pins them
  equal, so every task that adds node config keys edits `ansible/group_vars/client.yml` too), constants at the top,
  test-first, ROS2 logging in nodes, servo safety (least dramatic joints first, ask before big joints), READMEs and
  tests/README.md, deploy only through `scripts/deploy-nodes.sh` with output in `.logs/`.
- Python stays in web_ui, mcp_server and the existing Python calibration node and core (user decision).
- Arm safety (MEMORY feedback_arm_motion_safety): wrist roll only with the gripper at most half open and the arm lifted
  and forward; negative elbow_flex stretches the arm; a stretched arm with shoulder_lift past about 1.85 rad stalls.
- Head camera mapping stream 576x324; intrinsics valid with the focus lock, which is held while the mapping stream has a
  subscriber (`capture.py`).
- Current head range pan -90..90 deg, tilt -30..+30 deg (+ is down, `head.urdf`); the pose set must work inside it.
- Arm FK: `urdf_angle = measured_angle + offset` (client.yml comment on `joint_offsets_rad`); the arm URDF root is
  joined to the robot base_link by `arm.base_in_base_link` {x 0.0592, y -0.05, z 0.100, yaw 0}.
- `ansible/group_vars/client.yml` is also edited by open tasks of the in-progress epic ground-detection-robustness, so
  that epic is this one's blocker (ticket.py orders cross-epic file overlaps through blockers).
- Tasks about an hour, at most 8 files each.

## Decisions

### D1: Depth is the only height source; RGB and amplitude decide where, never how high

Every height and every lethal decision comes from ToF depth (raw, gated or guided-upsampled). RGB decides where a
boundary sits, and colour helps classify pixels that have no depth. Alternative: monocular or learned depth (non-goal).
Serves A5, A7.

### D2: RGB edges come from the guided-filter upsampling in head_rgbd_cpp, switched on

`head_rgbd_cpp/src/core/upsample.cpp:guided_upsample` already exists: a masked guided filter
D = GF(I, D0*M) / GF(I, M) over the registered depth with the RGB image as guide (He et al., guided filter; design D8
of head-rgb-d-fusion-of-camera). In the upsampled depth, the edges follow the RGB edges, and the 1-3 px holes that the
jump-edge filter leaves are filled from the matching side. It is published as `/head_rgbd/depth_upsampled/*` at the
RGB size (576x324), one frame per RGB-D pair (2 Hz). Upsampling is set to `enabled: true`, and new unit tests pin
three behaviours: the box edge follows the guide edge, a textured flat floor stays flat, and a hole wider than the
radius stays empty.

Alternatives:
- Canny edges plus boundary snapping in ground_estimator_cpp. Rejected: every edge pixel then needs a height, and
  texture edges need their own guard.
- Lowering the decimation of point_cloud_xyz globally. Rejected: it costs CPU in every ToF consumer.

(my call) Serves A3, A5.

### D3: Edge points sharpen lethal boundaries, only next to depth-lethal points

Per guidance frame, ground_estimator_cpp deprojects the upsampled depth (every `guidance.sample_step_px` pixel) into
the fixed frame and keeps it for at most `guidance.max_age_s`. On each ToF frame it takes the guidance points within
`guidance.edge.max_shift_m` (xy) of this frame's stable lethal obstacle points whose height is >= `edge.min_height_m`.
These are voxel-decimated at `edge.voxel_m` and appended to `obstacle_points`. Drops work the same way, with height
<= -min_height and drop points. Without a stable lethal point nearby, no edge point is added (RGB never alone). Edge
points never remove or delay a depth lethal point (my call: safety over tightness). Serves A3, A5.

### D4: Warning areas keep their cell-aligned polygons

The polygons of non-lethal warning areas (steps under the rule, ramps, colour) stay on the 10 cm cell grid of
ground-detection-robustness D8/D18. That grid is what keeps the warning churn under 10 % and what costmap_zones keys
cells by. Only lethal boundaries are refined (D3). Alternative: snapping warning polygons to edges each frame.
Rejected: polygon churn. (my call, Q3) Serves A3.

### D5: No-return areas: no depth, inside the ToF view, colour not floor

Per guidance frame, ground_estimator_cpp takes each sampled RGB pixel that has no upsampled depth and casts its ray
onto the tracked floor plane. The pixel becomes a no-return sample when all of these hold:
- the floor hit lies within [`no_return.min_range_m`, `guidance.max_range_m`];
- the floor hit projects inside the ToF image, with `no_return.tof_margin_px` margin, using the ToF model from
  `/tof/camera_info` and the TF at the pair stamp;
- its colour does not match the floor colour model (existing `FloorColourModel`, Mahalanobis >
  `colour.max_mahalanobis`, model ready).

Samples are counted per fixed-frame temporal cell (10 cm). A cell is a no-return cell in a frame when it has at least
`cell_min_samples` samples and the mismatching no-return share is at least `cell_min_share`. Stability comes from an
N-of-M hysteresis over guidance frames (`window_frames`, `enter_frames`). Cells within `clearance_cells` of a stable
lethal or warning ToF cell are excluded, because that is the occlusion shadow of an object depth already sees.
8-connected groups of at least `min_cells` cells become warning areas: kind `no_return`, source `rgb`, cell-aligned
polygon, `height_m` and `slope_deg` null. They are never lethal: costmap_zones slows down in them, and Nav2 never marks
them. The floor hit is where the ray meets the floor, so the area's near boundary is the object's floor contact line,
and its far part may extend behind the object; that is conservative for a caution area. Serves A4, A5.

### D6: Warning area heights become nullable across producer and consumers

`WarningArea::height_m` and `slope_deg` become `std::optional<double>`. The JSON carries null when no height was
measured. Every existing producer still sets both. The consumers accept null:
- `nodes/costmap_zones/costmap_zones/ground_warnings.py:GroundArea`;
- mcp_server `GroundWarning` / `GroundSummaryWarning`;
- the web_ui `groundLayer.ts`, which draws no_return in its own colour.

The producer of null values (guidance-node-wiring) depends on all three consumer tasks, and the orchestrator deploys
the consumers with it. Alternative: a placeholder 0 height. Rejected: the no-placeholder rule. Cross-epic contract
`ground-warning-no-return`. Serves A4.

### D7: Amplitude gate in arducam_tof

After run 4's `flying_pixel_filter`, a valid depth pixel becomes 0 when amplitude * (z / 1 m)^2 <
`amplitude_min_normalised` (`amplitude_gate: true`). Normalising by range keeps the dim far floor and drops the dark
near surfaces, whose depth is biased. The function is a ROS-free numpy module (`arducam_tof/amplitude.py`) so that
replay scripts can load it by path. The threshold is chosen on recorded frames. If the data shows the board bias is
not explained by amplitude, an amplitude-dependent depth correction fitted on the board is allowed within the same
acceptance (left to the builder). The gate applies to every consumer of `/tof/depth/image_raw`, like the flying-pixel
filter (Q4). Cross-epic contract `tof-depth-amplitude-gate`. Serves A2.

### D8: Recordings through a web_ui recorder and a record script

web_ui (an existing, long-lived rclpy node) gets a recorder API. It subscribes on demand to:
- the ToF cloud, depth, amplitude and camera_info;
- the guidance pair (upsampled depth, its camera_info, RGB JPEG);
- IMU and head joint states.

It looks up the TF of every message at its stamp (dropping, never guessing) and writes a compressed npz. The npz
layout is a superset of the 2026-10-22 recordings (`ground-recording-npz`). `scripts/ground_record.py` drives the
scenes: it points the head over the web_ui websocket, starts the recordings, captures `/ground/state` lines and
downloads the files. No rclpy probe, no SSH. Alternatives:
- rosbag2 on the client. Rejected: a new DDS participant, the class of problem that broke tof_obstacle_cloud.
- The websocket image stream. Rejected: it carries lossy 8-bit previews.

Serves A1, A8.

### D9: Replay fixture v2

Version 2 of run 4's binary adds a guidance section: the RGB and ToF camera models, and per RGB-D pair its stamp,
base_T_rgb, odom_T_base, the JPEG bytes and the upsampled depth as lossless 16-bit PNG. The C++ reader decodes v1 and
v2. Fixtures keep every pair at the live rate (2 Hz) and size (576x324), so the replay matches the live guidance. The cap is 3 MB per file without pairs (15 s windows) and 6 MB with pairs (10 s windows, at least 8 s; about 0.2 MB of PNG and 35 kB of JPEG per pair). Exports above the cap fail. A small ToF frame subset (depth +
amplitude, with the truth polygons `truth_board`, `truth_ramp`, `truth_empty` and the `floor_plane`, written by `export_tof_frames` in capture-baseline-recordings) goes to `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz` for the gate's real-data test. Plain
git, no LFS (my call: total under 30 MB, no new tooling). Serves A1, A6.

### D10: Ground truth from the ChArUco board pose

The robot stays static, so base_link is fixed. The board pose in base_link comes from ChArUco detection in the
recorded RGB frames (`shared/ros2_common/head_rgbd_calibration/charuco.py:detect`, `make_board`; solvePnP with the
recorded RGB model): the median over >= 5 detections, with a corner spread <= 5 mm. The box and the ramp are placed
against the board's edges, and their dimensions are measured by the user (Q1). Their footprints follow from the board
pose: `scripts/ground_scenes.yaml` gives the sheet, and `truth.yaml` is generated next to the fixtures.
Alternative: the ToF cell medians (as on 2026-10-22). Rejected: about 5 cm uncertainty, the size of the effect being
measured. (my call) Serves A1, A6.

### D11: Run plan with live steps between code runs

Recording needs deployed code and the user's scene, so the work runs as three plan-execute runs. The live tasks
(capture-baseline-recordings, capture-gated-recordings, guidance-live-check-and-cpu) state their deploy precondition
in their Notes, and the orchestrator deploys before dispatching them. live-report-guidance-metrics runs in run 2, after capture-baseline-recordings has committed `truth.yaml`. Serves A1.

### D12: Guidance freshness and the 2 Hz mapping stream

A guidance frame is used only while it is no older than `guidance.max_age_s` (0.6 s) relative to the ToF cloud stamp,
moved through the fixed frame (odom). An older frame or a frame without TF contributes nothing, and that cycle
publishes no edge points and no no-return update. The mapping stream stays at 2 Hz (Q2). Serves A3.

### D13: CPU measured with the client-cpu-budget method

Before/after numbers come from `rate(container_cpu_usage_seconds_total{host="client"})` through the existing
`scripts/node_cpu_compare.py`, run with a new config `scripts/ground_guidance_cpu.yaml` (runbook client-cpu-budget).
The three features are switched on by three deploys at different times, so each node gets its own `switch_time`:
- head_rgbd_cpp: live step 1;
- arducam_tof: live step 2;
- ground_estimator_cpp: the final deploy.

The orchestrator records each deploy's `date +%s`. The config sets `window_h: 1` and `settle_min: 10`, so per node the
before window is [switch - 1 h, switch] and the after window is [switch + 10 min, switch + 70 min]; the robot is static,
so the load is steady. A node's added CPU is its after mean minus its before mean. The total is the sum of the three
deltas, and the target is at most 0.15 of one core. Over that is a finding, not a pass. Mimir keeps 1 day: a switch
outside 70 min to 23 h before the run is a named "not measurable" row, never an estimate. (my call) Serves A6.

### D14: Baseline first

capture-baseline-recordings replays the new recordings with run 4's behaviour (guidance off, gate off) and writes
those lines into the README before the gate or the guidance is tuned. Every later acceptance prints its metric next to
that baseline. Serves A1.

### D15: Tiers and rule unchanged

The tiers stay free / warning / lethal. `traversability.max_step_fraction` (0.3333, a 0.04 m step) and
`max_slope_deg` 15 stay as they are, and the replay acceptance asserts the shipped values. The no_return kind is a
warning (caution) tier member. Serves A7.

### D16: User 2026-10-22 approved (yes): user sets and keeps the scen
User 2026-10-22 approved (yes): user sets and keeps the scene; box/ramp dimensions asked at capture time (user decision)

### D17: User 2026-10-22 approved (yes): keep head mapping RGB at 2 H
User 2026-10-22 approved (yes): keep head mapping RGB at 2 Hz (user decision)

### D18: User 2026-10-22 approved (yes): RGB edges refine only lethal
User 2026-10-22 approved (yes): RGB edges refine only lethal boundaries (user decision)

### D19: User 2026-10-22 approved (yes): amplitude gate on /tof/depth
User 2026-10-22 approved (yes): amplitude gate on /tof/depth/image_raw for every consumer (user decision)

### D20: One umbrella epic; the approved plans are reused, not duplicated

User decision 2026-10-22: "Go for all 3 within one epic". The approved todo epic rgb-and-amplitude-guided-gro became
this epic (`ticket.py rename`), and the 20 tasks of the approved todo epic head-and-arm-calibration-fro moved in with
their requirement and AC text, approvals (H1-H9 unchanged, its Q1-Q4 as HQ1-HQ4) and decisions (its D<n> is D<n+30>,
its Q<n> is Q<n+4>; references inside the moved task files renumbered the same way). The arm joint offset follow-up
(arm-joint-offsets-from-the-g) stays a separate epic, now blocked by this one. Items 1 and 2 reuse the recorder,
record script, scene truth, replay v2, amplitude gate and guidance tasks; new tasks only add what they lack.
Alternative: three separate epics; rejected by the user. Serves G3.

### D21: Head-from-arm calibration is applied before the ground scene is recorded

Every recording stores the TF of its moment (base_T_cloud, base_T_rgb). If the head chain changes after the
recordings, the replay judges an old geometry and the live run a new one, so the head-from-arm live step
(calib-robot-runbook) runs before capture-baseline-recordings (dependency), and the 1.9-2.2 deg plane-vs-base tilt is
first measured on the calibrated chain. The ChArUco board of the ground scene is the 6x5 board of the calibration
(D45); when it is the same physical board, it leaves the scene for the calibration and goes back before the baseline
recording, and its pose is measured again then (D22). (my call, Q9) Serves G1, G3.

### D22: The live scene is recorded with truth from the board pose and measured placements

The scene recorded is the current live layout (box, ramp and board ahead and to the right of the robot), at the head
poses where the live failures were seen: home (pan 0, the board about 0.4 m ahead) and the box at pan -30. The board
pose comes from ChArUco (D10). The box and ramp footprints are no longer assumed against the board's edges: the user
measures each placement against the board (offset of the object's reference corner along and across the board, and
its yaw), the scene sheet holds those numbers, and `scripts/ground_scene_truth.py` builds the polygons. The ToF frame
subset gains the box (`truth_box`, `box_height_m`) and frames of the box scene, so the filter retune can measure the
box edges on real data. Alternative: rearrange the scene to fit the old sheet; rejected, the user asked for this scene.
(my call) Serves G1, G2.

### D23: The estimator passes the live thresholds on the gated replay, guidance off, before it is deployed

ground-estimator-live-scene replays the gated fixtures (filter and amplitude gate on, head chain applied) with the
shipped config and `Guidance::off`, first reproduces the live failures, then fixes the estimator until: box lethal
share >= 0.95 (home and the pan -30 box scene), board lethal frames 0, empty-floor lethal share <= 0.05 (0 lethal points
at p95), ramp lethal frames 0, churn < 0.10, stray lethal share <= 0.05 and far-side warnings (ramp, step and drop kinds,
0.20 m behind the box) in <= 5 % of frames (my call). The plane tilt is reported with its cause. Thresholds are never
loosened; the rule and the lethal latency stay; run 4's fixtures keep passing. guidance-replay-acceptance then runs
with the guidance on and depends on it. Serves G1.

### D24: Live acceptance through web_ui captures, recorded in the ground-robustness runbook

ground-live-acceptance captures `/ground/state` over the web_ui websocket with `scripts/ground_record.py` (two 60 s
windows per scene keep the runbook's 2 minutes within the recorder's 60 s cap), judges every window with
`scripts/ground_noise_report.py` and the truth ROIs, and writes the Results of `docs/runbooks/ground-robustness.md`
(its Scene rewritten for the new layout, its Procedure moved from topic_scraper to web_ui). A failing row escalates
with the metric; it never closes the task. Serves G1.

### D25: The ground layer goes on after the live acceptance; Nav2 checked with short goals and the user present

ground-voxel-layer-on flips `ground_voxel_layer.enabled` only after every live Results row passed (the other keys,
including the 15 s decay, unchanged). nav2-goal-across-the-scene then drives two goals of at most 1.5 m (across the
scene around the box, and back) through mcp_server's guarded navigation tools under the base lease, Nav2's 0.25 m/s
cap unchanged, with the user at the robot; the local costmap is read through web_ui. Its runbook is a new file,
because the ground-robustness runbook forbids driving. (my call, Q11) Serves G1.

### D26: The flying-pixel filter is retuned on the recorded raw depth, then turned on

The baseline recordings are taken with the filter off, so the ToF frame subset holds raw depth. tof-flying-pixel-retune
first measures why the current rule removes about 70 % (per scene, per range band), then retunes the keys or the rule
inside `flying_pixel_filter` (left to the builder: a plane-consistent test, a neighbour count or a range-dependent
threshold), judged on the real frames: floor kept >= 95 % with the filter alone (user), mixed pixels at the box edges
removed >= 80 % (my call), box top and board kept >= 95 %, filter plus amplitude gate keep >= 90 % of the floor (my
call: the ground fit must not starve again). It runs after tof-amplitude-gate (same files) and is deployed with it at
live step 2, so the gated fixtures carry both. Cross-epic contract `tof-flying-pixel-retune`. (my call, Q10) Serves
G2.

### D27: Live filter check from the node's counters

After the live step 2 deploy, tof-filter-live-check reads `tof_flying_pixels_removed_total` and
`tof_amplitude_gated_pixels_total` from Mimir (the Prometheus API `scripts/node_cpu_compare.py` uses), with the head
at home for 10 min: the flying-pixel removed share must be <= 0.10 of all pixels (my call; it was about 0.70), and the
ground plane must be valid in >= 95 % of the `/ground/state` lines of every gated recording. (my call, Q10) Serves G2.

### D28: Live steps need the user, and say so

Every live task names its preconditions and stops with `blocked`, `whoCanClear: user` when one is missing: the scene
in place and unchanged (board within 5 mm of `truth.yaml` after the baseline), the light on, the robot static
(docked and charging), head moves allowed; the arm moves only in calib-robot-runbook with the user's confirms;
driving only in nav2-goal-across-the-scene with the user present. Live data only through web_ui, mcp_server tools and
Mimir; never rclpy, the `ros2` CLI or SSH on the client. Serves G1, G2, G3.

### D29: Run plan: four code runs and five live steps

Extends D11. The orchestrator runs:
- Live step 0 (user, any time before run 1 ends): board-on-gripper-hardware-st (the Hardware answers).
- Run 1 (code): every task without a live precondition: the RGB and amplitude run 1 tasks and the head-from-arm code
  tasks. File order on `ansible/group_vars/client.yml` and web_ui: rgbd-upsampling-on, guidance-config-keys,
  web-ui-ground-recorder-api, then the head-from-arm chain from joint-state-still-capture (its dependency on
  web-ui-ground-recorder-api is file order only).
- Live step 1 (user: arm moves with confirms, robot docked; then the scene, light on, robot static, head moves): deploy
  head_rgbd_calibration, head_state_publisher, mcp_server, web_ui, base_servos and head_rgbd_cpp; calib-robot-runbook
  (apply the head chain); the user puts the board back into the scene; capture-baseline-recordings.
- Run 2 (code): tof-amplitude-gate, then tof-flying-pixel-retune; live-report-guidance-metrics.
- Live step 2 (user: scene unchanged, light on, robot static, head moves): deploy arducam_tof (filter on, gate on);
  capture-gated-recordings; tof-filter-live-check.
- Run 3 (code): ground-estimator-live-scene, then guidance-replay-acceptance.
- Live step 3 (user: scene, light, robot static, head moves): deploy ground_estimator_cpp, costmap_zones, mcp_server,
  web_ui; guidance-live-check-and-cpu; ground-live-acceptance.
- Run 4 (code): ground-voxel-layer-on.
- Live step 4 (user present at the robot, undocked; short slow goals): deploy nav2_bringup; nav2-goal-across-the-scene.

Each live task states its deploy precondition; the orchestrator deploys before dispatching it (deploying mcp_server
kills a running agent session: check first). Serves G1, G2, G3.

### D31: Calibrate the head chain against the trusted arm with the board on the gripper

The 6x5 board clipped to the fixed jaw is moved by the arm; the head camera sees it over many arm poses and head
angles. Arm FK (so101 URDF, measured joints plus `joint_offsets_rad`, `base_in_base_link`) gives the board in base_link
up to the board's fixed pose in `gripper_link`, which is fitted (6 DOF). The arm is not fitted. Alternatives: a floor
board at several spots (cannot see yaw/x/y against the board's own pose; the old plan), a joint head+arm fit (moved to
the follow-up). User decision 2026-10-22. Serves H1.

### D32: Free parameters and the degeneracies fixed

Free: `head_mount_joint` x, y, z, roll, pitch, yaw; `head_tilt` origin pitch (tilt zero) and roll (tilt roll); the
board pose in `gripper_link`. Names `mount_z_m`, `mount_roll_deg`, `mount_pitch_deg`, `tilt_zero_deg`,
`tilt_roll_deg` equal `floor_fit.PARAM_NAMES`; new `mount_x_m`, `mount_y_m`, `mount_yaw_deg`. Fixed at the applied
chain: `head_tilt` xyz (the 0.04 m offset is degenerate with mount z), `head_pan` origin (D33), `head_camera_joint` and
the optical joints (RGB-D calibration; the camera pitch is degenerate with the tilt zero), intrinsics; the whole arm.
(my call for the camera lever: not in the user's list.) Serves H1.

### D33: The pan zero is folded into the mount yaw

A pan zero offset and the mount yaw both rotate about the pan axis; with the mount tilted 1.5 deg they differ only to
second order (about 0.04 deg per deg), so they are one parameter. It is written as `head_mount_joint` rpy yaw in
`head_chain.json` (format unchanged): TF and look_at are exact, and a raw pan command of 0 in the Head tab points off
by the fitted yaw. Alternative: a servo pan zero in base_servos (re-zeroes the servo, needs a bridge redeploy and moves
every recorded pan angle). Open question Q5. Serves H1.

### D34: Floor-fit values as start values and cross-check

The fit starts from the applied chain (the floor-fit z, roll, pitch, tilt zero and tilt roll). For those five,
`floor_delta` = fitted minus the applied floor result; over `max_floor_delta_mm` 10 / `max_floor_delta_deg` 1.5 the
result fails. x, y and yaw get sanity limits against their start (`max_start_delta_mm` 30, `max_start_delta_deg` 5). An
optional prior (`floor_prior_weight`, default 0) adds one residual per floor parameter; off by default because the
floor fit carries the garage floor slope (normal RMS 0.83 deg). The floor values come from the robot's
`applied/applied.json` `head_chain` record: today's `apply_floor` (`floor_results.py`) already writes `{session,
result, applied_at}` there, so the 2026-10-22 apply recorded session floor-20261022-064337, result r-20261022-065211;
a record without the new `floor` key whose session matches `FLOOR_SESSION_ID_RE` is that floor result itself, and its
`floor_fit/<sid>/results/<rid>/result.json` `params` are the cross-check values. From now on both applies write
`source` and `floor`. Open question Q6. Serves H1, H3.

### D35: One robust least-squares fit with sigma, held-out arm poses and limits

Per view `cv2.solvePnP` seeds the board mount; one `scipy.optimize.least_squares` (Huber, `huber_f_scale_px`) over the
ChArUco corner reprojection of all fit views; sigma from `(J^T J)^-1 * s^2`; the largest parameter correlation is
reported. Every `holdout_every`-th arm pose is held out with all its views. Limits: `rms_px` 1.0, `holdout_rms_px`
1.5, `holdout_board_mm` 5.0 (board centre from the head vs from arm FK on held-out views), `max_sigma_mm` 2.0,
`max_sigma_deg` 0.2, `min_views` 40, `min_arm_poses` 16, `min_pan_span_deg` 30, `min_tilt_span_deg` 10, plus D34. A
result with any failed limit has `passed: false` and cannot be applied. Serves H3.

### D36: Extend the head_rgbd_calibration node, reuse the floor-fit machinery

Sessions gain kind `head_arm` in the existing store (RGB still poses with seven measured joints); compute runs in the
HTTP handler like `compute_floor` and writes `result.json`, `report.md` and, when passed, `head_chain.json`; apply goes
through one writer shared with `apply_floor` (copy into `applied/`, `head_chain` record in `applied.json`, the apply
path unit restarts head_state_publisher and the head consumers). Routes under `/api/head-arm/` beside `/api/floor/`.
Python because it extends an existing Python node (user decision). Serves H1, H4, H9.

### D37: Robot motion through mcp_server tools, head aimed by the node

An MCP client (`robot_client.py`, modelled on `nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool`, token
`/etc/ros2/mcp_server/token`, group `mcp-token`) calls `take_control`, `move_arm_joints`, `set_head_angles`,
`release_control`, `get_arm_state`, so the leases, the floor guard, the body self-collision check, the roll guard and
the battery gate apply. The node aims the head itself: nominal board centre from arm FK, pan/tilt solved on the applied
head chain (`aim_head`), plus the configured `head_offsets_deg` per arm pose so the board moves across the image and the
pan/tilt span grows; offsets outside the URDF limits are skipped. Serves H2.

### D38: Arm safety: confirms, tiers, checked poses, holding exits

Order: "Move to base pose" (confirm naming the joints that move; no board attached) -> the user clips the board and
presses "Board attached" -> tier 0 (wrist_roll only) -> tier 1 (wrist_flex and wrist_roll) -> a confirm naming
elbow_flex, shoulder_lift, shoulder_pan -> tier 2 -> back to the base pose -> hold. Declining the confirm skips tier 2.
The gripper target is sent once with the base pose (`gripper_rad`, at most the half-open `max_roll_gripper_rad`) and
never again. Before every command the path from the measured joints to the target is FK-checked (D39); a refusal stops
before anything moves. Every exit (end, stop, tool error, exception, confirm timeout) releases with
`keep_position: true`: the arm holds a checked pose and mcp_server keeps the head where it is (`reset.head: kept`,
`control.py::release_head`). "Park" (`release_control` with park) is a separate confirmed action after the user
removed the board; it is refused while `get_control_state` reports `arm.carrying`, and its release homes the head after
the arm (the board stays outside the head envelope anyway). The board is never a carried object: `move_arm_joints`
writes no carry record, the sequence never sends `carried_object_radius_m` (the keep release publishes the frozen
measured-arm footprint without a carried circle; the robot does not drive), and `gripper_rad` leaves the jaws without a
holding load on the clip. `move_arm_joints` targets are measured follower angles (`arm.py::move_joints`: "measured
(follower) joint space, as reported by get_arm_state"; `urdf = measured + offset`). speed_scale <= 0.3. User decision 2026-10-22 (supersedes the old gate Q4: elbow moves now need the confirm).
Open question Q7. Serves H2.

### D39: Pose check in the shared core

`arm_pose_check.py` checks a pose and a joint-linear path (samples every `path_step_rad`): tool point >= 0.10 m and
board outline points >= 0.05 m above the floor; board points and arm link capsules (upper arm, forearm, wrist) at least
`margin_m` from the body obstacles (a copy of mcp_server's `body` boxes and cylinders, pinned equal by the deploy test)
and from the head envelope (a vertical cylinder about the pan axis of the given head URDF containing the head at every
pan/tilt inside that URDF's joint limits, so it covers a widened tilt range too; pinned against the URDF visuals); board points clear of the upper arm and forearm; a wrist_roll change only
with the gripper at most half open and the tool >= 0.15 m high; shoulder_lift over 1.85 rad with a negative elbow_flex
refused; joints inside the arm URDF limits. mcp_server keeps its own checks on top. Serves H2.

### D40: Every head-chain consumer reads the applied override

head_state_publisher already applies it (delivered by the floor epic). mcp_server rebuilds HeadKinematics when
`applied/applied.json` changes (mtime checked per call), the front camera sits on `head_camera_link` with the applied
intrinsics, and web_ui serves the head URDF with the overrides applied per request. Serves H4.

### D41: One guided "Head from arm" section on the Calibrate page

Built like HeadFloorFit (MUI, step cards, polling). It replaces nothing; the "Arm calibration - deferred" line stays
for the follow-up epic. Serves H5.

### D42: Tilt range widened to the reprinted mount's recorded limit

Default -30..+60 deg (+ is down); when the hardware step records a smaller tilt-down limit, that value. The pose set does
not depend on it (it works within -30..+30). Kept from the old gate Q1. Serves H6.

### D43: Synthetic scenes first, robot runbook last

Each fit and check is written test-first against synthetic scenes (corners projected through a known perturbed chain,
0.3 px noise); the shipped pose set is pinned by a test that also fits a synthetic scene generated on it. The epic ends
with a runbook executed with the user on the robot. Serves H7.

### D44: The arm joint offset solve moves to a follow-up epic

arm-offsets-plate-solver, arm-offsets-compute-apply and arm-calibrate-ui-section moved verbatim to epic
arm-joint-offsets-from-the-g (blocked by this epic), to be re-planned there against this epic's parts. User decision
2026-10-22. Serves H8.

### D45: The existing 6x5 board is the gripper board

`models/charuco-6x5-40mm` (`scripts/markers_generate.yaml` `charuco:`, ids 0-14) on a clip on the fixed jaw; no new
plate. Kept from the old gate Q2. Serves H2.

### D47: The board mount is fitted; the measurement is only a start value

Standard hand-eye: the target-in-gripper pose is unknown and fitted (6 DOF, D32), so neither the solver nor the pose
set waits for the hardware step. The pose set ships with a nominal mount from the jaw geometry, checked with an extra
`provisional_margin_m` and a +-1 cm / +-5 deg mount band; the user's measurement replaces the nominal start value in
calib-robot-runbook, and the poses stay provisional until a dry run without the board there. Open question Q8 ("no"
puts arm-board-pose-set back behind board-on-gripper-hardware-st). (my call pending Q8) Serves H2.

### D46: Guided step-by-step process

Step cards for the physical steps (clip the board, remove it), the confirms and the result; automation only where it
cuts work without adding risk (scripted arm poses and head offsets). Kept from the old gate. Serves H5.

### D48: User 2026-10-22 approved (yes): fold pan zero into the head_
User 2026-10-22 approved (yes): fold pan zero into the head_mount_joint yaw (user decision)

### D49: User 2026-10-22 approved (yes): floor-fit values as start va
User 2026-10-22 approved (yes): floor-fit values as start values and pass/fail cross-check, weighted prior off (user decision)

### D50: User 2026-10-22 approved (yes): clip board only after base p
User 2026-10-22 approved (yes): clip board only after base pose, hold arm on every stop, park only after board removal confirmed (user decision)

### D51: User 2026-10-22 approved (yes): solver fits the board-on-jaw
User 2026-10-22 approved (yes): solver fits the board-on-jaw pose, measurement only a start value, poses provisional until a dry run (user decision)

## Research

| Lens | Ran / skipped | Adopted | Rejected |
|---|---|---|---|
| Codebase (planner, grep/read) | ran | guided_upsample exists and is off (`nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`); upsampled depth is published at the RGB size with one header per pair (`nodes/head_rgbd_cpp/src/head_rgbd_node.cpp:publish_pair`); amplitude is already pixel-aligned mono16 (`nodes/bridges/arducam_tof/arducam_tof/bridge.py` AMPLITUDE_TOPIC); FloorColourModel (`nodes/ground_estimator_cpp/include/ground_estimator_cpp/colour.hpp`); ChArUco detect (`shared/ros2_common/head_rgbd_calibration/charuco.py:detect`); web_ui already has a TF buffer and on-demand subscriptions (`nodes/web_ui/web_ui/bridge.py`) | rosbag2 on the client (DDS participant); websocket image capture (8-bit JPEG previews, `nodes/web_ui/web_ui/msg_serializer.py`) |
| Live brief (ground-live.md, 2026-10-22) | ran | mixed pixels, board albedo bias, no returns on the black ramp, gripper zone x < 0.37 excluded | ToF cell-median ground truth (5 cm uncertainty) |
| External library docs | skipped: no new library API. OpenCV imdecode, undistortPoints and solvePnP are standard, and the guided filter is in-repo | - | OpenCV ximgproc guidedFilter (the in-repo masked version handles holes; ximgproc does not) |
| Prior art (guided depth upsampling) | ran from the in-repo design (head-rgb-d-fusion-of-camera D8) | joint guided upsampling aligns depth edges to image edges (He, Sun, Tang, "Guided Image Filtering", TPAMI 2013) | learned depth completion (non-goal) |
| Codebase, umbrella re-plan 2026-10-22 (grep/read) | ran | `nodes/nav2_bringup/config/nav2_params.yaml` ground_voxel_layer `enabled: false` (commit a04b7df1) pinned by `tests/test_nav_ground_voxel_layer.py::test_ground_layer_settings_equal_the_tof_layer_settings`; `ansible/group_vars/client.yml` arducam_tof `flying_pixel_filter: false`, `min_range_m: 0.25`; `nodes/ground_estimator_cpp/test/replay_fixture.hpp` run 4 metrics with ROI constants of the old layout (`BOX_LETHAL_ROI`, `FAR_SIDE_ROI` kinds ramp/step only); `docs/runbooks/ground-robustness.md` Results empty and Safety forbidding driving; `nodes/mcp_server/mcp_server/tools.py:navigate_to_pose`, `move_relative` (base lease, battery and tilt gates); Nav2 speed cap 0.25 m/s (`nav2_params.yaml` vx_max); Mimir at `scripts/node_cpu_compare.py` `prometheus_url` | topic_scraper_api for the live acceptance (needs an enable/deploy/disable cycle; the web_ui recorder captures the same lines) |
| research-code-internal | ran (this re-plan read the delivered code: `shared/ros2_common/head_rgbd_calibration/{chain,floor_fit,urdf_text,charuco,config}.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/{floor_sweep,floor_results,floor_api,capture,store,__main__}.py`, `nodes/mcp_server/mcp_server/{head_kinematics,self_collision,body_model,tools}.py`, `nodes/web_ui/frontend/src/calibrate/`) | reuse `UrdfChain`, `arm_in_robot_base`, `head_chain_json`, `apply_overrides`, `parse_head_chain`, floor `apply_floor` writer, `base_urdf`, `rgb_qos`, `write_json_atomic`; mcp `call_robot_tool` pattern | a new node; mcp_server's C++ self-collision kernel from the node (not importable there) |
| research-code-external | skipped: ChArUco, solvePnP and least_squares are already used in the core | - | - |
| research-papers | skipped: hand-eye / kinematic calibration by bundle adjustment over a trusted arm is standard | - | - |
| research-docs | skipped: no new external API beyond the mcp SDK client claude_agent already uses (mcp==2.3.0) | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| ground-live.md root cause 1 | mixed pixels at edges; run 4 filters them in arducam_tof | rejected: delivered by ground-detection-robustness/tof-flying-pixel-filter, reused (contract tof-depth-flying-pixel-filter) |
| ground-live.md D1 | replay fixtures + C++ replay of the real node | replay-format-v2, guidance-replay-metrics (extend run 4's replay-harness) |
| ground-live.md ground truth | board x 0.37-0.61, box x 0.60-0.78, ramp no returns | scene-truth-from-board (re-measured from the board pose), guidance-replay-acceptance |
| ground-live.md gripper zone | exclude x < 0.37 in every metric | guidance-replay-metrics |
| brief facts: head_rgbd registers ToF into RGB at 288x162, guided upsampling off | enable and pin the upsampling | rgbd-upsampling-on |
| brief facts: CIELAB floor colour model (rgb-d-ground-estimation-and D6) | reuse for the no-return colour test | no-return-core |
| brief facts: amplitude pixel-aligned with depth | amplitude gate | tof-amplitude-gate |
| user: recordings without rclpy probes | web_ui recorder + record script | ground-recording-layout, web-ui-ground-recorder-api, ground-record-script |
| user: baseline first | baseline replay lines before tuning | capture-baseline-recordings |
| client-cpu-budget runbook | CPU before/after via node_cpu_compare | guidance-live-check-and-cpu |
| ground-robustness runbook (topic_scraper capture) | live capture through the web_ui websocket instead (user) | ground-record-script, live-report-guidance-metrics |
| user 2026-10-22, item 1 | record THIS scene with the recorder, not ad-hoc rclpy | capture-baseline-recordings, scene-truth-from-board (D22) |
| user 2026-10-22, item 1 | fix the estimator until the live acceptance passes | ground-estimator-live-scene, ground-live-acceptance |
| user 2026-10-22, item 1 | ground_voxel_layer back on, Nav2 reaches a goal across the scene | ground-voxel-layer-on, nav2-goal-across-the-scene |
| user 2026-10-22, item 2 | retune the flying-pixel filter on recorded raw depth, keep >= 95 % floor, remove box-edge mixed pixels, back on | replay-format-v2 (truth_box), capture-baseline-recordings, tof-flying-pixel-retune, tof-filter-live-check |
| user 2026-10-22, item 3 | head-from-arm calibration epic folded in, follow-up kept separate | the 20 moved tasks, calib-robot-runbook before capture-baseline-recordings (D21) |
| ground-detection-robustness record (README Review, 2026-10-22) | live acceptance A8 unmet, moved | ground-live-acceptance |
| code: `docs/runbooks/ground-robustness.md` `## Safety` forbids driving (`tests/test_ground_robustness_runbook.py::test_runbook_forbids_driving`) | the Nav2 check needs its own runbook | nav2-goal-across-the-scene |
| brief 1 (dispatcher) | floor fit delivered: override format, FK core, head_state_publisher chain, floor sweep/compute/apply | rejected: delivered (urdf-chain-fk-core, head-chain-override-format, head-state-publisher-chain dropped) |
| brief 1 | reuse the floor-fit session/compute/apply machinery | head-arm-compute-and-apply, head-arm-sequence-http-api |
| brief 1 | mcp_server HeadKinematics, web_ui head URDF and front camera still ignore the override | mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| brief 2 | the floor cannot observe yaw, mount x/y, pan zero | head-from-arm-board-solver |
| user decision 2026-10-22 | arm trusted; fit the head chain from the board on the gripper; floor values as prior/cross-check | head-from-arm-board-solver, head-arm-compute-and-apply |
| user decision 2026-10-22 | arm offset solve to a follow-up epic | calib-docs-and-test-index (D44; the move itself done by ticket.py) |
| user decision (safety) | least dramatic joints first, UI confirm for elbow/shoulder, FK check vs body and head, roll rules | arm-pose-safety-check, arm-board-pose-set, head-arm-pose-capture-sequen, head-arm-calibrate-ui-sectio |
| user decision | held-out limits before Apply; applied through the override to every consumer | head-from-arm-board-solver, head-arm-compute-and-apply, mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| code: `nodes/mcp_server/mcp_server/body_model.py`, client.yml mcp_server `body` | mcp's body model has no head and does not know the board | arm-pose-safety-check |
| code: client.yml `test_ros2_nodes_entry_matches_defaults` | node config keys must land in client.yml in the same task | joint-state-still-capture, robot-mcp-client, arm-board-pose-set |
| reuse | `nodes/claude_agent/claude_agent/robot_stop.py:call_robot_tool` | robot-mcp-client |
| reuse | `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py:apply_floor` | head-arm-compute-and-apply |
| reuse | `shared/ros2_common/head_rgbd_calibration/chain.py:UrdfChain` | head-from-arm-board-solver, arm-pose-safety-check |
| reuse | `shared/ros2_common/head_rgbd_calibration/charuco.py:detect` | joint-state-still-capture |
| old plan | floor-board head grid, head_chain sessions, tof_joint recompute | rejected: the floor fit replaced the floor board; the camera joint is fixed so tof_joint is unchanged |
| old plan | drive-straight yaw check | rejected: the robot is charging, no driving (user) |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Depth is the only height source | edge-points-core, no-return-core, guidance-replay-acceptance |
| D2 Guided upsampling switched on | rgbd-upsampling-on, guidance-config-keys |
| D3 Edge points next to depth-lethal points | guidance-frame-core, edge-points-core, guidance-node-wiring, guidance-replay-acceptance |
| D4 Warning areas keep cell polygons | guidance-node-wiring |
| D5 No-return areas | guidance-frame-core, no-return-core, guidance-node-wiring, guidance-replay-acceptance |
| D6 Nullable warning heights | area-heights-nullable, costmap-nullable-heights, mcp-nullable-heights, web-ui-no-return-areas |
| D7 Amplitude gate | tof-amplitude-gate, capture-gated-recordings |
| D8 Recordings through web_ui | ground-recording-layout, web-ui-ground-recorder-api, ground-record-script |
| D9 Replay fixture v2 | replay-format-v2, guidance-replay-metrics |
| D10 Ground truth from the board | scene-truth-from-board, capture-baseline-recordings |
| D11 Run plan with live steps | capture-baseline-recordings, capture-gated-recordings, guidance-live-check-and-cpu |
| D12 Guidance freshness | guidance-config-keys, guidance-node-wiring |
| D13 CPU measured | guidance-live-check-and-cpu |
| D14 Baseline first | capture-baseline-recordings |
| D15 Tiers and rule unchanged | guidance-replay-acceptance, live-report-guidance-metrics |
| D16 User 2026-10-22 approved (yes): user sets and keeps the scen | capture-baseline-recordings, scene-truth-from-board |
| D17 User 2026-10-22 approved (yes): keep head mapping RGB at 2 H | rgbd-upsampling-on |
| D18 User 2026-10-22 approved (yes): RGB edges refine only lethal | no-return-core |
| D19 User 2026-10-22 approved (yes): amplitude gate on /tof/depth | tof-amplitude-gate |
| D20 One umbrella epic | calib-robot-runbook, capture-baseline-recordings |
| D21 Head chain applied before the recordings | calib-robot-runbook, capture-baseline-recordings |
| D22 Live scene with measured placements | scene-truth-from-board, replay-format-v2, capture-baseline-recordings |
| D23 Estimator passes on the gated replay | ground-estimator-live-scene, guidance-replay-acceptance |
| D24 Live acceptance through web_ui | ground-live-acceptance, live-report-guidance-metrics |
| D25 Ground layer on, Nav2 check | ground-voxel-layer-on, nav2-goal-across-the-scene |
| D26 Flying-pixel retune on raw depth | tof-flying-pixel-retune, capture-gated-recordings |
| D27 Live filter check | tof-filter-live-check |
| D28 Live steps need the user | capture-baseline-recordings, capture-gated-recordings, tof-filter-live-check, ground-live-acceptance, nav2-goal-across-the-scene, calib-robot-runbook |
| D29 Run plan | joint-state-still-capture, capture-baseline-recordings, capture-gated-recordings, guidance-live-check-and-cpu, ground-live-acceptance, nav2-goal-across-the-scene |
| D31 Calibrate the head chain against the trusted arm | head-from-arm-board-solver, head-arm-compute-and-apply |
| D32 Free parameters and the degeneracies fixed | calib-config-and-view-types, head-from-arm-board-solver |
| D33 The pan zero is folded into the mount yaw | head-from-arm-board-solver |
| D34 Floor-fit values as start values and cross-check | head-from-arm-board-solver, head-arm-compute-and-apply |
| D35 One robust least-squares fit with sigma, held-out arm poses and limits | calib-config-and-view-types, head-from-arm-board-solver |
| D36 Extend the node, reuse the floor-fit machinery | session-kinds-joint-poses, joint-state-still-capture, head-arm-compute-and-apply, head-arm-sequence-http-api |
| D37 Robot motion through mcp_server tools, head aimed by the node | robot-mcp-client, head-arm-pose-capture-sequen, calib-ansible-deploy-config |
| D38 Arm safety: confirms, tiers, checked poses, holding exits | arm-board-pose-set, head-arm-pose-capture-sequen, head-arm-sequence-http-api, head-arm-calibrate-ui-sectio |
| D39 Pose check in the shared core | arm-pose-safety-check, arm-board-pose-set, calib-ansible-deploy-config |
| D40 Every head-chain consumer reads the applied override | mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| D41 One guided Head from arm section | calib-frontend-api, head-arm-calibrate-ui-sectio |
| D42 Tilt range widened | board-on-gripper-hardware-st, head-tilt-range-widened |
| D43 Synthetic scenes first, robot runbook last | head-from-arm-board-solver, arm-board-pose-set, calib-docs-and-test-index, calib-robot-runbook |
| D44 The arm joint offset solve moves to a follow-up epic | calib-docs-and-test-index |
| D45 The existing 6x5 board is the gripper board | calib-config-and-view-types, board-on-gripper-hardware-st |
| D46 Guided step-by-step process | head-arm-calibrate-ui-sectio, head-arm-pose-capture-sequen |
| D47 The board mount is fitted; the measurement is only a start value | head-from-arm-board-solver, arm-board-pose-set, calib-robot-runbook |
| D48 User 2026-10-22 approved (yes): fold pan zero into the head_ | head-from-arm-board-solver, head-arm-compute-and-apply |
| D49 User 2026-10-22 approved (yes): floor-fit values as start va | head-from-arm-board-solver, head-arm-compute-and-apply |
| D50 User 2026-10-22 approved (yes): clip board only after base p | head-arm-pose-capture-sequen, head-arm-calibrate-ui-sectio |
| D51 User 2026-10-22 approved (yes): solver fits the board-on-jaw | arm-board-pose-set, head-from-arm-board-solver |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `ground-recording-layout` | ground-recording-layout | web-ui-ground-recorder-api, replay-format-v2 | `nodes/web_ui/web_ui/ground_recorder.py` |
| `ground-recorder-api` | web-ui-ground-recorder-api | ground-record-script | `nodes/web_ui/web_ui/server.py` |
| `record-script` | ground-record-script | capture-baseline-recordings, capture-gated-recordings, guidance-live-check-and-cpu | `scripts/ground_record.py` |
| `replay-format-v2` | replay-format-v2 | guidance-replay-metrics, capture-baseline-recordings | `scripts/ground_replay_export.py` |
| `scene-truth` | scene-truth-from-board | guidance-replay-metrics, capture-baseline-recordings, live-report-guidance-metrics | `scripts/ground_scene_truth.py` |
| `guidance-replay-harness` | guidance-replay-metrics | capture-baseline-recordings, capture-gated-recordings, guidance-replay-acceptance | `nodes/ground_estimator_cpp/test/replay_fixture.hpp` |
| `baseline-fixtures` | capture-baseline-recordings | tof-amplitude-gate, capture-gated-recordings, live-report-guidance-metrics | `nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml` |
| `amplitude-gate` | tof-amplitude-gate | capture-gated-recordings | `nodes/bridges/arducam_tof/arducam_tof/amplitude.py` |
| `gated-fixtures` | capture-gated-recordings | guidance-replay-acceptance | `nodes/ground_estimator_cpp/test/data/replay_rgbd/` |
| `upsampled-depth-on` | rgbd-upsampling-on | guidance-config-keys, capture-baseline-recordings | `ansible/group_vars/client.yml` |
| `guidance-config` | guidance-config-keys | guidance-frame-core, edge-points-core, no-return-core, guidance-node-wiring, guidance-replay-metrics | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp` |
| `guidance-frame` | guidance-frame-core | edge-points-core, no-return-core, guidance-node-wiring | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp` |
| `edge-points` | edge-points-core | guidance-node-wiring | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edge_points.hpp` |
| `no-return-tracker` | no-return-core | guidance-node-wiring | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp` |
| `nullable-area-heights` | area-heights-nullable | no-return-core, guidance-node-wiring | `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp` |
| `guided-node` | guidance-node-wiring | guidance-replay-acceptance | `nodes/ground_estimator_cpp/src/ground_node.cpp` |
| `live-report` | live-report-guidance-metrics | guidance-live-check-and-cpu | `scripts/ground_noise_report.py` |
| `flying-pixel-retune` | tof-flying-pixel-retune | tof-filter-live-check | `nodes/bridges/arducam_tof/arducam_tof/config.py` |
| `live-scene-estimator` | ground-estimator-live-scene | ground-live-acceptance | `nodes/ground_estimator_cpp/test/replay_fixture.hpp` |
| `live-acceptance` | ground-live-acceptance | ground-voxel-layer-on | `docs/runbooks/ground-robustness.md` |
| `ground-layer-on` | ground-voxel-layer-on | nav2-goal-across-the-scene | `nodes/nav2_bringup/config/nav2_params.yaml` |
| `head-arm-calib-config` | calib-config-and-view-types | head-from-arm-board-solver, arm-pose-safety-check, joint-state-still-capture | `shared/ros2_common/head_rgbd_calibration/config.py` |
| `head-arm-view-types` | calib-config-and-view-types | head-from-arm-board-solver, session-kinds-joint-poses | `shared/ros2_common/head_rgbd_calibration/views.py` |
| `head-arm-fit` | head-from-arm-board-solver | head-arm-compute-and-apply, arm-board-pose-set | `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py` |
| `head-aim` | head-from-arm-board-solver | arm-board-pose-set, head-arm-pose-capture-sequen | `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py` |
| `head-arm-synthetic-scenes` | head-from-arm-board-solver | head-arm-compute-and-apply, arm-board-pose-set | `tests/head_arm_synthetic.py` |
| `arm-pose-check` | arm-pose-safety-check | arm-board-pose-set, head-arm-pose-capture-sequen | `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py` |
| `calib-session-kinds` | session-kinds-joint-poses | joint-state-still-capture, head-arm-compute-and-apply | `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py` |
| `calib-still-capture` | joint-state-still-capture | head-arm-pose-capture-sequen | `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py` |
| `calib-robot-client` | robot-mcp-client | head-arm-pose-capture-sequen, head-arm-sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py` |
| `arm-sequence-config` | arm-board-pose-set | head-arm-pose-capture-sequen, calib-ansible-deploy-config | `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` |
| `calib-sequencer` | head-arm-pose-capture-sequen | head-arm-sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py` |
| `calib-head-arm-results` | head-arm-compute-and-apply | head-arm-sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py` |
| `head-arm-calib-api` | head-arm-sequence-http-api | calib-frontend-api, head-arm-calibrate-ui-sectio | `nodes/head_rgbd_calibration/tests/fixtures/api/` |

Cross-epic contracts (`docs/plans/contracts.json`): owned here are `tof-depth-amplitude-gate`,
`ground-warning-no-return`, `ground-edge-points`, `ground-recording-npz` and `head-rgbd-guidance-pair`. Used here are
`tof-depth-flying-pixel-filter`, `ground-traversability-rule`, `head-rgbd-points`, `tof-points-filtered`,
`head-camera-stream-rate` and `tof-depth-z`. Owned here since the umbrella re-plan: `tof-flying-pixel-retune`. Used here since then: `nav2-tof-voxel-bands`.

Cross-epic (`docs/plans/contracts.json`): this epic uses `head-chain-override` (`urdf_text.py`), `urdf-chain-fk`
(`chain.py`) and `head-chain-applied-record` (`floor_results.py`; the record gains `source` and `floor`, compatible)
from head-chain-floor-plane-fit-a, and `head-rgbd-applied-calibration` (the `applied/` directory and its intrinsics).
It owns `head-arm-fit`, `head-arm-calib-api`, `calib-session-kinds` and `arm-pose-check` for the follow-up epic
arm-joint-offsets-from-the-g.

## Review focus

- edge-points-core / guidance-node-wiring: no path where a guidance point becomes lethal without a stable depth
  lethal point within max_shift_m in the same frame, and the existing lethal points and their latency are untouched.
- no-return-core: never lethal, null heights, clearance next to depth regions, hysteresis that keeps churn < 10 %.
- guidance-frame-core: undistortion and projection conventions (pixel centres, plumb_bob D, TF direction
  base_T_cam vs cam_T_base), and the ToF coverage test.
- tof-amplitude-gate: retention of the far tilted floor; this change reaches every ToF consumer (head floor fit,
  RGB-D calibration, Nav2 ToF voxel layer).
- web-ui-ground-recorder-api: a message without TF is dropped, never recorded with a guessed pose; subscriptions are
  destroyed after the window; nothing is published.
- The replay assertions match the approved numbers; no threshold is loosened to pass.
- ground-estimator-live-scene: the replay reproduces the live failures before any fix; no threshold, ROI or warm-up
  is changed to pass; the rule and the lethal latency are unchanged; run 4's fixtures still pass.
- tof-flying-pixel-retune: the floor retention is measured on real frames per range band; the change reaches every
  ToF consumer; 0 pixels stay 0 and never invalidate neighbours.
- ground-voxel-layer-on: only `enabled` changes, and only after every live Results row passed.
- nav2-goal-across-the-scene and every live task: the user's presence and the static-robot rules are preconditions,
  not notes.
- The solver: the D32 parameter set (a free degenerate parameter makes sigma explode; a fixed one that should be free
  biases the result), the pan-zero folding (D33), held-out split by arm pose, sigma scaling, the floor cross-check.
- Frames: URDF rpy is fixed-axis xyz (`urdf_handoff.RPY_SEQUENCE`), head_tilt + is down, optical frames z forward; arm
  URDF root vs robot base_link via `base_in_base_link`; `urdf = measured + offset`; the pose set's joint convention
  equals what `move_arm_joints` takes.
- Arm safety: no command before its path check, the gripper sent once, the confirms only from the UI dialogs, every exit
  holds (keep_position true), Park only after the board-removed confirm, the head envelope really contains the head.
- Every consumer reading `head_chain.json`, and an unusable override falling back to the URDF with a warning.
- No placeholder data: a missing joint sample or detection means no view, never zeros.

## Left to the builder

- Defaults of the guidance keys inside their validated ranges, chosen on the replay (guidance-replay-acceptance).
- The guided filter's radius/eps/min_support within the existing ranges (rgbd-upsampling-on).
- Whether the amplitude gate alone meets G2, or needs the amplitude-dependent correction of D7 as well
  (tof-amplitude-gate).
- The internal structure of ground_recorder.py and the bridge wiring, within the routes and layout stated.
- The 10 s / 15 s trim windows inside the 3 MB cap.
- The retuned flying-pixel rule and keys within D26 (tof-flying-pixel-retune).
- Which estimator modules change to meet D23 (ground-estimator-live-scene), within the rule and the latency pin.
- The exact goal poses of the Nav2 check within D25 (at most 1.5 m, around the box).
- The exact base pose, pose values and head offsets of the shipped pose set, within D38/D39 and the tests that pin
  clearance, visibility, coverage and a passing synthetic fit.
- The internal parameter vector layout, `x_scale`, the Huber `f_scale` default (1 px) and `max_nfev`.
- UI layout inside the section, following HeadFloorFit.

## Assumptions and unknowns

- The board bias is amplitude-related (dark ink). Checked by tof-amplitude-gate's first step on the recorded frames;
  if not, the D7 correction path applies.
- The guided upsampling does not copy texture into depth on the board. Pinned synthetically by rgbd-upsampling-on and
  on real data by guidance-replay-acceptance (no lethal point on the board).
- The robot stays static and charging between capture-baseline-recordings and capture-gated-recordings, and the
  scene stays in place. capture-gated-recordings re-measures the board pose and stops when it moved more than 5 mm.
- The mapping RGB stream and the ToF field of view overlap enough at the scene poses to see the box, the board and
  the ramp in one view. Confirmed when the head poses are chosen in capture-baseline-recordings.
- Recording size in /tmp on the client: about 20-30 MB per 30 s scene, kept at most `max_files`, deleted after
  download.
- The gated replay reproduces the live failures of 2026-10-22: checked first by ground-estimator-live-scene against
  the live `.ndjson` of the same recording; a mismatch is fixed in the harness before any estimator fix.
- The 1.9-2.2 deg plane-vs-base tilt is partly head geometry: measured again after the head chain is applied (D21);
  the remainder is named by ground-estimator-live-scene.
- The ToF frame rate and the 43,200-pixel frame size used for the live removed share come from the deployed
  arducam_tof config: read there, not assumed.
- The robot can be undocked and driven for two short goals (battery and tilt gates of mcp_server pass): checked in
  nav2-goal-across-the-scene; a refusal is reported, not worked around.
- `move_arm_joints` targets use the `get_arm_state` convention; whether that is measured or URDF angles is probed by
  arm-board-pose-set in `nodes/mcp_server/mcp_server/arm.py` and pinned by a test.
- The board on the jaw decodes at 0.25-0.65 m on the 576 px stream and the arm carries it at speed_scale <= 0.3: checked
  in the runbook with tier 0 first.
- The node's service user can read `/etc/ros2/mcp_server/token` with group `mcp-token`: checked in
  calib-ansible-deploy-config and on the robot.
- Arm moves while docked and charging pass the mcp battery gate: checked in the runbook; a refusal is reported, not
  worked around.
- The front camera stream has the same aspect ratio as the mapping stream: probed in mcp-front-camera-applied.

## Open questions

- Q1: Before capture-baseline-recordings, can you set the scene: robot static and charging, the 5 cm box with its front face on the far edge of the ChArUco board, the black ramp's long side against one side edge of the board, an empty floor area in head reach; give the box top length x width, the ramp width and which side; and leave it all in place until capture-gated-recordings is done? [answer: yes (D16): user sets and keeps the scene; box/ramp dimensions asked at capture time; the placements are measured against the board (D22)]
- Q2: Keep the head mapping RGB stream at 2 Hz, so edge points and no_return areas refresh twice a second and are moved through odom in between, instead of raising it (more client CPU, contract head-camera-stream-rate)? [answer: yes (D17): keep head mapping RGB at 2 Hz]
- Q3: Refine only lethal boundaries (obstacles, drops) with the RGB edges, and keep the non-lethal warning areas on their 10 cm cell polygons so warning churn stays under 10 %? [answer: yes (D18): RGB edges refine only lethal boundaries]
- Q4: Apply the amplitude gate to /tof/depth/image_raw for every consumer (Nav2 ToF layer, head floor fit, RGB-D calibration), as the flying-pixel filter is, rather than only on the ground estimator's path? [answer: yes (D19): amplitude gate on /tof/depth/image_raw for every consumer]
- Q5: Fold the pan zero into the head_mount_joint yaw in head_chain.json (TF and head_look_at exact; a raw pan 0 in the Head tab stays off by the fitted yaw, expected under 3 deg) instead of re-zeroing the pan servo? [answer: yes (D48, HQ1): fold pan zero into the head_mount_joint yaw]
- Q6: Use the floor-fit values as start values and a pass/fail cross-check (10 mm on mount z, 1.5 deg on roll, pitch, tilt zero, tilt roll) with the weighted prior off by default, since the floor fit carries the garage floor slope? [answer: yes (D49, HQ2): floor-fit values as start values and pass/fail cross-check, weighted prior off]
- Q7: Clip the board only after the arm holds the base pose, hold the arm in place on every stop or error, and park only after you confirm the board is removed (no automatic park with the board on)? [answer: yes (D50, HQ3): clip board only after base pose, hold arm on every stop, park only after board removal confirmed]
- Q8: Let the solver fit the board-on-jaw pose (standard hand-eye: target-in-gripper unknown) with your measurement only as a start value, so the pose set and the solver do not wait for the hardware step and the poses stay provisional until a dry run without the board? (no: arm-board-pose-set waits for the hardware step) [answer: yes (D51, HQ4): solver fits the board-on-jaw pose, measurement only a start value, poses provisional until a dry run]
- Q9: Run the head-from-arm calibration live step before recording the ground scene, so every recording and replay carries the calibrated head chain? If the scene's ChArUco board is the board you clip to the gripper, it leaves the scene for the calibration and you put it back and re-measure the box and ramp placements against it before the baseline recording (D21). [recommended: yes, calibrate the head first]

## Irreversible operations in scope

none (fixtures, configs and runbooks are committed and revertible; recordings live in /tmp on the client and
`.logs/` on the Mac; the head-from-arm Apply writes `applied/` files that the next apply or a re-seed replaces, the
previous chain kept in its result directory; the Nav2 check drives two short goals with the user present and deletes
nothing; nothing is force-pushed).
