# RGB and amplitude guided ground edges and no-return areas design

## Summary

Ground estimation keeps depth as its only height source. Two image streams that already exist get used for more:
the ToF amplitude image and the head RGB stream registered to the ToF by head_rgbd_cpp. They decide which depth pixels
to trust, where the boundaries of lethal regions are, and where depth is missing although something is there.
Headline decisions: an amplitude gate in arducam_tof (D7); RGB edges through the existing guided-filter upsampling in
head_rgbd_cpp, which is switched on (D2); "edge points" that sharpen lethal obstacle and drop boundaries in
ground_estimator_cpp, only next to regions that depth already found lethal (D3); a "no_return" caution warning area
where the RGB sees non-floor and the ToF returns nothing (D5). Before anything changes, new recordings with RGB +
amplitude are captured through web_ui and give a baseline (D8, D14). Ground truth comes from the ChArUco board pose
(D10). Every step is judged on replay against that truth.

## Intent and why

User request 2026-10-22: "keep depth computation from depth stream, but empower edges and areas detection with RGB
stream, so it's more precise". Today the ground estimator only sees the decimated ToF cloud (240x180 decimated by 4,
5 cm voxels, 10 cm temporal cells), so:
- lethal edges land about 5 cm off;
- mixed ToF pixels make fake ramps behind objects;
- the printed ChArUco board reads -4..+3 cm (albedo bias);
- the black rubber ramp gives no ToF returns at all and looks like free floor.

The robot navigates on that costmap (2.5 cm resolution), so a 5 cm edge error and an invisible black obstacle are both
real driving risks. The approved order is:
1. a baseline after ground-detection-robustness run 4;
2. amplitude confidence;
3. RGB-guided edges and no-return areas.

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

## Non-goals

- Learning-based segmentation.
- Heights from RGB (no monocular depth).
- Graded traversability cost (declined by the user 2026-10-22).
- A higher head mapping stream rate (stays 2 Hz, contract head-camera-stream-rate, Q2).
- Refining the polygons of non-lethal warning areas (they stay cell-aligned, D4, Q3).

## Anti-goals

- A lethal mark that only RGB supports, or an obstacle on a texture edge (board squares, tile seams, shadows).
- Lethal output that is removed or delayed by RGB: the lethal latency of ground-detection-robustness
  (`hysteresis.enter_frames` frames) is unchanged.
- Placeholder data on any topic: no guessed height on a no_return area, no guessed pose on a recorded sample, no
  depth invented inside a large hole.
- Warning churn above 10 % because of the new areas.
- An rclpy probe or SSH session on the client to get live data.
- A new node.

## Constraints

- CLAUDE.md: C++ for robot nodes except web_ui and mcp_server; Python type hints, docstrings with types, constants on
  top, pydantic configs with unknown keys rejected; config files over CLI; Ansible mirrors every node config change
  (`ros2_nodes` schema); READMEs and `tests/README.md` updated; no placeholder data on topics.
- No new nodes (user). Live data only through the web_ui websocket/API (user). Recordings: robot static and charging,
  head moves allowed (user).
- The free / warning / lethal tiers and the rule (step > 0.04 m or slope > 15 deg) are unchanged (user).
- This epic starts only after ground-detection-robustness has merged. It is the README blocker. Checked on main
  2026-10-22: none of its run 4 artifacts exist yet. They come from these run 4 tasks of that epic:
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

## Research

| Lens | Ran / skipped | Adopted | Rejected |
|---|---|---|---|
| Codebase (planner, grep/read) | ran | guided_upsample exists and is off (`nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`); upsampled depth is published at the RGB size with one header per pair (`nodes/head_rgbd_cpp/src/head_rgbd_node.cpp:publish_pair`); amplitude is already pixel-aligned mono16 (`nodes/bridges/arducam_tof/arducam_tof/bridge.py` AMPLITUDE_TOPIC); FloorColourModel (`nodes/ground_estimator_cpp/include/ground_estimator_cpp/colour.hpp`); ChArUco detect (`shared/ros2_common/head_rgbd_calibration/charuco.py:detect`); web_ui already has a TF buffer and on-demand subscriptions (`nodes/web_ui/web_ui/bridge.py`) | rosbag2 on the client (DDS participant); websocket image capture (8-bit JPEG previews, `nodes/web_ui/web_ui/msg_serializer.py`) |
| Live brief (ground-live.md, 2026-10-22) | ran | mixed pixels, board albedo bias, no returns on the black ramp, gripper zone x < 0.37 excluded | ToF cell-median ground truth (5 cm uncertainty) |
| External library docs | skipped: no new library API. OpenCV imdecode, undistortPoints and solvePnP are standard, and the guided filter is in-repo | - | OpenCV ximgproc guidedFilter (the in-repo masked version handles holes; ximgproc does not) |
| Prior art (guided depth upsampling) | ran from the in-repo design (head-rgb-d-fusion-of-camera D8) | joint guided upsampling aligns depth edges to image edges (He, Sun, Tang, "Guided Image Filtering", TPAMI 2013) | learned depth completion (non-goal) |

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

Cross-epic contracts (`docs/plans/contracts.json`): owned here are `tof-depth-amplitude-gate`,
`ground-warning-no-return`, `ground-edge-points`, `ground-recording-npz` and `head-rgbd-guidance-pair`. Used here are
`tof-depth-flying-pixel-filter`, `ground-traversability-rule`, `head-rgbd-points`, `tof-points-filtered`,
`head-camera-stream-rate` and `tof-depth-z`.

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

## Left to the builder

- Defaults of the guidance keys inside their validated ranges, chosen on the replay (guidance-replay-acceptance).
- The guided filter's radius/eps/min_support within the existing ranges (rgbd-upsampling-on).
- Whether the amplitude gate alone meets G2, or needs the amplitude-dependent correction of D7 as well
  (tof-amplitude-gate).
- The internal structure of ground_recorder.py and the bridge wiring, within the routes and layout stated.
- The 10 s / 15 s trim windows inside the 3 MB cap.

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

## Open questions

- Q1: Before capture-baseline-recordings, can you set the scene: robot static and charging, the 5 cm box with its front face on the far edge of the ChArUco board, the black ramp's long side against one side edge of the board, an empty floor area in head reach; give the box top length x width, the ramp width and which side; and leave it all in place until capture-gated-recordings is done? [recommended: yes; the truth comes from the board pose, and the capture tasks stop if the board moved more than 5 mm]
- Q2: Keep the head mapping RGB stream at 2 Hz, so edge points and no_return areas refresh twice a second and are moved through odom in between, instead of raising it (more client CPU, contract head-camera-stream-rate)? [recommended: yes, keep 2 Hz]
- Q3: Refine only lethal boundaries (obstacles, drops) with the RGB edges, and keep the non-lethal warning areas on their 10 cm cell polygons so warning churn stays under 10 %? [recommended: yes]
- Q4: Apply the amplitude gate to /tof/depth/image_raw for every consumer (Nav2 ToF layer, head floor fit, RGB-D calibration), as the flying-pixel filter is, rather than only on the ground estimator's path? [recommended: yes; one depth stream, and the floor-retention test guards the floor fit]

## Irreversible operations in scope

none (fixtures and configs are committed and revertible. Recordings live in /tmp on the client and `.logs/` on the
Mac. No data is deleted outside them, and nothing is force-pushed).
