# RealSense D435 head switchover design

## Summary

The Intel RealSense D435 (serial 935723024187, USB 8086:0b07) on the pan/tilt head becomes the robot's only head
RGB-D sensor and the only camera. It runs on the client as the apt `realsense2_camera` node (`/head/d435/*`, frames
`d435_*`), owned by Ansible. Its depth feeds a renamed obstacle cloud node, laser_filter, Nav2 and ground_estimator_cpp.
Its aligned depth plus colour feed head_rgbd_cpp. That node is reduced to an adapter that keeps the `/head_rgbd/*`
contract for RTAB-Map, ground guidance, marker_detector and web_ui. MCP keeps the camera name `front` for the D435 colour
and loses `gripper`. The calibration screen is re-based on the D435. Arm joint offsets and the tool point are re-solved
from the head D435 watching a gripper-held ChArUco board. The head ToF, the head Camera Module 3 and the gripper UVC
camera are retired, together with their consumers and ToF-only code.

## Intent and why

User request (verbatim): "start implementation for the switchover, assume similar position as current head unit.
restore ground estimation and other 3d view functionality, replace existing gripper position estimation and MCP tools.
it's a bit epic including doing necessary recalibration, so it might also touch the calibration screen." Context: "It
will permanently replace gripper camera (eventually), but will be mounted on separate head."

Ground estimation, the Nav2 head inputs and the 3D view were switched off on 2026-10-24 "until a RealSense D435 replaces
the head ToF". The D435 is now on the robot, so those features come back on it. The old sensors go, so the robot keeps
one consistent sensor model.

## Goals

The README goals, each covered by the tasks in Decision coverage:

- G1 D435 runs on the client as realsense2_camera, deployed by Ansible, with udev for 0b07, a matched-fps profile,
  explicit camera_name/namespace and the live topic list pinned in config.
- G2 D435 on the pan/tilt head chain in head.urdf / head_state_publisher, at a position similar to the old head unit,
  replacing the ToF and head RGB frames.
- G3 Ground estimation restored on D435 depth: ground_estimator_cpp and head_rgbd_cpp enabled, obstacle cloud and
  self-filter re-pointed, replay fixtures re-recorded.
- G4 Nav2 head inputs disabled on 2026-10-24 re-enabled on the D435 cloud.
- G5 Every web_ui 3D view feature (Head RGB-D layer, Ground layer, ToF obstacles layer, Head tab RGB-D view) shows D435
  data.
- G6 Arm joint offset / tool point solve re-done from the head D435 watching a gripper-held ChArUco board.
- G7 Every MCP tool using the gripper camera, head RGB, ToF or RGB-D is D435-backed; claude_agent prompt, skills and
  evals match.
- G8 Recalibration on the D435: head mount extrinsics through the pan/tilt chain, floor fit, intrinsics check against
  factory; the web_ui calibration screen drives it.
- G9 Head ToF, head RGB and gripper UVC camera retired (present: false) with their consumers. ToF-only code paths and
  relays are removed or re-pointed. marker_detector moves to D435 colour+depth.
- G10 READMEs, tests/README.md and MEMORY.md reflect the D435; all lint and tests green; deployed to the client.

## Non-goals

- Physical mounting or printing of the D435 head.
- Moving web_ui or mcp_server off Python.
- Learned or markerless gripper tracking (DREAM, URDF-mesh ICP) or a live gripper-tracking node.
- RTAB-Map 3D mapping changes beyond re-pointing its source topics.
- Retiring the lidar or changing lidar-based planning.
- New MCP capabilities (for example a per-pixel depth tool). Existing tools are re-based; none is added.

## Anti-goals

- Raw D435 images crossing WiFi (the Pi 5 fell to 4-5 Hz over WiFi in prior art; UDPv4 fragment drops in-tree).
- Placeholder data on any topic while the D435 is starting, missing or dropping frames (CLAUDE.md rule).
- The enabled memory_low sum above the 2560 MiB cap of tests/test_unit_cgroups.py, or the client CPU saturated by
  host-side align.
- A second copy of the D435 factory extrinsics in several configs (one source: head.urdf).
- Servo motion beyond the least-dramatic rule in any live step without the user watching.

## Constraints

- CLAUDE.md: TDD, type hints, pydantic config, file config over CLI, no placeholder topic data, the `ros2_nodes`
  schema, deploy only via `scripts/deploy-nodes.sh` with logs in `.logs/`, READMEs and tests/README.md updated with the
  code, servo least-dramatic-movement rule.
- Memory notes: no ad-hoc rclpy probes on the client (use the web_ui websocket, topic_scraper_api, Mimir/Loki). Deploying
  mcp_server/claude_agent kills a running agent session. Arm motion safety (wrist roll only half-open and lifted;
  negative elbow stretches; FK-check poses).
- Installed on the Pi: ros-jazzy-realsense2-camera 4.58.4, librealsense2 2.58.4 (apt, V4L2 backend). Firmware must
  be >= 5.17.3.10.
- Pi 5 USB budget: a 5 V 5 A supply (1.6 A) is needed for the ~0.7 A D435.
- Grep guards: tests/test_head_camera_live_refs.py forbids `front_camera` names and `640x480`/`10 fps` near
  "front"/"head" in mcp_server, claude_agent and READMEs unless "calibrat"/"intrinsic" is nearby. The guard
  nodes/web_ui/tests/test_stale_names_gone.py forbids `image_rect` in frontend .ts/.tsx; topic names live in config
  only.
- Cross-epic file overlap: the in-progress ToF epics are this epic's blockers (D15).

## Decisions

### D1: head_rgbd_cpp stays as the D435 adapter stage; consumers keep the /head_rgbd/* contract

Decision: head_rgbd_cpp subscribes to the D435 aligned depth (`/head/d435/aligned_depth_to_color/image_raw`, 16UC1 mm
in the colour optical frame), the raw colour image and the colour camera_info, all on the client. It pairs them by exact
stamp, throttles to `output.rate_hz` (2.0 Hz, the old mapping rate), downscales depth by `output.divisor` and JPEG-encodes
the RGB. It publishes `/head_rgbd/{rgb/image_raw/compressed, rgb/camera_info, depth/image_raw, depth/camera_info,
points}` with the colour header. Removed as ToF-only: ToF registration, guided upsampling, the flying-pixel filter, the
motion compensation and the ToF intrinsics file. `/head_rgbd/depth_upsampled/*` is retired, and ground guidance reads
`/head_rgbd/depth/*`.
Alternatives: (a) consumers read `/head/d435/*` directly. Rejected: each of RTAB-Map, ground guidance, marker_detector
and web_ui would need its own throttle, downscale and compression, and the server's RTAB-Map would pull raw images over
WiFi. (b) A thin relay (image_transport republish plus topic_tools throttle). Rejected: several processes, no
exact-stamp set guarantee, no cloud.
Why: it keeps one client-side owner of the cross-host RGB-D contract (map3d_topics.yaml), and deletes code instead of
adding it. (my call)

### D2: D435 profile: depth 640x360 @ 15, colour 640x360 @ 15, align on, pointcloud and infra off

Decision: `depth_module.depth_profile: 640x360x15`, `rgb_camera.color_profile: 640x360x15` (one fps, so RGB-depth
hardware sync holds), `align_depth.enable: true`, `pointcloud.enable: false`, `enable_infra1/2: false`, no IMU
arguments (the D435 has none), `publish_tf: false` (D4). Parameters live in
`nodes/bridges/realsense_d435/config/realsense_d435.yaml`, passed as `config_file:=`. Launch arguments are
`camera_namespace:=head camera_name:=d435 serial_no:=_935723024187`.
Alternatives: 848x480x90 + 1280x720x30 (today's line): no HW sync, align at 1280x720 on the Pi CPU, 90 fps nobody
consumes. 848x480x30: min-Z 195 mm, about 2.4x the align and cloud work. 480x270: min-Z 120 mm, but too coarse for
ChArUco and for the agent's images. Align off: head_rgbd_cpp would need its own registration, which D1 deletes.
Why: min-Z is 150 mm at 640x360, which keeps the gripper-held board (0.2-0.5 m) and the floor (0.3-2 m) in range. 15
Hz is the legal fps nearest the 10 Hz the depth consumers were sized for. Align output at 640x360 is the cheapest
aligned pair that still gives the agent a usable image. Measured, not assumed: the bring-up and budget tasks gate on
>= 14 Hz depth under full load and on the realsense CPU from Mimir. (my call)

### D3: names: namespace head, camera d435, node type realsense_d435

Decision: topics `/head/d435/...`, frames `d435_link`, `d435_depth_frame`, `d435_depth_optical_frame`,
`d435_color_frame`, `d435_color_optical_frame`. The Ansible node type and instance become `realsense_d435`, and the bridge
dir becomes `nodes/bridges/realsense_d435`. The old `realsense_d435i` entry was for another product id and an IMU; it is
replaced, not kept. The udev rule moves from ros2_base (applied only by the base playbook, which CLAUDE.md forbids running
by hand) into the node's own deploy step, so `scripts/deploy-nodes.sh client realsense_d435` applies it.
Alternatives: default `camera/camera` (collides with `camera_link` documented by static_tf_publisher). Keeping the
D435i name (it caused the 0b3a-only udev bug and the stale IMU docs).
Why: explicit names stop any TF collision. Product-correct names stop the next PID or IMU mistake. (my call)

### D4: head.urdf owns every D435 frame; the driver publishes no TF

Decision: head.urdf gains `d435_joint` (head_link -> d435_link, fixed, the calibrated mount, CAD start at the old head
unit's centre `0.02 0 0`). It also gains the fixed `d435_depth_joint`, `d435_depth_optical_joint`, `d435_color_joint` and
`d435_color_optical_joint`. The depth frame is the identity at the left imager plus the optical rotation. The colour
frame is the factory depth-to-colour extrinsic, captured once at bring-up from `rs-enumerate-devices -c`.
`publish_tf: false` on the driver. HEAD_CHAIN_JOINTS (calibration overrides) gains `d435_joint` in place of
`head_camera_joint`. The old head frames stay in the URDF until their last consumer moves, and are then removed by
`retire-tof-and-head-rgb-urdf`.
Alternatives: the driver publishes `d435_link -> optical` TF and the URDF stops at `d435_link`. Rejected: mcp_server
HeadKinematics, the calibration UrdfChain and the web_ui head model all do FK from head.urdf and would each need a
copy of the colour extrinsic.
Why: one source of truth for every frame that consumers compute FK through. (my call)

### D5: obstacle cloud from the full-FOV depth; node renamed d435_obstacle_cloud

Decision: rtabmap_util `point_cloud_xyz` on `/head/d435/depth/image_rect_raw` + `/head/d435/depth/camera_info` publishes
`/head/d435/points` in `d435_depth_optical_frame`. The decimation and voxel are re-derived for 640x360. The node is
renamed `d435_obstacle_cloud` and the old instance becomes `tof_obstacle_cloud present: false`. laser_filter turns it
into `/head/d435/points_filtered`.
Alternatives: realsense `pointcloud.enable` (full resolution on the Pi CPU, and coupled to the aligned stream).
depth_image_proc (a new dependency for what point_cloud_xyz does already). The aligned depth (cropped to the 69x42 colour
FOV, which loses the floor edges).
Why: proven config-only path, full 87x58 depth FOV, no new code. (my call)

### D6: Nav2 head inputs re-enabled on the D435 cloud and /ground/*

Decision: every line commented `disabled (user 2026-10-24): lidar-only ...` in nav2_params.yaml is re-enabled. The ToF
voxel layer, the collision monitor sources and CliffStop read `/head/d435/points_filtered`. The ground voxel layer and
the KeepoutFilter read `/ground/*`. Ranges and heights are re-derived from the D435 (min-Z 0.15 m, the cloud's 3 m max).
tests/test_nav_lidar_only.py is deleted because the lidar-only state ends (that test is no longer accurate). Its
replacement, tests/test_nav_d435_inputs.py, pins the head inputs on D435 topics. Source: README goal "(my call: the
lidar-only decision was conditional on this)".

### D7: ground_estimator_cpp on D435: re-pointed inputs, a sensor-neutral no-return model, re-tuned on D435 replays

Decision: `cloud_topic: /head/d435/points_filtered`. Guidance reads `/head_rgbd/depth/{image_raw,camera_info}` and
`/head_rgbd/rgb/image_raw/compressed`. `guidance.tof_info_topic` is renamed `guidance.depth_info_topic` and set to
`/head/d435/depth/camera_info`, and `no_return.tof_margin_px` becomes `no_return.margin_px`. Every frame-count window
(temporal, hysteresis, no_return) is scaled by 1.5 and rounded up, so its time span at 15 Hz matches the old 10 Hz span.
The ToF replay fixtures (home, box, ramp) are replaced by D435 recordings of the same scenes. The ToF-tuned noise limits
are re-tuned until the existing replay assertions (box lethal, board not lethal, no phantom drops) pass on the D435
data.
Alternatives: keep the ToF names (they would lie about the sensor). Keep the frame counts (the windows would shrink by a
third and change hysteresis behaviour).
Why: same estimator (prior art: stronger than Nav2/laserscan alternatives), honest config, acceptance on real D435 data.
(my call)

### D8: MCP: `front` is the D435 colour, `gripper` is gone, intrinsics from camera_info

Decision: `topics.head_camera: /head/d435/color/image_raw/compressed`. `cameras.front` hangs on `d435_color_frame`
with a zero mount (FK through head.urdf, D4), and its intrinsics come from `/head/d435/color/camera_info` through
`CameraIntrinsics.from_camera_info`. The `gripper` camera, its topic, mount, intrinsics file and every
`camera == "gripper"` branch are removed. One `Camera = Literal["front"]` definition replaces the three copies. The
`camera` parameter stays on the tools, so tool schemas, stored lookup camera names, prompts and workflows keep their
shape. Grasp verification crops the jaw from the front image, with the head aimed at the FK tool point. `get_map_summary`
`obstacles_3d` reads the D435 cloud (`topics.depth_points`). The head tool descriptions name the D435.
Alternatives: rename the camera to `head` (touches every prompt, workflow and stored lookup for no behaviour change).
Drop the `camera` parameter (larger schema change for the agent).
Why: smallest honest API change. The `front` name has been the head camera since the head existed. (my call)

### D9: gripper position estimation: FK stays the estimator; offsets and tool point re-solved from the head D435

Decision: the absorbed epic arm-joint-offsets-from-the-g (user decision 3) supplies three tasks, re-planned here:
`arm-offsets-plate-solver`, `arm-offsets-compute-apply` and the Arm section in `calibrate-head-and-arm-ui-se`. Camera =
D435 colour with factory intrinsics. There is no tof_joint.json. The plate solver also returns `tool_offset_m` (Q2).
Results are accepted only on held-out poses at several head pan/tilt settings (reprojection px and FK tool-point mm),
never on fit residual alone (prior-art fix-as-ac). The board is the rigid 6x5 ChArUco, beyond min-Z. The output is a
reviewed `joint_offsets_rad` / `tool_offset_m` snippet committed to client.yml, never a runtime override.
Alternatives: robot_calibration (Jazzy; fits the same problem, but would replace the in-tree session store, sequencer and
UI that already exist); easy_handeye2 / MoveIt calibration (camera fixed to the base, weak on SO-101 class arms).
Why: the in-tree head_arm_fit, PoseChecker, sequencer and UI already implement the pan/tilt-head-plus-gripper-board
formulation. (my call)

### D10: calibration re-based on the D435; factory intrinsics and extrinsics stay authoritative

Decision: the RGB-D (RGB + ToF stereo) session becomes a "D435 intrinsics check". It captures still colour + aligned
depth pairs of the ChArUco board, fits the colour intrinsics, and compares them with the factory camera_info
(`intrinsics.compare_sdk`). It also measures the board-plane depth bias from the aligned depth. The report passes or
fails against limits; Apply writes no intrinsics (the factory values stay in use). Head floor fit runs on
`/head/d435/points` in `d435_depth_optical_frame`. Head-from-arm runs on the D435 colour. The applied files
tof_calibration.yaml, head_camera_calibration.yaml, head_rgbd_extrinsics.yaml and tof_joint.json are retired, and the
apply restart units become head_rgbd_cpp + head_state_publisher. ToF-only maths (extrinsics.py stereo, urdf_handoff.py,
calibrate_tof, detect_tof) and the ToF+CM3 offline CLI (scripts/head_rgbd_calibrate.py) are deleted. On-chip
self-calibration (OCC) is not run unless the intrinsics check fails.
Alternatives: apply fitted intrinsics over the factory ones (adds a second intrinsics source to every consumer for a
camera that is factory calibrated). Run OCC at bring-up (needs the node stopped and Viewer/API tooling; the docs say it
"may indeed never be required").
Why: the check proves the factory values on this unit, and nothing new has to be kept in sync. (my call)

### D11: retirement: present: false, then consumers, then sources

Decision: arducam_tof, head_camera_cpp and gripper_uvc_camera become `present: false` (node types stay as uninstall
stubs, the repo pattern). This happens only after every consumer is re-pointed. Then their source dirs, the CSI boot
overlays, the ToF SDK task, the `/camera_0` master2master relay and the steamdeck_ui camera consumer are deleted.
Removals are pinned by retirement tests on `git ls-files`. marker_detector moves to D435 colour + `/head_rgbd/depth`,
and its zoom stage is removed together with every other zoom consumer (Q1).

### D12: web_ui on the D435

Decision: Head tab: D435 colour, a depth preview of `/head/d435/depth/image_rect_raw` (on demand,
`depth_topic`/`depth_range_m`), and the RGB-D view on `/head_rgbd/*`. Amplitude and click-to-zoom are removed. Map tab: the
3D obstacles layer reads the D435 cloud (label "3D obstacles (D435 + lidar)"), and the Head RGB-D and Ground layers get
data once D1/D7 run. Arm tab: the gripper camera view, the 'gripper' 3D camera mode and `gripper_camera_mount` are
removed. The root pin test of that mount equal to `cameras.gripper.mount` goes with them, because it is no longer
accurate. Calibrate page: sections re-based per D10, plus the Arm section (D9). The ground recorder records the D435
cloud, the `/head_rgbd` pair, IMU and the ground state.

### D13: raw images stay on the client; RTAB-Map reads its sources from map3d_topics.yaml

Decision: every raw D435 consumer (head_rgbd_cpp, obstacle cloud, laser_filter, web_ui, mcp_server, marker_detector,
calibration) runs on the client. Only head_rgbd_cpp's compressed/decimated outputs cross WiFi, through map3d_uplink.
`rtabmap_mapping.launch.py` reads RGB_SOURCE/RGB_INFO_SOURCE/DEPTH_SOURCE from map3d_topics.yaml (reuse fix-as-ac).
map3d_topics.yaml carries the D435 frame names and 640x360/320x180 sizes.

### D14: memory and CPU budget measured, not estimated

Decision: realsense_d435 starts at memory_low 128M / memory_max 512M. The retirements free arducam_tof (64M),
head_camera_cpp and gripper_uvc_camera before head_rgbd_cpp and ground_estimator_cpp are enabled, so the enabled sum
stays <= 2560 MiB. After a day of running, `d435-memory-and-cpu-budget` sets the figures from Mimir (p50 / 2x peak, the
repo rule) and records realsense, head_rgbd_cpp, ground and obstacle cloud CPU with scripts/node_cpu_compare.py.
(my call)

### D15: overlapping ToF-bound epics are superseded; they are this epic's blockers

Decision: ticket.py refuses tasks that share files with open tasks of in-progress epics. So the four in-progress epics
whose open tasks overlap are this epic's README blockers. Recommendation per epic (no epic is edited here):
- drivable-ground-tof-filterin (20 open). Superseded: tof-amplitude-gate, tof-flying-pixel-retune, tof-filter-live-check,
  capture-baseline-recordings, capture-gated-recordings, guidance-replay-acceptance, guidance-live-check-and-cpu,
  live-report-guidance-metrics, ground-estimator-live-scene, ground-live-acceptance, ground-voxel-layer-on. Independent,
  finish first: stopbox-backing-and-turns, collision-source-stall-fix, nav2-goal-across-the-scene,
  retro-fixes-live-check, agent-retro-skill-lessons, tests-test-named-maps-py-tes, map3d-uplink-and-leader-brid,
  head-tilt-range-widened, calib-docs-and-test-index.
- tof-3d-obstacle-detection-fo (final-suite-and-runbook, escalated): superseded by nav2-live-check-on-d435.
- rgb-d-ground-estimation-and (ground-final-suite-runbook, escalated): superseded by ground-live-acceptance-on-d4.
- tof-depth-fix-and-ground-noi (head-remount-floor-runbook, blocked): superseded by d435-recalibration-runs (floor fit).
- head-chain-floor-plane-fit-a: every task done; close as delivered (no overlap).
- todo head-rgb-d-fusion-runbook: superseded entirely (ToF+CM3 runbook). todo printable-markers-runbook and
  rtab-map-3d-mapping-runbook name it as a blocker; re-point them to this epic.
- todo arm-joint-offsets-from-the-g: absorbed (user decision 3); its three tasks are re-planned here.
How the user closes superseded tasks (ticket.py has no wontfix state) is Q3.

### D16: a read-only D435 diagnostics script

Decision: `scripts/d435_diag.sh` follows bno055_diag.sh: SSH reads only (systemctl, journal, lsusb, sysfs speed,
`rs-enumerate-devices` with `--device` only while the unit is stopped) and topic_scraper_api for topics and rates (it
adds `sensor_msgs/msg/CameraInfo` to allowed_types so stream rates are measurable without raw images). It never starts
an rclpy participant. Live tasks cite its output as evidence. (my call)

## Research

| Lens | Ran / skipped | Adopted ideas | Rejected ideas |
|---|---|---|---|
| research-code-internal (cartographers, reuse hunt) | ran: docs/research/2026-10-27-d435-discovery*.md, -reuse.md | reuse of charuco, intrinsics.compare_sdk, head_arm_fit, floor_fit, UrdfChain, PoseChecker, calibration node, camera_geometry, point_cloud_xyz, laser_filter, head_rgbd_cpp cloud stage (reuse.md "ideas") | hand-tuning librealsense filters before measuring (reuse.md) |
| research-docs | ran: docs/research/2026-10-27-d435-docs.md | equal fps for HW sync (datasheet T4-6), explicit camera_name/namespace (README 2.9), min-Z table (T4-12), firmware >= 5.17.3.10 via rs-fw-update (6.1-6.3), device_info/journal for FW (6.5) | decimation_filter at the driver (it changes the published depth stream); OCC at bring-up (5.3, D10) |
| research-code-external (prior art) | ran: docs/research/2026-10-27-d435-prior-art.md | in-tree ground_estimator retargeted (a); board on gripper + FK prior (b); held-out acceptance at several pan/tilt poses (c); no raw images over WiFi (d) | robot_calibration port (D9); apriltag_ros (the in-tree ChArUco detector covers it); DREAM; depthimage_to_laserscan; STVL change (already STVL); groundfloor-segmentation benchmark; ICP refinement; calibrateRobotWorldHandEye; Kalib (non-goals or covered) |
| research-papers | skipped: no novel algorithmic core (plane fit and PnP in-tree), recorded in discovery.md section 9 | - | - |
| research-reviewer | skipped: plan-changing claims were probed (0b07, apt versions, transports); the rest are live ACs | - | - |

## Research inputs

Recent retros: `ticket.py retro --history` returned `[]` (no done epic has a RETRO.md), so no retro proposals apply.

Plan inputs from the briefs, each placed:

- reuse charuco make_board/detect/board_corners_3d (reuse.md) - calibration-config-on-d435, arm-offsets-plate-solver
- reuse intrinsics.calibrate/compare_sdk (reuse.md) - d435-intrinsics-check-comput
- reuse head_arm_fit fit_head_from_arm/board_pnp/camera_pose/aim_head (reuse.md) - head-arm-calibration-on-d435, arm-offsets-plate-solver
- reuse floor_fit fit_head_chain_floor/svd_plane (reuse.md) - floor-sweep-on-d435-depth
- reuse UrdfChain, rewrite_joint_origin, apply_overrides, head_chain_json (reuse.md) - calibration-library-d435-fra, d435-frames-on-the-head-urdf
- reuse PoseChecker (reuse.md) - head-arm-calibration-on-d435, d435-recalibration-runs
- reuse calibration node capture/sequencer/store/floor_sweep/head_arm_api/floor_api (reuse.md) - calibration-config-on-d435, floor-sweep-on-d435-depth, head-arm-calibration-on-d435
- reuse web_ui calibrate screens and rgbd_calibration proxy (reuse.md) - calibrate-rgb-d-section-on-d, calibrate-head-and-arm-ui-se
- reuse camera_geometry CameraIntrinsics/pixel_to_ground/project_point_to_pixel/solve_mount_pose (reuse.md) - shared-d435-camera-geometry, mcp-front-camera-on-d435
- reuse MCP camera_scene/camera_tools/camera_calib/jaw_crop (reuse.md) - mcp-camera-tools-front-only, mcp-grasp-check-on-d435
- reuse perception encode_jpeg/image_to_bgr/recompress_jpeg (reuse.md) - mcp-image-tools-front-only
- reuse web_ui depth/cloud serialisers and frontend rgbd.ts/RgbdView (reuse.md) - web-ui-head-tab-config-on-d4, head-tab-on-d435
- reuse point_cloud_xyz config (reuse.md) - d435-obstacle-cloud-node
- reuse laser_filter cloud self-filter (reuse.md) - laser-filter-on-the-d435-clo
- reuse head_rgbd_cpp cloud.cpp build_cloud/decode (reuse.md) - head-rgb-d-node-on-d435-alig
- reuse ground_estimator_cpp core (reuse.md, prior-art.md) - ground-estimator-on-d435-inp
- reuse marker_detector pose.estimate_pose, tof_plane.fit_plane/plane_chooser (reuse.md) - marker-detector-on-d435
- reuse scripts/markers_generate.py, markers/charuco.py (reuse.md) - d435-recalibration-runs (reprint only if the board is lost)
- reuse scripts/node_cpu_compare.py (reuse.md) - d435-memory-and-cpu-budget, monitoring-and-cpu-compare-o
- reuse scripts/head_rgbd_calibration/frames.py capture_bag (reuse.md) - retired with the CLI in retire-offline-rgb-d-calibra (the screen replaces it, D10)
- reuse gen_reach_map.py (reuse.md) - not needed: no body-model change (D4 changes the head only); recorded here
- reuse realsense2_camera launch args, extrinsics, device_info (docs.md) - d435-node-type-and-profile, d435-live-bring-up-and-topic
- reuse ros-jazzy-realsense2-camera from apt at 848x480@30 decimation 2 (prior-art.md) - rejected for 640x360@15 (D2)
- reuse robot_calibration (prior-art.md) - rejected (D9)
- reuse apriltag_ros / aruco_ros (prior-art.md) - rejected: in-tree ChArUco detector (do-not-build list)
- reuse mcp mark/candidate pattern for D435 RGB (prior-art.md) - mcp-camera-tools-front-only; per-pixel depth is a non-goal
- do-not-build depth-to-RGB registration and upsampling (reuse.md, docs.md, prior-art.md) - retire-rgb-d-registration-an, retire-rgb-d-upsampling-and
- do-not-build ToF-to-RGB stereo extrinsics (reuse.md) - retire-tof-stereo-calibratio
- do-not-build new ChArUco detector, depth-to-cloud node, pixel geometry, mount solver, browser colormap, gripper-pose solver (reuse.md) - honoured by every task above
- do-not-build custom firmware tooling or a librealsense source build (docs.md) - d435-live-bring-up-and-topic (rs-fw-update; RSUSB build only if the V4L2 build fails the 30 min soak, then a discovered task)
- do-not-build MoveIt calibration, DREAM, fixed min_obstacle_height floor removal, raw depth to the LLM (prior-art.md) - non-goals
- fix-as-ac stale bridge docs (reuse.md) - top-level-docs-and-memory-on
- fix-as-ac realsense node retired (reuse.md) - d435-node-type-and-profile
- fix-as-ac topic names unverified, pin from the live node (reuse.md) - d435-live-bring-up-and-topic
- fix-as-ac rtabmap hardcoded sources (reuse.md) - rtab-map-sources-from-config
- fix-as-ac ToF names hardcoded in shared calibration (reuse.md) - calibration-library-d435-fra, calibration-config-on-d435
- fix-as-ac four copies of URDF origin to 4x4 (reuse.md) - calibration-library-d435-fra (one helper in shared/ros2_common, used by mcp head_kinematics and sag in mcp-head-kinematics-on-d435)
- fix-as-ac inline depth deprojection duplicated (reuse.md) - shared-d435-camera-geometry, replay-export-depth-model, marker-detector-on-d435
- fix-as-ac MCP never reads camera_info (reuse.md) - shared-d435-camera-geometry, mcp-front-camera-on-d435
- fix-as-ac raw images across hosts drop (reuse.md, prior-art.md) - rtab-map-sources-from-config, web-ui-and-mcp-live-check-on
- fix-as-ac udev 0b07, fps match, frame collision, firmware (discovery.md, docs.md) - d435-udev-rule-via-node-depl, d435-node-type-and-profile, d435-live-bring-up-and-topic
- fix-as-ac drop redundant enable_accel/gyro (docs.md) - d435-node-type-and-profile
- fix-as-ac Pi 5 5V/5A supply (docs.md, prior-art.md) - d435-live-bring-up-and-topic
- fix-as-ac USB3 5000M and 30 min soak without VIDIOC_QBUF/No such device (prior-art.md) - d435-live-bring-up-and-topic
- fix-as-ac measured depth rate under full load (prior-art.md) - d435-memory-and-cpu-budget
- fix-as-ac calibration accepted on held-out poses at several pan/tilt settings (prior-art.md) - arm-offsets-plate-solver, d435-recalibration-runs
- fix-as-ac rigid ChArUco board with rotation diversity (prior-art.md) - arm-offsets-plate-solver
- fix-as-ac gripper beyond min-Z in head-view poses (prior-art.md) - head-arm-calibration-on-d435
- idea adopted: aligned depth feeds head_rgbd_cpp cloud stage only (reuse.md) - D1
- idea adopted: measure CPU before tuning (reuse.md) - d435-memory-and-cpu-budget
- idea adopted: gripper position via head_arm_fit with depth plus FK as a check (reuse.md) - arm-offsets-plate-solver
- idea adopted: one on-chip health check only if needed (docs.md) - D10
- idea adopted: clip/align only where consumed (docs.md) - D2 (align consumed by head_rgbd_cpp)
- idea adopted: re-record replay fixtures with scripts/ground_record.py (reuse.md) - record-d435-ground-replay-fi
- idea noted: ground no_return and guidance assume ToF (reuse.md) - ground-estimator-on-d435-inp
- idea open decision: overlapping ToF epics (reuse.md) - D15
- idea rejected: intra-process component container (docs.md) - consumers are separate systemd units; revisit only if the budget task measures a DDS copy cost
- idea rejected: eye_on_base robot_base_frame = head tilt link, calibrateRobotWorldHandEye, ICP refinement, Kalib, groundfloor benchmark (prior-art.md) - covered by D9 or out of scope

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 head_rgbd_cpp adapter stage | head-rgb-d-node-on-d435-alig, retire-rgb-d-registration-an, retire-rgb-d-upsampling-and, head-rgb-d-build-deps-cleanu, enable-rgb-d-and-ground-node |
| D2 D435 profile | d435-node-type-and-profile, d435-live-bring-up-and-topic, d435-memory-and-cpu-budget |
| D3 names and node type | d435-node-type-and-profile, d435-udev-rule-via-node-depl, top-level-docs-and-memory-on |
| D4 URDF owns D435 frames | d435-frames-on-the-head-urdf, d435-live-bring-up-and-topic, calibration-library-d435-fra, retire-tof-and-head-rgb-urdf, head-chain-overrides-on-d435 |
| D5 obstacle cloud | d435-obstacle-cloud-node, laser-filter-on-the-d435-clo |
| D6 Nav2 re-enabled | nav2-voxel-layer-on-d435-clo, nav2-collision-sources-on-d4, nav2-ground-layer-and-keepou, nav2-live-check-on-d435 |
| D7 ground estimator on D435 | ground-estimator-on-d435-inp, ground-recorder-on-d435-topi, replay-export-depth-model, record-d435-ground-replay-fi, tune-ground-estimator-on-d43, ground-live-acceptance-on-d4 |
| D8 MCP front camera | shared-d435-camera-geometry, mcp-intrinsics-from-camera-i, mcp-front-camera-on-d435, mcp-head-kinematics-on-d435, mcp-camera-tools-front-only, mcp-image-tools-front-only, mcp-approach-tools-front-onl, mcp-grasp-check-on-d435, mcp-depth-summary-on-d435-cl, mcp-gripper-camera-config-re, agent-prompt-and-config-on-d, agent-skills-and-evals-on-d4 |
| D9 gripper position estimation | arm-offsets-plate-solver, arm-offsets-compute-apply, calibrate-head-and-arm-ui-se, d435-recalibration-runs |
| D10 calibration on D435 | calibration-library-d435-fra, head-chain-overrides-on-d435, d435-intrinsics-check-maths, calibration-tests-off-the-cl, retire-tof-fits-in-calibrati, calibration-config-on-d435, d435-intrinsics-check-comput, retire-tof-stereo-calibratio, retire-offline-rgb-d-calibra, floor-sweep-on-d435-depth, head-arm-calibration-on-d435, calibration-apply-restart-un, calibrate-rgb-d-section-on-d, calibrate-head-and-arm-ui-se, d435-recalibration-runs |
| D11 retirement | mcp-zoom-retired, web-ui-zoom-route-retired, marker-detector-on-d435, marker-detector-zoom-retired, retire-arducam-tof-in-ansibl, retire-gripper-camera-in-ans, retire-head-camera-in-ansibl, retire-csi-camera-boot-overl, delete-arducam-tof-source, delete-head-camera-cpp-sourc, delete-uvc-camera-source, steamdeck-camera-relay-remov, retire-tof-and-head-rgb-urdf, retire-rgb-d-registration-an, retire-rgb-d-upsampling-and |
| D12 web_ui | web-ui-head-tab-config-on-d4, web-ui-obstacles-topic-on-d4, web-ui-arm-tab-config-withou, web-ui-zoom-route-retired, head-tab-on-d435, map-3d-obstacles-layer-on-d4, arm-tab-without-gripper-came, ground-recorder-on-d435-topi, web-ui-and-mcp-live-check-on |
| D13 raw images on client, RTAB-Map sources | rtab-map-sources-from-config, web-ui-and-mcp-live-check-on |
| D14 budget | d435-node-type-and-profile, enable-rgb-d-and-ground-node, d435-memory-and-cpu-budget, monitoring-and-cpu-compare-o |
| D15 superseded epics | final-suite-and-deploy |
| D16 diagnostics script | d435-diagnostics-script, d435-live-bring-up-and-topic |

## Review focus

- No placeholder data: head_rgbd_cpp publishes nothing without an exact-stamp colour + aligned depth + camera_info set,
  and every MCP/web_ui path answers "not available" instead of synthesising (tests per task).
- TF: no frame published twice (driver `publish_tf: false`, the URDF owns the d435 frames); every consumer resolves
  `d435_*` frames through head.urdf FK at the measured head angles.
- Memory budget test and CPU: the enabled memory_low sum after each Ansible task; the realsense node rate under load.
- Retirement order: no present:false lands before every consumer of that producer is re-pointed (depends_on).
- Servo safety in the live tasks: head pan/tilt only for floor and fixture runs; the arm tiers of the head-arm and arm
  sequences keep the shoulder confirm; FK-checked poses.
- Tests changed or deleted are named with the reason (no longer accurate). No test is weakened to pass.

## Left to the builder

- C++ structure of the head_rgbd_cpp adapter (exact-time sync policy, where throttling happens), within D1's contract.
- point_cloud_xyz decimation/voxel numbers for 640x360, within "cloud <= 3000 points per frame, 5 cm voxel".
- The re-tuned ground limits, within "the replay assertions pass and no limit is widened beyond what the D435 replays
  need" (each change cites the replay metric).
- Exact wording of the claude_agent camera note and skills, within D8.
- Layout of the Calibrate D435 intrinsics check UI, within D10's fields.

## Assumptions and unknown facts

- The live topic names under 4.58.4 with namespace head / name d435 are assumed `/head/d435/{color/image_raw,
  color/image_raw/compressed, color/camera_info, depth/image_rect_raw, depth/camera_info,
  aligned_depth_to_color/image_raw, aligned_depth_to_color/camera_info}`. The bring-up task probes this and pins the
  real list.
- `serial_no:=_935723024187`: the underscore form is the realsense-ros way to keep a numeric serial a string. The
  bring-up task confirms it from the startup log line.
- Firmware version unknown: the bring-up task reads it.
- Factory depth-to-colour extrinsic unknown: the bring-up task captures it (rs-enumerate-devices -c, unit stopped).
- D435 physical pose: similar to the old head unit (user). The CAD start is `0.02 0 0` on head_link, refined by the
  head-from-arm fit.
- compressed_image_transport is installed on the Pi (discovery probe); the bring-up topic list confirms `/compressed`.
- The Pi 5 power supply is 5 V 5 A: the bring-up task checks `usb_max_current_enable` / the PSU log line.
- Server-side RTAB-Map inputs were not traced beyond map3d_topics.yaml. rtab-map-sources-from-config reads them from
  the launch file and pins them.

## Open questions
- Q1: Zoom (head_camera_cpp full-resolution crop) has no D435 equivalent: retire it with head_camera_cpp in mcp_server (get_camera_image zoom), web_ui click-to-zoom and marker_detector, rather than emulate it as a digital crop of the 640x360 D435 frame? [recommended: yes, retire it everywhere; mcp-zoom-retired, web-ui-zoom-route-retired and marker-detector-zoom-retired assume this]
- Q2: Tool point: fit tool_offset_m in the plate solve as the fitted board pose composed with the clip's CAD board-to-jaw-tip offset (new config plate_mount.tool_in_board_m), so the 2026-10-10 tool point is re-done from the same D435 views? [recommended: yes; accepted only if the held-out tool error is under 5 mm, else the 2026-10-10 value stays and the report says so]
- Q3: The superseded ToF tasks in drivable-ground-tof-filterin, tof-3d-obstacle-detection-fo, rgb-d-ground-estimation-and and tof-depth-fix-and-ground-noi are this epic's blockers (ticket.py file-overlap rule) and cannot be marked wontfix: may the orchestrator amend each superseded task (ticket.py amend) to a note pointing at the task here that replaces it and close those epics, after the independent tasks listed in design D15 finish? [recommended: yes]
- Q4: If the D435 firmware reads below 5.17.3.10 at bring-up, flash it with rs-fw-update (unit stopped, flash backup with -b first) without a further stop? [recommended: yes; it is listed as an irreversible operation]

## Irreversible operations in scope

- D435 firmware flash with `rs-fw-update` (only if the read firmware is below 5.17.3.10). Safeguard: the unit is stopped,
  a flash backup is taken with `rs-fw-update -b` first, and `-r` recovery is documented in the runbook (Q4).
- Deleting tracked sources of retired nodes (nodes/bridges/arducam_tof, nodes/head_camera_cpp, nodes/bridges/uvc_camera,
  scripts/head_rgbd_calibrate.py and scripts/head_rgbd_calibration/, ToF-only C++ and calibration modules). Recoverable from git
  history. Pinned by retirement tests.
- Uninstalling units, venvs and configs on the client by `present: false` deploys (arducam_tof, head_camera_cpp,
  gripper_uvc_camera, tof_obstacle_cloud), and removing the CSI camera boot overlays (a reboot). The applied ToF/CM3
  calibration files under /var/lib/ros2/head_rgbd_calibration/applied stay on disk (not deleted).
- Committing new arm `joint_offsets_rad` / `tool_offset_m` to client.yml and deploying mcp_server (changes every arm
  motion). Safeguard: the result passes the held-out limits and the snippet is reviewed in the commit; the old values are
  in git.
