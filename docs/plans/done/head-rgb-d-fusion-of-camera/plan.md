---
title: Head RGB-D fusion of camera and ToF
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
owner: Szymon Richert
rung: 1
---

# Head RGB-D fusion of camera and ToF

## What and why

What: Register head ToF depth into the Camera Module 3 frame: aligned RGB-D pair, coloured cloud, calibration tool and written methodology
How: Calibration (intrinsics, extrinsics), approximate-time sync, reprojection with occlusion, optional guided upsampling, C++ node reusing depth_image_proc
Why: 3D mapping and perception need colour and depth that line up

The head carries a Camera Module 3 (RGB, 1152x648 @ 30 fps, rolling shutter, autofocus) and an Arducam ToF (240x180 depth + amplitude, ~10 Hz, up to 4 m) on one rigid frame with a known spacing. A fusion node registers ToF depth into the RGB camera so downstream consumers (3D mapping, obstacle display, the agent and the web UI) get a time-aligned, RGB-registered depth image and a coloured point cloud. The methodology (calibration, synchronisation, registration, upsampling, validation) is written down and measured.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: Methodology document (in the node README / docs): intrinsic calibration of both cameras (the ToF intrinsics from its SDK vs a checkerboard on the amplitude image), extrinsic ToF-to-RGB calibration on the rigid head frame (checkerboard seen by both; result feeds the head URDF), time synchronisation of 10 Hz depth with 30 Hz RGB (approximate time, latency budget, motion/pan-tilt and rolling-shutter error), registration (reproject ToF points into the RGB frame with occlusion handling), optional RGB-guided depth upsampling (e.g. joint bilateral / guided filter) and its validity mask, and the validation procedure with measured reprojection error and depth accuracy.
- 1.2: A fusion node (C++, reusing depth_image_proc register / point_cloud_xyzrgb where they fit, own code only for what they lack) publishing an RGB-registered depth image (16UC1, in head_camera_optical_frame, RGB resolution or a configurable fraction), its camera_info, and a coloured point cloud at a configurable reduced rate; nothing published without real synchronised inputs and valid TF.
- 1.3: A calibration tool/script producing the extrinsics and intrinsics files consumed by the node and the head URDF, with tests on synthetic data.
- 1.4: Unit tests for the projection/registration/upsampling maths on synthetic scenes; CPU budget measured on the Pi 5 (Grafana) within an agreed figure; Ansible node type, config, README; deployed and validated on the robot against a target at known distances.
- 1.5: The RTAB-Map epic consumes the registered RGB-D pair from this node.

## Agreed scope

Rung 1, migrated from the epic format: the epic's goals were its agreed scope.

## Non-goals

- Learning-based depth completion (no GPU).
- Changing the head camera or ToF drivers beyond what synchronisation needs.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Registration on the client in a new C++ node `head_rgbd_cpp`, exact-stamp pair output

`nodes/head_rgbd_cpp` (rclcpp, executable `head_rgbd_node`) owns the fusion. Outputs (namespace `/head_rgbd`):
`depth/image_raw` (16UC1 mm, 0 = no data, frame `head_camera_optical_frame`, stamp = the paired RGB frame's stamp),
`depth/camera_info` (the RGB camera_info scaled to the depth size), `rgb/image_raw/compressed` (the paired RGB
CompressedImage passed through unchanged, no re-encode) and `rgb/camera_info` (passed through), all four with one
header per pair; `points` (PointCloud2 XYZRGB) at `cloud.rate_hz`; `depth_upsampled/image_raw` + `camera_info` only
when upsampling is enabled. Stamping the depth with the RGB stamp makes every pair exact, so consumers (RTAB-Map's
rgbd_sync, the web UI) never re-pair and a downstream relay cannot split a pair. Alternatives: registering on the server
inside RTAB-Map's container (the RTAB-Map epic's former D5: no motion compensation, no flying-pixel filter, and only one
consumer gets the result), a Python node (CPU; MEMORY: C++ for non-web nodes), depth_image_proc RegisterNode on the
client (D7). Serves G2, G5, U1, U3.

### D2: Inputs: ToF depth plus the head camera mapping stream, subscribed lazily; the focus lock follows

The node pairs `/tof/depth/image_raw` with `/head_camera/mapping/image_raw/compressed` (576x324 JPEG, decimated from the
30 fps capture, `mapping.rate_hz` default 2, max 10) and `/head_camera/mapping/camera_info`. It subscribes its inputs
only while one of its outputs has a subscriber (checked every second; inputs released after `lazy.idle_unsubscribe_s`,
default 5 s, without output subscribers). Because head_camera_cpp locks focus at `lens_position` (default 1.0 dioptre,
about 1 m) exactly while the mapping image topic has subscribers and `mapping.lock_focus` is true, and keeps continuous
autofocus otherwise, the focus is locked only while a fusion or mapping consumer runs (U4, the RTAB-Map epic's
`mapping.lock_focus`). The mapping stream and its focus lock were planned in the RTAB-Map epic; because that epic now
depends on this one (U3), its two tasks move here unchanged in substance (`head-camera-mapping-config`,
`head-camera-mapping-publish`). The pair rate therefore equals the mapping rate; a consumer wanting 10 Hz registered depth
raises `mapping.rate_hz` (about 3 ms of JPEG encode per frame in head_camera_cpp). Alternatives: the 30 fps
`/head_camera/image_raw/compressed` (forces a 30 fps encode and decode, about 0.3 core), raw `image_raw` (fragment
drops over UDPv4), composing into head_camera_cpp's process (couples the camera owner to the fusion lifecycle).
(my call) Serves G2, U4.

### D3: Calibration: ChArUco, OpenCV, offline from a short rosbag of still poses

One printed matte ChArUco board (6x5 squares, 40 mm squares, 30 mm DICT_5X5_100 markers: 240x200 mm, so the board fits A4
landscape and its two-colour STL plate (250x210 mm) fits the 256x256 mm Bambu bed; the PDF and STL/3MF come from the
marker-and-calibration-board generator (`uv run python -m scripts.markers_generate`, a blocker of this epic);
`rgbd-calibration-frames` tests that this tool's yaml `board` equals the generator's `charuco` spec; 7x5 x 40 mm was dropped because 280 mm does not fit the bed; laser-printed: toner
absorbs IR) serves all three calibrations. Capture: `scripts/head_rgbd_calibrate.py` (mode `capture`) records a rosbag
on the client over ssh (`ros2 bag record` of `/tof/amplitude/image_raw`, `/tof/depth/image_raw`,
`/head_camera/mapping/image_raw/compressed`, `/head_camera/mapping/camera_info`, `/tf`, `/tf_static`; subscribing the
mapping topic engages the focus lock) and copies it to `.logs/head_rgbd/`. The operator holds the board still in
20-30 poses (0.5-1.5 m, tilted up to 45 deg, covering the image corners). Offline, the tool reads the bag with the
`rosbags` library, keeps still windows (consecutive-frame image difference under a threshold for 0.5 s in both
cameras), pairs RGB and ToF frames within 50 ms, normalises the 16-bit amplitude to 8 bit (percentile stretch), detects
ChArUco corners (`cv2.aruco.CharucoDetector`), runs `cv2.calibrateCamera` per camera (plumb_bob) and
`cv2.stereoCalibrate` with `CALIB_FIX_INTRINSIC` on corners seen by both. RGB intrinsics are calibrated on the 576x324
mapping image at the locked lens position and written as ROS camera_calibration YAML to
`nodes/head_camera_cpp/config/head_camera_calibration.yaml` (head_camera_cpp scales it to each published size); the ToF
intrinsics go to `nodes/head_rgbd_cpp/config/tof_calibration.yaml`; the extrinsic and the run report (RMS values, lens
position, board, date, SDK intrinsics for comparison) to `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml`. The
RTAB-Map epic's planned checkerboard RGB-only script (`camera-calibration-script`) is folded into this tool so there is
one calibration path. Alternatives: plain checkerboard (fails on partial views at 240x180), ROS camera_calibration GUI
(needs 8-bit amplitude and a display; not testable), Kalibr (needs motion or an IMU; overkill for a rigid head),
topic_scraper_api snapshots (no mono16 support, JPEG loses amplitude range, no exact pairing). (my call on board and
capture) Serves G1, G3, U2.

### D4: The head URDF is the only extrinsic source; a hand-off rewrites `tof_joint`

The node never reads the extrinsics file; it takes `tof_optical_frame -> head_camera_optical_frame` from TF, published by
robot_state_publisher from `nodes/web_ui/urdf/head.urdf` (pan/tilt epic D5). The calibration tool's `urdf` mode
computes the `tof_joint` origin (xyz and rpy relative to `head_link`) that reproduces the calibrated transform given the
URDF's `head_camera_joint` and both optical joints, and rewrites only that `<origin>` element, keeping comments and
layout, and stamping a comment with the calibration date and stereo RMS. `--check` reports the drift between the URDF
and the calibration without writing. The web_ui URDF test keeps its structure checks and accepts calibrated values
within tolerance (spacing 0.05 +- 0.015 m, ToF left of RGB). Alternatives: a static TF publisher with the calibrated
transform (two sources), the node loading the extrinsics file (two sources). Serves G3.

### D5: Time synchronisation: fix the ToF stamp, pair with ApproximateTime, stamp the pair with the RGB time

`TofBridge.publish_frame` stamps `now()` after `requestFrame` returns and after the depth/amplitude conversions, so the
ToF stamp is late by exposure plus SDK processing plus conversion, while head_camera_cpp stamps at the libcamera sensor
timestamp (start of exposure). The bridge takes the clock reading immediately after `requestFrame` returns and subtracts
a configured `stamp_latency_ms` (pydantic, 0-200, code default 0 = current behaviour; the deployed value is the measured
ToF-RGB offset, D11). The node pairs ToF depth with the RGB image and camera_info with
`message_filters::sync_policies::ApproximateTime` (queue 10, `sync.max_pair_dt_s` default 0.06: half a ToF period plus
jitter), so at 10 Hz ToF the residual pair offset is at most about 50 ms before motion compensation. The RGB rolling
shutter (about 15-20 ms readout) is not modelled per row: its constant half-readout part is absorbed into the measured
ToF-RGB offset, and the remaining absolute error against TF time is about 0.6 px at 20 deg/s at the output size,
documented. The SDK frame timestamp is not used: its clock domain is undocumented (my call). Serves G1, G2, U2.

### D6: Motion: tf2 time-travel compensation plus a motion gate

For each pair the node looks up `T_rgb(t_rgb) <- tof(t_tof)` through `motion.fixed_frame` (default `odom`) with
`tf2_ros::Buffer::lookupTransform(target, t_rgb, source, t_tof, fixed, timeout)`, which folds head joint motion and base
odometry between the two capture times into the registration transform. It also computes the ToF frame's angular and
linear rate in the fixed frame over `motion.rate_window_s` (0.05 s) before `t_tof` and drops the pair when the rate
exceeds `motion.max_angular_rate_deg_s` (default 10) or `motion.max_linear_rate_m_s` (0.3): a ToF frame integrates
four phase captures, so intra-frame blur cannot be compensated. Any TF lookup failure drops the pair (logged, counted;
no fallback to the static transform). At 30 deg/s a 33 ms error is about 4 px at 288x162; compensation brings it under
the 1 px alignment target. Alternatives: gating only (drops every frame of a slow pan), compensation only (keeps blurred
frames), depth_image_proc's single lookup at the depth stamp (no compensation). Serves G2.

### D7: Registration: own z-buffer at half the mapping size, footprint splat, jump-edge filter

ToF fx about 191 at 240 px and the RGB fx about 220 at 288 px give a near 1:1.15 mapping at the default output (divisor 2
of the 576x324 mapping image = 288x162), so registration needs no upsampling there. Per frame: (1) jump-edge filter on
the raw ToF depth: a valid pixel is cleared when any valid 8-neighbour differs by more than
`max(filter.jump_abs_mm, filter.jump_rel * z)` (defaults 30 mm, 0.05) or it has fewer than `filter.min_valid_neighbours`
(3) valid neighbours, which removes mixed (flying) pixels at depth steps and isolated speckle while keeping slanted
planes up to 70 deg; (2) back-project with a ray table precomputed once from the ToF calibration (`cv::undistortPoints`)
times depth; (3) transform with the motion-compensated `T`; (4) project with the RGB intrinsics scaled to the output size,
including the RGB plumb_bob distortion so pixels line up with the raw (unrectified) RGB image; (5) splat each point over
the rectangle spanned by its pixel's projected corners, clipped to the image, keeping the nearest depth per output pixel
(z-buffer). Output depth is z in the RGB optical frame in mm; pixels with no ToF support (occlusion shadows, edges,
outside the ToF field of view) stay 0. depth_image_proc's RegisterNode is rejected: in Jazzy register.cpp the
`fill_upsampling_holes` rectangle bounds check misses the `u2 < 0` / `u1 >= width` cases (out-of-bounds write), it looks
up TF only at the depth stamp, and it has no flying-pixel or confidence handling; its footprint-splat idea is kept, with
correct clipping. Configurable `output.divisor` 1 (576x324, splat footprint about 2.3 px) or 2 (default). Serves G2.

### D8: Optional masked guided-filter upsampling, off by default

When `upsampling.enabled`, the node upsamples the registered depth by 2 to 576x324 guided by the decoded mapping RGB
(grey, float [0, 1]) with `cv::ximgproc::guidedFilter` in normalised-convolution form: `D = GF(I, D0 * M) / GF(I, M)`,
where `D0` is the nearest-upsampled registered depth and `M` its validity mask, so holes never bleed zeros into
neighbours. A pixel is valid only when `GF(I, M) >= upsampling.min_support` (default 0.5) and it lies within
`upsampling.radius` (default 4) of a valid registered pixel; every other pixel is 0, never interpolated across a hole.
Alternatives: brute-force joint bilateral (O(N r^2), too slow), fast global smoother (2-4x the cost, kept as the
documented alternative), weighted median, bilateral solver, TGV/MRF, ip_basic (cost or LiDAR-sparse assumptions).
Expected cost about 3-5 ms per frame on the Pi 5 (measured in the runbook). Serves G1, G2, U2.

### D9: Coloured cloud from depth_image_proc's conversions, decimated

On pairs selected by a rate decimator (`cloud.rate_hz`, default 1.0), the node decodes the RGB JPEG at the output size
(`cv::IMREAD_REDUCED_COLOR_2` at divisor 2) and builds `points` with depth_image_proc's `convertDepth` / `convertRgb`
helpers (`depth_image_proc/conversions.hpp`) and an `image_geometry::PinholeCameraModel` of the depth camera_info:
about 47k points at 288x162. The full PointCloudXyzrgbNode is not loaded: it would re-pair inputs and could not be
decimated without splitting pairs. If the conversions header is not exported in the installed Jazzy package, the builder
writes the back-projection on the existing ray maths and says so in the commit. Serves G2.

### D10: Ansible node type `head_rgbd_cpp`, background tier, metrics

client.yml gets node type `head_rgbd_cpp` (native colcon build like `map_guard_cpp`: `shared/ros2_common_cpp` then the
package, Release, `-j2`), config at `/etc/ros2/head_rgbd_cpp/config.yaml` plus the ToF calibration file, apt
dependencies (rclcpp, message_filters, tf2_ros, tf2_eigen, sensor_msgs, cv_bridge, image_geometry,
camera_calibration_parsers, depth_image_proc, libopencv-dev, libopencv-contrib-dev, libyaml-cpp-dev, prometheus-cpp-dev),
background tier (cpu_weight/io_weight 100, memory_low 48M, memory_max 256M until measured: MEMORY says constrain
background work, keep headroom for robot nodes), Prometheus metrics port, and the ros2_nodes entry present and enabled.
The head_camera_cpp block gets `lens_position: 1.0` and `mapping: {enabled: true, lock_focus: true, rate_hz: 2.0, width:
576, height: 324, jpeg_quality: 80}` with `af_mode` left continuous; arducam_tof gets `stamp_latency_ms` (measured value
filled by the runbook). Metrics: `head_rgbd_pairs_published_total`, `head_rgbd_pairs_dropped_total{reason}` (no_tf,
motion, no_calibration, no_camera_info, size_mismatch), `head_rgbd_pair_dt_seconds` and `head_rgbd_process_seconds`
histograms, `head_rgbd_valid_ratio` gauge, plus the process CPU series from ros2_common_cpp. (my call on tier and caps)
Serves G4.

### D11: Methodology written down, measured on the robot

The methodology (section below) lives in `docs/head-rgbd-fusion-methodology.md`, linked from the node README, with a
results table filled from the robot runbook `docs/runbooks/head-rgbd-fusion.md`. The calibration tool's `validate` mode
computes the metrics offline from bags: depth bias and RMSE in a central ROI at known distances, edge-alignment error of
a board in front of a far background, valid-pixel ratio, and the ToF-RGB time offset from a pan sweep. CPU comes from
Grafana (cAdvisor `container_cpu_usage_seconds_total` of `ros2-head_rgbd_cpp.service`, and the node's
`process_cpu_seconds_total`). Serves G1, G4, U2.

### D12: RTAB-Map consumes the pair

Amendments to epic `rtab-map-rgb-d-3d-mapping-on` (U3): this epic becomes its blocker; its uplink carries
`/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/rgb/camera_info`, `/head_rgbd/depth/image_raw` (as compressedDepth)
and `/head_rgbd/depth/camera_info` unthrottled (already at the mapping rate; independent throttles could split pairs)
instead of the raw ToF depth and ToF camera_info; the server drops `depth_image_proc::RegisterNode` and feeds
rgbd_sync exact pairs; map3d_manager watches the two `/uplink/head_rgbd/*` camera_infos; the head camera mapping stream,
focus lock and calibration tasks move to this epic. Serves G5, U3.

## Open questions

none

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-papers (rgbd-fusion-research.md) | ran | Zhang intrinsics (doi:10.1109/34.888718); ChArUco (doi:10.1016/j.patcog.2014.01.005); stereoCalibrate with fixed intrinsics, joint depth/colour as reference (doi:10.1109/TPAMI.2012.125); fixed focus (doi:10.1109/WACV.2016.7477646); jump-edge filter (doi:10.1145/1921264.1921293); guided filter, masked (doi:10.1109/TPAMI.2012.213); depth bias check after warm-up (doi:10.1016/j.cviu.2009.11.002, doi:10.1109/TCI.2015.2510506); temporal offset measured and compensated (doi:10.1109/IROS.2013.6696514) | Kalibr (doi:10.1109/IROS.2013.6696514: needs motion/IMU); brute-force joint bilateral (doi:10.1145/1276377.1276497: cost); TGV/edge-weighted (doi:10.1109/ICCV.2013.127: seconds per frame); bilateral solver (arXiv:1511.03296: cost vs gain); ip_basic (doi:10.1109/CRV.2018.00013: LiDAR-sparse); per-row rolling shutter (doi:10.1109/CVPR.2013.179: below 1 px); multipath correction (doi:10.1109/ICSENS.2014.6985077) |
| research-docs (same brief, section 6, image_pipeline jazzy register.cpp) | ran | depth_image_proc conversions for the cloud; message_filters ApproximateTime; tf2 time-travel lookup | RegisterNode (bounds bug in `fill_upsampling_holes`, TF at depth stamp only, no flying-pixel handling); PointCloudXyzrgbNode as a node (re-pairs, no decimation); ROS camera_calibration GUI (8-bit only, untestable) |
| research-code-internal | ran (in the brief's Plan inputs and this planner's read of arducam_tof bridge.py, head_camera_cpp plans, map_guard_cpp, ros2_common_cpp) | `depth_to_mm`, `camera_info_msg`, `TofBridge.publish_frame` (bridge.py), the RTAB-Map epic's planned calibration script and mapping stream, ros2_common_cpp config/metrics, scripts/cpp-node-test.sh | a second calibration script |
| research-code-external | skipped | (the papers lens covered the analogues; no open design fork beyond D1-D3, settled by the user's U1-U4) | |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief reuse | depth_to_mm (bridge.py) | head-rgbd-node-wiring (input is its 16UC1, 0 = invalid) |
| brief reuse | camera_info_msg (bridge.py) | rgbd-intrinsics-calibration (SDK intrinsics recorded and compared) |
| brief reuse | TofBridge.publish_frame | tof-stamp-latency-fix |
| brief reuse | RTAB-Map planned camera calibration script (solve_calibration) | rgbd-intrinsics-calibration (folded in, D3) |
| brief reuse | depth_image_proc RegisterNode | rejected: bounds bug, no motion compensation (D7); footprint idea in rgbd-registration-zbuffer |
| brief reuse | depth_image_proc PointCloudXyzrgbNode | rgbd-cloud-builder (its conversions helpers, D9) |
| brief reuse | message_filters ApproximateTime | head-rgbd-node-wiring |
| brief reuse | cv::ximgproc guidedFilter / fastGlobalSmootherFilter / weightedMedianFilter | rgbd-guided-upsampling (guidedFilter); FGS and WMF rejected: cost, documented alternative only |
| brief reuse | cv::stereoCalibrate, cv::aruco::CharucoDetector | rgbd-intrinsics-calibration, rgbd-extrinsic-calibration |
| brief reuse | camera_ros SensorTimestamp stamping | tof-stamp-latency-fix (RGB reference documented); head_camera_cpp already stamps at sensor time |
| brief do-not-build | ChArUco detection and calibration solvers | rgbd-intrinsics-calibration, rgbd-extrinsic-calibration use OpenCV |
| brief do-not-build | time-sync matcher | head-rgbd-node-wiring uses message_filters |
| brief do-not-build | bilateral/guided/FGS/WMF filters | rgbd-guided-upsampling uses ximgproc |
| brief do-not-build | Kalibr or continuous-time calibration | rejected: static stereoCalibrate suffices (D3) |
| brief do-not-build | learned or optimisation-based completion | rejected: CPU budget, epic non-goal |
| brief do-not-build | per-row rolling-shutter model | rejected: constant offset absorbed (D5) |
| brief do-not-build | multipath correction | rejected: documented only (rgbd-methodology-doc) |
| brief fix-as-ac | ToF stamp taken at publish time | tof-stamp-latency-fix (stamp at frame return minus latency), rgbd-final-suite-and-runbook (offset <= 10 ms) |
| brief fix-as-ac | continuous autofocus during calibration and fusion | head-camera-mapping-config, head-camera-mapping-publish, rgbd-calibration-frames (capture subscribes the mapping topic), rgbd-extrinsic-calibration (report records lens position) |
| brief fix-as-ac | register rectangle-fill bounds bug | rgbd-registration-zbuffer (footprint crossing each border is clipped) |
| brief fix-as-ac | zero-distortion assumption | rgbd-intrinsics-calibration (zero-D RMS <= 0.3 px else measured D) |
| brief fix-as-ac | flying pixels pass to registration | tof-jump-edge-filter |
| brief fix-as-ac | placeholder depth from upsampler | rgbd-guided-upsampling (outside the mask = 0) |
| brief fix-as-ac | motion misregistration | rgbd-motion-compensation, head-rgbd-node-wiring |
| brief fix-as-ac | CPU <= 0.2 core | rgbd-final-suite-and-runbook |
| brief ideas adopted | register at RGB/4 by default | rgbd-registration-zbuffer, head-rgbd-package-config (divisor 2 of the mapping image) |
| brief ideas adopted | masked guided filter | rgbd-guided-upsampling |
| brief ideas adopted | ChArUco + stereoCalibrate(CALIB_FIX_INTRINSIC) | rgbd-intrinsics-calibration, rgbd-extrinsic-calibration |
| brief ideas adopted | depth-bias check after warm-up, LUT only if > 1 cm | rgbd-validation-metrics, rgbd-final-suite-and-runbook |
| brief ideas adopted | tf2 time-travel or motion gating | rgbd-motion-compensation (both) |
| brief ideas rejected | joint bilateral brute force, TGV, bilateral solver, ip_basic, Kalibr | rejected: see Research |
| user U1 | fusion node with registered depth, camera_info, coloured cloud | head-rgbd-node-wiring, rgbd-cloud-builder |
| user U2 | methodology written and measured | rgbd-methodology-doc, rgbd-validation-metrics, rgbd-final-suite-and-runbook |
| user U3 | RTAB-Map consumes the pair | head-rgbd-node-wiring (exact pairs); RTAB-Map epic amended (D12) |
| user U4 | focus locked only while needed | head-camera-mapping-publish, head-rgbd-node-wiring (lazy inputs) |

## Irreversible operations

- none (calibration files and URDF values are versioned in git; deploys are reversible by redeploying the previous
  commit).

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-22)

Closed by user decision 2026-10-22 (answer A): every remaining task is done, verified and merged. The one leftover
live runbook task rgbd-final-suite-and-runbook moved, requirement and acceptance conditions verbatim, to the follow-up epic head-rgb-d-fusion-runbook (docs/plans/todo/head-rgb-d-fusion-runbook),
blocked by drivable-ground-tof-filterin. The runbook moved to head-rgb-d-fusion-runbook.

### design.md: Summary

A new C++ node `nodes/head_rgbd_cpp` on the client Pi 5 registers the head Arducam ToF depth (240x180, 10 Hz) into the
head Camera Module 3 image and publishes an exact-stamp RGB-D pair: a 16UC1 registered depth image in
`head_camera_optical_frame` (default 288x162, half the 576x324 mapping image, RGB/4 of the 1152x648 stream), its
camera_info, the paired RGB JPEG passed through unchanged with its camera_info, and a coloured cloud at a reduced rate.
Registration is our own z-buffer (depth_image_proc's RegisterNode has a Jazzy out-of-bounds bug and no motion
compensation) with a jump-edge flying-pixel filter, tf2 time-travel motion compensation and a motion gate; RGB-guided
upsampling (masked guided filter) is optional and off by default. Intrinsics and the ToF-to-RGB extrinsic come from an
offline ChArUco calibration tool over a short rosbag; the extrinsic is written into the head URDF, the single geometry
source. The head camera locks focus only while its mapping stream has subscribers (this node subscribes lazily). The
RTAB-Map epic consumes the pair instead of registering on the server. The methodology is written down with measured
targets and validated on the robot.

### design.md: Intent and why

The head carries an RGB camera and a ToF depth camera a few centimetres apart on one rigid pan/tilt frame. 3D mapping,
obstacle display, the agent and the web UI need colour and depth that line up in time and in pixels. Without a
registration step every consumer has to solve calibration, synchronisation and occlusion itself, and the obvious
off-the-shelf path (depth_image_proc register) silently misregisters while the head moves. The user wants one node that
does it right and a written, measured methodology (U1, U2), and wants RTAB-Map to use that node's pair (U3) with the
camera focus locked only while calibrated intrinsics are needed (U4).

### design.md: Goals

- G1: Methodology document: intrinsic calibration of both cameras (ToF SDK intrinsics vs a calibration on the amplitude
  image), extrinsic ToF-to-RGB calibration feeding the head URDF, time synchronisation (approximate time, latency budget,
  pan/tilt motion and rolling-shutter error), registration with occlusion handling, optional RGB-guided upsampling and
  its validity mask, and the validation procedure with measured reprojection error and depth accuracy.
- G2: A C++ fusion node publishing an RGB-registered 16UC1 depth image in `head_camera_optical_frame` (RGB resolution or
  a configurable fraction), its camera_info and a coloured point cloud at a configurable reduced rate; nothing published
  without real synchronised inputs, a loaded calibration and valid TF.
- G3: A calibration tool producing the intrinsics and extrinsics files consumed by the node and the head URDF, tested on
  synthetic data.
- G4: Unit tests on synthetic scenes for projection, registration, filtering, upsampling and sync/motion gating; CPU on
  the Pi 5 measured in Grafana within 0.2 core; Ansible node type, config, README; deployed and validated against a
  target at known distances.
- G5: The RTAB-Map epic consumes the registered RGB-D pair from this node.

### design.md: Non-goals

- Learning-based depth completion (no GPU), optimisation-based upsampling (TGV/MRF, fast bilateral solver).
- Changing the head camera or ToF drivers beyond what synchronisation and the calibrated-intrinsics focus lock need.
- Multipath correction, per-row rolling-shutter modelling, continuous-time (Kalibr) calibration.
- Feeding Nav2 costmaps (the ToF obstacle epic uses the raw ToF).

### design.md: Anti-goals

- Placeholder data on any topic: interpolated depth outside the validity mask, a camera_info without a calibration,
  depth published while TF or the RGB pair is missing, zeros published "to keep the topic alive".
- A second geometry source: the node reading extrinsics from a file while the URDF says something else.
- Permanent focus lock (close-up shots keep continuous AF outside mapping/fusion use).
- Fusion cost that pushes the already busy client Pi 5 over budget (> 0.2 core at the default path), or a 30 fps raw or
  JPEG RGB stream decoded by the node.
- Two calibration scripts for the same camera (the RTAB-Map epic's planned RGB-only script is folded into this tool).

### design.md: Constraints

- CLAUDE.md: C++ for new non-web nodes, established libraries first (OpenCV calib3d/aruco/ximgproc, message_filters,
  tf2, camera_calibration_parsers, image_geometry, depth_image_proc conversions, rosbags), pydantic for Python config,
  config files over CLI, TDD, type hints, no placeholder data on topics, numpy guard for Python nodes, Ansible kept in
  sync via the `ros2_nodes` schema, deploy only through `scripts/deploy-nodes.sh`.
- C++ packages build and test with `scripts/cpp-node-test.sh <pkg>` (docker, ros:jazzy-ros-base), shared helpers in
  `shared/ros2_common_cpp` (config loading, Prometheus metrics, process CPU series).
- Cross-process transport on the client is Fast DDS UDPv4 with 1 MiB buffers: 2.2 MB raw frames drop fragments
  (front-camera epic D2), so the node never subscribes raw RGB.
- Plans against the end state of the blocking epics: head_camera_cpp (front-camera epic) publishes `/head_camera/*` with
  camera_info only from a `camera_info_file` and `af_mode` continuous|auto|manual; the pan/tilt epic's
  robot_state_publisher publishes `head_camera_optical_frame` and `tof_optical_frame` from `nodes/web_ui/urdf/head.urdf`
  and the head joint states; arducam_tof publishes `/tof/depth/image_raw` (16UC1 mm, 0 invalid) and
  `/tof/amplitude/image_raw` (mono16) at <= 10 Hz.
- Servo safety: the only motion used for validation is a small head pan sweep inside the pan/tilt epic's verified range.

### design.md: Methodology

#### Calibration

- Focus: intrinsics are valid only at a fixed focus (autofocus changes focal length and distortion, WACV 2016
  doi:10.1109/WACV.2016.7477646). The RGB is calibrated at `lens_position` with the mapping lock engaged; the run report
  records the lens position, and changing `lens_position` invalidates the calibration (the README says so).
- Intrinsics (Zhang, doi:10.1109/34.888718): ChArUco (doi:10.1016/j.patcog.2014.01.005) on the 576x324 RGB and on the
  normalised 240x180 amplitude image, `cv2.calibrateCamera`, plumb_bob. For the ToF the tool runs twice: with distortion
  fixed to zero (`CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K1 | CALIB_FIX_K2 | CALIB_FIX_K3`) and free. If the zero-distortion
  RMS is <= 0.3 px the SDK's "already undistorted" claim holds and D is written as zero; otherwise the measured D is
  written. The SDK fx/fy/cx/cy from `/tof/camera_info` are recorded next to the measured values (difference reported in
  %).
- Extrinsic: `cv2.stereoCalibrate(..., flags=CALIB_FIX_INTRINSIC)` over corner IDs seen in both images of a still pair
  (joint depth/colour calibration as the reference method, doi:10.1109/TPAMI.2012.125). Check (doi:10.1109/TPAMI.2014.2363827):
  the board plane from RGB PnP, transformed into the ToF frame, against the median ToF depth inside the board polygon:
  reported as plane depth bias.
- Depth bias (wiggling, doi:10.1016/j.cviu.2009.11.002; warm-up and reflectivity, doi:10.1109/TCI.2015.2510506): measured
  at 0.5/1/2/3 m after a 20 minute warm-up; a per-distance correction is added only if the residual bias exceeds 1 cm
  (it is then a new task, not silently skipped). Multipath near corners and floors (doi:10.1109/ICSENS.2014.6985077) is
  documented, not corrected.

#### Synchronisation

- Stamps: RGB at start of exposure (libcamera SensorTimestamp); ToF at frame return minus the measured
  `stamp_latency_ms`.
- Pairing: ApproximateTime with `max_pair_dt_s` 0.06; output stamped with the RGB stamp (exact pairs downstream).
- Latency budget: ToF exposure plus processing (offset removed by `stamp_latency_ms`), pairing residual <= 50 ms at
  10 Hz ToF (compensated by D6), node processing <= 15 ms p95 at the default path.
- Error model: misalignment in output pixels = pan rate x residual time x (output fx / 1 rad); at 288 px (fx about 220)
  1 deg is about 3.8 px. Rolling shutter: constant half-readout absorbed into the offset; per-row skew under 1 output
  pixel below 30 deg/s.

#### Registration

- Ray table from the ToF calibration, motion-compensated transform from TF, projection with distortion into the RGB
  intrinsics scaled to the output size, footprint splat clipped to the image, nearest depth wins (occlusion).
- Occlusion shadows (RGB sees, ToF does not) stay 0; points behind the RGB camera or outside `[depth.min_m, depth.max_m]`
  (0.1, 4.0) are skipped.

#### Filtering

- ToF confidence thresholding stays in the bridge (`confidence_threshold`); it does not remove flying pixels (SCCG 2008,
  doi:10.1145/1921264.1921293), so the jump-edge filter of D7 runs before reprojection.

#### Upsampling

- Optional masked guided filter (doi:10.1109/TPAMI.2012.213) to 576x324 with the validity rule of D8; fast global
  smoother (doi:10.1109/TIP.2014.2366600) documented as the alternative. Evaluated with the ToFMark-style RMSE against
  registered depth held out at full resolution on synthetic scenes (doi:10.1109/ICCV.2013.127 protocol).

#### Validation metrics and targets

| Metric | How | Target |
|---|---|---|
| RGB intrinsic RMS reprojection | calibrateCamera on the capture bag | <= 0.5 px at 576x324 |
| ToF intrinsic RMS reprojection | calibrateCamera on normalised amplitude | <= 0.3 px |
| Stereo RMS | stereoCalibrate, RGB pixels | <= 1.0 px at 576x324 |
| Plane depth bias (ToF vs RGB PnP) | extrinsic check on board poses | <= 15 mm median |
| Depth accuracy, raw ToF | flat matte target, perpendicular, central 20x20 ROI, 50 frames, after 20 min warm-up, tape reference | abs(bias) <= 20 mm and RMSE <= 20 mm at 0.5/1/2 m; <= 40 mm at 3 m |
| Registration adds no bias | median(registered) - median(raw) over the same target | abs <= 5 mm |
| Edge alignment | board 1 m in front of a wall >= 2.5 m: distance from registered depth-step pixels to the RGB board outline | median <= 1.0 px at 288x162 |
| Valid-pixel ratio | flat wall at 1.5 m, inside the ToF field of view | >= 0.85 of output pixels filled |
| ToF-RGB time offset after the fix | pan sweep +-20 deg at 20 deg/s, offset from ToF-projected vs RGB corner tracks | abs <= 10 ms |
| Motion | the same sweep with compensation on | median edge alignment <= 1.5 px; pairs above the rate limit dropped |
| CPU | Grafana, 10 min, `mapping.rate_hz` 10, upsampling off | head_rgbd_cpp <= 0.2 core p95; upsampling on reported |
| Placeholder check | ToF stopped, TF missing, no calibration | nothing on `/head_rgbd/*` |

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 Client-side node, exact-stamp pair | head-rgbd-package-config, head-rgbd-node-wiring |
| D2 Inputs and lazy subscription, focus lock | head-camera-mapping-config, head-camera-mapping-publish, head-rgbd-node-wiring, head-rgbd-node-type |
| D3 ChArUco calibration from a rosbag | rgbd-calibration-frames, rgbd-intrinsics-calibration, rgbd-extrinsic-calibration, rgbd-calibrate-cli |
| D4 URDF is the only extrinsic source | urdf-extrinsics-handoff, rgbd-final-suite-and-runbook |
| D5 Time synchronisation | tof-stamp-latency-fix, head-rgbd-node-wiring |
| D6 Motion compensation and gate | rgbd-motion-compensation, head-rgbd-node-wiring |
| D7 Own z-buffer registration and jump-edge filter | tof-jump-edge-filter, rgbd-registration-zbuffer |
| D8 Masked guided upsampling | rgbd-guided-upsampling, head-rgbd-node-wiring |
| D9 Coloured cloud | rgbd-cloud-builder, head-rgbd-node-wiring |
| D10 Ansible node type and metrics | head-rgbd-node-type |
| D11 Methodology and validation | rgbd-validation-metrics, rgbd-methodology-doc, rgbd-final-suite-and-runbook |
| D12 RTAB-Map consumes the pair | head-rgbd-node-wiring, head-rgbd-node-type |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `head-camera-mapping-core` | head-camera-mapping-config | head-camera-mapping-publish | `nodes/head_camera_cpp/include/head_camera_cpp/mapping_stream.hpp` |
| `head-camera-mapping-topics` | head-camera-mapping-publish | head-rgbd-node-wiring | `nodes/head_camera_cpp/src/head_camera_node.cpp` |
| `head-rgbd-calib-config` | rgbd-calibration-frames | rgbd-intrinsics-calibration, rgbd-extrinsic-calibration, rgbd-validation-metrics, rgbd-calibrate-cli | `scripts/head_rgbd_calibration/config.py` |
| `head-rgbd-calib-frames` | rgbd-calibration-frames | rgbd-intrinsics-calibration, rgbd-calibrate-cli | `scripts/head_rgbd_calibration/frames.py` |
| `rgbd-synthetic-scenes` | rgbd-intrinsics-calibration | rgbd-extrinsic-calibration, rgbd-validation-metrics, rgbd-calibrate-cli | `tests/rgbd_synthetic.py` |
| `head-rgbd-charuco` | rgbd-intrinsics-calibration | rgbd-extrinsic-calibration, rgbd-validation-metrics | `scripts/head_rgbd_calibration/charuco.py` |
| `head-rgbd-extrinsics-file` | rgbd-extrinsic-calibration | urdf-extrinsics-handoff, rgbd-validation-metrics, rgbd-calibrate-cli | `scripts/head_rgbd_calibration/extrinsics.py` |
| `head-rgbd-urdf-handoff` | urdf-extrinsics-handoff | rgbd-calibrate-cli | `scripts/head_rgbd_calibration/urdf_handoff.py` |
| `head-rgbd-validate` | rgbd-validation-metrics | rgbd-calibrate-cli | `scripts/head_rgbd_calibration/validate.py` |
| `head-rgbd-config` | head-rgbd-package-config | tof-jump-edge-filter, rgbd-registration-zbuffer, rgbd-guided-upsampling, rgbd-cloud-builder, head-rgbd-node-wiring | `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp` |
| `head-rgbd-tof-filter` | tof-jump-edge-filter | head-rgbd-node-wiring | `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp` |
| `head-rgbd-registration` | rgbd-registration-zbuffer | rgbd-motion-compensation, rgbd-guided-upsampling, head-rgbd-node-wiring | `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp` |
| `head-rgbd-synthetic-scene` | rgbd-registration-zbuffer | rgbd-motion-compensation, rgbd-guided-upsampling | `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp` |
| `head-rgbd-motion` | rgbd-motion-compensation | head-rgbd-node-wiring | `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp` |
| `head-rgbd-upsampling` | rgbd-guided-upsampling | head-rgbd-node-wiring | `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp` |
| `head-rgbd-cloud` | rgbd-cloud-builder | head-rgbd-node-wiring | `nodes/head_rgbd_cpp/include/head_rgbd_cpp/cloud.hpp` |
| `head-rgbd-topics` | head-rgbd-node-wiring | head-rgbd-node-type | `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp` |

### design.md: Review focus

- registration.cpp: footprint clipping at all four borders, z-buffer ordering, distortion applied in the right direction
  (undistort ToF rays, distort RGB projection), scaled intrinsics for the output divisor.
- motion.cpp and the node: the time-travel lookup argument order (target, t_rgb, source, t_tof, fixed), no fallback to
  the static transform on TF failure, the gate window.
- Placeholder rules: no output without calibration, RGB camera_info, TF or a pair; upsampler mask; the cloud only from
  real pairs; camera_info only scaled from a real one.
- Lazy inputs: inputs released when outputs lose subscribers, so the focus lock releases (U4).
- tof-stamp-latency-fix: clock read immediately after `requestFrame`, before conversions.
- Calibration tool: still-window selection never pairs frames more than 50 ms apart; YAML formats readable by
  camera_calibration_parsers; URDF rewrite touches only the `tof_joint` origin.

### design.md: Left to the builder

- ChArUco board defaults within: squares 5-8 by 4-6, square 30-50 mm, any DICT_4X4/5X5 dictionary, as long as the
  printed board fits A4/A3 and the tests use the same definition. The shipped default is fixed at 6x5, 40/30 mm,
  DICT_5X5_100 (coordinated with marker-and-calibration-board: its generator prints this board and
  `rgbd-calibration-frames` asserts equality with its `charuco` spec; the STL plate must fit a 256x256 mm bed and the row count stays odd).
- Whether the capture uses mcap or sqlite3 storage, as long as `rosbags` reads it and the tests write the same format.
- Internal structure of the C++ core (free functions vs small classes), provided the contract headers named above exist.
- Exact metric bucket boundaries; the metrics port, chosen free per tests/test_node_metrics_wiring.py.

### design.md: Assumptions and unknowns

- `depth_image_proc/conversions.hpp` is installed and exported by ros-jazzy-depth-image-proc: confirmed by the builder in
  the cpp-node-test container; fallback named in D9.
- `libopencv-contrib-dev` (ximgproc) installs next to ROS Jazzy's OpenCV 4.6 on noble arm64 and has a rosdep key or is
  added to `scripts/cpp-node-test.Dockerfile`: confirmed by the package-config task's build.
- `ros2 bag record` is available on the client (ros-jazzy-ros-base includes rosbag2): confirmed in the runbook's first
  capture; if missing, the node-type task's apt list gains it.
- ToF stamp latency and the IMX708 readout: unknown; measured by the pan-sweep offset in the runbook.
- Arducam SDK factory wiggling correction: unknown; the depth accuracy step measures the residual.
- Grafana per-unit CPU for client units exists through cAdvisor (used by the RTAB-Map epic's dashboards); the runbook
  uses an Explore query if no panel lists head_rgbd_cpp.
- The head is built and the pan/tilt URDF/TF is live: blocking epic deliverable.

### approvals.json

- G1: Methodology document: intrinsic calibration of both cameras (ToF SDK intrinsics vs a calibration on the amplitude image), extrinsic ToF-to-RGB calibration feeding the head URDF, time synchronisation of 10 Hz depth with the RGB stream (approximate time, latency budget, pan/tilt motion and rolling-shutter error), registration with occlusion handling, optional RGB-guided upsampling and its validity mask, and the validation procedure with measured reprojection error and depth accuracy. [decisions: D3, D5, D7, D8, D11] [tasks: rgbd-methodology-doc, rgbd-validation-metrics, rgbd-final-suite-and-runbook] [status: approved]
- G2: A C++ fusion node (reusing depth_image_proc where it fits, own code only for what it lacks) publishing an RGB-registered 16UC1 depth image in head_camera_optical_frame (RGB resolution or a configurable fraction), its camera_info and a coloured point cloud at a configurable reduced rate; nothing published without real synchronised inputs and valid TF. [decisions: D1, D2, D5, D6, D7, D8, D9] [tasks: tof-stamp-latency-fix, head-camera-mapping-config, head-camera-mapping-publish, head-rgbd-package-config, tof-jump-edge-filter, rgbd-registration-zbuffer, rgbd-motion-compensation, rgbd-guided-upsampling, rgbd-cloud-builder, head-rgbd-node-wiring] [status: approved]
- G3: A calibration tool producing the intrinsics and extrinsics files consumed by the node and the head URDF, with tests on synthetic data. [decisions: D3, D4] [tasks: rgbd-calibration-frames, rgbd-intrinsics-calibration, rgbd-extrinsic-calibration, urdf-extrinsics-handoff, rgbd-calibrate-cli] [status: approved]
- G4: Unit tests for the projection/registration/upsampling maths on synthetic scenes; CPU budget measured on the Pi 5 (Grafana) within 0.2 core; Ansible node type, config, README; deployed and validated on the robot against a target at known distances. [decisions: D7, D10, D11] [tasks: tof-jump-edge-filter, rgbd-registration-zbuffer, rgbd-motion-compensation, rgbd-guided-upsampling, head-rgbd-node-type, rgbd-final-suite-and-runbook] [status: approved]
- G5: The RTAB-Map epic consumes the registered RGB-D pair from this node (its uplink, server pipeline and status are amended; it is blocked by this epic). [decisions: D12, D1] [tasks: head-rgbd-node-wiring, head-rgbd-node-type] [status: approved]
- U1: A fusion node registering ToF depth into the RGB camera, publishing a registered depth image, its camera_info and a coloured cloud. [decisions: D1, D7, D9] [tasks: head-rgbd-node-wiring, rgbd-registration-zbuffer, rgbd-cloud-builder] [status: approved]
- U2: The methodology is written down and measured: calibration, synchronisation, registration, upsampling, validation. [decisions: D3, D5, D8, D11] [tasks: rgbd-methodology-doc, rgbd-validation-metrics, rgbd-final-suite-and-runbook] [status: approved]
- U3: The RTAB-Map epic consumes the registered RGB-D pair from this node. [decisions: D12, D1] [tasks: head-rgbd-node-wiring, head-rgbd-node-type] [status: approved]
- U4: The head camera keeps continuous autofocus normally and locks focus (about 1 m) only while mapping or fusion consumers need calibrated intrinsics, consistent with the RTAB-Map epic's mapping.lock_focus. [decisions: D2] [tasks: head-camera-mapping-config, head-camera-mapping-publish, head-rgbd-node-wiring, head-rgbd-node-type] [status: approved]
