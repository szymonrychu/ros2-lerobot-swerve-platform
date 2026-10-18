# Head RGB-D fusion methodology

How the head ToF depth is registered into the Camera Module 3 image by `nodes/head_rgbd_cpp`, how the cameras are
calibrated and synchronised, and how the result is validated. Written for a mid-level ROS 2 developer: every step names
its target and the tool that measures it. Design: `docs/plans/in-progress/head-rgb-d-fusion-of-camera/design.md`;
robot procedure: `docs/runbooks/head-rgbd-fusion.md`; node reference: `nodes/head_rgbd_cpp/README.md`.

## Hardware and geometry

Two cameras sit a few centimetres apart on one rigid pan/tilt frame (`head_link`):

| Camera | Sensor | Image | Intrinsics (approx.) | Field of view |
|---|---|---|---|---|
| RGB | Raspberry Pi Camera Module 3 (IMX708), rolling shutter, stamped at start of exposure | 1152x648 stream, 576x324 mapping image (JPEG, decimated, 2 Hz default) | fx ~ 440 at 1152 px, ~ 220 at 288 px | ~ 66 deg horizontal |
| ToF | Arducam ToF (4-phase continuous wave), 10 Hz, 16UC1 depth in mm (0 = invalid) plus mono16 amplitude | 240x180 | fx = fy ~ 191, cx = 120, cy = 90 (SDK, "already undistorted") | ~ 64 deg x 50 deg |

At the default output (`output.divisor` 2 of the 576x324 mapping image = 288x162) the RGB fx of about 220 against the
ToF fx of about 191 gives a near 1:1.15 pixel mapping: one ToF pixel covers about 1.15 output pixels, so registration
needs no upsampling there. At divisor 1 (576x324) the footprint of one ToF pixel is about 2.3 output pixels.

Frames come from the head URDF `nodes/web_ui/urdf/head.urdf`, published by `head_state_publisher`
(robot_state_publisher): `head_link` -> `head_camera_joint` -> `head_camera_link` -> `head_camera_optical_frame`, and
`head_link` -> `tof_joint` -> `tof_link` -> `tof_optical_frame`. Both optical joints use rpy `-1.5708 0 -1.5708`
(x right, y down, z forward). The URDF is the only extrinsic source: the node reads `tof_optical_frame ->
head_camera_optical_frame` from TF and never a file. Only the `tof_joint` origin is calibrated; the nominal spacing is
0.05 m (camera y -0.025, ToF y +0.025, ToF left of RGB).

## Calibration

All three calibrations use one tool, `scripts/head_rgbd_calibrate.py`, configured by `scripts/head_rgbd_calibrate.yaml`.

### Focus lock

Intrinsics are valid only at a fixed focus: autofocus changes focal length and distortion (WACV 2016,
doi:10.1109/WACV.2016.7477646). The RGB is calibrated at `lens_position` (default 1.0 dioptre, about 1 m) with the
mapping lock engaged: head_camera_cpp locks focus exactly while its mapping stream has subscribers. The capture records
the mapping topic, which engages the lock; the first `selection.skip_start_s` (2 s) of frames are ignored while the focus
settles. The run report records the lens position, and changing `lens_position` invalidates the calibration.

### Board and printing

One ChArUco board (doi:10.1016/j.patcog.2014.01.005) serves RGB intrinsics, ToF intrinsics and the extrinsic: 6x5
squares, 40 mm squares, 30 mm markers, `DICT_5X5_100` (240x200 mm, fits A4 landscape; the two-colour plate of 250x210 mm
fits the 256x256 mm print bed). Generate it with `uv run python -m scripts.markers_generate`; the tool's `board` config
must equal the generator's `charuco` spec (pinned by a test). Print it laser-printed and matte: toner absorbs infrared,
so the ToF amplitude image shows the pattern, and gloss would cause specular saturation. Keep it flat on a rigid backing.

### Capture protocol

`capture` records a rosbag on the client over ssh (`ros2 bag record` of `/tof/amplitude/image_raw`,
`/tof/depth/image_raw`, `/tof/camera_info`, the mapping image and camera_info, `/tf`, `/tf_static`) and copies it to
`.logs/head_rgbd/`. The operator holds the board still in 20-30 poses at 0.5-1.5 m, tilted up to 45 deg, covering the
image corners, with the head at rest. Offline the tool keeps still windows (consecutive-frame difference under
`still_rgb_mad` / `still_tof_mad` for `still_window_s` = 0.5 s in both cameras) and pairs RGB and ToF frames at most
`max_pair_dt_s` = 50 ms apart; at least `min_views` = 12 pairs are required. Never move the board during a window.

### Intrinsics

Zhang's method (doi:10.1109/34.888718), `cv2.calibrateCamera`, plumb_bob.

- RGB: on the 576x324 mapping image; written as ROS camera_calibration YAML to
  `nodes/head_camera_cpp/config/head_camera_calibration.yaml` (head_camera_cpp scales it to each published size).
  Target RMS <= 0.5 px.
- ToF: on the 16-bit amplitude image normalised to 8 bit by a percentile stretch (1 and 99). The tool calibrates twice:
  distortion fixed to zero (`CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K1 | CALIB_FIX_K2 | CALIB_FIX_K3`) and free. Decision
  rule: if the zero-distortion RMS is <= 0.3 px the SDK's "already undistorted" claim holds and D is written as zero;
  otherwise the measured D is written. Output `nodes/head_rgbd_cpp/config/tof_calibration.yaml`. Target RMS <= 0.3 px.
- SDK comparison: the SDK fx/fy/cx/cy from `/tof/camera_info` are recorded next to the measured values and the
  difference is reported in percent in the run report.

A calibration over its RMS limit is rejected and nothing is written.

### Stereo extrinsic

`cv2.stereoCalibrate(..., flags=CALIB_FIX_INTRINSIC)` over the corner ids seen in both images of a still pair: intrinsics
stay fixed so the extrinsic absorbs no intrinsic error (joint depth/colour calibration as the reference method,
doi:10.1109/TPAMI.2012.125). Target stereo RMS <= 1.0 px at 576x324. Result:
`nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml` (also the run report: RMS values, lens position, board, date).

### Plane depth check

The board plane from the RGB PnP pose is transformed into the ToF frame and compared with the median ToF depth inside the
board polygon (doi:10.1109/TPAMI.2014.2363827). The median difference is the plane depth bias; target <= 15 mm.

### Depth bias at known distances

ToF depth has a distance-dependent bias (wiggling, doi:10.1016/j.cviu.2009.11.002) that also drifts with temperature and
reflectivity (doi:10.1109/TCI.2015.2510506). After a 20 minute warm-up, measure a flat matte target perpendicular to the
camera at 0.5, 1, 2 and 3 m (tape reference) with `capture-validation`. Correction rule: a per-distance correction is
applied when the residual bias exceeds 1 cm; at or below 1 cm no correction is applied, and an uncorrected bias above
1 cm fails the depth-bias metric.

### URDF hand-off and redeploy

`uv run python -m scripts.head_rgbd_calibrate urdf` computes the `tof_joint` origin (xyz and rpy relative to `head_link`)
that reproduces the calibrated transform given `head_camera_joint` and both optical joints, and rewrites only that
`<origin>` element, keeping comments and layout and stamping a comment with the calibration date and stereo RMS.
`urdf --check` prints the drift (mm, deg) between the URDF and the calibration without writing. Commit the URDF, then
redeploy so the new transform reaches TF: `scripts/deploy-nodes.sh client head_state_publisher` (head_rgbd_cpp reads TF,
so it needs no redeploy for a URDF change; it does for a changed `tof_calibration.yaml`).

## Synchronisation

- Stamps. RGB: libcamera `SensorTimestamp`, the start of exposure. ToF: the bridge reads the clock immediately after
  `requestFrame` returns, before any conversion, and subtracts the configured `stamp_latency_ms` (0-200, code default 0;
  the deployed value is the measured ToF-RGB offset). Before the fix `publish_frame` stamped `now()` after the depth and
  amplitude conversions, so the ToF stamp was late by exposure plus SDK processing plus conversion. The SDK frame
  timestamp is not used: its clock domain is undocumented.
- Pairing. `message_filters::sync_policies::ApproximateTime` (queue 10) pairs ToF depth with the RGB image and
  camera_info; `sync.max_pair_dt_s` = 0.06 (half a ToF period plus jitter). At 10 Hz ToF the residual pair offset is at
  most about 50 ms before motion compensation (Motion). Larger offsets are dropped (`pair_dt`).
- Exact-stamp outputs. Depth, depth camera_info, RGB, RGB camera_info, the upsampled pair and the cloud of a pair all carry
  the RGB header and stamp, so consumers (RTAB-Map `rgbd_sync`) use an exact-time policy and never re-pair.
- Latency budget. ToF exposure plus processing: removed by `stamp_latency_ms`. Pairing residual: <= 50 ms at 10 Hz,
  compensated by the tf2 time travel. Node processing: <= 15 ms p95 at the default path (`head_rgbd_process_seconds`).
- Error model. Misalignment in output pixels = pan rate x residual time x (output fx per rad). At 288 px (fx ~ 220),
  1 deg is about 3.8 px, so 30 deg/s with 33 ms error is about 4 px. Rolling shutter (about 15-20 ms readout) is not
  modelled per row: its constant half-readout part is absorbed into the measured offset, and the remaining error against
  TF time is about 0.6 px at 20 deg/s at the output size, under 1 px below 30 deg/s.

## Motion

- Time-travel compensation. For each pair the node calls `lookupTransform(target, t_rgb, source, t_tof, fixed)` through
  `motion.fixed_frame` (default `odom`): the transform folds head joint motion and base odometry between the two capture
  times into the registration transform. A lookup failure drops the pair (`no_tf`); there is no fallback to the static
  transform.
- The gate. The ToF frame's angular and linear rate in the fixed frame are measured over `motion.rate_window_s` (0.05 s)
  before `t_tof`. The pair is dropped (`motion`) above `motion.max_angular_rate_deg_s` (10) or
  `motion.max_linear_rate_m_s` (0.3).
- Why blur is gated. A ToF frame integrates four phase captures, so intra-frame motion blur mixes depths inside the frame
  and cannot be compensated by a rigid transform; compensation fixes the capture-time offset only. Gating only would drop
  every frame of a slow pan; compensation only would keep blurred frames. Both together are used.

## Registration

1. Ray table. Precomputed once from the ToF calibration (`cv::undistortPoints`); the back-projection is the ray times
   depth.
2. Transform. Points go into the RGB optical frame with the motion-compensated transform.
3. Distortion. Projection uses the RGB intrinsics scaled to the output size and includes the RGB plumb_bob distortion, so
   registered pixels line up with the raw (unrectified) RGB image.
4. Footprint splat. Each point is splat over the rectangle spanned by its pixel's projected corners, clipped to the image
   at all four borders.
5. Z-buffer. The nearest depth per output pixel wins. Output depth is z in the RGB optical frame in mm.
6. Occlusion shadows. Pixels the RGB sees but the ToF does not (behind a nearer object, at edges, outside the ToF field of
   view) stay 0. Points behind the camera or outside `[depth.min_m, depth.max_m]` (0.1, 4.0) are skipped.

Why not depth_image_proc's RegisterNode: in Jazzy `register.cpp` the `fill_upsampling_holes` rectangle bounds check misses
the `u2 < 0` and `u1 >= width` cases (out-of-bounds write), it looks up TF only at the depth stamp (no motion
compensation), and it has no flying-pixel handling. Its footprint-splat idea is kept, with correct clipping.

## Filtering

- Jump-edge filter (before reprojection, on the raw ToF depth; flying pixels, doi:10.1145/1921264.1921293). A valid pixel is
  cleared when any valid 8-neighbour differs by more than `max(filter.jump_abs_mm, filter.jump_rel * z)` (defaults 30 mm,
  0.05) or it has fewer than `filter.min_valid_neighbours` (3) valid neighbours. This removes mixed pixels at depth steps
  and isolated speckle while keeping slanted planes up to about 70 deg.
- Confidence thresholding stays in the bridge (`confidence_threshold`). It removes low-amplitude pixels (dark or distant
  surfaces, noise) but does not remove flying pixels: a mixed pixel at a depth edge can have high amplitude and pass.
- Multipath (doi:10.1109/ICSENS.2014.6985077) near corners and floors biases depth towards the far surface. It is an
  accepted error source of the ToF sensor and is not corrected.

## Upsampling

Optional (`upsampling.enabled`, off by default). The registered depth is upsampled by 2 to 576x324 guided by the decoded
mapping RGB (grey, float [0, 1]) with `cv::ximgproc::guidedFilter` (doi:10.1109/TPAMI.2012.213) in normalised-convolution
form `D = GF(I, D0 * M) / GF(I, M)`, where `D0` is the nearest-upsampled registered depth and `M` its validity mask, so
holes never bleed zeros into neighbours.

- Validity mask. A pixel is valid only when `GF(I, M) >= upsampling.min_support` (0.5) and it lies within
  `upsampling.radius` (4) of a valid registered pixel. Every other pixel is 0; nothing is interpolated across a hole.
- Cost. About 3-5 ms per frame on the Pi 5 (measured in the runbook, reported against the CPU target).
- When to enable. Only for a consumer that needs the full 576x324 size (colour-edge-aligned depth for the UI or dense
  meshing); RTAB-Map consumes the 288x162 registered depth and does not need it. The fast global smoother
  (doi:10.1109/TIP.2014.2366600) is the documented alternative at 2-4x the cost. Evaluation uses the ToFMark-style RMSE
  against registered depth held out at full resolution on synthetic scenes (protocol of doi:10.1109/ICCV.2013.127).

## Validation

`scripts/head_rgbd_calibrate.py validate` computes the metrics offline from the calibration and validation bags and prints
a table of measured value, limit and pass for every target (an unmeasured target fails). Validation bags are recorded with
`capture-validation <suffix>` (`0.5m`, `1.0m`, `2.0m`, `3.0m`, `sweep`, `wall`) and include the `/head_rgbd/*` outputs.
Pan-sweep motion stays inside the pan/tilt epic's verified range (+-20 deg at 20 deg/s).

| Metric | How it is measured | Target |
|---|---|---|
| RGB intrinsic RMS | `calibrateCamera` on the capture bag (576x324) | <= 0.5 px |
| ToF intrinsic RMS | `calibrateCamera` on the normalised amplitude | <= 0.3 px |
| Stereo RMS | `stereoCalibrate`, RGB pixels | <= 1.0 px at 576x324 |
| Plane depth bias | ToF vs RGB PnP plane, board poses | <= 15 mm median |
| Raw ToF depth accuracy | flat matte target, perpendicular, central 20x20 ROI, 50 frames, 20 min warm-up, tape reference (`0.5m`..`3.0m` bags) | abs(bias) <= 20 mm and RMSE <= 20 mm at 0.5/1/2 m; <= 40 mm at 3 m |
| Registration adds no bias | median(registered) - median(raw) over the same target | abs <= 5 mm |
| Edge alignment | board 1 m in front of a wall >= 2.5 m (`wall` bag): distance from registered depth-step pixels to the RGB board outline | median <= 1.0 px at 288x162 |
| Valid-pixel ratio | flat wall at 1.5 m, inside the ToF field of view | >= 0.85 of output pixels filled |
| ToF-RGB time offset | `sweep` bag, +-20 deg at 20 deg/s, offset from ToF-projected vs RGB corner tracks | abs <= 10 ms |
| Motion | the same sweep with compensation on | median edge alignment <= 1.5 px; pairs above the rate limit dropped |
| CPU | Grafana, 10 min, `mapping.rate_hz` 10, upsampling off (see CPU budget) | <= 0.2 core p95; upsampling on reported |
| Placeholder check | ToF stopped, TF missing, no calibration | nothing on `/head_rgbd/*` |

### Results

The robot runbook `docs/runbooks/head-rgbd-fusion.md` (date, commit and operator noted there).

| Metric | Target | Measured | Pass |
|---|---|---|---|
| RGB intrinsic RMS | <= 0.5 px | | |
| ToF intrinsic RMS | <= 0.3 px | | |
| Stereo RMS | <= 1.0 px | | |
| Plane depth bias | <= 15 mm | | |
| Raw ToF depth accuracy 0.5 m | abs(bias), RMSE <= 20 mm | | |
| Raw ToF depth accuracy 1 m | abs(bias), RMSE <= 20 mm | | |
| Raw ToF depth accuracy 2 m | abs(bias), RMSE <= 20 mm | | |
| Raw ToF depth accuracy 3 m | abs(bias), RMSE <= 40 mm | | |
| Registration adds no bias | abs <= 5 mm | | |
| Edge alignment | median <= 1.0 px | | |
| Valid-pixel ratio | >= 0.85 | | |
| ToF-RGB time offset | abs <= 10 ms | | |
| Motion | median edge alignment <= 1.5 px | | |
| CPU | <= 0.2 core p95 | | |
| Placeholder check | 0 messages | | |

## CPU budget

Target: head_rgbd_cpp at most 0.2 core p95 over 10 minutes with `mapping.rate_hz` 10 and upsampling off; the node runs in
the background tier (cpu_weight 100, memory_max 256M) so robot nodes keep their headroom. Read it in Grafana:
cAdvisor `container_cpu_usage_seconds_total` of `ros2-head_rgbd_cpp.service`, e.g.
`rate(container_cpu_usage_seconds_total{id=~".*ros2-head_rgbd_cpp.service"}[1m])` (cores), and the node's own
`rate(process_cpu_seconds_total{job=~".*head_rgbd.*"}[1m])` from its Prometheus port 19117; take the p95 of the 10 minute
window (`quantile_over_time(0.95, ...[10m:15s])`). Use an Explore query when no dashboard panel lists the unit. Also check
`head_rgbd_process_seconds` (p95 <= 15 ms). Repeat with `upsampling.enabled: true` and report both.
