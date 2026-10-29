# head_rgbd_cpp

The D435 adapter of the head RGB-D contract (design D1). It pairs the RealSense D435 aligned depth with the colour
image (exact or approximate stamps), throttles the sets to `output.rate_hz`, removes the mixed pixels at depth edges,
downscales the depth, JPEG-encodes the colour and publishes the `/head_rgbd/*` topics that RTAB-Map (through
map3d_uplink), ground guidance, marker_detector and web_ui read, plus a coloured point cloud. A second, independent
stage republishes every frame of the full-FOV D435 depth with the same edge mask on `/head/d435/depth_filtered/*`, the
input of `d435_obstacle_cloud` (and so of laser_filter, ground_estimator_cpp, Nav2 and the collision monitor).
Registration is done by the camera (`align_depth.enable: true` of `realsense_d435`); the node has no calibration file
and needs no TF. The executable is `head_rgbd_node` (ROS node name `head_rgbd`).

## Layout

- `src/core/*.cpp` build into the static library `head_rgbd_core` (no ROS node code): the config (`config.cpp`) and the
  cloud stage (`cloud.cpp`: `build_cloud`, `RateDecimator`) the node uses. It links `ros2_common_cpp::depth_mask`, the
  edge mask. `test/core/test_*.cpp` each become a gtest named after the file; `test/core/synthetic_d435.hpp` renders
  the synthetic D435 box scene of `test_edge_mask_scene.cpp`.
- `src/pairing.cpp`, `include/head_rgbd_cpp/pairing.hpp`: `FramePairer`, the depth/colour pairing on message_filters
  with per-frame accounting; `test/test_pairing.cpp` runs it on synthetic stamp streams without ROS spinning.
- `src/head_rgbd_node.cpp`, `include/head_rgbd_cpp/head_rgbd_node.hpp`: `HeadRgbdNode` and its pure helpers
  `downscale_depth`, `scale_camera_info` and `SetThrottle`; `src/main.cpp`: the executable. `test/test_node.cpp` tests
  the helpers and runs the node in-process with probe publishers of the D435 inputs.
- `config/head_rgbd.yaml`: every key with its default, meaning and unit.

## Topics

The set outputs are best_effort, depth 1, under `output_namespace` (default `/head_rgbd`). The filtered depth outputs
are reliable, depth 1, under `filtered_depth.output_namespace` (default `/head/d435/depth_filtered`): reliable because
`d435_obstacle_cloud` (rtabmap_util point_cloud_xyz, `qos: 0`) subscribes with the system default; best-effort
subscribers match it too.

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `depth_topic` | `sensor_msgs/Image` 16UC1, mm | D435 depth aligned to colour: colour size |
| in | `rgb_image_topic` | `sensor_msgs/Image` rgb8 or bgr8 | raw D435 colour (client-local; raw images never cross WiFi) |
| in | `rgb_info_topic` | `sensor_msgs/CameraInfo` | colour camera_info; the latest one is kept |
| in | `filtered_depth.depth_topic` | `sensor_msgs/Image` 16UC1, mm | full-FOV D435 depth (`/head/d435/depth/image_rect_raw`), 15 Hz |
| in | `filtered_depth.info_topic` | `sensor_msgs/CameraInfo` | its camera_info; the latest one is kept |
| out | `<ns>/depth/image_raw` | `sensor_msgs/Image` 16UC1, mm | colour size / `output.divisor`, 0 = no depth, edge-masked |
| out | `<ns>/depth/camera_info` | `sensor_msgs/CameraInfo` | the colour camera_info scaled to the depth size |
| out | `<ns>/rgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | the colour at full size, JPEG `output.jpeg_quality`, format `bgr8; jpeg compressed bgr8` |
| out | `<ns>/rgb/camera_info` | `sensor_msgs/CameraInfo` | the colour camera_info |
| out | `<ns>/points` | `sensor_msgs/PointCloud2` XYZRGB | only when `cloud.enabled`; at most `cloud.rate_hz`, only while it has subscribers |
| out | `<filtered ns>/image_raw` | `sensor_msgs/Image` 16UC1, mm | every input frame, edge-masked, same header, size and encoding as the input |
| out | `<filtered ns>/camera_info` | `sensor_msgs/CameraInfo` | the latest input camera_info with the image header (point_cloud_xyz pairs them exactly) |

`<ns>/depth_upsampled/*` is not advertised: the aligned depth already has the colour geometry. With
`filtered_depth.enabled: false` the filtered topics are not advertised.

### Pairing

The depth and colour subscriptions feed `FramePairer`, which drives a `message_filters` synchroniser through
`Synchronizer::add` (queue `PAIR_QUEUE_SIZE` = 5 frames per stream): `ExactTime` for `pairing.policy: exact` (equal
stamps), `ApproximateTime` with `setMaxIntervalDuration(pairing.max_interval_s)` for `approximate` (at most half a
15 Hz frame, 1/30 s, so a colour frame never pairs with the next depth frame). A pair is checked, then throttled
(`SetThrottle`: a fixed grid of 1 / `output.rate_hz` on the colour stamps, so a 15 Hz stream gives `rate_hz` on
average), then published. Every message of a set (both images, both camera_infos and the cloud) carries the colour
header exactly, stamp and `frame_id` (`d435_color_optical_frame`), also under approximate pairing, so consumers pair
them with an exact-time policy.

Accounting: `FramePairer` keeps the stamps of the frames not yet paired. A frame is counted
`head_rgbd_dropped_total{reason="unpaired"}` exactly once, when it can no longer pair: a newer pair of its stream
passed it, or more than `PAIR_QUEUE_SIZE` newer frames of its stream arrived. `head_rgbd_frames_total{stream}` counts
the inputs and `head_rgbd_pairs_total` the pairs before the throttle, so `pairs / frames(colour)` is the pairing rate.
(The 2026-10-28 live numbers, 323 sets published against 2294 unpaired in ~3 min, compared throttled 2 Hz sets with
per-frame drops of two 15 Hz streams, about 5400 frames: about 42 % of the frames, not 87 % of the sets, went
unpaired.)

`head_rgbd_pair_stamp_delta_seconds` records the colour minus depth stamp of every pair candidate: each pair, and each
unpaired colour frame against the nearest depth frame seen. Read it live to pick the policy:

| Histogram shape | Meaning | Action |
|---|---|---|
| everything in the `(-0.0005, 0.0005]` bucket | exact pairs, unpaired frames have no partner at all | keep `exact`; the loss is frame drops (USB, CPU), not stamps |
| unpaired colour frames a few ms off (`0.0005 < abs <= 0.02`) | depth and colour of one frameset carry slightly different stamps | `approximate` with `max_interval_s` above that offset |
| unpaired colour frames about one period off (`abs` > 0.05) | a frame of one stream was lost | `exact` is right; fix the loss at the source |

### Depth edge mask

`ros2_common_cpp::depth_mask` (shared/README.md) removes the mixed and flying pixels a stereo camera puts between the
edge of a nearer object and the background behind it: a valid pixel becomes 0 (no depth) when a valid pixel of its
`edge_mask.window_px` x `window_px` neighbourhood is nearer by more than `max(edge_mask.abs_m, edge_mask.rel x z)`.
Only the far side of a jump goes, so an object keeps its edge pixels; `grow_px` widens the removed region. The aligned
depth of a set is masked at full size before the downscale (so `/head_rgbd/depth` and the cloud, i.e. RTAB-Map and the
ground guidance, never see the veil); the full-FOV depth is masked frame by frame for the obstacle cloud. The
librealsense post-processing filters realsense2_camera exposes (decimation, spatial, temporal, hole filling) smooth,
average or fill depth; none removes the pixels at a jump, which is why the mask runs here (none is enabled,
`realsense_d435` README, Filters).

The defaults (3 px window, 1 cm, 2.5 %) come from `test/core/test_edge_mask_scene.cpp`: synthetic D435 depth (factory
intrinsics of the runbook d435-switchover, 640x360, 1 mm quantisation, 0.2 % per-pixel jitter) of the floor and the
5 cm box of `tiny_truth.yaml`, seen from the head home pose (pan 0, tilt 30 deg) and the box-scene pose (pan 35, tilt
29.4 deg) by the depth and the colour camera, with mixed pixels injected as linear blends 1 to 3 px wide across every
box edge that occludes the floor. The test requires >= 95 % of the mixed pixels removed, >= 95 % of the floor pixels at
0.3-2.0 m and >= 95 % of the box-top pixels kept. The floor 2 m ahead deepens about 1.5 % per pixel row at the home
pose, so `rel` stays above it; floor whose depth grows faster than `rel` per pixel (beyond about 3.5 m at the home
pose, outside the obstacle cloud's 3 m) is removed too. Real D435 noise is not in this scene: the live check
(`realsense_d435` README, Filters) confirms the floor is kept on the robot.

Depth downscale (`output.divisor` 2): an output pixel takes the first pixel of its 2x2 source block, in row-major order
from the top-left one (the nearest-neighbour sample), whose depth lies in [`depth.min_m`, `depth.max_m`]; 0 when none
does. Depth is never interpolated across missing pixels. With divisor 1 only the range applies. The depth camera_info
divides width, height and the first two rows of K and P by the divisor (output pixel u samples source pixel
divisor * u); D and R are kept.

## Configuration

`/etc/ros2/head_rgbd_cpp/config.yaml`, path overridable with `HEAD_RGBD_CONFIG`. Unknown keys are rejected at every
level (the ToF-era `tof_depth_topic`, `tof_calibration_file`, `sync`, `motion`, `filter` and `upsampling` included) and
any error exits 78 with one `<key>: <problem>` line per error.

| Key | Default | Meaning |
|---|---|---|
| `depth_topic` | `/head/d435/aligned_depth_to_color/image_raw` | aligned depth input (16UC1, mm) |
| `rgb_image_topic` | `/head/d435/color/image_raw` | raw colour input (rgb8 or bgr8) |
| `rgb_info_topic` | `/head/d435/color/camera_info` | colour camera_info input |
| `output_namespace` | `/head_rgbd` | prefix of every set output topic |
| `output.divisor` | 2 | depth size = colour size / divisor (1 or 2); the colour stays at full size |
| `output.rate_hz` | 2.0 | published sets per second (0 < v <= 15), the mapping rate RTAB-Map is budgeted for |
| `output.jpeg_quality` | 80 | JPEG quality of the published colour (1..100) |
| `depth.min_m` / `depth.max_m` | 0.15 / 4.0 | valid depth range, m (0.15 m is the D435 min-Z at 640x360); outside it the depth is 0 |
| `cloud.enabled` | true | publish the coloured point cloud |
| `cloud.rate_hz` | 1.0 | point cloud rate, Hz (0 < v <= 10); clouds are built from published sets only |
| `lazy.idle_unsubscribe_s` | 5.0 | release a stage's inputs this long after the last subscriber of its outputs left, s |
| `edge_mask.enabled` | true | apply the edge mask to both depths |
| `edge_mask.window_px` | 3 | neighbourhood side, px (odd, 3..9) |
| `edge_mask.abs_m` | 0.01 | smallest jump that removes a pixel, m (>= 0) |
| `edge_mask.rel` | 0.025 | jump that removes a pixel per metre of its depth (0..1); `abs_m` or `rel` must be > 0 |
| `edge_mask.grow_px` | 0 | widen the removed region, px (0..2) |
| `filtered_depth.enabled` | true | advertise the full-rate filtered depth |
| `filtered_depth.depth_topic` | `/head/d435/depth/image_rect_raw` | full-FOV depth input (16UC1, mm) |
| `filtered_depth.info_topic` | `/head/d435/depth/camera_info` | its camera_info |
| `filtered_depth.output_namespace` | `/head/d435/depth_filtered` | outputs `<ns>/image_raw` and `<ns>/camera_info` |
| `pairing.policy` | `exact` | `exact` (ExactTime) or `approximate` (ApproximateTime) |
| `pairing.max_interval_s` | 0.02 | widest colour-to-depth stamp spread of an approximate pair, s (0 < v <= 1/30) |
| `metrics_port` | 19117 | Prometheus port, `null` disables |

## Metrics

Served on `127.0.0.1:<metrics_port>` (`/metrics`), plus `robot_node_info` and the `process_*` series.

| Series | Type | Meaning |
|---|---|---|
| `head_rgbd_sets_published_total` | counter | sets published |
| `head_rgbd_dropped_total{reason}` | counter | input sets dropped, by reason; `unpaired` counts frames (see below) |
| `head_rgbd_frames_total{stream}` | counter | input frames offered to the pairing, `stream` = `depth` or `colour` |
| `head_rgbd_pairs_total` | counter | depth/colour pairs formed, before the set throttle |
| `head_rgbd_pair_stamp_delta_seconds` | histogram | colour minus depth stamp of each pair candidate (Pairing) |
| `head_rgbd_process_seconds` | histogram | processing time of one published set (mask, downscale, JPEG, cloud) |
| `head_rgbd_valid_ratio` | gauge | fraction of published depth pixels holding a depth, last set |
| `head_rgbd_edge_mask_removed_ratio{stage}` | gauge | fraction of the valid pixels the mask removed in the last frame, `stage` = `aligned` or `filtered` |
| `head_rgbd_filtered_depth_published_total` | counter | full-FOV depth frames republished |
| `head_rgbd_filtered_depth_dropped_total{reason}` | counter | full-FOV depth frames not republished: `bad_input`, `no_camera_info` |
| `head_rgbd_filtered_depth_process_seconds` | histogram | processing time of one full-FOV frame (copy, mask) |

Sets thinned by the throttle are not drops. Drop logs are throttled to one per 5 s.

## Lazy inputs

A 1 s timer counts the subscribers of each stage's output publishers, independently: the set stage (the five
`/head_rgbd` outputs) and the filtered stage (the two depth_filtered outputs). With at least one, the stage creates
its input subscriptions; with none for `lazy.idle_unsubscribe_s` it destroys them and forgets its camera_info. An idle
stage therefore subscribes to nothing and costs no decode, copy or encode work. On the robot `d435_obstacle_cloud`
always subscribes the filtered depth, so that stage runs at the D435's 15 Hz; the sets run only while RTAB-Map, the
ground guidance, marker_detector or web_ui read them.

## No placeholder data

Nothing is published for a set that is not a paired aligned depth + colour pair with a camera_info; the set is
dropped and counted in `head_rgbd_dropped_total{reason}`:

| reason | Dropped when |
|---|---|
| `bad_input` | the depth is not little-endian 16UC1, the colour is not rgb8 or bgr8, an image's data is shorter than its size, or the image is smaller than `output.divisor` |
| `size_mismatch` | the depth and colour sizes differ |
| `no_camera_info` | no camera_info arrived yet, the latest one is not of the colour size, or its K is zero or has a non-positive focal length |
| `unpaired` | a depth or colour frame can no longer pair (one count per frame, Pairing) |

Likewise a full-FOV depth frame is republished only when it is little-endian 16UC1 with data of its size
(`bad_input` otherwise) and a camera_info of its size with a usable K arrived (`no_camera_info` otherwise); the
filtered topics then simply have no new sample. The mask only ever sets pixels to 0 (no depth); it never invents a
depth.

Failure mode: the obstacle cloud now depends on this node. If head_rgbd_cpp stops, `/head/d435/depth_filtered` and so
`/head/d435/points` stop, and the collision monitor's D435 sources time out (`source_timeout` 1.0 s), which stops
motion as a D435 outage does (nodes/nav2_bringup/README.md). The node runs in the normal cgroup tier for that reason
(ansible `head_rgbd_cpp` node type), like d435_obstacle_cloud.

## Test

`scripts/cpp-node-test.sh head_rgbd_cpp` (gtests and ament linters) and `scripts/cpp-node-test.sh head_rgbd_cpp
--lint-only`. Root wiring tests: `tests/test_head_rgbd_deploy.py`, `tests/test_d435_depth_filter_wiring.py`.
