# head_rgbd_cpp

Registers the head ToF depth into the Camera Module 3 frame: an aligned RGB-D pair and a coloured point cloud. The
executable `head_rgbd_node` (ROS node name `head_rgbd`) wires the fusion stages (filter, registration, motion,
upsampling, cloud; sources under `src/core`) to ROS topics, TF and metrics.

## Layout

- `src/core/*.cpp` build into the static library `head_rgbd_core` (no ROS node code); `test/core/test_*.cpp` each
  become a gtest named after the file.
- `src/head_rgbd_node.cpp`, `include/head_rgbd_cpp/head_rgbd_node.hpp`: `HeadRgbdNode`; `src/main.cpp`: the executable.
  `test/test_node.cpp` runs the node in-process (probe publishers, a `tf2_ros::Buffer` fed with static and timed
  transforms, `test/data/tof_calibration.yaml`).
- `config/head_rgbd.yaml`: every key with its default, meaning and unit.
- `test/data/tof_calibration.yaml`: camera_calibration fixture of the 240x180 ToF (fx=fy=191, cx=120, cy=90).

## Topics

All outputs are best_effort, depth 1 (like the mapping stream), under `output_namespace` (default `/head_rgbd`).

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `tof_depth_topic` | `sensor_msgs/Image` 16UC1, mm | ToF depth, frame = the ToF optical frame |
| in | `rgb_image_topic` | `sensor_msgs/CompressedImage` | the head_camera mapping stream (JPEG); raw RGB is never subscribed (UDPv4 fragment drops) |
| in | `rgb_info_topic` | `sensor_msgs/CameraInfo` | camera_info of the mapping stream (best_effort) |
| out | `<ns>/depth/image_raw` | `sensor_msgs/Image` 16UC1, mm | depth registered into the RGB camera, size = RGB size / `output.divisor` |
| out | `<ns>/depth/camera_info` | `sensor_msgs/CameraInfo` | RGB K scaled to the output size, P from it, same D and R |
| out | `<ns>/rgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | the input RGB, unchanged |
| out | `<ns>/rgb/camera_info` | `sensor_msgs/CameraInfo` | the input camera_info, unchanged |
| out | `<ns>/points` | `sensor_msgs/PointCloud2` XYZRGB | only when `cloud.enabled`; at most `cloud.rate_hz`, only while it has subscribers |
| out | `<ns>/depth_upsampled/image_raw` | `sensor_msgs/Image` 16UC1, mm | only when `upsampling.enabled` (needs `output.divisor` 2); RGB size |
| out | `<ns>/depth_upsampled/camera_info` | `sensor_msgs/CameraInfo` | only when `upsampling.enabled`; the RGB camera_info |

The three inputs are synchronised with `ApproximateTime` (queue `sync.queue_size`, maximum interval
`sync.max_pair_dt_s`). The exact-stamp rule: every message of one published set (depth, depth camera_info, RGB, RGB
camera_info, the upsampled pair and the cloud) carries the RGB header, stamp and `frame_id` exactly, so consumers
(RTAB-Map) can pair them with an exact-time policy. The ToF stamp is used only for the pair gate and the TF lookups.

Geometry comes from TF (the head URDF), never from a file. With `motion.compensate` the transform is
`lookupTransform(rgb_frame, t_rgb, tof_frame, t_tof, motion.fixed_frame)`; otherwise the static transform at `t_rgb`.
The ToF frame rate is measured with two lookups `motion.rate_window_s` apart ending at `t_tof`.

## Configuration

`/etc/ros2/head_rgbd_cpp/config.yaml`, path overridable with `HEAD_RGBD_CONFIG`. Unknown keys are rejected at every
level and any error exits 78 with one `<key>: <problem>` line per error.

| Key | Default | Meaning |
|---|---|---|
| `tof_depth_topic` | `/tof/depth/image_raw` | ToF depth input |
| `rgb_image_topic` | `/head_camera/mapping/image_raw/compressed` | RGB input (mapping stream) |
| `rgb_info_topic` | `/head_camera/mapping/camera_info` | RGB camera_info input |
| `output_namespace` | `/head_rgbd` | prefix of every output topic |
| `tof_calibration_file` | deployed: `/var/lib/ros2/head_rgbd_calibration/applied/tof_calibration.yaml` | ToF intrinsics (camera_calibration YAML) |
| `output.divisor` | 2 | output size = RGB size / divisor (1 or 2) |
| `sync.max_pair_dt_s` | 0.06 | largest ToF-RGB stamp difference of a pair, s (0 < v <= 0.2) |
| `sync.queue_size` | 10 | approximate-time queue length (>= 2) |
| `motion.compensate` | true | compensate the capture offset through the fixed frame |
| `motion.fixed_frame` | `odom` | TF frame that does not move with the robot |
| `motion.max_angular_rate_deg_s` | 10.0 | pairs with a faster ToF frame are dropped, deg/s |
| `motion.max_linear_rate_m_s` | 0.3 | same, m/s |
| `motion.rate_window_s` | 0.05 | window the rates are measured over, s |
| `motion.tf_timeout_s` | 0.05 | wait for a TF transform, s (0 = no wait) |
| `filter.jump_rel` | 0.05 | relative depth jump marking a flying pixel |
| `filter.jump_abs_mm` | 30 | minimum absolute jump, mm |
| `filter.min_valid_neighbours` | 3 | valid neighbours (of 8) a pixel needs |
| `depth.min_m` / `depth.max_m` | 0.1 / 4.0 | valid depth range, m |
| `upsampling.enabled` | true | guided-filter upsampling to the RGB size |
| `upsampling.radius` | 4 | guided filter radius, px |
| `upsampling.eps` | 0.001 | guided filter regularisation |
| `upsampling.min_support` | 0.5 | minimum filter support of a valid pixel |
| `cloud.enabled` | true | publish the coloured point cloud |
| `cloud.rate_hz` | 1.0 | point cloud rate, Hz (0 < v <= 10) |
| `lazy.idle_unsubscribe_s` | 5.0 | release the inputs this long after the last output subscriber left |
| `metrics_port` | 19117 | Prometheus port, `null` disables |

## Metrics

Served on `127.0.0.1:<metrics_port>` (`/metrics`), plus `robot_node_info` and the `process_*` series.

| Series | Type | Meaning |
|---|---|---|
| `head_rgbd_pairs_published_total` | counter | registered pairs published |
| `head_rgbd_pairs_dropped_total{reason}` | counter | synchronised sets dropped, by reason (see below) |
| `head_rgbd_pair_dt_seconds` | histogram | absolute ToF-RGB stamp difference of every synchronised set |
| `head_rgbd_process_seconds` | histogram | processing time of one set (drops included) |
| `head_rgbd_valid_ratio` | gauge | fraction of registered depth pixels holding a depth, last published pair |

Drop logs are throttled to one per 5 s.

## Lazy inputs and focus lock

A 1 s timer counts the subscribers of the output publishers. With at least one, the node creates its three input
subscriptions; with none for `lazy.idle_unsubscribe_s` it destroys them. Idle, the node therefore subscribes to nothing:
head_camera_cpp sees no mapping subscriber, keeps its continuous autofocus and releases the focus lock it takes (about
1 m) while the mapping stream is consumed. The first subscriber of any output takes the lock again within about 1 s.

## No placeholder data

Nothing is published for a set that cannot be registered from real data; the set is dropped and counted in
`head_rgbd_pairs_dropped_total{reason}`:

| reason | Dropped when |
|---|---|
| `no_calibration` | `tof_calibration_file` is missing or invalid (logged once at start; the node keeps running and publishes nothing) |
| `size_mismatch` | the ToF image size differs from the calibration |
| `bad_input` | the ToF image is not 16UC1 or its data is shorter than its size |
| `no_camera_info` | the RGB camera_info has an all-zero or non-positive K, no size, non-plumb_bob D, or a size that cannot be scaled |
| `pair_dt` | the ToF and RGB stamps differ by more than `sync.max_pair_dt_s` |
| `motion` | the ToF frame moves faster than `motion.max_angular_rate_deg_s` or `motion.max_linear_rate_m_s` |
| `no_tf` | a TF lookup fails within `motion.tf_timeout_s` |

A JPEG that does not decode skips the upsampled depth or the cloud of that set (logged, throttled); colour is never
made up.

## Upsampling

On by default (`upsampling.enabled: true`). The registered depth (288x162) is nearest-upsampled by 2 and smoothed along the
edges of the grey RGB image in masked normalised form, so the depth edges of obstacles and drops follow the RGB edges at
camera resolution (pinned: pixels 2 px or more from the guide edge read the box or floor depth within 5 mm) while holes never bleed zeros and a no-return area
wider than `radius` stays empty. RGB only moves where an existing depth edge lies; texture on flat floor (board, tile
seams, shadows) is not copied into depth. Tuning: the shipped `radius` 4, `eps` 0.001, `min_support` 0.5 are validated
by `test/core/test_upsample.cpp` (`BoxEdgeFollowsTheGuideEdge`, `TexturedFlatFloorStaysFlat`, `LargeHoleStaysEmpty`);
none needed changing.

Consumers: `/head_rgbd/depth_upsampled/image_raw` (16UC1 at the RGB size, with the RGB
camera_info, one header per pair) is the edge input for the ground guidance of `ground_estimator_cpp`; any subscriber of the topic is a consumer. Cost: one extra guided-filter pass per set, visible in
`head_rgbd_process_seconds`.

## Methodology

Calibration, synchronisation, registration, filtering, upsampling and validation (with targets and a results table) are
written down in [`docs/head-rgbd-fusion-methodology.md`](../../docs/head-rgbd-fusion-methodology.md). Defaults in short:

- Output 288x162 (`output.divisor` 2 of the 576x324 mapping image), registered by an own z-buffer with footprint splat.
- Pairing: ApproximateTime, `sync.max_pair_dt_s` 0.06; the whole set is stamped with the RGB stamp (exact pairs).
- Motion: tf2 time travel through `odom`; pairs above 10 deg/s or 0.3 m/s are dropped.
- Jump-edge filter 30 mm / 5 % / 3 valid neighbours; depth range 0.1-4.0 m.
- Upsampling on (masked guided filter, radius 4, `eps` 0.001, `min_support` 0.5); cloud 1 Hz.
- Calibration source: the deployed `tof_calibration_file` is the applied state file `/var/lib/ros2/head_rgbd_calibration/applied/tof_calibration.yaml`
  (not under `/etc/ros2`). It is written by Apply on the web UI Calibrate page (RGB-D section) and survives Ansible
  deploys. When the applied dir is empty, the deploy seeds it from the committed `nodes/head_rgbd_cpp/config/tof_calibration.yaml`
  (if present); with no file the node publishes nothing. Apply restarts the node so it re-reads the file.
- Geometry only from the head URDF; intrinsics from `tof_calibration.yaml` and the RGB camera_info, valid only at the
  locked `lens_position`. CPU target 0.2 core p95.

## Test

`scripts/cpp-node-test.sh head_rgbd_cpp`
