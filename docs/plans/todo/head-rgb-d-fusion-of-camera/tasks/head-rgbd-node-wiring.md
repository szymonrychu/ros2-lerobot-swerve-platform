---
title: Head RGBD node wiring
status: todo
repo: ros2-lerobot-sverve-platform
value: "A running fusion node turns live ToF and RGB into an exact-stamp registered RGB-D pair and cloud, only while someone uses them and only from real data."
files: [nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp, nodes/head_rgbd_cpp/src/head_rgbd_node.cpp, nodes/head_rgbd_cpp/src/main.cpp, nodes/head_rgbd_cpp/test/test_node.cpp, nodes/head_rgbd_cpp/CMakeLists.txt, nodes/head_rgbd_cpp/README.md]
verify: scripts/cpp-node-test.sh head_rgbd_cpp
depends_on: [tof-jump-edge-filter, rgbd-motion-compensation, rgbd-guided-upsampling, rgbd-cloud-builder, head-camera-mapping-publish]
owns: [head-rgbd-topics]
uses: [head-rgbd-config, head-rgbd-tof-filter, head-rgbd-registration, head-rgbd-motion, head-rgbd-upsampling, head-rgbd-cloud, head-camera-mapping-topics]
model: sonnet
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head RGBD node wiring

## Requirement

- `HeadRgbdNode` (rclcpp, executable `head_rgbd_node` from `src/main.cpp`, node name `head_rgbd`; D1, D2, D5, D6):
  - Startup: loads the config; loads `tof_calibration_file` with `camera_calibration_parsers::readCalibration`; a missing or invalid file is logged once and the node runs but publishes nothing (counted `no_calibration`).
  - Publishers (best_effort, depth 1, under `output_namespace`): `depth/image_raw` (16UC1), `depth/camera_info`, `rgb/image_raw/compressed`, `rgb/camera_info`, `points` (only when `cloud.enabled`), `depth_upsampled/image_raw` and `depth_upsampled/camera_info` (only when `upsampling.enabled`).
  - Lazy inputs (U4): a 1 s timer checks the publishers' subscription counts; with at least one subscriber it creates `message_filters::Subscriber`s for the ToF depth (`sensor_msgs/Image`), the RGB image (`sensor_msgs/CompressedImage`) and the RGB camera_info (best_effort) synchronised by `ApproximateTime` (queue `sync.queue_size`, `setMaxIntervalDuration(sync.max_pair_dt_s)`); with no subscriber for `lazy.idle_unsubscribe_s` it destroys them, so head_camera_cpp sees no mapping subscriber and releases the focus lock.
  - Per synchronised set: drop (counted by reason, nothing published) when: the ToF image size differs from the calibration (`size_mismatch`); the RGB camera_info has an all-zero K (`no_camera_info`); `evaluate_pair` rejects (`pair_dt`, `motion`); any TF lookup fails within `motion.tf_timeout_s` (`no_tf`). Otherwise: `filter_jump_edges` -> `T_rgb_tof` from `tf2_ros::Buffer::lookupTransform(rgb_frame, t_rgb, tof_frame, t_tof, fixed_frame)` (or the static lookup at t_rgb when `motion.compensate` is false) and the ToF frame rate from two lookups `motion.rate_window_s` apart -> `register_depth` into the RGB camera_info scaled by `output.divisor` -> publish depth, depth camera_info (scaled K, P, same D, frame of the RGB header), the RGB CompressedImage unchanged and the RGB camera_info unchanged, all with the RGB header (stamp and frame_id); when upsampling is enabled, decode the RGB, `guided_upsample`, publish at the RGB camera_info size; when the cloud decimator is due and `points` has subscribers, `decode_rgb_for_depth` + `build_cloud`.
  - Metrics via `ros2_common_cpp::Metrics` on `metrics_port`: `head_rgbd_pairs_published_total`, `head_rgbd_pairs_dropped_total{reason}`, `head_rgbd_pair_dt_seconds` (histogram), `head_rgbd_process_seconds` (histogram), `head_rgbd_valid_ratio` (gauge), plus the process series. Drop logs are throttled (5 s).
- CMakeLists: the node library and `head_rgbd_node` executable, `test/test_node.cpp` gtest with rclcpp, install rules for the executable and `config/`.
- Tests first in `test/test_node.cpp` (in-process publishers, a `tf2_ros::Buffer` fed with static and timed transforms, the test calibration `test/data/tof_calibration.yaml`), seen failing: with no output subscriber the node does not subscribe its inputs (the RGB topic shows zero subscriptions) and does after a subscriber appears, and releases them after the idle time; a synchronised ToF + RGB + camera_info set with valid TF yields one depth, depth camera_info, RGB and RGB camera_info, all with the RGB stamp and frame, depth 288x162 for a 576x324 camera_info, depth values matching `register_depth` on the same input, and the RGB payload byte-identical; nothing is published (and the matching drop counter rises) without TF, without the calibration file, with a zero-K camera_info, with a wrong ToF size, or during a 20 deg/s motion; the cloud is published at most at `cloud.rate_hz`; the upsampled topics exist only when enabled.
- `nodes/head_rgbd_cpp/README.md` (node contract): sections `## Topics` (subscriptions, publications, QoS, the exact-stamp rule), `## Configuration` (table of every key), `## Metrics`, `## Lazy inputs and focus lock`, `## No placeholder data` (each drop reason).

## Acceptance conditions

- a1: inputs are subscribed only while an output has a subscriber and are released afterwards [evidence: command: cd nodes/head_rgbd_cpp && uv run --frozen pytest test/test_node.cpp -q]
- a2: a synchronised set publishes registered depth, its camera_info and the unchanged RGB pair, all with the RGB stamp [evidence: command: cd nodes/head_rgbd_cpp && uv run --frozen pytest test/test_node.cpp -q]
- a3: nothing is published without TF, calibration, camera_info, the right ToF size, or during fast motion, each drop counted [evidence: command: cd nodes/head_rgbd_cpp && uv run --frozen pytest test/test_node.cpp -q]
- a4: the cloud respects its rate and upsampled outputs exist only when enabled [evidence: command: cd nodes/head_rgbd_cpp && uv run --frozen pytest test/test_node.cpp -q]
- a5: the README documents topics, configuration, metrics, lazy inputs and the placeholder rules [evidence: doc-section: nodes/head_rgbd_cpp/README.md#No placeholder data]
- a6: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

## Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp`
- `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp`
- `nodes/head_rgbd_cpp/src/main.cpp`
- `nodes/head_rgbd_cpp/test/test_node.cpp`
- `nodes/head_rgbd_cpp/CMakeLists.txt`
- `nodes/head_rgbd_cpp/README.md`

## Interfaces

- Topics `/head_rgbd/depth/image_raw`, `/head_rgbd/depth/camera_info`, `/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/rgb/camera_info`, `/head_rgbd/points`, `/head_rgbd/depth_upsampled/*` (part `head-rgbd-topics`), consumed by the RTAB-Map epic's uplink and by future web UI / agent views.
- Executable `head_rgbd_cpp head_rgbd_node`, env `HEAD_RGBD_CONFIG`.

## Expected outcome

`ros2 run head_rgbd_cpp head_rgbd_node` publishes exact-stamp RGB-D pairs from live inputs and nothing otherwise.

## Steps

- Write test_node.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement the node, main and CMake targets; rerun until `PASS`.
- Write the README; commit `feat(head_rgbd_cpp): fusion node wiring`.

## Notes

- The node never reads the extrinsics file: geometry comes from TF (head URDF, design D4).
- The mapping stream (head-camera-mapping-publish) publishes best_effort depth 1; match it.
- Raw RGB is never subscribed (UDPv4 fragment drops, design Constraints).
