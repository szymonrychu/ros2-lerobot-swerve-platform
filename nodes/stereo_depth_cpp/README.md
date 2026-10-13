# stereo_depth_cpp

rclcpp port of the [stereo_depth](../stereo_depth/README.md) node: synced left/right IMX219 images, OpenCV StereoSGBM
(3-way), 16UC1 depth in millimetres. It has the same contract as the Python node: same topics, QoS, config file,
drop reasons and metric names, so the cut-over is enabling one node type and disabling the other.

Why: the Python node pays rclpy deserialization of two 480x360 bgr8 images (about 518 KB each) per pair, at the camera
rate, before the rate cap drops most of them, plus numpy glue and the `tobytes()` of the output. On the CPU-bound
client Pi 5 that is the part a C++ node removes. SGBM, `remap` and `cvtColor` were already OpenCV C++ code.

It runs as a standalone process (`stereo_depth_node`) in the background cgroup tier (design decision D3), so the robot
keeps CPU priority. The same class is also an `rclcpp_components` component (`stereo_depth_cpp::StereoDepthNode`), so
it can later be composed into `overview_container` for zero-copy raw frames.

Deployment is not part of this package: T2 registers the `stereo_depth_cpp` node type in Ansible and T8 cuts over.
Until then the Python `stereo_depth` node stays the deployed one.

## Contract

| | |
|---|---|
| Inputs | `left_image_topic` (default `/overview_camera/image_raw`) and `right_image_topic` (default `/stereo/right/image_raw`), `sensor_msgs/Image`, sensor-data QoS (best effort, keep last 5) |
| Pairing | `message_filters` ApproximateTime policy, queue 5, `setMaxIntervalDuration(sync_slop_s)`; each pair is re-checked against `sync_slop_s` |
| Depth | `depth_topic` (default `/stereo/depth/image_raw`), `sensor_msgs/Image` 16UC1, millimetres, 0 = invalid (REP 118, the ToF contract), little-endian |
| Camera info | `camera_info_topic` (default `/stereo/camera_info`): the rectified left camera (K from P, plumb_bob with zero distortion, identity R, P) |
| Disparity | `disparity_topic` (default `/stereo/disparity`), `stereo_msgs/DisparityImage` 32FC1, only when `publish_disparity: true` |
| Header | all outputs share one header: the left image's stamp and `frame_id` (default `stereo_left_optical_frame`) |
| Output QoS | reliable, keep last 5 |
| Rate | at most `max_rate_hz`; a pair up to `RATE_JITTER_FRACTION` (0.1) of a period early still takes the slot |

Per pair, in the Python node's order (`StereoDepthCore::process` in `src/pipeline.cpp`):

1. Stamp skew above `sync_slop_s`: dropped, reason `skew`.
2. No valid calibration: dropped, reason `no_calibration`.
3. Before the next rate slot: dropped, reason `rate_limit` (no work done, not logged).
4. An image that is not mono8, bgr8, rgb8, bgra8 or rgba8, is empty or is truncated: dropped, reason `bad_image`.
5. Left, right and calibration sizes differ: dropped, reason `size_mismatch`.
6. Grey, rectify (`cv::remap` with fixed-point maps), StereoSGBM, Z = f * T / d in mm.
7. No valid depth pixel: dropped, reason `no_valid_pixels`.
8. Otherwise depth, camera_info (and disparity) are published.

No placeholder data: nothing is published without a valid calibration, without a pair, or for a pair that failed any
gate. Every drop is counted, and all but `rate_limit` are logged (throttled to one line per 10 s).

The calibration is the one the Python calibration tool writes (`stereo_left.yaml`, `stereo_right.yaml`, `stereo.yaml`
in `calibration_dir`, see the [stereo_depth README](../stereo_depth/README.md)). The files are validated with the same
rules as the pydantic models (sizes, matrix shapes, finite values, focal lengths, distortion coefficient count, a
rectified pair sharing fx, fy, cx and cy, left P[3] = 0, right P[3] < 0, baseline within 2 % of |T|). Every
`calibration_check_s` the file signature (inode, mtime, size) is compared: a calibration that appears is loaded, a
changed one is reloaded and an invalid one is dropped, with no restart.

## Config

The same YAML file and keys as the Python node: `STEREO_DEPTH_CONFIG`, default `/etc/ros2/stereo_depth/config.yaml`,
so the Ansible config block is shared. `src/config.cpp` mirrors `nodes/stereo_depth/stereo_depth/config.py` field by
field: unknown keys are rejected, and so are values out of the pydantic bounds, a `num_disparities` that is not a
multiple of 16, an even `block_size`, `p1 >= p2`, `min_depth_m >= max_depth_m`, and empty topics or `frame_id`
(both are stripped). An empty file gives the defaults. Any error is printed (`config error: <path>: <key>: <problem>`,
every error at once) and the process exits **78**, which the systemd unit treats as "do not restart".

| Key | Default | Rule |
|---|---|---|
| `left_image_topic` | `/overview_camera/image_raw` | non-empty |
| `right_image_topic` | `/stereo/right/image_raw` | non-empty |
| `depth_topic` | `/stereo/depth/image_raw` | non-empty |
| `camera_info_topic` | `/stereo/camera_info` | non-empty |
| `publish_disparity` | `false` | bool |
| `disparity_topic` | `/stereo/disparity` | non-empty |
| `calibration_dir` | `/var/lib/ros2/camera_calibration` | string |
| `calibration_check_s` | 5.0 | > 0 |
| `num_disparities` | 48 | > 0, multiple of 16 |
| `block_size` | 5 | 1..11, odd |
| `p1`, `p2` | 200, 800 | > 0, p1 < p2 |
| `uniqueness_ratio` | 10 | >= 0 |
| `speckle_window_size` | 100 | >= 0 |
| `speckle_range` | 2 | >= 0 |
| `disp12_max_diff` | 1 | >= -1 |
| `pre_filter_cap` | 31 | 1..63 |
| `min_depth_m`, `max_depth_m` | 0.3, 4.0 | > 0, min < max |
| `sync_slop_s` | 0.002 | > 0, <= 0.1 |
| `max_rate_hz` | 5.0 | > 0, <= 30 |
| `matcher_threads` | 2 | 1..4 (`cv::setNumThreads`) |
| `frame_id` | `stereo_left_optical_frame` | non-empty |
| `metrics_port` | 19113 | 1..65535; `null` falls back to `METRICS_PORT`, then off |

## Metrics

Served by `ros2_common_cpp::Metrics` on `127.0.0.1:<metrics_port>/metrics`, with the Python node's names and buckets,
so Grafana panels and alerts carry over:

| Metric | Type | Meaning |
|---|---|---|
| `stereo_depth_frames_published_total` | counter | depth frames published (depth and camera_info) |
| `stereo_depth_frames_dropped_total{reason}` | counter | pairs that published nothing; reason is one of the six above |
| `stereo_depth_matcher_seconds` | histogram | rectify plus StereoSGBM wall time per pair (buckets 0.01 .. 2 s) |
| `stereo_depth_pair_skew_seconds` | histogram | left/right stamp difference of every synced pair (buckets 0.1 ms .. 0.1 s) |
| `stereo_depth_calibration_loaded` | gauge | 1 while a valid calibration is loaded |
| `robot_node_info{node="stereo_depth"}`, `robot_node_start_time_seconds{node}` | gauge | from ros2_common_cpp |

## Parity with the Python node

`test/gen_fixtures.py` runs the Python node's own functions (in the `nodes/stereo_depth` uv environment) and writes
`test/fixtures/`:

- `left.png`, `right.png`: the shifted synthetic pair of `nodes/stereo_depth/tests/conftest.py` (20 px disparity).
- `ideal/` and `distorted/`: calibration files written by `write_stereo_calibration` (the conftest ideal pair, and the
  same pair with lens distortion, a different raw K and small rectification rotations so `remap` does real work),
  plus the Python depth (`depth_mm.png`) and SGBM disparity (`disparity_fp.png`, fixed point + 32768) of the pair sent
  as bgr8 Images through `image_to_grey`, `rectify`, `compute_disparity` and `disparity_to_depth_mm`.
- `conversion.yaml`: `disparity_to_depth_mm` on hand-made rows (NaN, inf, ranges, rounding, uint16 clamp).
- `config.yaml`: the DepthConfig defaults, accepted texts with their parsed values, and every text the pydantic model
  rejects (the generator asserts each one raises `ConfigError`).
- `calibration_invalid.yaml`: 30 calibration file sets the Python loader rejects (each one checked).

The gtests then assert that the C++ depth is within 1 mm of the Python depth on valid pixels with an identical invalid
mask (whole path, both calibrations, every encoding), that the C++ conversion of the Python disparity equals the Python
depth, and that the C++ config and calibration loaders accept and reject exactly what Python does. The parity holds
across OpenCV versions: the fixtures come from the pinned pip OpenCV (`opencv-python-headless` 4.13) and the C++ build
uses Ubuntu 24.04's `libopencv-dev` (4.6).

Regenerate after a change to the Python node, and check that the committed fixtures are current:

```bash
cd nodes/stereo_depth
uv run --frozen python ../stereo_depth_cpp/test/gen_fixtures.py          # rewrite test/fixtures
uv run --frozen python ../stereo_depth_cpp/test/gen_fixtures.py --check  # exit 1 naming stale/missing/extra files
```

## Bench

`scripts/bench_compare.py` (settings in `scripts/bench_compare.yaml`, or `BENCH_COMPARE_CONFIG`) runs the Python and
the C++ depth paths on the same input for `pairs` pairs and prints the process CPU seconds per pair of each. Both paths
do what the node does per Image pair: decode both images to grey, rectify, SGBM, depth conversion and the 16UC1 output
bytes. The Python path runs in-process; the C++ path is the package's `stereo_depth_bench` binary, which reads a job
file written by the script (`cpp_command`, by default `ros2 run stereo_depth_cpp stereo_depth_bench` in the deployed
colcon workspace `/opt/ros2-ws`). On the client, once the package is built there:

```bash
cd nodes/stereo_depth && uv run --frozen python ../stereo_depth_cpp/scripts/bench_compare.py
# bench_compare: 30 pairs, bgr8, left.png/right.png, .../fixtures/ideal/calibration
# python cpu_s_per_pair=... wall_s_per_pair=... pairs=30 valid_pixels=...
# cpp cpu_s_per_pair=... wall_s_per_pair=... pairs=30 valid_pixels=...
# cpp/python cpu ratio=...
```

On the M4 Mac (Python native, C++ in the arm64 container, distorted calibration, 2 matcher threads) both paths cost
about 0.014 CPU s per pair (ratio 0.97) with an identical valid pixel count: the compute is OpenCV in both. The bench
cannot include the rclpy deserialization of the two incoming images per camera frame, which is the saving the port is
for; measure that on the robot with `rate(container_cpu_usage_seconds_total{id=~".*stereo_depth.*"}[5m])` before and
after the cut-over, once both cameras stream.

## Layout

| Path | What |
|---|---|
| `include/stereo_depth_cpp/config.hpp`, `src/config.cpp` | DepthConfig, `validate`, `load_config`, `load_config_or_exit` (exit 78) |
| `include/stereo_depth_cpp/depth.hpp`, `src/depth.cpp` | `create_matcher` (SGBM 3-way with the config), `compute_disparity`, `disparity_to_depth_mm` |
| `include/stereo_depth_cpp/calibration.hpp`, `src/calibration.cpp` | calibration file loading and validation, rectification maps, file signature |
| `include/stereo_depth_cpp/pipeline.hpp`, `src/pipeline.cpp` | `image_to_grey`, `RateCap`, `StereoDepthCore` (gates, matching, metrics) |
| `src/stereo_depth_node.cpp` | the rclcpp node: subscriptions, synchronizer, publishers, calibration timer; component registration |
| `src/bench_main.cpp` | `stereo_depth_bench`, the C++ side of the bench |
| `test/` | gtests, `fixtures.{hpp,cpp}` helpers, `gen_fixtures.py`, `fixtures/` |
| `scripts/bench_compare.{py,yaml}` | Python vs C++ CPU bench |

Everything except `stereo_depth_node.cpp` is the `stereo_depth_core` library, which does not link rclcpp. The
executables are `stereo_depth_node` (standalone, generated by `rclcpp_components_register_node`) and
`lib/stereo_depth_cpp/stereo_depth_bench`. The calibration and matcher-grid bench tools stay in the Python node
(`python3 -m stereo_depth.calibrate`, `python3 -m stereo_depth.bench`).

## Tests

Build, gtests and the ament linters (clang-format with the root `.clang-format`, cppcheck, xmllint) run in docker on
`ros:jazzy-ros-base` (native arm64 on the Mac, amd64 in CI):

```bash
./scripts/cpp-node-test.sh stereo_depth_cpp              # build + gtests + lint
./scripts/cpp-node-test.sh stereo_depth_cpp --lint-only  # linters only
(cd nodes/stereo_depth && uv run --frozen python ../stereo_depth_cpp/test/gen_fixtures.py --check)
```

- `test_config.cpp`: the C++ keys are the Python fields in order; the defaults and every accepted text give the Python
  values; every Python rejection is rejected; error texts name the field; all errors are reported at once; a missing
  file is an error; an invalid or missing config file exits 78; a valid file from `STEREO_DEPTH_CONFIG` loads.
- `test_depth.cpp`: exact millimetres, NaN/inf/non-positive/below-min-disparity/out-of-range as 0, uint16 clamp and
  shape; the conversion rows and each case's Python disparity give the Python depth; the matcher carries every config
  value; a 20 px shift gives depth within 2 %, disparity in pixels, a zero left border, an all-zero textureless pair and
  zero depth outside the range.
- `test_calibration.cpp`: both Python-written calibrations load with the expected fx, baseline and size; every
  invalid set of `calibration_invalid.yaml` is rejected; the reason names the file; the single-file loaders throw;
  ideal maps are the identity and distorted maps move pixels; the signature tracks missing, unchanged and changed files.
- `test_node_logic.cpp`: every encoding decodes, rgb/bgr channel order, padded rows, unsupported/empty/truncated
  images; the rate cap with its jitter; no calibration, skew (checked first, takes no rate slot), size mismatch, bad
  image (after the rate slot), rate limit (never matched) and no valid pixels are dropped and counted with nothing to
  publish; whole-path depth parity within 1 mm and an identical mask for both calibrations and every encoding;
  calibration load, unchanged check, reload and drop on invalid files; the metric names, labels and buckets.

## Rollback to the Python node

The Python `stereo_depth` node stays in the repo and in `group_vars` (present, disabled after the cut-over). To roll
back, set `enabled: true` for `stereo_depth` and `enabled: false` for `stereo_depth_cpp` in
`ansible/group_vars/client.yml` and redeploy both with `./scripts/deploy-nodes.sh client stereo_depth stereo_depth_cpp`
(output to `.logs/`). Both read the same config block, calibration files and metrics port, so nothing else changes.
