# stereo_depth

**Purpose:** Real depth from the two IMX219 cameras on the Pi 5 (left on cam0, right on cam1). The node pairs the
synced left and right images, rectifies them, runs OpenCV StereoSGBM and publishes depth on the same contract as the
Arducam ToF (`16UC1`, millimetres, `0` = invalid), so the web UI Head tab shows it unchanged. It never publishes
fake depth. Client only.

**Audience:** Mid-level Python developer with ROS2 (rclpy, sensor_msgs, message_filters).

## Topics

| Direction | Topic (default) | Type | Notes |
|---|---|---|---|
| in | `/overview_camera/image_raw` | `sensor_msgs/Image` | Left camera. It keeps the overview topic so every overview consumer keeps working. |
| in | `/stereo/right/image_raw` | `sensor_msgs/Image` | Right camera. |
| out | `/stereo/depth/image_raw` | `sensor_msgs/Image` `16UC1` | Depth in mm in the rectified left image. `0` means no reading. |
| out | `/stereo/camera_info` | `sensor_msgs/CameraInfo` | The rectified left camera: `P` from `stereo_left.yaml`, `K = P[:, :3]`, zero distortion, identity `R`. |
| out | `/stereo/disparity` | `stereo_msgs/DisparityImage` | Only with `publish_disparity: true`. `32FC1` pixels; unmatched pixels are negative (below `min_disparity`). |

The depth, the camera_info and the disparity of a pair share one header: the **left image's stamp** and `frame_id`
(`stereo_left_optical_frame`). No TF is published for that frame until the mount is measured.

Input images may be `mono8`, `bgr8`, `rgb8`, `bgra8` or `rgba8` (camera_ros `RGB888` gives `bgr8`). They are read as
numpy views of `Image.data` (no cv_bridge), padded rows included.

## Pipeline

1. `message_filters.ApproximateTimeSynchronizer` (queue 5, slop `sync_slop_s`) pairs the two image topics
   (sensor-data QoS). libcamera SyncMode keeps the two sensors in step; the slop is the gate.
2. **Rate cap:** pairs beyond `max_rate_hz` are dropped before any work. A pair up to 10% of a period early still
   takes its slot (camera jitter), but over any run the published rate never exceeds `max_rate_hz`.
3. Grey conversion, then `cv2.remap` with maps built once per calibration by `cv2.initUndistortRectifyMap(...,
   CV_16SC2)`.
4. `cv2.StereoSGBM` in `MODE_SGBM_3WAY`, `minDisparity 0`. Its fixed-point output is divided by 16.
5. `depth.py: disparity_to_depth_mm` (restored from the retired node): `Z = fx * B / d`, with `fx = P_left[0]` and
   `B = -P_right[3] / P_right[0]`. The result is rounded to mm and clamped to uint16. A pixel is `0` when `d` is not
   finite, `d <= 0` (unmatched: SGBM marks those `-1`), or `Z` is outside `[min_depth_m, max_depth_m]`.

WLS filtering is not used: it needs `opencv-contrib`, and the 5 fps budget has no room for it.

## No placeholder data

Nothing is published for a pair in any of these cases. Each one is logged (throttled to one line per 10 s per case)
and counted in `stereo_depth_frames_dropped_total{reason}`:

| reason | When |
|---|---|
| `skew` | The left and right stamps differ by more than `sync_slop_s`. |
| `no_calibration` | The calibration files are missing or invalid. The log line names the file and the reason. |
| `rate_limit` | The pair arrived before the next `max_rate_hz` slot. Counted only, not logged: it is the cap working. |
| `bad_image` | An image is empty, truncated or has an unsupported encoding. |
| `size_mismatch` | The two images differ in size, or differ from the calibrated size. |
| `no_valid_pixels` | Matching gave no valid depth pixel at all, for example a covered lens or a blank wall. |

**Calibration check:** every `calibration_check_s` the node compares the files' inode, mtime and size with the last
check. A calibration that appears is loaded, and a changed one is reloaded, so a new calibration needs no restart
or redeploy. Files that become invalid stop the depth output until they are fixed.

## Calibration files

These are the `stereo-calibration-files` contract (`stereo_depth/calibration.py`). The calibration tool writes them
and this node reads them. They live in `calibration_dir` (default `/var/lib/ros2/camera_calibration`), which is host
state that survives redeploys:

- `stereo_left.yaml` and `stereo_right.yaml`: ROS camera_info YAML, as written by `camera_calibration` /
  `camera_info_manager`, with `image_width`, `image_height`, `camera_name`, `camera_matrix`, `distortion_model`
  (`plumb_bob` or `rational_polynomial`), `distortion_coefficients`, `rectification_matrix` and `projection_matrix`.
  camera_ros reads the same files through `camera_info_url`.
- `stereo.yaml`: `image_width`, `image_height`, `R` (3x3), `T` (3x1, metres), `Q` (4x4) and `rms` (px).

`load_stereo_calibration(dir)` returns the calibration, or `None` with the reason. It rejects:

- a missing or non-mapping file, or a matrix of the wrong shape;
- non-finite values, or a focal length in K or P that is not positive;
- a distortion coefficient count that does not fit the model;
- image sizes that differ between the three files;
- a rectified pair that does not share fx, fy, cx and cy;
- a left `P[3]` that is not 0, or a right `P[3] = -fx*B` that is not negative;
- a baseline from P that is more than 2% off `|T|`.

`write_stereo_calibration(dir, calibration)` writes all three files atomically (temp file plus rename).
`stereo_calibration_from_opencv(...)` builds the model from `cv2.stereoCalibrate` / `cv2.stereoRectify` outputs.

## Calibration procedure

`python3 -m stereo_depth.calibrate` (`stereo_depth/calibrate.py`) captures board pairs and writes the three files
above. Config: `STEREO_CALIBRATE_CONFIG`, default `config/calibrate.yaml` (keys: `mode` capture or solve, `board`
chessboard or charuco, `squares_x`, `squares_y`, `square_size_m`, `marker_size_m`, `dictionary`, `capture_count`,
`capture_interval_s`, `image_dir`, `output_dir`, `min_pairs`, `rms_limit_px`, the two image topics and
`sync_slop_s`).

1. Mount both cameras rigidly (the calibration is only valid for this mount; the IMX219 fixed lenses do not drift).
2. Print the board flat and stiff (default: chessboard 9x6 squares, 25 mm). Measure the printed square edge and set
   `square_size_m`. For a ChArUco board also set `marker_size_m` and `dictionary`; at 480x360 its markers must be at
   least about 15 px in the image (for example 7x5 squares of 35 mm with 26 mm markers at 0.6 m).
3. Capture with `overview_camera` running (the tool subscribes to `/overview_camera/image_raw` and
   `/stereo/right/image_raw`). On the client:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /opt/ros2-nodes/stereo_depth/venv/bin/activate
   cd /opt/ros2-lerobot-swerve-platform/nodes/stereo_depth
   cp config/calibrate.yaml /tmp/calibrate.yaml          # edit board, mode: capture
   STEREO_CALIBRATE_CONFIG=/tmp/calibrate.yaml python3 -m stereo_depth.calibrate
   ```

   A pair is saved (`image_dir/pair_NNNN_left.png` / `_right.png`) only when the board is found in both images and
   at most once per `capture_interval_s`. Move and tilt the board between saves: near and far, all image corners,
   tilts up to about 30 degrees, always fully inside both images.
4. Solve: set `mode: solve` and run the same command. It runs `cv2.calibrateCamera` per camera,
   `cv2.stereoCalibrate` with `CALIB_FIX_INTRINSIC` and `cv2.stereoRectify` (alpha 0, zero disparity), and prints the
   stereo RMS, baseline and focal length.
5. Check the RMS. Above `rms_limit_px` (default 0.5 px) nothing is written and the tool exits 1: recapture with a
   flatter, sharper board covering more of the image. Fewer than `min_pairs` usable pairs also exits 1.
6. No restart is needed for this node: it re-checks the files every `calibration_check_s` and loads the new
   calibration. `overview_camera` reads `stereo_left.yaml` / `stereo_right.yaml` for its `camera_info_url` only at
   launch, so restart it (`sudo systemctl restart ros2-overview_camera`) to publish the calibrated camera_info.

The written files pass both readers: this node's `load_stereo_calibration` and the overview_camera launch gate (a
12-element `P` with non-zero focal terms).

## Benchmark

`python3 -m stereo_depth.bench` (`stereo_depth/bench.py`) times a grid of sizes, `num_disparities`, matchers (`bm`
StereoBM, `sgbm_3way`, `sgbm`) and thread counts on a synthetic textured pair, and prints the median wall time per
pair and the cores used at `target_fps` (process CPU time per pair times the rate). Config: `STEREO_BENCH_CONFIG`,
default `config/bench.yaml` (`sizes`, `num_disparities`, `matchers`, `threads`, `block_size`, `runs`,
`target_fps`). Run it on the client with the same two `source` lines and `cd` as above:

```bash
python3 -m stereo_depth.bench
```

Record the client table in this README (CPU estimate) once it has run on the Pi.

## Config

The config is the YAML file named by `STEREO_DEPTH_CONFIG` (default `/etc/ros2/stereo_depth/config.yaml`). It is the
`stereo-depth-config` contract, validated by pydantic (`stereo_depth/config.py`, `DepthConfig`: frozen,
`extra="forbid"`). Unknown keys and invalid values exit with status 1. An empty file gives the defaults.

| Key | Default | Allowed | Meaning |
|---|---|---|---|
| `left_image_topic` | `/overview_camera/image_raw` | non-empty | Left camera image. |
| `right_image_topic` | `/stereo/right/image_raw` | non-empty | Right camera image. |
| `depth_topic` | `/stereo/depth/image_raw` | non-empty | Depth output. |
| `camera_info_topic` | `/stereo/camera_info` | non-empty | camera_info output. |
| `publish_disparity` | `false` | bool | Also publish the DisparityImage. |
| `disparity_topic` | `/stereo/disparity` | non-empty | DisparityImage output. |
| `calibration_dir` | `/var/lib/ros2/camera_calibration` | path | Directory of the three calibration files. |
| `calibration_check_s` | `5.0` | > 0 | Calibration file check period. |
| `num_disparities` | `48` | multiple of 16, > 0 | Disparity range. The near limit is `fx * B / num_disparities`. |
| `block_size` | `5` | odd, 1..11 | SGBM block size. |
| `p1` | `200` | > 0, < `p2` | Penalty for a disparity change of 1 (`8 * blockSize^2`). |
| `p2` | `800` | > `p1` | Penalty for larger changes (`32 * blockSize^2`). |
| `uniqueness_ratio` | `10` | >= 0 | The best match must beat the second best by this many percent. |
| `speckle_window_size` | `100` | >= 0 | Largest speckle region that gets invalidated. `0` turns the filter off. |
| `speckle_range` | `2` | >= 0 | Disparity variation allowed within one region. |
| `disp12_max_diff` | `1` | >= -1 | Left-right check tolerance in px. `-1` turns the check off. |
| `pre_filter_cap` | `31` | 1..63 | Pre-filter clip. It absorbs small gain differences between the cameras. |
| `min_depth_m` | `0.3` | > 0, < `max_depth_m` | Nearer depth becomes 0. |
| `max_depth_m` | `4.0` | > `min_depth_m` | Farther depth becomes 0. |
| `sync_slop_s` | `0.002` | > 0, <= 0.1 | Largest left/right stamp difference. |
| `max_rate_hz` | `5.0` | > 0, <= 30 | Depth rate cap. |
| `matcher_threads` | `2` | 1..4 | `cv2.setNumThreads`. Threads cut wall time, not CPU seconds. |
| `frame_id` | `stereo_left_optical_frame` | non-empty | Header frame of every output. |
| `metrics_port` | `19113` | 1..65535 or null | Prometheus `/metrics` on 127.0.0.1. `null` falls back to `METRICS_PORT`, then off. |

## Metrics

Node name `stereo_depth`, served through `ros2-metrics` on port 19113.

| Metric | Type | Meaning |
|---|---|---|
| `stereo_depth_frames_published_total` | counter | Pairs published (depth and camera_info). Its rate is the depth fps. |
| `stereo_depth_frames_dropped_total{reason}` | counter | Pairs that published nothing (reasons above). |
| `stereo_depth_matcher_seconds` | histogram | Rectify plus SGBM wall time per pair. |
| `stereo_depth_pair_skew_seconds` | histogram | Left/right stamp difference of every synced pair. |
| `stereo_depth_calibration_loaded` | gauge | 1 while a valid calibration is loaded. |

## CPU estimate

This estimate comes from the brief (`docs/plans/2026-10-13-stereo-depth-briefs/stereo-brief.md`, section 4). The M4
times were scaled x5 to the Pi 5 Cortex-A76, and the on-device benchmark replaces them once it has run.

- SGBM 3WAY at 480x360 with 48 disparities: about **0.4 core at 5 fps**, which fits the client's free 0.3-0.45 core.
  At 640x480 with 64 disparities it is about 0.7 core.
- Rectification (two `remap` calls plus grey conversion) and the transport of two images: under 0.1 core at 5 fps.
- `matcher_threads: 2` halves the wall time per pair (latency). It does not reduce the CPU seconds.
- The unit runs in the background cgroup tier: under load the depth rate drops, and the robot nodes keep their CPU.

## Code layout

- `stereo_depth/config.py` - `DepthConfig`, `load_config()` and `config_path_from_env()`.
- `stereo_depth/calibration.py` - the calibration models, loader and writer, `rectification_maps()`, `rectify()` and
  `calibration_signature()`.
- `stereo_depth/depth.py` - `disparity_to_depth_mm()`, `create_matcher()` and `compute_disparity()`.
- `stereo_depth/messages.py` - `image_to_grey()`, `depth_image_msg()`, `camera_info_msg()`, `disparity_msg()` and
  `stamp_ns()`. It needs no rclpy.
- `stereo_depth/node.py` - `StereoDepthNode` (sync, gates, rate cap, publish) and `run()`.
- `stereo_depth/calibrate.py` - the calibration tool: `CalibrateConfig`, board detection (`board_points()`),
  `solve()`, `write_calibration()` (RMS gate), pair files and `CaptureNode`.
- `stereo_depth/bench.py` - the matcher benchmark: `BenchConfig`, `run_bench()` and `format_table()`.
- `config/calibrate.yaml`, `config/bench.yaml` - the tool defaults.
- `stereo_depth/__main__.py` - the entry point, `python3 -m stereo_depth`.

## Dependencies

- `numpy` and `opencv-python-headless`, pinned like mcp_server, whose aarch64 wheels already deploy on the client.
  `numpy` is declared explicitly because `sensor_msgs` imports it at runtime (the numpy dependency guard).
- `pydantic`, `pyyaml`, `prometheus-client`, and `ros2-metrics` (path dependency on `shared/ros2_metrics`).
- `rclpy`, `message_filters`, `sensor_msgs`, `stereo_msgs` and `std_msgs` come from the ROS 2 Jazzy install on the
  robot.

## Tests

Run in this directory:

```bash
uv run pytest tests -v
uv run poe lint
```

`tests/conftest.py` replaces rclpy, message_filters and the message packages with doubles, so no ROS 2 install is
needed. It also builds the synthetic pairs: a random texture, and the same texture shifted by a known disparity.

- `tests/test_config.py` - defaults, every key, frozen model, rejection of unknown keys and invalid values (multiple
  of 16, odd block, p1 < p2, depth range, slop, rate), file and YAML errors, env path.
- `tests/test_calibration.py`:
  - round trip of the written files and the ROS camera_info layout;
  - loading a `cameracalibrator` file;
  - an OpenCV `stereoRectify` rig that keeps its baseline;
  - every rejection rule, with the reason;
  - CV_16SC2 maps, identity rectification, row alignment;
  - the change signature.
- `tests/test_depth.py`:
  - the restored disparity to mm conversion;
  - SGBM on synthetic shifted pairs, with depth within 2% of `f*B/d`;
  - a zero left border, a textureless pair and unrelated images giving no depth;
  - the out-of-range clamp and the matcher settings.
- `tests/test_messages.py` - grey decoding per encoding (padded rows, unsupported, truncated), the 16UC1 layout and
  header, the rectified camera_info, the DisparityImage and the stamp helper.
- `tests/test_node.py`:
  - synchronizer topics and slop, and the calibration timer;
  - publishing under the left header, with values within 2% and the metrics;
  - no publish without a calibration or with an invalid one, and pick-up and reload of new files;
  - the skew gate, size mismatches, bad encodings and all-invalid pairs;
  - the rate cap (jitter and a long run), the optional disparity and the matcher threads;
  - `run()` and `main()`.
- `tests/test_calibrate.py` (boards rendered with known K, zero distortion, R and a 60 mm T):
  - the repo config, env path and config rejections (unknown key, missing file, marker size, dictionary, squares);
  - chessboard object points, board detection in both images (chessboard and ChArUco) and no board;
  - the solve recovering the baseline within 1% with RMS under 0.5 px (chessboard and ChArUco), and too few pairs;
  - the written files loading in `load_stereo_calibration` and passing the overview_camera launch gate;
  - the RMS gate refusing to write;
  - saving and loading pairs (a half pair is skipped), `run_solve()` with and without pairs;
  - the capture node: topics and slop, the interval, board and count gates, undecodable images.
- `tests/test_bench.py` - the repo grid, env path, config rejections, the shifted synthetic pair, each matcher kind,
  one timed cell, the table shape and `main()`.

## Deploy

Ansible deploys the node on the client as a native uv node with `src_shared` (for `ros2-metrics`), in the
background cgroup tier. Deploy only through the script:

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client stereo_depth 2>&1 | tee .logs/deploy-client.log
```

Until both cameras are mounted and calibrated, the node runs and publishes nothing, and
`stereo_depth_calibration_loaded` stays 0.
