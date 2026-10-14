# depthd

C++17 GPU depth pipeline for the Jetson Nano vision coprocessor. It captures the two IMX219 cameras, rectifies and
matches them on the GPU, post-processes the disparity, optionally fuses monocular depth, and publishes depth and
RGB over a cabled ZeroMQ link (wire format: `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`) to the
ROS 2 bridge on the Pi. This keeps stereo work off the saturated Pi 5 CPU.

## Layout

```
CMakeLists.txt        finds and links every dependency; globs src/*.cpp and test/test_*.cpp
include/depthd/       public headers (config.hpp: the config schema; capture.hpp, pairing.hpp: stereo capture; calibration.hpp, rectify.hpp: calibration and GPU rectification; matcher.hpp, depth.hpp: matching and depth)
src/                  library sources; main.cpp -> depthd and bench_main.cpp -> depthd-bench when present
config/depthd.yaml    the one example and default config (equals the struct defaults)
test/                 gtest binaries, one per test_*.cpp
```

A task adds files under `src/` and `test/` without touching CMakeLists.txt. The two mains are excluded from the
library glob. The build runs on the Jetson only (Ubuntu 20.04, L4T R32.6.1, gcc 8.4, CUDA 10.2, OpenCV 4.8 CUDA,
TensorRT 8.0.1); there is no ROS on the Jetson.

## Build and test (remote)

From the repository root:

```
D=${DEPTHD_DEV:-depthd-dev-$(git rev-parse --abbrev-ref HEAD | tr / -)}
rsync -azR --delete --exclude build/ jetson/depthd shared/ros2_common_cpp jetson@192.168.1.32:$D/ && ssh -o BatchMode=yes jetson@192.168.1.32 "cd ~/$D/jetson/depthd && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j3 && ctest --test-dir build --output-on-failure"
```

Each branch builds in its own remote directory (`~/depthd-dev-<branch>`), so parallel tasks never share a build tree
(a shared one corrupts `.d` dependency files). Override with `DEPTHD_DEV`.

Expected: `100% tests passed, 0 tests failed`.

## Capture

`depthd::StereoCapture` (`include/depthd/capture.hpp`) runs one GStreamer pipeline per IMX219.
`build_pipeline_description(config, side)` generates them (pinned by `test/test_capture_pipeline.cpp`):

```
left:  nvarguscamerasrc name=left_src sensor-id=0 sensor-mode=3 ee-mode=0 tnr-mode=1 aelock=false awblock=false
         ! video/x-raw(memory:NVMM),width=1640,height=1232,format=NV12,framerate=30/1 ! tee name=left_tee
       left_tee. ! queue max-size-buffers=2 leaky=downstream ! nvvidconv
         ! video/x-raw(memory:NVMM),width=640,height=480,format=GRAY8 ! appsink name=left_sink max-buffers=1 drop=true sync=false
       left_tee. ! queue name=rgb_queue max-size-buffers=1 leaky=downstream ! nvvidconv
         ! video/x-raw(memory:NVMM),width=640,height=480,format=I420 ! nvjpegenc name=rgb_jpeg quality=80
         ! appsink name=rgb_sink max-buffers=2 drop=true sync=false
right: nvarguscamerasrc name=right_src sensor-id=1 ... ! nvvidconv ! video/x-raw(memory:NVMM),...,format=GRAY8 ! appsink name=right_sink ...
```

- **Mode.** `capture.sensor_mode` maps to the Argus mode index (`imx219_sensor_mode`: 0 3264x2464@21, 1 3264x1848@28,
  2 1920x1080@30, 3 1640x1232@30, 4 1280x720@60, 5 1280x720@120); any other mode fails start naming
  `capture.sensor_mode`. Edge enhancement is off (`ee-mode=0`) and temporal noise reduction is fast (`tnr-mode=1`):
  sharpening halos and heavy TNR smear hurt matching.
- **GRAY8 at processing size** comes out of the VIC (`nvvidconv`) inside NVMM. With `capture.zero_copy: true` the
  NvBuffer is mapped into CUDA (`ExtractFdFromNvBuffer` -> `NvEGLImageFromFd` -> `cuGraphicsEGLRegisterImage` ->
  `cuGraphicsResourceGetMappedEglFrame`) and copied device to device into a pooled `cv::cuda::GpuMat`, so the NVMM
  buffer goes straight back to the camera and no frame touches the CPU. With `zero_copy: false` the caps are system
  memory and the frame is uploaded. Both paths are tested on the cameras; T16 measures which is faster. The pool holds
  at most 16 GPU frames per camera (no per-frame `cudaMalloc`); a consumer that holds more starves it and frames drop.
- **RGB JPEG.** The left pipeline tees the NV12 NVMM stream into `nvvidconv` -> `nvjpegenc` (hardware, NVMM in, quality
  `rgb.jpeg_quality`, size `rgb.width` x `rgb.height`). A probe on `rgb_queue` keeps one buffer per `1/rgb.fps` (half a
  sensor frame of tolerance) and drops the rest before they are scaled or encoded. There is no `videorate`: it would
  drop the sensor timestamps. `RgbJpeg` points into the encoder's output buffer (kept alive by its `hold`), so the
  JPEG is not copied. `set_jpeg_quality()` changes the running encoder (the `[live]` key).
- **Timestamps.** nvarguscamerasrc attaches the sensor start-of-frame time as `GstBufferMetaData` qdata
  (`{frame_num, timestamp, sensor_data}`, CLOCK_MONOTONIC ns; measured about 3.8 ms before the buffer reaches the
  source pad). A probe on each source pad records it by PTS, and the sink looks it up by PTS (PTS survives `nvvidconv`,
  `tee` and `nvjpegenc`, the qdata does not). Without it the capture uses PTS + pipeline base time (the pipeline
  clock is CLOCK_MONOTONIC). `ClockOffset` converts to CLOCK_REALTIME with an offset sampled at start (tightest of
  three bracketing reads) and refreshed every second, so chrony steering realtime onto the Pi's clock is followed.
- **Camera phase.** The two sensors are not synchronised; each one's frame phase is fixed when its stream starts
  (both run off the Tegra MCLK: measured stable over 40 s) and it was 12-17 ms out of step in practice, which no 8 ms
  window can pair. At start, after 5 frames per camera, the capture measures the phase from the sensor timestamps and
  restarts the right camera (a new phase each time) until the skew is within half of `capture.pair_window_ms`, at most
  20 times (typically 1-5 restarts, about 1.5 s each); then it pairs at whatever phase it has and logs it. The Argus
  `syncSensor` single-session pattern is the fallback if T16's skew histogram shows this is not enough.

### AE/AWB lock

Auto exposure and white balance run for `capture.settle_frames` frames on each camera after the phase is aligned.
Then, with `capture.lock_exposure_after_settle: true`, the capture sets `aelock` and `awblock` on both sources at
once, so both cameras keep the same exposure for matching. About 10 frames later it reads the values back from each
sensor's V4L2 controls (`/dev/video<sensor-id>`: `exposure` in microseconds, `gain` with 16 = 1x) and logs them once:

```
depthd capture: exposure locked: left (sensor-id 0) exposure 33329 us gain 6.31x (raw 101); right (sensor-id 1) exposure 33330 us gain 6.44x (raw 103);
```

Any camera restart unlocks both, and they settle and lock again together.

### Pairing

`depthd::StereoPairer` (`include/depthd/pairing.hpp`) keeps at most 3 frames per side, sorted by capture time, and
judges the earlier of the two queue heads against the other head (its nearest counterpart):

- if the earlier frame's own successor is nearer to that counterpart, the earlier frame's partner was never captured:
  it is dropped as an **orphan**;
- if `|right - left| <= capture.pair_window_ms` (inclusive) the two are emitted as a `StereoPair` (stamped with the
  left time, plus `right_minus_left_ns`); with a window under half a frame period this is the nearest pair;
- otherwise the earlier frame can never pair and is dropped for **skew**.

A frame is never reused: frames older than one already paired or dropped on the same side are orphans, as are frames
pushed out of a full queue and frames flushed by `reset()` when a camera fails. Every judged candidate (paired or
skew-dropped) goes into the skew histogram, buckets <= 1, 2, 4, 8, 16, 33 ms and above (`SkewHistogram`, Prometheus
style `cumulative()`); T16 reads it to judge the window. `pairing_stats()` returns `pairs_emitted`, `dropped_skew`,
`dropped_orphan` and the histogram.

### Health and failures

`status()` is `starting` (phase alignment, settling, no pair yet), `ok`, or `camera_error` with a reason naming the
camera (`left camera (sensor-id 0): ...`). A camera that delivers no frame for 2 s, or whose pipeline posts an error or
EOS, raises `camera_error` (read by `/health`); queued frames are flushed and no pair is emitted while it lasts, and no
RGB JPEG comes from a failing left camera. There is never a placeholder frame. The failed pipeline is restarted at
most every 5 s, after which the phase is aligned and AE/AWB settle again; the error clears with the first new frame.
Callbacks run on GStreamer streaming threads and must return quickly.

### Tests on the cameras

`test_capture_pipeline` has a live test for both `zero_copy` settings that is skipped by default, because the cameras
belong to one Argus client at a time and ctest must not take them from a running depthd. Stop depthd and run:

```
DEPTHD_HW_TEST=1 ./build/test_capture_pipeline --gtest_filter='*Live*'
```

It asserts in-window pairs at about 30 fps, 640x480 CV_8UC1 frames with real content, the exposure-lock log, and JPEGs
at about `rgb.fps`. The missing-camera test (sensor-ids 5 and 6) runs in the default suite and does not touch the
cameras.

## Config

`config/depthd.yaml` documents every key. All keys are optional, unknown keys are rejected, and an invalid value is
rejected with a message naming the key. `[live]` keys can change at runtime (`depthd::runtime_mutable`).

| Key | Default | Rule |
|---|---|---|
| capture.left_sensor_id / right_sensor_id | 0 / 1 | >= 0, different |
| capture.sensor_mode.{width,height,fps} | 1640 / 1232 / 30 | > 0 |
| capture.settle_frames | 30 | >= 0 |
| capture.lock_exposure_after_settle | true | |
| capture.pair_window_ms | 8.0 | > 0 |
| capture.zero_copy | true | |
| processing.width / height | 640 / 480 | multiple of 8 |
| processing.matcher | cuda_sgm | cuda_sgm, vpi, libsgm |
| processing.num_disparities | 128 | 64, 128, 256 |
| processing.p1 / p2 / uniqueness_ratio | 10 / 120 / 5 | p1 >= 0, p2 >= p1, ratio 0..100 |
| processing.min_depth_m / max_depth_m | 0.2 / 6.0 | 0 < min < max |
| postprocess.lr_check.{enabled,max_diff} [live] | true, 1 | |
| postprocess.confidence.enabled [live] | true | |
| postprocess.speckle.{enabled,max_size,max_diff} [live] | true, 200, 1 | |
| postprocess.bilateral.enabled [live] | true | |
| postprocess.hole_fill.{enabled,max_hole_px} [live] | true, 64 | |
| postprocess.temporal.{enabled,alpha,motion_threshold} [live] | true, 0.4, 12 | alpha 0..1 |
| postprocess.wls.enabled [live] | false | |
| mono.enabled [live] | true | |
| mono.model | dav2_small | dav2_small, midas_small |
| mono.engine_path | /var/lib/depthd/models/dav2_small_266.engine | |
| mono.rate_hz [live] | 2.0 | > 0 |
| rgb.enabled | true | |
| rgb.width / height | 640 / 480 | multiple of 8 |
| rgb.jpeg_quality [live] | 80 | 1..100 |
| rgb.fps | 10 | > 0 |
| transport.bind_address | 10.77.0.2 | |
| transport.pub_port / http_port | 5550 / 8090 | 1024-65535, different |
| transport.zstd_level / send_hwm | 1 / 2 | 1..22 / 1..1000 |
| calibration.path | /var/lib/depthd/stereo.yaml | |
| calibration.max_rms_px | 0.5 | > 0 |
| frame_ids.depth / rgb | stereo_left_optical_frame / overview_camera_optical_frame | not empty |

## Calibration file and rectification

`calibration.path` (default `/var/lib/depthd/stereo.yaml`) is OpenCV FileStorage YAML. `include/depthd/calibration.hpp`
is the one definition: the calibration tool (T20) writes it and depthd reads it. `test/fixtures/stereo_valid.yaml` is a
complete valid example (640x480, 60 mm baseline) used by both sides' tests; `stereo_invalid_rms.yaml` is the same with
`rms: 0.9`.

| Key | Type | Shape | Meaning |
|---|---|---|---|
| `image_width`, `image_height` | int | - | size the calibration was recorded at |
| `rms` | real | - | stereo reprojection RMS, px (must be <= `calibration.max_rms_px`) |
| `baseline_m` | real | - | metres, 0.03-0.12, equal to the norm of `T` within 1 % |
| `K_left`, `K_right` | double | 3x3 | unrectified camera matrices |
| `D_left`, `D_right` | double | 1x5 (or 1x8) | distortion k1 k2 p1 p2 k3 |
| `R`, `T` | double | 3x3, 3x1 | right camera relative to left (`T[0] < 0`, metres) |
| `R1`, `R2` | double | 3x3 | rectification rotations (`cv::stereoRectify`, alpha 0) |
| `P1`, `P2` | double | 3x4 | rectified projections, `P2(0,3) = -fx * baseline_m` |
| `Q` | double | 4x4 | disparity-to-depth matrix |

Every key is required. depthd rejects the file, keeping the reason for `/health`, on: a missing key or wrong matrix
shape, a non-finite value, a non-positive focal length, `rms` above `calibration.max_rms_px`, `baseline_m` outside
0.03-0.12 m or more than 1 % away from the norm of `T`, or an aspect ratio more than 1 % away from the processing size.
A valid file recorded at another resolution with the same aspect (for example 1280x960 for 640x480) is scaled to the
processing size: K, P (rows 0 and 1) and Q (column 3) scale with the size ratio; D, R, T and the baseline do not.

`depthd::Rectifier` loads the file once, builds the `initUndistortRectifyMap` maps, uploads them, and rectifies a
`GpuMat` pair with `cv::cuda::remap` (linear). `snapshot()` exposes the rectified left `P`, the baseline, `Q` and the
unrectified left `K`/`D` for camera_info. With a missing or invalid file the rectifier is `uncalibrated`, `reason()`
says why, and `rectify()` writes nothing, so no depth is produced. `reload()` re-reads the file and swaps the maps
atomically (a bad file on reload makes it `uncalibrated` too).

## Matcher interface and depth conversion

`include/depthd/matcher.hpp` is the common stereo matcher interface (design decision D3: the matcher is picked by
benchmark). `depthd::StereoMatcher` has:

- `compute(left, right, stream) -> cv::cuda::GpuMat`: rectified `CV_8UC1` inputs of equal size (anything else throws
  `std::invalid_argument`); returns `CV_16SC1` disparity times 16, left-image referenced; values `<= 0` are invalid.
- `name()`: the config name (`cuda_sgm`, later `vpi`, `libsgm`).
- `supports_lr_check()`: whether the matcher can also produce the right-referenced disparity for the left-right check.

`make_matcher(Config)` (or `make_matcher(Processing)`) builds `processing.matcher` and throws `std::invalid_argument`
for an unknown name or unsupported parameters. `cuda_sgm` (`src/matcher_cuda_sgm.cpp`) is `cv::cuda::createStereoSGM`
with `processing.num_disparities` (64, 128 or 256), `p1`, `p2`, `uniqueness_ratio` and `MODE_HH4`; it has no LR check.
A new matcher adds its own source and one branch in `src/matcher.cpp`.

`include/depthd/depth.hpp`: `disparity_to_depth_mm(disp16, f_px, baseline_m, min_depth_m, max_depth_m [, out], stream)`
runs on the GPU (OpenCV CUDA ops on the given stream) and returns `CV_16UC1` millimetres, the ToF contract where
0 means invalid:

```
Z_mm = round(f_px * baseline_m * 1000 / (disp16 / 16))      saturated at 65535, never wrapped
Z_mm = 0   where disp16 <= 0 (includes the matchers' negative invalid marker and INT16_MIN)
Z_mm = 0   where Z_mm < round(min_depth_m * 1000) or Z_mm > round(max_depth_m * 1000)   (limits inclusive)
```

`f_px` is the focal length of the rectified left projection (`P[0][0]`) and `baseline_m` comes from the calibration
(`Rectification::baseline_m()`). Example: f 400 px, B 60 mm, disparity 40 px gives 600 mm. Tests:
`test/test_depth.cpp` (exact values, every invalid case, saturation) and `test/test_matcher.cpp` (a textured pair
shifted by 40 px gives 600 mm within 2% on at least 90% of the interior for `cuda_sgm` at 64 and 128 disparities).
