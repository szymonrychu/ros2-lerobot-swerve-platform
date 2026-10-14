# depthd

C++17 GPU depth pipeline for the Jetson Nano vision coprocessor. It captures the two IMX219 cameras, rectifies and
matches them on the GPU, post-processes the disparity, optionally fuses monocular depth, and publishes depth and
RGB over a cabled ZeroMQ link (wire format: `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`) to the
ROS 2 bridge on the Pi. This keeps stereo work off the saturated Pi 5 CPU.

## Layout

```
CMakeLists.txt        finds and links every dependency; globs src/*.cpp and test/test_*.cpp
include/depthd/       public headers (config.hpp: the config schema; capture.hpp, pairing.hpp: stereo capture; calibration.hpp, rectify.hpp: calibration and GPU rectification; publisher.hpp, http_api.hpp: ZeroMQ publisher, HTTP API and metrics; matcher.hpp, depth.hpp: matching and depth; postprocess.hpp, temporal.hpp: disparity post-processing; pipeline.hpp, health.hpp, daemon.hpp: the service wired end to end)
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

## Running depthd

```
depthd [--config PATH]        run (default /etc/depthd/config.yaml) until SIGTERM or SIGINT
depthd --check-config PATH    exit 0 and print "ok: PATH", or exit 1 and print the reason on stderr
```

Ansible runs `--check-config` on the new file before it restarts the service. SIGTERM stops every thread and returns
within 2 s (systemd). A config error (or a matcher the config names that cannot be built) or a calibration error at
startup does not exit: it is logged, and the process stays up serving `/health` with the reason, so systemd never
enters a crash loop. A link address that cannot be bound yet is retried every 2 s, a camera that cannot be started
every 5 s (`src/daemon.cpp`).

### Data flow

```
cameras (StereoCapture, a FrameSource)
  pair callback --> input mailbox (1 pair; a newer pair replaces it, counted drops{reason="busy"})
  rgb callback  --> gates (left camera healthy, age) (JPEG from nvjpegenc) --> topic "rgb" at rgb.fps
match thread, one CUDA stream per pair, into one of two slots:
  gates (calibrated, both cameras healthy, size, age <= 100 ms) --> rectify --> match --> post-process
  --> depth mm (CV_16UC1, 0 = invalid) --> download into the slot
encode thread: zstd --> topic "depth" --> slot freed
```

The two slots circulate between the match and the encode threads, so zstd and the send of one frame overlap matching
of the next and no more than two frames are ever in flight; GPU buffers are allocated once per slot and reused. The
capture callbacks only swap a pair into the mailbox, so capture never waits on matching. The `depth` header carries the
calibration intrinsics, the rectified `P`, `capture_realtime_ns` of the left frame, and the health fields. `rgb` flows
whether or not a calibration exists; its header carries the left unrectified `K`/`D` (scaled to the RGB size) only when
calibrated, and `calibrated` is false otherwise. `fused` (the monocular stage, `Pipeline::publish_fused`) obeys the
same gates as `depth`.

Nothing is published that depthd cannot vouch for: no `depth` or `fused` without a valid calibration, none while either
camera is in error, no `rgb` while the left camera is in error, nothing from a stale frame, and never a synthetic or
placeholder frame. Each refusal is counted in `depthd_drops_total{reason}` (`busy`, `stale`, `uncalibrated`,
`camera_error`, `bad_frame`, `error`, `rgb_stale`, `rgb_camera_error`, `fused_rejected`) and in
`Pipeline::counters()`.

### Latency budget (G5)

A pair older than 100 ms (`PipelineOptions::max_pair_age_ns`), or more than 100 ms in the future, when matching starts
is dropped and counted as `stale`. Per-stage times go to `depthd_stage_time_ms{stage}`: `capture` (capture to match
start), `rectify`, `match`, `post` (and `post_<stage>` per post-processing stage), `encode`. The budget from capture to
the wire is 100 ms at the configured fps; the mailbox and the two slots bound the queueing so a slow match drops pairs
instead of growing latency.

### Health states

`GET /health` and every wire header report one state; the first that applies wins (`src/health.cpp`):

| State | Meaning | What flows |
|---|---|---|
| `config_error` | the config file, or the matcher it names, is invalid; reason names it | nothing; only `/health` |
| `camera_error` | a camera failed or could not be started; reason names the camera | nothing of that camera's kind |
| `uncalibrated` | no valid calibration at `calibration.path`; reason says why | `rgb` only |
| `starting` | binding the link, aligning the cameras, or no depth frame yet | `rgb` once the left camera runs |
| `ok` | depth is published; the reason names a stall when no depth came for over 2 s | everything |

The health fields also carry `fps`, `last_pair_age_ms`, `pairs_dropped_total`, `gpu_temp_c` (thermal zone `GPU-therm`)
and `nvgpu_errors_total` (nvgpu/gk20a error records in `/dev/kmsg`, counted from the start of the ring buffer so a FECS
loop at boot shows up). Tests: `test/test_pipeline.cpp` (a fake FrameSource of synthetic shifted pairs and the fixture
calibration, read back over a loopback SUB socket), `test/test_health.cpp`, `test/test_daemon.cpp` (`--check-config`,
config error kept up, SIGTERM within 2 s).

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
- **Runtime phase re-alignment.** The phase does drift over minutes (observed live: 1.55 ms at start, most pairs at
  8-33 ms half an hour later, depth down to 0.1 Hz, then a stall). `PhaseDriftMonitor` (`phase_monitor.hpp`) watches
  the skew of every candidate the pairer judges and the time since the last pair while both cameras are healthy and
  aligned. When the median skew of the trailing `capture.realign_window_s` exceeds `capture.realign_skew_ms`, or no
  pair came for `capture.realign_stall_s`, the watchdog re-runs the startup alignment from a right-camera restart
  (AE/AWB unlock and settle again), logs `camera phase drifted (...)` and counts it in
  `depthd_camera_realignments_total{reason="skew"|"stall"}`. Re-alignments are at least `capture.realign_cooldown_s`
  apart. While aligning, at start or at runtime, no pair reaches the pipeline and no RGB is published.

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
- `name()`: the config name (`cuda_sgm`, `vpi`, `libsgm`).
- `supports_lr_check()`: whether `compute()` already applies a left-right consistency check (max diff 1 px). Then the
  post-processing `lr_check` stage is skipped and the matcher is not run a second time on the mirrored pair.

`make_matcher(Config)` (or `make_matcher(Processing)`) builds `processing.matcher` and throws `std::invalid_argument`
for an unknown name or unsupported parameters. `cuda_sgm` (`src/matcher_cuda_sgm.cpp`) is `cv::cuda::createStereoSGM`
with `processing.num_disparities` (64, 128 or 256), `p1`, `p2`, `uniqueness_ratio` and `MODE_HH4`; its `compute()` checks left-right consistency itself (OpenCV `checkConsistency`, max diff 1 px).
A new matcher adds its own source and one branch in `src/matcher.cpp`.

### The three matchers

All three take `processing.num_disparities` (64, 128 or 256; anything else throws), return `CV_16SC1` disparity * 16,
and `supports_lr_check()` is true for `cuda_sgm` and `libsgm` (both check inside) and false for `vpi`. Select with
`processing.matcher`; T16's on-board benchmark compares them on equal terms.

| `processing.matcher` | Implementation | Parameters used | Limits |
|---|---|---|---|
| `cuda_sgm` | `cv::cuda::StereoSGM`, `MODE_HH4` (`matcher_cuda_sgm.cpp`) | num_disparities, p1, p2, uniqueness_ratio | LR check inside (max diff 1 px, `postprocess.lr_check.max_diff` does not apply); uniqueness is OpenCV's percent |
| `vpi` | VPI 1.1 C API, CUDA backend (`matcher_vpi.cpp`, `find_package(vpi 1.1)`) | num_disparities (`maxDisparity`), fixed `windowSize` 5 | exposes only windowSize/maxDisparity; p1, p2 and uniqueness_ratio do not apply; no VIC or PVA stereo on the Nano (Xavier only), CUDA backend only; maxDisparity <= 256 |
| `libsgm` | libSGM 3.1.0 device-pointer API (`matcher_libsgm.cpp`) | num_disparities, p1, p2, uniqueness_ratio (libSGM uniqueness = 1 - ratio/100), LR check (max diff 1), subpixel, 8 paths | needs 64, 128 or 256 disparities; 8-bit input |

- **vpi.** The U8 inputs are wrapped from the GpuMat memory with `vpiImageCreateCUDAMemWrapper` (no copy). The Q10.5
  disparity (`U16`, the only format the CUDA backend accepts) is halved to disparity * 16, and every pixel whose
  confidence is below 32767 (`kVpiConfidenceMin`, VPI's documented default threshold) is set to -1. That value comes
  from the synthetic test: the shifted pair (40 px) keeps 100% of its pixels, all within 2% of 600 mm, at every
  threshold from 1 to 65280, and an uncorrelated pair keeps under 1% (`VpiConfidence.UncorrelatedPairIsInvalid`). So
  the vpi accuracy test needs no relaxed coverage threshold: all three matchers use the same 90% bar.
- **libsgm.** `CMakeLists.txt` fetches the pinned tag `3.1.0` with `FetchContent` (built for `sm_53` only, no samples
  or tests; `cmake/patch_libsgm.cmake` keeps nvcc 10.2 on C++14 for the `.cu` files) and runs `sgm::StereoSGM` with
  `EXECUTE_INOUT_CUDA2CUDA`, 16-bit subpixel output (4 fractional bits, the interface's scale), the LR check and
  uniqueness inside libSGM. Its invalid marker is negative. First configure needs network access to GitHub.
- Both run on their own (default) CUDA stream: the caller's stream is synchronised before the call.

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
shifted by 40 px gives 600 mm within 2% on at least 90% of the interior for `cuda_sgm`, `vpi` and `libsgm` at 64 and 128 disparities).

## Post-processing

`include/depthd/postprocess.hpp`: `PostProcessor(config, matcher_native_lr_check)` cleans the matcher's `CV_16SC1`
disparity * 16 map in place (invalid <= 0). `run(disp, left, right_disp, stream)` takes the rectified left image
(`CV_8UC1`) as guidance and, only when `lr_check` is on and the matcher has no native check (`needs_right_disparity()`),
the right-referenced disparity from `compute_right_disparity(matcher, left, right)` (both images mirrored and swapped
through any matcher). A missing `right_disp` throws `std::invalid_argument`; the check is never silently skipped.

Stages run in this order, each only when its `postprocess.*` switch is on (all [live]: `set_postprocess` takes effect
on the next run). A disabled stage launches nothing, records no event and reports `ran=false, ms=0`. Every stage that
runs writes 0 for invalid pixels. `timings()` returns, per stage, the last run's cost and the mean over the runs it ran
in, measured with CUDA events on the run's stream (filters are built on first use, outside the timed window).

| Stage | Config | What it does | Cost, 640x480 Nano |
|---|---|---|---|
| `lr_check` | `lr_check.enabled`, `max_diff` (px) | invalid when `abs(dL(x) - dR(x - dL)) > max_diff`, when the right pixel is invalid or when `x - dL` is outside the image. Skipped when `supports_lr_check()` (cuda_sgm and libsgm check inside, so only vpi runs it; for the others the matcher would otherwise run twice per frame) | 0.4 ms |
| `confidence` | `confidence.enabled` | invalid unless 1 px <= d < `num_disparities - 1` px (saturated at the search range) and the partner pixel lies inside the right image. Uniqueness itself is inside the matcher (`processing.uniqueness_ratio`) | 0.2 ms |
| `speckle` | `speckle.enabled`, `max_size`, `max_diff` (px) | 4-connected regions of valid pixels whose neighbours differ by at most `max_diff` and that hold fewer than `max_size` pixels become invalid (CUDA label equivalence, one flag read back per iteration) | 5.5 ms |
| `bilateral` | `bilateral.enabled` | `cv::cuda::DisparityBilateralFilter` (radius 3, 1 iteration, `ndisp = num_disparities * 16`) guided by the left image: it snaps disparity edges to image edges. Pixels that were invalid stay 0 | 1.5 ms |
| `hole_fill` | `hole_fill.enabled`, `max_hole_px` | 4-connected invalid regions of at most `max_hole_px` pixels are filled, each pixel from the first valid pixel in each of 4 directions whose left-image grey is closest to its own (edge aware; ties take the smaller disparity). Larger holes stay 0: prefer invalid over wrong | 3.5 ms |
| `wls` | `wls.enabled` (off by default) | ximgproc WLS filter on the CPU (lambda 8000, sigma 1.5). Invalid pixels are first filled with a local mean so they do not drag neighbours, and set back to 0 afterwards | CPU |
| `temporal` | `temporal.enabled`, `alpha`, `motion_threshold` | `TemporalFilter`, see below | 0.4 ms |

Costs are the measured means of the synthetic 640x480 run in the test suite's style (random disparity with holes); the
real figures are in `/metrics` of the running service.

`TemporalFilter(alpha, motion_threshold)` (`include/depthd/temporal.hpp`) is a per-pixel EMA on valid pixels:
`out = alpha * new + (1 - alpha) * history`. Motion is the grey frame difference of the left image (no odometry, D4):
where `abs(grey - previous grey) > motion_threshold` the history is dropped and the new value passes through. The
first frame, `reset()`, a size change and switching the stage off also drop history. An invalid pixel stays invalid and
loses its history, so the filter never turns an invalid pixel valid. `PostProcessor` runs it last and owns one; it can
also be used alone.

`src/postprocess_kernels.cu` holds the kernels (raw device pointers, nvcc 10.2 at C++14); `CMakeLists.txt` enables the
CUDA language for it (sm_53, shared cudart). Tests: `test/test_postprocess.cpp` (every stage, toggles, order, timing
flags, bad inputs) and `test/test_temporal.cpp`.

## Transport and HTTP API

Both listen on `transport.bind_address` (the Jetson end of the direct link, `10.77.0.2`), so nothing is reachable from
the LAN side.

| Port | Protocol | Config key | Served by |
|---|---|---|---|
| 5550 | ZeroMQ PUB over TCP | `transport.pub_port` | `depthd::Publisher` |
| 8090 | HTTP | `transport.http_port` | `depthd::HttpApi` (cpp-httplib) |

### ZeroMQ publisher

`depthd::Publisher` (`include/depthd/publisher.hpp`) uses the libzmq C API (focal ships no cppzmq): one PUB socket,
`send_depth`, `send_fused` and `send_rgb`. Messages are the three frames of `ros2_common_cpp/depth_wire.hpp`
(topic `depth` | `fused` | `rgb`, header, zstd depth or JPEG); the publisher sets the header kind and encodes with
`depth_wire` only. The pipeline never waits on the link: `ZMQ_SNDHWM` is `transport.send_hwm`, `ZMQ_CONFLATE` is off
(frames queue in order), `ZMQ_XPUB_NODROP` makes a full high-water mark an error instead of a silent drop, and every
send is `ZMQ_DONTWAIT`. A dropped send returns false and is counted in `send_drops()`
(`depthd_zmq_send_drops_total`; feed it with `Metrics::set_zmq_send_drops`). A message that cannot be encoded returns
false without counting.

### HTTP API

`depthd::HttpApi` (`include/depthd/http_api.hpp`) owns the running config and the calibration state; the pipeline plugs
in through `HttpApiHooks` (`health`, `capture_pair`, `apply_config`, `on_calibration`) and `Metrics`. Errors are
`{"error": "<message>"}`.

| Request | Answer |
|---|---|
| `GET /health` | `{state, reason, fps, last_pair_age_ms, pairs_dropped_total, gpu_temp_c, nvgpu_errors_total, matcher, mono_state}`; `state` is `ok`, `starting`, `uncalibrated`, `camera_error` or `config_error` (see Health states) |
| `GET /config` | the running config (same JSON as `config/depthd.yaml`) |
| `PUT /config` | body: a JSON object, nested (`{"postprocess":{"temporal":{"alpha":0.7}}}`) or dotted keys (`{"rgb.jpeg_quality":60}`). 200 with the new config for `runtime_mutable` keys (`postprocess.*`, `mono.enabled`, `mono.rate_hz`, `rgb.jpeg_quality`); 409 `{error, key, keys}` naming the keys that need a restart (a restart-only key sent with its current value is not a change); 400 with the validation message for an invalid, unknown or ill-typed value, bad JSON, or a refused `apply_config` hook. Nothing changes on any error. |
| `GET /calibration` | `{status: ok\|uncalibrated, reason, rms, baseline_m, created_utc, path, last_reload_error}` of `calibration.path`; `created_utc` is the file's optional `created_utc` key, else its modification time |
| `POST /calibration/reload` | reloads the file: 200 with the new status, or 422 with the `reason`; a failed reload keeps the previous calibration active |
| `GET /calibration/capture` | latest unrectified pair as `multipart/form-data`: PNG parts `left` and `right` (GRAY8, processing size) and text parts `left_capture_realtime_ns` and `right_capture_realtime_ns`; 503 when there is no fresh pair. This is the calibration tool's capture path |
| `GET /metrics` | Prometheus text (below) |

Metrics: `depthd_frames_total{kind}`, `depthd_pairs_total`, `depthd_drops_total{reason}` (`skew`, `orphan`, plus
`Metrics::inc_drop` reasons), `depthd_pair_skew_ms` (histogram), `depthd_stage_time_ms{stage}` (histogram, buckets
1-500 ms), `depthd_fps`, `depthd_gpu_temp_celsius`, `depthd_nvgpu_errors_total`, `depthd_zmq_send_drops_total`,
`depthd_camera_realignments_total{reason}` (`skew`, `stall`).

`third_party/httplib.h` is cpp-httplib v0.14.3 (MIT, Yuji Hirose) vendored unmodified as a single header. Tests:
`test/test_publisher.cpp` (loopback SUB decodes depth, fused and rgb; a full high-water mark drops and counts) and
`test/test_http_api.cpp` (every endpoint and status code).
