# depthd

C++17 GPU depth pipeline for the Jetson Nano vision coprocessor. It captures the two IMX219 cameras, rectifies and
matches them on the GPU, post-processes the disparity, optionally fuses monocular depth, and publishes depth and
RGB over a cabled ZeroMQ link (wire format: `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`) to the
ROS 2 bridge on the Pi. This keeps stereo work off the saturated Pi 5 CPU.

## Layout

```
CMakeLists.txt        finds and links every dependency; globs src/*.cpp and test/test_*.cpp
include/depthd/       public headers (config.hpp: the config schema)
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
