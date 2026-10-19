# head_camera_cpp

The head RGB camera node: an rclcpp node that owns the head Camera Module 3 (IMX708, CSI cam0) through libcamera. It
publishes a 30 fps default stream (1152x648) and, on request, autofocused full-resolution zoom crops. The Camera Module 3
and the Arducam ToF (`arducam_tof`, CSI cam1) form the robot head on the pan/tilt stand. The node replaces `camera_ros`
and the retired `pi_cameras` node: nothing else opens the camera, and `pi_cameras` and `camera_ros` are gone from the
client.

## Topics

All names are under `topic_namespace` (default `/head_camera`). Nothing is published until real data exists: with no
frame there is no message, and failed zooms publish no image.

| Topic | Type | Direction | QoS | Content |
|---|---|---|---|---|
| `/head_camera/image_raw` | `sensor_msgs/Image` | publish | reliable, keep_last 1 | Default view, `width` x `height`, 30 fps; only encoded while a subscriber exists |
| `/head_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | publish | reliable, keep_last 1 | The same frame as JPEG (`jpeg_quality`); only encoded while a subscriber exists |
| `/head_camera/camera_info` | `sensor_msgs/CameraInfo` | publish | reliable, keep_last 1 | With each frame, only with a calibration file (see below) |
| `/head_camera/zoom/command` | `std_msgs/String` | subscribe | reliable, keep_last 10 | JSON zoom request |
| `/head_camera/zoom/image` | `sensor_msgs/CompressedImage` | publish | reliable, keep_last 2 | JPEG crop |
| `/head_camera/zoom/result` | `std_msgs/String` | publish | reliable, keep_last 10 | JSON result |

Every header carries `frame_id` `head_camera_optical_frame`. This node does not publish a TF for that frame; the
pan/tilt head's transform tree owns it, and no consumer here depends on it.

## Zoom interface

The contract lives once in `include/head_camera_cpp/zoom_json.hpp`; this section mirrors it. Consumers: the
`mcp_server` `get_camera_image` tool (agent) and the web_ui Head tab (click-to-zoom).

**Request**, JSON on `/head_camera/zoom/command`:

```json
{"request_id": "r1", "u": 0.62, "v": 0.41, "zoom": 4, "max_px": 1024, "af": true}
```

| Field | Type | Meaning |
|---|---|---|
| `request_id` | string, required, non-empty | Echoed in the result. A command without a usable one is logged and dropped (no result can be addressed to it) |
| `u`, `v` | numbers in [0, 1], required | Point in the default view, top-left origin |
| `zoom` | integer, required | One of `zoom.factors` (2, 4 or 8) |
| `max_px` | integer, optional | `zoom.min_max_px`..`zoom.max_max_px` (256..1024), default `zoom.default_max_px`; longest side of the output |
| `af` | bool, optional, default `true` | Run windowed autofocus before the still |

Unknown keys are rejected (`invalid`).

**Image**, `/head_camera/zoom/image`: JPEG of longest side <= `max_px`, taken from the 4608x2592 still, so a 4x zoom is
sharp detail rather than an upscale. It is published before the result, and only for statuses that carry an image.

**Result**, JSON on `/head_camera/zoom/result`, always exactly these keys:

```json
{"request_id": "r1", "status": "ok", "message": "", "af_state": "focused", "lens_position": 2.1,
 "crop": [2880, 1000, 1152, 648], "full": [4608, 2592], "out": [1024, 576], "stamp_ns": 1760000000123456789,
 "stamp_s": 1760000000.123456, "phase_ms": {"af": 410, "switch": 520, "capture": 90, "restore": 480}}
```

`crop` is the region cut from the full frame (x, y, w, h), `full` that frame's size, `out` the JPEG size (`null`
without an image), `lens_position` the lens position in dioptres after the capture (`null` when unknown), and
`phase_ms` the time of each phase (0 for a phase that did not run).

| `status` | Meaning | Image |
|---|---|---|
| `ok` | Image taken | yes |
| `af_failed` | Focus did not converge; the picture is taken anyway, the caller may retry | yes |
| `busy` | Another zoom is running (answered at once, the camera is not touched) | no |
| `timeout` | A phase or the watchdog timed out | no |
| `camera_error` | The camera failed to switch, start or encode | no |
| `invalid` | Malformed fields, but a usable `request_id` | no |

**Stamp pairing.** A `CompressedImage` carries no request id, so the image and result are paired by time:
`result.stamp_ns` equals the image's `header.stamp` exactly (the capture stamp of the source frame). Because the image
is published first, a consumer that sees a result with `stamp_ns != 0` can pick the image with that stamp. `stamp_ns`
is 0 where there is no image.

**Timing.** Allow about 1.5-3 s for a request with autofocus (AF up to 2.5 s, mode switch up to 1.5 s, capture, then
restore); the whole request is bounded by `zoom.watchdog_s` (6 s). The default stream pauses for the duration of the
switch and restore and resumes at 30 fps afterwards.

## State machine

One request at a time (a second one gets `busy`). Phases:

1. **AF**: windowed autofocus (window `zoom.af_window_fraction` of the sensor, around the point) triggered on the running
   stream and polled until focused, failed or `zoom.af_timeout_s`. Skipped with `af: false`. A failed or timed-out
   focus carries on as `af_failed`: the picture is still taken.
2. **SWITCH**: stop the video stream, configure the full-resolution still, start it, and discard `zoom.settle_frames`
   frames, all within `zoom.switch_timeout_s` (else `timeout`). A configure or start failure is `camera_error`.
3. **CROP**: cut the region around the point (`zoom_region`, clamped to the frame), scale to `max_px` and JPEG-encode at
   `zoom.jpeg_quality`. An encode error is `camera_error`.
4. **RESTORE**, always, after success and after every failure: stop, configure the default stream, start it. On
   failure the node waits `zoom.restore_retry_delay_s` and retries once, then re-acquires the camera and tries again,
   and when that fails too it exits with status 1 so systemd restarts the unit.

A watchdog of `zoom.watchdog_s` over phases 1-3 turns into `timeout`; restore still runs. The result is published only
after restore, so a consumer that gets it can rely on the 30 fps stream coming back.

## Mapping stream

With `mapping.enabled: true` the node also publishes a small stream for the RGB-D fusion node (`head_rgbd_cpp`) and, through it, RTAB-Map:

| Topic | Type | Content |
|---|---|---|
| `<ns>/mapping/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG (`mapping.jpeg_quality`, default 80), `mapping.width` x `mapping.height` (default 576x324), `mapping.rate_hz` (default 2 Hz) |
| `<ns>/mapping/camera_info` | `sensor_msgs/CameraInfo` | Calibration scaled to the mapping size, same header (stamp = capture stamp, `frame_id`) as the image |

Both are best_effort, depth 1. Frames are resized (INTER_AREA), encoded and published only when due (at most `mapping.rate_hz`) and
only while the image topic has a subscriber, so 30 fps input costs at most 2 encodes per second. Nothing is published
while a zoom holds the camera in still mode.

**Calibration requirement.** `camera_info` is published only when `camera_info_file` loads (the calibration is scaled with
the same rules as `camera_info`; there are no placeholder intrinsics). Without a calibration the image is still published
and `camera_info` is not, so `head_rgbd_cpp` publishes nothing. The intrinsics are valid only at the focus they were
calibrated at, hence the focus lock.

**Calibration source.** The deployed `camera_info_file` is the applied state file
`/var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml` (not under `/etc/ros2`). It is written by Apply on the web UI Calibrate page
(RGB-D section) and survives Ansible deploys. When the applied dir is empty, the deploy seeds it from the committed
`head_camera_calibration.yaml`; a missing file means no `camera_info`. Apply restarts the node to re-read it.

**Focus lock.** The camera keeps the configured `af_mode` (default continuous) normally. While `mapping.enabled`,
the runtime parameter `mapping.lock_focus` is true and `<ns>/mapping/image_raw/compressed` has at least one subscriber,
the node applies `AfMode` Manual and `LensPosition` = `lens_position`; when the last subscriber leaves, or
`ros2 param set /head_camera mapping.lock_focus false`, it restores the configured `af_mode`. A zoom still autofocuses on its
point; its RESTORE phase returns the lens to the lock when it is active, else to the configured AF mode. Set
`mapping.lock_focus` false to keep continuous AF while mapping.

## camera_info policy

`camera_info` is published (with each default frame, same stamp and `frame_id`) only when `camera_info_file` names a
ROS camera_calibration YAML that loads and validates; the intrinsics are scaled to the published size. With no file the
node logs once and publishes no `camera_info`, and a file that fails to load is logged as an error and also leaves the
topic silent: an uncalibrated `CameraInfo` with a zero K matrix would be placeholder data and mislead depth or
projection consumers.

## Build and test

```bash
scripts/cpp-node-test.sh head_camera_cpp              # build, unit tests and ament linters in a ros:jazzy container
scripts/cpp-node-test.sh head_camera_cpp --lint-only  # linters only
```

`head_camera_core` (`src/core/*.cpp`, globbed) holds the pure logic and links neither libcamera nor rclcpp; tests are
`test/core/test_*.cpp`, one gtest per file.

## Configuration

`/etc/ros2/head_camera_cpp/config.yaml`, or the file named by `HEAD_CAMERA_CONFIG`. Unknown keys at any level, and any
invalid value, stop the node with exit code 78. Every error line names its key (`zoom.watchdog_s: ...`).

| Key | Default | Meaning |
|---|---|---|
| `camera_id` | `/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a` | libcamera camera id of the head IMX708 |
| `frame_id` | `head_camera_optical_frame` | `frame_id` of every published header |
| `topic_namespace` | `/head_camera` | Prefix of the topics: `image_raw`, `image_raw/compressed`, `camera_info`, `zoom/command`, `zoom/image`, `zoom/result` |
| `sensor_mode` | `[2304, 1296]` | Sensor mode (width, height) of the default stream |
| `width` | `1152` | Published image width; even, > 0 |
| `height` | `648` | Published image height; even, > 0, same aspect as `sensor_mode` within 1 px |
| `frame_duration_us` | `33333` | Frame duration in microseconds (33333 = 30 fps); > 0 |
| `af_mode` | `continuous` | Autofocus: `continuous`, `auto` or `manual` |
| `lens_position` | `1.0` | Dioptres (>= 0): the focus-lock position while mapping and the position of `af_mode: manual` |
| `mapping.enabled` | `false` | Enable the 2 Hz mapping stream (`<ns>/mapping/image_raw/compressed`, `<ns>/mapping/camera_info`) |
| `mapping.lock_focus` | `true` | Runtime parameter: lock the lens at `lens_position` while the mapping image has subscribers; continuous AF otherwise |
| `mapping.rate_hz` | `2.0` | Mapping stream rate, 0 < r <= 10 |
| `mapping.width` / `mapping.height` | `576` / `324` | Even, within the stream size, stream aspect within 1 px |
| `mapping.jpeg_quality` | `80` | 1..100 |
| `jpeg_quality` | `70` | Quality (1..100) of `image_raw/compressed` |
| `camera_info_file` | none | Optional YAML calibration file for `camera_info`; deployed: `/var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml` (see Calibration source) |
| `metrics_port` | `19116` | Prometheus port (1..65535); `null` disables metrics |
| `zoom.still_size` | `[4608, 2592]` | Full-resolution still (width, height) the crops are cut from |
| `zoom.factors` | `[2, 4, 8]` | Allowed zoom factors; non-empty subset of 2, 4, 8 |
| `zoom.default_max_px` | `1024` | Longest side of a zoom image when the request names none |
| `zoom.min_max_px` | `256` | Smallest accepted `max_px`; <= `default_max_px` |
| `zoom.max_max_px` | `1024` | Largest accepted `max_px`; >= `default_max_px`, <= 1024 |
| `zoom.jpeg_quality` | `85` | Quality (1..100) of the zoom image |
| `zoom.af_timeout_s` | `2.5` | How long to wait for autofocus to lock; > 0 |
| `zoom.switch_timeout_s` | `1.5` | How long to wait for a camera mode switch; > 0 |
| `zoom.settle_frames` | `2` | Frames discarded after a switch before the still is taken; >= 0 |
| `zoom.still_buffers` | `2` | Buffers of the still stream; >= 1 |
| `zoom.watchdog_s` | `6.0` | Whole-request deadline; must exceed `af_timeout_s + switch_timeout_s` |
| `zoom.restore_retry_delay_s` | `0.5` | Delay before retrying to restore the default stream; >= 0 |
| `zoom.af_window_fraction` | `0.125` | Autofocus window as a fraction of the sensor; 0 < f <= 1 |

## Metrics

Prometheus text on `127.0.0.1:<metrics_port>/metrics` (default 19116; scraped through `monitoring_agent_node_targets`).

| Metric | Type | Labels | Meaning |
|---|---|---|---|
| `head_camera_frames_captured_total` | counter | | Video frames delivered by the camera |
| `head_camera_frames_published_total` | counter | `topic` (`image_raw`, `image_raw_compressed`, `camera_info`) | Messages published |
| `head_camera_capture_fps` | gauge | | Frames per second delivered by the camera (30 expected) |
| `head_camera_encode_seconds` | histogram | | JPEG encode time of one video frame |
| `head_camera_zoom_requests_total` | counter | `status` | Zoom results by status |
| `head_camera_zoom_phase_seconds` | histogram | `phase` (`af`, `switch`, `capture`, `restore`) | Time per zoom phase |

## Deploy

Ansible node type `head_camera_cpp` in `ansible/group_vars/client.yml` (native, `ros2 run head_camera_cpp
head_camera_node`), entry `head_camera_cpp` on the client, background cgroup tier (`memory_low` 128M, `memory_max` 384M,
weights 100, `nice` 10). It is built with colcon into `/opt/ros2-ws` in this order: the pinned libcamera fork, then
`ros2_common_cpp`, then this package.

```bash
./scripts/deploy-nodes.sh client head_camera_cpp 2>&1 | tee .logs/deploy-client.log
```

**libcamera fork dependency.** The node links the Raspberry Pi libcamera fork (`raspberrypi/libcamera`, tag
`v0.7.2+rpt20260817`, `rpi/pisp` pipeline and IPA), built from source. The `colcon_source` dict and the
`patches/libcamera-0001-add-package-xml.patch` (adds the package.xml colcon needs) are the ones `pi_cameras` used, so
the build stamp is unchanged and nothing rebuilds. The apt `ros-jazzy-libcamera` and `ros-jazzy-camera-ros` packages
must not be installed; `camera_ros` was purged from `/opt/ros2-ws` when `pi_cameras` was retired.

Verify on the robot: `ros2 topic hz /head_camera/image_raw/compressed` shows about 30 Hz, `head_camera_capture_fps` is
about 30, and publishing a request on `/head_camera/zoom/command` yields an image and an `ok` result.
