# jetson_bridge_cpp

Pi side of the Jetson vision coprocessor link. `jetson/depthd` on the Jetson streams depth and RGB frames over a cabled
ZeroMQ link (wire format: `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`); this package turns them into
the ROS messages the Head tab, `web_ui` and `mcp_server` already read, and publishes nothing when the Jetson is absent
or the frame is stale.

The package holds the pure core (`jetson_bridge_core`: config, conversion, staleness policy; no sockets, no node, no
clock) and the node on top of it (`ros2 run jetson_bridge_cpp jetson_bridge_node`): a ZeroMQ SUB thread, five
publishers and Prometheus metrics. No JPEG is decoded and no raw `/overview_camera/image_raw` is produced on the Pi.

## Config

`JETSON_BRIDGE_CONFIG` (default `/etc/ros2/jetson_bridge_cpp/config.yaml`), lax YAML coercion, unknown keys rejected,
an invalid file exits 78. `config.hpp` holds the defaults and is the one definition of the topic contract on the Pi.

| Key | Default | Meaning |
|---|---|---|
| `endpoint` | `tcp://10.77.0.2:5550` | depthd PUB socket on the direct link (must start with `tcp://`) |
| `depth_topic` | `/stereo/depth/image_raw` | `Image` 16UC1 mm, from wire `depth` |
| `camera_info_topic` | `/stereo/camera_info` | `CameraInfo`, rectified left camera, sent with each depth frame |
| `fused_topic` | `/stereo/depth/fused` | `Image` 16UC1 mm, from wire `fused` |
| `rgb_topic` | `/overview_camera/image_raw/compressed` | `CompressedImage` jpeg, from wire `rgb`, bytes unchanged |
| `rgb_info_topic` | `/overview_camera/camera_info` | `CameraInfo`, unrectified left camera |
| `max_age_ms` | 500 | frames older than this (capture stamp vs Pi clock) are dropped |
| `absent_after_ms` | 2000 | no frame of any kind for this long means the Jetson is absent |
| `metrics_port` | 19114 | `127.0.0.1` exporter; `null` disables (else `METRICS_PORT`) |

## QoS and behaviour

All five publishers are reliable, volatile, keep last 5 (the retired `stereo_depth` node's QoS). The SUB socket uses
`RCVHWM 2` and runs on its own thread; each message is `[topic, header, payload]`, decoded with `depth_wire`, then
checked by `StalenessPolicy`. `header.stamp` is the Jetson's capture time. `camera_info` is published only with a
calibrated header.

## Failure behaviour

The bridge never publishes made-up data. Each of these is counted in `jetson_bridge_dropped_total{reason}` and
nothing is published:

| reason | cause |
|---|---|
| `malformed` | message is not exactly three frames |
| `decode_error` | `depth_wire` rejected it (bad magic, version, sizes, topic/kind mismatch, zstd) |
| `stale` | capture stamp more than `max_age_ms` old |
| `future` | capture stamp more than 50 ms ahead of the Pi clock |
| `uncalibrated` | depth or fused frame from a Jetson without a calibration |

While the Jetson is absent (nothing for `absent_after_ms`, or never) the topics stay silent and
`jetson_bridge_jetson_up` is 0. A Jetson sending only stale frames is present (`jetson_up 1`) but still silent.

## Metrics (`127.0.0.1:19114`)

- `jetson_bridge_frames_total{kind=depth|fused|rgb}`: republished frames.
- `jetson_bridge_dropped_total{reason}`: see above.
- `jetson_bridge_latency_seconds{kind}`: histogram, Pi receive time minus `capture_realtime_ns`.
- `jetson_bridge_jetson_up`: 1 while frames arrive within `absent_after_ms`.
- Jetson health from the latest decoded header: `jetson_depthd_pipeline_fps`, `jetson_depthd_stage_ms{stage}`
  (capture, rectify, match, post, mono, encode), `jetson_depthd_pairs_dropped_total`, `jetson_depthd_gpu_temp_c`,
  `jetson_depthd_nvgpu_errors_total`. They keep their last value when the Jetson goes absent; read them with
  `jetson_bridge_jetson_up`.
- `process_*` (CPU, memory) from `ros2_common_cpp` `ProcessCollector`, plus `robot_node_info`.

## Conversion (`jetson_bridge_cpp/convert.hpp`)

| Function | Output | Contract |
|---|---|---|
| `to_depth_image(header, mm)` | `sensor_msgs/Image` | `16UC1`, millimetres, `0` = invalid, little-endian, `step = 2*width`; frame `stereo_left_optical_frame` (the header's `frame_id`) |
| `to_camera_info(header, kRectified)` | `sensor_msgs/CameraInfo` | `/stereo/camera_info`: rectified left camera, `P` as sent, `K` taken from `P`, `D` zeros (`plumb_bob`), `R` identity; the depth image's stamp, frame and size |
| `to_compressed(header, jpeg)` | `sensor_msgs/CompressedImage` | overview image, `format = "jpeg"`, bytes unchanged; frame `overview_camera_optical_frame` |
| `to_camera_info(header, kRaw)` | `sensor_msgs/CameraInfo` | `/overview_camera/camera_info`: unrectified left `K`, `D` as sent (`plumb_bob`), `R` identity, `P` = `K` with a zero translation column |
| `stamp_from_realtime_ns(ns)` | `builtin_interfaces/Time` | the Jetson's `capture_realtime_ns` (floor division, `nanosec` never negative) |

- `header.stamp` of every message is the Jetson's `capture_realtime_ns`; chrony keeps the Pi and Jetson clocks in step.
- An empty `frame_id` in the wire header falls back to the contract frame of the message type.
- `to_camera_info` returns `std::nullopt` unless the wire header's `calibrated` flag is set: an uncalibrated Jetson sends
  zeros, which are never republished.
- `to_depth_image` and `to_compressed` throw `std::invalid_argument` on a header of the wrong kind, a pixel count that is
  not `width*height`, or an empty JPEG. The fused frame (`kind = fused`) converts like a depth frame.

## Staleness rule (`StalenessPolicy`)

`StalenessPolicy(max_age_ms = 500, absent_after_ms = 2000, max_future_ms = 50)`; time is the Pi's `CLOCK_REALTIME`,
passed in by the caller.

- `accept(header, now_ns)` rejects (`kTooOld`) a frame whose capture stamp is more than `max_age_ms` older than now, and
  (`kFromFuture`) one more than `max_future_ms` ahead of now. Exactly on the limit is accepted.
- Every call counts as an arrival, accepted or not: a Jetson that sends only stale frames is present but unusable, not
  absent. `absent(now_ns)` is true when nothing arrived for more than `absent_after_ms`, or nothing ever arrived.
- A rejected frame is only counted (`rejected_too_old()`, `rejected_from_future()`, `accepted()`), never republished and
  never replaced by placeholder data.

## Tests

`./scripts/cpp-node-test.sh jetson_bridge_cpp` (docker, ros:jazzy-ros-base; also clang-format, cppcheck, xmllint).
`test/test_convert.cpp` checks each conversion and the staleness policy; `test/test_config.cpp` defaults, overrides and
rejections; `test/test_node.cpp` feeds an in-process loopback PUB and checks each topic, plus the stale, future,
undecodable, uncalibrated and absent cases (nothing published, counters incremented) and the metrics.
