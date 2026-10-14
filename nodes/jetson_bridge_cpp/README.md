# jetson_bridge_cpp

Pi side of the Jetson vision coprocessor link. `jetson/depthd` on the Jetson streams depth and RGB frames over a cabled
ZeroMQ link (wire format: `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`); this package turns them into
the ROS messages the Head tab, `web_ui` and `mcp_server` already read, and publishes nothing when the Jetson is absent
or the frame is stale.

This package so far holds the pure core (`jetson_bridge_core`, no sockets, no ROS node, no clock): the conversion
functions and the staleness policy. The ZeroMQ subscriber node, config and metrics build on it.

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
`test/test_convert.cpp` checks each conversion field by field, the uncalibrated case, every staleness case (limits,
configurable limits, counters, absence) and the stamp conversion.
