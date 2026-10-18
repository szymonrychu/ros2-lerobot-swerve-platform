# head_camera_cpp

The head RGB camera node: an rclcpp node that owns the head Camera Module 3 (IMX708) through libcamera. It publishes
a 30 fps default stream and, on request, autofocused full-resolution zoom crops. The Camera Module 3 and the Arducam
ToF form the robot head on the pan/tilt stand.

This is the first version of the README: the package skeleton and the validated configuration. The node itself, its
topics, the zoom protocol and deployment are documented as they land.

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
| `jpeg_quality` | `70` | Quality (1..100) of `image_raw/compressed` |
| `camera_info_file` | none | Optional YAML calibration file for `camera_info` |
| `metrics_port` | `19113` | Prometheus port (1..65535); `null` disables metrics |
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
