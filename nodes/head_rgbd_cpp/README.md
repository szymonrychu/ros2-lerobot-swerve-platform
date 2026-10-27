# head_rgbd_cpp

The D435 adapter of the head RGB-D contract (design D1). It pairs the RealSense D435 aligned depth with the colour
image by exact stamp, throttles the sets to `output.rate_hz`, downscales the depth, JPEG-encodes the colour and
publishes the `/head_rgbd/*` topics that RTAB-Map (through map3d_uplink), ground guidance, marker_detector and web_ui
read, plus a coloured point cloud. Registration is done by the camera (`align_depth.enable: true` of
`realsense_d435`); the node has no calibration file and needs no TF. The executable is `head_rgbd_node` (ROS node name
`head_rgbd`).

## Layout

- `src/core/*.cpp` build into the static library `head_rgbd_core` (no ROS node code): the config (`config.cpp`) and the
  cloud stage (`cloud.cpp`: `build_cloud`, `RateDecimator`) the node uses. `test/core/test_*.cpp` each become a gtest
  named after the file.
- `src/head_rgbd_node.cpp`, `include/head_rgbd_cpp/head_rgbd_node.hpp`: `HeadRgbdNode` and its pure helpers
  `downscale_depth`, `scale_camera_info` and `SetThrottle`; `src/main.cpp`: the executable. `test/test_node.cpp` tests
  the helpers and runs the node in-process with probe publishers of the three D435 inputs.
- `config/head_rgbd.yaml`: every key with its default, meaning and unit.

## Topics

All outputs are best_effort, depth 1, under `output_namespace` (default `/head_rgbd`).

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `depth_topic` | `sensor_msgs/Image` 16UC1, mm | D435 depth aligned to colour: colour size, colour stamp |
| in | `rgb_image_topic` | `sensor_msgs/Image` rgb8 or bgr8 | raw D435 colour (client-local; raw images never cross WiFi) |
| in | `rgb_info_topic` | `sensor_msgs/CameraInfo` | colour camera_info; the latest one is kept |
| out | `<ns>/depth/image_raw` | `sensor_msgs/Image` 16UC1, mm | colour size / `output.divisor`, 0 = no depth |
| out | `<ns>/depth/camera_info` | `sensor_msgs/CameraInfo` | the colour camera_info scaled to the depth size |
| out | `<ns>/rgb/image_raw/compressed` | `sensor_msgs/CompressedImage` | the colour at full size, JPEG `output.jpeg_quality`, format `bgr8; jpeg compressed bgr8` |
| out | `<ns>/rgb/camera_info` | `sensor_msgs/CameraInfo` | the colour camera_info |
| out | `<ns>/points` | `sensor_msgs/PointCloud2` XYZRGB | only when `cloud.enabled`; at most `cloud.rate_hz`, only while it has subscribers |

`<ns>/depth_upsampled/*` is not advertised: the aligned depth already has the colour geometry.

Pairing: the depth and colour subscriptions feed a `message_filters` `ExactTime` synchroniser (queue
`PAIR_QUEUE_SIZE` = 5 stamps); the D435 aligned depth carries the colour stamp. A pair is checked, then throttled
(`SetThrottle`: a fixed grid of 1 / `output.rate_hz` on the colour stamps, so a 15 Hz stream gives `rate_hz` on average),
then published. Every message of a set (both images, both camera_infos and the cloud) carries the colour header
exactly, stamp and `frame_id` (`d435_color_optical_frame`), so consumers pair them with an exact-time policy.

Depth downscale (`output.divisor` 2): an output pixel takes the first pixel of its 2x2 source block, in row-major order
from the top-left one (the nearest-neighbour sample), whose depth lies in [`depth.min_m`, `depth.max_m`]; 0 when none
does. Depth is never interpolated across missing pixels. With divisor 1 only the range applies. The depth camera_info
divides width, height and the first two rows of K and P by the divisor (output pixel u samples source pixel
divisor * u); D and R are kept.

## Configuration

`/etc/ros2/head_rgbd_cpp/config.yaml`, path overridable with `HEAD_RGBD_CONFIG`. Unknown keys are rejected at every
level (the ToF-era `tof_depth_topic`, `tof_calibration_file`, `sync`, `motion`, `filter` and `upsampling` included) and
any error exits 78 with one `<key>: <problem>` line per error.

| Key | Default | Meaning |
|---|---|---|
| `depth_topic` | `/head/d435/aligned_depth_to_color/image_raw` | aligned depth input (16UC1, mm) |
| `rgb_image_topic` | `/head/d435/color/image_raw` | raw colour input (rgb8 or bgr8) |
| `rgb_info_topic` | `/head/d435/color/camera_info` | colour camera_info input |
| `output_namespace` | `/head_rgbd` | prefix of every output topic |
| `output.divisor` | 2 | depth size = colour size / divisor (1 or 2); the colour stays at full size |
| `output.rate_hz` | 2.0 | published sets per second (0 < v <= 15), the mapping rate RTAB-Map is budgeted for |
| `output.jpeg_quality` | 80 | JPEG quality of the published colour (1..100) |
| `depth.min_m` / `depth.max_m` | 0.15 / 4.0 | valid depth range, m (0.15 m is the D435 min-Z at 640x360); outside it the depth is 0 |
| `cloud.enabled` | true | publish the coloured point cloud |
| `cloud.rate_hz` | 1.0 | point cloud rate, Hz (0 < v <= 10); clouds are built from published sets only |
| `lazy.idle_unsubscribe_s` | 5.0 | release the inputs this long after the last output subscriber left, s |
| `metrics_port` | 19117 | Prometheus port, `null` disables |

## Metrics

Served on `127.0.0.1:<metrics_port>` (`/metrics`), plus `robot_node_info` and the `process_*` series.

| Series | Type | Meaning |
|---|---|---|
| `head_rgbd_sets_published_total` | counter | sets published |
| `head_rgbd_dropped_total{reason}` | counter | input sets or frames dropped, by reason (see below) |
| `head_rgbd_process_seconds` | histogram | processing time of one published set (downscale, JPEG, cloud) |
| `head_rgbd_valid_ratio` | gauge | fraction of published depth pixels holding a depth, last set |

Sets thinned by the throttle are not drops. Drop logs are throttled to one per 5 s.

## Lazy inputs

A 1 s timer counts the subscribers of the output publishers. With at least one, the node creates its three input
subscriptions; with none for `lazy.idle_unsubscribe_s` it destroys them and forgets the camera_info. Idle, the node
therefore subscribes to nothing and costs no decode, copy or encode work.

## No placeholder data

Nothing is published for a set that is not an exact-stamp aligned depth + colour pair with a camera_info; the set is
dropped and counted in `head_rgbd_dropped_total{reason}`:

| reason | Dropped when |
|---|---|
| `bad_input` | the depth is not little-endian 16UC1, the colour is not rgb8 or bgr8, an image's data is shorter than its size, or the image is smaller than `output.divisor` |
| `size_mismatch` | the depth and colour sizes differ |
| `no_camera_info` | no camera_info arrived yet, the latest one is not of the colour size, or its K is zero or has a non-positive focal length |
| `unpaired` | a depth or colour frame left the pairing queue without an exact-stamp partner (one count per frame) |

## Test

`scripts/cpp-node-test.sh head_rgbd_cpp` (gtests and ament linters) and `scripts/cpp-node-test.sh head_rgbd_cpp
--lint-only`.
