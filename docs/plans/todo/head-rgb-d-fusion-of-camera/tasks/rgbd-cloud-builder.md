---
title: RGBD cloud builder
status: todo
repo: ros2-lerobot-sverve-platform
value: "Consumers get a coloured 3D point cloud from the registered pair at a low, configurable rate."
files: [nodes/head_rgbd_cpp/include/head_rgbd_cpp/cloud.hpp, nodes/head_rgbd_cpp/src/core/cloud.cpp, nodes/head_rgbd_cpp/test/core/test_cloud.cpp]
verify: scripts/cpp-node-test.sh head_rgbd_cpp
depends_on: [head-rgbd-package-config]
owns: [head-rgbd-cloud]
uses: [head-rgbd-config]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# RGBD cloud builder

## Requirement

- `cloud.hpp/.cpp` (D9):
  - `class RateDecimator` (`bool due(int64_t stamp_ns)`: first stamp, then whenever at least `1e9 / rate_hz` ns passed since the last accepted; a backwards stamp resets).
  - `cv::Mat decode_rgb_for_depth(const std::vector<uint8_t> &jpeg, int width, int height)`: `cv::imdecode` with `IMREAD_REDUCED_COLOR_2` when the target is half the JPEG size, else full decode plus `INTER_AREA` resize; empty Mat on a corrupt JPEG.
  - `sensor_msgs::msg::PointCloud2::SharedPtr build_cloud(const sensor_msgs::msg::Image &depth, const sensor_msgs::msg::CameraInfo &depth_info, const cv::Mat &rgb)`: XYZRGB cloud in the depth frame with depth_image_proc's `convertDepth<uint16_t>` and `convertRgb` (`depth_image_proc/conversions.hpp`) and `image_geometry::PinholeCameraModel`; unorganised points with depth 0 dropped (`is_dense` true); returns nullptr when rgb is empty or sizes differ (no cloud with made-up colour).
- Tests first in `test/core/test_cloud.cpp`, seen failing: a 288x162 depth of a plane at 1 m with a known camera_info produces points with z = 1.0 m and x/y matching the pinhole back-projection within 1 mm, colours taken from the matching RGB pixel; zero-depth pixels produce no points; a corrupt JPEG gives an empty Mat and no cloud; the reduced decode of a 576x324 JPEG returns 288x162; the decimator at 1 Hz accepts 1 of every 2 pairs at 2 Hz over 10 s (+-1).

## Acceptance conditions

- a1: the cloud reproduces pinhole geometry and RGB colours from a synthetic pair [evidence: test: nodes/head_rgbd_cpp/test/core/test_cloud.cpp]
- a2: no points for invalid depth and no cloud without a real RGB image [evidence: test: nodes/head_rgbd_cpp/test/core/test_cloud.cpp]
- a3: the cloud rate decimator emits the configured rate [evidence: test: nodes/head_rgbd_cpp/test/core/test_cloud.cpp]

## Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/cloud.hpp`
- `nodes/head_rgbd_cpp/src/core/cloud.cpp`
- `nodes/head_rgbd_cpp/test/core/test_cloud.cpp`

## Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/cloud.hpp`: `RateDecimator`, `decode_rgb_for_depth`, `build_cloud` (part `head-rgbd-cloud`).

## Expected outcome

Tested coloured-cloud construction reusing depth_image_proc's conversions.

## Steps

- Write test_cloud.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement cloud.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): decimated coloured cloud`.

## Notes

- If `depth_image_proc/conversions.hpp` is not installed or exported by ros-jazzy-depth-image-proc, write the back-projection with image_geometry and say so in the commit message (design D9).
