---
title: RGBD registration zbuffer
status: todo
repo: ros2-lerobot-sverve-platform
value: "ToF depth is reprojected into the RGB camera with correct occlusion and no out-of-image writes, giving pixel-aligned depth for colour."
files: [nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp, nodes/head_rgbd_cpp/src/core/registration.cpp, nodes/head_rgbd_cpp/test/core/test_registration.cpp, nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp]
verify: scripts/cpp-node-test.sh head_rgbd_cpp
depends_on: [head-rgbd-package-config]
owns: [head-rgbd-registration, head-rgbd-synthetic-scene]
uses: [head-rgbd-config]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# RGBD registration zbuffer

## Requirement

- `registration.hpp/.cpp` (D7), no ROS types:
  - `struct PinholeModel { cv::Matx33d K; cv::Vec<double, 5> D; int width; int height; }` and `PinholeModel scale_model(const PinholeModel &m, int width, int height)` (K scaled, D unchanged; throws on an aspect change > 1 px).
  - `class RayTable` built once from the ToF model (`cv::undistortPoints` of every pixel centre and of the pixel corners) giving the normalised ray per pixel and per corner.
  - `cv::Mat register_depth(const cv::Mat &tof_depth_mm, const RayTable &rays, const Eigen::Isometry3d &T_rgb_tof, const PinholeModel &rgb_out, const DepthConfig &range, RegistrationStats *stats = nullptr)`: for each valid ToF pixel with z in range: 3D point = ray * z; transform with `T_rgb_tof`; skip points with z <= 0; project the pixel's four corner points (same depth) into `rgb_out` with plumb_bob distortion (`cv::projectPoints` or the equivalent closed form, tested equal); fill the clipped bounding rectangle of the projected corners with the transformed point's z in mm, keeping the minimum per output pixel (z-buffer); output CV_16UC1 of `rgb_out` size, 0 where no point landed.
  - `cv::Rect clip_footprint(double u_min, double v_min, double u_max, double v_max, int width, int height)`: the integer pixel rectangle covered, clipped to the image; empty when fully outside (fix-as-ac: depth_image_proc's `fill_upsampling_holes` bounds bug).
- `test/core/synthetic_scene.hpp` (shared by motion and upsampling tests): planes and boxes rendered into a ToF depth image through a `PinholeModel` at a given ToF pose, ground-truth depth rendered into an RGB model at a given RGB pose, plus the RGB=ToF identity helpers.
- Tests first in `test/core/test_registration.cpp`, seen failing: identical models and identity transform reproduce the input exactly; a fronto-parallel plane at 1 m with the RGB 0.05 m right of the ToF lands shifted by fx_out * 0.05 / 1.0 px (+-0.5 px) and its depth is 1000 mm; a slanted plane matches the rendered RGB ground truth within 5 mm on >= 95 % of covered pixels; occlusion: a box at 0.8 m in front of a wall at 2.5 m, seen from the offset RGB: every output pixel covered by the box holds box depth (nearest wins) and the occlusion shadow (wall visible to RGB, hidden from the ToF) stays 0, never wall-interpolated; a footprint crossing each of the four borders (including u_max < 0 and u_min >= width) returns the clipped or empty rectangle, and a plane partly outside the RGB view produces correct border columns without any write outside the Mat (AddressSanitizer is not available, so the test also checks a guard band around a sub-Mat view stays untouched); the default 288x162 output from the default 240x180 ToF on a 1.5 m wall covers >= 90 % of the overlap area (splat closes the 1:1.15 gaps); distortion: a non-zero RGB k1 moves projected points to the distorted positions given by `cv::projectPoints` within 0.05 px.

## Acceptance conditions

- a1: a known baseline produces the expected disparity and depth on a synthetic plane [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: occlusion keeps the nearest depth and leaves occlusion shadows empty [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: footprints crossing any image border are clipped with no out-of-image write [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a4: the default 288x162 output fills the ToF overlap of a flat wall [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a5: RGB distortion is applied consistently with OpenCV [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

## Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`
- `nodes/head_rgbd_cpp/src/core/registration.cpp`
- `nodes/head_rgbd_cpp/test/core/test_registration.cpp`
- `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp`

## Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`: `PinholeModel`, `scale_model`, `RayTable`, `register_depth`, `clip_footprint`, `RegistrationStats` (part `head-rgbd-registration`).
- `nodes/head_rgbd_cpp/test/core/synthetic_scene.hpp` (part `head-rgbd-synthetic-scene`).

## Expected outcome

Tested z-buffer registration from the ToF into any scaled RGB model, ready for the motion-compensated transform.

## Steps

- Write synthetic_scene.hpp and test_registration.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement registration.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): z-buffer ToF-to-RGB registration`.

## Notes

- Opus: projection direction, distortion order (undistort ToF rays, distort RGB projection) and the z-buffer are where correctness lives.
- Do not use depth_image_proc::RegisterNode (design D7); its footprint idea is kept with correct clipping.
- Output depth is z in the RGB optical frame (not ray length), in mm, rounded.
