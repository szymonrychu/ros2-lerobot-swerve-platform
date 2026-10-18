---
title: RGBD guided upsampling
status: todo
repo: ros2-lerobot-sverve-platform
value: "Optionally, registered depth is upsampled to the RGB mapping size along real RGB edges, without ever inventing depth across holes."
files: [nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp, nodes/head_rgbd_cpp/src/core/upsample.cpp, nodes/head_rgbd_cpp/test/core/test_upsample.cpp]
verify: scripts/cpp-node-test.sh head_rgbd_cpp
depends_on: [rgbd-registration-zbuffer]
owns: [head-rgbd-upsampling]
uses: [head-rgbd-config, head-rgbd-registration, head-rgbd-synthetic-scene]
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

# RGBD guided upsampling

## Requirement

- `upsample.hpp/.cpp` (D8): `cv::Mat guided_upsample(const cv::Mat &depth_mm, const cv::Mat &guide_bgr, const UpsamplingConfig &cfg, UpsampleStats *stats = nullptr)`: guide size = 2x depth size (throws otherwise); D0 = nearest-upsampled depth, M = its validity mask (float 0/1); I = grey guide in [0, 1]; `num = cv::ximgproc::guidedFilter(I, D0 * M, radius, eps)`, `den = guidedFilter(I, M, radius, eps)`; output = num / den where `den >= cfg.min_support` and the pixel lies within `cfg.radius` of a valid D0 pixel (distance transform of the mask), else 0 (fix-as-ac: no placeholder depth outside the mask); CV_16UC1 mm.
- Tests first in `test/core/test_upsample.cpp` with `synthetic_scene.hpp`, seen failing: a depth step (1 m / 2 m) whose RGB edge is sharp: the upsampled step lies within 1 px of the RGB edge, while plain nearest upsampling is off by up to 2 px (both measured); a hole wider than 2 * radius stays 0 in its interior; pixels next to a hole are not pulled toward 0 (within 2 % of the true depth); a slanted plane's upsampled RMSE against the rendered 576x324 ground truth is lower than nearest-neighbour upsampling; a guide of the wrong size throws; all-invalid input gives all-zero output.

## Acceptance conditions

- a1: upsampled depth edges follow the RGB edges more closely than nearest upsampling [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: holes larger than the radius stay empty and zeros never bleed into valid depth [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: upsampling lowers the error against ground truth on a slanted plane [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

## Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`
- `nodes/head_rgbd_cpp/src/core/upsample.cpp`
- `nodes/head_rgbd_cpp/test/core/test_upsample.cpp`

## Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/upsample.hpp`: `guided_upsample`, `UpsampleStats` (part `head-rgbd-upsampling`).

## Expected outcome

A tested optional upsampler with a validity mask; disabled by default.

## Steps

- Write test_upsample.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement upsample.cpp with cv::ximgproc::guidedFilter; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): masked guided-filter upsampling`.

## Notes

- Library first: cv::ximgproc::guidedFilter (O(N), radius-independent); do not hand-write box-filter guided filtering.
