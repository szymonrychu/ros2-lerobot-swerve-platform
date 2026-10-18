---
title: ToF jump edge filter
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Smeared ToF pixels on object edges are removed before they become floating depth points in the registered image and the 3D map.
files: [nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp, nodes/head_rgbd_cpp/src/core/tof_filter.cpp, nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp]
verify: scripts/cpp-node-test.sh head_rgbd_cpp
depends_on: [head-rgbd-package-config]
owns: [head-rgbd-tof-filter]
uses: [head-rgbd-config]
model: sonnet
estimate: 45
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--tof-jump-edge-filter
verified_head: ""
run: ""
provides: []
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--tof-jump-edge-filter
---

# ToF jump edge filter

## Requirement

- `tof_filter.hpp/.cpp` (D7, fix-as-ac: flying pixels): `cv::Mat filter_jump_edges(const cv::Mat &depth_mm, const FilterConfig &cfg, FilterStats *stats = nullptr)` on CV_16UC1 (0 = invalid): a valid pixel becomes 0 when any valid 8-neighbour differs by more than `max(cfg.jump_abs_mm, cfg.jump_rel * z)` or it has fewer than `cfg.min_valid_neighbours` valid neighbours; image borders count missing neighbours as invalid; the input is not modified; `FilterStats` counts removed jump and isolated pixels.
- Tests first in `test/core/test_tof_filter.cpp` on synthetic scenes, seen failing: a 1.0 m foreground / 3.0 m background step with one column of mixed pixels at 2.0 m: the mixed column is removed and no pixel deeper than 1 px from the step on either side is touched; flat planes at 0.5, 2 and 4 m and planes slanted 30, 60 and 70 deg (depth computed through a 240x180 pinhole with fx=191) keep >= 99 % of their pixels; an isolated valid pixel in an invalid region is removed; an all-invalid image stays all-invalid; noise of 1 % of depth on a plane removes < 1 % of pixels; the input Mat is unchanged.

## Acceptance conditions

- a1: mixed pixels on a synthetic depth step are removed while the surfaces on both sides stay [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a2: flat and slanted planes up to 70 deg survive the filter [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: isolated speckle is removed and invalid pixels stay invalid [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]

## Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`
- `nodes/head_rgbd_cpp/src/core/tof_filter.cpp`
- `nodes/head_rgbd_cpp/test/core/test_tof_filter.cpp`

## Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/tof_filter.hpp`: `filter_jump_edges`, `FilterStats` (part `head-rgbd-tof-filter`).

## Expected outcome

A tested flying-pixel filter for the raw ToF depth, called by the node before registration.

## Steps

- Write test_tof_filter.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement tof_filter.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): jump-edge flying-pixel filter`.

## Notes

- The bridge's confidence threshold does not remove flying pixels (SCCG 2008, doi:10.1145/1921264.1921293); this filter runs on every frame before reprojection.
