---
title: RGBD motion compensation
status: todo
repo: ros2-lerobot-sverve-platform
value: "Depth stays on the right pixels while the head pans or the robot turns, and frames too blurred to fix are dropped instead of smearing the map."
files: [nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp, nodes/head_rgbd_cpp/src/core/motion.cpp, nodes/head_rgbd_cpp/test/core/test_motion.cpp]
verify: scripts/cpp-node-test.sh head_rgbd_cpp
depends_on: [rgbd-registration-zbuffer]
owns: [head-rgbd-motion]
uses: [head-rgbd-config, head-rgbd-registration, head-rgbd-synthetic-scene]
model: opus
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# RGBD motion compensation

## Requirement

- `motion.hpp/.cpp` (D6), Eigen only, no tf2 types:
  - `Eigen::Isometry3d compensated_transform(const Eigen::Isometry3d &fixed_T_rgb_at_trgb, const Eigen::Isometry3d &fixed_T_tof_at_ttof)` = `inverse(fixed_T_rgb) * fixed_T_tof`: the transform that maps ToF points captured at t_tof into the RGB frame at t_rgb (the same result as tf2's time-travel `lookupTransform(rgb, t_rgb, tof, t_tof, fixed)`).
  - `MotionRate frame_rate(const Eigen::Isometry3d &fixed_T_a, const Eigen::Isometry3d &fixed_T_b, double dt_s)`: angular rate (deg/s, from the relative rotation angle) and linear rate (m/s) of a frame between two poses.
  - `PairDecision evaluate_pair(int64_t t_rgb_ns, int64_t t_tof_ns, const MotionRate &tof_rate, const SyncConfig &, const MotionConfig &)`: `accept`, or a drop reason `pair_dt` (|t_rgb - t_tof| > max_pair_dt_s) or `motion` (angular or linear rate above the limits); `MotionConfig::compensate = false` keeps the gate and makes the node use the static transform.
- Tests first in `test/core/test_motion.cpp` using `synthetic_scene.hpp` and `register_depth`, seen failing: the head pans at 30 deg/s around the pan axis (the URDF pan/tilt geometry: pan axis 0.04 m below the cameras), t_rgb - t_tof = 33 ms: registering with the compensated transform matches the ground-truth RGB depth with median edge error <= 0.5 px at 288x162, while the static transform is off by >= 3 px (both measured, proving compensation matters); base rotation of 0.5 rad/s with a static head is compensated the same way; `frame_rate` of a 10 deg rotation over 0.5 s is 20 deg/s and of a 0.1 m translation over 0.5 s is 0.2 m/s; the gate drops at 15 deg/s with the 10 deg/s limit and accepts at 5 deg/s; pair_dt 0.07 s is dropped with the 0.06 s limit; reason strings are the metric labels `pair_dt` and `motion`.

## Acceptance conditions

- a1: the compensated transform registers a panning head within 0.5 px where the static transform misses by 3 px or more [evidence: test: nodes/head_rgbd_cpp/test/core/test_motion.cpp]
- a2: frame angular and linear rates are computed from two poses [evidence: test: nodes/head_rgbd_cpp/test/core/test_motion.cpp]
- a3: pairs too far apart in time or captured during fast motion are dropped with their reason [evidence: test: nodes/head_rgbd_cpp/test/core/test_motion.cpp]

## Files

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`
- `nodes/head_rgbd_cpp/src/core/motion.cpp`
- `nodes/head_rgbd_cpp/test/core/test_motion.cpp`

## Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/motion.hpp`: `compensated_transform`, `frame_rate`, `MotionRate`, `evaluate_pair`, `PairDecision` (part `head-rgbd-motion`).

## Expected outcome

Tested motion maths: the node only performs the two TF lookups and calls these functions.

## Steps

- Write test_motion.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement motion.cpp; rerun until `PASS`.
- Commit `feat(head_rgbd_cpp): motion compensation and gate`.

## Notes

- Opus: frame conventions (which side is inverted) are the classic bug here; test both directions.
- A ToF frame integrates four phase captures, so blur during fast motion is gated, not corrected (design D6).
