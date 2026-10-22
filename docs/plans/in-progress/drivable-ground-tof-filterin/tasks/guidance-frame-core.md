---
title: Guidance frame core
status: done
repo: ros2-lerobot-sverve-platform
value: "Each RGB-D pair turned into floor-relative depth samples and no-return floor hits, the input of both guidance features"
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp, nodes/ground_estimator_cpp/src/core/guidance.cpp, nodes/ground_estimator_cpp/test/core/test_guidance.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [guidance-config-keys, replay-format-v2]
owns: [guidance-frame]
uses: [guidance-config]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--guidance-frame-core
branch: drivable-ground-tof-filterin--guidance-frame-core
verified_head: 7363ecfe7e78e916350331fc67fb71836c02b2b7
run: drivable-ground-tof-filterin
provides: [guidance-frame]
acs_for_audit: []
process_gaps: ["No red-phase output reported for the seven Guidance tests (TDD tests-first evidence)."]
---

# Guidance frame core

## Requirement

The guidance frame and its sampling, pure core code without ROS node wiring (part guidance-frame, design D3, D5, D12). It uses part guidance-config.

- `include/ground_estimator_cpp/guidance.hpp` + `src/core/guidance.cpp`:
  - `struct CameraModel {cv::Matx33d K; cv::Vec<double, 5> D; int width; int height;}` and `std::optional<CameraModel> camera_model_of(const sensor_msgs::msg::CameraInfo &)`, which is nullopt for an all-zero K, a non-plumb_bob model or a zero size: no made-up calibration.
  - `class RayGrid`: the normalised rays of the sampling grid (every `sample_step_px` pixel centre), built once per model and step with `cv::undistortPoints`, and rebuilt only when the model or the step changes.
  - `struct GuidanceFrame {int64_t stamp_ns; std::string frame_id; cv::Mat depth_mm /*CV_16UC1*/; cv::Mat bgr /*CV_8UC3*/; CameraModel model;}` and `std::optional<GuidanceFrame> decode_guidance(const Image & depth, const CameraInfo & info, const CompressedImage & rgb, std::string * why)`. It rejects, naming why: a depth that is not 16UC1 or is shorter than its size; a JPEG that does not decode (`cv::imdecode`); a JPEG size different from the depth; a camera_info size different from the depth.
  - `struct GuidanceSamples {pcl::PointCloud<pcl::PointXYZ> points; std::vector<float> heights; std::vector<cv::Vec3b> colours; pcl::PointCloud<pcl::PointXYZ> no_return; std::vector<cv::Vec3b> no_return_colours;}` (points in base_link, colours R G B).
  - `GuidanceSamples sample_guidance(const GuidanceFrame &, const RayGrid &, const Eigen::Isometry3d & base_T_cam, const Plane & plane, const CameraModel & tof, const Eigen::Isometry3d & tof_T_base, const GuidanceConfig &)`:
    - a grid pixel with depth z becomes the point ray * z in the camera frame, moved to base_link. Its signed height above the plane is kept when its range is <= `max_range_m`;
    - a grid pixel with depth 0 becomes a no-return sample when all of these hold: its ray meets the plane in front of the camera; the hit's horizontal range is within [`no_return.min_range_m`, `max_range_m`]; the hit, moved to the ToF frame and projected with the ToF model (plumb_bob), lands inside the ToF image by at least `no_return.tof_margin_px`;
    - every other pixel is not a sample.
- Tests first (seen failing) in `test/core/test_guidance.cpp`, synthetic, using the test_node head pose (0.45 m, 30 deg down):
  - `Guidance.RayGridMatchesProjection`: project then grid rays round trip within 1e-6 with a non-zero plumb_bob D;
  - `Guidance.FlatFloorHeightsAreZero`: a rendered floor depth gives |height| <= 1 mm;
  - `Guidance.HoleOnTheFloorBecomesNoReturnSamplesAtTheRightPlace`: the hit is within 5 mm of the rendered floor point;
  - `Guidance.HitOutsideTofViewIsNoSample`;
  - `Guidance.HitOutsideRangeIsNoSample`;
  - `Guidance.BadInputsAreRejectedWithAReason`: wrong encoding, short data, corrupt JPEG, size mismatch;
  - `Guidance.CameraModelOfRejectsEmptyCalibration`.

## Acceptance conditions

- a1: rays, floor heights, no-return hits, the ToF view and range tests, and input rejection pass [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: sample_guidance is in the core [evidence: symbol: nodes/ground_estimator_cpp/src/core/guidance.cpp::sample_guidance]
- a3: decode_guidance names why it rejects an input [evidence: symbol: nodes/ground_estimator_cpp/src/core/guidance.cpp::decode_guidance]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/guidance.hpp`
- `nodes/ground_estimator_cpp/src/core/guidance.cpp`
- `nodes/ground_estimator_cpp/test/core/test_guidance.cpp`

## Interfaces

Builds part guidance-frame in `guidance.hpp`: `CameraModel`, `camera_model_of`, `RayGrid`, `GuidanceFrame`, `decode_guidance`, `GuidanceSamples`, `sample_guidance`. Uses `GuidanceConfig` (part guidance-config) and `Plane` (`plane_fit.hpp`).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the seven `Guidance.*` tests.

## Steps

- write `test/core/test_guidance.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no header)
- implement `guidance.hpp`/`guidance.cpp`; the command PASSES
- commit `feat(ground_estimator_cpp): guidance frame sampling`

## Ruling

- 2026-10-22 accepted: verifier agent died; verification on HEAD: Summary 395 tests, 0 failures, cpp-node-test PASS

## Notes

- Conventions as in `nodes/head_rgbd_cpp/include/head_rgbd_cpp/registration.hpp`: OpenCV pixel centres, plumb_bob with D applied on projection, undistortPoints for rays. Reuse the rule, not the code (different package).
- The upsampled depth is z along the optical axis in mm.
- `test/core/synthetic_floor.hpp` has the head pose helpers.
