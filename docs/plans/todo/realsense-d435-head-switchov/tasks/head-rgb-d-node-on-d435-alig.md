---
title: Head RGB-D node on D435 aligned input
status: todo
repo: ros2-lerobot-sverve-platform
value: "RTAB-Map, ground guidance, marker_detector and web_ui keep their /head_rgbd contract, now fed by the D435."
files: [nodes/head_rgbd_cpp/src/head_rgbd_node.cpp, nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp, nodes/head_rgbd_cpp/src/core/config.cpp, nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp, nodes/head_rgbd_cpp/test/test_node.cpp, nodes/head_rgbd_cpp/test/core/test_config.cpp, nodes/head_rgbd_cpp/config/head_rgbd.yaml, nodes/head_rgbd_cpp/README.md]
verify: "scripts/cpp-node-test.sh head_rgbd_cpp && scripts/cpp-node-test.sh head_rgbd_cpp --lint-only"
depends_on: []
model: opus
review: ""
verified_head: ""
points: 8
expected_minutes: 91
expected_tokens: 6000000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Head RGB-D node on D435 aligned input

## Requirement

Goal G3: "Ground estimation is restored on D435 depth: ground_estimator_cpp (and head_rgbd_cpp or its replacement) enabled ..." Design D1 (head_rgbd_cpp stays as the D435 adapter stage), D13.

Rework `HeadRgbdNode` so its input is the D435 aligned pair instead of ToF + Camera Module 3:
- Config (`config.hpp`/`config.cpp`, `config/head_rgbd.yaml`): `depth_topic` (default `/head/d435/aligned_depth_to_color/image_raw`, 16UC1 mm, already registered to colour), `rgb_image_topic` (`/head/d435/color/image_raw`, raw rgb8/bgr8), `rgb_info_topic` (`/head/d435/color/camera_info`), `output_namespace`, `output.divisor` (1 or 2, depth only), `output.rate_hz` (0 < v <= 15, default 2.0, the mapping rate RTAB-Map is budgeted for), `output.jpeg_quality` (1..100, default 80), `depth.min_m` (default 0.15, D435 min-Z at 640x360) / `depth.max_m` (4.0), `cloud.enabled`/`cloud.rate_hz`, `lazy.idle_unsubscribe_s`, `metrics_port`. Unknown keys are rejected as today.
- Removed keys (ToF-only, no backward compatibility): `tof_depth_topic`, `tof_calibration_file`, `sync.*`, `motion.*`, `filter.*`, `upsampling.*`.
- Node: exact-time pairing of depth + colour (the D435 aligned depth carries the colour stamp) with the latest camera_info of that size. Sets are throttled to `output.rate_hz`. Depth is downscaled by the divisor with nearest-valid (no interpolation across zeros), and depth camera_info is scaled to match. The RGB is JPEG-encoded at full size. Outputs `/head_rgbd/{depth/image_raw, depth/camera_info, rgb/image_raw/compressed, rgb/camera_info, points}`, every message of a set carrying the colour header (stamp and `d435_color_optical_frame`). `/head_rgbd/depth_upsampled/*` is no longer advertised. The node no longer includes motion.hpp, tof_filter.hpp, upsample.hpp or registration.hpp and no longer reads a calibration file; those sources are deleted by the two retire tasks.
- No placeholder data: a set is dropped (metric `head_rgbd_dropped_total{reason}`) when depth is not 16UC1 (`bad_input`), depth and colour sizes differ (`size_mismatch`), camera_info is missing or has a zero K (`no_camera_info`), or no exact partner arrives (`unpaired`).
- `test/test_node.cpp`: new gtests `PublishesTheAlignedSetWithTheColourStampAndFrame`, `ThrottlesSetsToTheOutputRate`, `DownscalesDepthAndCameraInfoByTheDivisor`, `DropsNon16uc1Depth`, `DropsAMismatchedSize`, `DropsWithoutCameraInfo`, `DoesNotAdvertiseUpsampledTopics`. Kept: `InputsAreSubscribedOnlyWhileAnOutputHasSubscribers`, `CloudIsPublishedAtMostAtTheConfiguredRate`. Removed as no longer accurate (the ToF behaviours they test are deleted): `PublishesRegisteredPairWithTheRgbStampAndFrame`, `DropsWithoutTransform`, `DropsWithoutCalibrationFile`, `DropsAZeroCameraMatrix` (replaced by `DropsWithoutCameraInfo`), `DropsAWrongTofSize`, `DropsDuringFastMotion`, `UpsampledTopicsExistOnlyWhenEnabled`, `UpsampledDepthHasTheRgbSizeWhenEnabled`. `test/core/test_config.cpp` covers the new keys and rejects the removed ones.
- `README.md`: inputs, outputs, config table, drop reasons.
Notes: reuse `src/core/cloud.cpp` build_cloud as is. CMakeLists globs `src/core`; it may need the removed headers' includes dropped only. The client.yml config block is changed by head-rgb-d-build-deps-cleanu.

## Acceptance conditions

- AC1: an exact-stamp aligned depth + colour pair becomes one /head_rgbd set with the colour stamp and frame (gtest PublishesTheAlignedSetWithTheColourStampAndFrame) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC2: sets are throttled to output.rate_hz and depth plus its camera_info are downscaled by the divisor (gtests ThrottlesSetsToTheOutputRate, DownscalesDepthAndCameraInfoByTheDivisor) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC3: non-16UC1 depth, mismatched sizes and missing camera_info publish nothing (gtests DropsNon16uc1Depth, DropsAMismatchedSize, DropsWithoutCameraInfo) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC4: no depth_upsampled topic is advertised and the ToF config keys are rejected (gtest DoesNotAdvertiseUpsampledTopics, test_config) [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp]
- AC5: ament linters pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp --lint-only]

## Files

- `nodes/head_rgbd_cpp/src/head_rgbd_node.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/head_rgbd_node.hpp`
- `nodes/head_rgbd_cpp/src/core/config.cpp`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`
- `nodes/head_rgbd_cpp/test/test_node.cpp`
- `nodes/head_rgbd_cpp/test/core/test_config.cpp`
- `nodes/head_rgbd_cpp/config/head_rgbd.yaml`
- `nodes/head_rgbd_cpp/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the new gtests in `test/test_node.cpp` and `test/core/test_config.cpp`, remove the listed ToF ones; run `scripts/cpp-node-test.sh head_rgbd_cpp`; expect failures (config keys and node behaviour missing).
- Rework config and node per the Requirement; rerun until `cpp-node-test: head_rgbd_cpp PASS`.
- Run `scripts/cpp-node-test.sh head_rgbd_cpp --lint-only`; commit `feat(head_rgbd_cpp): D435 aligned depth and colour as the input`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
