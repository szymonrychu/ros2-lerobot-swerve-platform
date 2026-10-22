---
title: Guidance replay metrics
status: done
repo: ros2-lerobot-sverve-platform
value: "Edge, no-return and board metrics against the truth, printed for every replay"
files: [nodes/ground_estimator_cpp/test/replay_fixture.hpp, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/test/data/replay/tiny_truth.yaml, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [replay-format-v2, scene-truth-from-board, guidance-config-keys]
owns: [guidance-replay-harness]
uses: [replay-format-v2, scene-truth, guidance-config]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--guidance-replay-metrics
branch: drivable-ground-tof-filterin--guidance-replay-metrics
verified_head: 3b55c265daeba45c6eb654ac700e1b2f375bafd2
run: drivable-ground-tof-filterin
provides: [guidance-replay-harness, truth-replay-metrics]
acs_for_audit: [a4]
process_gaps: ["TDD evidence missing: no red/green tails (tests.red, verification.evidence) for test_replay.cpp; no listed TDD exception applies"]
---

# Guidance replay metrics

## Requirement

The replay harness for the guidance (part guidance-replay-harness, design D9, D10). It feeds the v2 pairs (part replay-format-v2) to the real GroundNode with the topics of part guidance-config, reads the scene truth (part scene-truth), and adds the metrics the acceptance asserts. No per-scene assertion here; that comes in guidance-replay-acceptance.

- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`:
  - `SceneTruth load_truth(const std::string & path)` (yaml-cpp) holds the board, box, ramp and empty polygons and the box front edge segment, in base_link metres. Every missing key is an error.
  - The metrics are plain functions over `std::vector<ReplayFrame>` and a `SceneTruth`, frames after the 2 s warm-up, ignoring points with x < 0.37 (gripper zone, as run 4):
    - `box_front_edge_error_m`: per frame, the obstacle points within 0.10 m of the box footprint are taken, and the edge is sampled every 0.02 m over its middle 80 %. For each sample, the distance along the edge normal to the nearest obstacle point within 0.02 m of the sample's line is measured; a sample without such a point counts as the 0.10 m search limit. The result is the p95 over all samples of all frames.
    - `box_width_error_m`: per frame, the extent of those obstacle points along the edge direction minus the true width. The result is the p95 of the absolute value.
    - `outside_lethal_frames_share`: the share of frames with any obstacle point more than 0.03 m outside the box footprint and within 0.15 m of it.
    - `no_return_cover_share`: the share of frames in which the no_return areas cover >= 70 % of the ramp footprint. Coverage is rasterised on a 0.01 m grid.
    - `no_return_stray_share`: the share of frames with a no_return area cell centre more than 0.10 m (one cell, my call) outside the ramp footprint.
    - `empty_lethal_share`: the share of frames with an obstacle point inside the empty rectangle.
  - Truth-based overloads of run 4's metrics. Run 4 (ground-detection-robustness/replay-fixtures-and-test, part replay-harness) defines `box_lethal_share`, `board_lethal_frames`, `stray_lethal_share`, `ramp_lethal_frames`, `far_side_warning_frames`, `churn_share` and `plane_share` over its fixed 2026-10-22 ROI constants. This task adds overloads of the same names taking a `SceneTruth`, with the truth polygons in place of the constants:
    - `box_lethal_share(frames, truth)`: the box ROI is the box footprint grown by 0.03 m;
    - `board_lethal_frames(frames, truth)`: frames with an obstacle point inside the board polygon shrunk by 0.02 m;
    - `stray_lethal_share(frames, truth)`: x >= 0.37, hypot(x, y) <= 1.2, outside the grown box footprint;
    - `ramp_lethal_frames(frames, truth)`: the ramp footprint.

    `churn_share` and `plane_share` have no ROI and are used unchanged. Run 4's constant-ROI versions stay for its own fixtures. Every metric name means one thing; the truth overload differs only in where its ROI comes from.
  - `ReplayFrame` keeps the parsed warning areas with kind, source and nullable heights.
- `nodes/ground_estimator_cpp/test/test_replay.cpp` harness:
  - For a v2 fixture, each pair is published at its restamped time, after the clouds before it. The TF buffer gets base_link <- the RGB frame and odom <- base_link at the pair stamp. The upsampled depth (Image 16UC1), its camera_info (the RGB model) and the CompressedImage (JPEG) go out with one header, on the shipped config's `guidance.*` topics, and the ToF camera_info on `guidance.tof_info_topic` once per cloud.
  - At <= 1 Hz an XYZRGB cloud goes out on `rgbd_cloud_topic` (it feeds the floor colour model as head_rgbd_cpp does). It is built from the pair: the depth halved (the minimum valid depth of each 2x2 block, 0 when none), the JPEG decoded with `IMREAD_REDUCED_COLOR_2`, deprojected with the halved model, and the colour packed as float rgb.
  - `enum class Guidance {off, on}` overrides `guidance.enabled` of the loaded config.
  - One `run_scene(path, Guidance)` returns the frames, and one `print_metrics(label, frames, truth)` prints all the metrics on one line.
- Tests (written first, seen failing), on hand-built frames: one per new metric and per truth overload, both sides of each threshold and of the x < 0.37 exclusion; `SceneTruth.LoadsAndRejectsMissingKeys` on `test/data/replay/tiny_truth.yaml` (committed) and a broken copy written at test time; `Harness.PublishesPairsOfTinyV2`: a probe rclcpp node in the test subscribes to the guidance topics and to `rgbd_cloud_topic`, and receives the 2 pairs, each with one shared header, and the derived XYZRGB cloud. GroundNode does not consume the pairs yet (guidance-node-wiring), so the test never depends on it.
- README: the `## Replay tests` section gains the new metric definitions.

## Acceptance conditions

- a1: every new metric has unit tests on both sides of its thresholds, the truth loader rejects missing keys, and the harness publishes the tiny_v2 pairs [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the box front edge metric is in the replay fixture header [evidence: symbol: nodes/ground_estimator_cpp/test/replay_fixture.hpp::box_front_edge_error_m]
- a3: the no-return cover metric is in the replay fixture header [evidence: symbol: nodes/ground_estimator_cpp/test/replay_fixture.hpp::no_return_cover_share]
- a4: the README defines the new metrics [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]

## Files

- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/test/data/replay/tiny_truth.yaml`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

Builds part guidance-replay-harness in `test/replay_fixture.hpp` and `test/test_replay.cpp`:
- `SceneTruth`, `load_truth`, the six new metric functions, the truth overloads of run 4's `box_lethal_share`, `board_lethal_frames`, `stray_lethal_share` and `ramp_lethal_frames`, `Guidance`, `run_scene` and `print_metrics`;
- the pair, ToF camera_info and derived XYZRGB publishing.

Uses replay-format-v2, scene-truth and guidance-config (topic and enabled keys). Run 4's metric functions and its replay tests stay unchanged.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the metric, truth-loader and harness tests; `tiny_truth.yaml` committed.

## Steps

- write the metric, truth and harness tests in `test/test_replay.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (missing functions)
- implement them in `replay_fixture.hpp` and the harness; the command PASSES
- update the README metric definitions; commit `test(ground_estimator_cpp): guidance replay metrics and harness`

## Ruling

- 2026-10-22 accepted: verifier agent died; verification: Summary 397 tests, 0 failures, cpp-node-test PASS

## Notes

Restamping and TF filling follow run 4's `run_frame` pattern in `test_replay.cpp`; the pairs interleave with the clouds by stamp. The derived XYZRGB cloud mimics `nodes/head_rgbd_cpp/src/core/cloud.cpp:build_cloud` (packed float rgb, see colour.hpp `to_labelled_msg` for the packing used here).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)
