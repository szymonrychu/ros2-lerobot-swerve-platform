---
title: Ground estimator on D435 inputs
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Ground estimation reads the D435 cloud and guidance with honest, sensor-neutral config."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/src/core/config.cpp, nodes/ground_estimator_cpp/src/core/guidance.cpp, nodes/ground_estimator_cpp/src/ground_node.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/test/test_node.cpp, nodes/ground_estimator_cpp/test/test_replay.cpp]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && scripts/cpp-node-test.sh ground_estimator_cpp --lint-only"
depends_on: [head-rgb-d-node-on-d435-alig]
model: opus
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Ground estimator on D435 inputs

## Requirement

Goal G3: "Ground estimation is restored on D435 depth ..." Design D7. Reuse-brief idea: "ground_estimator no_return and guidance assume ToF characteristics (ground_estimator.yaml:102-125, tof_info_topic)".

- Config (`include/ground_estimator_cpp/config.hpp`, `src/core/config.cpp`, `config/ground_estimator.yaml`):
  - `cloud_topic` default `/head/d435/points_filtered` (comment: D435 cloud, 15 Hz).
  - `guidance.depth_topic` `/head_rgbd/depth/image_raw`, `guidance.info_topic` `/head_rgbd/depth/camera_info` (depth_upsampled is retired, D1).
  - `guidance.tof_info_topic` renamed `guidance.depth_info_topic`, default `/head/d435/depth/camera_info` (the camera model of the sensor that produced the cloud, for no-return areas). `guidance.no_return.tof_margin_px` renamed `guidance.no_return.margin_px` (constant renamed accordingly). The old keys are rejected as unknown (no backward compatibility).
  - Every frame-count window is scaled by 1.5 and rounded up, so its time span at 15 Hz equals the old 10 Hz span: temporal window_frames 8->12, min_frames 4->6; hysteresis window_frames 5->8, enter_frames 3->5, exit_frames 1->2, area_enter_frames 3->5, area_join_frames 8->12, area_kind_frames 10->15, area_history_frames 20->30; no_return window_frames 4->6, enter_frames 3->5. The bounds are widened only where a scaled default needs it. The comments say "frames at 15 Hz".
- `src/ground_node.cpp` and `src/core/guidance.cpp` use the renamed fields; log and diagnostic text say "depth camera_info" instead of "ToF camera_info".
- Tests: `test/core/test_config.cpp` asserts the new defaults and keys, and that `tof_info_topic` and `tof_margin_px` are rejected. `test/test_node.cpp` and `test/test_replay.cpp` use the renamed fields (the old names are no longer accurate); their assertions are unchanged.
Notes: the client.yml ground_estimator_cpp block is rewritten by enable-rgb-d-and-ground-node; the replay fixtures are replaced by record-d435-ground-replay-fi; noise limits are re-tuned by tune-ground-estimator-on-d43.

## Acceptance conditions

- AC1: defaults read the D435 cloud and /head_rgbd/depth guidance with depth_info_topic and margin_px; the ToF keys are rejected (test_config) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC2: every frame-count window default is the 10 Hz value times 1.5 rounded up (test_config) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC3: the node and replay tests pass with the renamed fields [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- AC4: ament linters pass [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp --lint-only]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/src/core/guidance.cpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update `test/core/test_config.cpp` (new defaults, renamed keys, rejected old keys); run `scripts/cpp-node-test.sh ground_estimator_cpp`; expect failures.
- Rename the fields and defaults, scale the windows, update node, guidance and the two node tests; rerun until `cpp-node-test: ground_estimator_cpp PASS`.
- Run the lint-only pass; commit `feat(ground_estimator_cpp): D435 inputs and a sensor-neutral no-return model`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
