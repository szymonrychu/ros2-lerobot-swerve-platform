---
title: Wire shadow gate on replay
status: done
repo: ros2-lerobot-sverve-platform
value: "On the user's real scene the box is a stable obstacle and no fake ramp appears behind it."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp, nodes/ground_estimator_cpp/src/ground_node.cpp, nodes/ground_estimator_cpp/test/test_node.cpp, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, nodes/ground_estimator_cpp/test/core/test_config.cpp]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q"
depends_on: [replay-fixtures-and-test, shadow-gate-and-segment-step, tof-flying-pixel-filter]
owns: []
uses: [replay-harness, occlusion-gate, segment-step, mixed-pixel-scene, replay-fix-config]
model: sonnet
estimate: 90
actual: ""
review: ""
branch: main
verified_head: 0dfbd483
run: ground-detection-robustness
provides: []
acs_for_audit: []
process_gaps: []
---

# Wire shadow gate on replay

## Requirement

Put the occlusion shadow gate and the boundary-segment step to work in GroundNode and prove them on the real recordings (design D14, D16, D17; brief D1).

- `src/ground_node.cpp`, `include/ground_estimator_cpp/ground_node.hpp`: per ToF cloud in `process_plane`, after the plane is known and before `cell_filter_.apply`, compute the plane-relative heights of the base_link cloud, call `occluded_mask` (sensor origin = translation of base_T_cloud, its height above the plane) and pass only the points that are not occluded to the temporal filter, segmentation and rule; the plane fit keeps using every point. Occluded points are never published. A gauge `ground_occluded_points` (points dropped in the last cloud), registered like the existing gauges (no series before the first real value).
- Tests first in `test/test_node.cpp` (seen failing), multi-frame with the shipped temporal and hysteresis settings: a 5 cm box with its mixed-pixel tail (mixed-pixel-scene) gives no step or ramp warning area whose centroid lies beyond the box's far edge after warm-up, and obstacle points in >= 95 % of frames; empty noisy floor drops < 1 % of its points and gives 0 lethal points; a 10 cm drop-off stays a lethal drop; the gauge appears with its first real value; every existing NodeTest stays green.
- Replay assertions in `test/test_replay.cpp` (metrics of replay-harness, frames after warm-up): box and home scenes `box_lethal_share >= 0.95` and `far_side_warning_frames == 0`; home scene `board_lethal_frames == 0`. Write them first and record the failing numbers.
- If the replay needs different D16/D17 defaults, retune them together in `include/ground_estimator_cpp/config.hpp`, `config/ground_estimator.yaml`, `ansible/group_vars/client.yml` and `Config.DefaultsMatchTheSpecification` in `test/core/test_config.cpp`, and state the replay numbers before and after in the commit message. The traversability rule keys stay (0.04 m, 15 deg).
- Notes: uses replay-harness, occlusion-gate, segment-step, mixed-pixel-scene, replay-fix-config. The gripper zone x < 0.37 is excluded by the metrics (fixed by the head floor fit in head-chain-floor-plane-fit-a). README text comes in replay-acceptance-and-docs. Deploy with `scripts/deploy-nodes.sh client ground_estimator_cpp` after merge (orchestrator).

## Acceptance conditions

- a1: the node drops occluded points before the temporal filter and publishes the ground_occluded_points gauge [evidence: symbol: nodes/ground_estimator_cpp/src/ground_node.cpp::GroundNode::process_plane]
- a2: a synthetic box with a mixed-pixel tail gives no warning beyond its far edge and stays lethal in >= 95 % of frames; empty floor and drop-off behaviour unchanged [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: on the box and home recordings the box is lethal in >= 95 % of frames with no ramp or step warning on its far side, and the board is never lethal in the home recording [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: shipped yaml and client.yml stay equal [evidence: test: tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`

## Interfaces

GroundNode keeps every topic and schema; adds the gauge `ground_occluded_points`. Uses `occluded_mask` (occlusion-gate), the segment step (segment-step), the replay metrics (replay-harness) and the mixed-pixel tail (mixed-pixel-scene).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new NodeTests and the replay assertions: box lethal >= 95 % and no far-side ramp or step warning in the box and home recordings, board never lethal in home.

## Steps

- write the NodeTests and the replay assertions; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS on them; note the failing replay numbers
- wire the gate and the gauge in ground_node.cpp; rerun; retune D16/D17 defaults only if the replay still fails, with config.hpp, yaml, client.yml and the defaults test together
- `uv run pytest tests/test_ground_estimator_deploy.py -q` PASSES
- commit `feat(ground_estimator_cpp): drop occluded points and step by segment` with the replay numbers before and after

## Ruling

- 2026-10-22 accepted: Verifier timed out on docker (no verdict). Orchestrator reran scripts/cpp-node-test.sh ground_estimator_cpp on HEAD 0dfbd483, clean tree: Summary: 356 tests, 0 errors, 0 failures, 0 skipped; PASS; replays box_lethal_share 1.000 all box scenes, board 0, far_side 0. Last fix was a stray backup-file deletion (deletion exception). Stray lethal and churn are delivered by warning-area-stability and replay-acceptance-and-docs.

## Notes

See the Requirement's notes. If the replay cannot reach a threshold with the gate and segment step, report the metric and value as the finding; do not loosen a threshold.
