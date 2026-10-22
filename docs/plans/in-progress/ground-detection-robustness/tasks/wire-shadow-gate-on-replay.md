---
title: Wire shadow gate on replay
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
