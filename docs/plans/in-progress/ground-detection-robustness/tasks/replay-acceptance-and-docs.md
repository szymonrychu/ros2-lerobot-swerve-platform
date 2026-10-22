---
title: Replay acceptance and docs
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The live-round acceptance holds on real data in every build, and the changes are documented for the next developer."
files: [nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/test/test_node.cpp, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [warning-area-stability]
owns: []
uses: [replay-harness, area-stability]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--replay-acceptance-and-docs
branch: ground-detection-robustness--replay-acceptance-and-docs
verified_head: ""
run: ""
provides: []
---

# Replay acceptance and docs

## Requirement

Lock in the live-round acceptance on the real recordings and document the live-round changes (design D14-D19; brief D1, D2; approved items A10-A14).

- `test/test_replay.cpp` (replay-harness metrics, frames after warm-up, gripper zone x < 0.37 excluded), written as assertions:
  - box and home scenes: `box_lethal_share >= 0.95`, `far_side_warning_frames == 0`;
  - home and ramp scenes: `board_lethal_frames == 0` (the board edge is in the ramp view);
  - ramp scene: `ramp_lethal_frames == 0` (D19: no returns on the black rubber, never lethal);
  - every scene: `stray_lethal_share <= 0.05` (no lethal point within 1.2 m outside the box at p95 of frames, as the approved empty-floor goal) and `churn_share < 0.10`.
- `test/test_node.cpp`: a latency pin: a 5 cm box placed on floor the node has tracked for 20 frames gives obstacle points no later than `hysteresis.enter_frames` frames (0.3 s at 10 Hz) after the first frame in which the temporal filter outputs box heights for its cells, with the shipped config (area stability does not delay lethal output).
- `nodes/ground_estimator_cpp/README.md`: new sections `## Occlusion gate` (D16, keys, gauge), `## Segment step` (D17), `## Warning area stability` (D18, the diagnosis breakdown from warning-area-stability's commit, the keys), the config table rows for every key added in this round, and `## Replay tests` updated with the final per-scene metric lines and the assertions above.
- Notes: if an assertion fails, the failing metric and its value are reported as the finding (no threshold is loosened). Uses replay-harness and area-stability.

## Acceptance conditions

- a1: the replay asserts box lethal >= 95 %, no far-side warning, board and ramp never lethal, stray lethal <= 5 % of frames and churn < 10 % in the scenes listed [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: a NodeTest pins the lethal latency of a new 5 cm box at hysteresis.enter_frames frames after its cells are output [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the README documents the occlusion gate [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Occlusion gate]
- a4: the README documents the segment step [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Segment step]
- a5: the README documents warning area stability [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Warning area stability]
- a6: the README replay section carries the final per-scene metrics [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]

## Files

- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

No new interfaces: assertions over the replay-harness metrics, one latency NodeTest and README sections. Uses area-stability.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with every replay assertion of the Requirement and the latency pin; the README carries the four sections with the final per-scene numbers.

## Steps

- write the replay assertions and the latency NodeTest; `scripts/cpp-node-test.sh ground_estimator_cpp` PASSES (if one fails, report the metric and its value as the finding)
- write the README sections and the config table rows; commit `test(ground_estimator_cpp): live-round replay acceptance`

## Notes

The thresholds are the approved ones (A10-A14); none may be loosened. Board never lethal is strict (0 frames); stray lethal follows the approved empty-floor p95 goal (<= 5 % of frames).
