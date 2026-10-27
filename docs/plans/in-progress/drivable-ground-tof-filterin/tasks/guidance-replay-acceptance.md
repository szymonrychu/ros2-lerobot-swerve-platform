---
title: Guidance replay acceptance
status: done
repo: ros2-lerobot-sverve-platform
value: "The approved edge, no-return and safety numbers locked in on real recordings"
files: [nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, nodes/ground_estimator_cpp/src/core/edge_points.cpp, nodes/ground_estimator_cpp/src/core/no_return.cpp, nodes/ground_estimator_cpp/test/core/test_edge_points.cpp, nodes/ground_estimator_cpp/test/core/test_no_return.cpp, nodes/ground_estimator_cpp/README.md]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible"
depends_on: [guidance-node-wiring, capture-gated-recordings, ground-estimator-live-scene]
owns: []
uses: [guided-node, guidance-replay-harness, gated-fixtures]
model: opus
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--guidance-replay-acceptance
branch: drivable-ground-tof-filterin--guidance-replay-acceptance
verified_head: 1f35ad68592fa438d2e1cd10f76606bad24846ac
run: ""
provides: []
points_actual: 1
---

# Guidance replay acceptance

## Requirement

Lock in the guided acceptance on the real gated recordings (design D1, D3, D5, D15; goals G3, G4, G5). It uses parts guided-node, guidance-replay-harness and gated-fixtures.

- `test/test_replay.cpp` `ReplayGuided.<Scene>` tests replay each gated fixture with `Guidance::on` and with `Guidance::off`, print both lines, and assert. Frames are those after warm-up, with x < 0.37 excluded.
  - home and box: `box_front_edge_error_m <= 0.02`; `box_width_error_m <= 0.02`; both strictly smaller than with guidance off; `outside_lethal_frames_share <= 0.05`; `box_lethal_share(frames, truth) >= 0.95`.
  - home and ramp (the board is in view): `board_lethal_frames(frames, truth) == 0`. The textured board never gets a lethal point.
  - ramp: `ramp_lethal_frames(frames, truth) == 0` and `no_return_cover_share >= 0.90`.
  - every scene: `no_return_stray_share <= 0.05` (tolerance 0.10 m, one cell, my call); `stray_lethal_share(frames, truth) <= 0.05`; `churn_share < 0.10` (run 4's metric, no ROI) with the no_return areas included.
  - empty: `empty_lethal_share <= 0.05`, and no no_return area in more than 5 % of frames.
  - rule unchanged (D15): the loaded shipped config has `traversability.max_step_fraction == 0.3333` and `max_slope_deg == 15`.
- Run 4's replay acceptance tests on its own fixtures keep passing unchanged, and so do the `ReplayLiveScene.<Scene>` tests of ground-estimator-live-scene (guidance off; design D23): the guidance never undoes the live-scene numbers.
- Metric names: the truth overloads and the new metrics are defined in guidance-replay-metrics; `churn_share` and `plane_share` come unchanged from run 4 (ground-detection-robustness/replay-fixtures-and-test).
- Tuning allowed only through the guidance keys within their validated ranges (`config/ground_estimator.yaml`, mirrored in `ansible/group_vars/client.yml`), and through fixes in `src/core/edge_points.cpp` or `src/core/no_return.cpp` with their unit tests still green. A threshold above is never loosened. When a metric cannot be met, the failing metric and its value are the finding.
- README `nodes/ground_estimator_cpp/README.md` `## RGB and amplitude guidance`: a before/after table per scene (baseline, gated, guided) for every metric above, and the final key values.

## Acceptance conditions

- a1: the guided replays meet the edge, board, ramp no-return, stray, churn and empty-floor thresholds; guided edges beat guidance off; the rule is unchanged; run 4's replay tests still pass [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the shipped yaml and client.yml carry the same keys after tuning [evidence: test: tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a3: the README carries the before/after table [evidence: doc-section: nodes/ground_estimator_cpp/README.md#RGB and amplitude guidance]
- a4: no assertion threshold of the Requirement was loosened [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/src/core/edge_points.cpp`
- `nodes/ground_estimator_cpp/src/core/no_return.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edge_points.cpp`
- `nodes/ground_estimator_cpp/test/core/test_no_return.cpp`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

No new interfaces: replay assertions over part guidance-replay-harness on part gated-fixtures, run through part guided-node. Key values only change inside their validated ranges.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with every `ReplayGuided.*` assertion and run 4's replay tests; the README carries the baseline / gated / guided table.

## Steps

- write the `ReplayGuided.*` assertions; `scripts/cpp-node-test.sh ground_estimator_cpp` shows which metrics fail with the default keys
- tune the keys or fix edge_points/no_return with their unit tests green; the command PASSES
- mirror tuned values in client.yml; `uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible` PASS
- README table; commit `test(ground_estimator_cpp): guided replay acceptance`

## Notes

- If a metric cannot be met, report the metric and its value as the finding; never loosen a threshold.
- The gripper zone x < 0.37 stays excluded (the self-filter margin, not this epic).
- Deploy with `scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones web_ui` (orchestrator; mcp_server separately).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-22 amended: planner: umbrella re-plan (approval G1, D23): the live-scene replay tests of ground-estimator-live-scene keep passing with the guidance on

- 2026-10-27 done: superseded by realsense-d435-head-switchov/tune-ground-estimator-on-d43

- 2026-10-27 actuals: no agent records
