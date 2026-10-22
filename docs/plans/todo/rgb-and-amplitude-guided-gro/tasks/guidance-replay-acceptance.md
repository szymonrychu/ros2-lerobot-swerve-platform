---
title: Guidance replay acceptance
status: todo
repo: ros2-lerobot-sverve-platform
value: "The approved edge, no-return and safety numbers locked in on real recordings"
files: [nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, nodes/ground_estimator_cpp/src/core/edge_points.cpp, nodes/ground_estimator_cpp/src/core/no_return.cpp, nodes/ground_estimator_cpp/test/core/test_edge_points.cpp, nodes/ground_estimator_cpp/test/core/test_no_return.cpp, nodes/ground_estimator_cpp/README.md]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible"
depends_on: [guidance-node-wiring, capture-gated-recordings]
owns: []
uses: [guided-node, guidance-replay-harness, gated-fixtures]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Guidance replay acceptance

## Requirement

Lock in the guided acceptance on the real gated recordings (design D1, D3, D5, D15; goals G3, G4, G5). It uses parts guided-node, guidance-replay-harness and gated-fixtures.

- `test/test_replay.cpp` `ReplayGuided.<Scene>` tests replay each gated fixture with `Guidance::on` and with `Guidance::off`, print both lines, and assert. Frames are those after warm-up, with x < 0.37 excluded.
  - home and box: `box_front_edge_error_m <= 0.02`; `box_width_error_m <= 0.02`; both strictly smaller than with guidance off; `outside_lethal_frames_share <= 0.05`; `box_lethal_share >= 0.95`.
  - home and ramp (the board is in view): `board_lethal_frames == 0` and `board_lethal_points_frames == 0`. The textured board never gets a lethal point.
  - ramp: `ramp_lethal_frames == 0` and `no_return_cover_share >= 0.90`.
  - every scene: `no_return_stray_share <= 0.05`; `stray_lethal_share <= 0.05`; `churn_share < 0.10` with the no_return areas included.
  - empty: `empty_lethal_share <= 0.05`, and no no_return area in more than 5 % of frames.
  - rule unchanged (D15): the loaded shipped config has `traversability.max_step_fraction == 0.3333` and `max_slope_deg == 15`.
- Run 4's replay acceptance tests on its own fixtures keep passing unchanged.
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
