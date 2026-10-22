---
title: Guidance config keys
status: done
repo: ros2-lerobot-sverve-platform
value: "One validated, documented place for every guidance setting, mirrored in Ansible"
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/src/core/config.cpp, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, tests/test_ground_estimator_deploy.py, tests/README.md]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible"
depends_on: [rgbd-upsampling-on]
owns: [guidance-config]
uses: [upsampled-depth-on]
model: ""
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--guidance-config-keys
branch: drivable-ground-tof-filterin--guidance-config-keys
verified_head: 432cb1e569098c2a7ff9d1bc073606bcf6d88f71
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: []
---

# Guidance config keys

## Requirement

The `guidance` config block of ground_estimator_cpp (part guidance-config, design D3, D5, D12), validated like every other block (unknown keys rejected, named errors, exit 78).

- `include/ground_estimator_cpp/config.hpp`: `GuidanceConfig`, `EdgeGuidanceConfig` and `NoReturnConfig`, with meaning, unit and range on every field. Defaults are below (my call; guidance-replay-acceptance may retune values inside the ranges).

  `guidance` fields:
  - `enabled: true`;
  - `depth_topic: /head_rgbd/depth_upsampled/image_raw`, `info_topic: /head_rgbd/depth_upsampled/camera_info`, `rgb_topic: /head_rgbd/rgb/image_raw/compressed`, `tof_info_topic: /tof/camera_info`;
  - `max_age_s: 0.6` (0 < v <= 2): the oldest guidance frame still used, relative to the cloud stamp;
  - `sample_step_px: 2` (1..8): pixel stride of the sampling grid;
  - `max_range_m: 2.0` (0 < v <= bands.max_range_m).

  `guidance.edge` fields:
  - `enabled: true`;
  - `max_shift_m: 0.06` (0 < v <= 0.15): largest xy distance of an edge point from a stable lethal point of the frame;
  - `min_height_m: 0.02` (0 < v <= the max step 0.04): smallest |height| of an edge point;
  - `voxel_m: 0.02` (0.005..0.05): voxel of the edge point decimation.

  `guidance.no_return` fields:
  - `enabled: true`;
  - `min_range_m: 0.3` (0 <= v < guidance.max_range_m);
  - `cell_min_samples: 6` (>= 1) and `cell_min_share: 0.8` (0 < v <= 1);
  - `window_frames: 4` (1..20) and `enter_frames: 3` (1..window_frames);
  - `min_cells: 2` (>= 1), `clearance_cells: 1` (0..3), `tof_margin_px: 4` (0..20).
- `src/core/config.cpp`: parsing and validation with named errors (`guidance.edge.max_shift_m: ...`). Cross-key checks: `max_range_m <= bands.max_range_m`, `edge.min_height_m <= traversability max step`, `no_return.enter_frames <= no_return.window_frames`, `no_return.min_range_m < max_range_m`.
- `config/ground_estimator.yaml` documents every key (meaning, unit, range); `ansible/group_vars/client.yml` mirrors the block in the ground_estimator_cpp config.
- `tests/test_ground_estimator_deploy.py`: `test_config_keys_equal_the_shipped_yaml_keys` covers the block, and `test_topics_match_their_producers` maps the four guidance topics to their producers (head_rgbd_cpp, arducam_tof).
- Tests first (seen failing) in `test/core/test_config.cpp`: defaults; both sides of every range; every cross-key check; unknown key in each sub-block rejected.
- No behaviour change in the node yet (guidance-node-wiring wires it).

## Acceptance conditions

- a1: every guidance key is validated on both sides of its range, including the cross-key checks, and unknown keys are rejected [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the shipped yaml and client.yml carry the same keys [evidence: test: tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a3: the guidance topics map to their producers [evidence: test: tests/test_ground_estimator_deploy.py::test_topics_match_their_producers]
- a4: Ansible checks pass [evidence: command: uv run poe test-ansible]
- a5: GuidanceConfig exists with its sub-blocks [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp::GuidanceConfig]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_ground_estimator_deploy.py`
- `tests/README.md`

## Interfaces

Builds part guidance-config: `GuidanceConfig`, `EdgeGuidanceConfig`, `NoReturnConfig` in `config.hpp` and `Config::guidance`; `validate`/`from_yaml` handle the block.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new config tests; `uv run pytest tests/test_ground_estimator_deploy.py -q` PASS; yaml and client.yml carry the block.

## Steps

- write the config tests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS
- implement parsing and validation; the command PASSES
- document the yaml, mirror client.yml, update the deploy test; `uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible` PASS
- commit `feat(ground_estimator_cpp): guidance config keys`

## Notes

Follow the existing block style in `src/core/config.cpp` (named errors per key, cross-key checks after the block). Ground-detection-robustness added `hysteresis.*` keys the same way; keep their order and comments.
