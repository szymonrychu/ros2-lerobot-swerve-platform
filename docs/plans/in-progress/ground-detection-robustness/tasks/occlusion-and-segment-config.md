---
title: Occlusion and segment config
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "One validated, deployed definition of the new occlusion and segment-step settings for the two fixes that use them."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/src/core/config.cpp, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q"
depends_on: [ground-robustness-runbook]
owns: [replay-fix-config]
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--occlusion-and-segment-config
branch: ground-detection-robustness--occlusion-and-segment-config
verified_head: ""
run: ""
provides: []
---

# Occlusion and segment config

## Requirement

Add the config keys of the occlusion shadow gate (design D16) and the boundary-segment step (D17) to ground_estimator_cpp, with validation, shipped defaults and the client.yml mirror, so shadow-gate-and-segment-step and wire-shadow-gate-on-replay build on one definition (part replay-fix-config). No behaviour change in this task: the keys are read by later tasks.

- `include/ground_estimator_cpp/config.hpp`: a new `OcclusionConfig occlusion` member of `Config` with `bool enabled = true`, `double azimuth_bin_deg = 1.0` (0.1..10), `double elevation_margin_deg = 0.5` (0..5), `double min_occluder_height_m = 0.02` (> 0, < bands.obstacle_max_m), `double min_height_drop_m = 0.01` (> 0); `EdgesConfig` gains `double step_segment_m = 0.10` (0.02..0.5) and `int step_segment_min_points = 3` (1..50). Each field commented with meaning, unit and range like the existing ones.
- `src/core/config.cpp`: parse and validate them (`validate` names every bad key as `occlusion.<key>: <problem>` / `edges.<key>: ...`, unknown keys under `occlusion` rejected like every other block); `from_yaml` reads them.
- `config/ground_estimator.yaml`: an `occlusion:` block and the two edges keys with the defaults above, each line documented (meaning, unit, range), so `Config.ShippedYamlDocumentsEveryKey` and `Config.ShippedYamlLoadsWithTheDefaults` hold.
- `ansible/group_vars/client.yml`: the ground_estimator_cpp config block mirrors the yaml exactly (`tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys`).
- Tests first in `test/core/test_config.cpp` (seen failing): `Config.DefaultsMatchTheSpecification` extended with the new defaults; new `Config.RejectsOcclusionRanges` (each bound on both sides, min_occluder_height_m >= bands.obstacle_max_m rejected) and `Config.RejectsSegmentStepRanges`; `Config.RejectsUnknownKeysAtEveryLevel` covers the occlusion block.
- Notes: semantics of the keys are those of D16 and D17 in design.md; the README config table is written by replay-acceptance-and-docs. The traversability rule keys do not change (brief D3).

## Acceptance conditions

- a1: the occlusion and segment-step keys have the specified defaults, are validated with named errors and reject unknown keys [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the shipped yaml documents and loads every new key, and client.yml mirrors it [evidence: test: tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a3: OcclusionConfig and the two EdgesConfig fields exist in config.hpp [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp::OcclusionConfig]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`

## Interfaces

Builds part replay-fix-config: `Config::occlusion` (`OcclusionConfig`), `EdgesConfig::step_segment_m`, `EdgesConfig::step_segment_min_points` in `include/ground_estimator_cpp/config.hpp`, read by `validate` and `from_yaml`. Existing keys and the traversability rule are unchanged.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the extended config tests; `uv run pytest tests/test_ground_estimator_deploy.py -q` PASS; the shipped yaml and client.yml carry the new keys with documented ranges.

## Steps

- extend `test/core/test_config.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS on the new tests
- add the fields, parsing and validation, the yaml block; the command PASSES
- mirror the yaml in client.yml; `uv run pytest tests/test_ground_estimator_deploy.py -q` PASSES
- commit `feat(ground_estimator_cpp): occlusion and segment step config keys`

## Notes

Design D16 and D17 give the semantics. This is the config foundation of the live round (like D13); it lands first so the two core fixes build on one definition.
