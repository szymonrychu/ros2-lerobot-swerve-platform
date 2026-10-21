---
title: Ground temporal wiring
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The ground node publishes steps, drops and obstacles from denoised cell heights on the robot."
files: [nodes/ground_estimator_cpp/src/core/config.cpp, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp, nodes/ground_estimator_cpp/src/ground_node.cpp, nodes/ground_estimator_cpp/test/test_node.cpp, ansible/group_vars/client.yml, nodes/ground_estimator_cpp/README.md]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [ground-cell-temporal-median, tof-calibrated-k-deploy]
owns: []
uses: [ground-cell-median]
model: sonnet
estimate: 80
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-depth-fix-and-ground-noi--ground-temporal-wiring
branch: tof-depth-fix-and-ground-noi--ground-temporal-wiring
verified_head: ""
run: ""
provides: []
---

# Ground temporal wiring

## Requirement

Wire the temporal median into ground_estimator_cpp (design D4, D6; approval A3).

- `src/core/config.cpp` parses and validates a `temporal:` section into `Config::temporal` (unknown keys rejected, exit 78 like the rest): enabled bool; 0.02 <= cell_size_m <= 0.5; 1 <= window_frames <= 30; 1 <= min_frames <= window_frames; 0 < max_age_s <= 5. `test/core/test_config.cpp` covers the defaults and each bound (tests first).
- `config/ground_estimator.yaml` and the ground_estimator_cpp config block in `ansible/group_vars/client.yml` gain the commented `temporal:` section with the defaults of task ground-cell-temporal-median.
- `ground_node.hpp/.cpp`: the node owns one `CellMedianFilter`; `process_plane` runs `segment` (and so clusters, regions and the rule) on the filter output instead of the raw cloud; the plane fit keeps the raw cloud. The filter is reset when the plane is lost (no tracked plane). A gauge `ground_temporal_cells` reports the live cell count.
- `test/test_node.cpp` (tests first): NoisyFlatFloorGivesNoObstacleOrDropPoints (the noisy flat floor of the core test, 20 clouds: obstacle and drop clouds empty after warm-up, a plane state is published); BoxOnNoisyFloorIsStillAnObstacle (a 30 cm box on the same floor gives obstacle points after min_frames clouds). Existing node tests keep every assertion: static scenes are fed `temporal.min_frames` times at 0.1 s spacing, or the test config disables temporal where the test is about something else.
- The traversability rule and its config (wheel_radius_m 0.06, max_step_fraction 0.3333, max_slope_deg 8) stay unchanged; existing rule tests on both sides of the thresholds still pass.
- `nodes/ground_estimator_cpp/README.md` documents the temporal section, the warm-up delay (min_frames / 10 Hz) and the new gauge.

## Acceptance conditions

- a1: the temporal section is parsed with its bounds and unknown keys rejected (test_config) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: a noisy flat floor gives empty obstacle and drop clouds after warm-up and a 30 cm box on it is still an obstacle (test_node NoisyFlatFloorGivesNoObstacleOrDropPoints, BoxOnNoisyFloorIsStillAnObstacle) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the shipped defaults, the client.yml block and the README agree, and the root ground tests and ansible checks pass [evidence: command:uv run pytest tests/test_ground_estimator_deploy.py tests/test_ground_estimator_readme.py tests/test_ground_docs.py -q && uv run poe lint && uv run poe test-ansible]
- a4: segmentation runs on the filter output in process_plane [evidence: symbol:nodes/ground_estimator_cpp/src/ground_node.cpp::process_plane]

## Files

- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

Uses `ground-cell-median` (`CellMedianFilter`, `TemporalConfig`). Adds the `temporal:` YAML section and the gauge `ground_temporal_cells`. Keep every topic, the state JSON, the rule config and `segment()`.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new config and node tests; the root ground tests and ansible checks pass.

## Steps

- Write the test_config.cpp cases and the two test_node.cpp tests; run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL.
- Parse `temporal:` in config.cpp, wire the filter into GroundNode::process_plane, reset it when no plane is tracked, add the gauge; adapt the existing node tests (feed min_frames times or disable temporal) without weakening assertions; rerun, see PASS.
- Add the section to config/ground_estimator.yaml and the client.yml block; update README.md; run the root ground tests, `uv run poe lint`, `uv run poe test-ansible`.
- Commit `feat(ground_estimator_cpp): classify on per-cell temporal medians`.

## Notes

tests/test_ground_estimator_readme.py may pin that the README names every config key; tests/test_ground_estimator_deploy.py may compare the client.yml block with the defaults file.
