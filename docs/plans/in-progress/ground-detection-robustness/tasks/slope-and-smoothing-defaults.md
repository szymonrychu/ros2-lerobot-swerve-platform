---
title: Slope and smoothing defaults
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The user's 14 deg ramp is rideable (warning, slow-down) and the floor plane stops jittering with single fits."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/test/core/test_state.cpp, nodes/ground_estimator_cpp/test/core/test_traversability.cpp, nodes/ground_estimator_cpp/test/test_node.cpp]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible"
depends_on: [robust-config-keys, cell-spread-gate, lethal-point-gates]
owns: [ground-traversability-rule]
uses: [robust-config, spread-gate, lethal-gates]
model: ""
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--slope-and-smoothing-defaults
branch: ground-detection-robustness--slope-and-smoothing-defaults
verified_head: ""
run: ""
provides: []
---

# Slope and smoothing defaults

## Requirement

Change the two existing defaults this epic decides: the ramp rule `traversability.max_slope_deg` 8 -> 15 (design D2, user ground truth: the ~14 deg ramp is a safe skew) and the plane smoothing `fit.smoothing` 0.5 -> 0.2 (design D10, brief fix 6, after the spread gate), with every pin that encodes the old values.

- `include/ground_estimator_cpp/config.hpp` struct defaults, `config/ground_estimator.yaml` and the ground_estimator_cpp block of `ansible/group_vars/client.yml`: `max_slope_deg: 15.0` (comment: steepest rideable surface, degrees, <= slope.roll_error_deg and slope.pitch_error_deg) and `smoothing: 0.2`; yaml and client.yml stay equal (`tests/test_ground_estimator_deploy.py`).
- Pins, updated test-first (see each fail against the old defaults first):
  - `test/core/test_config.cpp`: defaults 15.0 and 0.2;
  - `test/core/test_state.cpp` `TraversabilityIsTheConfiguredRuleEvenWithoutPlane`: max_slope_deg 15.0;
  - `test/core/test_traversability.cpp`: `classify_cluster` at exactly 15.0 deg -> warning, 15.01 -> obstacle by `slope`; a dense 14 deg ramp scene -> warning kind `ramp` with no lethal points although it rises above 0.04 m; a dense 17 deg ramp -> obstacle by `slope` (replacing the 6 / 10 deg pair); the step-height pins unchanged;
  - `test/test_node.cpp`: any test that relied on the 8 deg limit or the 0.5 smoothing (plane convergence and hold timing) updated to the new defaults with the same intent.
- `tests/test_ground_estimator_deploy.py` stays green unchanged (its bound moved in robust-config-keys).

Interfaces: none new; the `ground-traversability-rule` contract value (max_slope_deg 15) is defined here in config/ground_estimator.yaml.

Notes: `nodes/ground_estimator_cpp/test/data/state_example.json` is a sample document read by mcp_server and web_ui tests (its 8.0 is example data, not a default); leave it. The tilt_guard error limits are roll 15 / pitch 20 deg (client.yml swerve_controller tilt_guard ~1803).

## Acceptance conditions

- a1: the defaults are max_slope_deg 15 and smoothing 0.2 in config.hpp, the shipped yaml and client.yml [evidence: test: tests/test_ground_estimator_deploy.py::test_ros2_nodes_entry]
- a2: exactly 15 deg is a warning, 15.01 deg an obstacle; a 14 deg ramp is a ramp warning without lethal points and a 17 deg ramp an obstacle [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the state reports max_slope_deg 15 [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_state.cpp`
- `nodes/ground_estimator_cpp/test/core/test_traversability.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`

## Interfaces

Defaults only: traversability.max_slope_deg 15.0 and fit.smoothing 0.2 in config.hpp, config/ground_estimator.yaml and client.yml (contract ground-traversability-rule).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS and `uv run pytest tests/test_ground_estimator_deploy.py -q` PASS with the new defaults; a 14 deg ramp is a warning, 17 deg an obstacle.

## Steps

- update the pins in test_config.cpp, test_state.cpp, test_traversability.cpp (15 / 15.01, 14 / 17 deg) and test_node.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL against the old defaults
- change the defaults in config.hpp, the yaml and client.yml
- run the verify command, see PASS; commit `feat(ground_estimator_cpp): 15 deg ramp rule and plane smoothing 0.2`

## Notes

See the Notes paragraph of the Requirement; design: docs/plans/todo/ground-detection-robustness/design.md.
