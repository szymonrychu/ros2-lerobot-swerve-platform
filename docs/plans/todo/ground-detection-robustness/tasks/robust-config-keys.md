---
title: Robust config keys
status: todo
repo: ros2-lerobot-sverve-platform
value: "Every new robustness setting is validated and deployed in one place, and the ramp limit can rise to the tilt_guard error limits."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/src/core/config.cpp, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, tests/test_ground_estimator_deploy.py, tests/README.md]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py tests/test_head_remount_runbook.py tests/test_nav_ground_voxel_layer.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: []
owns: [robust-config]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Robust config keys

## Requirement

Add every new ground_estimator_cpp setting this epic needs, with validation, and mirror it into the deployed config (design D3, D5, D7, D11, D13, D2 bound). Behaviour of the node does not change in this task: the keys are read and validated, their consumers land in later tasks.

- `include/ground_estimator_cpp/config.hpp`, `src/core/config.cpp`: new keys, each with its default, read with the existing `read<>` helpers and rejected by name when out of range (extra keys stay forbidden):
  - `temporal.max_spread_m` (double, default 0.025, 0 < v <= 0.2): largest robust spread of a cell's per-frame medians before the cell is unknown (consumer cell-spread-gate).
  - new section `hysteresis` -> `struct HysteresisConfig { bool enabled = true; int window_frames = 5; int enter_frames = 3; int exit_frames = 1; }` in `Config::hysteresis`; valid when 1 <= exit_frames <= enter_frames <= window_frames <= 30 and enter_frames * 2 > window_frames (a class needs a strict majority to enter), errors named `hysteresis.<key>` (consumer cell-class-hysteresis).
  - `traversability.lethal_min_points` (int, default 2, 1..50): fewest unclustered lethal points within `edges.cluster_tolerance_m` of each other for any of them to be lethal (consumer lethal-point-gates).
  - `edges.slope_bin_m` (double, default 0.10, 0.02 <= v <= 0.5): edge of the xy bins a cluster is sampled in for its slope fit, equal to `temporal.cell_size_m` by default (consumer edge-step-and-ramp-slope).
  - `edges.slope_min_cells` (int, default 3, 2..50): fewest slope bins a cluster must cover for a slope estimate; below it the slope is unknown (consumer edge-step-and-ramp-slope).
- Validation change (D2): `traversability.max_slope_deg` must be `<= min(slope.roll_error_deg, slope.pitch_error_deg)` (was `<= min(roll_warn_deg, pitch_warn_deg)`); the error text names the error limits. The default value 8.0 is NOT changed here (task slope-and-smoothing-defaults changes it with its test pins).
- `config/ground_estimator.yaml`: every new key with a comment giving meaning, unit and range, in the style of the file.
- `ansible/group_vars/client.yml` ground_estimator_cpp `config:` block: the same new keys and comments, so it stays key-for-key and value-for-value equal to the yaml (`tests/test_ground_estimator_deploy.py::test_ros2_nodes_entry` and `test_config_keys_equal_the_shipped_yaml_keys`).
- `tests/test_ground_estimator_deploy.py::test_limits_match_tilt_guard_and_swerve`: assert `max_slope_deg <= min(tilt roll_error_deg, pitch_error_deg)` instead of the warn limits; add `test_robustness_keys_present` asserting the new settings exist in the client config with the defaults above.
- Tests first in `test/core/test_config.cpp`: defaults of every new key; both sides of every new range (e.g. `temporal: {max_spread_m: 0}` rejected, `0.2` accepted, `0.21` rejected; `hysteresis: {window_frames: 5, enter_frames: 3}` accepted, `{window_frames: 6, enter_frames: 3}` rejected (no strict majority), `exit_frames: 4` with enter 3 rejected, `window_frames: 31` rejected; `traversability: {lethal_min_points: 0}` rejected; `edges: {slope_min_cells: 1}` and `edges: {slope_bin_m: 0.01}` rejected); `traversability: {max_slope_deg: 15}` accepted with the default slope section, `15.5` rejected, `max_slope_deg: 12` with `slope: {roll_error_deg: 11, roll_warn_deg: 8}` rejected naming `traversability.max_slope_deg`; unknown key `hysteresis: {foo: 1}` rejected.
- `tests/README.md`: the changed and added deploy tests documented.

Interfaces: `HysteresisConfig`, `Config::hysteresis`, `TemporalConfig::max_spread_m`, `TraversabilityConfig::lethal_min_points`, `EdgesConfig::slope_bin_m`, `EdgesConfig::slope_min_cells` in config.hpp (part robust-config).

Notes: keep the existing comment `0 < v <= min(slope.roll_warn_deg, slope.pitch_warn_deg)` on max_slope_deg in config.hpp, the yaml and client.yml updated to the error limits. client.yml is also edited by head-chain-floor-plane-fit-a (epic blocker): rebase onto it before starting.

## Acceptance conditions

- a1: every new key has its default, is range-checked on both sides and rejected by its dotted name; unknown hysteresis keys are rejected [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: max_slope_deg is validated against min(slope.roll_error_deg, slope.pitch_error_deg): 15 accepted with the defaults, 15.5 rejected [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: client.yml ground config equals config/ground_estimator.yaml key for key and value for value [evidence: test: tests/test_ground_estimator_deploy.py::test_ros2_nodes_entry]
- a4: the deploy test pins max_slope_deg to the tilt_guard error limits [evidence: test: tests/test_ground_estimator_deploy.py::test_limits_match_tilt_guard_and_swerve]
- a5: the new settings are present in the deployed config with their defaults [evidence: test: tests/test_ground_estimator_deploy.py::test_robustness_keys_present]
- a6: the HysteresisConfig struct exists in config.hpp [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp::HysteresisConfig]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `tests/test_ground_estimator_deploy.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
