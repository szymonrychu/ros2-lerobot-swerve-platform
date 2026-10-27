---
title: ToF amplitude gate
status: done
repo: ros2-lerobot-sverve-platform
value: "Dark-surface depth bias removed at the source, so the board stops reading as a step"
files: [nodes/bridges/arducam_tof/arducam_tof/amplitude.py, nodes/bridges/arducam_tof/arducam_tof/bridge.py, nodes/bridges/arducam_tof/arducam_tof/config.py, nodes/bridges/arducam_tof/tests/test_amplitude.py, nodes/bridges/arducam_tof/tests/test_bridge.py, nodes/bridges/arducam_tof/README.md, ansible/group_vars/client.yml, tests/test_arducam_tof_config.py]
verify: "(cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint) && uv run pytest tests/test_arducam_tof_config.py -q && uv run poe test-ansible"
depends_on: [capture-baseline-recordings]
owns: [amplitude-gate]
uses: [baseline-fixtures]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--tof-amplitude-gate
branch: drivable-ground-tof-filterin--tof-amplitude-gate
verified_head: 7dece440b9162fb337d1706af8a2a5f44f5a93d8
run: ""
provides: []
points_actual: 1
---

# ToF amplitude gate

## Requirement

Amplitude confidence for the ToF depth in arducam_tof (part amplitude-gate, design D7, cross-epic contract tof-depth-amplitude-gate). It uses part baseline-fixtures: `tests/data/amplitude_frames.npz`, written by capture-baseline-recordings through `export_tof_frames` (replay-format-v2), with keys `tof_depth_mm`, `tof_amplitude`, `tof_base_T`, `tof_K`, `tof_D`, `tof_size`, `scene`, `truth_board`, `truth_ramp`, `truth_empty` (xy corners, base_link m) and `floor_plane` (nx ny nz d, base_link).

- `arducam_tof/amplitude.py`, a ROS-free and SDK-free numpy module, loadable by path from root scripts:
  - `normalised_amplitude(amplitude: np.ndarray, depth_mm: np.ndarray) -> np.ndarray`, which is amplitude * (z / 1000 mm)^2;
  - `amplitude_gate(depth_mm: np.ndarray, amplitude: np.ndarray, min_normalised: float) -> tuple[np.ndarray, int]`: the gated uint16 depth (a valid pixel below the threshold becomes 0, REP 118, never interpolated) and the number of pixels removed. 0 pixels stay 0. It is vectorised: no per-pixel Python loop.
- `arducam_tof/config.py` `TofConfig`: `amplitude_gate: bool = True` and `amplitude_min_normalised: float` (>= 0, upper bound documented), with meaning and unit in the docstring. The default is chosen from the real-data test below and stated in the README.
- `arducam_tof/bridge.py` `publish_frame`: applies the gate after run 4's `flying_pixel_filter` when `amplitude_gate` is true, and counts the removed pixels in `tof_amplitude_gated_pixels_total`. The amplitude image is published unchanged.
- First step, recorded in the commit message: on `amplitude_frames.npz`, measure the board pixels' height above the plane against the normalised amplitude. If the board bias is not explained by amplitude (the gate cannot reach G2 at >= 95 % floor retention), add an amplitude-dependent depth correction fitted on the board pixels (`amplitude_correction`, a monotonic lookup in the same module, its own config keys, its own tests) within the same acceptance.
- Tests first (seen failing):
  - `tests/test_amplitude.py` real-data tests on `amplitude_frames.npz`:
    - `test_gate_brings_board_heights_within_2_cm`: after the gate, the p95 |height| of the board pixels is <= 0.02 m. The pixel heights come from the recorded tof_base_T and the plane, and board pixels are those whose floor point lies inside the board polygon;
    - `test_gate_keeps_floor_pixels`: >= 95 % of the empty-floor pixels between 0.3 and 2.0 m are kept;
    - `test_gate_on_ramp_keeps_no_return`: the ramp footprint keeps no valid pixel that the gate invented. The gate only removes;
  - `tests/test_amplitude.py` synthetic tests: `test_gate_zeroes_low_normalised_amplitude`, `test_invalid_pixels_stay_invalid`, `test_normalisation_by_range_squared`;
  - `tests/test_bridge.py`: `test_published_depth_is_gated_and_counted`, and `amplitude_gate: false` publishes the flying-pixel-filtered depth unchanged;
  - `tests/test_amplitude.py`: `test_amplitude_keys_validated` (TofConfig defaults, ranges, unknown key still rejected).
- Ansible: the arducam_tof `config` in `ansible/group_vars/client.yml` sets both keys with comments. `tests/test_arducam_tof_config.py` `DRIVER_KEYS` gains them.
- README `nodes/bridges/arducam_tof/README.md`: the gate, its keys and counter, why it exists (dark-surface bias, board -4..+3 cm), and the measured before/after numbers.

## Acceptance conditions

- a1: on recorded frames the board pixels read within 2 cm (p95) after the gate [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_brings_board_heights_within_2_cm]
- a2: on recorded frames >= 95 % of the floor pixels between 0.3 and 2.0 m are kept [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_keeps_floor_pixels]
- a3: the gate never adds a valid pixel on the ramp [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_on_ramp_keeps_no_return]
- a4: low normalised amplitude is zeroed, invalid pixels stay invalid, normalisation is by range squared [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_gate_zeroes_low_normalised_amplitude]
- a5: the published depth is gated and counted, and unchanged with the gate off [evidence: test: nodes/bridges/arducam_tof/tests/test_bridge.py::test_published_depth_is_gated_and_counted]
- a6: the keys are validated [evidence: test: nodes/bridges/arducam_tof/tests/test_amplitude.py::test_amplitude_keys_validated]
- a7: client.yml sets the keys and the deployed config loads in the real driver [evidence: test: tests/test_arducam_tof_config.py::test_deployed_config_holds_only_keys_the_driver_accepts]
- a8: node tests and lint pass [evidence: command: cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint]

## Files

- `nodes/bridges/arducam_tof/arducam_tof/amplitude.py`
- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_amplitude.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_arducam_tof_config.py`

## Interfaces

Builds part amplitude-gate:
- `arducam_tof/amplitude.py`: `normalised_amplitude`, `amplitude_gate`, and `amplitude_correction` only if the data needs it;
- `TofConfig.amplitude_gate`, `TofConfig.amplitude_min_normalised`;
- counter `tof_amplitude_gated_pixels_total`.

Topics, encodings and the z-depth meaning of /tof/depth/image_raw are unchanged (contract tof-depth-z). New cross-epic contract tof-depth-amplitude-gate, applied after tof-depth-flying-pixel-filter.

## Expected outcome

`cd nodes/bridges/arducam_tof && uv run pytest tests -q` PASS with the new tests; `uv run poe lint` clean; `uv run pytest tests/test_arducam_tof_config.py -q` PASS; client.yml sets both keys; the README states the before/after board and floor numbers.

## Steps

- load `tests/data/amplitude_frames.npz` and measure the board heights against the normalised amplitude; note the result for the commit message
- write the tests; `cd nodes/bridges/arducam_tof && uv run pytest tests -q` FAILS on them
- implement the module, the config keys, the publish_frame call and the counter; the same command PASSES; `uv run poe lint` PASSES
- set client.yml and DRIVER_KEYS; `uv run pytest tests/test_arducam_tof_config.py -q && uv run poe test-ansible` PASS
- update the README; commit `feat(arducam_tof): amplitude gate for low-signal depth`

## Notes

- `bridge.py` imports the Arducam SDK at the top. Keep `amplitude.py` free of it and of rclpy, so root scripts can load it with `importlib.util.spec_from_file_location`, as `tests/test_arducam_tof_config.py` does with config.py.
- The head floor fit and the RGB-D calibration read this depth too (contract consumers): the floor retention test is their guard. Deploy with `scripts/deploy-nodes.sh client arducam_tof` (orchestrator).

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-27 done: superseded by realsense-d435-head-switchov/laser-filter-on-the-d435-clo

- 2026-10-27 actuals: no agent records
