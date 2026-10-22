---
title: ToF flying pixel retune
status: todo
repo: ros2-lerobot-sverve-platform
value: "The ToF filter goes back on without starving the ground fit, so mixed pixels at object edges stop faking ramps and drops"
files: [nodes/bridges/arducam_tof/arducam_tof/bridge.py, nodes/bridges/arducam_tof/arducam_tof/config.py, nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py, nodes/bridges/arducam_tof/tests/test_config.py, nodes/bridges/arducam_tof/tests/test_bridge.py, nodes/bridges/arducam_tof/README.md, ansible/group_vars/client.yml, tests/test_arducam_tof_config.py]
verify: "(cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint) && uv run pytest tests/test_arducam_tof_config.py -q && uv run poe test-ansible"
depends_on: [tof-amplitude-gate, capture-baseline-recordings]
owns: [flying-pixel-retune]
uses: [baseline-fixtures, amplitude-gate]
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

# ToF flying pixel retune

## Requirement

Retune the ToF flying-pixel filter in arducam_tof on real recorded depth and turn it back on (design D26; approval G2;
cross-epic contract tof-flying-pixel-retune). Live, the 3x3 rule "jump > max(30 mm, 5 % of depth)"
(`arducam_tof/bridge.py:flying_pixel_filter`, task ground-detection-robustness/tof-flying-pixel-filter) removed about
70 % of the pixels (`tof_flying_pixels_removed_total` about 300k/s at 10 Hz on 43,200 px) and starved the ground fit,
so it is off in `ansible/group_vars/client.yml`. It uses part baseline-fixtures: the raw-depth frame subset
`nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz` (filter off when recorded) with `truth_board`,
`truth_ramp`, `truth_empty`, `truth_box`, `box_height_m` and `floor_plane`, and part amplitude-gate (`amplitude.py`).

- First step, before any change, recorded in the commit message and the README: run the current rule on every frame of
  the subset and report the removed share per scene, on the floor (inside `truth_empty` and every floor pixel outside
  the object polygons grown by 0.05 m) by range band (0.3-0.6, 0.6-1.0, 1.0-1.5, 1.5-2.0 m), on the box top and on
  the board, so the cause of the 70 % is named with numbers (for example the depth step between neighbouring rows of
  a grazing floor, sensor noise, or the min_range_m cut).
- Then change the keys and, where the numbers show the rule itself is wrong, the rule inside
  `flying_pixel_filter` (for example a test against the plane through the neighbours, a minimum count of
  disagreeing neighbours, or a range-dependent threshold), keeping it vectorised numpy with no per-pixel Python loop,
  0 pixels never compared and never invalidating neighbours, and a removed pixel set to 0 (REP 118, no placeholder).
  New keys, if any, go into `TofConfig` with meaning, unit and range in the docstring, and into the client.yml block.
- Definitions on the real frames (heights from `tof_K`, `tof_D`, `tof_base_T` and `floor_plane`; footprints from the
  truth polygons): floor pixels as in the first step, 0.3-2.0 m range; box-top pixels inside `truth_box` shrunk by
  0.02 m with height 0.03-0.08 m; board pixels inside `truth_board` shrunk by 0.02 m; mixed pixels at the box edges:
  pixels whose floor point lies outside `truth_box` grown by 0.015 m (the box faces) and inside it grown by 0.15 m,
  with height strictly between 0.012 m and 0.038 m above the plane, counted on the unfiltered frame.
- Thresholds, on the real frames, in every scene that contains the class:
  - floor pixels kept >= 95 % with the filter alone (user requirement);
  - mixed pixels at the box edges removed >= 80 % (my call, design D26);
  - box-top pixels and board pixels kept >= 95 % each;
  - filter followed by the amplitude gate (`amplitude_gate` with the shipped `amplitude_min_normalised`) keeps
    >= 90 % of the floor pixels (my call, design D26: the ground fit must not starve again).
- The synthetic tests of ground-detection-robustness in `tests/test_bridge.py` (tilted floor kept, mixed band behind
  the box removed, invalid pixels stay invalid, published depth filtered and counted) keep passing; a changed rule
  updates their parameters only when the real-data numbers above justify it, never their thresholds.
- `ansible/group_vars/client.yml` arducam_tof block: `flying_pixel_filter: true` with the retuned keys and a comment
  naming the real-data numbers (date, floor kept, mixed removed). `tests/test_arducam_tof_config.py` gains
  `test_flying_pixel_filter_on_in_client` (the deployed block has the filter on and its keys load in the real
  driver's `TofConfig`) and keeps `DRIVER_KEYS` in sync.
- README `nodes/bridges/arducam_tof/README.md` `## Flying-pixel filter`: why it was off, the first-step numbers, the
  chosen rule and keys, the before/after table per scene, and that the order in `publish_frame` is min_range cut,
  flying-pixel filter, amplitude gate.
- Tests first (seen failing) in `nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py` on the committed
  `amplitude_frames.npz`: `test_real_floor_pixels_kept`, `test_real_box_edge_mixed_pixels_removed`,
  `test_real_box_top_and_board_kept`, `test_real_filter_and_gate_keep_floor`; `tests/test_config.py` gains
  `test_flying_pixel_retuned_defaults` (the shipped defaults equal the client.yml values, ranges enforced, unknown key
  still rejected).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: on the recorded frames the filter alone keeps >= 95 % of the floor pixels 0.3-2.0 m in every scene [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_floor_pixels_kept]
- a2: on the recorded frames >= 80 % of the mixed pixels at the box edges are removed [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_box_edge_mixed_pixels_removed]
- a3: box-top and board pixels are kept >= 95 % [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_box_top_and_board_kept]
- a4: filter plus amplitude gate keep >= 90 % of the floor pixels [evidence: test:nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py::test_real_filter_and_gate_keep_floor]
- a5: the shipped defaults equal the client.yml values and are validated [evidence: test:nodes/bridges/arducam_tof/tests/test_config.py::test_flying_pixel_retuned_defaults]
- a6: client.yml turns the filter on and the deployed block loads in the real driver [evidence: test:tests/test_arducam_tof_config.py::test_flying_pixel_filter_on_in_client]
- a7: the synthetic filter tests, every node test and the lint pass [evidence: command:cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint]
- a8: the README documents the retune with the first-step and before/after numbers [evidence: doc-section:nodes/bridges/arducam_tof/README.md#Flying-pixel filter]
- a9: the first-step numbers naming the cause of the 70 % removal are in the commit message and no threshold of the Requirement was loosened [evidence: judgement]

## Files

- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_flying_pixel_real.py`
- `nodes/bridges/arducam_tof/tests/test_config.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_arducam_tof_config.py`

## Interfaces

Builds part flying-pixel-retune (cross-epic contract tof-flying-pixel-retune):
`nodes/bridges/arducam_tof/arducam_tof/bridge.py:flying_pixel_filter` (same name, possibly new keyword parameters
for a changed rule) and the `TofConfig` keys `flying_pixel_filter`, `flying_pixel_jump_ratio`,
`flying_pixel_min_jump_mm` plus any new key; `/tof/depth/image_raw` keeps 16UC1 mm with 0 = invalid. Keeps the
counter `tof_flying_pixels_removed_total`. Uses part baseline-fixtures (`tests/data/amplitude_frames.npz` keys
`tof_depth_mm`, `tof_amplitude`, `tof_base_T`, `tof_K`, `tof_D`, `tof_size`, `scene`, `truth_board`, `truth_ramp`,
`truth_empty`, `truth_box`, `box_height_m`, `floor_plane`) and part amplitude-gate (`arducam_tof/amplitude.py`).

## Expected outcome

`cd nodes/bridges/arducam_tof && uv run pytest tests -q` PASS including the four real-data tests; the client.yml
block has `flying_pixel_filter: true` with the retuned keys; the README has the before/after table.

## Steps

- Run the current rule over `amplitude_frames.npz` in the shell (numpy one-off, no committed scratch file) and record
  the per-scene, per-band removed shares; name the cause.
- Write `tests/test_flying_pixel_real.py` and `test_flying_pixel_retuned_defaults`;
  `uv run pytest tests/test_flying_pixel_real.py tests/test_config.py -q` FAILS on the current keys.
- Change the keys (and the rule if the numbers demand it); the same command PASSES; `uv run pytest tests -q` PASSES
  (the synthetic tests of `test_bridge.py` included).
- Set the client.yml block, add `test_flying_pixel_filter_on_in_client`; `uv run pytest tests/test_arducam_tof_config.py -q`
  and `uv run poe test-ansible` PASS.
- README section; commit `fix(arducam_tof): retune the flying-pixel filter on recorded depth and turn it on` with the
  first-step numbers in the message.

## Notes

- Code task of run 2 (design D29); the orchestrator deploys arducam_tof at live step 2 together with the amplitude
  gate, before capture-gated-recordings.
- The filter reaches every `/tof/depth/image_raw` consumer (head floor fit, RGB-D calibration, Nav2 ToF voxel layer,
  ground estimator): the floor retention test is what guards them.
- Prior art: `nodes/head_rgbd_cpp/src/core/tof_filter.cpp:filter_jump_edges` (the same 8-neighbour rule on the
  registered RGB-D image).
