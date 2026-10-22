---
title: ToF flying pixel filter
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Object edges stop dragging fake ramps and steps into every ToF consumer, at the source."
files: [nodes/bridges/arducam_tof/arducam_tof/bridge.py, nodes/bridges/arducam_tof/arducam_tof/config.py, nodes/bridges/arducam_tof/tests/test_bridge.py, nodes/bridges/arducam_tof/tests/test_config.py, nodes/bridges/arducam_tof/README.md, ansible/group_vars/client.yml, tests/test_arducam_tof_config.py, tests/README.md]
verify: "(cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint) && uv run pytest tests/test_arducam_tof_config.py -q"
depends_on: [occlusion-and-segment-config]
owns: [tof-depth-flying-pixel-filter]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--tof-flying-pixel-filter
branch: ground-detection-robustness--tof-flying-pixel-filter
verified_head: ""
run: ""
provides: []
---

# ToF flying pixel filter

## Requirement

Remove ToF mixed ("flying") pixels at depth discontinuities in arducam_tof before point_cloud_xyz decimates the image (design D15, brief root cause 1). Every object edge currently drags a 10-25 cm tail of intermediate heights into /tof/points_filtered (fake ramps and steps behind the 5 cm box).

- `arducam_tof/bridge.py`: a module-level `flying_pixel_filter(depth_mm: np.ndarray, jump_ratio: float, min_jump_mm: int) -> np.ndarray` next to `depth_to_mm` / `radial_to_z`: a valid pixel becomes 0 (invalid, REP 118; no placeholder value) when a valid pixel of its 3x3 neighbourhood differs from it by more than max(min_jump_mm, jump_ratio * its depth). Vectorised numpy (shifted views), no per-pixel Python loop; 0 pixels are never compared. `publish_frame` applies it to the full-resolution z-depth (after `radial_to_z`, before publishing) when `flying_pixel_filter` is true, and increments a Prometheus counter `tof_flying_pixels_removed_total` by the number of pixels it zeroed.
- `arducam_tof/config.py` `TofConfig`: `flying_pixel_filter: bool = True`, `flying_pixel_jump_ratio: float` (0 < v <= 0.5) and `flying_pixel_min_jump_mm: int` (0 <= v <= 1000), documented in the class docstring with meaning and unit; defaults chosen from the synthetic tests below and stated in the README.
- Tests first (seen failing), synthetic depth images in `tests/test_bridge.py`, rendered with a pinhole of the sensor's 240x180 size from a camera 0.45 m above the floor pitched 30 deg down (the test_node head pose):
  - `test_flying_pixel_filter_keeps_tilted_floor`: a flat floor covering 0.3-2.0 m (grazing incidence at the far end) with 5 mm depth noise keeps >= 95 % of its valid pixels;
  - `test_flying_pixel_filter_removes_mixed_pixels_behind_box`: a 5 cm box top (x 0.60-0.78 ahead of the camera) with the floor behind it and a mixed band of 1-2 pixel rows at the box's far top edge whose depths are linear blends of the edge depth and the floor depth behind: >= 95 % of the band pixels become 0, and >= 95 % of the box-top and floor pixels farther than 2 pixels from the edge survive;
  - `test_flying_pixel_filter_keeps_invalid_pixels_invalid`: 0 pixels stay 0 and do not invalidate their neighbours;
  - `test_published_depth_has_flying_pixels_removed`: through the existing Harness, a frame with a mixed band publishes depth with the band zeroed and the counter incremented; with `flying_pixel_filter: false` the depth is published unchanged.
- `tests/test_config.py`: `test_flying_pixel_keys_validated` (defaults, ranges, unknown key still rejected).
- Ansible in sync (CLAUDE.md): the arducam_tof `config` block in `ansible/group_vars/client.yml` sets the three keys with comments; `tests/test_arducam_tof_config.py` `DRIVER_KEYS` gains them (the deployed config still loads in the real driver).
- READMEs: `nodes/bridges/arducam_tof/README.md` (Config and Depth sections: the filter, its keys, the counter, why it exists); `tests/README.md` if a root test description changes.
- Notes: the replay data is recorded downstream of point_cloud_xyz and keeps its mixed pixels, so the ground estimator's own defences (other tasks) do not depend on this filter. The head floor fit and the RGB-D calibration read this depth too (cross-epic contract tof-depth-flying-pixel-filter): the flat floor and a flat board must survive, which the floor test pins. Deploy with `scripts/deploy-nodes.sh client arducam_tof` after merge (orchestrator).
- Prior art (reuse the rule, not the code: different language and node): `nodes/head_rgbd_cpp/src/core/tof_filter.cpp:filter_jump_edges` (task head-rgb-d-fusion-of-camera/tof-jump-edge-filter) applies the same 8-neighbour jump rule with `jump_rel` 0.05 and `jump_abs_mm` 30 (`nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`) to the registered RGB-D image only; /tof/depth/image_raw and so point_cloud_xyz and /tof/points_filtered are unfiltered today. Start from those defaults and keep them unless the floor test needs otherwise; head_rgbd_cpp then filters already-filtered depth, which it tolerates (0 pixels are invalid there too).

## Acceptance conditions

- a1: a tilted flat floor 0.3-2.0 m keeps >= 95 % of its pixels [evidence: test: nodes/bridges/arducam_tof/tests/test_bridge.py::test_flying_pixel_filter_keeps_tilted_floor]
- a2: the mixed pixels between a 5 cm box top and the floor behind it are removed (>= 95 %) while box top and floor away from the edge survive (>= 95 %) [evidence: test: nodes/bridges/arducam_tof/tests/test_bridge.py::test_flying_pixel_filter_removes_mixed_pixels_behind_box]
- a3: invalid pixels stay invalid and never invalidate neighbours [evidence: test: nodes/bridges/arducam_tof/tests/test_bridge.py::test_flying_pixel_filter_keeps_invalid_pixels_invalid]
- a4: the published depth is filtered and counted, and unchanged when the filter is off [evidence: test: nodes/bridges/arducam_tof/tests/test_bridge.py::test_published_depth_has_flying_pixels_removed]
- a5: the three keys are validated by TofConfig [evidence: test: nodes/bridges/arducam_tof/tests/test_config.py::test_flying_pixel_keys_validated]
- a6: client.yml sets the keys and the deployed config loads in the real driver [evidence: test: tests/test_arducam_tof_config.py::test_deployed_config_holds_only_keys_the_driver_accepts]
- a7: node tests and lint pass [evidence: command: cd nodes/bridges/arducam_tof && uv run pytest tests -q && uv run poe lint]

## Files

- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/tests/test_config.py`
- `nodes/bridges/arducam_tof/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_arducam_tof_config.py`
- `tests/README.md`

## Interfaces

`arducam_tof/bridge.py:flying_pixel_filter(depth_mm, jump_ratio, min_jump_mm) -> np.ndarray` (uint16, 0 = invalid); `TofConfig.flying_pixel_filter`, `flying_pixel_jump_ratio`, `flying_pixel_min_jump_mm`; counter `tof_flying_pixels_removed_total`. Topics, encodings and the z-depth meaning of /tof/depth/image_raw are unchanged (contract tof-depth-z); new cross-epic contract tof-depth-flying-pixel-filter.

## Expected outcome

`cd nodes/bridges/arducam_tof && uv run pytest tests -q` PASS with the four new bridge tests and the config test, `uv run poe lint` clean, `uv run pytest tests/test_arducam_tof_config.py -q` PASS; client.yml sets the three keys.

## Steps

- write the four bridge tests and the config test; `cd nodes/bridges/arducam_tof && uv run pytest tests -q` FAILS on them
- implement `flying_pixel_filter`, the config keys, the publish_frame call and the counter; the same command PASSES; `uv run poe lint` PASSES
- add the keys to client.yml and DRIVER_KEYS; `uv run pytest tests/test_arducam_tof_config.py -q` PASSES
- update the READMEs; commit `feat(arducam_tof): drop flying pixels at depth edges`

## Notes

See the Requirement's notes and prior art. No placeholder data: removed pixels are 0, never interpolated.

- 2026-10-22 amended: planner: add head_rgbd_cpp jump edge filter prior art
