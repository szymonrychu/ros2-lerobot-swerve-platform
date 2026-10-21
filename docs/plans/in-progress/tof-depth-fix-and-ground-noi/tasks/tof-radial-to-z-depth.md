---
title: ToF radial to z depth
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The ToF floor is flat again: the 2.7 cm dome from treating radial distance as depth is gone for every depth consumer."
files: [nodes/bridges/arducam_tof/arducam_tof/bridge.py, nodes/bridges/arducam_tof/arducam_tof/config.py, nodes/bridges/arducam_tof/tests/test_bridge.py, nodes/bridges/arducam_tof/tests/test_config.py, nodes/bridges/arducam_tof/tests/data/floor_capture.json, nodes/bridges/arducam_tof/README.md]
verify: "cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: []
owns: [tof-depth-z]
uses: []
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-depth-fix-and-ground-noi--tof-radial-to-z-depth
branch: tof-depth-fix-and-ground-noi--tof-radial-to-z-depth
verified_head: ""
run: ""
provides: []
---

# ToF radial to z depth

## Requirement

arducam_tof publishes z-depth and the calibrated ToF K (design D1, D8; approval A1).

- `config.py` `TofConfig` gains `tof_calibration_file: Path | None = None` (a camera_calibration YAML, the same file head_rgbd_cpp reads) and `depth_is_radial: bool = True`. Unknown keys stay rejected.
- `bridge.py`: at camera open, load the K (fx, fy, cx, cy) of `tof_calibration_file` with yaml + pydantic (image_width/image_height must equal the SDK frame size, else log an error and use the SDK intrinsics); a missing or invalid file logs a warning once and falls back to the SDK intrinsics (a real calibration, never made-up values). The distortion coefficients of the file are NOT used.
- A module-level function `ray_norm(u: np.ndarray, v: np.ndarray, intrinsics: Intrinsics) -> np.ndarray` returns sqrt(((u-cx)/fx)^2 + ((v-cy)/fy)^2 + 1); `radial_to_z(depth_mm: np.ndarray, intrinsics: Intrinsics) -> np.ndarray` divides a uint16 depth image by the per-pixel ray norm (pixel centres u = column, v = row), rounds, keeps 0 as invalid and returns uint16. The per-pixel factor is cached per (shape, intrinsics), not recomputed every frame.
- `publish_frame` applies `radial_to_z` after `depth_to_mm` when `depth_is_radial` is true, with the same intrinsics that go out in `/tof/camera_info`; `camera_info_msg` carries that K (P equal to K, D zeros). When no intrinsics exist at all (no file, SDK without focal length) depth is published unconverted and camera_info is not published, as today.
- Fixture `tests/data/floor_capture.json`: frame 0 of `/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/tof10.json` copied verbatim (688 /tof/points_filtered points, xyz in tof_optical_frame, plus `frame_pose` in map), with `sdk_k` [190.92, 191.27, 120.0, 90.0] and `calibrated_k` [207.787, 208.171, 114.743, 95.996] added and a `source` note (robot static, head pan 0 / tilt 30 down, flat wood floor, 2026-10-21). The points were made with the SDK K treating the SDK value as z, so u = fx_sdk*x/z + cx_sdk, v = fy_sdk*y/z + cy_sdk and the SDK value is z.
- Tests first (red seen) in `tests/test_bridge.py`: (a) `test_radial_depth_becomes_z_depth`: synthetic rays of a plane at known z through `radial_to_z` give that z within 1 mm, centre pixel unchanged, 0 stays 0; (b) `test_floor_capture_dome_is_flat_after_radial_to_z`: on the fixture, fit a least-squares plane (3 sigma-clip passes) to the floor points 0.3-0.9 m ahead in map (frame_pose), median residual of the centre columns (|u-120| < 30) minus the edges (|u-120| > 70) is > 2.0 cm as captured and < 1.0 cm after radial_to_z with calibrated_k; (c) `test_camera_info_uses_the_calibration_file_k`, `test_missing_calibration_file_falls_back_to_sdk_k`, `test_calibration_file_of_another_size_falls_back_to_sdk_k`, `test_depth_is_radial_false_publishes_depth_unchanged`. Config tests in `tests/test_config.py` for the two keys.
- README of the node: depth is z-depth, K source and fallback, the two keys, the fixture and how it was taken.

Notes: brief /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/briefs/tof-depth-fix.md; evidence script /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/domefit/fit.py (radial with calibrated K: dome 2.7 cm -> -0.2 cm). head_rgbd_cpp and the calibration plane check read the same depth as z with their own K; after this change their input is the z they assume, so no code change there (D8). Verify numpy is a node dependency (it already is).

## Acceptance conditions

- a1: radial_to_z turns the radial distance of every pixel into z-depth with the given K, 0 stays invalid [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_radial_depth_becomes_z_depth]
- a2: on the committed floor capture the centre-minus-edge dome is above 2 cm as captured and below 1 cm after radial_to_z with the calibrated K [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_floor_capture_dome_is_flat_after_radial_to_z]
- a3: /tof/camera_info carries the K of tof_calibration_file with zero distortion [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_camera_info_uses_the_calibration_file_k]
- a4: a missing calibration file falls back to the SDK K for both conversion and camera_info [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_missing_calibration_file_falls_back_to_sdk_k]
- a5: a calibration file whose image size differs from the SDK frame falls back to the SDK K [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_calibration_file_of_another_size_falls_back_to_sdk_k]
- a6: depth_is_radial false publishes the SDK depth unchanged [evidence: test:nodes/bridges/arducam_tof/tests/test_bridge.py::test_depth_is_radial_false_publishes_depth_unchanged]
- a7: node tests and lint pass [evidence: command:cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run poe lint]
- a8: the node README documents z-depth, the K source and fallback and both config keys [evidence: judgement]

## Files

- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/tests/test_config.py`
- `nodes/bridges/arducam_tof/tests/data/floor_capture.json`
- `nodes/bridges/arducam_tof/README.md`

## Interfaces

Part `tof-depth-z` (contract): `arducam_tof/bridge.py::ray_norm(u, v, intrinsics) -> np.ndarray`, `arducam_tof/bridge.py::radial_to_z(depth_mm, intrinsics) -> np.ndarray`; `/tof/depth/image_raw` 16UC1 mm z-depth; `/tof/camera_info` K from `tof_calibration_file` (SDK K fallback), D zero. `config.py::TofConfig.tof_calibration_file: Path | None`, `TofConfig.depth_is_radial: bool = True`. Keep topic names, encodings, header sharing, stamp rule, metrics.

## Expected outcome

`cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q` passes with the floor-capture dome test; lint clean.

## Steps

- Copy frame 0 of /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/tof10.json into tests/data/floor_capture.json with sdk_k, calibrated_k and source added.
- Write the new tests in tests/test_bridge.py and tests/test_config.py; run `uv run --frozen pytest tests -q`, see FAIL (no radial_to_z, unknown config keys).
- Implement ray_norm, radial_to_z, the calibration file loading and fallback, the camera_info K; rerun, see PASS.
- `uv run poe lint`; update README.md.
- Commit `fix(arducam_tof): publish z-depth and the calibrated ToF K`.

## Notes

Read the calibration YAML with yaml + a small pydantic model (camera_matrix.data, image_width, image_height); never apply distortion_coefficients. The cache key includes the intrinsics so a reopen with another K recomputes. `test_good_frame_publishes_depth_amplitude_and_camera_info_with_one_header` and the depth tests may need the radial conversion accounted for (centre pixel unchanged, off-centre scaled); do not weaken their assertions.
