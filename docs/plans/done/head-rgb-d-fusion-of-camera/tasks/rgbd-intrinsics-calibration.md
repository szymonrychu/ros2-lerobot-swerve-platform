---
title: RGBD intrinsics calibration
status: done
repo: ros2-lerobot-sverve-platform
value: "Both head cameras get measured intrinsics from one printed ChArUco board, so registration and RTAB-Map use real calibrations instead of datasheet guesses."
files: [scripts/head_rgbd_calibration/charuco.py, scripts/head_rgbd_calibration/intrinsics.py, tests/rgbd_synthetic.py, tests/test_head_rgbd_calib_intrinsics.py]
verify: "uv run pytest tests/test_head_rgbd_calib_intrinsics.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration"
depends_on: [rgbd-calibration-frames]
owns: [rgbd-synthetic-scenes, head-rgbd-charuco]
uses: [head-rgbd-calib-config, head-rgbd-calib-frames]
model: sonnet
estimate: 75
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--rgbd-intrinsics-calibration
verified_head: f851d784c1f6a4c0976fbe0f27ce7f8d4053700f
run: head-rgb-d-fusion-of-camera
provides: [rgbd-synthetic-scenes, head-rgbd-charuco]
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--rgbd-intrinsics-calibration
acs_for_audit: [a2, a3, a4]
process_gaps: []
---

# RGBD intrinsics calibration

## Requirement

- `scripts/head_rgbd_calibration/charuco.py`: `make_board(cfg: BoardConfig) -> cv2.aruco.CharucoBoard` and `detect(image: np.ndarray, board) -> Detection | None` (`cv2.aruco.CharucoDetector`, corner ids and sub-pixel image points; None when fewer than 6 corners), used for both the RGB (grey) and the normalised ToF amplitude.
- `scripts/head_rgbd_calibration/intrinsics.py`: `calibrate(detections: list[Detection], board, image_size) -> IntrinsicsResult` (`cv2.calibrateCamera`, plumb_bob; K, D, RMS, per-view errors, view count) refusing fewer than `limits.min_views` views (raises naming the count); `calibrate_tof(detections, board, image_size, limits) -> IntrinsicsResult` that runs with distortion fixed at zero (`CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K1 | CALIB_FIX_K2 | CALIB_FIX_K3`) and free, and returns the zero-D result when its RMS <= `limits.tof_zero_distortion_rms_px`, else the free one, with `zero_distortion_rms` recorded either way (fix-as-ac: zero-distortion assumption); `compare_sdk(result, sdk_camera_info) -> dict` (fx/fy/cx/cy differences in % against the SDK `/tof/camera_info`); `write_camera_calibration_yaml(path, result, camera_name, image_size)` writing the ROS camera_calibration format (`image_width`, `image_height`, `camera_name`, `camera_matrix`, `distortion_model`, `distortion_coefficients`, `rectification_matrix` identity, `projection_matrix`) that camera_calibration_parsers reads; writing refuses (raises, writes nothing) when RMS exceeds the camera's limit (`rgb_max_rms_px` 0.5 / `tof_max_rms_px` 0.3).
- `tests/rgbd_synthetic.py` (shared synthetic scenes for every calibration and validation test): render the configured ChArUco board through a known pinhole (K, optional plumb_bob D) at a given board pose with `cv2.aruco.CharucoBoard.generateImage` + homography warp, for an RGB camera (576x324, fx=fy=432, cx=288, cy=162) and a ToF camera (240x180, fx=fy=191, cx=120, cy=90) related by a known extrinsic (default ToF 0.05 m to the left of RGB, 1 deg yaw); a pose generator spanning 0.5-1.5 m and +-40 deg tilt; ToF amplitude rendered as uint16 with noise and depth as uint16 mm of the board plane.
- Tests first in `tests/test_head_rgbd_calib_intrinsics.py`, seen failing: 20 synthetic RGB views recover fx, fy within 1 % and cx, cy within 3 px; 20 synthetic ToF amplitude views at 240x180 (after `normalise_amplitude`) recover fx, fy within 2 % and cx, cy within 2 px; with zero true distortion the ToF result is the zero-D model; with true k1 = -0.1 the free model is chosen and k1 is recovered within 0.03; partial board views (half the board outside the image) still detect >= 6 corners; fewer than min_views raises; the YAML round-trips through `yaml.safe_load` with the right keys and shapes; RMS over the limit writes nothing; the SDK comparison reports the % difference.

## Acceptance conditions

- a1: synthetic RGB and ToF views recover the known intrinsics within tolerance [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]
- a2: the ToF zero-distortion model is used only when its RMS is within 0.3 px, else the measured distortion [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]
- a3: too few views or RMS over the limit write nothing [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]
- a4: calibrations are written as ROS camera_calibration YAML [evidence: test: tests/test_head_rgbd_calib_intrinsics.py]

## Files

- `scripts/head_rgbd_calibration/charuco.py`
- `scripts/head_rgbd_calibration/intrinsics.py`
- `tests/rgbd_synthetic.py`
- `tests/test_head_rgbd_calib_intrinsics.py`

## Interfaces

- `scripts/head_rgbd_calibration/charuco.py:make_board`, `detect`, `Detection` (part `head-rgbd-charuco`).
- `scripts/head_rgbd_calibration/intrinsics.py:calibrate`, `calibrate_tof`, `compare_sdk`, `write_camera_calibration_yaml`, `IntrinsicsResult`.
- `tests/rgbd_synthetic.py` (part `rgbd-synthetic-scenes`): `render_rgb_view`, `render_tof_view`, `board_poses`, `RGB_K`, `TOF_K`, `TRUE_EXTRINSIC`.

## Expected outcome

Synthetic ChArUco sets recover both cameras' intrinsics and write YAML files head_camera_cpp and head_rgbd_cpp load.

## Steps

- Write tests/rgbd_synthetic.py and the tests; run `uv run pytest tests/test_head_rgbd_calib_intrinsics.py -q`; see them fail.
- Implement charuco.py and intrinsics.py; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): ChArUco intrinsics for head RGB and ToF`.

## Notes

- Folds in the RTAB-Map epic's planned RGB-only `camera-calibration-script` (design D3): one calibration path, ChArUco instead of a checkerboard because the 240x180 amplitude image often shows partial boards.
- opencv-python-headless >= 4.9 ships `cv2.aruco.CharucoDetector` (objdetect); no contrib wheel needed.
- RGB is calibrated at the 576x324 mapping size; head_camera_cpp scales a calibration to each published size (front-camera D3).

## Ruling

Accepted by the orchestrator: verified and merged into main (root 1507 passed); head-rgbd-package-config verified by hand: scripts/cpp-node-test.sh head_rgbd_cpp PASS after freeing the Docker VM disk (the apt signature failure was a full Docker disk).
