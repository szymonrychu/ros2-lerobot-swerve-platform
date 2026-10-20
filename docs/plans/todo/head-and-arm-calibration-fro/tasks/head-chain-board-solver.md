---
title: Head chain board solver
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head height, mount tilt, tilt zero and camera lever are measured from a board on the floor instead of guessed from CAD."
files: [shared/ros2_common/head_rgbd_calibration/head_chain_fit.py, tests/head_arm_synthetic.py, tests/test_head_chain_fit.py]
verify: "uv run --frozen pytest tests/test_head_chain_fit.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types, urdf-chain-fk-core, head-chain-override-format]
owns: [head-chain-fit, head-arm-synthetic-scenes]
uses: [head-arm-calib-config, head-arm-view-types, urdf-chain-fk, head-chain-override]
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

# Head chain board solver

## Requirement

The head chain fit from floor-board views (owns `head-chain-fit` and `head-arm-synthetic-scenes`; design D1, D3, D8, D12).

- New `shared/ros2_common/head_rgbd_calibration/head_chain_fit.py`:
  - `fit_head_chain(views: list[FloorView], camera_matrix: np.ndarray, dist: np.ndarray, head_urdf: str, cfg: HeadArmFitConfig) -> HeadChainFit`.
  - Model: board corner (`board_corners_3d(cfg.board)`) -> spot pose on the floor (x_s, y_s, yaw_s; z = `cfg.board_z_m`; roll = pitch = 0) -> robot base_link -> head camera optical frame through `UrdfChain` of `head_urdf` with origin overrides built from the parameters and the view's measured `head_pan`/`head_tilt` -> `cv2.projectPoints` with the fixed intrinsics.
  - Free parameters exactly as design D3: `head_mount_joint` z, roll, pitch; `head_tilt` origin pitch (tilt zero) and roll (skew); `head_camera_joint` x, z, roll, yaw; per spot x, y, yaw. Every other origin component comes from `head_urdf` (so mount x, y, yaw keep whatever the given URDF holds).
  - Init: per view `cv2.solvePnP` of the board corners; spot x/y/yaw from the median over that spot's views mapped through the URDF chain; head parameters start at the URDF values.
  - One `scipy.optimize.least_squares(loss="huber", f_scale=cfg.huber_f_scale_px)` over the corner residuals of all views except the held-out ones (every `head_limits.holdout_every`-th view in view order); sigma per parameter from `(J^T J)^-1 * sum(r^2) / (m - n)` at the solution.
  - `HeadChainFit` (frozen dataclass): `origins: dict[str, Origin]` for the three HEAD_CHAIN_JOINTS (full xyz/rpy), `params: dict[str, float]`, `sigma: dict[str, float]`, `urdf_params: dict[str, float]` (start values), `rms_px`, `holdout_rms_px`, `views`, `spots`, `holdout_views`, `failed_limits: list[str]` (each `head_limits.*` key exceeded: `rms_px`, `holdout_rms_px`, `max_sigma_mm` for translation parameters, `max_sigma_deg` for angle parameters, `min_views`, `min_spots`), `passed: bool`.
  - `head_report_markdown(fit, cfg, session, date) -> str`: a parameter table (name, value, sigma, URDF value, change in mm or deg), RMS, held-out RMS, view and spot counts, limits with pass/fail.
  - ValueError when fewer than 3 views are given or a view lacks `head_pan` or `head_tilt` (message names the view id).
- New `tests/head_arm_synthetic.py` (helper, not a test; also used by the arm fit and the node tests): `SYNTH_CAMERA_MATRIX` (fx = fy = 440, cx 288, cy 162, 576x324, zero distortion); `HeadArmTruth` (head origins as overrides of head.urdf, arm offsets, plate pose in gripper_link); `floor_scene(truth, spots: list[tuple[x, y, yaw]], grid: list[tuple[pan, tilt]], noise_px: float, seed: int) -> list[FloorView]` projecting the board corners through the true chain, keeping only views with >= 6 corners inside the image, adding Gaussian pixel noise.
- New `tests/test_head_chain_fit.py`:
  - truth: mount z +12 mm, roll +1.0 deg, pitch -1.5 deg, tilt zero +2.0 deg, camera x +5 mm, z -4 mm, roll +0.5 deg, yaw -0.7 deg; 3 spots at 0.5-0.9 m; a 5 x 4 pan/tilt grid with tilt 35-60 deg down; 0.3 px noise: every free head parameter within 3 sigma of the truth and within 1.5 mm / 0.15 deg; `rms_px` < 0.5, `holdout_rms_px` < 0.6, `passed`;
  - noiseless: every parameter within 1e-6;
  - two spots only: `min_spots` in `failed_limits`, `passed` false;
  - 3 px noise: a sigma limit is named in `failed_limits`;
  - the fixed origin components equal the URDF;
  - `head_report_markdown` contains the parameter table header and every parameter name.

## Acceptance conditions

- a1: a perturbed head chain is recovered from noisy floor views within tolerance and sigma [evidence: test:tests/test_head_chain_fit.py::test_recovers_perturbed_head_chain_with_noise]
- a2: noiseless views recover the chain exactly [evidence: test:tests/test_head_chain_fit.py::test_noiseless_recovery_exact]
- a3: too few spots fail the min_spots limit [evidence: test:tests/test_head_chain_fit.py::test_two_spots_fail_min_spots]
- a4: a sigma limit is named when the data is too noisy [evidence: test:tests/test_head_chain_fit.py::test_noisy_views_fail_a_sigma_limit]
- a5: fixed components stay at the URDF values [evidence: test:tests/test_head_chain_fit.py::test_fixed_components_stay_at_urdf]
- a6: fit_head_chain exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/head_chain_fit.py::fit_head_chain]
- a7: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/head_chain_fit.py`
- `tests/head_arm_synthetic.py`
- `tests/test_head_chain_fit.py`

## Interfaces

- `head_chain_fit.py::fit_head_chain(views, camera_matrix, dist, head_urdf, cfg) -> HeadChainFit`, `HeadChainFit`, `head_report_markdown` (part `head-chain-fit`).
- `tests/head_arm_synthetic.py::HeadArmTruth`, `floor_scene`, `SYNTH_CAMERA_MATRIX` (part `head-arm-synthetic-scenes`).

## Expected outcome

The head height, mount tilt, tilt zero and camera lever are measured from a board on the floor instead of guessed from CAD.

## Steps

- write the synthetic helper and the tests (red), implement the fit, verify. Model this carefully: the URDF rpy is fixed-axis xyz; head_tilt + is down.
- Run `uv run --frozen pytest tests/test_head_chain_fit.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

