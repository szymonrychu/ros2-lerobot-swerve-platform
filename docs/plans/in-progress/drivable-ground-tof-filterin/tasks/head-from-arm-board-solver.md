---
title: Head from arm board solver
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Head yaw, mount x/y and the rest of the head chain are measured against the trusted arm, which the floor fit cannot see."
files: [shared/ros2_common/head_rgbd_calibration/head_arm_fit.py, tests/head_arm_synthetic.py, tests/test_head_arm_fit.py]
verify: "uv run --frozen pytest tests/test_head_arm_fit.py tests/test_head_arm_calib_config.py -q && uv run poe lint"
depends_on: [calib-config-and-view-types]
owns: [head-arm-fit, head-aim, head-arm-synthetic-scenes]
uses: [head-arm-calib-config, head-arm-view-types, urdf-chain-fk, head-chain-override]
model: opus
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--head-from-arm-board-solver
branch: drivable-ground-tof-filterin--head-from-arm-board-solver
verified_head: ""
run: ""
provides: []
---

# Head from arm board solver

## Requirement

The head chain fit from views of the board on the gripper (owns `head-arm-fit`, `head-aim`,
`head-arm-synthetic-scenes`; design D31, D32, D33, D34, D35, D43; approvals H1, H3, H7).

- New `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`:
  - `PARAM_NAMES = ("mount_x_m", "mount_y_m", "mount_z_m", "mount_roll_deg", "mount_pitch_deg", "mount_yaw_deg",
    "tilt_zero_deg", "tilt_roll_deg")`; the five shared with `floor_fit.PARAM_NAMES` keep their meaning
    (`head_mount_joint` z/roll/pitch, `head_tilt` origin pitch and roll).
  - `fit_head_from_arm(views: list[ArmBoardView], camera_matrix: np.ndarray, dist: np.ndarray, head_urdf: str,
    arm_urdf: str, cfg: HeadArmFitConfig, floor: dict[str, float] | None) -> HeadArmFit`.
  - Model per view: board corner (`board_corners_3d(cfg.board)`) -> board pose in `gripper_link` (6 free) -> arm FK
    `UrdfChain.from_text(arm_urdf).transform("gripper_link", arm joints, offsets=cfg.joint_offsets_rad)` ->
    `arm_in_robot_base(cfg.base_in_base_link)` -> robot base_link -> inverse of the `head_camera_optical_frame` pose of
    `UrdfChain.from_text(head_urdf)` at the measured `head_pan`/`head_tilt` with `origins` overrides built from the
    parameters -> `cv2.projectPoints` with the given intrinsics.
  - Free: the eight `PARAM_NAMES` (`head_mount_joint` x, y, z, roll, pitch, yaw; `head_tilt` origin pitch and roll) and
    the 6-DOF board mount. The pan zero is folded into `mount_yaw_deg` (D33); the `head_pan` origin, `head_tilt` xyz,
    `head_camera_joint` and the optical joints keep the values of `head_urdf` (the applied chain); the arm, the offsets,
    the gauge and the intrinsics are fixed.
  - Init: head parameters from `head_urdf` (`start_params`); board mount from `cfg.board_mount` refined by per-view
    `cv2.solvePnP` mapped through the start head chain and the arm FK into `gripper_link` (median translation, mean
    rotation via `scipy.spatial.transform.Rotation.mean`).
  - Hold-out by arm pose: every `limits.holdout_every`-th distinct `arm_pose` (sorted) is held out with all its views.
  - One `scipy.optimize.least_squares(loss="huber", f_scale=cfg.huber_f_scale_px)` over the fit views' corner residuals;
    when `cfg.floor_prior_weight > 0` and `floor` is given, one extra residual per shared floor parameter:
    `floor_prior_weight * (value - floor[name]) / unit` (unit 1 mm for `mount_z_m`, 0.1 deg for angles).
  - Sigma per parameter from `(J^T J)^-1 * sum(r^2) / (m - n)` on the corner residuals at the solution;
    `max_correlation` = the parameter pair with the largest absolute correlation and its value.
  - Validation: `holdout_rms_px` (held-out views reprojected with the fitted chain and mount);
    `holdout_board_mm` (RMS over held-out views of the distance between the board centre from arm FK with the fitted
    mount and the board centre from that view's solvePnP mapped through the fitted head chain).
  - Limits (`failed_limits` names each one exceeded; `passed` is true only when none): `rms_px`, `holdout_rms_px`,
    `holdout_board_mm`, `max_sigma_mm` (x, y, z), `max_sigma_deg` (angles), `min_views` (fit views), `min_arm_poses`
    (distinct arm poses), `min_pan_span_deg` and `min_tilt_span_deg` (peak-to-peak of the measured head angles over all
    views), `floor_delta:<name>` (|fitted - floor[name]| over `max_floor_delta_mm` for `mount_z_m` or
    `max_floor_delta_deg` for the four angles), `start_delta:<name>` (|fitted - start| for `mount_x_m`, `mount_y_m` over
    `max_start_delta_mm`, `mount_yaw_deg` over `max_start_delta_deg`), and `floor_cross_check` when `floor` is None.
  - `HeadArmFit` (frozen dataclass) with `to_json()` (NaN and inf as None): `origins: dict[str, Origin]` for
    `head_mount_joint` and `head_tilt` (full xyz/rpy), `params`, `sigma`, `start_params`, `floor_delta`,
    `max_correlation`, `board_mount` (xyz, rpy), `rms_px`, `holdout_rms_px`, `holdout_board_mm`, `views`, `arm_poses`,
    `holdout_views`, `pan_span_deg`, `tilt_span_deg`, `per_view: list[{view_id, arm_pose, rms_px, held_out}]` (the
    per-pose error shape of the rgbd result, contract `rgbd-result-pose-errors`), `failed_limits`, `passed`.
  - `head_arm_report_markdown(fit: HeadArmFit, cfg: HeadArmFitConfig, session: str, date: str) -> str`: parameter
    table (name, start, fitted, delta, sigma, floor value and delta), RMS, held-out RMS and board error, coverage, the
    largest correlation, limits with pass/fail, and the line "pan zero folded into mount_yaw_deg".
  - `aim_head(chain: UrdfChain, point_base: np.ndarray) -> tuple[float, float]` (owns `head-aim`): pan/tilt (rad) that
    put `point_base` (robot base_link) on the `head_camera_optical_frame` z axis, solved numerically on the given chain,
    clamped to that chain's `head_pan`/`head_tilt` limits.
  - ValueError when fewer than 3 views are given or a view lacks a joint (`require_joints`).
- New `tests/head_arm_synthetic.py` (helper, not a test; also used by arm-board-pose-set and the node tests):
  `SYNTH_CAMERA_MATRIX` (fx = fy = 440, cx 288, cy 162, 576x324, zero distortion); `HeadArmTruth` (head origin
  overrides of the given head URDF, board mount); `arm_board_scene(truth, head_urdf, arm_urdf, arm_poses:
  list[dict[str, float]], head_offsets_deg: list[tuple[float, float]], noise_px: float, seed: int, cfg:
  HeadArmFitConfig) -> list[ArmBoardView]`: the head aimed at the true board centre with `aim_head` on the true chain
  plus each offset, corners projected through the true chain, views with fewer than 6 corners inside the image dropped,
  Gaussian pixel noise; `SYNTH_ARM_POSES` (at least 24 poses spanning wrist_roll, wrist_flex, elbow_flex and
  shoulder_pan, tool point above the floor) for the tests.
- New `tests/test_head_arm_fit.py`, every scene on `nodes/web_ui/urdf/head.urdf` with the applied floor-fit chain of
  2026-10-22 as overrides (mount z 0.246, pitch 1.50 deg, roll -0.32 deg, tilt zero -6.50 deg, tilt roll 0.92 deg) and
  `nodes/web_ui/urdf/so101_arm.urdf`:
  - truth mount x +8 mm, y -6 mm, z +5 mm, roll +0.4, pitch -0.5, yaw +1.5, tilt zero +0.6, tilt roll -0.4 deg from the
    start, board mount 10 mm / 8 deg off its init, 0.3 px noise: every parameter within 3 sigma and within 1.5 mm /
    0.15 deg, `rms_px` < 0.5, `holdout_rms_px` < 0.6, `holdout_board_mm` < 2, `passed`;
  - noiseless: every parameter within 1e-6;
  - a truth with a `head_pan` origin yaw of +1.0 deg and mount yaw 0: `mount_yaw_deg` within 0.05 deg of +1.0 and
    `rms_px` < 0.5 (pins D33);
  - all views within 10 deg of pan: `min_pan_span_deg` failed;
  - `floor` 2 deg off the truth tilt zero: `floor_delta:tilt_zero_deg` failed; `floor=None`: `floor_cross_check` failed;
  - `floor_prior_weight` 100 with `floor` 2 deg off: the fitted tilt zero moves toward the floor value;
  - 3 px noise: a sigma limit failed;
  - `head_camera_joint`, `head_tilt` xyz and the `head_pan` origin equal the URDF in `origins`/the model;
  - `aim_head` puts a point on the optical axis within 1e-6 rad and clamps to the URDF limits;
  - `head_arm_report_markdown` contains every parameter name; a view missing a joint raises ValueError naming it.

## Acceptance conditions

- a1: a perturbed head chain is recovered from noisy board-on-gripper views within tolerance and sigma [evidence: test:tests/test_head_arm_fit.py::test_recovers_perturbed_head_chain_with_noise]
- a2: noiseless views recover the chain exactly [evidence: test:tests/test_head_arm_fit.py::test_noiseless_recovery_exact]
- a3: a pan zero is absorbed by the mount yaw [evidence: test:tests/test_head_arm_fit.py::test_pan_zero_folds_into_mount_yaw]
- a4: too little pan coverage fails the span limit [evidence: test:tests/test_head_arm_fit.py::test_narrow_pan_span_fails]
- a5: the floor cross-check fails a result off the floor values and without them [evidence: test:tests/test_head_arm_fit.py::test_floor_cross_check_limits]
- a6: the optional floor prior pulls toward the floor values [evidence: test:tests/test_head_arm_fit.py::test_floor_prior_weight_pulls]
- a7: noisy data fail a sigma limit [evidence: test:tests/test_head_arm_fit.py::test_noisy_views_fail_a_sigma_limit]
- a8: fixed components stay at the URDF values [evidence: test:tests/test_head_arm_fit.py::test_fixed_components_stay_at_urdf]
- a9: aim_head points the optical axis and clamps to the limits [evidence: test:tests/test_head_arm_fit.py::test_aim_head]
- a10: fit_head_from_arm exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/head_arm_fit.py::fit_head_from_arm]
- a11: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py`
- `tests/head_arm_synthetic.py`
- `tests/test_head_arm_fit.py`

## Interfaces

- `head_arm_fit.py::fit_head_from_arm`, `HeadArmFit`, `PARAM_NAMES`, `head_arm_report_markdown` (part `head-arm-fit`).
- `head_arm_fit.py::aim_head(chain, point_base) -> (pan, tilt)` (part `head-aim`).
- `tests/head_arm_synthetic.py::arm_board_scene`, `HeadArmTruth`, `SYNTH_CAMERA_MATRIX`, `SYNTH_ARM_POSES` (part
  `head-arm-synthetic-scenes`).
- Uses `chain.py::UrdfChain`, `arm_in_robot_base`, `pose_from_xyz_rpy` (contract `urdf-chain-fk`) and
  `urdf_text.py::Origin` (contract `head-chain-override`).

## Expected outcome

Head yaw, mount x/y and the rest of the head chain are measured against the trusted arm, which the floor fit cannot
see; a bad or degenerate result is refused by name.

## Steps

- Write `tests/head_arm_synthetic.py` and `tests/test_head_arm_fit.py` first; run them: red on the missing module.
- Implement `head_arm_fit.py`; run
  `uv run --frozen pytest tests/test_head_arm_fit.py tests/test_head_arm_calib_config.py -q && uv run poe lint`;
  expect every test passed and lint clean.

## Notes

- Model the floor fit's structure (`floor_fit.py`: `chain_origins`, `json_float`, sigma from the Jacobian) rather than
  inventing a second style; reuse its helpers by import where they fit.
- Degeneracies to keep fixed are listed in design D32; if a free parameter's sigma explodes in the noiseless test, the
  parameterisation is wrong, not the tolerance.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
