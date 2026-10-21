---
title: Floor plane chain fit
status: todo
repo: ros2-lerobot-sverve-platform
value: "Measures the head mount height, roll, pitch and tilt zero from the floor the ToF sees, with sigma and limits that refuse a bad fit."
files: [shared/ros2_common/head_rgbd_calibration/floor_fit.py, shared/ros2_common/head_rgbd_calibration/config.py, tests/head_floor_synthetic.py, tests/test_head_floor_fit.py, tests/README.md, shared/README.md]
verify: "uv run --frozen pytest tests/test_head_floor_fit.py tests/test_head_arm_calib_chain.py -q && uv run poe lint"
depends_on: [head-chain-override-format, urdf-chain-fk-core]
owns: [head-floor-fit]
uses: [head-chain-override, urdf-chain-fk]
model: opus
estimate: 80
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Floor plane chain fit

## Requirement

The floor-plane head chain fit core (owns `head-floor-fit`; design D2, D3, D4). numpy and scipy only, no ROS, no OpenCV.

- `shared/ros2_common/head_rgbd_calibration/config.py` gains (StrictModel, every key defaulted, ranges validated):
  - `FloorPlaneConfig`: `ransac_distance_m` 0.015 (0 < v <= 0.1), `ransac_iterations` 200 (>= 10), `min_inliers` 300 (>= 10), `min_inlier_ratio` 0.5 (0 < v <= 1), `max_range_m` 1.8 (> 0), `candidate_band_m` 0.25 (> ransac_distance_m), `seed` 0.
  - `FloorFitLimits`: `min_observations` 30, `min_poses` 8, `min_pan_span_deg` 60.0, `max_height_rms_mm` 8.0, `max_normal_rms_deg` 0.6, `max_sigma_z_mm` 3.0, `max_sigma_deg` 0.3, `max_delta_z_m` 0.10, `max_delta_deg` 10.0 (all > 0).
  - `FloorFitConfig`: `plane: FloorPlaneConfig`, `limits: FloorFitLimits`, `sigma_height_mm` 5.0 and `sigma_normal_deg` 0.5 (residual scales, > 0), `huber_scale` 1.0 (> 0).
- New `shared/ros2_common/head_rgbd_calibration/floor_fit.py`:
  - `PlaneFit(normal: np.ndarray, height_m: float, inliers: int, candidates: int, rms_m: float)`: unit `normal` points from the floor toward the sensor; `height_m` is the sensor origin's distance above the plane (the plane is `normal . p = -height_m`).
  - `predicted_floor(chain: UrdfChain, pan_rad: float, tilt_rad: float, origins: dict | None) -> tuple[np.ndarray, float]`: floor normal and height in `tof_optical_frame` for base_link z = 0 (floor = base_link z 0, `robot.urdf`), from `UrdfChain.transform("tof_optical_frame", {"head_pan": pan, "head_tilt": tilt}, origins)`.
  - `fit_plane(points: np.ndarray, prior: tuple[np.ndarray, float], cfg: FloorPlaneConfig, rng: np.random.Generator) -> PlaneFit | None`: finite points within `max_range_m` and within `candidate_band_m` of the prior plane are candidates; RANSAC over 3-point samples with `ransac_distance_m`, then an SVD least-squares refit on the inliers; None (never a made-up plane) when candidates or inliers are below `min_inliers` or the inlier ratio is below `min_inlier_ratio`.
  - `FloorObservation(pose: int, pan_rad: float, tilt_rad: float, plane: PlaneFit)`.
  - `fit_head_chain_floor(observations: list[FloorObservation], urdf_text: str, cfg: FloorFitConfig) -> FloorFitResult`: free parameters `head_mount_joint` z, roll, pitch and the `head_tilt` origin pitch (tilt zero); every other origin component of every joint stays at the value in `urdf_text` (the caller passes the repo URDF with the applied overrides). Residuals per observation: the two tangent components of `n_obs - n_pred` over `sigma_normal_deg` and `(h_obs - h_pred)` over `sigma_height_mm`; `scipy.optimize.least_squares(loss="huber", f_scale=huber_scale)` from the URDF values; sigma per parameter from `(J^T J)^-1 * s^2` (s^2 = robust cost * 2 / (m - n)).
  - `FloorFitResult`: `passed`, `failed_limits` (names of every limit not met, including too few observations or poses, pan span, RMS, sigma, delta from the URDF value), `params` `{mount_z_m, mount_roll_deg, mount_pitch_deg, tilt_zero_deg: {value, sigma, urdf}}`, `height_rms_mm`, `normal_rms_deg`, `poses` (per pose: pan_deg, tilt_deg, observations, height_residual_mm, normal_residual_deg), `origins: dict[str, Origin]` with the new `head_mount_joint` and `head_tilt` origins (all other components unchanged), and `to_json() -> dict` (plain floats and lists).
  - Too few observations returns `passed: false` naming the limit without calling the solver.
- New `tests/head_floor_synthetic.py`: `synthetic_clouds(urdf_text, truth_origins, grid, noise_m, outlier_ratio, rng)` builds floor points inside the ToF field of view (60 x 45 deg, 0.3-1.8 m), seen through a known perturbed chain, as point arrays in `tof_optical_frame`, plus wall points as outliers.
- New `tests/test_head_floor_fit.py` on `nodes/web_ui/urdf/head.urdf`:
  - `fit_plane` recovers a known plane within 2 mm and 0.1 deg with 5 mm noise and 20 % outliers; returns None for a cloud with too few floor points.
  - `predicted_floor` at pan 0, tilt 0 on the URDF gives height 0.35 m (0.31 + 0.04) and a normal along the optical -y axis (up).
  - recovery: truth = mount z +0.03 m, roll +1.5 deg, pitch -3.0 deg, tilt zero +2.0 deg, 15-pose grid (pan -60..60 step 30, tilt 12, 20, 28 deg down), 5 clouds per pose, 5 mm noise: each parameter within 2 mm or 0.15 deg, `passed` true, the fixed components (mount x, y, yaw, tilt xyz, tilt roll and yaw) equal the URDF values exactly.
  - degeneracy guard: a grid with pan 0 only fails `min_pan_span_deg` and does not report a pass.
  - limits: too few observations, a 0.2 m height bias (delta limit) and 30 mm noise (RMS) each give `passed: false` naming the limit.
- `tests/README.md` lists both new test files; `shared/README.md` names `floor_fit.py` and the config models.

## Acceptance conditions

- a1: fit_plane recovers a noisy plane with outliers and refuses a sparse cloud [evidence: test:tests/test_head_floor_fit.py::test_fit_plane_recovers_plane_and_refuses_sparse_cloud]
- a2: the predicted floor at zero angles matches the URDF height [evidence: test:tests/test_head_floor_fit.py::test_predicted_floor_at_zero_matches_urdf]
- a3: the fit recovers mount z, roll, pitch and tilt zero and keeps every fixed component [evidence: test:tests/test_head_floor_fit.py::test_fit_recovers_perturbed_chain]
- a4: a pan-0-only grid fails the pan span limit [evidence: test:tests/test_head_floor_fit.py::test_pan_span_limit_guards_degenerate_grid]
- a5: each limit failure is named and blocks a pass [evidence: test:tests/test_head_floor_fit.py::test_limits_refuse_bad_results]
- a6: fit_head_chain_floor exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/floor_fit.py::fit_head_chain_floor]
- a7: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/floor_fit.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `tests/head_floor_synthetic.py`
- `tests/test_head_floor_fit.py`
- `tests/README.md`
- `shared/README.md`

## Interfaces

- `floor_fit.py::PlaneFit`, `FloorObservation`, `FloorFitResult`, `predicted_floor`, `fit_plane`, `fit_head_chain_floor` (contract `head-floor-fit`).
- `config.py::FloorPlaneConfig`, `FloorFitLimits`, `FloorFitConfig`.
- Uses `chain.py::UrdfChain` and `urdf_text.py::Origin`.

## Expected outcome

Given ToF floor planes at measured pan/tilt, the core returns the fitted head_mount_joint and head_tilt origins with sigma, residuals and pass/fail; synthetic scenes recover a known chain.

## Steps

- Write `tests/head_floor_synthetic.py` and `tests/test_head_floor_fit.py` first; run `uv run --frozen pytest tests/test_head_floor_fit.py -q` and see it fail on the missing module.
- Implement the config models, `fit_plane`, `predicted_floor`, then `fit_head_chain_floor`.
- Run `uv run --frozen pytest tests/test_head_floor_fit.py tests/test_head_arm_calib_chain.py -q && uv run poe lint`; expect every test passed and lint clean.
- Update `tests/README.md` and `shared/README.md`.

## Notes

- Design D2, D3, D4. head_tilt + is down (rotation about +y); the floor normal in tof_optical_frame at pan = tilt = 0 points along optical -y.
- Mount pitch and tilt zero are the same rotation at pan 0; only pan spread separates them: keep the pan span limit.
- No ROS imports: the module is used by the node and by root tests.
