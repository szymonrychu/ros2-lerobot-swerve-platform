# Head chain floor-plane fit after the mount reprint implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Head chain override format

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/urdf_text.py`, `tests/test_head_chain_override.py`, `tests/README.md`, `shared/README.md`
- Verify: `uv run --frozen pytest tests/test_head_chain_override.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q && uv run poe lint`
- Verified head: 3d3d7e5768ca1bcc0f6e5d5316bbbccbc66343a1

#### Requirement

The `head_chain.json` override format and one applier used by every consumer (owns the `head-chain-override` contract, design D1 and D7; moved from head-and-arm-calibration-fro, whose later tasks use it unchanged).

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py` stays standard-library only (the head_state_publisher launch loads it by path):
  - `rewrite_joint_origin(urdf_text: str, joint: str, origin: Origin, note: str) -> str`: replaces the joint's `<origin>` xyz/rpy and a `<!-- <joint> calibrated: note -->` comment above it; all other bytes unchanged. `rewrite_tof_joint` becomes a call of it and gives byte-identical output to today (the `tof_joint calibrated` marker is unchanged), so `tests/test_head_rgbd_urdf_handoff.py` passes unchanged. A head_tilt rewrite keeps the joint's `<axis>` and `<limit>`.
  - Constants `HEAD_CHAIN_FILE = "head_chain.json"`, `TOF_JOINT_FILE = "tof_joint.json"`, `APPLIED_MARKER_FILE = "applied.json"`, `HEAD_CHAIN_JOINTS = ("head_mount_joint", "head_tilt", "head_camera_joint")`.
  - `parse_head_chain(data: object) -> dict[str, Origin]`: accepts `{"joints": {<name>: {"xyz": [3 numbers], "rpy": [3 numbers]}}, "note": str, ...}`; only names in HEAD_CHAIN_JOINTS, at least one joint, finite numbers, bool refused; ValueError naming the offending key otherwise.
  - `head_chain_json(origins: dict[str, Origin], note: str, session: str, result: str) -> str`: the writer (`joints`, `note`, `session`, `result`; keys sorted; 9 decimals).
  - `apply_overrides(urdf_text: str, applied_dir: Path) -> tuple[str, list[str]]`: applies `head_chain.json` (each joint) and then `tof_joint.json` (the existing `{xyz, rpy, note}`); a missing file is skipped silently; an unreadable or invalid file is skipped with one warning string naming the file and the reason; returns the text and the warnings.
- New `tests/test_head_chain_override.py`: writer/parser round trip; each chain joint rewritten with every other byte unchanged; tof override still applied after the chain; invalid files (bad JSON, unknown joint, 2-vector, NaN, bool, empty joints) each give exactly one warning and leave that file's joints unchanged; empty dir gives the identical text; the module imports only standard-library modules (checked with `ast` on the source).
- `tests/test_head_rgbd_urdf_handoff.py` and `tests/test_head_tof_joint_override.py` pass unchanged.
- `tests/README.md` lists `tests/test_head_chain_override.py` with one line per test; `shared/README.md` names the head_chain.json format and `apply_overrides`.

#### Acceptance conditions

- a1: head_chain.json round trips through the writer and parser [evidence: test:tests/test_head_chain_override.py::test_head_chain_json_round_trip]
- a2: apply_overrides applies the chain and then the tof joint [evidence: test:tests/test_head_chain_override.py::test_apply_overrides_chain_then_tof]
- a3: each invalid file gives one warning and changes nothing [evidence: test:tests/test_head_chain_override.py::test_invalid_override_files_warn_and_keep_urdf]
- a4: urdf_text stays standard-library only [evidence: test:tests/test_head_chain_override.py::test_urdf_text_imports_only_stdlib]
- a5: the existing tof_joint handoff tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `tests/test_head_chain_override.py`
- `tests/README.md`
- `shared/README.md`

#### Interfaces

- `urdf_text.py::rewrite_joint_origin`, `parse_head_chain`, `head_chain_json`, `apply_overrides`, `HEAD_CHAIN_FILE`, `TOF_JOINT_FILE`, `APPLIED_MARKER_FILE`, `HEAD_CHAIN_JOINTS` (contract `head-chain-override`).
- `head_chain.json`: `{joints: {head_mount_joint|head_tilt|head_camera_joint: {xyz: [3], rpy: [3]}}, note, session, result}`.

#### Expected outcome

One format and one applier for the calibrated head chain; the tof_joint rewrite is byte-identical to today.

#### Steps

- Write `tests/test_head_chain_override.py` first; run `uv run --frozen pytest tests/test_head_chain_override.py -q` and see it fail on the missing functions.
- Implement in `urdf_text.py` (standard library only), keep `rewrite_tof_joint` as a call of `rewrite_joint_origin`.
- Run `uv run --frozen pytest tests/test_head_chain_override.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q && uv run poe lint`; expect every test passed and lint clean.
- Update `tests/README.md` and `shared/README.md`.

#### Notes

- Moved unchanged from head-and-arm-calibration-fro (design D1); its later tasks (head-chain-board-solver, head-chain-compute-apply, mcp-head-kinematics-applied, web-ui-applied-head-urdf) use this API as written, so do not rename anything.
- Design: `ticket.py path head-chain-floor-plane-fit-a design` (D1, D7).

### S2: URDF chain FK core

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/chain.py`, `tests/test_head_arm_calib_chain.py`, `tests/README.md`, `shared/README.md`
- Verify: `uv run --frozen pytest tests/test_head_arm_calib_chain.py -q && uv run poe lint`
- Verified head: 2bb4a6d9cbecc18d43ed623ba52ad96a8336981b

#### Requirement

A numpy forward-kinematics chain over URDF text, shared by the head fit, the arm fit and the arm pose sequence (owns `urdf-chain-fk`, design D3; moved from head-and-arm-calibration-fro, whose head and arm fits use it unchanged).

- New `shared/ros2_common/head_rgbd_calibration/chain.py` (numpy and scipy `Rotation`; no ikpy, no ROS):
  - `UrdfChain.from_text(urdf_text: str) -> UrdfChain`: parses joints (`fixed`, `revolute`, `continuous`; any other type raises ValueError naming the joint), parents, origins and axes.
  - `UrdfChain.transform(link: str, positions: dict[str, float], origins: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] | None = None, offsets: dict[str, float] | None = None) -> np.ndarray`: 4x4 pose of `link` in the URDF root. Movable joint angle = `positions[name] + offsets.get(name, 0.0)` (urdf_angle = measured_angle + offset, as `nodes/mcp_server/mcp_server/ik.py::ArmKinematics`). `origins` replaces a joint's URDF origin (xyz, rpy) and nothing else. A movable joint on the path missing from `positions` raises KeyError naming it (never 0). Unknown link raises KeyError.
  - `UrdfChain.joint_names(link) -> list[str]` (root to link), `UrdfChain.origin(joint) -> tuple[xyz, rpy]`, `UrdfChain.limits(joint) -> tuple[float, float] | None`.
  - `pose_from_xyz_rpy(xyz, rpy) -> np.ndarray`, `xyz_rpy_from_pose(T) -> tuple[xyz, rpy]` (fixed-axis xyz, the URDF convention, `urdf_handoff.RPY_SEQUENCE`).
  - `arm_in_robot_base(base: Mapping[str, float]) -> np.ndarray` (keys `x`, `y`, `z`, `yaw`; a pydantic model's `model_dump()` fits): transform from the arm URDF root (`base_link` of so101_arm.urdf) to robot base_link (x, y, z, yaw).
- New `tests/test_head_arm_calib_chain.py` on the repo URDFs (`nodes/web_ui/urdf/head.urdf`, `nodes/web_ui/urdf/so101_arm.urdf`):
  - head camera optical frame at pan = tilt = 0 equals the hand-computed product of the URDF origins (mount 0.15 0.10 0.31, tilt 0.04, camera 0.02 -0.025 0, optical rpy -1.5708 0 -1.5708);
  - head_pan +0.5 rotates the camera about +z (CCW from above); head_tilt + pitches down (the optical z axis gains a negative base_link z component);
  - an origin override changes exactly that joint; offsets add to measured; a missing joint raises KeyError naming it;
  - so101 gripper_link at zero joints equals the product of origins; xyz/rpy round trip within 1e-12;
  - `arm_in_robot_base` with the default gauge maps the arm root to (0.0592, -0.05, 0.100).
- `tests/README.md` lists `tests/test_head_arm_calib_chain.py`; `shared/README.md` names `chain.py`.

#### Acceptance conditions

- a1: the head camera pose at zero equals the URDF origin product [evidence: test:tests/test_head_arm_calib_chain.py::test_head_camera_pose_at_zero_matches_urdf_origins]
- a2: pan and tilt signs follow the URDF conventions [evidence: test:tests/test_head_arm_calib_chain.py::test_pan_left_and_tilt_down_signs]
- a3: overrides replace one joint origin and offsets add to measured angles [evidence: test:tests/test_head_arm_calib_chain.py::test_origin_override_and_offsets]
- a4: a missing joint position raises naming the joint [evidence: test:tests/test_head_arm_calib_chain.py::test_missing_joint_raises]
- a5: the arm root maps into robot base_link by the gauge [evidence: test:tests/test_head_arm_calib_chain.py::test_arm_in_robot_base_default_gauge]
- a6: UrdfChain exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/chain.py::UrdfChain]
- a7: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/chain.py`
- `tests/test_head_arm_calib_chain.py`
- `tests/README.md`
- `shared/README.md`

#### Interfaces

- `chain.py::UrdfChain.from_text`, `UrdfChain.transform`, `UrdfChain.joint_names`, `UrdfChain.origin`, `UrdfChain.limits`, `pose_from_xyz_rpy`, `xyz_rpy_from_pose`, `arm_in_robot_base` (contract `urdf-chain-fk`).

#### Expected outcome

A numpy FK over the repo head and arm URDFs, pinned by tests on the real URDF files.

#### Steps

- Write `tests/test_head_arm_calib_chain.py` first; run `uv run --frozen pytest tests/test_head_arm_calib_chain.py -q` and see it fail on the missing module.
- Implement `chain.py` (numpy and scipy Rotation, fixed-axis xyz rpy as `urdf_handoff.RPY_SEQUENCE`).
- Run `uv run --frozen pytest tests/test_head_arm_calib_chain.py -q && uv run poe lint`; expect every test passed and lint clean.

#### Notes

- Moved unchanged from head-and-arm-calibration-fro (design D1); its head and arm fits and the arm pose sequence use it as written.
- Mimic `nodes/mcp_server/mcp_server/ik.py::ArmKinematics` for urdf_angle = measured + offset.

### S3: Floor plane chain fit

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/floor_fit.py`, `shared/ros2_common/head_rgbd_calibration/config.py`, `tests/head_floor_synthetic.py`, `tests/test_head_floor_fit.py`, `tests/README.md`, `shared/README.md`
- Verify: `uv run --frozen pytest tests/test_head_floor_fit.py tests/test_head_arm_calib_chain.py -q && uv run poe lint`
- Verified head: 1346b4feacad33ba33989b1a6681fa7cd3919990

#### Requirement

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

#### Acceptance conditions

- a1: fit_plane recovers a noisy plane with outliers and refuses a sparse cloud [evidence: test:tests/test_head_floor_fit.py::test_fit_plane_recovers_plane_and_refuses_sparse_cloud]
- a2: the predicted floor at zero angles matches the URDF height [evidence: test:tests/test_head_floor_fit.py::test_predicted_floor_at_zero_matches_urdf]
- a3: the fit recovers mount z, roll, pitch and tilt zero and keeps every fixed component [evidence: test:tests/test_head_floor_fit.py::test_fit_recovers_perturbed_chain]
- a4: a pan-0-only grid fails the pan span limit [evidence: test:tests/test_head_floor_fit.py::test_pan_span_limit_guards_degenerate_grid]
- a5: each limit failure is named and blocks a pass [evidence: test:tests/test_head_floor_fit.py::test_limits_refuse_bad_results]
- a6: fit_head_chain_floor exists [evidence: symbol:shared/ros2_common/head_rgbd_calibration/floor_fit.py::fit_head_chain_floor]
- a7: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/floor_fit.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `tests/head_floor_synthetic.py`
- `tests/test_head_floor_fit.py`
- `tests/README.md`
- `shared/README.md`

#### Interfaces

- `floor_fit.py::PlaneFit`, `FloorObservation`, `FloorFitResult`, `predicted_floor`, `fit_plane`, `fit_head_chain_floor` (contract `head-floor-fit`).
- `config.py::FloorPlaneConfig`, `FloorFitLimits`, `FloorFitConfig`.
- Uses `chain.py::UrdfChain` and `urdf_text.py::Origin`.

#### Expected outcome

Given ToF floor planes at measured pan/tilt, the core returns the fitted head_mount_joint and head_tilt origins with sigma, residuals and pass/fail; synthetic scenes recover a known chain.

#### Steps

- Write `tests/head_floor_synthetic.py` and `tests/test_head_floor_fit.py` first; run `uv run --frozen pytest tests/test_head_floor_fit.py -q` and see it fail on the missing module.
- Implement the config models, `fit_plane`, `predicted_floor`, then `fit_head_chain_floor`.
- Run `uv run --frozen pytest tests/test_head_floor_fit.py tests/test_head_arm_calib_chain.py -q && uv run poe lint`; expect every test passed and lint clean.
- Update `tests/README.md` and `shared/README.md`.

#### Notes

- Design D2, D3, D4. head_tilt + is down (rotation about +y); the floor normal in tof_optical_frame at pan = tilt = 0 points along optical -y.
- Mount pitch and tilt zero are the same rotation at pan 0; only pan spread separates them: keep the pan span limit.
- No ROS imports: the module is used by the node and by root tests.

### S4: Head state publisher chain

- Status: done
- Files: `nodes/head_state_publisher/launch/head_description.py`, `nodes/head_state_publisher/launch/head_state_publisher.launch.py`, `tests/test_head_tof_joint_override.py`, `tests/test_head_chain_launch.py`, `tests/README.md`, `nodes/head_state_publisher/README.md`
- Verify: `uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_chain_launch.py tests/test_head_state_publisher_config.py -q && uv run poe lint`
- Verified head: 5502ba9a7bd47b1abfde290fde639c73480d7e29

#### Requirement

head_state_publisher publishes TF from the applied head chain as well as the applied tof_joint (design D7 and D8; moved from head-and-arm-calibration-fro).

- `nodes/head_state_publisher/launch/head_description.py` (standard library only, runs on the system Python): `robot_description(urdf_path: Path, applied_dir: Path, urdf_text_path: Path) -> str` loads `urdf_text.py` by path and returns `apply_overrides(urdf, applied_dir)` text; each warning is logged with `LOGGER.warning`; a urdf_text module that fails to load logs a warning and returns the URDF unchanged.
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`: `APPLIED_DIR = Path("/var/lib/ros2/head_rgbd_calibration/applied")` replaces `TOF_JOINT_OVERRIDE`; the docstring names both override files. The apply path unit already restarts head_state_publisher on `applied.json` changes (no Ansible change).
- `tests/test_head_tof_joint_override.py` follows the new signature with the same behaviours (tof override applied, unusable override ignored with a warning).
- New `tests/test_head_chain_launch.py`: a `head_chain.json` in a tmp applied dir changes `head_mount_joint`, `head_tilt` and `head_camera_joint` origins in the description; chain and tof overrides together; an invalid chain file logs a warning, keeps the URDF chain and still applies the tof override; `generate_launch_description()` with the constant monkeypatched to the tmp dir passes the overridden text as `robot_description`.
- `tests/test_head_state_publisher_config.py` passes.
- `tests/README.md` lists `tests/test_head_chain_launch.py` and the changed `tests/test_head_tof_joint_override.py`; `nodes/head_state_publisher/README.md` names both override files in `applied/`.

#### Acceptance conditions

- a1: the applied head chain reaches robot_description [evidence: test:tests/test_head_chain_launch.py::test_head_chain_override_reaches_robot_description]
- a2: an invalid chain file warns and keeps the tof override [evidence: test:tests/test_head_chain_launch.py::test_invalid_chain_warns_and_keeps_tof_override]
- a3: the launch passes the overridden text [evidence: test:tests/test_head_chain_launch.py::test_launch_uses_applied_dir]
- a4: tof override and launch config tests pass [evidence: command:uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_state_publisher_config.py -q]
- a5: root lint passes [evidence: command:uv run poe lint]

#### Files

- `nodes/head_state_publisher/launch/head_description.py`
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `tests/test_head_tof_joint_override.py`
- `tests/test_head_chain_launch.py`
- `tests/README.md`
- `nodes/head_state_publisher/README.md`

#### Interfaces

- `head_description.py::robot_description(urdf_path: Path, applied_dir: Path, urdf_text_path: Path) -> str`; launch constant `APPLIED_DIR`.

#### Expected outcome

After an apply and the path unit restart, `/tf` from head_state_publisher carries the fitted head_mount_joint and head_tilt origins and the applied tof_joint.

#### Steps

- Write `tests/test_head_chain_launch.py` and adapt `tests/test_head_tof_joint_override.py` first; run them and see them fail on the old signature.
- Change `head_description.py` and the launch file.
- Run `uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_chain_launch.py tests/test_head_state_publisher_config.py -q && uv run poe lint`; expect every test passed and lint clean.

#### Notes

- Moved from head-and-arm-calibration-fro (design D1, D8). The apply path unit (`ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.path.j2`, PathChanged on applied.json) already restarts head_state_publisher; no Ansible change.
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py` is already deployed with head_state_publisher (client.yml src list).

### S5: Floor sweep capture

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `nodes/head_rgbd_calibration/tests/conftest.py`, `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`, `nodes/head_rgbd_calibration/tests/test_config.py`, `ansible/group_vars/client.yml`, `nodes/head_rgbd_calibration/README.md`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_config.py -q && uv run poe lint && cd ../.. && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint-ansible`
- Verified head: e3bc611a

#### Requirement

The guided head sweep that captures floor planes (owns `floor-sweep`; design D5, D11). Nothing is published except head commands; no placeholder data: a pose without a valid plane is skipped with its reason, never filled.

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` and the shipped `config/head_rgbd_calibration.yaml` (every key at its default with a one-line comment): `floor: FloorSweepConfig` with `joint_states_topic` `/head/joint_states`, `command_topic` `/head/joint_commands`, `cloud_topic` `/tof/points`, `command_frame_id` `head_rgbd_calibration`, `pan_deg` [-60, -30, 0, 30, 60], `tilt_deg` [12, 20, 28] (+ is down; each value validated in -30..30), `reach_tol_deg` 1.0, `still_tol_deg` 0.2, `still_window_s` 0.5, `move_timeout_s` 6.0, `settle_s` 0.5, `clouds_per_pose` 5 (>= 1), `cloud_timeout_s` 3.0, `max_stamp_dt_s` 0.1, `fit: FloorFitConfig` (from the shared core).
- `ansible/group_vars/client.yml`: the `head_rgbd_calibration` ros2_nodes `config` block equals the shipped YAML (`tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults` passes); the `head_rgbd_calibration` node-type defaults gain `apt_packages: [ros-jazzy-sensor-msgs-py]` (point_cloud2 reader, as laser_filter).
- New `head_rgbd_calibration/floor_sweep.py`: `FloorSweep(node, cfg: NodeConfig, urdf_text: Callable[[], str], clock, sleep)`; one sweep at a time in a worker thread; `Busy` while one runs.
  - `start() -> str`: new session id `floor-YYYYMMDD-HHMMSS` under `<state_dir>/floor_fit/<sid>/`; creates the `/head/joint_states` and `/tof/points` subscriptions and the command publisher (all destroyed on every exit); refuses with a reason when no head joint state arrives within `move_timeout_s` (nothing moved).
  - Grid order is serpentine (pan sweeps alternate direction per tilt row). Per pose: publish a `sensor_msgs/JointState` (`name` [head_pan, head_tilt], radians, `header.frame_id` = `command_frame_id`); wait until both joints are within `reach_tol_deg` of the target and still (spread under `still_tol_deg` over `still_window_s`), else skip the pose with `did not reach pan/tilt` after `move_timeout_s`; wait `settle_s`; take the next `clouds_per_pose` clouds stamped after the still time. Each cloud's pan/tilt is the joint state nearest its header stamp within `max_stamp_dt_s` (else that cloud is dropped with a counted reason); points come from `sensor_msgs_py.point_cloud2.read_points_numpy(..., skip_nans=True)`; the plane is `floor_fit.fit_plane` with the prior from `predicted_floor` on `urdf_text()`. A pose with no valid plane is skipped with the reason.
  - Every exit (done, `stop()`, error, exception) commands the head back to the pan/tilt it had at start, then destroys the ROS handles.
  - Writes `observations.json` (`session`, `created`, `grid`, `observations` [{pose, pan_rad, tilt_rad, normal, height_m, inliers, candidates, rms_m, stamp_ns}], `skipped` [{pose, pan_deg, tilt_deg, reason}]) atomically when the sweep ends, also after a stop (the poses done so far).
  - `state() -> {running, session_id, pose, total, observations, skipped, status, error}`.
- `tests/conftest.py` stubs `sensor_msgs.msg.JointState`, `sensor_msgs.msg.PointCloud2` and `sensor_msgs_py.point_cloud2` when not importable.
- New `tests/test_floor_sweep.py` (fake node, fake clock and sleep, synthetic clouds): command order and frame_id per pose; serpentine order; unreachable pose skipped with reason; cloud with no joint state within `max_stamp_dt_s` dropped; restore to the start pan/tilt on done, stop, error and exception; handles destroyed on every exit; observations.json content; Busy; no head joint state -> refused and nothing commanded.
- `tests/test_config.py`: the floor defaults and the tilt range check.
- `nodes/head_rgbd_calibration/README.md`: a "Head floor fit" section: the sweep, the `floor` config keys, the files in `floor_fit/<sid>/` and `applied/`, the limits, and the HTTP routes of the design's Floor fit API table (built by floor-fit-http-api in this epic; the epic merges as one).

#### Acceptance conditions

- a1: each pose is commanded in serpentine order with the calibration frame_id [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_commands_serpentine_grid_with_frame_id]
- a2: an unreachable pose and an unmatched cloud are skipped with their reasons [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_skips_unreachable_pose_and_unmatched_cloud]
- a3: the head returns to its start pose and handles are destroyed on every exit [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_restores_start_pose_on_every_exit]
- a4: observations.json holds the measured planes and skipped poses [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_writes_observations_json]
- a5: no head joint state refuses the start and commands nothing [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_refuses_without_joint_state]
- a6: floor config defaults and tilt range check [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_floor_defaults_and_tilt_range]
- a7: client.yml config block equals the shipped YAML [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q]
- a8: node lint and ansible lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint && cd ../.. && uv run poe lint-ansible]
- a9: the node README documents the floor fit [evidence: doc-section:nodes/head_rgbd_calibration/README.md#Head floor fit]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/conftest.py`
- `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/README.md`

#### Interfaces

- `floor_sweep.py::FloorSweep(node, cfg, urdf_text, clock, sleep)` with `start() -> str`, `stop()`, `state() -> dict`, `Busy` (contract `floor-sweep`).
- `observations.json` in `<state_dir>/floor_fit/<sid>/` as in the requirement.
- `config.py::FloorSweepConfig`, `NodeConfig.floor`.

#### Expected outcome

Start moves only the head through the grid, records one plane per matched cloud, writes observations.json and puts the head back where it was.

#### Steps

- Extend `tests/conftest.py` stubs, write `tests/test_floor_sweep.py` and the config test first; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_sweep.py tests/test_config.py -q` and see them fail.
- Implement `config.py`, the shipped YAML, then `floor_sweep.py` (pattern of `capture.py`: ROS handles created on start and destroyed on every exit, a lock for shared state).
- Mirror the YAML into the client.yml `head_rgbd_calibration` config block and add the apt package.
- Run the task verify command; expect every test passed, node lint and ansible lint clean.
- Write the README section.

#### Ruling

- 2026-10-21 accepted: workflow verifier was invoked with --prev-head placeholder (harness bug); orchestrator reran verify-task.py on e3bc611a: passed, 42 tests, lint, deploy tests, lint-ansible, no debt/parking; remaining reviewer findings at round cap were declined as false positives (self.skip method, conftest stub)

#### Notes

- Design D5, D6, D11. Head commands as the web UI head tab (`nodes/web_ui/web_ui/config.py` HEAD_TAB_DEFAULT_COMMAND_TOPIC, joint names head_pan/head_tilt, radians).
- client.yml is also edited by client-cpu-budget-recovery-a (this epic's blocker): rebase on it before editing.
- Servo safety: head only, the grid stays inside -60..60 pan and 12..28 tilt down.

### S6: Floor fit HTTP API

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_api.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`, `nodes/head_rgbd_calibration/tests/test_floor_results.py`, `nodes/head_rgbd_calibration/tests/test_floor_api.py`, `nodes/head_rgbd_calibration/tests/test_main.py`, `nodes/head_rgbd_calibration/tests/fixtures/api/floor_state.json`, `nodes/head_rgbd_calibration/tests/fixtures/api/floor_result.json`
- Verify: `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`
- Verified head: 5f48504182c0574e26acf06648da62477864809f

#### Requirement

Compute, apply and the HTTP routes of the head floor fit (owns `head-floor-fit-api` and `head-chain-applied-record`; design D6, D7, D8). Same error body `{ok: false, message}` and status mapping as `app.py`.

- New `head_rgbd_calibration/floor_results.py`:
  - `base_urdf(cfg) -> str`: `cfg.head_urdf_file` text with `urdf_text.apply_overrides(text, <state_dir>/applied)` (the applied tof_joint and any applied head chain), each warning logged.
  - `compute_floor(cfg, sid, now) -> dict`: reads `floor_fit/<sid>/observations.json`, runs `fit_head_chain_floor(observations, base_urdf(cfg), cfg.floor.fit)`, writes `floor_fit/<sid>/results/<rid>/result.json` (`{id, session, created, passed, failed_limits, params, height_rms_mm, normal_rms_deg, poses, skipped}`) and, only when passed, `head_chain.json` from `head_chain_json` with every joint of the currently applied head chain plus the new `head_mount_joint` and `head_tilt` origins (note: date, height RMS, normal RMS). Synchronous (milliseconds; no child process).
  - `apply_floor(cfg, sid, rid, now, run) -> dict`: refuses (NotApplicable, 409) a result without `passed: true` or when `applied/applied.json` is absent (no RGB-D calibration applied, so the ToF pose the fit assumed is not the deployed one); copies `head_chain.json` into `applied/` (temp file + `os.replace`), then rewrites `applied/applied.json` atomically with every existing key unchanged plus `head_chain: {session, result, applied_at}` (the change starts the apply path unit, which restarts head_state_publisher); returns `{head_chain: {...}, units}` with `apply.unit_state` per `cfg.restart_units` against the head chain `applied_at`.
  - `applied_head_chain(cfg) -> dict | None`: `{session, result, note, joints}` read from `applied/head_chain.json` itself (so an RGB-D apply that rewrites applied.json does not hide it); None when absent or invalid.
- New `head_rgbd_calibration/floor_api.py`: `floor_router(sweep, capture, job, cfg, run) -> APIRouter`:
  - `GET /api/floor/state` -> `{sweep: sweep.state(), applied: applied_head_chain(cfg)}`.
  - `POST /api/floor/sweep/start` -> state; 409 while an RGB-D capture, a compute or a sweep runs.
  - `POST /api/floor/sweep/stop` -> state (also when idle).
  - `GET /api/floor/sessions` -> `[{id, created, observations, skipped, results}]`.
  - `POST /api/floor/sessions/{sid}/compute` -> the result body; 409 while a sweep runs; 404 for an unknown or malformed id (`store.InvalidId` rules for ids).
  - `GET /api/floor/sessions/{sid}/results/{rid}` -> result.json; `POST .../apply` -> the apply body.
- `__main__.py`: builds `FloorSweep` (urdf_text = `base_urdf(cfg)`), `app.include_router(floor_router(...))` after `build_app`; shutdown stops the sweep first (head restored), then the capture, the job and the node.
- Fixtures `tests/fixtures/api/floor_state.json` and `tests/fixtures/api/floor_result.json` (checked with `tests/contract.py` like the others; the web UI tests import them).
- New `tests/test_floor_results.py` (synthetic observations from `tests/head_floor_synthetic.py` loaded by path, tmp state dir): a passing compute writes result.json and head_chain.json with the fitted joints and the previously applied joints kept; a failing compute writes no head_chain.json; apply copies the file, keeps every applied.json key and adds `head_chain`; apply refused for a failed result and without applied.json; `applied_head_chain` survives an applied.json rewrite without the key.
- New `tests/test_floor_api.py` (fake sweep, capture and job): every route body matches its fixture; 409 cases; ids validated. `tests/test_main.py`: shutdown order with the sweep first.
- The whole node suite passes.

#### Acceptance conditions

- a1: a passing compute writes result.json and head_chain.json keeping the applied joints [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_compute_writes_result_and_head_chain]
- a2: a failing compute writes no head_chain.json [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_failed_compute_writes_no_head_chain]
- a3: apply copies head_chain.json and adds the head_chain record keeping every applied.json key [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_apply_keeps_applied_keys_and_adds_record]
- a4: apply is refused for a failed result and without an applied RGB-D calibration [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_results.py::test_apply_refusals]
- a5: route bodies match the floor fixtures and busy cases answer 409 [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_api.py::test_routes_match_fixtures_and_conflicts]
- a6: shutdown stops the sweep first [evidence: test:nodes/head_rgbd_calibration/tests/test_main.py::test_shutdown_stops_floor_sweep_first]
- a7: the node suite and lint pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_api.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_floor_results.py`
- `nodes/head_rgbd_calibration/tests/test_floor_api.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api/floor_state.json`
- `nodes/head_rgbd_calibration/tests/fixtures/api/floor_result.json`

#### Interfaces

- `floor_results.py::base_urdf`, `compute_floor`, `apply_floor`, `applied_head_chain` (contract `head-chain-applied-record`).
- `floor_api.py::floor_router(sweep, capture, job, cfg, run) -> APIRouter`; the routes of the design's Floor fit API table; fixtures `floor_state.json`, `floor_result.json` (contract `head-floor-fit-api`).

#### Expected outcome

The Calibrate page can start a sweep, compute a result and apply a passing one; after Apply, `applied/head_chain.json` exists and applied.json changed, so head_state_publisher restarts on the fitted chain.

#### Steps

- Write `tests/test_floor_results.py`, `tests/test_floor_api.py`, the fixtures and the shutdown test first; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_floor_results.py tests/test_floor_api.py tests/test_main.py -q` and see them fail.
- Implement `floor_results.py`, `floor_api.py`, then the `__main__.py` wiring.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run poe lint`; expect every test passed and lint clean.

#### Notes

- Design D6, D7. Do not edit `app.py`, `store.py` or `compute.py` (in-progress per-pose-quality-hints-in-th); the router is included in `__main__.py`.
- Reuse `apply.unit_state` and `store.write_json_atomic`; load `tests/head_floor_synthetic.py` by path as `tests/pose_factory.py` loads `tests/rgbd_synthetic.py`.

- 2026-10-21 run head-chain-floor-plane-fit-a: dependency-failed on floor-sweep-capture

### S7: Head floor fit UI step

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`, `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`, `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`, `nodes/web_ui/README.md`, `docs/runbooks/head-floor-fit.md`
- Verify: `cd nodes/web_ui/frontend && npx vitest run src/calibrate/headFloorApi.test.ts src/calibrate/HeadFloorFit.test.tsx src/calibrate/CalibratePage.test.tsx && npm run lint`
- Verified head: bf96c27d48bad3e4dad792c4fcf35a110dd117ca

#### Requirement

The "Head floor fit" step on the web UI Calibrate page and the robot runbook (uses `head-floor-fit-api`; design D9, D10). The web UI proxy `/api/calibration/rgbd/{path}` already forwards `floor/...` to the node; no backend change.

- New `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`: typed calls under `/api/calibration/rgbd/floor` (`getFloorState`, `startSweep`, `stopSweep`, `computeFloor(sid)`, `getFloorResult(sid, rid)`, `applyFloor(sid, rid)`) with runtime type guards in the style of `rgbdApi.ts`; a body that fails its guard is an error, never shown as data.
- New `HeadFloorFit.tsx`: a step card: the precondition text (robot on a flat floor, not on blocks; about 2 m in front and 1.5 m to each side clear; only the head moves); Start sweep; progress (pose i of N, observations, skipped poses with reasons) polled while running; Stop; Compute when a sweep has ended; the result: a table of mount z (mm), mount roll, mount pitch, tilt zero (deg) with current, fitted, delta and sigma, height and normal RMS, per-pose residuals, pass/fail with the failed limits; Apply enabled only for a passed result, then the unit restart state; the currently applied head chain (session, result, note).
- `CalibratePage.tsx`: renders `<HeadFloorFit />` after the RGB-D section when `config.rgbd` is set.
- Tests (vitest): `headFloorApi.test.ts` (guards accept `floor_state.json` and `floor_result.json` from `nodes/head_rgbd_calibration/tests/fixtures/api/`, reject a malformed body); `HeadFloorFit.test.tsx` (start, progress, compute, the table values, Apply disabled for a failed result and enabled for a passed one, errors shown); `CalibratePage.test.tsx` (the step shows with rgbd configured and not without).
- `nodes/web_ui/README.md`: the Calibrate page section names the Head floor fit step.
- New `docs/runbooks/head-floor-fit.md`: preconditions (flat floor, robot not on blocks, area clear, no agent session if mcp_server is deployed), deploy via `scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher web_ui` with output in `.logs/` (invoke the `ansible-deploy` skill first), the UI steps, and the acceptance checks with `scripts/topic_scraper_collect.py` on `/ground/state` at home and at two other pan/tilt poses: plane height 0 +- 0.01 m, plane_vs_base < 0.6 deg, fewer than 10 obstacle points on the flat floor; a results table filled when the orchestrator runs it with the user after merge.

#### Acceptance conditions

- a1: the API guards accept the node fixtures and reject malformed bodies [evidence: test:nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts::accepts the floor fixtures and rejects malformed bodies]
- a2: the step runs start, progress, compute and apply with Apply only for a passed result [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx]
- a3: the Calibrate page shows the step only with rgbd configured [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx]
- a4: the runbook holds the acceptance checks [evidence: doc-section:docs/runbooks/head-floor-fit.md#Acceptance checks]
- a5: the web UI README names the step [evidence: judgement:]
- a6: frontend lint passes [evidence: command:cd nodes/web_ui/frontend && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/README.md`
- `docs/runbooks/head-floor-fit.md`

#### Interfaces

- `headFloorApi.ts::getFloorState`, `startSweep`, `stopSweep`, `computeFloor`, `getFloorResult`, `applyFloor` and their guards; `HeadFloorFit` component.
- Node bodies: the `head-floor-fit-api` fixtures in `nodes/head_rgbd_calibration/tests/fixtures/api/`.

#### Expected outcome

The Calibrate page shows a Head floor fit step that runs the sweep, shows the fit and applies it; the runbook gives the robot acceptance checks.

#### Steps

- Write `headFloorApi.test.ts`, `HeadFloorFit.test.tsx` and the CalibratePage test first; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate/headFloorApi.test.ts src/calibrate/HeadFloorFit.test.tsx src/calibrate/CalibratePage.test.tsx` and see them fail.
- Implement the API module, the component and the page line.
- Run the task verify command; expect every test passed and eslint clean.
- Write `docs/runbooks/head-floor-fit.md` (with an `## Acceptance checks` heading) and the web UI README line.

#### Notes

- Design D9, D10. Do not edit `rgbdApi.ts` or `RgbdCapture.tsx` (in-progress per-pose epic); copy the guard helpers' style.
- The runbook is executed by the orchestrator with the user after merge; deploy only through `scripts/deploy-nodes.sh` with output in `.logs/`; mcp_server is not deployed by this epic.
- Telemetry through `scripts/topic_scraper_collect.py` (CLAUDE.md), topic_scraper_api enabled first.

- 2026-10-21 run head-chain-floor-plane-fit-a: dependency-failed on floor-fit-http-api

### S8: Floor fit pose-mean limits, partial pose skip and tilt roll

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/floor_fit.py`, `shared/ros2_common/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `tests/test_head_floor_fit.py`, `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`, `docs/runbooks/head-floor-fit.md`, `tests/README.md`
- Verify: `uv run pytest tests/ -q -k 'floor or head_rgbd or calib' && uv run poe lint && cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 60246c9e

#### Requirement

Floor fit robustness after the first robot sweep (2026-10-22 00:09, session floor-20261022-000848, result r-20261022-000946 refused: height RMS 9.9 mm > 8, normal RMS 1.45 deg > 0.6). Offline analysis (scratchpad/scripts/ext.py, q2.py, grid.py; scratchpad/floorfit/compute1.json): the same pose gives plane normals 1.15 deg apart cloud to cloud (structured per-cloud noise), so per-observation RMS can never pass 0.6 deg; two poses were contaminated by objects in view (pose 3: 1 cloud, -36 mm, 2.6 deg; pose 12: 2 clouds, +14 mm, 2.5 deg); with them removed, freeing head_tilt roll gives 1.16 +- 0.27 deg (significant with the applied tof_joint rpy ~[0, 2.08, -5.3] deg) and pose-mean normal ~0.66 deg, height ~7.8 mm.
Changes (shared/ros2_common/head_rgbd_calibration floor_fit.py / config.py / chain handling, nodes/head_rgbd_calibration floor_sweep.py and node yaml defaults; NOT ansible/group_vars/client.yml, which another in-progress epic is editing):
1. The limits max_normal_rms_deg and max_height_rms_mm apply to POSE-MEAN residuals (mean of a pose's observations), not per observation; report both per-pose-mean and per-observation RMS in the result and report.
2. New sweep key min_clouds_per_pose (default: half of clouds_per_pose, rounded up): a pose with fewer fitted clouds is skipped with reason "partly blocked: k of n clouds".
3. clouds_per_pose default 5 -> 10 in the code default and the node yaml (the client.yml value is changed separately by the orchestrator after the other epic merges; the code must work with either).
4. Free head_tilt roll as a fitted parameter (chain_origins; head_tilt is already an override joint, the RGB-ToF extrinsic stays rigid); include it in params with value/sigma/urdf and in the applied head_chain.json; sigma limit max_sigma_deg applies.
Tests first: synthetic chain with a known tilt roll recovered within sigma; pose-mean limits pass when per-cloud normals scatter 1.2 deg but pose means agree; a pose with k < min clouds is skipped; existing tests stay green. Update READMEs (head_rgbd_calibration, shared) and tests/README.md, and docs/runbooks/head-floor-fit.md (clear a ~1.8 m half-disc in front before the sweep; fallback grid pan [-30, 30, 60] x tilt [20, 24, 28] if the floor cannot be cleared).

#### Acceptance conditions

- a1: the height and normal limits are evaluated on pose-mean residuals [evidence: test:tests/test_head_floor_fit.py::test_limits_use_pose_mean_residuals]
- a2: a pose with fewer than min_clouds_per_pose fitted clouds is skipped as partly blocked [evidence: test:nodes/head_rgbd_calibration/tests/test_floor_sweep.py::test_partly_blocked_pose_is_skipped]
- a3: head_tilt roll is fitted and recovered on a synthetic chain [evidence: test:tests/test_head_floor_fit.py::test_tilt_roll_is_recovered]
- a4: clouds_per_pose defaults to 10 in code and node yaml [evidence: test:tests/test_head_floor_fit.py::test_floor_sweep_defaults]

#### Files

- `shared/ros2_common/head_rgbd_calibration/floor_fit.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_sweep.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `tests/test_head_floor_fit.py`
- `nodes/head_rgbd_calibration/tests/test_floor_sweep.py`
- `docs/runbooks/head-floor-fit.md`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

A re-sweep on a cleared floor passes the limits (pose-mean normal ~0.55 deg, height ~4 mm expected) and Apply puts the home floor at 0 +- 1 cm.

What is true and observable when the task is done.

#### Steps

1. Write the failing tests (pose-mean limits, partly blocked pose, tilt roll recovery, defaults).
2. Implement in floor_fit.py / config.py / floor_sweep.py and the node yaml.
3. Update the READMEs and the runbook.
4. Run the verify command and commit.

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S9: Head floor fit UI shows tilt roll and pose-mean RMS

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`, `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`, `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`, `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint`
- Verified head: f84a572d5a9a4943a3c05c3f8c78245bd3375290

#### Requirement

The floor-fit result now carries params.tilt_roll_deg (value, sigma, urdf) and both pose-mean and per-observation RMS (task floor-fit-pose-mean-limits-p, shared/ros2_common/head_rgbd_calibration/floor_fit.py and the node result.json; read the real field names there). The web UI Head floor fit step (nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx, headFloorApi.ts) must show the tilt roll row in the parameter table (current, fitted, delta, sigma like the other rows), show the pose-mean RMS that the limits use next to the per-observation RMS (labelled so the user sees which one is checked), and show skipped poses with their reason (including "partly blocked: k of n clouds"). FloorResult type and its runtime guard gain the new keys (older results without them still render). Tests first in HeadFloorFit.test.tsx / headFloorApi.test.ts. Update nodes/web_ui/README.md (Head floor fit section) if it lists the table rows.

#### Acceptance conditions

- a1: the parameter table shows the head_tilt roll row with current, fitted, delta and sigma [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx -t "shows the tilt roll row"]
- a2: the pose-mean RMS checked by the limits and the per-observation RMS are both shown and labelled [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx -t "labels pose-mean and per-observation RMS"]
- a3: results without the new keys still render [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/headFloorApi.test.ts -t "accepts results without the new keys"]
- a4: frontend type check, tests and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`
- `nodes/web_ui/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

The Head floor fit step lists tilt roll with the other parameters and shows which RMS the limits check.

What is true and observable when the task is done.

#### Steps

1. Write the failing tests.
2. Extend the FloorResult type, guard and the table.
3. Run the verify command, update the README, commit.

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none

## Rulings (migrated)

- floor-sweep-capture: accept true, at 2026-10-21T21:32:44Z: workflow verifier was invoked with --prev-head placeholder (harness bug); orchestrator reran verify-task.py on e3bc611a: passed, 42 tests, lint, deploy tests, lint-ansible, no debt/parking; remaining reviewer findings at round cap were declined as false positives (self.skip method, conftest stub)
