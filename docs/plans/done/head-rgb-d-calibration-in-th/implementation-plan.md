# Head RGB-D calibration in the web UI Calibrate page implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Calib core config charuco

- Status: done
- Files: `pyproject.toml`, `uv.lock`, `shared/ros2_common/head_rgbd_calibration/__init__.py`, `shared/ros2_common/head_rgbd_calibration/config.py`, `shared/ros2_common/head_rgbd_calibration/charuco.py`, `scripts/head_rgbd_calibration/config.py`, `scripts/head_rgbd_calibration/charuco.py`, `tests/test_head_rgbd_calib_core.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint`
- Verified head: ccc34ea588a7643e105221b0f3994222d47879bf

#### Requirement

Create the shared calibration core package and move its first two modules into it; maths and behaviour unchanged (design D2).

- `shared/ros2_common/head_rgbd_calibration/__init__.py`: package docstring naming its modules and stating it never imports `scripts`, `rosbags`, `subprocess` or ssh code.
- `shared/ros2_common/head_rgbd_calibration/config.py`: `StrictModel` (pydantic `BaseModel`, `extra="forbid"`, defined here; the core does not import `scripts.markers`), and `BoardConfig`, `SelectionConfig`, `LimitsConfig` moved from `scripts/head_rgbd_calibration/config.py` with identical fields, defaults and validators.
- `shared/ros2_common/head_rgbd_calibration/charuco.py`: `scripts/head_rgbd_calibration/charuco.py` moved with `git mv`, importing `BoardConfig` from the core config.
- `scripts/head_rgbd_calibration/config.py` keeps `CaptureConfig`, `OutputsConfig`, `ValidationConfig`, `HeadCameraConfig`, `CalibrationConfig`, `load_calibration_config` and the topic constants, built on the core `StrictModel`, `BoardConfig`, `SelectionConfig`, `LimitsConfig` imported from the core (so `scripts.head_rgbd_calibration.config.BoardConfig` still resolves for its importers).
- `scripts/head_rgbd_calibration/charuco.py` is re-created as a transitional one-import re-export of the core names (`Detection`, `detect`, `make_board`, `MIN_CORNERS`) so the remaining importers keep working; task calib-core-retire-shims deletes it.
- Root `pyproject.toml`: `ros2-common` added to the `dev` dependency group with `[tool.uv.sources] ros2-common = { path = "shared", editable = true }`; `uv.lock` regenerated with `uv lock`, so `uv run --frozen pytest` imports `ros2_common.head_rgbd_calibration`.
- New `tests/test_head_rgbd_calib_core.py`: an AST scan proves no module under `shared/ros2_common/head_rgbd_calibration/` imports `scripts`, `rosbags` or `subprocess`; the core `BoardConfig`, `SelectionConfig`, `LimitsConfig` reject unknown keys and their defaults equal the `board`, `selection`, `limits` blocks of `scripts/head_rgbd_calibrate.yaml`; the core `detect` finds the synthetic board of `tests/rgbd_synthetic.py`.
- Every existing calibration test passes with no assertion changed.
- Docs (doc files, outside the file cap): `shared/README.md` gains a `## head_rgbd_calibration` section (what the package holds and who imports it: the CLI under `scripts/` and the head_rgbd_calibration node); `tests/README.md` lists the new test file.
- Root `uv run poe lint` passes (root lint covers `shared/`).

#### Acceptance conditions

- a1: the core package imports no scripts, rosbags or subprocess code [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_imports_no_scripts_rosbags_or_subprocess]
- a2: the core board, selection and limits models keep the CLI yaml defaults and reject unknown keys [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_config_defaults_match_cli_yaml]
- a3: the core detector finds the synthetic board [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_detect_finds_synthetic_board]
- a4: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q]
- a5: root lint passes with the core under shared/ [evidence: command:uv run poe lint]
- a6: the root project resolves ros2-common from shared, editable [evidence: command:grep -q 'ros2-common = { path = "shared", editable = true }' pyproject.toml]
- a7: the shared README documents the package [evidence: doc-section:shared/README.md#head_rgbd_calibration]

#### Files

- `pyproject.toml`
- `uv.lock`
- `shared/ros2_common/head_rgbd_calibration/__init__.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/charuco.py`
- `scripts/head_rgbd_calibration/config.py`
- `scripts/head_rgbd_calibration/charuco.py`
- `tests/test_head_rgbd_calib_core.py`

#### Interfaces

- `shared/ros2_common/head_rgbd_calibration/config.py::StrictModel`, `BoardConfig`, `SelectionConfig`, `LimitsConfig` (fields unchanged from `scripts/head_rgbd_calibration/config.py`).
- `shared/ros2_common/head_rgbd_calibration/charuco.py::make_board(cfg) -> cv2.aruco.CharucoBoard`, `detect(image, board, upscale=1) -> Detection | None`, `Detection`, `MIN_CORNERS`.
- Contract `rgbd-calib-core` (owned here, design Contracts table).

#### Expected outcome

`from ros2_common.head_rgbd_calibration.charuco import detect` works in the root venv and in any node venv that depends on ros2-common; the CLI and every calibration test behave exactly as before.

#### Steps

- Write `tests/test_head_rgbd_calib_core.py` first; `uv run --frozen pytest tests/test_head_rgbd_calib_core.py -q` fails on the missing package (red).
- Add ros2-common to the root dev group and `uv lock`; `git mv` charuco; move the three config models and StrictModel; re-create the transitional `scripts/head_rgbd_calibration/charuco.py` re-export.
- Run the verify command: every listed test passes, `uv run poe lint` prints no findings.

#### Notes

- `scripts/markers/models.py:StrictModel` stays for the markers code; the core defines its own so it never imports `scripts`.
- README updates (`shared/README.md`, `tests/README.md`) are doc files outside the 8-file cap.
- Design D2; the transitional re-export is removed by calib-core-retire-shims.

### S2: Calib core frames intrinsics

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/frames.py`, `shared/ros2_common/head_rgbd_calibration/intrinsics.py`, `scripts/head_rgbd_calibration/frames.py`, `scripts/head_rgbd_calibration/intrinsics.py`, `tests/test_head_rgbd_calib_core.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint`
- Verified head: 842b9c850d6b57b69f5e16f0625520a46c0a4312

#### Requirement

Move the frame selection and the intrinsic solver into the core; maths and behaviour unchanged (design D2).

- `shared/ros2_common/head_rgbd_calibration/frames.py`: `NS_PER_S`, `MAX_UINT8`, `RGB_DIFF_BLUR_SIGMA`, `StampedImage`, `CameraInfoData`, `FrameSet`, `FramePair`, `normalise_amplitude`, `mean_abs_diff`, `rgb_difference`, `tof_diff_fraction`, `nearest_index`, `still_pairs` moved unchanged from `scripts/head_rgbd_calibration/frames.py`; no rosbags import.
- `scripts/head_rgbd_calibration/frames.py` keeps `TopicNames`, `TYPESTORE`, `ROS_CLI_ENV`, `BAG_STORAGE`, `STEP_NAMES`, `BAG_METADATA`, `capture_commands`, `bag_message_count`, `capture_bag`, `stamp_ns`, `decode_mono16`, `decode_compressed`, `decode_camera_info`, `read_frames`, and imports the moved names from the core (its importers keep resolving `scripts.head_rgbd_calibration.frames.still_pairs` and the dataclasses).
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py`: the whole module moved with `git mv`, importing from the core `charuco`, `config`, `frames`. `scripts/head_rgbd_calibration/intrinsics.py` is re-created as a transitional re-export of its public names, deleted by calib-core-retire-shims.
- `tests/test_head_rgbd_calib_core.py` gains: core `still_pairs` selects one pair per still window of a synthetic FrameSet; core `rgb_difference` of two frames differing only by a per-row gain (mains flicker bands) is below 1 grey level; core intrinsics `calibrate` recovers the synthetic camera matrix within the tolerance the existing intrinsics test uses.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes; `shared/README.md` section lists the new modules.

#### Acceptance conditions

- a1: core still_pairs selects one pair per still window [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_still_pairs_one_pair_per_window]
- a2: core rgb_difference ignores mains flicker bands [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_rgb_difference_ignores_flicker_bands]
- a3: core intrinsics recover the synthetic camera [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_intrinsics_recover_synthetic_camera]
- a4: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q]
- a5: root lint passes [evidence: command:uv run poe lint]
- a6: still_pairs lives in the core frames module [evidence: symbol:shared/ros2_common/head_rgbd_calibration/frames.py::still_pairs]

#### Files

- `shared/ros2_common/head_rgbd_calibration/frames.py`
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py`
- `scripts/head_rgbd_calibration/frames.py`
- `scripts/head_rgbd_calibration/intrinsics.py`
- `tests/test_head_rgbd_calib_core.py`

#### Interfaces

- `shared/ros2_common/head_rgbd_calibration/frames.py::still_pairs(frames, sel) -> list[FramePair]`, `rgb_difference`, `tof_diff_fraction`, `normalise_amplitude`, `nearest_index`, dataclasses `StampedImage`, `CameraInfoData`, `FrameSet`, `FramePair`.
- `shared/ros2_common/head_rgbd_calibration/intrinsics.py::calibrate`, `calibrate_tof`, `compare_sdk`, `write_camera_calibration_yaml`, `IntrinsicsResult`.

#### Expected outcome

The pure frame functions and the intrinsic solver import from the core without rosbags; the CLI still reads bags through `scripts/head_rgbd_calibration/frames.py`.

#### Steps

- Add the three new tests to `tests/test_head_rgbd_calib_core.py`; run them: red on the missing core modules.
- Move the pure part of frames.py and `git mv` intrinsics.py; re-create the transitional intrinsics re-export.
- Run the verify command: all pass, lint clean.

#### Notes

- `decode_*` and `read_frames` stay in scripts (they use the rosbags typestore messages).
- Do not change any threshold or formula (design non-goal).

### S3: Calib core extrinsics urdf

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/extrinsics.py`, `shared/ros2_common/head_rgbd_calibration/urdf_text.py`, `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`, `scripts/head_rgbd_calibration/extrinsics.py`, `scripts/head_rgbd_calibration/urdf_handoff.py`, `tests/test_head_rgbd_calib_core.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint`
- Verified head: 604b4210cbc736c449560c103a7cb92a59c2ed57

#### Requirement

Move the stereo extrinsic and the URDF hand-off into the core, with the tof_joint text rewrite split into a stdlib-only module the head_state_publisher launch can load (design D2, D8).

- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`: the whole module moved with `git mv`, importing from the core.
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`: imports only standard-library modules. Holds `TOF_JOINT`, `NOTE_MARKER`, `NUMBER_FORMAT`, `Origin` (frozen dataclass, `xyz` and `rpy` float triples), `format_vec` and `rewrite_tof_joint(urdf_text, origin, note)`, moved from `urdf_handoff.py` with byte-identical output.
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`: the rest of `scripts/head_rgbd_calibration/urdf_handoff.py` (joint constants, `JointInfo`, `HeadChain`, `Drift`, `to_matrix`, `from_matrix`, `extrinsic_matrix`, `parse_vec`, `read_chain`, `tof_joint_origin`, `check_drift`), importing `Origin`, `format_vec`, `rewrite_tof_joint` from `urdf_text` so `urdf_handoff.rewrite_tof_joint` still resolves.
- `scripts/head_rgbd_calibration/extrinsics.py` and `scripts/head_rgbd_calibration/urdf_handoff.py` are re-created as transitional re-exports, deleted by calib-core-retire-shims.
- `tests/test_head_rgbd_calib_core.py` gains: every import in `urdf_text.py` is in `sys.stdlib_module_names` (AST scan); loading `urdf_text.py` by file path with `importlib.util.spec_from_file_location` in a `python -I` subprocess leaves `numpy` out of `sys.modules`; core `rewrite_tof_joint` on `nodes/web_ui/urdf/head.urdf` with a fixed origin changes only the tof_joint origin and the calibration comment (every other line identical); core `stereo_calibrate` recovers the synthetic extrinsic within the existing extrinsics test tolerance.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes; `shared/README.md` lists the modules and notes that `urdf_text.py` must stay stdlib-only.

#### Acceptance conditions

- a1: urdf_text imports only the standard library [evidence: test:tests/test_head_rgbd_calib_core.py::test_urdf_text_is_stdlib_only]
- a2: urdf_text loads by file path without numpy [evidence: test:tests/test_head_rgbd_calib_core.py::test_urdf_text_loads_by_path_without_numpy]
- a3: the core rewrite changes only the tof_joint origin and its comment [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_rewrite_tof_joint_touches_only_tof_joint]
- a4: the core stereo extrinsic recovers the synthetic pose [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_stereo_extrinsic_recovers_synthetic_pose]
- a5: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`
- `scripts/head_rgbd_calibration/extrinsics.py`
- `scripts/head_rgbd_calibration/urdf_handoff.py`
- `tests/test_head_rgbd_calib_core.py`

#### Interfaces

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py::rewrite_tof_joint(urdf_text: str, origin: Origin, note: str) -> str`, `Origin(xyz, rpy)`, `format_vec`, `TOF_JOINT`, `NOTE_MARKER` (stdlib only; loaded by path by head-urdf-tof-joint-override).
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py::read_chain`, `tof_joint_origin`, `check_drift`.
- `shared/ros2_common/head_rgbd_calibration/extrinsics.py::stereo_calibrate`, `plane_depth_bias`, `write_extrinsics_yaml`, `load_extrinsics_yaml`, `ExtrinsicResult`, `PlaneCheck`.

#### Expected outcome

`urdf_text.py` loads in a bare `python3 -I` by file path; extrinsics and the URDF hand-off import from the core; CLI `urdf` mode output is unchanged.

#### Steps

- Add the four tests; run: red.
- `git mv` extrinsics.py; split urdf_handoff.py into `urdf_text.py` (stdlib) and `urdf_handoff.py`; re-create the two transitional re-exports.
- Run the verify command: all pass, lint clean.

#### Notes

- head_state_publisher runs on the system Python without numpy guarantees: keep `urdf_text.py` free of numpy, scipy and package-relative imports.

### S4: Calib core calibrate run

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/calibrate.py`, `scripts/head_rgbd_calibration/pipeline.py`, `scripts/head_rgbd_calibration/validate.py`, `scripts/head_rgbd_calibrate.py`, `tests/rgbd_synthetic.py`, `tests/test_head_rgbd_calibrate_cli.py`, `tests/test_head_rgbd_validate.py`, `tests/test_head_rgbd_calib_core.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint`
- Verified head: 177a16117ab3bf8a312e9bed7111a9069b4c9578

#### Requirement

Move the calibration run into the core so it can run from stored poses with their detections, never from a bag; the CLI gives identical results (design D2, D5).

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`: `detect_tof(amplitude, board, sel)` and `CalibrationRun` moved from `scripts/head_rgbd_calibration/pipeline.py`; `PoseView` (frozen dataclass: `rgb: Detection | None`, `tof: Detection | None`, `depth: np.ndarray | None`); `calibrate_detections(views, rgb_size, tof_size, tof_info, board_cfg, limits) -> CalibrationRun`, the fit half of today's `run_calibration` (RGB `calibrate` on views with an RGB detection, `calibrate_tof` on views with a ToF detection, `stereo_calibrate` on views with both, `plane_depth_bias` on those with depth, `compare_sdk`; `pair_count = len(views)`; `ValueError` when `tof_info` is None, same message as today); `failed_limits(run, limits: LimitsConfig)` and `report_markdown(run, limits: LimitsConfig, session, lens, date)` producing byte-identical text for the same inputs.
- `scripts/head_rgbd_calibration/pipeline.py`: `run_calibration(frames, cfg)` becomes `still_pairs` + `detect`/`detect_tof` per pair + core `calibrate_detections`; `failed_limits`/`report_markdown` callers (`write_calibration`, `scripts/head_rgbd_calibrate.py`) pass `cfg.limits`.
- `scripts/head_rgbd_calibration/pipeline.py`, `scripts/head_rgbd_calibration/validate.py`, `scripts/head_rgbd_calibrate.py`, `tests/rgbd_synthetic.py`, `tests/test_head_rgbd_calibrate_cli.py`, `tests/test_head_rgbd_validate.py` import moved names from the core directly, not through the transitional re-exports.
- `tests/test_head_rgbd_calib_core.py` gains: `calibrate_detections` on the views built from a synthetic FrameSet equals `run_calibration` on that FrameSet (RMS values, view counts, K, extrinsic `t` within 1e-9); `failed_limits` names each exceeded `limits.*` key; `report_markdown` of a fixed run equals a golden string.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes; `scripts/README.md` notes that the maths lives in `ros2_common.head_rgbd_calibration` and the CLI wraps it.

#### Acceptance conditions

- a1: calibrate_detections matches run_calibration on the same frames [evidence: test:tests/test_head_rgbd_calib_core.py::test_calibrate_detections_matches_run_calibration]
- a2: failed_limits names each exceeded limits key [evidence: test:tests/test_head_rgbd_calib_core.py::test_failed_limits_names_limit_keys]
- a3: the report text is unchanged [evidence: test:tests/test_head_rgbd_calib_core.py::test_report_markdown_golden]
- a4: the CLI and validation tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_urdf_handoff.py -q]
- a5: root lint passes [evidence: command:uv run poe lint]
- a6: calibrate_detections exists in the core [evidence: symbol:shared/ros2_common/head_rgbd_calibration/calibrate.py::calibrate_detections]

#### Files

- `shared/ros2_common/head_rgbd_calibration/calibrate.py`
- `scripts/head_rgbd_calibration/pipeline.py`
- `scripts/head_rgbd_calibration/validate.py`
- `scripts/head_rgbd_calibrate.py`
- `tests/rgbd_synthetic.py`
- `tests/test_head_rgbd_calibrate_cli.py`
- `tests/test_head_rgbd_validate.py`
- `tests/test_head_rgbd_calib_core.py`

#### Interfaces

- `shared/ros2_common/head_rgbd_calibration/calibrate.py::calibrate_detections(views: list[PoseView], rgb_size: tuple[int, int], tof_size: tuple[int, int], tof_info: CameraInfoData | None, board_cfg: BoardConfig, limits: LimitsConfig) -> CalibrationRun`.
- `detect_tof(amplitude, board, sel) -> Detection | None`, `failed_limits(run, limits) -> list[str]`, `report_markdown(run, limits, session, lens, date) -> str`, `PoseView`, `CalibrationRun`.

#### Expected outcome

The node can calibrate from stored detections; `scripts/head_rgbd_calibrate.py calibrate` writes the same files and report as before for the same bag.

#### Steps

- Add the parity, limits and golden-report tests; run: red.
- Move `detect_tof`, `CalibrationRun`, `failed_limits`, `report_markdown`; extract `calibrate_detections` from `run_calibration`; switch the listed importers to the core.
- Run the verify command: all pass, lint clean.

#### Notes

- `run_calibration` keeps its signature for the CLI; only the fit half moves.

### S5: Calib node scaffold config

- Status: done
- Files: `nodes/head_rgbd_calibration/pyproject.toml`, `nodes/head_rgbd_calibration/uv.lock`, `nodes/head_rgbd_calibration/head_rgbd_calibration/__init__.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`, `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`, `nodes/head_rgbd_calibration/tests/test_config.py`, `pyproject.toml`, `.github/workflows/ci.yml`
- Verify: `(cd nodes/head_rgbd_calibration && uv sync && uv run --frozen pytest tests/test_config.py -q && uv run --frozen poe lint) && uv run --frozen pytest tests/test_ci_workflow.py -q`
- Verified head: unknown

#### Requirement

Create the robot-side calibration node `head_rgbd_calibration` as a uv project with its validated config (design D1).

- `nodes/head_rgbd_calibration/pyproject.toml` (project `head-rgbd-calibration`, package `head_rgbd_calibration`, Python `>=3.12,<3.15`): dependencies fastapi, uvicorn[standard], pydantic>=2, pyyaml, numpy>=1.26, opencv-python-headless>=4.9,<5, ros2-common (`[tool.uv.sources] ros2-common = { path = "../../shared", editable = true }`); dev group ruff==0.15.20, vulture, poethepoet, pytest, httpx; poe `lint` (ruff check, ruff format --check, vulture) and `lint-fix` like `nodes/web_ui/pyproject.toml`; `uv.lock` generated.
- `head_rgbd_calibration/__init__.py` docstring; `tests/__init__.py` if the repo's node test layout needs it.
- `head_rgbd_calibration/config.py`: `NodeConfig` (the core `StrictModel`; not named CalibrationConfig) with constants at the top: `state_dir` (`/var/lib/ros2/head_rgbd_calibration`), `http_host` (`127.0.0.1`), `http_port` (18110), `topics` (`rgb` `/head_camera/mapping/image_raw/compressed`, `amplitude` `/tof/amplitude/image_raw`, `depth` `/tof/depth/image_raw`, `tof_info` `/tof/camera_info`), `qos` (`tof_depth` 10, `rgb_depth` 5, both >= 1), `max_capture_s` 900 (> 0), `compute_timeout_s` 600 (> 0), `compute_nice` 10 (0..19), `head_camera_config_file` (`/etc/ros2/head_camera_cpp/config.yaml`), `default_lens_position` 1.0, `head_urdf_file` (`/opt/ros2-lerobot-swerve-platform/nodes/web_ui/urdf/head.urdf`), `restart_units` (`ros2-head_camera_cpp.service`, `ros2-head_rgbd_cpp.service`, `ros2-head_state_publisher.service`), `restart_check_s` 60, `thumbnail_width_px` 320, `coverage` (`cols` 4 in 3..6, `rows` 3 in 2..4), and the core `board`, `selection`, `limits`; `CONFIG_ENV = "HEAD_RGBD_CALIBRATION_CONFIG"`; `load_config(path) -> NodeConfig` (empty file gives defaults, unknown keys fail naming the key). Type hints and docstrings with input and output types.
- `config/head_rgbd_calibration.yaml`: every default, each key commented.
- `tests/test_config.py`: defaults equal the shipped yaml; unknown keys rejected naming the key; out-of-range values rejected; `board`, `selection`, `limits` equal those of `scripts/head_rgbd_calibrate.yaml` (read relative to the repo root).
- Root `pyproject.toml` `lint-nodes` runs `(cd nodes/head_rgbd_calibration && uv run --frozen poe lint)`; `.github/workflows/ci.yml` node matrix includes `head_rgbd_calibration` so `tests/test_ci_workflow.py` stays green.

#### Acceptance conditions

- a1: the config defaults equal the shipped yaml [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py::test_defaults_match_shipped_yaml -q]
- a2: unknown keys and out-of-range values are rejected naming the key [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py::test_rejects_unknown_and_out_of_range -q]
- a3: board, selection and limits equal the CLI config [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py::test_board_selection_limits_match_cli -q]
- a4: the node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]
- a5: CI and root lint-nodes cover the node [evidence: command:uv run --frozen pytest tests/test_ci_workflow.py -q && grep -q "nodes/head_rgbd_calibration" pyproject.toml]

#### Files

- `nodes/head_rgbd_calibration/pyproject.toml`
- `nodes/head_rgbd_calibration/uv.lock`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__init__.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `pyproject.toml`
- `.github/workflows/ci.yml`

#### Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py::NodeConfig`, `load_config(path: Path) -> NodeConfig`, `CONFIG_ENV`.
- Contract `rgbd-calib-node-config` (owned here).

#### Expected outcome

`cd nodes/head_rgbd_calibration && uv sync && uv run pytest tests -q` passes and `uv run poe lint-nodes` at the root covers the node; CI runs its tests.

#### Steps

- Write `tests/test_config.py`; `uv run --frozen pytest tests/test_config.py -q`: red.
- Write `pyproject.toml`, `uv lock`, `config.py`, the shipped yaml; add the node to root `lint-nodes` and the CI matrix.
- Run the verify command: all pass, lint clean.

#### Ruling

- 2026-10-19 accepted: verifier crashed on --prev-head unknown (tooling); orchestrator re-ran the task verify on HEAD 52434e47: 16 passed, lint clean, 14 passed (ci_workflow, node_lint_coverage), clean tree

#### Notes

- Pattern: `nodes/web_ui/pyproject.toml` (ros2-common editable path source, poe lint) and `nodes/marker_detector`.
- README is created by calib-node-session-store.

### S6: Calib pose counter in core

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/poses.py`, `scripts/head_rgbd_calibration/live.py`, `tests/test_head_rgbd_calib_poses.py`, `tests/test_head_rgbd_calib_live.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calib_live.py -q && uv run poe lint`
- Verified head: dabec178777ffbd66e86536e41a0d9c6107c2876

#### Requirement

Move the live pose counter into the core and extend it so the robot node gets every captured pose with its detections and depth (design D2, D4).

- `shared/ros2_common/head_rgbd_calibration/poses.py`: `PoseCounter` moved from `scripts/head_rgbd_calibration/live.py` with `MOVING`, `NO_TOF_FRAME`, `NO_RGB_BOARD`, `NO_TOF_BOARD`, `captured_status`, `TOF_HISTORY_S`; new `WAITING = "waiting for frames"` and frame kinds `RGB`, `AMPLITUDE`, `DEPTH`.
- `PoseCounter(board, sel, start_count=0)`; `add(kind, frame, raw=None) -> list[PoseEvent]`; `PoseEvent(status: str, pose: CapturedPose | None)`; `CapturedPose(number, rgb: StampedImage, rgb_raw: bytes | None, amplitude: StampedImage, depth: StampedImage | None, rgb_detection: Detection, tof_detection: Detection)`; property `status` is `WAITING` until at least one RGB and one amplitude frame arrived, then the last judged status.
- Stillness and pairing rules identical to the moved code: `rgb_difference` below `still_rgb_mad`, every ToF pair in the still window below `still_tof_mad`, ToF frame within `max_pair_dt_s`, `skip_start_s`, one pose per still window, RGB detection then `detect_tof`.
- Depth frames are kept for `TOF_HISTORY_S` like amplitude; a captured pose takes the depth frame nearest the chosen amplitude stamp when within `max_pair_dt_s`, otherwise `depth` is None (never a substitute frame).
- `CapturedPose` carries the detections computed by the judgement (no second detection of the same frame) and the raw compressed bytes passed with the RGB frame.
- `start_count` makes numbering continue for a reopened session (`pose N captured` counts from it).
- `scripts/head_rgbd_calibration/live.py` imports the counter from the core and adapts `monitor` to `PoseEvent`, so the CLI keeps working until retire-ssh-live-counter deletes it; the PoseCounter tests move from `tests/test_head_rgbd_calib_live.py` to the new `tests/test_head_rgbd_calib_poses.py` (relay and monitor tests stay where they are).
- Root `uv run poe lint` passes; `tests/README.md` and the `shared/README.md` section updated (doc files).

#### Acceptance conditions

- a1: one pose is counted per still window with the board in both cameras [evidence: test:tests/test_head_rgbd_calib_poses.py::test_counts_one_pose_per_still_window]
- a2: a moving board is never counted [evidence: test:tests/test_head_rgbd_calib_poses.py::test_moving_board_is_not_counted]
- a3: the status names a missing ToF board [evidence: test:tests/test_head_rgbd_calib_poses.py::test_status_names_missing_tof_board]
- a4: depth within max_pair_dt_s is paired with the pose [evidence: test:tests/test_head_rgbd_calib_poses.py::test_depth_paired_within_max_dt]
- a5: depth beyond max_pair_dt_s is absent, not substituted [evidence: test:tests/test_head_rgbd_calib_poses.py::test_depth_absent_beyond_max_dt]
- a6: the captured pose carries the judged detections and raw bytes [evidence: test:tests/test_head_rgbd_calib_poses.py::test_captured_pose_carries_judged_detections]
- a7: start_count continues the numbering [evidence: test:tests/test_head_rgbd_calib_poses.py::test_start_count_continues_numbering]
- a8: status is waiting until both streams delivered a frame [evidence: test:tests/test_head_rgbd_calib_poses.py::test_waiting_until_both_streams]
- a9: the remaining live CLI tests pass and root lint passes [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_live.py -q && uv run poe lint]

#### Files

- `shared/ros2_common/head_rgbd_calibration/poses.py`
- `scripts/head_rgbd_calibration/live.py`
- `tests/test_head_rgbd_calib_poses.py`
- `tests/test_head_rgbd_calib_live.py`

#### Interfaces

- `shared/ros2_common/head_rgbd_calibration/poses.py::PoseCounter(board, sel, start_count=0)`, `.add(kind, frame, raw=None) -> list[PoseEvent]`, `.status`, `.count`; `PoseEvent`, `CapturedPose`; kinds `RGB`, `AMPLITUDE`, `DEPTH`; statuses `WAITING`, `MOVING`, `NO_TOF_FRAME`, `NO_RGB_BOARD`, `NO_TOF_BOARD`, `captured_status(n)`.
- Contract `rgbd-pose-counter` (owned here; consumed by calib-node-live-capture).

#### Expected outcome

The capture node can feed frames as they arrive and receive complete poses (RGB with raw JPEG, amplitude, optional depth, both detections) with the same stillness rules the terminal counter used.

#### Steps

- Write `tests/test_head_rgbd_calib_poses.py` (port the PoseCounter tests from `tests/test_head_rgbd_calib_live.py`, add the new ones); run: red.
- Move and extend the counter; adapt `live.py`'s `monitor` to `PoseEvent`.
- Run the verify command: all pass, lint clean.

#### Notes

- Never store a pose without both detections; never fill a missing depth (CLAUDE.md no placeholder data).

### S7: Calib node session store

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/images.py`, `nodes/head_rgbd_calibration/tests/test_store.py`, `nodes/head_rgbd_calibration/tests/test_images.py`, `nodes/head_rgbd_calibration/README.md`, `nodes/README.md`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py tests/test_images.py -q && uv run --frozen poe lint)`
- Verified head: 4e01ced4a938764cf4426091e2e2f8c5ad8d0101

#### Requirement

Session store and thumbnails of the calibration node: only selected poses persist under the state dir, with safe ids and atomic writes (design D3).

- `head_rgbd_calibration/store.py` `SessionStore(state_dir: Path)` with the layout of design D3:
  - `create_session(now, lens_position) -> str` (id `calib-YYYYmmdd-HHMMSS`, `-N` suffix on a same-second clash), `list_sessions()` newest first with `pose_count` and `result_count`, `load_session(sid)`, `set_tof_info(sid, CameraInfoData)` (`session.json` stores `tof_info` as the `CameraInfoData` fields `k`, `p`, `d`, `width`, `height`, `distortion_model`, and `rgb_size` / `tof_size`), `set_image_sizes(sid, rgb_size, tof_size)` (sizes are recorded from the first pose; a pose of another size is refused with `ValueError`).
  - `add_pose(sid, pose: CapturedPose) -> str`: pose id zero-padded from `next_pose` in `session.json`, never reused after a delete; writes `rgb.jpg` (the raw JPEG bytes as received; encoded from the decoded frame only when `rgb_raw` is None), `amplitude.png` and `depth.png` (uint16 PNG, depth only when present), `pose.json` (fields from the core `Detection` per camera - corner `ids` and `points` - plus the rgb and tof stamps), `rgb_thumb.jpg`, `tof_thumb.jpg`; the pose directory is written under a temporary name and renamed into place; `session.json` updated with `os.replace`.
  - `delete_pose(sid, pid)`; `load_views(sid) -> SessionViews` (`views: list[PoseView]` rebuilt from `pose.json` and `depth.png`, `rgb_size`, `tof_size`, `tof_info`, `lens_position`, `pose_ids`) for the compute job.
  - `coverage(sid) -> {rgb: {cols, rows, counts}, tof: {cols, rows, counts}}`: corner counts per grid cell (cfg `coverage` cols x rows) over every stored pose, from `pose.json`, with `rgb_size` and `tof_size` read from `session.json`; a session without poses gives all-zero counts.
  - `result_dir(sid, rid)`, `new_result_id(now)` (`r-YYYYmmdd-HHMMSS`), `list_results(sid)` (each `result.json`, newest first), `thumbnail_path(sid, pid, kind)`.
  - Path safety: ids match `SESSION_ID_RE`, `POSE_ID_RE` (`^\d{3,}$`), `RESULT_ID_RE`; the resolved path must stay under `state_dir`; violations raise `InvalidId`, missing items raise `NotFound` (both map to 404 later).
  - One `threading.Lock` serialises mutations.
- `head_rgbd_calibration/images.py`: `rgb_thumbnail(image, detection, width_px) -> bytes` (JPEG, corners drawn with `cv2.aruco.drawDetectedCornersCharuco`, scaled to `width_px`), `tof_thumbnail(amplitude, detection, percentiles)` (`percentiles` is the selection `amplitude_percentiles`, the same as `normalise_amplitude` uses) -> bytes | None` (core `normalise_amplitude` with the selection `amplitude_percentiles`, enlarged 2x, corners drawn; None for an empty frame, and such a pose is not stored). Pattern: `nodes/web_ui/web_ui/photos.py` (scaled JPEG, path safety).
- Tests `tests/test_store.py`, `tests/test_images.py` with poses built from the core synthetic board helpers in a `tmp_path` state dir.
- Docs (doc files): `nodes/head_rgbd_calibration/README.md` (purpose, `## State directory` layout, config keys table, tests list) and `nodes/README.md` lists the node.

#### Acceptance conditions

- a1: a pose is written completely or not at all [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_add_pose_writes_all_files_atomically -q]
- a2: pose ids are never reused after a delete [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_pose_ids_never_reused_after_delete -q]
- a3: traversal and malformed ids are rejected [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_rejects_traversal_ids -q]
- a4: coverage counts corners per grid cell for RGB and ToF [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_coverage_counts_corners_per_cell -q]
- a5: stored views round-trip into PoseView with depth [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_load_views_round_trip -q]
- a6: sessions are listed newest first with counts [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_store.py::test_sessions_listed_newest_first -q]
- a7: thumbnails carry the drawn corners and the ToF one is enlarged [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_images.py -q]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]
- a9: the README documents the state directory [evidence: doc-section:nodes/head_rgbd_calibration/README.md#State directory]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/images.py`
- `nodes/head_rgbd_calibration/tests/test_store.py`
- `nodes/head_rgbd_calibration/tests/test_images.py`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/README.md`

#### Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py::SessionStore` (methods in the requirement), `InvalidId`, `NotFound`, `SessionViews`, `SESSION_ID_RE`, `POSE_ID_RE`, `RESULT_ID_RE`.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/images.py::rgb_thumbnail`, `tof_thumbnail`.
- Contract `rgbd-calib-session-store` (owned here); layout of design D3.

#### Expected outcome

A session directory under a temp state dir holds exactly the D3 files per pose; ids from the outside can never reach a path outside the state dir.

#### Steps

- Write `tests/test_store.py` and `tests/test_images.py`; run: red.
- Implement the store and the images module; write the README.
- Run the verify command: all pass, lint clean.

#### Notes

- Pattern for thumbnails and path safety: `nodes/web_ui/web_ui/photos.py`.
- Coverage counts are derived only from stored corners (never invented).

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-scaffold-config

### S8: Retire ssh live counter

- Status: done
- Files: `scripts/head_rgbd_calibration/live.py`, `scripts/head_rgbd_calibration/live_relay.py`, `tests/test_head_rgbd_calib_live.py`, `scripts/head_rgbd_calibration/config.py`, `scripts/head_rgbd_calibrate.py`, `scripts/head_rgbd_calibrate.yaml`, `tests/test_head_rgbd_calibrate_cli.py`, `tests/test_head_rgbd_calib_retired.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint`
- Verified head: 8f3a28a1e8da0baa4cee174923b98f237201b71e

#### Requirement

Retire the ssh-streamed live counter; the web UI Calibrate page replaces it (design D2). Deletion pinned by a retirement test (TDD exception: deletion pinned by a retirement test).

- `git rm` `scripts/head_rgbd_calibration/live.py`, `scripts/head_rgbd_calibration/live_relay.py`, `tests/test_head_rgbd_calib_live.py`.
- `capture.live` removed from `CaptureConfig` (`scripts/head_rgbd_calibration/config.py`) and from `scripts/head_rgbd_calibrate.yaml`; a config that still sets `capture.live` fails validation with a message naming `live`.
- `scripts/head_rgbd_calibrate.py`: no `LIVE` hook or `start_live` import; `capture` records the bag (record, copy, remove) and prints its path.
- `tests/test_head_rgbd_calibrate_cli.py`: live-counter tests removed; a test proves `capture` runs exactly the record, copy and remove commands and nothing else.
- New `tests/test_head_rgbd_calib_retired.py`: `git ls-files` returns nothing for the three removed paths; `git grep` finds no `live_relay`, `start_live` or `capture.live` in tracked files under `scripts/` and `tests/` (this test file excluded).
- Docs (doc files): `scripts/README.md` capture protocol drops the live counter and points to the web UI Calibrate page for robot-side capture; `tests/README.md` updated.
- Root `uv run poe lint` passes.

#### Acceptance conditions

- a1: the live counter files are no longer tracked [evidence: test:tests/test_head_rgbd_calib_retired.py::test_live_counter_files_untracked]
- a2: no tracked script or test references the live counter [evidence: test:tests/test_head_rgbd_calib_retired.py::test_no_live_counter_references]
- a3: a config setting capture.live is rejected naming the key [evidence: test:tests/test_head_rgbd_calibrate_cli.py::test_capture_live_key_rejected]
- a4: capture runs only record, copy and remove [evidence: test:tests/test_head_rgbd_calibrate_cli.py::test_capture_runs_only_record_copy_remove]
- a5: root lint passes [evidence: command:uv run poe lint]
- a6: scripts/README.md no longer describes the live counter [evidence: command:! grep -q live_relay scripts/README.md]

#### Files

- `scripts/head_rgbd_calibration/live.py`
- `scripts/head_rgbd_calibration/live_relay.py`
- `tests/test_head_rgbd_calib_live.py`
- `scripts/head_rgbd_calibration/config.py`
- `scripts/head_rgbd_calibrate.py`
- `scripts/head_rgbd_calibrate.yaml`
- `tests/test_head_rgbd_calibrate_cli.py`
- `tests/test_head_rgbd_calib_retired.py`

#### Interfaces

- `scripts/head_rgbd_calibration/config.py::CaptureConfig` without `live`.
- `scripts/head_rgbd_calibrate.py::capture(cfg, suffix) -> int` records the bag only.

#### Expected outcome

`git ls-files scripts/head_rgbd_calibration/live.py scripts/head_rgbd_calibration/live_relay.py tests/test_head_rgbd_calib_live.py` prints nothing; the CLI capture records bags as before.

#### Steps

- Write `tests/test_head_rgbd_calib_retired.py` and the two CLI tests; run: red (files still tracked, key still accepted).
- `git rm` the three files; remove `capture.live` and the LIVE hook; update the docs.
- Run the verify command: all pass, lint clean.

#### Notes

- TDD exception for the deletions: deletion pinned by a retirement test (name it in `tests.note`).

### S9: Calib core retire shims

- Status: done
- Files: `scripts/head_rgbd_calibration/charuco.py`, `scripts/head_rgbd_calibration/intrinsics.py`, `scripts/head_rgbd_calibration/extrinsics.py`, `scripts/head_rgbd_calibration/urdf_handoff.py`, `tests/test_head_rgbd_calib_extrinsics.py`, `tests/test_head_rgbd_calib_intrinsics.py`, `tests/test_head_rgbd_urdf_handoff.py`, `tests/test_head_rgbd_calib_core.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calib_retired.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint`
- Verified head: 6f9c9c79b54f058f6cb72fec128c69298d9ad42e

#### Requirement

Delete the transitional re-export modules left by the core move so the calibration code lives in one place (design D2). Deletion pinned by a retirement test.

- `git rm` `scripts/head_rgbd_calibration/charuco.py`, `scripts/head_rgbd_calibration/intrinsics.py`, `scripts/head_rgbd_calibration/extrinsics.py`, `scripts/head_rgbd_calibration/urdf_handoff.py`.
- `tests/test_head_rgbd_calib_extrinsics.py`, `tests/test_head_rgbd_calib_intrinsics.py`, `tests/test_head_rgbd_urdf_handoff.py` (and any other tracked importer found by `git grep`) import from `ros2_common.head_rgbd_calibration` instead; no assertion changes.
- `tests/test_head_rgbd_calib_core.py` gains `test_moved_modules_left_scripts`: `git ls-files` returns nothing for the four paths, and `git grep` finds no `scripts.head_rgbd_calibration.charuco|intrinsics|extrinsics|urdf_handoff` import in tracked files.
- The whole calibration suite and the CLI tests pass; root `uv run poe lint` passes; `scripts/README.md` module list matches what is left under `scripts/head_rgbd_calibration/` (doc file).

#### Acceptance conditions

- a1: the four re-export modules are untracked and nothing imports them [evidence: test:tests/test_head_rgbd_calib_core.py::test_moved_modules_left_scripts]
- a2: the calibration suite passes [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_rgbd_calib_retired.py -q]
- a3: root lint passes [evidence: command:uv run poe lint]

#### Files

- `scripts/head_rgbd_calibration/charuco.py`
- `scripts/head_rgbd_calibration/intrinsics.py`
- `scripts/head_rgbd_calibration/extrinsics.py`
- `scripts/head_rgbd_calibration/urdf_handoff.py`
- `tests/test_head_rgbd_calib_extrinsics.py`
- `tests/test_head_rgbd_calib_intrinsics.py`
- `tests/test_head_rgbd_urdf_handoff.py`
- `tests/test_head_rgbd_calib_core.py`

#### Interfaces

- No new interface; importers use `ros2_common.head_rgbd_calibration.{charuco,intrinsics,extrinsics,urdf_handoff}`.

#### Expected outcome

`scripts/head_rgbd_calibration/` holds only CLI-side modules (config, frames, pipeline, validate); no transitional re-export remains.

#### Steps

- Add `test_moved_modules_left_scripts`; run: red.
- `git rm` the four re-exports; rewrite the remaining importers.
- Run the verify command: all pass, lint clean.

#### Notes

- TDD exception: deletion pinned by a retirement test.

### S10: Calib node compute job

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`, `nodes/head_rgbd_calibration/tests/test_compute.py`, `nodes/head_rgbd_calibration/tests/test_jobs.py`, `nodes/head_rgbd_calibration/README.md`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py tests/test_jobs.py -q && uv run --frozen poe lint)`
- Verified head: 9f608847e8b258c46cca89d99f6dfde543e3da85

#### Requirement

Compute the calibration on the robot from a session's stored poses in a niced child process, one job at a time (design D5).

- `head_rgbd_calibration/compute.py`:
  - `compute_result(store, sid, rid, cfg, now) -> dict`: `store.load_views`, core `calibrate_detections`, `failed_limits(run, cfg.limits)`, `report_markdown(run, cfg.limits, sid, lens, date)`; writes `results/<rid>/` under a temporary name then renames it: `head_camera_calibration.yaml` and `tof_calibration.yaml` (core `write_camera_calibration_yaml`, camera names `head_camera` and `tof` as the CLI writes), `head_rgbd_extrinsics.yaml` (core `write_extrinsics_yaml` with the same meta as `scripts/head_rgbd_calibration/pipeline.py:write_calibration`), `tof_joint.json` (`{xyz, rpy, note}` from core `read_chain` + `tof_joint_origin`, fed the URDF text read from `cfg.head_urdf_file` before `urdf_handoff.read_chain`), `head.urdf` (core `rewrite_tof_joint` of that file), `report.md`, `result.json` (`id`, `session`, `created`, `passed`, `failed_limits`, `rgb_rms_px`, `tof_rms_px`, `stereo_rms_px`, `plane_bias_mm`, `rgb_views`, `tof_views`, `stereo_pairs`, `lens_position`, `pose_ids`).
  - A run over the limits is written with `passed: false` (its report stays visible). A core `ValueError` (too few views, no ToF camera_info) writes only `result.json` with `passed: false` and `error`; there is no `report.md`, so `GET result` returns `report: null` (the frontend tolerates null).
  - `main(argv)` / `python -m head_rgbd_calibration.compute --config <path> --session <sid> --result <rid>` runs one computation and exits 0 when `result.json` was written.
- `head_rgbd_calibration/jobs.py` `ComputeJob(cfg, store, popen, clock, config_path)` (`config_path` is the node config path taken from the `HEAD_RGBD_CALIBRATION_CONFIG` env var the node reads): `start(sid) -> rid` spawns the child with `--config <config_path>` using `subprocess.Popen` at niceness `cfg.compute_nice`; `Busy` while one runs; `state() -> {running, session_id, result_id, started_at, error}`; kills the child after `compute_timeout_s` (error `timed out after N s`); a non-zero exit without `result.json` reports the last 20 stderr lines as `error`.
- Tests `tests/test_compute.py` (synthetic session written through the store: passing result and files; limits exceeded gives `passed: false`; too few poses gives `error` and no YAMLs; `tof_joint.json` and `head.urdf` agree) and `tests/test_jobs.py` (fake child commands: busy, timeout, failure tail).
- README `## Compute` section (doc file).

#### Acceptance conditions

- a1: a passing session writes every result file [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_passing_session_writes_all_result_files -q]
- a2: a run over the limits is kept with passed false [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_limits_exceeded_marks_not_passed -q]
- a3: too few poses report an error and write no YAMLs [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_too_few_poses_reports_error -q]
- a4: tof_joint.json and the rewritten head.urdf agree [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_compute.py::test_tof_joint_matches_rewritten_urdf -q]
- a5: one job at a time, killed after the timeout, failures report the stderr tail [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_jobs.py -q]
- a6: the child runs niced [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_jobs.py::test_child_is_niced -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/test_jobs.py`
- `nodes/head_rgbd_calibration/README.md`

#### Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py::compute_result(store, sid, rid, cfg, now) -> dict`, `main(argv) -> int`.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/jobs.py::ComputeJob`, `.start(sid) -> str`, `.state() -> dict`, `.stop()`.
- Contract `rgbd-calib-result-files` (owned here): the `results/<rid>/` file set of design D3 and the `result.json` keys.

#### Expected outcome

A compute on a stored session produces a result directory in one rename; the HTTP thread only starts and polls a child process.

#### Steps

- Write `tests/test_compute.py` and `tests/test_jobs.py`; run: red.
- Implement compute and the job runner.
- Run the verify command: all pass, lint clean.

#### Notes

- Memory: load only the session's poses (about 6 MB for 30); never a bag.
- The deployed checkout URDF path comes from `cfg.head_urdf_file`; the node never writes into the checkout.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-session-store

### S11: Calib node live capture

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`, `nodes/head_rgbd_calibration/tests/test_capture.py`, `nodes/head_rgbd_calibration/tests/conftest.py`, `nodes/head_rgbd_calibration/README.md`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py -q && uv run --frozen poe lint)`
- Verified head: f5dc7b623ece788e6e1f86fb4ca5570029351cf4

#### Requirement

Robot-side live capture: subscribe the head topics only while a session runs, with the QoS measured on the robot, and store each usable pose as it is judged (design D4).

- `head_rgbd_calibration/capture.py` `CaptureController(node, store, cfg, clock)`:
  - `start(session_id: str | None)`: creates a session or reopens an existing one (`NotFound` for an unknown id); reads `lens_position` from `cfg.head_camera_config_file` (absent key: `cfg.default_lens_position`; unreadable or invalid file: start fails with the reason and nothing is subscribed); builds the core `PoseCounter` with `start_count` = stored pose count; creates the subscriptions: amplitude and depth `sensor_msgs/Image` RELIABLE KEEP_LAST `qos.tof_depth`, `/tof/camera_info` RELIABLE depth 1, mapping `sensor_msgs/CompressedImage` BEST_EFFORT KEEP_LAST `qos.rgb_depth`; arms a timer that stops the capture after `max_capture_s`. Raises `Busy` while running. Any exception during start destroys whatever was already created.
  - `stop()`: destroys every subscription and the timer it created (subscribing the mapping topic engages the head camera focus lock, so nothing may outlive the capture).
  - Callbacks: decode `mono16` honouring `step` and `is_bigendian` (as `scripts/head_rgbd_calibration/frames.py:decode_mono16` does), `CompressedImage` with `cv2.imdecode`; a frame that does not decode is logged and dropped, never fed; header stamps; feed the counter (RGB with its raw bytes); each `CapturedPose` goes to `store.add_pose`; the first `/tof/camera_info` of a session is stored with `set_tof_info`.
  - `state() -> {running, session_id, status, pose_count, started_at, stops_at}` (status is the counter status; `waiting for frames` before both streams).
  - Thread safety: callbacks run on the executor thread, `state`/`start`/`stop` on HTTP threads; one lock guards shared state; detection and file writes never run in an HTTP handler.
  - Publishes nothing.
- `tests/conftest.py` installs stub `rclpy`, `rclpy.qos`, `sensor_msgs`, `sensor_msgs.msg` modules (pattern: `nodes/web_ui/tests/conftest.py`); `tests/test_capture.py` uses a fake node recording `create_subscription`/`destroy_subscription`/timer calls and synthetic board frames built with the core `make_board`.
- README `## Capture` section (doc file).

#### Acceptance conditions

- a1: ToF amplitude, depth and camera_info are subscribed RELIABLE [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_tof_subscriptions_are_reliable -q]
- a2: the mapping image is subscribed BEST_EFFORT [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_mapping_subscription_best_effort -q]
- a3: stop destroys every subscription and the timer [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_stop_destroys_all_subscriptions -q]
- a4: the capture stops by itself after max_capture_s [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_timeout_stops_capture -q]
- a5: a failed start leaves nothing subscribed [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_failed_start_leaves_nothing_subscribed -q]
- a6: a still board seen by both cameras is stored as a pose [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_still_board_pose_is_stored -q]
- a7: a reopened session continues the pose numbering [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_resume_continues_pose_numbers -q]
- a8: an undecodable frame is dropped, not fed [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_undecodable_frame_dropped -q]
- a9: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `nodes/head_rgbd_calibration/tests/test_capture.py`
- `nodes/head_rgbd_calibration/tests/conftest.py`
- `nodes/head_rgbd_calibration/README.md`

#### Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py::CaptureController(node, store, cfg, clock)`, `.start(session_id: str | None) -> dict`, `.stop() -> dict`, `.state() -> dict`, `Busy`.
- Uses `rgbd-pose-counter`, `rgbd-calib-session-store`, `rgbd-calib-node-config`.

#### Expected outcome

Starting a capture subscribes four topics with the measured QoS; stopping, timing out or a failed start leaves zero subscriptions (focus lock released); every still pose with the board in both cameras is on disk within one judgement.

#### Steps

- Write `tests/conftest.py` stubs and `tests/test_capture.py`; run: red.
- Implement the controller.
- Run the verify command: all pass, lint clean.

#### Notes

- Constraints measured on the robot: ToF topics RELIABLE at the publisher, BEST_EFFORT loses about half the frames under load; mapping topic BEST_EFFORT 2 Hz and any subscriber engages the focus lock (`nodes/head_camera_cpp/src/head_camera_node.cpp:280`).
- The node units already set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.
- Model opus: thread safety between the executor and HTTP threads.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-session-store

### S12: Calib node apply download

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`, `nodes/head_rgbd_calibration/tests/test_apply.py`, `nodes/head_rgbd_calibration/README.md`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py -q && uv run --frozen poe lint)`
- Verified head: 4c4b01889812424747cd29ce1363e47bb11f8d7a

#### Requirement

Apply a passing result to the robot atomically and package a result for download (design D6).

- `head_rgbd_calibration/apply.py`:
  - `APPLIED_FILES = ("head_camera_calibration.yaml", "tof_calibration.yaml", "head_rgbd_extrinsics.yaml", "tof_joint.json")` and `APPLIED_MARKER = "applied.json"` (the contract the Ansible path unit and the head nodes read).
  - `apply_result(store, sid, rid, now) -> dict`: refuses with `NotApplicable` unless `result.json` has `passed: true`; copies each of `APPLIED_FILES` into `<state_dir>/applied/` through a temporary file and `os.replace`, then writes `applied.json` (`session`, `result`, `applied_at`, `files`) last, also through `os.replace`.
  - `applied_state(store, units, run) -> dict | None`: `applied.json` plus per unit `{name, active_state, restarted}`, where `restarted` means the unit's `ActiveEnterTimestamp` (read with `systemctl show --timestamp=unix -p ActiveEnterTimestamp -p ActiveState <unit>`, unprivileged, through the injectable `run`) is not older than `applied_at`; `--timestamp=unix` values have the form `@<sec>` and the leading `@` is stripped before parsing; None when nothing is applied.
  - `download_zip(store, sid, rid) -> tuple[str, bytes]`: file name `<sid>-<rid>.zip` and a zip holding the three YAMLs, `tof_joint.json`, `head.urdf` and `report.md`; `NotApplicable` when the result has no YAMLs.
- Tests `tests/test_apply.py`: a passing result is applied with every file and `applied.json` written last; a failing result is refused and `applied/` is unchanged; an `os.replace` failure mid-apply leaves the previous `applied.json` untouched; restart flags from a fake `systemctl`; zip member names and content.
- README `## Apply and download` section (doc file).

#### Acceptance conditions

- a1: a passing result is applied with applied.json written last [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_apply_writes_marker_last -q]
- a2: a failing result is refused and applied/ is unchanged [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_failing_result_refused -q]
- a3: a failure mid-apply keeps the previous marker [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_interrupted_apply_keeps_previous_marker -q]
- a4: per-unit restart state is reported from systemctl show [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_units_restarted_after_apply -q]
- a5: the download zip holds the YAMLs, tof_joint.json, head.urdf and report.md [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_apply.py::test_download_zip_members -q]
- a6: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/tests/test_apply.py`
- `nodes/head_rgbd_calibration/README.md`

#### Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py::APPLIED_FILES`, `APPLIED_MARKER`, `apply_result`, `applied_state`, `download_zip`, `NotApplicable`.
- Contract `rgbd-calib-applied-files` (owned here): `<state_dir>/applied/{head_camera_calibration.yaml,tof_calibration.yaml,head_rgbd_extrinsics.yaml,tof_joint.json,applied.json}`; `tof_joint.json` is `{xyz: [3], rpy: [3], note}`.

#### Expected outcome

Apply on a passing result swaps the applied files atomically and touches `applied.json` last (the path unit trigger); the node can tell when each head unit restarted.

#### Steps

- Write `tests/test_apply.py`; run: red.
- Implement apply, state and zip.
- Run the verify command: all pass, lint clean.

#### Notes

- `systemctl show` needs no privilege; never call `systemctl restart` from the node (design D6).

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-compute-job

### S13: Calib node HTTP API

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`, `nodes/head_rgbd_calibration/tests/test_app.py`, `nodes/head_rgbd_calibration/tests/test_main.py`, `nodes/head_rgbd_calibration/README.md`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests -q && uv run --frozen poe lint)`
- Verified head: 703cb6f871b39239b5eda3a569c7b0d3cfa97889

#### Requirement

The node's localhost HTTP API and process entry point (design D1, D5, Contracts table "Node HTTP API").

- `head_rgbd_calibration/app.py` `build_app(capture, job, store, cfg, run) -> FastAPI` with exactly the routes of the design's Node HTTP API table; every error body is `{ok: false, message}`; `InvalidId`/`NotFound` give 404, `Busy`/`NotApplicable` give 409; capture start is refused while compute runs and compute while capture runs (409 naming the other job); compute is refused when the session has fewer poses than `limits.min_views` (409 naming both counts); thumbnails served as `image/jpeg`; download as `application/zip` with `Content-Disposition: attachment; filename="<sid>-<rid>.zip"`; `GET /api/applied` 404 when nothing is applied; `GET /api/sessions/{sid}` includes `coverage` from the store; `GET /api/state` carries `applied.json` only (no systemctl call), per-unit restart state appears only on `GET /api/applied`.
- `head_rgbd_calibration/__main__.py`: loads the config from `HEAD_RGBD_CALIBRATION_CONFIG`; `rclpy.init`; one node `head_rgbd_calibration`; `MultiThreadedExecutor` spinning in a daemon thread; uvicorn bound to `cfg.http_host:cfg.http_port` (127.0.0.1 by default, never 0.0.0.0); on shutdown stops any capture, kills a running compute child and destroys the node.
- Tests `tests/test_app.py` (TestClient with fake capture, job and a tmp store) and `tests/test_main.py` (bind host and port come from the config; shutdown stops capture).
- README `## HTTP API` section listing every route (doc file).

#### Acceptance conditions

- a1: GET /api/state reports capture, compute and applied [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_state_shape -q]
- a2: capture and compute exclude each other with 409 [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_capture_and_compute_exclusive -q]
- a3: compute needs limits.min_views poses [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_compute_needs_min_views -q]
- a4: unknown and traversal ids answer 404 [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_bad_ids_are_404 -q]
- a5: thumbnails and the zip are served with their content types [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_binary_routes_content_types -q]
- a6: applying a failing result answers 409 [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_app.py::test_apply_failing_result_409 -q]
- a7: the server binds the configured localhost address and shutdown stops the capture [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_main.py -q]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]
- a9: the README lists the HTTP API [evidence: doc-section:nodes/head_rgbd_calibration/README.md#HTTP API]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`
- `nodes/head_rgbd_calibration/tests/test_main.py`
- `nodes/head_rgbd_calibration/README.md`

#### Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py::build_app(capture, job, store, cfg, run) -> FastAPI`; routes exactly as the design's Node HTTP API table.
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__main__.py::main() -> None`.
- Contract `rgbd-calib-node-http-api` (owned here; consumed by web-ui-rgbd-calib-proxy and calib-node-ansible-deploy).

#### Expected outcome

`python3 -m head_rgbd_calibration` serves the API on 127.0.0.1:18110 and spins the ROS node; the whole node suite passes.

#### Steps

- Write `tests/test_app.py` and `tests/test_main.py`; run: red.
- Implement the app and the entry point; document the routes.
- Run the verify command: all pass, lint clean.

#### Notes

- Error body shape `{ok: false, message}` matches web_ui's `imu_calibration.error`.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-live-capture, calib-node-compute-job, calib-node-apply-download

### S14: Head URDF tof joint override

- Status: done
- Files: `nodes/head_state_publisher/launch/head_state_publisher.launch.py`, `nodes/head_state_publisher/launch/head_description.py`, `tests/test_head_tof_joint_override.py`, `tests/test_head_state_publisher_config.py`, `ansible/group_vars/client.yml`, `nodes/head_state_publisher/README.md`
- Verify: `uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_state_publisher_config.py -q && uv run poe lint`
- Verified head: 1479e10d1998f57cbd6ffcfc1920c13485299a86

#### Requirement

head_state_publisher uses an applied tof_joint without editing the repo URDF on the robot (design D8).

- New `nodes/head_state_publisher/launch/head_description.py`, standard library only: `robot_description(urdf_path: Path, override_path: Path, urdf_text_path: Path) -> str`. Reads the URDF; when `override_path` exists and holds JSON `{xyz: [3 numbers], rpy: [3 numbers], note: str}`, loads `urdf_text.py` by file path (`importlib.util.spec_from_file_location`, no package import) and returns `rewrite_tof_joint(urdf, Origin(xyz, rpy), note)`; a missing file returns the URDF unchanged; unreadable JSON or a wrong shape returns the URDF unchanged and logs a warning naming the file and the reason.
- `head_state_publisher.launch.py` loads `head_description` by path from its own directory and builds `robot_description` from `HEAD_URDF_PATH`, `TOF_JOINT_OVERRIDE = /var/lib/ros2/head_rgbd_calibration/applied/tof_joint.json` and `URDF_TEXT_PATH` (`<repo>/shared/ros2_common/head_rgbd_calibration/urdf_text.py`, resolved relative to the launch file).
- `ansible/group_vars/client.yml`: the head_state_publisher node type `src_paths` adds `shared/ros2_common/head_rgbd_calibration/urdf_text.py` (a change to it restarts the node on deploy); `tests/test_head_state_publisher_config.py` updated to match.
- New `tests/test_head_tof_joint_override.py`: override applied (tof_joint origin carries the JSON values, every other line identical); missing file gives the repo URDF; invalid JSON and wrong shape give the repo URDF plus a warning; `head_description.py` imports only `sys.stdlib_module_names`; the launch file references `head_description`, the applied path and `urdf_text.py`.
- `nodes/head_state_publisher/README.md` documents the override (doc file); `tests/README.md` updated.

#### Acceptance conditions

- a1: an applied tof_joint.json rewrites only the tof_joint origin [evidence: test:tests/test_head_tof_joint_override.py::test_override_rewrites_only_tof_joint]
- a2: without an override the repo URDF is published unchanged [evidence: test:tests/test_head_tof_joint_override.py::test_missing_override_keeps_repo_urdf]
- a3: an invalid override is ignored with a warning [evidence: test:tests/test_head_tof_joint_override.py::test_invalid_override_ignored_with_warning]
- a4: head_description is standard library only [evidence: test:tests/test_head_tof_joint_override.py::test_head_description_is_stdlib_only]
- a5: the launch file wires the override and the node type restarts on urdf_text changes [evidence: command:uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_state_publisher_config.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

#### Files

- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `nodes/head_state_publisher/launch/head_description.py`
- `tests/test_head_tof_joint_override.py`
- `tests/test_head_state_publisher_config.py`
- `ansible/group_vars/client.yml`
- `nodes/head_state_publisher/README.md`

#### Interfaces

- `nodes/head_state_publisher/launch/head_description.py::robot_description(urdf_path: Path, override_path: Path, urdf_text_path: Path) -> str`.
- Reads contract `rgbd-calib-applied-files` (`tof_joint.json`); loads `shared/ros2_common/head_rgbd_calibration/urdf_text.py` by path.

#### Expected outcome

After an Apply and the path-unit restart, `/tf_static` carries the calibrated tof_joint; without an applied file the repo URDF is published unchanged.

#### Steps

- Write `tests/test_head_tof_joint_override.py` and update `tests/test_head_state_publisher_config.py`; run: red.
- Implement `head_description.py`, wire the launch file, extend `src_paths`.
- Run the verify command: all pass, lint clean.

#### Notes

- The launch runs on the system Python (no node venv): stdlib only.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-apply-download

### S15: Calib node Ansible deploy

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`, `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.path.j2`, `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`, `tests/test_head_rgbd_calibration_deploy.py`, `ansible/README.md`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py tests/test_head_state_publisher_config.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: b34e4684b51bc7704dfec225b42cb8638688336c

#### Requirement

Deploy the calibration node, its state dir and the apply restart unit with the standard script; nothing a deploy does may overwrite a session or an applied calibration (design D1, D6, D7).

- `ansible/group_vars/client.yml` `ros2_node_type_defaults.head_rgbd_calibration`: `deploy_mode: native`, `node_launch_command: "python3 -m head_rgbd_calibration"`, `node_src_dir: nodes/head_rgbd_calibration`, `src_shared: true`, `memory_low: "64M"`, `memory_max: "768M"`, `cpu_weight: 200`, `io_weight: 200`, `oom_score_adjust: 0`, `config_path: /etc/ros2/head_rgbd_calibration`, `env: [HEAD_RGBD_CALIBRATION_CONFIG=/etc/ros2/head_rgbd_calibration/config.yaml]`, with a comment explaining the tier (interactive operator tool below the robot tiers; compute child niced).
- `ros2_nodes` entry `head_rgbd_calibration` (node_type, `present: true`, `enabled: true`, `config: |` equal to `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml` values, commented), placed after `head_rgbd_cpp`.
- New `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml` (pattern `bno055_state_dir.yml`): `/var/lib/ros2/head_rgbd_calibration`, `sessions/`, `applied/` owned by `ansible_user`, mode 0755; for each committed repo calibration file (`nodes/head_camera_cpp/config/head_camera_calibration.yaml`, `nodes/head_rgbd_cpp/config/tof_calibration.yaml`, `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml` in `{{ ros2_repo_dest }}`) a `stat` and a copy into `applied/` with `remote_src: true`, `force: false`, only when the source exists (the seed source files do not exist in the repo yet: `nodes/head_camera_cpp/config/` is absent and `nodes/head_rgbd_cpp/config/` holds only `head_rgbd.yaml`, so the stat guard skips them until they arrive with the head-rgb-d-fusion epic; the existing head_rgbd_cpp `config_files` entry is at `ansible/group_vars/client.yml:631`); templates `ros2-head-rgbd-calibration-apply.path` (`PathChanged=/var/lib/ros2/head_rgbd_calibration/applied/applied.json`, `Unit=ros2-head-rgbd-calibration-apply.service`, `WantedBy=multi-user.target`) and `ros2-head-rgbd-calibration-apply.service` (`Type=oneshot`, `ExecStart=/usr/bin/systemctl try-restart ros2-head_camera_cpp.service ros2-head_rgbd_cpp.service ros2-head_state_publisher.service`) from `ansible/playbooks/templates/`, daemon-reload on change, path unit enabled and started. No task removes, truncates or overwrites anything under `sessions/` or `applied/`.
- `ansible/playbooks/deploy_nodes_client.yml`: the setup include (tags `head_rgbd_calibration`, `setup`) before a "Deploy head_rgbd_calibration" step through `tasks/resolve_and_deploy.yml` (pattern of topic_scraper_api).
- New `tests/test_head_rgbd_calibration_deploy.py`: node type keys and tier; entry present, enabled, after head_rgbd_cpp, config equal to the node defaults file; state dir ownership and modes; every seed copy has `force: false` and `remote_src: true` and a source-exists condition; no `state: absent` or `force: true` task targets the state dir; path and service unit contents; the setup include precedes the deploy step with both tags; port 18110 appears once among client.yml `http_port`/`port`/`metrics_port` values.
- `uv run poe lint-ansible` and `uv run poe test-ansible` pass; `ansible/README.md` documents the node, the state dir, the seeding rule and the apply unit (doc file).

#### Acceptance conditions

- a1: the node type and its tier match the design [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_node_type_and_tier]
- a2: the ros2_nodes entry is present, enabled, after head_rgbd_cpp, with the node defaults [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a3: seeding never overwrites an applied file [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_seed_copies_never_overwrite]
- a4: no deploy task removes or overwrites the state dir [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_no_task_clears_state_dir]
- a5: the path unit watches applied.json and the service restarts exactly the three head units [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_apply_units_content]
- a6: the playbook runs the setup before deploying the node [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_playbook_setup_before_deploy]
- a7: the HTTP port is unique on the client [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_http_port_unique]
- a8: ansible lint and syntax checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.path.j2`
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`
- `tests/test_head_rgbd_calibration_deploy.py`
- `ansible/README.md`

#### Interfaces

- `ansible/group_vars/client.yml` node type and entry `head_rgbd_calibration`; `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`; units `ros2-head-rgbd-calibration-apply.path` and `.service`.
- Uses contracts `rgbd-calib-applied-files` (watched marker) and `rgbd-calib-node-http-api` (port 18110).

#### Expected outcome

`./scripts/deploy-nodes.sh client head_rgbd_calibration` installs the node, the state dir and the apply units; a redeploy never touches sessions or an applied calibration.

#### Steps

- Write `tests/test_head_rgbd_calibration_deploy.py`; run: red.
- Add the node type, entry, setup tasks, templates and playbook steps; document.
- Run the verify command: all pass, `uv run poe test-ansible` clean.

#### Notes

- Patterns: `ansible/playbooks/tasks/bno055_state_dir.yml`, `tasks/mcp_server_setup.yml`, topic_scraper_api deploy step, `playbooks/templates/*.j2`.
- Memory note "cgroups guarantee headroom": robot nodes keep their weights; this node sits below them.
- Deploy is done by the orchestrator after merge.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-http-api, head-urdf-tof-joint-override

### S16: Web UI RGBD calib proxy

- Status: done
- Files: `nodes/web_ui/web_ui/rgbd_calibration.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/imu_calibration.py`, `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/tests/test_rgbd_calibration_proxy.py`, `nodes/web_ui/tests/test_imu_calibration.py`, `nodes/web_ui/README.md`
- Verify: `(cd nodes/web_ui && uv run --frozen pytest tests/test_rgbd_calibration_proxy.py tests/test_imu_calibration.py tests/test_config.py -q && uv run --frozen poe lint)`
- Verified head: 74737f80df6dac343ea18804c87e5fa36373a28b

#### Requirement

web_ui forwards the RGB-D calibration calls to the node; the IMU section is unaffected (design D9).

- `nodes/web_ui/web_ui/config.py` `CalibrationConfig`: `rgbd_url: str | None = None` (an `http://` or `https://` URL; absent means no RGB-D section) and `rgbd_timeout_s: float = Field(10.0, gt=0)`.
- `nodes/web_ui/web_ui/imu_calibration.py` `get_config` answers `{"imu": True, "rgbd": <rgbd_url is set>, "arm": "deferred"}`.
- New `nodes/web_ui/web_ui/rgbd_calibration.py` `register_rgbd_calibration_routes(app, config)`: one route `/api/calibration/rgbd/{path:path}` for GET, POST, DELETE forwarding method, query and body to `<rgbd_url>/api/<path>` with `httpx.AsyncClient` (timeout `rgbd_timeout_s`; client lifecycle tied to the app); status code, `content-type`, `content-disposition` and body bytes pass through unchanged (JPEG thumbnails and zip downloads included); binary bodies are buffered, not streamed (the zip is a few MB); on POST the request `content-type` header is forwarded; a path not matching `^[A-Za-z0-9_.\-/]+$` or holding a `..` segment answers 400; no `rgbd_url` answers 404 `{ok: false, message: "RGB-D calibration not configured"}`; connect errors and timeouts answer 503 `{ok: false, message: "calibration node unreachable"}`. Registered in `server.py:build_app` before the SPA mount. Pattern: `agent_proxy.py`.
- New `nodes/web_ui/tests/test_rgbd_calibration_proxy.py` (httpx `MockTransport` or a patched client): config flag on and off; GET, POST, DELETE forwarded with path, query and body; binary bodies and headers passed through; POST content-type forwarded; traversal rejected; unreachable gives 503; unconfigured gives 404. `tests/test_imu_calibration.py` asserts the new `rgbd` flag and every existing IMU test passes unchanged otherwise.
- `nodes/web_ui/README.md` Calibrate section documents `rgbd_url`, `rgbd_timeout_s` and the forwarded routes, and lists the new test (doc file).

#### Acceptance conditions

- a1: /api/calibration/config reports the rgbd flag from rgbd_url [evidence: test:nodes/web_ui/tests/test_imu_calibration.py::test_config_reports_rgbd_flag]
- a2: GET, POST and DELETE are forwarded with path, query and body [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_forwards_method_path_query_body]
- a3: JPEG and zip bodies pass through with their headers [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_binary_bodies_and_headers_pass_through]
- a4: traversal paths are rejected [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_rejects_traversal_path]
- a5: an unreachable node answers 503 with a message [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_unreachable_node_is_503]
- a6: without rgbd_url the routes answer 404 [evidence: test:nodes/web_ui/tests/test_rgbd_calibration_proxy.py::test_not_configured_is_404]
- a7: the whole web_ui backend suite and lint pass (IMU unchanged) [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/rgbd_calibration.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/imu_calibration.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/tests/test_rgbd_calibration_proxy.py`
- `nodes/web_ui/tests/test_imu_calibration.py`
- `nodes/web_ui/README.md`

#### Interfaces

- `nodes/web_ui/web_ui/rgbd_calibration.py::register_rgbd_calibration_routes(app: FastAPI, config: AppConfig) -> None`.
- `nodes/web_ui/web_ui/config.py::CalibrationConfig.rgbd_url`, `.rgbd_timeout_s`.
- Contract `rgbd-calib-web-api` (owned here): `/api/calibration/rgbd/<rest>` maps to node `/api/<rest>`; `/api/calibration/config` carries `rgbd`.

#### Expected outcome

With `rgbd_url` set the page's RGB-D calls reach the node and binary bodies arrive intact; IMU calls behave exactly as before.

#### Steps

- Write `tests/test_rgbd_calibration_proxy.py` and the `rgbd` flag test; run: red.
- Implement config, flag, proxy module, registration; update the README.
- Run the verify command: all pass, lint clean.

#### Notes

- Pattern: `nodes/web_ui/web_ui/agent_proxy.py` (httpx, JSON error bodies). httpx is already a web_ui dependency.
- web_ui subscribes nothing for this section (design D1).

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-http-api

### S17: Head nodes use applied calib

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_head_rgbd_deploy.py`, `tests/test_head_camera_deploy.py`, `tests/test_web_ui_calibration_config.py`, `nodes/head_rgbd_cpp/README.md`, `nodes/head_camera_cpp/README.md`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_deploy.py tests/test_head_camera_deploy.py tests/test_web_ui_calibration_config.py tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: d79014dd166f1dab09c4b3c76b6d9b60ddadb51b

#### Requirement

The head nodes read the robot's applied calibration and the Calibrate page gets its RGB-D section; deploys keep it (design D7, D9, D11; cross-epic contract `head-rgbd-applied-calibration`).

- `ansible/group_vars/client.yml`:
  - head_camera_cpp `config`: `camera_info_file` (not set today, added): /var/lib/ros2/head_rgbd_calibration/applied/head_camera_calibration.yaml`, commented (written by Apply on the Calibrate page, seeded from the committed file when absent; missing file: no camera_info, as today).
  - head_rgbd_cpp `config`: `tof_calibration_file` (client.yml:1346; line 1330 is a comment): /var/lib/ros2/head_rgbd_calibration/applied/tof_calibration.yaml`; the head_rgbd_cpp node type `config_files` entry and its comment are removed (the seed in calib-node-ansible-deploy replaces it).
  - web_ui `config` `calibration:` gains `rgbd_url: http://127.0.0.1:18110`; the IMU keys stay.
- Tests: `tests/test_head_rgbd_deploy.py` (config equals the defaults plus `metrics_port` plus the applied `tof_calibration_file`; no `config_files`), `tests/test_head_camera_deploy.py` (applied `camera_info_file`), `tests/test_web_ui_calibration_config.py` (`rgbd_url` port equals the head_rgbd_calibration entry's `http_port`; IMU assertions unchanged).
- `nodes/head_camera_cpp/README.md` and `nodes/head_rgbd_cpp/README.md` say where the calibration comes from (the applied state paths under `/var/lib/ros2/head_rgbd_calibration/applied`, not `/etc/ros2/...`; Calibrate page; seeding) (doc files).
- `uv run poe test-ansible` passes.

#### Acceptance conditions

- a1: head_rgbd_cpp reads the applied ToF calibration and no longer copies a repo file [evidence: test:tests/test_head_rgbd_deploy.py::test_tof_calibration_file_is_the_applied_file]
- a2: head_camera_cpp reads the applied camera calibration [evidence: test:tests/test_head_camera_deploy.py::test_camera_info_file_is_the_applied_file]
- a3: web_ui points the RGB-D section at the node port and keeps the IMU names [evidence: test:tests/test_web_ui_calibration_config.py::test_web_ui_rgbd_url_points_at_calibration_node]
- a4: the IMU wiring test still passes [evidence: test:tests/test_web_ui_calibration_config.py::test_web_ui_enables_the_imu_calibration_section]
- a5: both node READMEs name the applied state paths instead of /etc/ros2/... [evidence: command:grep -q "head_rgbd_calibration/applied" nodes/head_rgbd_cpp/README.md && grep -q "head_rgbd_calibration/applied" nodes/head_camera_cpp/README.md && ! grep -q "/etc/ros2/head_rgbd_cpp/tof_calibration" nodes/head_rgbd_cpp/README.md]
- a6: ansible lint and syntax checks pass [evidence: command:uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_head_rgbd_deploy.py`
- `tests/test_head_camera_deploy.py`
- `tests/test_web_ui_calibration_config.py`
- `nodes/head_rgbd_cpp/README.md`
- `nodes/head_camera_cpp/README.md`

#### Interfaces

- `ansible/group_vars/client.yml`: head_camera_cpp `camera_info_file`, head_rgbd_cpp `tof_calibration_file`, web_ui `calibration.rgbd_url`.
- Owns cross-epic contract `head-rgbd-applied-calibration` (consumer: head-rgb-d-fusion-of-camera, task rgbd-final-suite-and-runbook).

#### Expected outcome

After deploy the head nodes read the applied files; the Calibrate page shows the RGB-D section next to IMU.

#### Steps

- Update the three root tests to the new values; run: red.
- Edit client.yml; update the two node READMEs.
- Run the verify command: all pass, `uv run poe test-ansible` clean.

#### Notes

- The in-progress task rgbd-final-suite-and-runbook greps `head_camera_calibration.yaml` in client.yml; the applied path keeps that true (design D11).
- No C++ change: both nodes already read these keys at startup.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-ansible-deploy, web-ui-rgbd-calib-proxy

### S18: RGBD calib frontend API

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`, `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`
- Verify: `(cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts && npx tsc --noEmit && npm run lint)`
- Verified head: d561360052b983f3b235f7437d2655955d70d66e

#### Requirement

A typed, tested frontend client for every RGB-D calibration call (design D10, Contracts `rgbd-calib-web-api`).

- New `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts` following `imuApi.ts` (`call<T>(url, init, validator)` returning `{ok: true, value} | {ok: false, status, message}` with the server `message` passed through):
  - types `CaptureState`, `ComputeState`, `RgbdState`, `SessionSummary`, `PoseSummary`, `Coverage` (`{cols, rows, counts: number[][]}`), `SessionDetail`, `ResultSummary`, `ResultDetail` (with `report`), `UnitRestart`, `AppliedState`;
  - `getState`, `startCapture(sessionId?)`, `stopCapture`, `listSessions`, `getSession`, `deletePose`, `computeSession`, `getResult`, `applyResult`, `getApplied` against `/api/calibration/rgbd/...` (paths of the design's Node HTTP API table with the `/api` prefix replaced);
  - `thumbnailUrl(sid, pid, kind: 'rgb' | 'tof')` (maps to the node routes `rgb.jpg` / `tof.jpg`) and `downloadUrl(sid, rid)` with every id `encodeURIComponent`-ed;
  - pure helpers `emptyCells(coverage): number` and `statusSeverity(status): 'success' | 'info' | 'warning'` (captured is success, moving and waiting info, the three `still - ...` messages warning).
- Validators reject wrong shapes (a 200 with a malformed body gives `ok: false`).
- New `rgbdApi.test.ts`: URL, method and body of every call; validator failure; server message passed through; helpers.

#### Acceptance conditions

- a1: every call uses the right URL, method and body and validates the response [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts]
- a2: the client compiles and lints clean [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]
- a3: the API client module exists [evidence: symbol:nodes/web_ui/frontend/src/calibrate/rgbdApi.ts::startCapture]

#### Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`

#### Interfaces

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`: the calls, types and helpers in the requirement.
- Contract `rgbd-calib-frontend-api` (owned here).

#### Expected outcome

Every component task imports its calls and types from one module; malformed responses never reach the UI as data.

#### Steps

- Write `rgbdApi.test.ts`; `npx vitest run src/calibrate/rgbdApi.test.ts`: red.
- Implement `rgbdApi.ts`.
- Run the verify command: tests pass, tsc and eslint clean.

#### Notes

- Pattern: `src/calibrate/imuApi.ts` and its test.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on web-ui-rgbd-calib-proxy

### S19: RGBD capture section UI

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`, `nodes/web_ui/frontend/src/calibrate/CoverageGrid.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`, `nodes/web_ui/README.md`
- Verify: `(cd nodes/web_ui/frontend && npx vitest run src/calibrate src/layout && npx tsc --noEmit && npm run lint)`
- Verified head: 7e074dd837064278cdf0671b3448b5a4fec8ad80

#### Requirement

The capture part of the RGB-D section: live status, pose count, thumbnails with corners, delete, and coverage (design D10). The IMU section keeps working.

- `CalibratePage.tsx`: `CalibrationConfig` gains `rgbd: boolean` (`body.rgbd === true`); renders `ImuCalibration` when `imu` and the new `RgbdCalibration` when `rgbd`.
- New `RgbdCalibration.tsx`: section root holding the open session id; calls `getState` on mount and owns `{capture, compute}`; passes them to `RgbdCapture` (and later `RgbdSessions`) as props; runs the 1000 ms poll (visible document only) while a capture or compute is running, so a page reload mid-capture or a Resume capture from `RgbdSessions` keeps polling; renders `RgbdCapture`.
- New `RgbdCapture.tsx`: Start capture (new session) and Stop buttons (`TOUCH_TARGET_PX`); while a capture runs, an `Alert` with the status line (severity from `statusSeverity`), the pose count and the auto-stop time; reads the capture state from props (the poll itself lives in `RgbdCalibration`: every 1000 ms only while a capture or compute runs and `document.visibilityState === 'visible'`, `visibilitychange` listener, interval cleared when both stop and on unmount); when `pose_count` changes the session detail is refetched; a thumbnail grid of the open session (RGB and ToF `img` per pose via `thumbnailUrl`, pose id, corner counts, a depth marker) with a delete `IconButton` opening `layout/ConfirmDialog`, then `deletePose` and a refresh; API errors shown in an error `Alert` with the server message.
- New `CoverageGrid.tsx`: RGB and ToF grids (cells shaded by corner count, empty cells marked) and the text `N of M image areas without corners` from `emptyCells`.
- Tests (`// @vitest-environment jsdom`, fetch stubbed with `vi.stubGlobal`): new `RgbdCapture.test.tsx` (a reload mid-capture resumes polling from the mounted state; start shows the status and count; a new pose's thumbnails appear; delete through the dialog calls DELETE and refreshes; polling stops on stop and while hidden; coverage hint text) and `CalibratePage.test.tsx` (both sections with `{imu: true, rgbd: true}`, IMU only with `rgbd: false`; existing assertions kept).
- `src/layout/designRules.test.ts` and `hardcodedSizes.test.ts` stay green; `nodes/web_ui/README.md` test list updated (doc file).

#### Acceptance conditions

- a1: the capture section shows live status, count and thumbnails and deletes through the dialog [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx]
- a2: the page renders the IMU and RGB-D sections from the config flags [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx src/calibrate/ImuCalibration.test.tsx src/calibrate/CalibrateRouting.test.tsx]
- a3: design rules still hold [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/layout/designRules.test.ts src/layout/hardcodedSizes.test.ts]
- a4: the frontend compiles and lints clean [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CoverageGrid.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `RgbdCalibration` (props: none; owns `{capture, compute}` and the poll), `RgbdCapture` (props: `sessionId: string | null`, `onSession(id)`, `capture`, `onStarted()`), `CoverageGrid` (props: `coverage: Coverage`, `label`).
- `CalibratePage` `CalibrationConfig { imu: boolean; rgbd: boolean }`.

#### Expected outcome

On the robot the operator presses Start, sees `waiting for frames`, then `moving` / `still - ...` / `pose N captured - move to the next pose`, the count rising and thumbnails with corners appearing, and can delete a pose.

#### Steps

- Write `RgbdCapture.test.tsx` and extend `CalibratePage.test.tsx`; run: red.
- Implement the components.
- Run the verify command: tests pass, tsc and eslint clean.

#### Notes

- Poll-on-press pattern and visibility handling: `ImuCalibration.tsx`; dialogs: `src/layout/ConfirmDialog.tsx`; sizes: `TOUCH_TARGET_PX`.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on rgbd-calib-frontend-api

### S20: RGBD sessions and results UI

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/RgbdSessions.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdSessions.test.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`, `nodes/web_ui/README.md`
- Verify: `(cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint)`
- Verified head: ea69da2f8a32658eabe1debf9ac4dac17573efb9

#### Requirement

Sessions and results in the RGB-D section: reopen any session, compute, read the report, apply a passing result, download (design D10, D6).

- New `RgbdSessions.tsx`: the session list (id, created, pose and result counts, newest first); Open sets the open session in `RgbdCalibration` (its thumbnails and coverage show through `RgbdCapture`); Resume capture calls `startCapture(id)`; Compute calls `computeSession(id)`, disabled while a capture or a compute runs, and shows a 409 server message; `RgbdSessions` receives `{capture, compute}` from `RgbdCalibration` as props (the poll is owned there and runs while a capture or compute runs); the result list refreshes when the compute in the props ends (an error from the job is shown); after Resume capture it calls `onStarted` so the root polls.
- New `RgbdResults.tsx`: results of the open session newest first: a passed or failed chip; a table of RGB, ToF and stereo RMS, plane bias and view counts next to their limits; failed limits listed; `report` as preformatted text in a collapsible; Apply only for passed results, through `ConfirmDialog` naming the restart of head_camera_cpp, head_rgbd_cpp and head_state_publisher, then `getApplied` polled every 2000 ms for at most 60 s until every unit is restarted, showing each unit's state; Download as an anchor to `downloadUrl` with the `download` attribute.
- `RgbdCalibration.tsx` renders `RgbdSessions` and `RgbdResults` next to `RgbdCapture`.
- Tests (jsdom, fetch stubbed): new `RgbdSessions.test.tsx` (open, resume, compute disabled while busy, 409 message, refresh at the end) and `RgbdResults.test.tsx` (table and chip, Apply hidden for failed, Apply through the dialog then per-unit restart state, Download href).
- `nodes/web_ui/README.md` describes the RGB-D section and lists the tests (doc file).

#### Acceptance conditions

- a1: sessions can be reopened, resumed and computed, with busy and 409 handling [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdSessions.test.tsx]
- a2: results show limits, apply passed ones through the dialog with restart state, and link the download [evidence: command:cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx]
- a3: the whole frontend suite, typecheck and lint pass [evidence: command:cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/calibrate/RgbdSessions.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdSessions.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `RgbdSessions` (props: `openSessionId`, `onOpen(id)`, `captureRunning`, `compute`, `onStarted()`), `RgbdResults` (props: `sessionId`).

#### Expected outcome

From the page the operator reopens an old session, recomputes, reads the report, applies a passing result and sees the three head nodes restart, or downloads the zip.

#### Steps

- Write `RgbdSessions.test.tsx` and `RgbdResults.test.tsx`; run: red.
- Implement the components and wire them into `RgbdCalibration`.
- Run the verify command: whole frontend suite, tsc and eslint clean.

#### Notes

- Apply is offered only for `passed: true` results (design D12).

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on rgbd-capture-section-ui

### S21: RGBD calib runbook and docs

- Status: done
- Files: `docs/runbooks/head-rgbd-fusion.md`, `docs/head-rgbd-fusion-methodology.md`, `scripts/README.md`, `MEMORY.md`
- Verify: `grep -q 'Calibrate page' docs/runbooks/head-rgbd-fusion.md && grep -q 'head_rgbd_calibration/applied' MEMORY.md`
- Verified head: d31b3933d830c732cee90092430ecf6cef88bfd3

#### Requirement

Document the robot-side calibration flow and the decisions (design D11). Prose task.

- `docs/runbooks/head-rgbd-fusion.md` steps 2 and 3 describe the web UI flow: ToF warm-up and board check as today; open the hamburger menu, Calibrate; Start capture; 20-30 poses 0.25-0.45 m from the head, resting the board about 3 s each, tilts and image corners; watch the status line, the pose count and the RGB and ToF coverage grids; delete bad poses; Stop; Compute; read the report against the limits (RGB <= 0.5 px, ToF <= 0.3 px, ToF-to-RGB extrinsic RMS <= 1.0 px, plane bias <= 15 mm) and recapture or prune when it fails; Apply (restarts head_camera_cpp, head_rgbd_cpp, head_state_publisher; the section shows each unit restarted); confirm `/head_rgbd/depth/image_raw` publishes at the mapping rate with a subscriber and that the focus is continuous again once the capture stopped; optionally Download and commit the zip's files to `nodes/head_camera_cpp/config/head_camera_calibration.yaml`, `nodes/head_rgbd_cpp/config/tof_calibration.yaml`, `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml` and `nodes/web_ui/urdf/head.urdf` (they seed `applied/` on a reflashed robot). Only steps 2 and 3 are edited; the rest of the runbook (steps 1 and 4-8) belongs to the fusion epic's rgbd-final-suite-and-runbook and is left as is.
- `docs/head-rgbd-fusion-methodology.md`: the capture description points to the robot-side flow and the state dir.
- `scripts/README.md`: the CLI is for bags, validation captures and offline calibration; robot capture is the Calibrate page.
- `MEMORY.md`: one decision line: head RGB-D calibration is captured, computed and applied on the robot (`/var/lib/ros2/head_rgbd_calibration/applied`, Apply restarts the head nodes; repo files only seed it).

#### Acceptance conditions

- a1: the runbook calibration steps use the Calibrate page and no longer the ssh live counter [evidence: command:grep -q "Calibrate page" docs/runbooks/head-rgbd-fusion.md && ! grep -q "live line" docs/runbooks/head-rgbd-fusion.md]
- a2: MEMORY.md records the robot-side delivery decision [evidence: command:grep -q "head_rgbd_calibration/applied" MEMORY.md]
- a3: the runbook still has its steps and results sections [evidence: doc-section:docs/runbooks/head-rgbd-fusion.md#Steps]
- a4: the documented flow matches the built UI and node behaviour (kept as a manual judgement check) [evidence: judgement:]

#### Files

- `docs/runbooks/head-rgbd-fusion.md`
- `docs/head-rgbd-fusion-methodology.md`
- `scripts/README.md`
- `MEMORY.md`

#### Interfaces

- `docs/runbooks/head-rgbd-fusion.md` steps 2-3; `MEMORY.md` decision line.

#### Expected outcome

The runbook's calibration steps can be followed at the robot with the Calibrate page alone.

#### Steps

- Edit the four documents; run the verify command (both greps succeed).

#### Notes

- Prose task (review skip). The runbook belongs to the in-progress task rgbd-final-suite-and-runbook, which executes it later; keep its `## Results` section untouched.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on head-nodes-use-applied-calib, rgbd-sessions-and-results-ui, head-urdf-tof-joint-override

### S22: Jetson retirement allowlist for the calibration core

- Status: done
- Files: `tests/test_jetson_retired.py`, `tests/README.md`
- Verify: `uv run --frozen pytest tests/test_jetson_retired.py -q && uv run poe lint`
- Verified head: 84bb39c7f5b9f1c2d5e945fa0ba89b17794125a8

#### Requirement

`tests/test_jetson_retired.py::test_no_live_jetson_stereo_or_overview_reference` passes on the integrated epic branch again. The calibration core moved from `scripts/head_rgbd_calibration/` into `shared/ros2_common/head_rgbd_calibration/` and new code (the `head_rgbd_calibration` node, the web_ui RGB-D section, the frontend) legitimately names the stereo extrinsic calibration ("stereo RMS", `stereo_calibrate`, `stereo_max_rms_px`).

- `ALLOWED_REFERENCES` drops the entries for `scripts/head_rgbd_calibration/*` files that no longer exist and names each new file that mentions the stereo calibration with the existing `rgbd-calibration` category and the same stereo-calibration regex; one entry per file, no directory-wide `**` glob (`test_allowed_references_use_only_the_known_categories` keeps passing).
- No retired Jetson / depthd / overview_camera reference is allowlisted by this change.
- Verify on the branch that contains every other task of this epic: the whole `tests/test_jetson_retired.py` passes.

#### Acceptance conditions

- a1: test_no_live_jetson_stereo_or_overview_reference passes with every epic task merged [evidence: test: tests/test_jetson_retired.py::test_no_live_jetson_stereo_or_overview_reference]
- a2: no directory-wide glob is added and categories stay known [evidence: test: tests/test_jetson_retired.py::test_allowed_references_use_only_the_known_categories]

#### Files

- `tests/test_jetson_retired.py`
- `tests/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

What is true and observable when the task is done.

tests/test_jetson_retired.py passes in full on the integrated epic branch; the allowlist names the moved calibration files instead of the deleted scripts paths.

#### Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

- Merge every other epic task into this worktree, run `uv run --frozen pytest tests/test_jetson_retired.py -q` and read the unallowed hits (expect failures naming shared/ros2_common/head_rgbd_calibration files).
- Edit ALLOWED_REFERENCES: drop entries for deleted scripts/head_rgbd_calibration files, add one rgbd-calibration entry per new file that names the stereo calibration.
- Re-run the verify command (all pass); update tests/README.md if a test description changes; commit `test: allowlist the moved head RGB-D calibration core in the Jetson retirement guard`.

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S23: Calib node cgroup tier and docs

- Status: done
- Files: `tests/test_unit_cgroups.py`, `ansible/group_vars/client.yml`, `tests/README.md`, `shared/ros2_common/head_rgbd_calibration/__init__.py`
- Verify: `uv run --frozen pytest tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe lint-ansible`
- Verified head: 46271e5d

#### Requirement

Close the integration gaps found by the epic's full-suite run and scope audit.

- `tests/test_unit_cgroups.py::test_every_client_type_is_tiered` fails on the epic branch (passes on main): the new `head_rgbd_calibration` node type is in no tier. Put it in the background tier (work the robot can lose: capture and compute must never starve robot nodes; memory feedback "cgroups guarantee headroom") by adding it to that tier's types in the test and setting its `ros2_node_type_defaults` cgroup keys in `ansible/group_vars/client.yml` to the background tier's weights; keep its own memory cap. Whole `tests/test_unit_cgroups.py` passes.
- `tests/README.md` lists and describes `tests/test_head_rgbd_calibration_deploy.py` (each test or test group), like the other entries.
- `shared/ros2_common/head_rgbd_calibration/__init__.py` docstring names every module of the package (config, charuco, frames, intrinsics, extrinsics, calibrate, poses, urdf_handoff, urdf_text, and any other present).

#### Acceptance conditions

- a1: every client node type is tiered, head_rgbd_calibration in the background tier [evidence: test: tests/test_unit_cgroups.py::test_every_client_type_is_tiered]
- a2: tier weights hold for the new type [evidence: test: tests/test_unit_cgroups.py::test_tiers_have_expected_weights]
- a3: tests/README.md describes test_head_rgbd_calibration_deploy.py [evidence: command: grep -q test_head_rgbd_calibration_deploy tests/README.md]

#### Files

- `tests/test_unit_cgroups.py`
- `ansible/group_vars/client.yml`
- `tests/README.md`
- `shared/ros2_common/head_rgbd_calibration/__init__.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

tests/test_unit_cgroups.py passes in full with head_rgbd_calibration in the background tier; tests/README.md describes test_head_rgbd_calibration_deploy.py; the core package docstring names every module.

#### Steps

- Run `uv run --frozen pytest tests/test_unit_cgroups.py -q` and see test_every_client_type_is_tiered fail.
- Add the type to the background tier (test and client.yml weights), add the README entry and the docstring; run the verify command (all pass) and commit `fix: background cgroup tier for head_rgbd_calibration, test docs`.

#### Ruling

- 2026-10-20 accepted: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S24: RGBD API contract node

- Status: done
- Files: `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`, `nodes/head_rgbd_calibration/tests/test_app.py`, `nodes/head_rgbd_calibration/tests/test_compute.py`, `nodes/head_rgbd_calibration/tests/fixtures/api`, `nodes/head_rgbd_calibration/README.md`, `shared/ros2_common/head_rgbd_calibration/config.py`
- Verify: `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/ -q && uv run --frozen poe lint) && uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint`
- Verified head: ee9e4ae5

#### Requirement

Whole-epic review found the node's HTTP responses and the frontend client disagree, which breaks the main operator flow on the robot. The node owns the API contract; this task fixes the node side and pins the contract as literal JSON examples that both sides test against.

- One response shape per route, documented in `nodes/head_rgbd_calibration/README.md`: `POST /api/capture/start` and `POST /api/capture/stop` return the same nested body as `GET /api/state` (`{capture, compute, applied}`), so the frontend can call `onState` with it. `POST /api/sessions/{sid}/results/{rid}/apply` returns the applied record plus the per-unit restart state (`units`: the same list `GET /api/applied` returns), or the frontend gets the units another documented way; pick one and make node, fixture and README agree. The applied record (`applied.json` and every response embedding it) uses one set of key names everywhere (today the node writes `session`/`result` while the TS type says `session_id`/`result_id`); choose the node's names and document them.
- `result.json` (compute.py) carries a `limits` object with every acceptance limit used to judge the result, including a plane depth bias limit: add `plane_max_bias_mm: float = 15.0` (gt 0) to `LimitsConfig` in `shared/ros2_common/head_rgbd_calibration/config.py` and to `failed_limits` (the methodology and runbook already gate on plane bias <= 15 mm), with the YAML default in `scripts/head_rgbd_calibrate.yaml`.
- Contract fixtures: literal JSON example bodies for state, capture start, capture stop, apply, applied, result under `nodes/head_rgbd_calibration/tests/fixtures/api/*.json`; node tests assert the real responses (TestClient with the existing fakes) have exactly the fixture's keys and value types (a structural match helper, not byte equality of stamps).
- Tests first, seen failing: start/stop return the nested state; apply response matches its fixture; result.json has limits including plane_max_bias_mm; failed_limits flags a plane bias over the limit.

#### Acceptance conditions

- a1: capture start and stop return the nested {capture, compute, applied} state matching tests/fixtures/api/capture_start.json and capture_stop.json [evidence: test: nodes/head_rgbd_calibration/tests/test_app.py::test_capture_start_and_stop_return_the_state_fixture]
- a2: apply and applied responses match their fixtures, one set of applied-record key names [evidence: test: nodes/head_rgbd_calibration/tests/test_app.py::test_apply_and_applied_match_the_fixtures]
- a3: result.json carries every limit including plane_max_bias_mm [evidence: test: nodes/head_rgbd_calibration/tests/test_compute.py::test_result_carries_the_limits]
- a4: a plane bias over plane_max_bias_mm fails the limits [evidence: test: tests/test_head_rgbd_calib_core.py::test_failed_limits_flags_plane_bias]

#### Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py`
- `nodes/head_rgbd_calibration/tests/test_app.py`
- `nodes/head_rgbd_calibration/tests/test_compute.py`
- `nodes/head_rgbd_calibration/tests/fixtures/api`
- `nodes/head_rgbd_calibration/README.md`
- `shared/ros2_common/head_rgbd_calibration/config.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

Start, Stop, Apply and applied responses of the node match the documented contract and the JSON fixtures; result.json carries all limits including plane_max_bias_mm.

#### Steps

- Write the fixtures and the failing tests (start/stop nested state, apply/applied fixtures, result limits, plane bias limit); run the verify command and see them fail.
- Change app.py/apply.py/compute.py/config.py to the contract; update the node README; run the verify command (all pass) and commit `fix(head_rgbd_calibration): one API contract pinned by JSON fixtures`.

#### Ruling

- 2026-10-20 accepted: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S25: Calib core shared decoders

- Status: done
- Files: `shared/ros2_common/head_rgbd_calibration/frames.py`, `scripts/head_rgbd_calibration/frames.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`, `tests/test_head_rgbd_calib_core.py`, `nodes/head_rgbd_calibration/tests/test_capture.py`
- Verify: `uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_frames.py -q && uv run poe lint && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/ -q && uv run --frozen poe lint)`
- Verified head: 022b04e7

#### Requirement

The scope audit found two implementations of the same sensor_msgs decoders: `scripts/head_rgbd_calibration/frames.py` (stamp_ns, decode_mono16, decode_compressed, decode_camera_info) and `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py` (stamp_ns, decode_mono16, decode_compressed, camera_info_data). Keep one.

- Move the four decoders into the core `shared/ros2_common/head_rgbd_calibration/frames.py` (duck-typed on the message fields, so they work for rosbags and rclpy messages alike; no rosbags or rclpy import in the core); `scripts/head_rgbd_calibration/frames.py` and the node's `capture.py` import them; the node's private copies are deleted.
- Tests first: core tests decode a mono16 Image (both endiannesses), a CompressedImage and a CameraInfo built from simple namespace objects, and stamp_ns; existing CLI and node capture tests pass unchanged.

#### Acceptance conditions

- a1: core frames decoders handle mono16 (both endiannesses), compressed images, camera_info and stamps [evidence: test: tests/test_head_rgbd_calib_core.py::test_core_decoders]
- a2: the node capture uses the core decoders and keeps no private copies [evidence: command: ! grep -nE "^def (stamp_ns|decode_mono16|decode_compressed|camera_info_data)" nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py]
- a3: CLI and node tests still pass [evidence: command: uv run --frozen pytest tests/test_head_rgbd_calib_frames.py -q && (cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/ -q)]

#### Files

- `shared/ros2_common/head_rgbd_calibration/frames.py`
- `scripts/head_rgbd_calibration/frames.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `tests/test_head_rgbd_calib_core.py`
- `nodes/head_rgbd_calibration/tests/test_capture.py`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

The four decoders exist once, in the core frames module; the CLI and the node import them; all CLI and node tests pass.

#### Steps

- Write tests/test_head_rgbd_calib_core.py::test_core_decoders against the core module; run it and see the import fail.
- Move the decoders, switch scripts/frames.py and capture.py to them, delete the node copies; run the verify command (all pass) and commit `refactor: one copy of the head RGB-D sensor_msgs decoders in the core`.

#### Ruling

- 2026-10-20 accepted: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

### S26: RGBD API contract frontend

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`, `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`, `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`, `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint`
- Verified head: c4ef29de

#### Requirement

The Calibrate page frontend must accept exactly what the node returns. Consume the contract fixtures owned by rgbd-api-contract-node (`nodes/head_rgbd_calibration/tests/fixtures/api/*.json`) in the frontend tests by importing those JSON files directly (relative import from `nodes/web_ui/frontend/src/calibrate/`; vitest can read outside src; if the vite/ts config blocks it, add the minimal config change), never a hand-written copy.

- `rgbdApi.ts`: `startCapture` / `stopCapture` / `applyResult` / `getApplied` validators accept the fixture bodies; `AppliedState` uses the node's key names; a malformed body still yields the "Unexpected calibration response" error.
- `RgbdResults.tsx`: shows each metric next to its limit read from `result.json`'s `limits` object (including plane bias against `plane_max_bias_mm`); the hard-coded `DEFAULT_LIMITS` copy is removed; Apply success shows the applied state and unit restarts from the documented response.
- `RgbdCapture.tsx` / `RgbdCalibration.tsx`: Start and Stop update the shown state from the response (no error when the node accepted the call).
- Tests first, seen failing against the fixtures: rgbdApi validators accept each fixture; Start/Stop and Apply component flows driven by fixture bodies show no error; results table shows the plane bias limit from the fixture.
- `npx tsc --noEmit && npm test && npm run lint` pass; README section of nodes/web_ui updated if wording changes.

#### Acceptance conditions

- a1: every rgbdApi call accepts its node fixture body [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/rgbdApi.test.ts]
- a2: Start, Stop and Apply driven by fixture bodies show the new state and no error [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdCapture.test.tsx]
- a3: the results table shows each limit from result.json limits, including plane bias, and DEFAULT_LIMITS is gone [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/RgbdResults.test.tsx]

#### Files

- `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts`
- `nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/RgbdCalibration.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

#### Expected outcome

The Calibrate page shows no false errors on Start, Stop or Apply and shows each limit from result.json next to its metric.

#### Steps

- Import the node fixtures in rgbdApi/RgbdCapture/RgbdResults tests; run `npm test` and see them fail.
- Fix the validators, AppliedState names, results limits and Start/Stop state handling; run the verify command (all pass) and commit `fix(web_ui): Calibrate RGB-D section follows the node API contract`.

#### Ruling

- 2026-10-20 accepted: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published

#### Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none

## Rulings (migrated)

- calib-node-scaffold-config: accept true, at 2026-10-19T21:05:39Z: verifier crashed on --prev-head unknown (tooling); orchestrator re-ran the task verify on HEAD 52434e47: 16 passed, lint clean, 14 passed (ci_workflow, node_lint_coverage), clean tree
- calib-core-shared-decoders: accept true, at 2026-10-20T09:26:57Z: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published
- calib-node-cgroup-tier-and-d: accept true, at 2026-10-20T09:26:57Z: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published
- rgbd-api-contract-frontend: accept true, at 2026-10-20T09:26:57Z: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published
- rgbd-api-contract-node: accept true, at 2026-10-20T09:26:57Z: Merged to main (e892b6d7, 2a94085f, ce334203) and verified live on the robot 2026-10-20: capture, compute, apply, head_rgbd pairs published
