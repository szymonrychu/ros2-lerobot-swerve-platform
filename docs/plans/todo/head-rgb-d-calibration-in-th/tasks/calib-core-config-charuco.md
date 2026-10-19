---
title: Calib core config charuco
status: todo
repo: ros2-lerobot-sverve-platform
value: "The board, selection and limits settings and the ChArUco detector become importable from node venvs: the base every robot-side calibration piece builds on."
files: [pyproject.toml, uv.lock, shared/ros2_common/head_rgbd_calibration/__init__.py, shared/ros2_common/head_rgbd_calibration/config.py, shared/ros2_common/head_rgbd_calibration/charuco.py, scripts/head_rgbd_calibration/config.py, scripts/head_rgbd_calibration/charuco.py, tests/test_head_rgbd_calib_core.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint"
depends_on: []
owns: [rgbd-calib-core]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib core config charuco

## Requirement

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

## Acceptance conditions

- a1: the core package imports no scripts, rosbags or subprocess code [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_imports_no_scripts_rosbags_or_subprocess]
- a2: the core board, selection and limits models keep the CLI yaml defaults and reject unknown keys [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_config_defaults_match_cli_yaml]
- a3: the core detector finds the synthetic board [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_detect_finds_synthetic_board]
- a4: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q]
- a5: root lint passes with the core under shared/ [evidence: command:uv run poe lint]
- a6: the root project resolves ros2-common from shared, editable [evidence: command:grep -q 'ros2-common = { path = "shared", editable = true }' pyproject.toml]
- a7: the shared README documents the package [evidence: doc-section:shared/README.md#head_rgbd_calibration]

## Files

- `pyproject.toml`
- `uv.lock`
- `shared/ros2_common/head_rgbd_calibration/__init__.py`
- `shared/ros2_common/head_rgbd_calibration/config.py`
- `shared/ros2_common/head_rgbd_calibration/charuco.py`
- `scripts/head_rgbd_calibration/config.py`
- `scripts/head_rgbd_calibration/charuco.py`
- `tests/test_head_rgbd_calib_core.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
