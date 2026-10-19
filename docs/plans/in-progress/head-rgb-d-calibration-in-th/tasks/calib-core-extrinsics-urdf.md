---
title: Calib core extrinsics urdf
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The stereo extrinsic, plane check and tof_joint rewrite are importable by the node and, stdlib-only, by the head_state_publisher launch."
files: [shared/ros2_common/head_rgbd_calibration/extrinsics.py, shared/ros2_common/head_rgbd_calibration/urdf_text.py, shared/ros2_common/head_rgbd_calibration/urdf_handoff.py, scripts/head_rgbd_calibration/extrinsics.py, scripts/head_rgbd_calibration/urdf_handoff.py, tests/test_head_rgbd_calib_core.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_core.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q && uv run poe lint"
depends_on: [calib-core-frames-intrinsics]
owns: []
uses: [rgbd-calib-core]
model: ""
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib core extrinsics urdf

## Requirement

Move the stereo extrinsic and the URDF hand-off into the core, with the tof_joint text rewrite split into a stdlib-only module the head_state_publisher launch can load (design D2, D8).

- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`: the whole module moved with `git mv`, importing from the core.
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`: imports only standard-library modules. Holds `TOF_JOINT`, `NOTE_MARKER`, `NUMBER_FORMAT`, `Origin` (frozen dataclass, `xyz` and `rpy` float triples), `format_vec` and `rewrite_tof_joint(urdf_text, origin, note)`, moved from `urdf_handoff.py` with byte-identical output.
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`: the rest of `scripts/head_rgbd_calibration/urdf_handoff.py` (joint constants, `JointInfo`, `HeadChain`, `Drift`, `to_matrix`, `from_matrix`, `extrinsic_matrix`, `parse_vec`, `read_chain`, `tof_joint_origin`, `check_drift`), importing `Origin`, `format_vec`, `rewrite_tof_joint` from `urdf_text` so `urdf_handoff.rewrite_tof_joint` still resolves.
- `scripts/head_rgbd_calibration/extrinsics.py` and `scripts/head_rgbd_calibration/urdf_handoff.py` are re-created as transitional re-exports, deleted by calib-core-retire-shims.
- `tests/test_head_rgbd_calib_core.py` gains: every import in `urdf_text.py` is in `sys.stdlib_module_names` (AST scan); loading `urdf_text.py` by file path with `importlib.util.spec_from_file_location` in a `python -I` subprocess leaves `numpy` out of `sys.modules`; core `rewrite_tof_joint` on `nodes/web_ui/urdf/head.urdf` with a fixed origin changes only the tof_joint origin and the calibration comment (every other line identical); core `stereo_calibrate` recovers the synthetic extrinsic within the existing extrinsics test tolerance.
- Every existing calibration test passes with no assertion changed; root `uv run poe lint` passes; `shared/README.md` lists the modules and notes that `urdf_text.py` must stay stdlib-only.

## Acceptance conditions

- a1: urdf_text imports only the standard library [evidence: test:tests/test_head_rgbd_calib_core.py::test_urdf_text_is_stdlib_only]
- a2: urdf_text loads by file path without numpy [evidence: test:tests/test_head_rgbd_calib_core.py::test_urdf_text_loads_by_path_without_numpy]
- a3: the core rewrite changes only the tof_joint origin and its comment [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_rewrite_tof_joint_touches_only_tof_joint]
- a4: the core stereo extrinsic recovers the synthetic pose [evidence: test:tests/test_head_rgbd_calib_core.py::test_core_stereo_extrinsic_recovers_synthetic_pose]
- a5: every existing calibration test passes unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_frames.py tests/test_head_rgbd_calib_intrinsics.py tests/test_head_rgbd_calib_extrinsics.py tests/test_head_rgbd_calibrate_cli.py tests/test_head_rgbd_calib_live.py tests/test_head_rgbd_validate.py tests/test_head_rgbd_urdf_handoff.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/extrinsics.py`
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py`
- `scripts/head_rgbd_calibration/extrinsics.py`
- `scripts/head_rgbd_calibration/urdf_handoff.py`
- `tests/test_head_rgbd_calib_core.py`

## Interfaces

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py::rewrite_tof_joint(urdf_text: str, origin: Origin, note: str) -> str`, `Origin(xyz, rpy)`, `format_vec`, `TOF_JOINT`, `NOTE_MARKER` (stdlib only; loaded by path by head-urdf-tof-joint-override).
- `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py::read_chain`, `tof_joint_origin`, `check_drift`.
- `shared/ros2_common/head_rgbd_calibration/extrinsics.py::stereo_calibrate`, `plane_depth_bias`, `write_extrinsics_yaml`, `load_extrinsics_yaml`, `ExtrinsicResult`, `PlaneCheck`.

## Expected outcome

`urdf_text.py` loads in a bare `python3 -I` by file path; extrinsics and the URDF hand-off import from the core; CLI `urdf` mode output is unchanged.

## Steps

- Add the four tests; run: red.
- `git mv` extrinsics.py; split urdf_handoff.py into `urdf_text.py` (stdlib) and `urdf_handoff.py`; re-create the two transitional re-exports.
- Run the verify command: all pass, lint clean.

## Notes

- head_state_publisher runs on the system Python without numpy guarantees: keep `urdf_text.py` free of numpy, scipy and package-relative imports.
