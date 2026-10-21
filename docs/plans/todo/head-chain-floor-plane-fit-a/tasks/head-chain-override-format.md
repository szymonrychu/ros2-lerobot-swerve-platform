---
title: Head chain override format
status: todo
repo: ros2-lerobot-sverve-platform
value: "One format and one applier for the calibrated head chain, so every consumer reads the same override the same way."
files: [shared/ros2_common/head_rgbd_calibration/urdf_text.py, tests/test_head_chain_override.py, tests/README.md, shared/README.md]
verify: "uv run --frozen pytest tests/test_head_chain_override.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q && uv run poe lint"
depends_on: []
owns: [head-chain-override]
uses: []
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

# Head chain override format

## Requirement

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

## Acceptance conditions

- a1: head_chain.json round trips through the writer and parser [evidence: test:tests/test_head_chain_override.py::test_head_chain_json_round_trip]
- a2: apply_overrides applies the chain and then the tof joint [evidence: test:tests/test_head_chain_override.py::test_apply_overrides_chain_then_tof]
- a3: each invalid file gives one warning and changes nothing [evidence: test:tests/test_head_chain_override.py::test_invalid_override_files_warn_and_keep_urdf]
- a4: urdf_text stays standard-library only [evidence: test:tests/test_head_chain_override.py::test_urdf_text_imports_only_stdlib]
- a5: the existing tof_joint handoff tests pass unchanged [evidence: command:uv run --frozen pytest tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `tests/test_head_chain_override.py`
- `tests/README.md`
- `shared/README.md`

## Interfaces

- `urdf_text.py::rewrite_joint_origin`, `parse_head_chain`, `head_chain_json`, `apply_overrides`, `HEAD_CHAIN_FILE`, `TOF_JOINT_FILE`, `APPLIED_MARKER_FILE`, `HEAD_CHAIN_JOINTS` (contract `head-chain-override`).
- `head_chain.json`: `{joints: {head_mount_joint|head_tilt|head_camera_joint: {xyz: [3], rpy: [3]}}, note, session, result}`.

## Expected outcome

One format and one applier for the calibrated head chain; the tof_joint rewrite is byte-identical to today.

## Steps

- Write `tests/test_head_chain_override.py` first; run `uv run --frozen pytest tests/test_head_chain_override.py -q` and see it fail on the missing functions.
- Implement in `urdf_text.py` (standard library only), keep `rewrite_tof_joint` as a call of `rewrite_joint_origin`.
- Run `uv run --frozen pytest tests/test_head_chain_override.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q && uv run poe lint`; expect every test passed and lint clean.
- Update `tests/README.md` and `shared/README.md`.

## Notes

- Moved unchanged from head-and-arm-calibration-fro (design D1); its later tasks (head-chain-board-solver, head-chain-compute-apply, mcp-head-kinematics-applied, web-ui-applied-head-urdf) use this API as written, so do not rename anything.
- Design: `ticket.py path head-chain-floor-plane-fit-a design` (D1, D7).
