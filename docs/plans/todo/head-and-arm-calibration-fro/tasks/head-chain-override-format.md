---
title: Head chain override format
status: todo
repo: ros2-lerobot-sverve-platform
value: "One format and one applier for the calibrated head chain, so every consumer reads the same override the same way."
files: [tests/test_head_chain_override.py]
verify: "uv run --frozen pytest tests/test_head_chain_override.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q && uv run poe lint"
depends_on: []
owns: []
uses: [head-chain-override]
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

Delivered by epic head-chain-floor-plane-fit-a, task head-chain-override-format (moved 2026-10-21, user decision "Both, quick fit first"; that epic is a blocker of this one). Its requirement is the one this task had: `urdf_text.py` `rewrite_joint_origin`, `parse_head_chain`, `head_chain_json`, `apply_overrides` and the file constants (contract `head-chain-override`, now owned there).

No code change here: confirm the delivered part is on the base branch and its tests pass, so the consumers in this epic (head-chain-board-solver, head-chain-compute-apply, mcp-head-kinematics-applied, web-ui-applied-head-urdf) can rely on it.

## Acceptance conditions

- a1: the override module and its test are tracked [evidence: command:git ls-files --error-unmatch shared/ros2_common/head_rgbd_calibration/urdf_text.py tests/test_head_chain_override.py]
- a2: the delivered override tests pass [evidence: command:uv run --frozen pytest tests/test_head_chain_override.py tests/test_head_rgbd_urdf_handoff.py -q]

## Files

- `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
- `tests/test_head_chain_override.py`

## Interfaces

- `urdf_text.py::rewrite_joint_origin`, `parse_head_chain`, `head_chain_json`, `apply_overrides`, `HEAD_CHAIN_FILE`, `TOF_JOINT_FILE`, `APPLIED_MARKER_FILE`, `HEAD_CHAIN_JOINTS` (contract `head-chain-override`).
- `head_chain.json`: `{joints: {head_mount_joint|head_tilt|head_camera_joint: {xyz: [3], rpy: [3]}}, note, session, result}`.

## Expected outcome

One format and one applier for the calibrated head chain, so every consumer reads the same override the same way.

## Steps

- Tests first (red), implement, verify.
- Run `uv run --frozen pytest tests/test_head_chain_override.py tests/test_head_rgbd_urdf_handoff.py tests/test_head_tof_joint_override.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-21 amended: moved to epic head-chain-floor-plane-fit-a (user D1 2026-10-21, quick floor fit first); this task only confirms the delivered part
