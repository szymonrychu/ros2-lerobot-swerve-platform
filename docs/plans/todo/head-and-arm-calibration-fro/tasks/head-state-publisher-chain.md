---
title: Head state publisher chain
status: todo
repo: ros2-lerobot-sverve-platform
value: "TF of the head camera and ToF follows the measured head chain after an apply, so mapping and obstacles use the real geometry."
files: [nodes/head_state_publisher/launch/head_description.py, nodes/head_state_publisher/launch/head_state_publisher.launch.py, tests/test_head_tof_joint_override.py, tests/test_head_chain_launch.py]
verify: "uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_chain_launch.py tests/test_head_state_publisher_config.py -q && uv run poe lint"
depends_on: [head-chain-override-format]
owns: []
uses: [head-chain-override]
model: ""
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head state publisher chain

## Requirement

head_state_publisher publishes TF from the applied head chain as well as the applied tof_joint (design D5).

- `nodes/head_state_publisher/launch/head_description.py` (standard library only, runs on the system Python): `robot_description(urdf_path: Path, applied_dir: Path, urdf_text_path: Path) -> str` loads `urdf_text.py` by path and returns `apply_overrides(urdf, applied_dir)` text; each warning is logged with `LOGGER.warning`; a urdf_text module that fails to load logs a warning and returns the URDF unchanged.
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`: `APPLIED_DIR = Path("/var/lib/ros2/head_rgbd_calibration/applied")` replaces `TOF_JOINT_OVERRIDE`; the docstring names both override files. The apply path unit already restarts head_state_publisher on `applied.json` changes (no Ansible change).
- `tests/test_head_tof_joint_override.py` follows the new signature with the same behaviours (tof override applied, unusable override ignored with a warning).
- New `tests/test_head_chain_launch.py`: a `head_chain.json` in a tmp applied dir changes `head_mount_joint`, `head_tilt` and `head_camera_joint` origins in the description; chain and tof overrides together; an invalid chain file logs a warning, keeps the URDF chain and still applies the tof override; `generate_launch_description()` with the constant monkeypatched to the tmp dir passes the overridden text as `robot_description`.
- `tests/test_head_state_publisher_config.py` passes.

Steps: tests first (red), implement, verify.

## Acceptance conditions

- a1: the applied head chain reaches robot_description [evidence: test:tests/test_head_chain_launch.py::test_head_chain_override_reaches_robot_description]
- a2: an invalid chain file warns and keeps the tof override [evidence: test:tests/test_head_chain_launch.py::test_invalid_chain_warns_and_keeps_tof_override]
- a3: the launch passes the overridden text [evidence: test:tests/test_head_chain_launch.py::test_launch_uses_applied_dir]
- a4: tof override and launch config tests pass [evidence: command:uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_state_publisher_config.py -q]
- a5: root lint passes [evidence: command:uv run poe lint]

## Files

- `nodes/head_state_publisher/launch/head_description.py`
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `tests/test_head_tof_joint_override.py`
- `tests/test_head_chain_launch.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
