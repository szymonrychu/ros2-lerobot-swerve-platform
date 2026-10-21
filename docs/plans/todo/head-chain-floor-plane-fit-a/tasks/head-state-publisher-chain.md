---
title: Head state publisher chain
status: todo
repo: ros2-lerobot-sverve-platform
value: "TF of the head camera and ToF follows the measured head chain after an apply, so the floor, Nav2 and the map use the real geometry."
files: [nodes/head_state_publisher/launch/head_description.py, nodes/head_state_publisher/launch/head_state_publisher.launch.py, tests/test_head_tof_joint_override.py, tests/test_head_chain_launch.py, tests/README.md, nodes/head_state_publisher/README.md]
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

head_state_publisher publishes TF from the applied head chain as well as the applied tof_joint (design D7 and D8; moved from head-and-arm-calibration-fro).

- `nodes/head_state_publisher/launch/head_description.py` (standard library only, runs on the system Python): `robot_description(urdf_path: Path, applied_dir: Path, urdf_text_path: Path) -> str` loads `urdf_text.py` by path and returns `apply_overrides(urdf, applied_dir)` text; each warning is logged with `LOGGER.warning`; a urdf_text module that fails to load logs a warning and returns the URDF unchanged.
- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`: `APPLIED_DIR = Path("/var/lib/ros2/head_rgbd_calibration/applied")` replaces `TOF_JOINT_OVERRIDE`; the docstring names both override files. The apply path unit already restarts head_state_publisher on `applied.json` changes (no Ansible change).
- `tests/test_head_tof_joint_override.py` follows the new signature with the same behaviours (tof override applied, unusable override ignored with a warning).
- New `tests/test_head_chain_launch.py`: a `head_chain.json` in a tmp applied dir changes `head_mount_joint`, `head_tilt` and `head_camera_joint` origins in the description; chain and tof overrides together; an invalid chain file logs a warning, keeps the URDF chain and still applies the tof override; `generate_launch_description()` with the constant monkeypatched to the tmp dir passes the overridden text as `robot_description`.
- `tests/test_head_state_publisher_config.py` passes.
- `tests/README.md` lists `tests/test_head_chain_launch.py` and the changed `tests/test_head_tof_joint_override.py`; `nodes/head_state_publisher/README.md` names both override files in `applied/`.

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
- `tests/README.md`
- `nodes/head_state_publisher/README.md`

## Interfaces

- `head_description.py::robot_description(urdf_path: Path, applied_dir: Path, urdf_text_path: Path) -> str`; launch constant `APPLIED_DIR`.

## Expected outcome

After an apply and the path unit restart, `/tf` from head_state_publisher carries the fitted head_mount_joint and head_tilt origins and the applied tof_joint.

## Steps

- Write `tests/test_head_chain_launch.py` and adapt `tests/test_head_tof_joint_override.py` first; run them and see them fail on the old signature.
- Change `head_description.py` and the launch file.
- Run `uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_chain_launch.py tests/test_head_state_publisher_config.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Moved from head-and-arm-calibration-fro (design D1, D8). The apply path unit (`ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.path.j2`, PathChanged on applied.json) already restarts head_state_publisher; no Ansible change.
- `shared/ros2_common/head_rgbd_calibration/urdf_text.py` is already deployed with head_state_publisher (client.yml src list).
