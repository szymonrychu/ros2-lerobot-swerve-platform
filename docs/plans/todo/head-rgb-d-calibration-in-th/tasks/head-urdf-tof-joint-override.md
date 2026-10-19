---
title: Head URDF tof joint override
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head TF uses the applied tof_joint after a restart, without editing the repo URDF on the robot."
files: [nodes/head_state_publisher/launch/head_state_publisher.launch.py, nodes/head_state_publisher/launch/head_description.py, tests/test_head_tof_joint_override.py, tests/test_head_state_publisher_config.py, ansible/group_vars/client.yml, nodes/head_state_publisher/README.md]
verify: "uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_state_publisher_config.py -q && uv run poe lint"
depends_on: [calib-node-apply-download, calib-core-extrinsics-urdf]
owns: []
uses: [rgbd-calib-applied-files, rgbd-calib-core]
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

# Head URDF tof joint override

## Requirement

head_state_publisher uses an applied tof_joint without editing the repo URDF on the robot (design D8).

- New `nodes/head_state_publisher/launch/head_description.py`, standard library only: `robot_description(urdf_path: Path, override_path: Path, urdf_text_path: Path) -> str`. Reads the URDF; when `override_path` exists and holds JSON `{xyz: [3 numbers], rpy: [3 numbers], note: str}`, loads `urdf_text.py` by file path (`importlib.util.spec_from_file_location`, no package import) and returns `rewrite_tof_joint(urdf, Origin(xyz, rpy), note)`; a missing file returns the URDF unchanged; unreadable JSON or a wrong shape returns the URDF unchanged and logs a warning naming the file and the reason.
- `head_state_publisher.launch.py` loads `head_description` by path from its own directory and builds `robot_description` from `HEAD_URDF_PATH`, `TOF_JOINT_OVERRIDE = /var/lib/ros2/head_rgbd_calibration/applied/tof_joint.json` and `URDF_TEXT_PATH` (`<repo>/shared/ros2_common/head_rgbd_calibration/urdf_text.py`, resolved relative to the launch file).
- `ansible/group_vars/client.yml`: the head_state_publisher node type `src_paths` adds `shared/ros2_common/head_rgbd_calibration/urdf_text.py` (a change to it restarts the node on deploy); `tests/test_head_state_publisher_config.py` updated to match.
- New `tests/test_head_tof_joint_override.py`: override applied (tof_joint origin carries the JSON values, every other line identical); missing file gives the repo URDF; invalid JSON and wrong shape give the repo URDF plus a warning; `head_description.py` imports only `sys.stdlib_module_names`; the launch file references `head_description`, the applied path and `urdf_text.py`.
- `nodes/head_state_publisher/README.md` documents the override (doc file); `tests/README.md` updated.

## Acceptance conditions

- a1: an applied tof_joint.json rewrites only the tof_joint origin [evidence: test:tests/test_head_tof_joint_override.py::test_override_rewrites_only_tof_joint]
- a2: without an override the repo URDF is published unchanged [evidence: test:tests/test_head_tof_joint_override.py::test_missing_override_keeps_repo_urdf]
- a3: an invalid override is ignored with a warning [evidence: test:tests/test_head_tof_joint_override.py::test_invalid_override_ignored_with_warning]
- a4: head_description is standard library only [evidence: test:tests/test_head_tof_joint_override.py::test_head_description_is_stdlib_only]
- a5: the launch file wires the override and the node type restarts on urdf_text changes [evidence: command:uv run --frozen pytest tests/test_head_tof_joint_override.py tests/test_head_state_publisher_config.py -q]
- a6: root lint passes [evidence: command:uv run poe lint]

## Files

- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `nodes/head_state_publisher/launch/head_description.py`
- `tests/test_head_tof_joint_override.py`
- `tests/test_head_state_publisher_config.py`
- `ansible/group_vars/client.yml`
- `nodes/head_state_publisher/README.md`

## Interfaces

- `nodes/head_state_publisher/launch/head_description.py::robot_description(urdf_path: Path, override_path: Path, urdf_text_path: Path) -> str`.
- Reads contract `rgbd-calib-applied-files` (`tof_joint.json`); loads `shared/ros2_common/head_rgbd_calibration/urdf_text.py` by path.

## Expected outcome

After an Apply and the path-unit restart, `/tf_static` carries the calibrated tof_joint; without an applied file the repo URDF is published unchanged.

## Steps

- Write `tests/test_head_tof_joint_override.py` and update `tests/test_head_state_publisher_config.py`; run: red.
- Implement `head_description.py`, wire the launch file, extend `src_paths`.
- Run the verify command: all pass, lint clean.

## Notes

- The launch runs on the system Python (no node venv): stdlib only.
