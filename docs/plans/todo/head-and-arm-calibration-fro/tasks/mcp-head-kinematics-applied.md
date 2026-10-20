---
title: MCP head kinematics applied
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent's head aiming and head-camera geometry use the measured chain, reloaded on apply without killing an agent session."
files: [nodes/mcp_server/mcp_server/head_kinematics.py, nodes/mcp_server/mcp_server/head.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/test_head_kinematics.py, nodes/mcp_server/tests/test_head_controller.py, nodes/mcp_server/tests/test_config.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py tests/test_head_tools.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint"
depends_on: [head-chain-override-format]
owns: []
uses: [head-chain-override]
model: ""
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP head kinematics applied

## Requirement

mcp_server aims and places the head from the applied head chain, reloaded without a restart (design D5, D11; MEMORY: an mcp_server restart kills a running agent session).

- `nodes/mcp_server/mcp_server/head_kinematics.py`: `HeadKinematics.from_text(urdf_text: str, source: str) -> HeadKinematics` (the path constructor delegates to it; same limits, links and look_at).
- `nodes/mcp_server/mcp_server/config.py`: `head.applied_dir: Path | None = Path("/var/lib/ros2/head_rgbd_calibration/applied")` (None disables the override).
- `nodes/mcp_server/mcp_server/head.py`: `AppliedHeadKinematics(urdf_path: Path, applied_dir: Path | None)` with `current() -> HeadKinematics`: builds from `ros2_common.head_rgbd_calibration.urdf_text.apply_overrides(urdf text, applied_dir)` and rebuilds when `applied/applied.json` mtime_ns changes, appears or disappears; logs every override warning and one info line per reload naming the applied session and result; a rebuild error keeps the previous kinematics and logs the error. `HeadController.kinematics` becomes a property returning `provider.current()`, so limits, `look_at` and `link_pose` (camera_scene, head tools) use the applied chain at each call.
- `nodes/mcp_server/mcp_server/ros_iface.py` builds the provider from `config.head.urdf_path` and `config.head.applied_dir`.
- Tests: `tests/test_head_kinematics.py`: `from_text` equals the path constructor. `tests/test_head_controller.py`: no applied dir -> repo URDF; a head_chain.json raising `head_mount_joint` z by 0.05 m changes the tilt `look_at` returns for the same floor point; a changed `applied.json` mtime reloads; a broken head_chain.json keeps the URDF chain with a logged warning; a rebuild exception keeps the previous kinematics. `tests/test_config.py`: `applied_dir` default and null.
- The mcp_server head tests and camera scene tests pass.

## Acceptance conditions

- a1: look_at follows an applied head chain [evidence: test:nodes/mcp_server/tests/test_head_controller.py::test_look_at_uses_applied_head_chain]
- a2: a changed applied.json reloads the kinematics without a restart [evidence: test:nodes/mcp_server/tests/test_head_controller.py::test_applied_marker_change_reloads_kinematics]
- a3: a broken override keeps a working chain [evidence: test:nodes/mcp_server/tests/test_head_controller.py::test_broken_override_keeps_previous_kinematics]
- a4: from_text equals the path constructor [evidence: test:nodes/mcp_server/tests/test_head_kinematics.py::test_from_text_equals_path]
- a5: head and camera scene tests pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py tests/test_head_tools.py tests/test_head_lease.py tests/test_camera_scene_head.py tests/test_config.py -q]
- a6: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/head_kinematics.py`
- `nodes/mcp_server/mcp_server/head.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`
- `nodes/mcp_server/tests/test_head_controller.py`
- `nodes/mcp_server/tests/test_config.py`

## Interfaces

- `HeadKinematics.from_text(urdf_text, source)`; `head.py::AppliedHeadKinematics.current()`; `HeadController.kinematics` property; config `head.applied_dir`.

## Expected outcome

The agent's head aiming and head-camera geometry use the measured chain, reloaded on apply without killing an agent session.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_controller.py tests/test_head_tools.py tests/test_camera_scene_head.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

