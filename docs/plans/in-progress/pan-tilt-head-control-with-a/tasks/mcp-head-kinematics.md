---
title: MCP head kinematics
status: done
repo: ros2-lerobot-sverve-platform
value: The agent can aim the head RGB camera exactly at a point despite the camera sitting off the pan/tilt axes.
files: [nodes/mcp_server/mcp_server/head_kinematics.py, nodes/mcp_server/tests/test_head_kinematics.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q && uv run --frozen poe lint"
depends_on: [head-urdf]
owns: [head-kinematics]
uses: [head-urdf]
model: opus
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--mcp-head-kinematics
branch: pan-tilt-head-control-with-a--mcp-head-kinematics
verified_head: cbc8f61af7496c4f67430b5b0fc54b624d513bac
run: pan-tilt-head-control-with-a
provides: [head-aim-error-metric, default-head-urdf-path]
acs_for_audit: []
process_gaps: []
---

# MCP head kinematics

## Requirement

- New `nodes/mcp_server/mcp_server/head_kinematics.py` (D8, U5): class `HeadKinematics(urdf_path: Path)` reading `nodes/web_ui/urdf/head.urdf` (ikpy chains as `ik.py` does, or a numpy chain parsed from the same URDF with ElementTree). Public API: `limits -> dict[str, tuple[float, float]]` (from the URDF); `link_pose(pan: float, tilt: float, link: str) -> np.ndarray` (4x4 pose of any head link or frame in base_link); `look_at(point_base: np.ndarray) -> tuple[float, float]` returning (pan, tilt) in radians for which the +z axis of `head_camera_optical_frame` passes through the point; raises `HeadAimError` (ValueError subclass) with the needed angles and the limits when the solution lies outside the limits, and when the point is closer than 0.15 m to the camera or the solve residual stays above 0.2 deg.
- The solve accounts for every offset (mount, tilt axis height, camera forward and lateral offset): `scipy.optimize.least_squares` over (pan, tilt) on the angle between the optical axis and the camera-to-point ray, bounded by the limits, seeded by the offset-free atan2 solution. Never returns clamped angles that miss the point.
- Docstrings document frames (base_link x forward, y left, z up, floor z = 0), units and sign conventions (pan + left, tilt + down).
- Tests first in new `nodes/mcp_server/tests/test_head_kinematics.py`, seen failing: at (0, 0) the optical axis is base_link +x; link_pose of head_camera_optical_frame and tof_optical_frame match the URDF origins; look_at residual < 0.2 deg for a grid of reachable floor and raised points (left, right, near, far, below the home view), and the forward kinematics of the returned angles puts the point on the axis within 2 mm at 2 m; a point straight behind the robot and a point needing tilt past +45 deg raise HeadAimError naming the angles; a point 0.1 m from the camera is refused; a point straight ahead on the floor at 0.6 m gives tilt near +30 deg (home view check).

## Acceptance conditions

- a1: look_at aims the RGB optical axis through reachable points within 0.2 deg, offsets included [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q]
- a2: unreachable or too-close points raise HeadAimError with the needed angles and limits, never a clamped aim [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q]
- a3: link_pose matches the head URDF frames [evidence: symbol: HeadKinematics]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/head_kinematics.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`

## Interfaces

- `mcp_server/head_kinematics.py:HeadKinematics` (`limits`, `link_pose`, `look_at`), `HeadAimError` (part `head-kinematics`).

## Expected outcome

mcp_server can compute any head frame pose and the pan/tilt that aims the RGB camera at a point.

## Steps

- Write `tests/test_head_kinematics.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): head kinematics and look-at`.

## Notes

- `ik.py:ArmKinematics` shows the ikpy URDF loading used in this package; scipy and numpy are already dependencies.
- Frames: base_link floor z = 0 (camera_scene docstring).

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-urdf
