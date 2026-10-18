---
title: MCP head tools
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent can set the head angles, look at a point once in base_link or map, and send the head home."
files: [nodes/mcp_server/mcp_server/head_tools.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/models.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/test_head_tools.py, nodes/mcp_server/tests/test_tools.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-head-controller]
owns: [head-tool-names]
uses: [head-controller]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP head tools

## Requirement

- New `mcp_server/head_tools.py` with `register(ctx: ToolContext)` added to `tools.TOOL_MODULES` and `TOOL_NAMES = ("set_head_angles", "head_look_at", "head_home")` (D9, U5, U6):
  - `set_head_angles(pan_deg: float, tilt_deg: float)`: moves the head (degrees in, degrees out); out-of-range is a tool error naming the limits in degrees.
  - `head_look_at(x: float, y: float, z: float, frame: Literal['base_link', 'map'] = 'base_link')`: map points are converted to base_link with the current robot map pose (tool error when the pose is unknown); aims once with `HeadKinematics.look_at` (no tracking) and moves; HeadAimError becomes a tool error with the needed angles.
  - `head_home()`: moves to the configured home.
  - Each returns status, commanded and measured pan/tilt in degrees and, for look_at, the target in base_link.
- `tools.py`: the three tools in `TOOL_SUBSYSTEM` as `"any"`, in `MOTION_TOOLS` (battery cut-off refuses them), absent from `RATE_SUBSYSTEMS` (explicit exclusion with a comment), and listed in the tool-name tuple; `INSTRUCTIONS` gains one sentence on the head tools and their lease need.
- `models.py` / `ros_iface.py`: `RobotState` gains optional `head: {pan_deg, tilt_deg}`, filled only from a fresh head sample (absent otherwise).
- `tests/test_tools.py`: `FakeRobot` gains a `head` backed by `FakeHeadBackend` so every existing test keeps passing.
- `nodes/mcp_server/README.md`: the three tools in the tool table (lease any, motion tool, degrees, aim once) and the head config section.
- Tests first in new `tests/test_head_tools.py`, seen failing: refused without either lease and allowed with the arm lease alone and with the base lease alone; refused under battery cut-off; out-of-range error names limits; look_at in base_link and map (pose used; unknown pose refused); look_at unreachable refused with angles and nothing published; head_home publishes the home; get_robot_state carries head angles only when fresh; head tools not in RATE_SUBSYSTEMS.

## Acceptance conditions

- a1: the three head tools need either lease and are refused under battery cut-off [evidence: test: nodes/mcp_server/tests/test_head_tools.py]
- a2: head_look_at aims once at a base_link or map point and refuses unreachable points without moving [evidence: test: nodes/mcp_server/tests/test_head_tools.py]
- a3: get_robot_state reports head angles only from a fresh sample [evidence: test: nodes/mcp_server/tests/test_head_tools.py]
- a4: the mcp_server README lists set_head_angles, head_look_at and head_home [evidence: command: grep -q head_look_at nodes/mcp_server/README.md && grep -q set_head_angles nodes/mcp_server/README.md && grep -q head_home nodes/mcp_server/README.md]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/head_tools.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_head_tools.py`
- `nodes/mcp_server/tests/test_tools.py`
- `nodes/mcp_server/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
