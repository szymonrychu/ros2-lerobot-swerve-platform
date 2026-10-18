---
title: MCP head tools
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--mcp-head-tools
branch: pan-tilt-head-control-with-a--mcp-head-tools
verified_head: 0497c6ccac6f2a6714cd30967f6b2c4ebb3d5594
run: pan-tilt-head-control-with-a
provides: [head-tool-names, head-angles-state]
acs_for_audit: []
process_gaps: []
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

- a1: the three head tools need either lease and are refused under battery cut-off [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q]
- a2: head_look_at aims once at a base_link or map point and refuses unreachable points without moving [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q]
- a3: get_robot_state reports head angles only from a fresh sample [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q]
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

- `mcp_server/head_tools.py:register`, `TOOL_NAMES` (part `head-tool-names`); `tools.py:TOOL_SUBSYSTEM`, `MOTION_TOOLS`, `RATE_SUBSYSTEMS`; `models.py:RobotState.head`.

## Expected outcome

The agent can call set_head_angles, head_look_at and head_home under either lease.

## Steps

- Write `tests/test_head_tools.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): head tools`.

## Notes

- `ControlManager.refusal` already supports "any".
- `RATE_SUBSYSTEMS` is derived from TOOL_SUBSYSTEM: exclude the head tools explicitly.
- Root `tests/test_claude_agent_config.py` classifies every registered tool: the agent task updates claude_agent; this task must not break mcp_server suites.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-head-controller

## Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.
