---
title: MCP head controller
status: todo
repo: ros2-lerobot-sverve-platform
value: "mcp_server can command the head and know when it actually arrived, from real joint states only."
files: [nodes/mcp_server/mcp_server/head.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/tool_context.py, nodes/mcp_server/tests/fakes.py, nodes/mcp_server/tests/test_head_controller.py, nodes/mcp_server/tests/test_config.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py tests/test_config.py -q && uv run --frozen poe lint"
depends_on: [mcp-head-kinematics]
owns: [head-controller]
uses: [head-kinematics]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/pan-tilt-head-control-with-a--mcp-head-controller
branch: pan-tilt-head-control-with-a--mcp-head-controller
verified_head: ""
run: pan-tilt-head-control-with-a
provides: []
acs_for_audit: []
process_gaps: []
---

# MCP head controller

## Requirement

- `config.py`: new `HeadSettings` (pydantic, `StrictModel`) under `McpServerConfig.head`: `urdf_path` (default `nodes/web_ui/urdf/head.urdf`, resolved like `arm.urdf_path`), `joint_names` (head_pan, head_tilt), `command_topic` `/head/joint_commands`, `state_topic` `/head/joint_states`, `home_pan_rad` 0.0, `home_tilt_rad` 0.5236, `tolerance_rad` 0.03, `move_timeout_s` 4.0, `state_max_age_s` 0.5; validation: home inside the URDF limits (D9, D4).
- New `mcp_server/head.py`: `HeadBackend` Protocol (`publish_head_command(positions: dict[str, float]) -> None`, `head_sample() -> JointSample | None`, `now()`, `sleep()`); `HeadController(backend, kinematics, settings)` with `measured() -> dict[str, float] | None` (None when missing or older than state_max_age_s, never zero-filled), `check(pan, tilt)` (raises `HeadError` naming the limits for out-of-range values, no clamping), `move(pan, tilt) -> HeadMoveResult` (publishes once with frame_id `autonomy`, waits until both joints are within tolerance or move_timeout_s; status `converged` | `timeout` | `no_feedback`, with measured angles), `home() -> HeadMoveResult`. `HeadMoveResult` model in head.py.
- `ros_iface.py`: `RosRobot` implements `HeadBackend` (subscription on `head.state_topic` with the sensor QoS used for joint states, publisher on `head.command_topic` with `COMMAND_QOS`, header.frame_id `autonomy`, JointState name/position only) and exposes `head: HeadController`.
- `tool_context.py`: `RobotApi` declares `head: HeadController`.
- `tests/fakes.py`: `FakeHeadBackend` (records commands, optional simulated follower moving at a set speed, settable stale/missing samples).
- Tests first in new `tests/test_head_controller.py` (and the config cases in `tests/test_config.py`), seen failing: defaults and home validation; measured None when stale or missing; out-of-range refused with limits named; move converges with a following fake, times out with a stuck one, reports no_feedback without samples; command carries frame_id autonomy and both joint names; home moves to the configured home.

## Acceptance conditions

- a1: HeadController moves, waits for convergence and reports converged, timeout or no_feedback [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py -q]
- a2: out-of-range angles are refused naming the limits, and stale samples are never reported as measured [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py -q]
- a3: head config defaults and home validation [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py -q]
- a4: RosRobot implements the head backend on /head/joint_states and /head/joint_commands [evidence: symbol: HeadController]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/head.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/tests/fakes.py`
- `nodes/mcp_server/tests/test_head_controller.py`
- `nodes/mcp_server/tests/test_config.py`

## Interfaces

- `mcp_server/head.py:HeadBackend`, `HeadController`, `HeadMoveResult`, `HeadError` (part `head-controller`); `config.py:HeadSettings`; `tool_context.py:RobotApi.head`; `tests/fakes.py:FakeHeadBackend`.

## Expected outcome

mcp_server has a head controller wired to ROS that reports real arrival or a named failure.

## Steps

- Write `tests/test_head_controller.py` and the config cases; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py tests/test_config.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): head controller`.

## Notes

- Pattern: `arm.py:ArmController` over `FakeArmBackend` (tests/fakes.py).
- No placeholder: `measured()` returns None for stale samples.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-head-kinematics
