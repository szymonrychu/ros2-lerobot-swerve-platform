---
title: Robot MCP client
status: todo
repo: ros2-lerobot-sverve-platform
value: "The calibrator moves the head and arm only through mcp_server's guarded tools (leases, collision and floor guards, battery gate)."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py, nodes/head_rgbd_calibration/pyproject.toml, nodes/head_rgbd_calibration/uv.lock, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, nodes/head_rgbd_calibration/tests/test_robot_client.py, nodes/head_rgbd_calibration/tests/test_config.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_robot_client.py tests/test_config.py -q && uv run poe lint"
depends_on: [joint-state-still-capture]
owns: [calib-robot-client]
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Robot MCP client

## Requirement

An MCP client of mcp_server so the node moves the head and arm only through the guarded tools (owns `calib-robot-client`; design D6).

- `nodes/head_rgbd_calibration/pyproject.toml`: dependency `mcp==2.3.0` (the pin of `nodes/claude_agent/pyproject.toml`); `uv.lock` refreshed with `uv lock`.
- `config.py` and the shipped YAML: `robot: RobotConfig` with `mcp_url` `http://127.0.0.1:18200/mcp`, `token_file` `/etc/ros2/mcp_server/token`, `call_timeout_s` 60.0, `move_timeout_s` 120.0 (> 0).
- New `head_rgbd_calibration/robot_client.py`:
  - `ToolResult(is_error: bool, text: str, unreachable: bool = False, data: dict | None = None)`.
  - `RobotClient(cfg: RobotConfig, call: Callable | None = None)`; `call_tool(name, args, timeout_s) -> ToolResult` following `nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool`: bearer token read per call, `create_mcp_http_client`, `streamable_http_client`, structured content returned as `data`; never raises; never puts the token in text or logs. `call` injects a replacement for the transport in tests.
  - Typed helpers (sync wrappers running the coroutine, safe from a worker thread): `set_head_angles(pan_deg, tilt_deg)`, `head_look_at(x, y, z)` (base_link), `head_home()`, `take_arm()` (`take_control(subsystem="arm")`), `move_arm_joints(targets, speed_scale)` (timeout `move_timeout_s`), `release_arm(keep_position: bool = False)` (`release_control(subsystem="arm", keep_position=...)`: False parks the arm home, True leaves the arm where it is; either way mcp_server homes the head when no lease remains), `arm_state()`. The head tools need the arm or the base lease (mcp_server `head_tools.py` LEASE_NOTE); the sequencer takes it, this client only exposes the calls. A tool error comes back as `is_error` with the tool's text.
- New `tests/test_robot_client.py` (fake transport): tool names and argument shapes per helper; unreachable -> `is_error`, `unreachable`, text `mcp_server unreachable at <url>`; missing token file -> `is_error` naming the file; timeout -> `is_error` with the timeout; the token string never appears in any text. `tests/test_config.py`: robot defaults.

## Acceptance conditions

- a1: each helper calls its tool with the documented arguments [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_helpers_call_tools_with_arguments]
- a2: unreachable mcp_server is a reported error, never an exception [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_unreachable_is_reported]
- a3: a missing token file is reported naming the file [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_missing_token_is_reported]
- a4: the token never appears in results [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_token_never_in_text]
- a8: release_arm passes keep_position through [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_release_arm_keep_position]
- a5: RobotClient exists [evidence: symbol:nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py::RobotClient]
- a6: the lockfile is in sync [evidence: command:cd nodes/head_rgbd_calibration && uv lock --check]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py`
- `nodes/head_rgbd_calibration/pyproject.toml`
- `nodes/head_rgbd_calibration/uv.lock`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/test_robot_client.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

## Interfaces

- `robot_client.py::RobotClient`, `ToolResult`, helpers `set_head_angles`, `head_look_at`, `head_home`, `take_arm`, `move_arm_joints`, `release_arm`, `arm_state` (part `calib-robot-client`); config `robot`.

## Expected outcome

The calibrator moves the head and arm only through mcp_server's guarded tools (leases, collision and floor guards, battery gate).

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_robot_client.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- the node's service user must read the token (group `mcp-token`); the Ansible side is in calib-ansible-deploy-config.
- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: builder check: head tools need a lease, three arm tiers with confirms, gripper set once, mcp-token via supplementary_groups, optional drive check
