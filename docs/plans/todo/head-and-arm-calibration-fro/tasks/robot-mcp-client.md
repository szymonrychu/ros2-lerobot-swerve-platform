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
  - Typed helpers (sync wrappers running the coroutine, safe from a worker thread): `set_head_angles(pan_deg, tilt_deg)`, `head_look_at(x, y, z)` (base_link), `head_home()`, `take_arm()` (`take_control(subsystem="arm")`), `move_arm_joints(targets, speed_scale)` (timeout `move_timeout_s`), `release_arm()` (`release_control(subsystem="arm")`, parks home), `arm_state()`. A tool error comes back as `is_error` with the tool's text.
- New `tests/test_robot_client.py` (fake transport): tool names and argument shapes per helper; unreachable -> `is_error`, `unreachable`, text `mcp_server unreachable at <url>`; missing token file -> `is_error` naming the file; timeout -> `is_error` with the timeout; the token string never appears in any text. `tests/test_config.py`: robot defaults.

Steps: tests first (red), implement, verify.

Notes: the node's service user must read the token (group `mcp-token`); the Ansible side is in calib-ansible-deploy-config.

## Acceptance conditions

- a1: each helper calls its tool with the documented arguments [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_helpers_call_tools_with_arguments]
- a2: unreachable mcp_server is a reported error, never an exception [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_unreachable_is_reported]
- a3: a missing token file is reported naming the file [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_missing_token_is_reported]
- a4: the token never appears in results [evidence: test:nodes/head_rgbd_calibration/tests/test_robot_client.py::test_token_never_in_text]
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
