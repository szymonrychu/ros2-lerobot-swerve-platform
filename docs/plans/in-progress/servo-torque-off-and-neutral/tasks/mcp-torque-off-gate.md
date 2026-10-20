---
title: MCP torque off gate
status: done
repo: ros2-lerobot-sverve-platform
value: The agent cannot move a released group and stops republishing a held arm setpoint into a limp arm.
files: [nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/arm.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/models.py, nodes/mcp_server/tests/test_torque_gate.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_torque_gate.py tests/test_lease_safety.py tests/test_battery_gate.py tests/test_head_lease.py -q && uv run --frozen poe lint"
depends_on: [bridge-torque-services-topic, filter-node-released-arm]
owns: []
uses: [torque-state-topic]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--mcp-torque-off-gate
branch: servo-torque-off-and-neutral--mcp-torque-off-gate
verified_head: fdc4fd547801805534588b8f2661e2b485413c01
run: servo-torque-off-and-neutral
provides: []
acs_for_audit: []
process_gaps: []
---

# MCP torque off gate

## Requirement

- Config (`config.py`, the topics section next to `follower_set_register`): `follower_torque_state` (`/follower/torque_state`), `base_torque_state` (`/swerve_drive/torque_state`), `head_torque_state` (`/head/torque_state`); `timeouts.servo_torque_stale_s` (default 3.0, > 0).
- `ros_iface.RosRobot` subscribes to the three (reliable + transient_local, depth 1), caches `released` with its receipt time, and exposes `servo_released(group: str) -> bool | None` for `arm`, `base`, `head` (None when no sample or older than `servo_torque_stale_s`). At the publish level, an arm command (`publish_command`) is not published while `arm` is released and a head command (`publish_head_command`) is not published while `head` is released (one debug log line, no exception), so keepalive republish and the head home-on-lease-release cannot reach a limp group.
- `tools.py`: `TOOL_SERVO_GROUPS` (next to `MOTION_TOOLS`) maps every motion tool to the servo groups it drives (arm tools -> arm; base motion tools -> base; head tools -> head; grasp and approach macros -> every group they move); a test checks every `MOTION_TOOLS` entry is mapped. `call_tool_with_digest` (after the lease check, before the battery and tilt gates) refuses a tool whose group is released with a tool error `<group> servo torque is off (released on the web UI Calibrate page); motion refused`, publishing nothing. Unknown or stale state refuses nothing (the bridge gate is the backstop).
- Arm lease: when `arm` becomes released while mcp_server holds the arm lease, the lease is dropped without publishing (`drop_lease`), `lost_note` reads `arm torque off`; `take_control` for the arm is refused with the same torque message while released. filter_node reports the active source `released` in that state (filter-node-released-arm); `lease_lost_to` treats it as the torque-off case, not a takeover.
- `models.py`: `get_robot_state` and `get_control_state` carry `servos_released: list[str]` (only groups with a fresh released state).
- Tests first, seen failing, in new `tests/test_torque_gate.py` with the fake backend: tool refused per group and nothing published; unknown/stale allows; publish suppressed at ros_iface level; lease dropped on release with the note; take_control refused; mapping covers MOTION_TOOLS.
- `README.md` gets `## Servo torque-off gate` (topics, mapping, messages).

## Acceptance conditions

- a1: motion tools of a released group are refused before anything is published; unknown state allows [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a2: arm and head commands are not published while their group is released [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a3: an arm release drops the held lease with the torque-off note and take_control is refused [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a4: TOOL_SERVO_GROUPS covers every MOTION_TOOLS entry [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a5: README documents the gate [evidence: doc-section: nodes/mcp_server/README.md#Servo torque-off gate]
- a6: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q -x && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/arm.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/tests/test_torque_gate.py`
- `nodes/mcp_server/README.md`

## Interfaces

- Uses part `torque-state-topic` (`/follower/torque_state`, `/swerve_drive/torque_state`, `/head/torque_state`, field `released`) and the filter_node active source `released`.
- `RosRobot.servo_released(group: str) -> bool | None`; `tools.TOOL_SERVO_GROUPS: dict[str, frozenset[str]]`; `servos_released` on the state models.

## Expected outcome

The agent cannot move a released group, does not keep republishing a held arm setpoint into a limp arm, and sees which groups are released.

## Steps

- Write `tests/test_torque_gate.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_torque_gate.py -q`; expect failures.
- Implement config, subscriptions, the gate, the publish suppression and the lease drop; run `uv run --frozen pytest tests/test_torque_gate.py tests/test_lease_safety.py tests/test_battery_gate.py tests/test_head_lease.py -q`; then the verify command; commit `feat(mcp_server): refuse motion of released servo groups`.

## Notes

- Pattern to copy: the battery cut-off gate (`MOTION_TOOLS`, `ALWAYS_ALLOWED_TOOLS` in tools.py, `call_tool_with_digest` tools.py:490) and the tilt gate.
- The arm keepalive republishes the held setpoint at 5 Hz (arm.py `keepalive_tick`, brief fact 14); the publish-level suppression plus the lease drop stop it.
- `tests/conftest.py` already stubs std_srvs; follow `tests/test_battery_gate.py` for the fake backend.

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on filter-node-released-arm
