---
title: MCP base servo registers
status: todo
repo: ros2-lerobot-sverve-platform
value: Overheat and servo-error events for the swerve servos keep reaching the agent after the base moves to its own bus.
files: [nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/test_on_demand_subscriptions.py, nodes/mcp_server/README.md, ansible/group_vars/client.yml, tests/test_mcp_server_config.py, tests/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py tests/test_monitor.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint"
depends_on: []
owns: [topics.base_servo_registers]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP base servo registers

## Requirement

- `TopicSettings` (`mcp_server/config.py`): new field `base_servo_registers: str = "/swerve_drive/servo_registers"` next to `servo_registers`, commented as the base bus bridge's JSON register dump (swerve servos; head joints join it when the pan/tilt epic adds them to that bus); the `servo_registers` comment says arm servos (follower bus).
- `ros_iface.py`: `RosRobot` subscribes `t.base_servo_registers` at start with the same `self.on_servo_registers` callback as `t.servo_registers`; `"base_servo_registers"` is added to `MOTION_CRITICAL_TOPICS`, so it is never on demand.
- `ansible/group_vars/client.yml` mcp_server `topics:` gains `base_servo_registers: /swerve_drive/servo_registers` with a comment; the `servo_registers` comment says arm servos.
- Tests first, seen failing: `tests/test_on_demand_subscriptions.py::test_motion_critical_list_names_the_monitor_and_motion_feeds` expects `base_servo_registers`; a new test in the same file delivers a JSON dump `{"fl_drive": {"present_temperature": 70, "status": 0}}` on the base topic through the rig and asserts it reaches `RobotMonitor` (`fl_drive` in `monitor.servos` and a critical `overheat` level for `fl_drive`). Root `tests/test_mcp_server_config.py::test_mcp_server_monitor_topics_match_their_producers` asserts `topics["base_servo_registers"] == "/swerve_drive/servo_registers"` (the namespace consistency with `base_servos` is asserted by split-bus-group-vars).
- `nodes/mcp_server/README.md` lists both register dumps wherever `/follower/servo_registers` is described as the body monitor feed; `tests/README.md` rows for the changed and new tests.

## Acceptance conditions

- a1: the base register dump topic is a config field with the base bus default [evidence: symbol: nodes/mcp_server/mcp_server/config.py:TopicSettings]
- a2: base_servo_registers is motion-critical and subscribed at start [evidence: test: nodes/mcp_server/tests/test_on_demand_subscriptions.py::test_motion_critical_list_names_the_monitor_and_motion_feeds]
- a3: a dump on the base topic reaches the robot monitor and raises the swerve servo overheat [evidence: test: nodes/mcp_server/tests/test_on_demand_subscriptions.py]
- a4: client.yml wires the base dump topic [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_monitor_topics_match_their_producers]
- a5: mcp_server targeted tests, root config test and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py tests/test_monitor.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

## Interfaces

- `mcp_server.config.TopicSettings.base_servo_registers: str` (owned contract `topics.base_servo_registers`; split-bus-group-vars asserts it equals `/<base_servos namespace>/servo_registers`).
- `mcp_server.ros_iface.MOTION_CRITICAL_TOPICS` gains `"base_servo_registers"`; `RosRobot.on_servo_registers(msg: String) -> None` unchanged and shared.

## Expected outcome

mcp_server reads both register dumps from start; swerve servo temperatures and status errors keep producing robot events once they arrive on `/swerve_drive/servo_registers`.

## Steps

- Edit the tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py -q` and `uv run pytest tests/test_mcp_server_config.py -q` from the root; expect failures (unknown field, missing subscription; the strict model also rejects the client.yml key until the field exists).
- Implement config, ros_iface, client.yml, docs; run the verify command; commit `feat(mcp_server): subscribe the base bus servo register dump`.

## Notes

- Design D6. `TopicSettings` is a `StrictModel` (`extra="forbid"`): the client.yml key and the field must land in this one task.
- `follower_set_register` stays `/follower/set_register` (gripper grip profiles are arm writes).
- Until the split is deployed nothing publishes the base topic: the subscription waits, no placeholder data.
- Deploying mcp_server ends a running agent session (memory); the deploy itself is the orchestrator's runbook step.
