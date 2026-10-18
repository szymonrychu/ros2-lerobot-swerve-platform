---
title: Split bus group vars
status: todo
repo: ros2-lerobot-sverve-platform
value: "The arm owns its bus and the swerve servos run in their own bridge on the second adapter, with every consumer wired to it."
files: [ansible/group_vars/client.yml, tests/test_servo_bus_split_config.py, tests/test_node_metrics_wiring.py, tests/test_battery_config.py, tests/README.md]
verify: "uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py tests/test_mcp_server_config.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [bridge-instance-identity, tilt-guard-torque-topic, mcp-base-servo-registers]
owns: [swerve_drive_servos entry]
uses: [BridgeConfig.node_name, TiltGuardConfig.torque_topic default, topics.base_servo_registers]
model: opus
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Split bus group vars

## Requirement

- `lerobot_follower` config (client.yml): only the six arm joints (IDs 1-6, gripper source/command steps unchanged); `extra_groups`, `velocity_command_timeout_s` and the swerve comments removed; `node_name: lerobot_follower` added; `device` stays `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00`; control_loop_hz stays the default 100; battery, `publish_effort_joints`, `direct_command_sources`, goal refresh and read failure keys unchanged; the `publish_current_joints` cost comment is restated for the 6-servo bus (+54 bytes per cycle) with the value still `[]`.
- `swerve_drive_servos` entry: `present: true`, `enabled: true`, `env: [FEETECH_SERVOS_CONFIG=/etc/ros2/swerve_drive_servos/config.yaml]`, config: `metrics_port: 19115`, `node_name: swerve_drive_servos`, `device:` the second adapter's by-id path recorded in design.md "Assumptions and unknowns" A1, `namespace: swerve_drive`, `log_joint_updates: false`, `enable_torque_on_start: true`, `register_publish_interval_s: 10`, `control_loop_hz: 50`, `battery_interval_s: 0`, `velocity_command_timeout_s: 0.3`, `goal_refresh_interval_s: 1.0`, `read_failure_limit: 3`, `read_retry_interval_s: 5.0`, and the eight joints moved verbatim with their comments: fl_drive 32 velocity inverted, fl_steer 33, fr_drive 35 velocity, fr_steer 34, rl_drive 39 velocity inverted, rl_steer 38, rr_drive 36 velocity, rr_steer 37. A comment marks where the pan/tilt epic adds `extra_groups: [{namespace: head, joint_names: [...]}]` (design D7); no head joints are configured. The old "moved onto the lerobot_follower bus" comment is replaced.
- `swerve_controller` `tilt_guard.torque_topic: /swerve_drive/set_register`.
- `monitoring_agent_node_targets` gains `{node: swerve_drive_servos, address: "127.0.0.1:19115", path: /metrics}`.
- Tests first, seen failing: new `tests/test_servo_bus_split_config.py` asserting: the follower serves exactly IDs 1-6, has no `extra_groups`, `node_name` lerobot_follower, the arm adapter path, control rate 100 (unset or 100); `swerve_drive_servos` is present and enabled with namespace `swerve_drive`, the exact eight joints above (ID, mode, inverted), rate 50, battery 0, metrics 19115, velocity timeout 0.3, torque on start; both devices are `/dev/serial/by-id/` paths and differ; the client's present feetech_servos entries are exactly these two, with distinct `node_name` and `metrics_port` and disjoint servo IDs; `swerve_controller` `joint_names` equal the base joint names and its `joint_states_topic` / `joint_commands_topic` are `/swerve_drive/joint_states` / `/swerve_drive/joint_commands`; `tilt_guard.torque_topic == "/swerve_drive/set_register"`; mcp_server `topics.base_servo_registers == "/swerve_drive/servo_registers"`, `topics.servo_registers == "/follower/servo_registers"`, `topics.follower_set_register == "/follower/set_register"`; the type defaults give the entry the critical cgroup tier (no per-entry `cpu_weight`/`io_weight`/`oom_score_adjust` override). `tests/test_node_metrics_wiring.py` `CONFIG_METRICS_PORTS` gains `"swerve_drive_servos": 19115`. `tests/test_battery_config.py` gains a test that exactly one client feetech_servos entry has `battery_interval_s > 0` (lerobot_follower) and `swerve_drive_servos` has 0. `tests/README.md` rows for all of them.

## Acceptance conditions

- a1: the follower config serves only the arm on its adapter [evidence: test: tests/test_servo_bus_split_config.py]
- a2: swerve_drive_servos carries the eight swerve joints verbatim at 50 Hz on its own by-id device with battery off and metrics 19115 [evidence: test: tests/test_servo_bus_split_config.py]
- a3: swerve_controller, the tilt guard and mcp_server point at the base bridge topics [evidence: test: tests/test_servo_bus_split_config.py]
- a4: the base bridge metrics endpoint has an Alloy target [evidence: test: tests/test_node_metrics_wiring.py::test_every_enabled_contract_node_has_a_matching_scrape_target]
- a5: exactly one feetech instance publishes the battery [evidence: test: tests/test_battery_config.py]
- a6: client.yml has no placeholder device value [evidence: command: ! git grep -n -E 'PENDING|by-id/REPLACE|CHANGEME' -- ansible/group_vars/client.yml]
- a7: root targeted tests, lint and ansible syntax pass [evidence: command: uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py tests/test_mcp_server_config.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_servo_bus_split_config.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_battery_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
