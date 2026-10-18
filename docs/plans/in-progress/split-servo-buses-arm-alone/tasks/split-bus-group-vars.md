---
title: Split bus group vars
status: done
repo: ros2-lerobot-sverve-platform
value: "The arm owns its bus and the swerve servos run in their own bridge on the second adapter, with every consumer wired to it."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, tests/test_servo_bus_split_config.py, tests/test_node_metrics_wiring.py, tests/test_battery_config.py, tests/README.md]
verify: "uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py tests/test_mcp_server_config.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [bridge-instance-identity, tilt-guard-torque-topic, mcp-base-servo-registers]
owns: [base_servos entry]
uses: [BridgeConfig.node_name, TiltGuardConfig.torque_topic default, topics.base_servo_registers]
model: opus
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/split-servo-buses-arm-alone--split-bus-group-vars
branch: split-servo-buses-arm-alone--split-bus-group-vars
verified_head: b0aae1373e120f3b36cd3d74332c996a8bbcbb7c
run: split-servo-buses-arm-alone
provides: [base-servos-entry]
acs_for_audit: []
process_gaps: []
---

# Split bus group vars

## Requirement

- `lerobot_follower` config (client.yml): only the six arm joints (IDs 1-6, gripper source/command steps unchanged); `extra_groups`, `velocity_command_timeout_s` and the swerve comments removed; `node_name: lerobot_follower` added; `device` stays `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00`; control_loop_hz stays the default 100; battery, `publish_effort_joints`, `direct_command_sources`, goal refresh and read failure keys unchanged; the `publish_current_joints` cost comment is restated for the 6-servo bus (+54 bytes per cycle) with the value still `[]`.
- The retired `swerve_drive_servos` entry (`present: false` since `03b43eb5`, already uninstalled on the robot) is deleted, and the playbook step `Deploy swerve_drive_servos` in `ansible/playbooks/deploy_nodes_client.yml` becomes `Deploy base_servos` (`_deploy_node_name: base_servos`, tags `[base_servos, apt, python, build, config]`, apply tag `base_servos`), keeping its position after `Deploy lerobot_follower` (design D1).
- New `base_servos` entry: `present: true`, `enabled: true`, `env: [FEETECH_SERVOS_CONFIG=/etc/ros2/base_servos/config.yaml]`, config: `metrics_port: 19115`, `node_name: base_servos`, `device:` the second adapter's by-id path recorded in design.md "Assumptions and unknowns" A1, `namespace: swerve_drive`, `log_joint_updates: false`, `enable_torque_on_start: true`, `register_publish_interval_s: 10`, `control_loop_hz: 50`, `battery_interval_s: 0`, `velocity_command_timeout_s: 0.3`, `goal_refresh_interval_s: 1.0`, `read_failure_limit: 3`, `read_retry_interval_s: 5.0`, and the eight joints moved verbatim with their comments: fl_drive 32 velocity inverted, fl_steer 33, fr_drive 35 velocity, fr_steer 34, rl_drive 39 velocity inverted, rl_steer 38, rr_drive 36 velocity, rr_steer 37. A comment marks where the pan/tilt epic adds `extra_groups: [{namespace: head, joint_names: [...]}]` (design D7); no head joints are configured. The old "moved onto the lerobot_follower bus" comment is replaced.
- `swerve_controller` `tilt_guard.torque_topic: /swerve_drive/set_register`.
- `monitoring_agent_node_targets` gains `{node: base_servos, address: "127.0.0.1:19115", path: /metrics}`.
- Tests first, seen failing: new `tests/test_servo_bus_split_config.py` asserting: the follower serves exactly IDs 1-6, has no `extra_groups`, `node_name` lerobot_follower, the arm adapter path, control rate 100 (unset or 100); `base_servos` is present and enabled with namespace `swerve_drive`, the exact eight joints above (ID, mode, inverted), rate 50, battery 0, metrics 19115, velocity timeout 0.3, torque on start; no `ros2_nodes` entry and no playbook step is named `swerve_drive_servos`; both devices are `/dev/serial/by-id/` paths and differ; the client's present feetech_servos entries are exactly these two, with distinct `node_name` and `metrics_port` and disjoint servo IDs; `swerve_controller` `joint_names` equal the base joint names and its `joint_states_topic` / `joint_commands_topic` are `/swerve_drive/joint_states` / `/swerve_drive/joint_commands`; `tilt_guard.torque_topic == "/swerve_drive/set_register"`; mcp_server `topics.base_servo_registers == "/swerve_drive/servo_registers"`, `topics.servo_registers == "/follower/servo_registers"`, `topics.follower_set_register == "/follower/set_register"`; the type defaults give the entry the critical cgroup tier (no per-entry `cpu_weight`/`io_weight`/`oom_score_adjust` override). `tests/test_node_metrics_wiring.py` `CONFIG_METRICS_PORTS` gains `"base_servos": 19115`. `tests/test_battery_config.py` gains a test that exactly one client feetech_servos entry has `battery_interval_s > 0` (lerobot_follower) and `base_servos` has 0. `tests/README.md` rows for all of them.

## Acceptance conditions

- a1: the follower config serves only the arm on its adapter [evidence: test: tests/test_servo_bus_split_config.py]
- a2: base_servos carries the eight swerve joints verbatim at 50 Hz on its own by-id device with battery off and metrics 19115 [evidence: test: tests/test_servo_bus_split_config.py]
- a3: swerve_controller, the tilt guard and mcp_server point at the base bridge topics [evidence: test: tests/test_servo_bus_split_config.py]
- a4: the base bridge metrics endpoint has an Alloy target [evidence: test: tests/test_node_metrics_wiring.py::test_every_enabled_contract_node_has_a_matching_scrape_target]
- a8: the base bridge has a tagged deploy step and the retired name is gone [evidence: test: tests/test_ansible_deploy_speed.py::test_every_node_has_a_tagged_deploy_step]
- a5: exactly one feetech instance publishes the battery [evidence: test: tests/test_battery_config.py]
- a6: client.yml has no placeholder device value [evidence: command: ! git grep -n -E 'PENDING|by-id/REPLACE|CHANGEME' -- ansible/group_vars/client.yml]
- a7: root targeted tests, lint and ansible syntax pass [evidence: command: uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py tests/test_mcp_server_config.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_servo_bus_split_config.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_battery_config.py`
- `tests/README.md`

## Interfaces

- Owned contract `base_servos entry` in `ansible/group_vars/client.yml` (`ros2_nodes`), read by serial-device-preflight, bus-budget-test and docs-and-diagnostics.
- Uses `BridgeConfig.node_name` (key `node_name`), `TiltGuardConfig.torque_topic` (same value as its default) and `TopicSettings.base_servo_registers`.
- Playbook step `Deploy base_servos` (renamed from `Deploy swerve_drive_servos`); `tests/test_ansible_deploy_speed.py::test_every_node_has_a_tagged_deploy_step` keeps entries and steps in sync.

## Expected outcome

One deploy of `lerobot_follower base_servos swerve_controller mcp_server monitoring` gives an arm-only follower and a base bridge on the second adapter, with all base consumers on `/swerve_drive/*`.

## Steps

- Confirm design.md A1 holds a real `/dev/serial/by-id/...` path (not `PENDING`); if not, stop as blocked: the orchestrator fills it from the user or runbook stage 0.
- Write `tests/test_servo_bus_split_config.py` and the two test edits; run `uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py -q`; expect failures on the follower's extra group and the absent base entry.
- Edit client.yml and the playbook step; run the verify command; commit `feat(ansible): swerve servos on their own bus via base_servos`.

## Notes

- Design D1, D3, D4, D7, D8. The left-wheel inversion (32, 39) was verified on the bench 2026-10-03: carry it, never change IDs, modes or inversion.
- The head is not configured here (non-goal); only the comment.
- The feetech schema assert in `ros2_node_deploy` already validates the new entry's joints.
- If a pre-existing suite failure blocks the verify command, clear it per the boy-scout rule as its own commit.

## Ruling

Accepted by the orchestrator: verified and merged (feetech 294, mcp_server 2499); docs-and-diagnostics failed only on a verifier crash, its checks pass by hand.
