---
title: Ansible servo neutral config
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "One client deploy turns the feature on with consistent names everywhere, pinned by a root test."
files: [ansible/group_vars/client.yml, tests/test_servo_neutral_config.py, tests/README.md, ansible/README.md]
verify: "uv run pytest tests/test_servo_neutral_config.py tests/test_head_servo_config.py tests/test_servo_bus_split_config.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [bridge-runtime-neutral-save, filter-node-released-arm, swerve-released-steering, mcp-arm-calibration-stale, web-ui-quiet-released-groups]
owns: []
uses: [neutral-services, arm-stale-marker]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--ansible-servo-neutral-config
branch: servo-torque-off-and-neutral--ansible-servo-neutral-config
verified_head: ""
run: ""
provides: []
---

# Ansible servo neutral config

## Requirement

- `ansible/group_vars/client.yml`, keeping the `ros2_nodes` schema:
  - `lerobot_follower` config: `neutral_enabled: true`, `neutral_exclude_joints: [gripper]` (D4, Q1), `neutral_stale_marker: /var/lib/ros2/arm/calibration_stale.json`, `torque_state_interval_s: 1.0`, with a comment block (D-C1, D-C2: follower only, the leader keeps its zero).
  - `base_servos` config: `neutral_enabled: true` on the primary `swerve_drive` group (the 4 steering servos; the wheels are velocity joints and never released) and on the `head` extra group; `torque_state_interval_s: 1.0`; the steering comment updated (re-zero from the web UI Calibrate page; `nodes/bridges/feetech_servos/scripts/calibrate_servos.py center` stays for offline use).
  - `filter_node`: `follower_torque_state_topic: /follower/torque_state`; `swerve_controller`: `torque_state_topic: /swerve_drive/torque_state`; `mcp_server`: the three torque state topics, `timeouts.servo_torque_stale_s`, `arm.calibration_stale_marker` with the same path.
  - `web_ui` `calibration.servo_groups`: arm (`follower`, neutral, `stale_marker` same path, `command_topics` the arm tab command topic), base (`swerve_drive`, neutral, the map goal topic), head (`head`, neutral, the head command topic).
- `server.yml` leader bridge is unchanged (no neutral keys, D-C2).
- New root test `tests/test_servo_neutral_config.py`, written first and seen failing: every web_ui servo group namespace is a group of a client feetech bridge and `neutral` matches its `neutral_enabled`; the base neutral joints are exactly fl/fr/rl/rr_steer (no velocity joint); `neutral_exclude_joints` of lerobot_follower is `[gripper]`; the stale marker path is identical in lerobot_follower, mcp_server and web_ui; the torque state topics in filter_node, swerve_controller, mcp_server equal `/{namespace}/torque_state` of a configured bridge group; the server.yml leader has no `neutral_*` key; web_ui `command_topics` equal the configured arm/head/goal topics; `/var/lib/ros2/arm` is created on the client (`ansible/playbooks/tasks/mcp_server_setup.yml`).
- `tests/README.md` lists the new test file and its tests; `ansible/README.md` documents the new keys and the marker file next to `/var/lib/ros2/arm`.

## Acceptance conditions

- a1: client.yml enables neutral on the follower (gripper excluded), steering and head groups with one marker path [evidence: test: tests/test_servo_neutral_config.py]
- a2: every consumer's torque state topic and the web_ui groups match configured bridge groups; the leader has no neutral keys [evidence: test: tests/test_servo_neutral_config.py]
- a3: tests/README.md lists test_servo_neutral_config.py [evidence: command: grep -q 'test_servo_neutral_config.py' tests/README.md]
- a4: root tests, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_servo_neutral_config.py`
- `tests/README.md`
- `ansible/README.md`

## Interfaces

- Uses parts `neutral-services` and `arm-stale-marker`; sets the config keys introduced by bridge-runtime-neutral-save, filter-node-released-arm, swerve-released-steering, mcp-torque-off-gate, mcp-arm-calibration-stale, web-ui-servo-routes and web-ui-quiet-released-groups.

## Expected outcome

One deploy of the client nodes turns the feature on with consistent names everywhere, pinned by a root test.

## Steps

- Write `tests/test_servo_neutral_config.py`; run `uv run pytest tests/test_servo_neutral_config.py -q`; expect failures.
- Edit client.yml; run the verify command (also `tests/test_head_servo_config.py` and `tests/test_servo_bus_split_config.py` must stay green); commit `feat(ansible): servo torque and neutral config`.

## Notes

- Do not deploy here; the runbook task deploys after merge.
- The head command steps stay as they are: they are relative to the servo centre and pinned by tests/test_head_servo_config.py.

- 2026-10-20 amended: builder check: full calibrate_servos.py path
