---
title: Final suite and runbook
status: done
repo: ros2-lerobot-sverve-platform
value: "Every suite is green, no live text puts the swerve servos on the arm bus, and the orchestrator has a staged robot runbook."
files: [tests/test_servo_bus_live_refs.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe lint-ansible && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)"
depends_on: [docs-and-diagnostics, bus-budget-test]
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: 202f5160a100b489feaf7fa2288e7c9717983c01
run: split-servo-buses-arm-alone
provides: []
acs_for_audit: [a4]
process_gaps: ["Implementer claimed the runbook was missing; it is in the task file's Steps section", "TDD skipped without a listed exception; prove the tests catch the change - commit first, then for each changed non-test file: git show <merge-base>:<path> > <path>; run the test (must fail); git show HEAD:<path> > <path>; run again (must pass); paste both tails into tests.red and verification.evidence"]
---

# Final suite and runbook

## Requirement

- New `tests/test_servo_bus_live_refs.py`, written first and seen failing if any hit remains: `test_no_live_text_puts_swerve_servos_on_the_arm_bus` runs `git grep -n -I -E` for `swerve servos (now )?run inside|shared with the swerve servos|on the arm bus|share the follower-arm serial bus|group .swerve_drive.|all servos incl. the swerve ones` over tracked files and fails on every hit outside `docs/plans/**`, `graphify-out/**` and this test file; `test_retired_swerve_drive_servos_name_is_gone` fails on any `swerve_drive_servos` outside `docs/plans/**`, `graphify-out/**`, `tests/**` and `MEMORY.md`; `test_no_feetech_entry_but_the_base_serves_swerve_ids` asserts that in client.yml only `base_servos` lists servo IDs 32-39 and only `lerobot_follower` lists IDs 1-6. `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, lint-ansible, test-ansible, feetech_servos, swerve_drive_controller, mcp_server).
- The staged runbook in this task's Steps is complete; the orchestrator executes it after merge (the implementer does not touch the robot).

## Acceptance conditions

- a1: no live text puts the swerve servos on the arm bus [evidence: test: tests/test_servo_bus_live_refs.py::test_no_live_text_puts_swerve_servos_on_the_arm_bus]
- a5: the retired swerve_drive_servos name is gone from live files [evidence: test: tests/test_servo_bus_live_refs.py::test_retired_swerve_drive_servos_name_is_gone]
- a2: only the base entry serves the swerve IDs and only the follower the arm IDs [evidence: test: tests/test_servo_bus_live_refs.py::test_no_feetech_entry_but_the_base_serves_swerve_ids]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe lint-ansible && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)]
- a4: the runbook covers device identification, deploy order keeping teleop configured, least-dramatic motion first and both bus rate gates [evidence: judgement]

## Files

- `tests/test_servo_bus_live_refs.py`
- `tests/README.md`

## Interfaces

- None new; `tests/test_servo_bus_live_refs.py`.

## Expected outcome

All suites green with a consistency check; the orchestrator rolls the split out on the robot with the runbook below.

## Steps

Implementer:
- Write `tests/test_servo_bus_live_refs.py`; run `uv run pytest tests/test_servo_bus_live_refs.py -q`; fix any real hit it reports; run the full verify command and quote each summary line; commit `test: no live text puts the swerve servos on the arm bus`.

Orchestrator runbook (after merge, robot reachable; every deploy through the ansible-deploy skill, logged to `.logs/`; no manual SSH, read-only Ansible ad-hoc commands only where noted):
0. Devices (before plan-execute when design.md A1/A2 are still `PENDING`, else before step 1). The user plugs the second adapter into the client Pi; the base servo chain is NOT moved yet. Identify read-only: `cd ansible && ansible client -m ansible.builtin.command -a 'ls -l /dev/serial/by-id/'`. Expect three entries: `usb-1a86_USB_Single_Serial_5A7A059004-if00` (arm, unchanged), one new adapter entry (A1) and the lidar's (A2, the one linking to `ttyUSB*`, Silicon Labs CP210x). Write A1/A2 into design.md. If the new adapter shows no unique by-id entry, stop and ask the user (A3). Baseline for the record: Grafana MCP `query_prometheus` `rate(servo_cycle_duration_seconds_count{host="client",node="lerobot_follower"}[5m])` (about 95.7) and `/battery_state` voltage.
1. Software first, base chain still on the arm adapter. Agent idle check is built into the client deploy. `mkdir -p .logs && ./scripts/deploy-nodes.sh client lerobot_follower base_servos swerve_controller mcp_server rplidar_a1 monitoring 2>&1 | tee .logs/deploy-client-bus-split.log` (one Ansible run; the serial preflight fails it before any restart if a device is missing). Expect: verify green; `./scripts/teleop_diag.sh` shows all three teleop topics PRESENT (arm teleop configured throughout); `base_servos` logs `Waiting for servos: [32, 33, 34, 35, 36, 37, 38, 39]` (read with `ansible client -m ansible.builtin.command -a 'journalctl -u ros2-base_servos -n 30 --no-pager'`); no `/swerve_drive/joint_states` yet (no placeholder data); `/scan` still published (`./scripts/swerve_diag.sh` and the lidar unit active).
2. Rewire (user, physical). Ask the user to power the servos off, move the base servo chain (IDs 32-39) from the arm adapter to the second adapter, power on, and confirm. Nothing is deployed in this step.
3. Base bus up. `base_servos` log shows all eight servos found and torque enabled, no repeated read failures; `./scripts/swerve_diag.sh`: `ros2-base_servos` and `ros2-swerve_controller` active, `/swerve_drive/joint_states` about 50 Hz, `/odom` present.
4. Bus rate gates, idle. Grafana: `rate(servo_cycle_duration_seconds_count{host="client",node="lerobot_follower"}[5m])` >= 98 and `{node="base_servos"}` >= 48; p99 `histogram_quantile(0.99, rate(servo_cycle_duration_seconds_bucket{host="client",node="lerobot_follower"}[5m]))` <= 0.01. topic_scraper workflow: set `enabled: true` for `topic_scraper_api` in `ansible/group_vars/client.yml` (local edit, not committed), `./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-client-scraper.log`, then `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select '/follower/joint_states:.header.stamp' --select '/swerve_drive/joint_states:.header.stamp' --interval 1.0 > .logs/bus-rates-idle.ndjson` for 60 s; rate per topic = delta `sample_seq` / delta `received_at_ns`: follower >= 98 Hz, base >= 48 Hz.
5. Least-dramatic motion. Arm first: a gripper-only move of about 0.1 rad through test_joint_api (`python scripts/joint_api_client.py post --joint gripper <value>`), then the user drives leader teleop briefly; `/rules/leader_vs_follower` on the scraper shows tracking. Base: with the user's go-ahead and a clear floor, robot MCP `move_relative` 0.03 m forward (steering stays centred, about 3 cm), then 0.10 m while collecting `.logs/bus-rates-driving.ndjson` as in step 4: follower stays >= 98 Hz while the base drives (the shared bus projected 66 Hz), base >= 48 Hz; `/odom` advances about 0.10 m.
6. Consumers. `ansible client -m ansible.builtin.shell -a 'source /opt/ros/jazzy/setup.bash && source /etc/profile.d/ros2_dds.sh && ros2 topic info -v /swerve_drive/set_register && ros2 topic info /battery_state' -e ansible_shell_executable=/bin/bash`: `/swerve_drive/set_register` subscribed by `base_servos` and published by swerve_controller; `/battery_state` has exactly one publisher, voltage within 0.2 V of the baseline. Robot MCP `get_body_state` reports swerve servo temperatures (fl_drive ... rr_steer) within 2 minutes.
7. Restore `topic_scraper_api` to `enabled: false` (`git checkout ansible/group_vars/client.yml`) and redeploy it.
Rollback (only if step 3-6 fail and cannot be fixed): power off, move the base chain back to the arm adapter, `git revert` the epic's commits, push, redeploy the same node list.

## Notes

- Design D10. A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- If the robot is offline, runbook steps 0-7 are reported as pending on the robot (each named), with everything else done.
- Servo safety (CLAUDE.md): gripper before any other arm joint; wheels at minimal speed and travel; stop and ask before anything larger.
