# Split servo buses: arm alone, base plus head design

## Summary

The SO-101 follower arm keeps the existing Feetech USB adapter alone; the eight swerve servos (IDs 32-39) move to a
second USB-serial adapter driven by a second instance of the unchanged `feetech_servos` bridge, named `base_servos` (shaped like the retired
`swerve_drive_servos`, namespace `swerve_drive`). Topics stay `/swerve_drive/joint_states` and
`/swerve_drive/joint_commands`; the two consumers of the bridge's per-instance topics (`set_register`,
`servo_registers`) are re-pointed. The base bus runs at 50 Hz (the swerve_controller loop), the arm bus stays at
100 Hz. A fake-bus byte-count test anchored on the robot's measured 95.7 Hz shows the arm-only bus at about 108 Hz of
capacity and base + a 2-servo head at about 69 Hz of capacity while driving. A staged runbook deploys the software
before the base servo chain is rewired, so arm teleop is never left unconfigured.

## Intent and why

The arm and the swerve base share one Feetech bus inside `lerobot_follower`. It runs at about 95.7 Hz against its
95 Hz gate (`ansible/group_vars/client.yml:1146-1148`); 126 extra bytes per cycle already broke the gate once
(93.4 Hz, rolled back, `MEMORY.md:140`). The fake-bus probe below shows it is worse while driving: the four wheel
goal writes are single transactions with replies, so a driving cycle costs about 8.9 ms of bus time against 4.3 ms
idle, which projects the shared bus to about 66 Hz while the base moves. The pan/tilt head (next epic) needs bus
capacity, and the arm teleop loop must stay fast. A second adapter (U1) gives the arm its own bus (U2) and puts the
base and later the head on the other (U3).

## Goals

- G1: `lerobot_follower` drives only the arm (IDs 1-6) on adapter `usb-1a86_USB_Single_Serial_5A7A059004`; on the
  robot `/follower/joint_states` publishes at >= 98 Hz over 60 s, idle and while the base drives (old gate 95 Hz).
- G2: `base_servos` (feetech_servos) drives IDs 32-39 on the second adapter through its
  `/dev/serial/by-id/` path, same joint names, modes, inversion, velocity watchdog (0.3 s) and startup torque
  behaviour; swerve_controller, the tilt guard and mcp_server keep working on the same joint topics.
- G3: the base bus config has room for a `head` extra group; `tests/test_bus_budget.py` (fake bus, real configs from
  client.yml) shows base + head fit the base rate target and the arm-only bus keeps >= 5 % headroom at 100 Hz.
- G4: Ansible entries, serial preflight, cgroup tier, metrics target, READMEs, tests, the teleop-diagnostics and
  ansible-deploy skills and MEMORY.md are updated; deployed and verified on the robot with least-dramatic motion first.

## Non-goals

- Adding the head servos, head IDs, head topics or the head lease (pan/tilt epic).
- Changing servo firmware, IDs, baud rate or calibration.
- Re-enabling `publish_current_joints` on the now faster arm bus (not in the approved goals).
- A udev rule: the adapters expose unique serial numbers, so `/dev/serial/by-id/` is stable (D2).

## Anti-goals

- Arm teleop left without a working bridge at any point of the rollout (a restarted old-config follower blocks
  forever in `_wait_for_servos` once IDs 32-39 leave its bus, `nodes/bridges/feetech_servos/bridge.py:70-93`).
- A tilt-guard `torque_off` write silently dropped because it still goes to `/follower/set_register`, where the
  drive joints are no longer known (`set_register.py:56`, `unknown_joint`).
- Two bridges publishing `/battery_state`, two ROS nodes named `feetech_servos_bridge`, or two metrics endpoints
  both labelled `lerobot_follower`.
- Any placeholder joint state on either bus (CLAUDE.md "No Placeholder Data on Topics").
- Bridge code forked per bus: one codebase, two configs.

## Constraints

- CLAUDE.md: Python type hints, constants at the top, docstrings with types, tests first, per-node uv, Ansible kept in
  sync with nodes via the `ros2_nodes` schema, deploy only through `scripts/deploy-nodes.sh` with logs in `.logs/`,
  servo tests least-dramatic first (gripper; wheel yaw axes, minimal amplitude).
- The bridge is one serial port per process (`run_bridge`), `extra_groups` share that port.
- Metrics ports: 19113 is claimed by the head-camera epic (`docs/plans/todo/front-camera-30-fps-stream-w/design.md:237`),
  19114 is the retired Jetson bridge asserted absent by `tests/test_jetson_retired.py:112-114`; this epic uses 19115.
- This epic starts after `remove-the-jetson-coprocesso` and `front-camera-30-fps-stream-w`; both edit
  `ansible/group_vars/client.yml` and `tests/README.md`, so tasks rebase onto them.
- Deploying mcp_server kills a running agent session (memory); client deploys wait for an idle agent.
- No robot access while planning: unknown device paths are confirmed before the run (Assumptions A1, A2).

## Decisions

### D1: Second `feetech_servos` instance `base_servos`, shaped like the retired `swerve_drive_servos`

A new client entry `base_servos` (node_type `feetech_servos`, `present: true, enabled: true`) takes primary namespace
`swerve_drive` and the eight joints moved verbatim from `lerobot_follower`'s `extra_groups` (IDs, `mode: velocity`,
`inverted: true` on 32 and 39), plus `velocity_command_timeout_s: 0.3`, `enable_torque_on_start: true`,
`goal_refresh_interval_s: 1.0`, `read_failure_limit: 3`, `read_retry_interval_s: 5.0`,
`register_publish_interval_s: 10`. `lerobot_follower` keeps only the arm and loses `extra_groups` and
`velocity_command_timeout_s`. This is the shape of the `swerve_drive_servos` entry removed in `03b43eb5` (own device,
namespace `swerve_drive`, same node type). The name is `base_servos` because the bus carries the base and later the
head (U3, U4) and the pan/tilt epic already plans against that name
(`docs/plans/todo/pan-tilt-head-control-with-a/design.md:70`) (my call). The retired `swerve_drive_servos` entry
(`present: false` since `03b43eb5`, already uninstalled on the robot by later deploys) is deleted and its playbook step
(`ansible/playbooks/deploy_nodes_client.yml:336-343`) is renamed to `Deploy base_servos`. Alternatives: re-enable the
name `swerve_drive_servos` (rejected: misnames a bus that will carry the head, and diverges from the sibling epic); one
process with two ports (rejected: the bridge loop, metrics and executor are single-bus, it would be a rewrite).
Serves G1, G2, U1, U2, U3.

### D2: Device addressing by `/dev/serial/by-id`, Ansible preflight, lidar pinned

Both feetech instances name their adapter by `/dev/serial/by-id/...` (existing pattern: client `5A7A059004`, server
`5A7A059057`, both unique CH34x serials), so no udev rule is added. `ros2_node_deploy` gains a preflight next to the
feetech schema assert (`roles/ros2_node_deploy/tasks/main.yml:41-62`): for an enabled feetech_servos node whose
config sets `device`, `stat` the path and fail with the `/dev/serial/by-id/` listing when it is missing. Restarts are
queued to the end of the play (`start_ros_nodes.yml`), so a missing adapter aborts the run before any unit restarts.
`rplidar_a1` still opens `/dev/ttyUSB0` by default (`nodes/bridges/rplidar_a1/launch/rplidar_a1.launch.py:15`);
a second USB-serial adapter can take that name on boot, so the lidar is pinned through `RPLIDAR_SERIAL_PORT` to its
by-id path (my call). Alternatives: udev `SYMLINK+=` rules (rejected: by-id already unique, one more file to keep);
`/dev/serial/by-path` (rejected: breaks when a cable moves port). Serves G2, G4, U1.

### D3: Instance identity: `node_name`, battery off on the base, metrics port 19115

New bridge config key `node_name` (default unset: rclpy node `feetech_servos_bridge` and metrics label
`lerobot_follower`, exactly today's behaviour, so the server's `lerobot_leader` is untouched). When set it names the
rclpy node and the `robot_node_info` label (`bridge.py:66,103,105`). Client: `lerobot_follower` and
`base_servos` set their own names. The battery monitor reads round-robin over the instance's servos
(`bridge.py:292`); the base sets `battery_interval_s: 0` and the bridge then creates no BatteryState publisher, so
`/battery_state` has one publisher (the follower, reading the 6 arm servos on the same pack). Base metrics on
`127.0.0.1:19115` with an Alloy target (D8). Serves G2, G4.

### D4: Base bus at 50 Hz, arm bus at 100 Hz

The base bridge runs `control_loop_hz: 50`, equal to swerve_controller's `control_loop_hz: 50`, which publishes
exactly one command per cycle, so the base sees every command (README: base and head run at lower rates) (my call).
At 100 Hz the base would not fit while driving (projection 15 ms per cycle, below). The arm keeps the default 100 Hz.
`/swerve_drive/joint_states` drops from about 95 Hz to 50 Hz; its consumers (swerve_controller FK at 50 Hz,
`joint_state_throttle_swerve` 10 Hz copy, mcp_server lazy reader, web_ui 3D) need no more. Serves G1, G3, U2, U3.

### D5: Bus budget test anchored on the robot measurement

`nodes/bridges/feetech_servos/tests/test_bus_budget.py` loads the `lerobot_follower` and `base_servos`
config blocks from `ansible/group_vars/client.yml` through `feetech_servos.config.load_config`, builds the emulated
buses of `tests/sts_bus.py` with exactly those servos, and runs the worst-case cycle of `test_bus_io.py:400`
(every goal due for refresh, wheel velocities changing, one state sync read). Projection: the measured shared cycle
(1/95.7 Hz = 10.45 ms, `MEMORY.md:140`) minus the fake wire time of the old 14-servo layout (idle, 4.28 ms) is the
non-bus remainder (6.17 ms), kept fixed (conservative, per-servo CPU savings are not credited); projected cycle =
remainder + fake wire time of the new layout. Probe results (real st3215 handler on the fake bus, this planning run):

| Layout | Fake wire time (ms) | Projected cycle (ms) | Capacity (Hz) | Target |
|---|---|---|---|---|
| shared 14 servos, idle / driving | 4.28 / 8.88 | 10.45 / 15.05 | 95.7 / 66 | 100 |
| arm only (6) | 3.12 | 9.29 | 108 | 100: 7 % headroom |
| arm + head (8) | 3.44 | 9.61 | 104 | 100 (U4 option) |
| base + head (10), driving | 8.24 | 14.41 | 69 | 50: 28 % headroom |

Assertions: arm projected cycle <= 0.95 x its period; base + head projected cycle (head = 2 position servos with IDs
outside every configured ID) <= 0.8 x the base period; arm + head projected cycle < arm period; fake wire time
alone < half the period on both buses. The test reads the real configs, so it fails until the split lands.
Serves G1, G3, U2, U3, U4.

### D6: Consumers of per-instance topics re-pointed

The bridge publishes `servo_registers` and subscribes `set_register` on its primary namespace (`bridge.py:144,283`).
After the split the base uses `/swerve_drive/servo_registers` and `/swerve_drive/set_register`:
- swerve_controller tilt guard `torque_topic` becomes `/swerve_drive/set_register` in client.yml and as the code
  default (`swerve_drive_controller/tilt_guard.py:94`), so a `torque_off` reaches the drive joints.
- mcp_server gets `topics.base_servo_registers` (default `/swerve_drive/servo_registers`) feeding the same
  `RobotMonitor.on_servo_registers` (`ros_iface.py:681`), listed in `MOTION_CRITICAL_TOPICS` so it is never on demand;
  overheat and servo-error events for the swerve servos keep working. `follower_set_register` (gripper grip
  profiles) stays on the arm.
Alternative: a bridge option to publish the base dump on `/follower/servo_registers` and route set_register by joint
(rejected: two publishers on one arm-namespaced topic hide which bus failed, and needs bridge routing code).
Joint topics are unchanged. Serves G2.

### D7: Head config room on the base bus, either bus possible

The head becomes an `extra_groups: [{namespace: head, joint_names: [...]}]` entry on `base_servos` in the
pan/tilt epic; this epic adds only a client.yml comment marking where, no head joints. `extra_groups` already parses
position joints with `command_min_steps`/`command_max_steps` (`config.py:203-307`). If U4 later moves the head under
the arm, the same entry fits `lerobot_follower` (projection 9.61 ms < 10 ms, D5). The lease (arm or base) is an
mcp_server matter, independent of the bus. Serves G3, U3, U4.

### D8: Ansible wiring

`base_servos` uses the `feetech_servos` type defaults: critical cgroup tier (CPUWeight/IOWeight 1000,
OOMScoreAdjust -900, MemoryLow 56M, MemoryMax 128M, `client.yml:103-118`, `tests/test_unit_cgroups.py:33-36`), no
per-entry override. The playbook step is renamed (D1). `monitoring_agent_node_targets` gains `{node: base_servos, address: "127.0.0.1:19115",
path: /metrics}`; `tests/test_node_metrics_wiring.py` gains the port.
Serves G4.

### D9: Docs and diagnostics

Root README node table, `ansible/README.md`, `nodes/bridges/feetech_servos/README.md` (bus layout and budget),
`nodes/swerve_drive_controller/README.md` (own bus, torque topic), `nodes/mcp_server/README.md` (register feeds),
`scripts/swerve_diag.sh` (expects `ros2-base_servos` active), `scripts/README.md`,
`.claude/skills/ansible-deploy/SKILL.md`, `.claude/skills/teleop-diagnostics/SKILL.md` (arm alone on its bus, 98 Hz
expectation, base bus separate), `tests/README.md`, MEMORY.md decision entry. Serves G4.

### D10: Staged robot verification, software before rewiring

Stage 0: user plugs the second adapter; device paths identified read-only. Stage 1: one deploy of
`lerobot_follower base_servos swerve_controller mcp_server rplidar_a1 monitoring` while the base chain is
still on the arm adapter: the follower serves the arm alone, the base bridge waits for IDs 32-39 on the empty bus
(no placeholder data, the base is idle, swerve_controller's joint-state timeout keeps it from driving). Stage 2:
power off, the user moves the base servo chain to the second adapter, power on. Stage 3: verify rates (Grafana
`servo_cycle_duration_seconds_count` rate per node and the topic_scraper workflow), arm teleop gripper first, base
3 cm forward at 0.03 m/s (steering stays centred), then a 10 cm move. Rollback after stage 2 needs the rewire undone
too. Serves G1, G4, U1, U2, U3.

## Research

| Lens | Ran or skipped | Adopted ideas | Rejected ideas |
|---|---|---|---|
| discovery (codebase brief) | ran: `pantilt-brief.md` (scratch, section 1 findings copied here) plus this planner's reads of `nodes/bridges/feetech_servos/{config,bridge,set_register,metrics}.py`, `tests/{sts_bus,test_bus_io}.py`, client.yml, ros2_node_deploy, mcp_server ros_iface/config, swerve tilt guard | fake bus byte counters (`tests/sts_bus.py:FakeStsPort.wire_time_s`); one process per bus; option B of the brief (separate instance, its own metrics port, battery publisher off) | option A of the brief (head as a third group on the shared bus: breaks the 95 Hz gate) |
| git history | ran: `git log -S swerve_drive_servos`; `03b43eb5` diff shows the removed entry (namespace swerve_drive, own device, IDs then 1-8) | entry shape: feetech instance, namespace swerve_drive, own device (D1) | the old name (the bus will carry the head, D1); old IDs 1-8 and `/dev/ttyUSB1` (servos are 32-39 now; ttyUSB names are unstable) |
| probe | ran: fake-bus cost of five layouts with the real st3215 packet handler (scratch `probe/bus_probe.py`) | numbers in D5, 50 Hz base rate (D4) | base bus at 100 Hz |
| research-code-internal | skipped as a separate run: the reuse is the existing bridge itself, found by direct reading (rows below) | reuse rows below | - |
| research-docs | skipped: no new library or API; st3215, rclpy and Ansible `stat`/`assert` are used as they already are in the repo | - | - |
| research-code-external | skipped: no design fork with external analogues (one more instance of an in-repo node) | - | - |
| research-papers | skipped: no algorithmic core | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief 1 | one process per bus; follower serves arm + `extra_groups` swerve on one port | split-bus-group-vars |
| brief 1 | `servo_registers`/`set_register` stay on the primary namespace | tilt-guard-torque-topic, mcp-base-servo-registers |
| brief 1 | velocity watchdog covers velocity joints; startup torque holds position | split-bus-group-vars (config carried verbatim), final-suite-and-runbook (checked on the robot) |
| brief 1 | bus budget risk: 95.7 Hz vs 95 Hz gate, +126 B dropped to 93.4 Hz | bus-budget-test (anchor), final-suite-and-runbook (gate) |
| brief 1 | fakes `tests/sts_bus.py`, `tests/ros_stubs.py` | bus-budget-test, bridge-instance-identity |
| brief option B | own metrics_port, battery publisher off | bridge-instance-identity, split-bus-group-vars |
| brief option A | head as a third group on the shared bus | rejected: bus gate (D5 numbers) |
| brief 2-3 | TF, URDF, agent and web UI head work | rejected: pan/tilt epic (non-goal) |
| brief 1 speed limit gap | per-joint goal_speed for position joints | rejected: head behaviour, pan/tilt epic |
| reuse | `feetech_servos.config.load_config` / `parse_extra_groups` (`nodes/bridges/feetech_servos/config.py`) | bus-budget-test, split-bus-group-vars |
| reuse | `FakeStsPort`, `make_bus`, `register_table` (`nodes/bridges/feetech_servos/tests/sts_bus.py`) | bus-budget-test |
| reuse | `BridgeCycle` worst-case cycle pattern (`tests/test_bus_io.py::test_follower_cycle_bus_time_keeps_the_control_rate`) | bus-budget-test |
| reuse | `run_bridge` stub harness (`tests/test_shutdown.py::bridge_module`) | bridge-instance-identity |
| reuse | playbook step `Deploy swerve_drive_servos` (`ansible/playbooks/deploy_nodes_client.yml:336-343`) | split-bus-group-vars (renamed to `Deploy base_servos`) |
| reuse | feetech schema assert (`ansible/roles/ros2_node_deploy/tasks/main.yml:41`) | serial-device-preflight |
| reuse | topic_scraper workflow (`scripts/topic_scraper_collect.py`), `scripts/swerve_diag.sh` rates | final-suite-and-runbook |
| finding | rplidar default `/dev/ttyUSB0` | serial-device-preflight |
| finding | tilt guard `torque_topic` `/follower/set_register` | tilt-guard-torque-topic, split-bus-group-vars |
| finding | mcp_server `servo_registers` covers "all servos incl. the swerve ones" (`config.py:126`) | mcp-base-servo-registers |
| finding | `NODE_NAME = "lerobot_follower"` and rclpy `Node("feetech_servos_bridge")` hard-coded | bridge-instance-identity |
| finding | metrics ports 19113 (head camera plan), 19114 (retired Jetson bridge test) | split-bus-group-vars (19115) |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Second feetech instance base_servos | split-bus-group-vars, docs-and-diagnostics |
| D2 by-id addressing, preflight, lidar pinned | serial-device-preflight, split-bus-group-vars |
| D3 Instance identity, battery off, port 19115 | bridge-instance-identity, split-bus-group-vars |
| D4 Base 50 Hz, arm 100 Hz | split-bus-group-vars, bus-budget-test |
| D5 Bus budget test anchored on the measurement | bus-budget-test |
| D6 Consumers re-pointed | tilt-guard-torque-topic, mcp-base-servo-registers, split-bus-group-vars |
| D7 Head config room | split-bus-group-vars, bus-budget-test |
| D8 Ansible wiring | split-bus-group-vars |
| D9 Docs and diagnostics | docs-and-diagnostics, bus-budget-test, tilt-guard-torque-topic, mcp-base-servo-registers |
| D10 Staged robot verification | final-suite-and-runbook |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `BridgeConfig.node_name` | bridge-instance-identity | split-bus-group-vars | `nodes/bridges/feetech_servos/config.py` |
| `TiltGuardConfig.torque_topic default` | tilt-guard-torque-topic | split-bus-group-vars | `nodes/swerve_drive_controller/swerve_drive_controller/tilt_guard.py` |
| `topics.base_servo_registers` | mcp-base-servo-registers | split-bus-group-vars | `nodes/mcp_server/mcp_server/config.py` |
| `base_servos entry` | split-bus-group-vars | serial-device-preflight, bus-budget-test, docs-and-diagnostics | `ansible/group_vars/client.yml` |

## Review focus

- client.yml joint entries of `base_servos`: IDs, `mode: velocity` and `inverted: true` exactly as before
  (left wheels 32 and 39); a wrong inversion drives a wheel backwards.
- Nothing on the client still sends base traffic to `/follower/*` (tilt guard torque, register dump readers).
- The preflight runs before any restart and only for enabled feetech nodes with a `device`.
- The bus budget test reads the real client.yml configs, not copies, and its anchor constants cite the measurement.
- Battery: exactly one `/battery_state` publisher; base bridge creates none.

## Left to the builder

- Test names and helper layout inside the new test files, within the ACs.
- Wording of the client.yml comments and README sections.
- Whether the preflight uses `ansible.builtin.stat` + `assert` or `fail` with a `find` listing, as long as the
  failure message lists `/dev/serial/by-id/` and the play fails before restarts.

## Assumptions and unknowns

- A1: the second adapter's `/dev/serial/by-id/` path. Value: `PENDING` until confirmed. Confirmed before
  plan-execute starts by the user's answer or by runbook stage 0 (read-only `ls -l /dev/serial/by-id/`); the
  orchestrator writes it here, split-bus-group-vars reads it from here.
- A2: the RPLidar's `/dev/serial/by-id/` path, confirmed the same way as A1; serial-device-preflight reads it here.
- A3: the second adapter has a unique USB serial like the two in use (if it has none, by-id collides and a by-path
  or udev rule is needed: stage 0 shows it; the orchestrator asks the user then).
- A4: the 95.7 Hz anchor was measured mostly with idle wheels; the projection treats it as idle. Confirmed on the
  robot by the stage 3 rates.
- A5: the base servos and the arm share one battery pack, so the follower's 6-servo battery reading covers the
  robot. Confirmed in stage 3 (`/battery_state` voltage unchanged versus stage 0).

## Irreversible operations in scope

- None in software: every change is a config or code commit, revertible with a redeploy.
- The physical rewiring (stage 2) is done by the user with the servos powered off; rolling back needs the rewire
  undone before redeploying the old config (runbook states it).
