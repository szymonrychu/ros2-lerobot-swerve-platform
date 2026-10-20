# Servo torque off and neutral recording in the web UI design

## Summary

Web UI controls (Calibrate page) to release servo torque per group (arm = follower arm, base = the 4 steering servos,
head = pan/tilt), record the present physical pose as the new neutral, and re-enable torque holding the present
position, optionally saving the recorded neutral into the servo EEPROM first. Everything that touches the bus runs
inside the running feetech bridge (the sole bus owner) through std_srvs/Trigger services; the bridge publishes a live
per-group torque state topic. While a group is released every command source for it is quieted (bridge gate,
set_register, filter_node, swerve_controller, mcp_server, web_ui), so no goal reaches a limp servo and re-enable cannot
jump. An arm re-zero writes a stale marker that flags the fitted arm joint offsets and the stored home until they are
re-calibrated.

## Intent and why

Re-zeroing a servo today needs SSH, a stopped node and `sudo nodes/bridges/feetech_servos/scripts/calibrate_servos.py center`. After the head mount
inversion (2026-10-20) and before the ChArUco arm calibration (epic head-and-arm-calibration-fro, blocked by this one)
the user needs to put joints at a physical zero by hand and store it, from the browser, without stopping nodes.

## Goals

- Three torque-off buttons (arm, base, head) with a confirm dialog; the released servos go limp, measured joint_states
  keep publishing.
- Per group: "Record neutral" captures the present pose and previews the shift; "Re-enable torque + save" writes the
  neutral into each servo's EEPROM (DefineMiddle inside the bridge), verifies it reads ~2048, keeps the angle limits,
  refreshes the bridge command limits, then enables torque holding the present position; "Re-enable torque" enables
  without saving.
- Re-enable never moves a joint: goal_position = present before torque_enable = 1, both bypassing the write cache.
- While released: the bridge drops every goal for the group, set_register cannot re-enable or move it,
  filter_node/swerve_controller/mcp_server/web_ui do not command it.
- Arm re-zero marks arm joint offsets and home stale (marker file); mcp_server reports it and refuses arm_home until a
  new home is recorded.
- Live torque state per group on `/{group}/torque_state`.
- The set_register `torque_enable=128` hole is closed.
- Ansible, READMEs, tests/README.md in sync; staged robot runbook with head_pan first.

## Non-goals

- Drive wheel servos: wheel mode, no neutral (D-C1).
- Leader arm re-zero (D-C2; the UI warns, follow-up).
- Software joint offsets (no offset in command_mapping; the neutral lives in the servo).
- Re-fitting arm offsets, tool offset, sag or the gripper camera mount after a re-zero (epic head-and-arm-calibration-fro).

## Anti-goals

- A joint moving on re-enable, or a goal written into a released servo.
- Any placeholder torque state (unknown is omitted, never defaulted).
- A partially centred group silently re-enabled as if all were saved.
- Stalling the base drive while another group saves: wheels are stopped first.

## Constraints

- CLAUDE.md: test-first, type hints, constants at top, ROS2 logging, no placeholder data on topics, Ansible in sync
  via `ros2_nodes` schema, READMEs and tests/README.md updated, deploy only with `scripts/deploy-nodes.sh`.
- Servo safety: bench on head_pan first, least dramatic joints first; ask the user before releasing the arm.
- web_ui ROS-thread affinity: every create/destroy on the bridge timer (apply_demand), publishes FIFO.
- The bridge executor is single-threaded and drained inside the control loop (`drain_callbacks`), so service callbacks
  run on the bus-owner thread; nothing may touch the bus from another thread.
- Tasks at most 8 files, about an hour each.

## Decisions

### D1: Torque release and enable live in the feetech bridge

Release: the group's position joints are gated in `BridgeCycle` first (pending targets dropped, new commands ignored,
no goal refresh), then torque_enable = 0 is written bypassing the `last_written` cache and verified by readback
(`set_startup_torque_state`). Enable: present position read (retry on None), goal_position = present written bypassing
the cache, torque_enable = 1 written bypassing the cache and verified, then the gate lifts; a joint that fails stays
released and is named in the response. Alternatives: set_register from web_ui (rejected: no hold, cache no-op,
sources keep writing, brief facts 12-14); a separate calibration process (rejected: the bridge is the sole bus owner).
Serves A1, A3.

### D2: Live per-group torque state topic

`/{group}/torque_state` (std_msgs/String JSON, reliable, transient_local, depth 1), published on every change and every
`torque_state_interval_s` (default 1.0 s). Per joint the value is the latest verified readback or register-dump
`torque_enable` reading; a joint with no reading yet is omitted. Alternative: the 10 s register dump only (rejected:
too slow, one servo per cycle). Serves A1, A5.

### D3: Neutral is the servo EEPROM centre, executed inside the bridge, in two steps

"Record neutral" (`/{group}/servo/record_neutral`) reads and keeps the present steps of the group's neutral joints and
publishes the capture (delta to 2048) - no write. "Re-enable torque + save" (`/{group}/servo/save_neutral`) checks every
joint is still within 20 steps of its capture, stops the bus's wheels, then per joint: read min/max_angle_limit,
UnLockEprom, DefineMiddle, LockEprom (finally), verify present within 2048 +- 3 (10 attempts, None retried), rewrite the
pre-centre limit numbers if they changed, refresh `command_limits` for joints whose limits come from EEPROM, then runs
D1 enable. Fail-safe for the hardware unknowns (DefineMiddle unlock need, limits vs offset, goal writes re-enabling
torque): a failed DefineMiddle, post-DefineMiddle verify (2048 +- 3) or limit restore aborts the remaining joints of that
save, no limit is written on an unverified servo, torque stays OFF on the whole group (released, gated), and the
response names the joints whose centre changed, the failed joint and the unattempted ones; success is false.
User D-C1 (EEPROM, inside the bridge); the two-step split and the limit policy are (my call), decided as Q2 (the user
may override at the gate).
Serves A2, A3.

### D4: Groups

arm = `/follower` group of lerobot_follower (torque: all 6 joints; neutral: the 5 arm joints, gripper excluded by
`neutral_exclude_joints`, (my call), decided as Q1); base = `/swerve_drive` group of base_servos (torque and neutral: the
4 steering servos 33/34/37/38, velocity joints are never released; while base is released the bridge stops the wheels
and drops drive commands, (my call), decided as Q3); the user may override both at the gate; head = `/head` group of base_servos. Neutral services exist only for groups
with `neutral_enabled: true`; the leader bridge (server.yml) has none (D-C2). Serves A1, A2, A4.

### D5: set_register closed for torque and goals

`torque_enable` accepts only 0 or 1 (128 = DefineMiddle is rejected as `bad_value`); every value is range-checked by
register size; for a released joint the registers torque_enable, goal_position, goal_speed, goal_time and acc are
refused (reason `released`). Serves A5.

### D6: Command sources quieted while a group is released

filter_node: while `/follower/torque_state` is released it publishes nothing on `/follower/joint_commands`, ends an
autonomy lease, and after re-enable the leader resumes only via the proximity rule (`proximity_required`).
swerve_controller: while `/swerve_drive/torque_state` is released it ignores cmd_vel, publishes no finite steering and
zero drive; on re-enable it seeds steering from measured positions and restarts the idle-recentre timer. mcp_server: a
torque-off gate in `call_tool_with_digest` refuses arm, base and head motion tools of a released group; arm release
drops the lease without publishing. web_ui: WS publishes to a released group's command topics and the arm action
routes answer an error. Unknown or stale torque state refuses nothing (the bridge gate is the backstop). The user
delegated the quieting ((my call) in userDecisions). Serves A5.

### D7: Arm re-zero stale marker

After a save on a group with `neutral_stale_marker`, the bridge writes (atomically) the marker JSON
`{"version": 1, "group", "saved_at", "joints": {name: {"before_steps", "after_steps"}}, "stale": ["joint_offsets", "home"]}`
at `/var/lib/ros2/arm/calibration_stale.json`; a later save re-adds both items. mcp_server reports it in
`get_arm_state`, refuses `arm_home` while `home` is stale and removes `home` on a successful `arm_set_home`; the
calibration epic removes `joint_offsets`; an empty list deletes the file. web_ui shows it on the arm card. User D-C1.
Serves A3.

### D8: Web UI Calibrate page section and released banners

A "Servo torque and neutral" section with one card per group: torque chip, per-joint torque, buttons Torque off
(confirm: support the arm/head; base: steering goes limp, wheels stay braked), Record neutral (shows per-joint shift in
degrees), Re-enable torque + save (confirm: writes the servo EEPROM; arm adds offsets/home stale and "teleop needs the
leader re-zeroed in the same pose"), Re-enable torque. Head, Arm and Map tabs show a banner and disable their motion
controls while their group is released. Save is refused under battery cut-off; release and enable never are (my call).
User D-C2 for the leader warning. Serves A1, A2, A4.

### D9: Bus time during save

Service callbacks run inline on the loop thread; a save takes about 0.6 s per joint (about 3 s for the 5 arm joints)
during which that bridge's loop stalls. Wheel stop is per bus: before the first EEPROM write the save commands 0 to
every velocity joint of the bus being saved, so those wheels are stopped before the stall even though that bridge's
velocity watchdog cannot run during it; a save stalls only its own bus, and wheels on another bus keep their own
bridge and watchdog. The stall is acceptable because the save runs while the group is still released (torque off,
supported by the user), before the re-enable; joint_states of that bus pause for the stall. One torque/neutral
operation runs at a time per bridge. (my call) Serves A2.

### D10: Bench order and safety

The runbook (robot-only steps executed post-merge by the orchestrator with the user) starts with a single-joint save on head_pan alone (head_tilt temporarily in `neutral_exclude_joints`), stopping on any mismatch, before any group save; then head_pan and head_tilt as a group, then the steering group (robot on
blocks or wheels braked), the arm only after the user confirms they support it. Serves A6.

### D11: Gripper excluded from the arm re-zero
2026-10-20 gate: the gripper is excluded from the arm re-zero (neutral_exclude_joints: [gripper]); arm torque off still releases it (user decision)

### D12: Base release is steering only, wheels braked
2026-10-20 gate: base release = the 4 steering servos; the drive wheels stay torqued and braked (user decision)

### D13: Two-step record then save
2026-10-20 gate: two steps as designed: Record neutral captures and previews; Re-enable torque + save writes the servo EEPROM; plain Re-enable torque without saving (user decision)

## Research

| Lens | Ran | Adopted | Rejected |
|---|---|---|---|
| code-internal (brief torque-neutral.md) | ran | bridge-side sequence (brief facts 12-17), cmd_center copy (`nodes/bridges/feetech_servos/scripts/calibrate_servos.py:297-350`), Trigger route pattern (`web_ui/server.py:1055-1087`), Calibrate page modules (`web_ui/imu_calibration.py`), filter proximity rule (`filter_node/arbitration.py`) | set_register-based torque from web_ui (brief fact 11-13), software offsets (`command_mapping.py:12,49`) |
| external | skipped: STS3215 behaviour is bench-tested, not documented reliably | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief fact 11 | set_register accepts torque_enable=128 without unlock | set-register-torque-guard |
| brief fact 12 | write_register cache makes torque 1 a no-op | bridge-torque-release-enable |
| brief fact 13 | hold-before-torque-on only at startup | bridge-torque-release-enable |
| brief fact 14 | sources keep writing goals while torque off | bridge-torque-release-enable, filter-node-released-arm, swerve-released-steering, mcp-torque-off-gate, web-ui-quiet-released-groups |
| brief fact 15 | no live torque state | bridge-torque-services-topic |
| brief fact 16 | read failures skip samples | bridge-torque-services-topic (omit unknown joints) |
| brief fact 17 | centre inside the loop, read_register None retry | bridge-runtime-neutral-save |
| brief fact 18 | command_limits stale after centre | bridge-runtime-neutral-save |
| brief fact 19 | follower re-zero stales offsets, home, teleop | bridge-runtime-neutral-save, mcp-arm-calibration-stale, calibrate-page-servo-section |
| brief fact 20 | steering neutral used by recentre and odometry | swerve-released-steering, ansible-servo-neutral-config |
| brief fact 21 | head steps pinned by tests/test_head_servo_config.py | ansible-servo-neutral-config (steps unchanged) |
| brief fact 22 | web_ui patterns and ROS-thread affinity | web-ui-servo-routes, web-ui-quiet-released-groups, calibrate-page-servo-section, released-group-banners |
| brief fact 23 | bench unknowns | suite-and-robot-runbook |
| brief fact 24 natural tasks a-f | task split | every task |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Torque in the bridge | bridge-torque-release-enable, bridge-torque-services-topic |
| D2 Torque state topic | bridge-torque-services-topic, bridge-runtime-neutral-save |
| D3 EEPROM neutral in two steps | bridge-runtime-neutral-save, web-ui-servo-routes |
| D4 Groups | bridge-runtime-neutral-save, ansible-servo-neutral-config |
| D5 set_register closed | set-register-torque-guard |
| D6 Sources quieted | filter-node-released-arm, swerve-released-steering, mcp-torque-off-gate, web-ui-quiet-released-groups |
| D7 Stale marker | bridge-runtime-neutral-save, mcp-arm-calibration-stale, web-ui-servo-routes |
| D8 Web UI section and banners | calibrate-page-servo-section, released-group-banners |
| D9 Bus time during save | bridge-runtime-neutral-save |
| D10 Bench order | suite-and-robot-runbook |
| D11 Gripper excluded from the arm re-zero | bridge-runtime-neutral-save, ansible-servo-neutral-config, calibrate-page-servo-section |
| D12 Base release is steering only, wheels braked | bridge-torque-release-enable, swerve-released-steering, ansible-servo-neutral-config |
| D13 Two-step record then save | bridge-runtime-neutral-save, web-ui-servo-routes, calibrate-page-servo-section |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `torque-gate` | bridge-torque-release-enable | bridge-torque-services-topic, set-register-torque-guard, bridge-runtime-neutral-save | `nodes/bridges/feetech_servos/torque_control.py` |
| `torque-state-topic` | bridge-torque-services-topic | bridge-runtime-neutral-save, filter-node-released-arm, swerve-released-steering, mcp-torque-off-gate, web-ui-servo-routes | `nodes/bridges/feetech_servos/torque_control.py` |
| `neutral-services` | bridge-runtime-neutral-save | web-ui-servo-routes, ansible-servo-neutral-config | `nodes/bridges/feetech_servos/neutral.py` |
| `arm-stale-marker` | bridge-runtime-neutral-save | mcp-arm-calibration-stale, web-ui-servo-routes, ansible-servo-neutral-config | `nodes/bridges/feetech_servos/neutral.py` |
| `servo-calibration-api` | web-ui-servo-routes | web-ui-quiet-released-groups, calibrate-page-servo-section | `nodes/web_ui/web_ui/servo_calibration.py` |
| `servo-torque-hook` | calibrate-page-servo-section | released-group-banners | `nodes/web_ui/frontend/src/calibrate/servoApi.ts` |

Cross-epic (docs/plans/contracts.json): `arm-calibration-stale-marker`, `servo-torque-state-topic` and
`calibration-config-servos-flag` are consumed by head-and-arm-calibration-fro.

## Review focus

- `torque_control.py` and the `BridgeCycle` gate: no goal_position write for a released id on any path (pending,
  refresh, startup, set_register); enable order (goal = present before torque 1) and cache bypass.
- `neutral.py`: EEPROM lock in `finally`, verify before limits, partial-failure handling, wheels stopped first.
- filter_node and mcp_server lease transitions on release and re-enable (no stale setpoint republished).
- web_ui: no ROS create/destroy outside apply_demand.

## Left to the builder

- Internal class and helper names inside the named modules, as long as the Interfaces in each task hold.
- How the frontend lays out the cards (MUI, touch targets from `theme.ts`).
- Whether swerve_controller publishes nothing or zero drive with NaN steering while released, as long as no finite
  steering target is published and the wheels are commanded 0 or left to the bridge watchdog.

## Assumptions and unknowns

- Whether a goal_position write with torque off re-enables torque on the STS3215: moot by design (no goal reaches a
  released servo); the runbook observes it if seen.
- Whether DefineMiddle needs the EEPROM unlock: the bridge always unlocks (as `cmd_center`); the D3 fail-safe keeps torque off on any unverified result; the runbook confirms it on head_pan alone first.
- Whether the angle limits move with the offset: the bridge restores the pre-centre numbers if they changed; bench
  records which happened in the feetech README.
- `/var/lib/ros2/arm` exists on the client (`ansible/playbooks/tasks/mcp_server_setup.yml:44`, owner ansible_user); the
  marker writer creates missing parents.

## Open questions
- Q1: Arm re-zero excludes the gripper (torque off still releases it), because its fixed open/close steps and the leader gripper mapping would shift? [answer: approved by the user at the gate 2026-10-20 (D11): gripper excluded]
- Q2: Two steps: Record neutral only captures and previews the shift; the EEPROM is written only by Re-enable torque + save (plus a plain Re-enable torque without saving)? [answer: decided (my call, user may override at the gate): Record neutral only captures and previews; Re-enable torque + save writes the EEPROM; a plain Re-enable torque exists]
- Q3: Base torque off releases only the 4 steering servos; the drive wheels stay torqued and braked (robot cannot be pushed by hand)? [answer: decided (my call, user may override at the gate): base release = the 4 steering servos; drive wheels stay torqued and braked]

## Irreversible operations in scope

- Servo EEPROM writes (DefineMiddle, angle limits): covered by the confirm dialog, the 20-step move check, verify before
  limits, and reversible by recording the old pose again; the runbook asks the user before the arm.
