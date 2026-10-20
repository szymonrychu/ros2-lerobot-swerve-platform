# Servo torque off and neutral: robot runbook

Robot-only steps: executed post-merge by the orchestrator with the user, not by the implementer.

Every deploy goes through the `ansible-deploy` skill and `scripts/deploy-nodes.sh`, logged to `.logs/`. Least dramatic joint first (CLAUDE.md). Every step lists its pass check as an observable value. On any mismatch: STOP and report, leave the robot as it is (released joints supported), and do not continue. Stop and ask before any step beyond the one described. If the robot is offline, report steps 1-7 as pending on the robot.

## Preconditions

1. claude_agent state API shows no running session (deploying mcp_server kills it). Pass: state API reports no active session. Mismatch: STOP and report.
2. Robot on blocks or wheels braked. Pass: user confirms.
3. topic_scraper_api enabled (`enabled: true` in `ansible/group_vars/client.yml`, deployed) and answering. Pass: `curl -s -o /dev/null -w '%{http_code}' http://client.ros2.lan:18100/rules` prints 200 (was it disabled before? note it for step 7). Mismatch: STOP and report.

## Deploy

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client lerobot_follower base_servos filter_node swerve_controller mcp_server web_ui 2>&1 | tee .logs/deploy-client-servo-neutral.log
```

Pass:
- exit code 0.
- `/follower/torque_state`, `/swerve_drive/torque_state`, `/head/torque_state` each publish `released: false` with every listed joint at 1.
- `/follower/joint_states` >= 95 Hz and `/swerve_drive/joint_states` about 50 Hz, measured with `scripts/topic_scraper_collect.py` (header_stamp_ns deltas).

Mismatch: STOP and report.

## Head pan single-joint bench

Before any group save. Set `neutral_exclude_joints: [head_tilt]` on the head group in `ansible/group_vars/client.yml` and deploy `base_servos`. Pass: `/head/torque_state` shows `neutral_enabled: true`.

The user supports the head.

1. Torque off head. Pass: `/head/torque_state` `released: true`, `head_pan` 0 and `head_tilt` 0; a Head tab command is refused (UI banner shown, backend rejects the WS publish); mcp `set_head_angles` answers `head servo torque is off`; `/head/joint_states` keeps publishing.
2. Turn pan by hand about 5 deg; Re-enable torque (no save). Pass: measured pan stays within 1 deg of the released pose for 5 s.
3. Torque off, turn pan about 5 deg, Record neutral. Pass: the reported shift is 40-75 steps.
4. Re-enable torque + save. Pass: response ok; `/head/joint_states` `head_pan` within 0.005 rad of 0; `/swerve_drive/servo_registers` `head_pan` `min_angle_limit` / `max_angle_limit` equal the pre-save numbers; `torque_enable` 1.

Any mismatch: STOP, leave the head released and supported, report.

5. Write the three answers into `## Findings` below and into the feetech README `## Bench findings`: goal write with torque off, DefineMiddle unlock need, angle limits after the offset.
6. Restore: release, turn pan back to straight, record, save (same pass checks as step 4).
7. Remove the temporary `neutral_exclude_joints` and redeploy `base_servos`. Pass: `/head/torque_state` `released: false`, joints at 1.

## Head pan

Head pan as a group save (head group without exclusions): Torque off head, turn pan about 3 deg, Record neutral, Re-enable torque + save. Pass: response ok; `head_pan` within 0.005 rad of 0 in `/head/joint_states`; limits in `/swerve_drive/servo_registers` equal the pre-save numbers; torque 1. Mismatch: STOP and report.

## Head tilt

Same flow for `head_tilt` (about 3 deg; the servo is inverted, so check the sign of the reported shift against the sign of the move). Pass: same checks for `head_tilt`: response ok, within 0.005 rad of 0, limits equal pre-save numbers, torque 1; `head_pan` still within 0.005 rad of 0. Mismatch: STOP and report.

## Steering

1. Torque off base. Pass: `/swerve_drive/torque_state` `released: true` with the 4 steer joints at 0; the drive wheels report 0 rad/s and torque on (1); `/swerve_drive/joint_commands` carries no finite steering position.
2. Re-enable holds. Pass: each steer within 1 deg of its released pose for 5 s.
3. Torque off, wheels set straight, Record neutral, Re-enable torque + save. Pass: response ok; each steer within 0.005 rad of 0; the idle recentre does not move a wheel more than 1 deg.

Mismatch: STOP and report.

## Arm

Start when the user gives the go and is supporting the folded arm.

1. Torque off arm. Pass: filter_node active source `released`; mcp `get_control_state` lists `arm` in `servos_released`.
2. Re-enable holds. Pass: each joint within 1 deg of its released pose for 5 s.
3. Only on a second go from the user: pose, Record neutral, Re-enable torque + save. Pass: mcp `get_arm_state` `calibration_stale` = [joint_offsets, home]; `arm_home` answers the stale-home refusal; the arm card shows the leader warning; the leader is not accepted until within 0.15 rad.

Mismatch: STOP and report.

## Findings

Filled in during the head pan single-joint bench (also copied to `nodes/bridges/feetech_servos/README.md` `## Bench findings`).

- Goal write with torque off: result to be written here from the bench
- DefineMiddle unlock need: result to be written here from the bench
- Angle limits after the offset: result to be written here from the bench

Final step: disable topic_scraper_api again (`enabled: false`, redeploy) if it was disabled before step 1.
