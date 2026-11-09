# Servo torque off and neutral: robot runbook

Robot-only steps: executed post-merge by the orchestrator with the user, not by the implementer.

Every deploy goes through the `ansible-deploy` skill and `scripts/deploy-nodes.sh`, logged to `.logs/`. Least dramatic joint first (CLAUDE.md). Every step lists its pass check as an observable value. On any mismatch: STOP and report, leave the robot as it is (released joints supported), and do not continue. Stop and ask before any step beyond the one described. If the robot is offline, report steps 1-7 as pending on the robot.

Where on the page: web UI, hamburger menu, Calibrate, tab **Servo home** (deep link `#/calibrate/servo-home`). Its
steps: 1 Record and save neutral (one card per group: Torque off, Record neutral, Re-enable torque + save with the
large-shift guard, Re-enable torque; optional, only after a servo swap or a moved horn), 2 Arm zero restore (optional),
3 Move to neutral pose, 4 Check and accept (the quality gate: every group fresh with torque on and nothing stale;
Accept records the check on the page only, nothing is written). The General section on top has Home arm, Home camera
and Home steering servos for the poses between the checks below.

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

## Arm zero restore

Repairs a follower arm re-zero recorded in the wrong pose (2026-11-09: `neutral recorded` logged shoulder_lift about
-901 steps and elbow_flex about +1029 steps) without moving the arm. The follower bridge rewrites each arm servo's EEPROM
offset so the arm, resting folded on its rest with torque off, reads its rest reference again. The gripper is never
touched. The user runs it alone on the web UI Calibrate page, tab **Servo home** (deep link `#/calibrate/servo-home`),
step 2 "Arm zero restore", panel "Arm zero restore". Details:
`nodes/bridges/feetech_servos/README.md` "Arm zero restore" and `nodes/web_ui/README.md`.

Needs `lerobot_follower`, `mcp_server` and `web_ui` deployed from this branch (`scripts/deploy-nodes.sh client
lerobot_follower mcp_server web_ui`, through the `ansible-deploy` skill). Reference: until a rest reference is saved
the bridge uses the seed in `ansible/group_vars/client.yml` (`zero_restore.seed_reference_rad`, the readings at rest
before the incident): shoulder_pan -0.018, shoulder_lift -1.542, elbow_flex 1.516, wrist_flex 0.862, wrist_roll -0.072
rad. The panel shows "Reference: seed from the bridge config".

Steps (one button each; the panel says what to expect under every button):

1. Rest the arm folded on its rest, jaws empty. Press **1. Torque off arm** and confirm. Expect: the arm goes limp,
   the card shows torque off for every joint.
2. Keep the arm still and press **2. Dry run**. Expect: a success message and the plan table. Today's expected values
   (Now within about 0.03 rad, Change within about 2 deg; the rest pose varies a little):

   | Joint | Now (rad) | Target (rad) | Change (deg) | Guard |
   |---|---|---|---|---|
   | shoulder_pan | about -0.002 | -0.018 | about -1.0 | ok |
   | shoulder_lift | about -0.160 | -1.542 | about -79.2 | large |
   | elbow_flex | about -0.063 | 1.516 | about +90.4 | large |
   | wrist_flex | about 0.934 | 0.862 | about -4.1 | ok |
   | wrist_roll | about -0.094 | -0.072 | about +1.2 | ok |

   Mismatch (a refusal, a Change far from these, or a "large" guard on another joint): STOP, press **4. Re-enable
   torque**, report. Nothing was written. Refusals: "release torque first", "torque not off", "arm not still" (hold
   still and run it again), an invalid rest reference.
3. Press **3. Apply**. Because shoulder_lift and elbow_flex change by more than 15 deg, the dialog "Apply a large zero
   change?" lists them; tick "The arm rests folded on its rest and the large change is intended" and press **Apply
   large change**. Apply within 60 s of the dry run without touching the arm, else it is refused ("dry run is N s old" or
   "moved since the dry run": run the dry run again). Expect: "zero restored for shoulder_pan, shoulder_lift,
   elbow_flex, wrist_flex, wrist_roll; torque stays off" and the result table: every joint Applied yes, Now (rad)
   equal to the Target of step 2 within 0.005 rad. The arm does not move. The calibration-stale warning
   disappears (joint_offsets cleared; home cleared too when it was listed, or listed now when home was re-recorded
   after the wrong re-zero: then store a new home from the Arm tab after Park). Mismatch (a failed joint): STOP and report the message; the failed joint got its
   old offset back, joints before it are restored, later joints untouched; torque stays off.
4. Press **4. Re-enable torque**. Expect: the arm holds its pose, the card shows torque on.
5. Park the arm: General section **Home arm** (or the Arm to camera flow's Park when it holds the board). Expect: it
   parks on its rest without a jump. Mismatch: press **Abort calibration** (it holds the arm) and report.

Optional, once the zero is right: with the arm resting folded and torque off, **Save current pose as rest reference**
(confirm **Save reference**) stores today's rest readings in `/var/lib/ros2/arm/rest_reference.json`; later restores
use it instead of the seed ("Reference: saved <time>"). It is disabled while the zero is stale (joint_offsets).

**Move to neutral pose** (Servo home flow, step 3, confirm **Move**: "The arm moves to the neutral pose: all joints at 0.
Keep the area clear.") moves every arm joint except the gripper to 0 rad along the home routing (`/arm/home`) with its body checks and
speed;
it needs torque on and a zero that is not stale.

Audit and logs: every apply writes `/var/lib/ros2/arm/zero_restore-<UTC time>.json` (per joint offset before/after,
reading before/after, limits). Every step logs `calib_event {...}` in the journal (`journalctl -u
ros2-lerobot_follower | grep calib_event`; Move to neutral in `ros2-mcp_server`).

## Large-shift guard

Re-enable torque + save is refused by the bridge (`save_neutral`, nothing written, torque stays off, the capture is
kept) when any recorded joint shift is above `neutral_max_shift_deg` (bridge config, default 15 deg; set explicitly to
15.0 for `lerobot_follower` in `ansible/group_vars/client.yml`, `base_servos` keeps the default for steering and head).
The Calibrate page shows each recorded shift on the card (a large one in red with `(above 15 deg)`) and in the save
confirm, which then warns, names the joints and needs the checkbox "The large shift is intended: save it" before
**Save large shift** calls `save_neutral_confirmed`. The bridge journal logs the refusal and every confirmed large shift
at WARNING. A large shift almost always means the pose was wrong at Record neutral (incident 2026-11-09: recorded in the
folded pose, elbow_flex +1030 steps / +90.5 deg, shoulder_lift -933 steps; Park could not converge afterwards): cancel,
put the joints in the neutral pose, record again.

Bench check (after deploying `lerobot_follower`, `base_servos` and `web_ui`; head pan as the least dramatic joint):
Torque off head, turn pan by hand about 20 deg, Record neutral. Pass: the card shows `head_pan` in red with
`(above 15 deg)`; the save confirm warns and **Save large shift** stays disabled until the checkbox is ticked. Cancel,
turn pan back to straight, Record neutral. Pass: the save confirm shows no warning; save as in "Head pan". Mismatch:
STOP and report.

## Findings

Filled in during the head pan single-joint bench (also copied to `nodes/bridges/feetech_servos/README.md` `## Bench findings`).

- Goal write with torque off: result to be written here from the bench
- DefineMiddle unlock need: result to be written here from the bench
- Angle limits after the offset: result to be written here from the bench

Final step: disable topic_scraper_api again (`enabled: false`, redeploy) if it was disabled before step 1.
