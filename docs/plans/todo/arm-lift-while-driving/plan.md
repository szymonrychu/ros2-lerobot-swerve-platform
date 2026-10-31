---
title: Arm lift while driving
repos: []
created: 2026-10-31
owner: ""
rung: ""
---

# Arm lift while driving

## What and why

What: while a NavigateToPose goal is active, the SO-101 follower arm moves from its parked home pose to a fixed,
FK-checked "drive pose" that keeps it out of the D435's view, and returns home when navigation ends.

Why, from the user: "Head runs mostly correct, but you've forgot, that most of the right side (normally) is not visible
due to arm home position. In order to fix it, simply lift it during driving." head-aims-along-path (in progress,
nodes/head_aim_cpp) aims the pan/tilt head along the path; its Q3 accepted that the parked arm occludes the view when
the head pans right. The arm is mounted front-right (base_link 0.059, -0.05, 0.10) and the camera front-left (0.15,
0.10, ~0.35), so a right pan looks straight across the parked arm.

Facts this plan relies on (read in the code, no robot access):

- Parked home (scripts/gen_nav_footprint.yaml:63-69, mirrors the stored /var/lib/ros2/arm/home.yaml): shoulder_pan
  -0.003, shoulder_lift -1.35, elbow_flex 1.578, wrist_flex -0.1, wrist_roll -1.637, gripper 0.1. The gripper
  overhangs the crossbar front by about 10 cm and that shadow is in the static footprint (client.yml:3034-3042).
- No named drive / carry pose exists. The only stored pose is home (`ArmController.stored_home`,
  nodes/mcp_server/mcp_server/arm.py:2425); "carry" in mcp_server is the carried-object record (carry.py), not a pose.
- mcp_server owns every safe arm motion: `ArmController.plan_home` (arm.py:2920) finds a collision-free route to any
  target pose through `home_routes` (arm.py:2887, raise shoulder/elbow first, lift the tool straight up, ...), checked
  leg by leg against the body model, which includes the head envelope as a cylinder about the pan axis
  (client.yml:1805-1806 `head_radius_m`, `head_height_m`), so it covers every pan/tilt angle.
- Arbitration: filter_node priority autonomy > web_ui > leader; the autonomy lease is sticky until released; after a
  release the leader resumes only by the proximity rule (never snaps); with follower torque off every source is dropped
  (nodes/filter_node/README.md "Command source arbitration"). mcp_server's /arm/home service = move home, then release
  (arm.py:2381).
- mcp_server already caches `/navigate_to_pose/_action/status` and has `RosRobot.nav_active` (ros_iface.py:2407).
  web_ui goals go through `/goal_pose` (bt_navigator), mcp_server goals through the action; both show in that status.
- Footprint: while no lease holds the arm, mcp_server's footprint tick follows the measured arm and publishes the static
  footprint whenever the arm shadow lies inside it (footprint.py:1-35). laser_filter removes lidar and D435 cloud points
  inside the footprint polygon up to z 1.0 m (nodes/laser_filter/README.md "D435 cloud self filter"), so a lifted arm
  whose shadow stays inside the static outline is neither an obstacle nor a footprint change.

## Scope ladder

Rungs are cumulative: each rung includes everything above it.

### Rung 1: MVP - auto lift and lower on navigation goals (recommended)

- 1.1: a `drive_pose` block in the mcp_server `arm` config (client.yml, pydantic in config.py): `enabled`, `joints`
  (rad, measured space, every arm joint), `lower_delay_s` (default 3.0), `speed_scale` (default 0.3), `tolerance_rad`.
- 1.2: drive pose values chosen with the mcp_server arm FK (see D4) and pinned by a root test
  (tests/test_drive_pose_config.py): pose and every leg of `home_routes` from `parked_arm_joints` clear the body and
  head envelope by `margin_m`, tool and wrist above `min_tool_height_m`, elbow not stretched (elbow_flex >= 0 or
  shoulder_lift <= 1.85), wrist_roll and gripper unchanged from home, the arm's ground shadow on every leg inside the
  static footprint outline, and no arm link capsule inside the D435 frustum for head tilt 30 deg down and pan 0 to -90
  deg (right) in 10 deg steps.
- 1.3: new `DrivePose` controller in mcp_server (nodes/mcp_server/mcp_server/drive_pose.py, rclpy-free), ticked from
  the existing node timer: on the first active goal it lifts when every precondition holds (enabled; arm at home per
  `at_home`; no lease held by agent or web UI; follower torque on; fresh joint states; not carrying; leader not moving;
  battery above cut-off), via `plan_home(drive_pose)` legs at `speed_scale`, then releases keep-position. When no goal
  has been active for `lower_delay_s` (succeeded, aborted, cancelled alike) it returns home (/arm/home path) only if
  the arm is still within `tolerance_rad` of the drive pose and no lease is held; otherwise it leaves the arm and logs.
  A skipped lift or lower never blocks, cancels or delays navigation; the reason is logged once per goal.
- 1.4: `get_control_state` reports `arm.drive_pose` (`lifted`, `lowering`, `home`, `skipped: <reason>`), so the agent
  and web UI see why the arm is where it is; a stop tool call aborts a drive-pose motion like any other.
- 1.5: unit tests for the controller (preconditions, goal chaining within `lower_delay_s`, cancel and abort, torque
  off, bridge down, lease taken mid-drive, arm moved mid-drive), README updates (mcp_server, nodes/README.md
  untouched unless the topic list changes, tests/README.md), MEMORY.md decision line.
- 1.6: deploy mcp_server with `enabled: false`; live check per the servo safety rule: lift and lower once by hand at
  speed_scale 0.1 with the user watching (no driving), confirm the head view while panning right via web_ui Head tab,
  then set `enabled: true`, redeploy and drive one short slow goal.
- Tradeoff: the robot starts moving while the arm still lifts (about 2 s); first metres are seen as today. Size: medium
  (one new mcp_server module, config, one root test, wiring, READMEs).

### Rung 2: operator control

- 2.1: `/arm/set_drive_pose` std_srvs/Trigger (store the measured pose after the same FK checks as 1.2, refusing with
  the failed check) and a web_ui "Set drive pose" button next to Set home; the stored pose (/var/lib/ros2/arm/
  drive_pose.yaml) overrides the config value.
- 2.2: runtime on/off: `/arm/drive_pose/enable` (std_srvs/SetBool) and a web_ui toggle on the MAP and ARM tabs showing
  the `arm.drive_pose` state; claude_agent prompt note that the arm lifts by itself while navigating.
- Tradeoff: touches web_ui frontend and backend and claude_agent; overlaps the in-progress webui-arm-tab-and-3d-goal
  plan (same arm button row). Size: medium.

### Rung 3: driving waits for the lift

- 3.1: mcp_server's own navigation (`RosRobot.navigate`, the approach and frontier tools) lifts synchronously before it
  sends the goal.
- 3.2: every goal (web_ui `/goal_pose` too): a custom Nav2 BT XML copying the current default
  (navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml) with a leading condition that waits up to N s
  for `/arm/drive_pose/state` = lifted or skipped (a small C++ BT condition plugin built in the existing nav2 overlay).
- Tradeoff: a BT plugin and overlay build on the Pi; every goal starts up to N s later. Size: large.

### Rung 4: complete - drive footprint without the overhang

- 4.1: while the arm is measured at the drive pose, mcp_server publishes a "drive" footprint (body plus the drive pose
  shadow, generated by gen_nav_footprint.py next to the static one) instead of the static one with the parked-gripper
  overhang, so Nav2 plans through tighter gaps; it falls back to static the moment the arm leaves the drive pose.
  head_aim_cpp footprint_max_x follows (config or the generated value).
- Tradeoff: a footprint change re-inflates both costmaps twice per goal and changes the collision_monitor StopBox
  per mode; a wrong state means a too-small footprint. Size: large.

## Agreed scope

not agreed yet

## Non-goals

- No change to the stored home pose, home routes or the static footprint (rungs 1-3).
- No new ROS node: the lift lives in mcp_server, which already owns the arm model, leases and footprint.
- No arm motion while the agent or web UI holds the arm lease, and no lease preemption.
- No navigation refusal or cancel when the lift is skipped or fails; the robot drives with today's view.
- No change to head_aim_cpp aiming (it already treats the arm as occlusion only).
- No lift for docking actions (opennav docking), only NavigateToPose.

## Design

Flow (rung 1): `RosRobot` timer tick (ros_iface.py, next to `on_footprint_tick`) calls `DrivePose.tick(now,
nav_active())`. `DrivePose` holds a small state machine: `home` -> (goal active, preconditions) -> `lifting` ->
`lifted` -> (no goal for `lower_delay_s`) -> `lowering` -> `home`; any skip goes to `skipped: <reason>` until the next
goal edge. Motions run on a worker thread so the timer never blocks: lift = `ArmController.acquire`, then
`home_motion(drive_pose, floor=None, speed_scale)` (reuses `plan_home`/`home_routes` for any target pose, arm.py:2282),
then `release()` (keep position, default torque); lower = `ArmController.home(keep_prior_control=False)`.

Preconditions read existing state: `ArmController.at_home` (arm.py:2464), `ArmController.torque_off`, `fresh_sample`,
ControlManager lease holders and `carrying_state`, `LeaderActivity.moving`, BatteryGuard. A lease taken mid-drive by
agent or web UI, or a measured pose off the drive pose at goal end, means hands off.

Footprint: after the keep release the existing tick follows the measured arm; the 1.2 test pins that the drive pose
shadow lies inside the static outline, so the published footprint stays static and ArmStop stays disabled. If the pose
were ever outside it, the existing dynamic footprint and ArmStop handle it (no new code).

Torque off or bridge down: `acquire` raises on torque off and `require_sample` on stale joint states; both become
`skipped: torque off` / `skipped: no joint states` and navigation proceeds. A bridge that dies mid-lift ends the motion
as today (move_joints result not converged), state `skipped: motion <status>`, arm left where it is.

reuse: `plan_home`/`home_routes`/`home_motion` (arm.py), `nav_active` (ros_iface.py:2407), the footprint tick
(footprint.py), mcp_server arm FK and body check (arm_geometry, self_collision) for the 1.2 test, gen_nav_footprint
`parked_arm_joints` as the home the test starts from, `footprint_geometry.static_outline`.

## Decisions

- D1: mcp_server commands the lift, not head_aim_cpp, a new node, or the Nav2 BT. It already has the arm FK, body and
  head collision model, collision-free routes, the autonomy lease, carry state and the footprint; head_aim_cpp is C++
  with no arm model, a new node would have to be C++ and duplicate all of that, and a BT node cannot run arm safety
  checks. (my call)
- D2: the trigger is the NavigateToPose goal status (any sender), the same signal head_aim_cpp uses, so web_ui and
  agent goals both lift. Lower after `lower_delay_s` without an active goal so chained goals (frontier exploration,
  replans) do not cycle the arm. (my call)
- D3: release the lease (keep position) once lifted instead of holding it through the drive: holding the sticky
  autonomy lease would lock out the web UI and the agent's take_control for the whole drive and keep the footprint
  forced static. The leader cannot snap the arm (proximity rule). (my call)
- D4: drive pose values are chosen offline with the mcp_server FK by the implementer: start from the parked home,
  change only shoulder_lift, elbow_flex and wrist_flex (wrist_roll and gripper unchanged, so no roll guard), pick the
  candidate that passes every 1.2 check with the least gravity torque on shoulder_lift (upper arm near vertical,
  forearm folded up and back, not stretched); the 1.2 test pins it and the user confirms it live before enabling.
  Alternative: the user jogs to a pose and stores it (rung 2.1). (my call)
- D5: no wait before driving in rung 1: every lift leg stays inside the static footprint and above the floor (1.2), so
  moving while lifting is safe; the cost is about 2 s of today's view at goal start. Waiting is rung 3. (my call)
- D6: skip, never fight: any lease holder, torque off, stale states, carrying, leader moving or a moved arm means the
  controller does nothing that goal. (my call)

## Open questions

- Q1: Lift only from the parked home and lower only when the arm is still at the drive pose, never touching an arm the
  agent, web UI or leader has moved? [recommended: yes]
- Q2: Drive pose chosen by an FK search (upper arm near vertical, forearm folded up, wrist_roll and gripper as home),
  pinned by a test that also checks no arm link enters the D435 frustum for pan 0 to -90 deg, then confirmed by you
  live at speed_scale 0.1 before auto mode is enabled? [recommended: yes]
- Q3: Navigation does not wait for the lift in rung 1 (robot may start moving during the ~2 s lift, every leg proven
  inside the footprint)? [recommended: yes; waiting is rung 3]
- Q4: When the lift is skipped (torque off, bridge down, lease held, carrying), drive anyway with today's view and
  report the reason in get_control_state, rather than refusing the goal? [recommended: yes]
- Q5: Lower the arm 3 s after the last goal ends, on success, abort and cancel alike? [recommended: yes]
- Q6: Ship with `enabled: false`, live-check, then flip to `true` in the same plan? [recommended: yes]

## Research

- Codebase (own discovery, read-only): facts cited in What and why and Design.
- research-docs: skipped; Nav2 goal status and BT Wait are already used or documented in-repo; would run for rung 3's
  BT condition plugin.
- research-code-external, research-papers: skipped; a fixed pose and a goal-status trigger need no prior art.
- ideas: a FK occlusion score (fraction of the head_aim floor look region hidden by the arm) could rank poses and
  later drive the head's pan choice; noted, not in any rung.

## Irreversible operations

- Deploy of mcp_server (client): its restart ends a running claude_agent session (MEMORY: deploy during agent
  session). Safeguard: check for a running session first; reversible by redeploying.
- Live arm motion: first lift at speed_scale 0.1 with the user watching, per the servo safety rule (shoulder_lift and
  elbow_flex are dramatic joints; the gate answer is the user's go-ahead for those two joints only).
