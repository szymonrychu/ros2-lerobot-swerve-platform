# Pan tilt head control with agent and web UI design

## Summary

Two Feetech servos (pan, tilt) carry the head RGB camera (head_camera_cpp, `/head_camera/*`) and the Arducam ToF (`/tof`)
on one rigid frame. They are a `head` servo group on the base bus node `base_servos` left by the split-bus epic, with per-joint speed
and acceleration caps and a configurable home pose (forward and down) that the bridge drives to at startup. One head URDF
(`nodes/web_ui/urdf/head.urdf`) is the geometry source for robot_state_publisher (TF for `head_camera_optical_frame` and
`tof_optical_frame`), for mcp_server (look-at maths, front camera pose) and for the web UI 3D view. The agent gets
`set_head_angles`, `head_look_at` and `head_home` under either the arm or the base lease; a release that leaves the agent
holding no lease homes the head. The web UI Head tab gets live pan/tilt sliders and a Home button.

## Intent and why

The head points the RGB camera and the ToF where the robot needs to look: the ground in front of it for driving, the
object for grasping, a frontier when exploring. Today the head mount does not exist in software: the Head tab shows
disabled sliders, `cameras.front` is validated to sit on `base_link`, and no TF exists for either head camera (the
head-camera epic explicitly left TF to this epic). The servo bus split (blocking epic) frees the bus capacity the two head
servos need.

## Goals

- G1: a `head` servo group (`head_pan`, `head_tilt`) on the base bus node `base_servos`: configurable IDs, ranges (defaults pan +-90
  deg, tilt -30/+45 deg, servo centre 2048 = straight and level), per-joint speed/acceleration caps, `/head/joint_states`
  published, commands on `/head/joint_commands`.
- G2: a configurable home pose (forward and down, not the servo centre) used at startup, on Home, and when the agent's
  controlling lease is released.
- G3: TF from robot_state_publisher on the head URDF (mount, pan, tilt, rigid head frame, both optical frames with the
  configured spacing) driven by `/head/joint_states`; static_tf_publisher honours roll and pitch.
- G4: agent head tools (set angles, look at a point once in base_link or map, home) under the arm or the base lease;
  releasing returns the head home; camera-scene code takes the head pose instead of assuming base_link.
- G5: web UI Head tab sliders + Home button; head drawn in the 3D view.
- G6: tests with fake buses/robots, READMEs, deployed; first live test +-5 deg pan only, then widened.

## Non-goals

- Continuous target tracking (U5: aim once).
- Auto-aiming from the zoom tool or a web click (the agent may call `head_look_at` and then zoom).
- Arbitration between the web UI and the agent for the head (orchestrator ruling R1, D13: web UI head commands are
  always allowed; last writer wins).
- A stop behaviour for the head: `stop` does not touch the head (moves are capped and short) (my call).
- Adding the head to the arm's self-collision body model (orchestrator ruling R2: not in this epic).
- Head camera intrinsics calibration (cameras.front intrinsics stay null until a calibration file exists).

## Anti-goals

- The arm bus or the swerve drive regressing: the follower bus gate (>= 95 Hz) and the base bus rate target from the
  split epic keep passing.
- Any servo moving at uncapped full speed after a brownout (D3 writes the caps with every goal).
- Placeholder data: no head joint state, TF or 3D head pose shown before a real sample (robot_state_publisher publishes
  the moving joints only from real joint states; the web UI disables the sliders and hides the 3D head until the first
  sample).
- A dramatic first live motion: the first test is +-5 deg pan only (CLAUDE.md servo safety, U3).
- Scope creep into tracking, arbitration services, a C++ head controller node, or calibration.

## Constraints

- ROS 2 Jazzy, Ubuntu 24.04, Pi 5 client. mcp_server, web_ui and claude_agent stay Python (MEMORY "Python nodes
  stay"); the feetech bridge and static_tf_publisher are existing Python nodes and stay Python; robot_state_publisher is
  the apt C++ node, so no new code node is written.
- TDD, test-first with fakes: `nodes/bridges/feetech_servos/tests/sts_bus.py` (emulated STS bus with byte counters),
  mcp_server `tests/fakes.py` / `FakeRobot`, vitest mocks. No robot in any automated test.
- No placeholder data on topics (CLAUDE.md).
- Servo safety (CLAUDE.md): least dramatic movement first; the runbook starts at +-5 deg pan.
- Deploys only through `scripts/deploy-nodes.sh` with logs in `.logs/`; deploying mcp_server or claude_agent kills a
  running agent session: check first (MEMORY).
- The base bus node `base_servos` blocks at startup until every configured servo answers (`bridge.py:_wait_for_servos`): the head
  servos must be installed with their IDs set before the head group is deployed, or the swerve drive stops working.
- Planned against the END state of the blocking epics; each task re-reads the files it touches. Expected end state:
  - Split-bus epic (its design.md D1-D7): `lerobot_follower` drives only the arm; the base bus is `base_servos`
    (`feetech_servos`, primary namespace `swerve_drive`, `control_loop_hz: 50`, its own `/dev/serial/by-id` adapter,
    `battery_interval_s: 0`, `node_name` set, metrics 19115) with a client.yml comment marking where the `head`
    `extra_groups` entry goes; `nodes/bridges/feetech_servos/tests/test_bus_budget.py` loads both configs from client.yml
    and shows base + a stand-in 2-servo `HEAD` within 0.8 of the 50 Hz period; feetech README `## Bus budget`,
    `ansible/README.md`, `tests/README.md`, ansible-deploy and teleop-diagnostics skills, MEMORY.md updated.
  - Head-camera epic: `nodes/head_camera_cpp` publishing `/head_camera/image_raw{,/compressed}` and
    `/head_camera/camera_info` with frame `head_camera_optical_frame`; mcp_server `topics.head_camera`, tool camera value
    `front` (described as the head RGB camera); claude_agent `DEFAULT_CAMERA_NOTE` rewritten (D11 there); web_ui Head
    tab `rgb_camera_topic: /head_camera/image_raw/compressed` with click-to-zoom (`HeadTab.tsx`, `HeadTab.test.tsx`,
    `camera/zoomApi.ts`), web_ui `server.py` `/api/camera/zoom`; `tests/test_head_camera_live_refs.py`.
  - Jetson removal epic: Head tab with ToF depth + amplitude only (no fused/stereo toggle), Overview tab gone,
    `tests/test_jetson_retired.py`, web_ui `config.py` / `default.yaml` / `bridge.py` without stereo fields.
  - `arducam_tof` publishes with `frame_id: tof_optical_frame` (config default).

## Orchestrator rulings

- R1: web UI head commands are always allowed (operator); last writer wins against the agent; the agent needs the arm
  or the base lease (D9, D13).
- R2: the head is not added to the arm self-collision model in this epic (non-goal).
- R3: the tilt sign is a config value with a documented default, positive tilt = looking down (matches home "forward
  and down"); decided and confirmed by the user (2026-10-18) (D2).

## Decisions

### D1: Head servos are a `head` extra group on the base bus node `base_servos`

The base bus node `base_servos` (split epic D1, D7) gets `extra_groups: [{namespace: head, joint_names: [head_pan, head_tilt]}]`, giving
`/head/joint_states` and `/head/joint_commands` with no new process, at the base bus rate of 50 Hz. The split epic's
bus budget (its D5) projects base + a 2-servo head at 14.4 ms per worst-case driving cycle against the 20 ms period
(69 Hz capacity, 28 % headroom); the capped 7-byte head goal block (D3) adds a few bytes and is re-checked by
`test_bus_budget.py` with the real head group. Head commands go straight to the bridge (no
filter_node), with `header.frame_id` `autonomy` (mcp_server) or `web_ui`; both are listed in the base bus node `base_servos`'s
`direct_command_sources` (no source range mapping on head joints anyway). Alternatives: a separate `head_servos` bridge
on its own adapter (needs a third USB adapter; U1 chose the base bus); a dedicated head controller node (most code, only
worth it for arbitration or smoothing, which are non-goals). Serves G1, U1.

### D2: Names, ranges, sign conventions and default IDs

Joints `head_pan` and `head_tilt`; topics `/head/joint_states`, `/head/joint_commands`. Radians on every interface
except the agent tools (degrees, D9). Servo centre 2048 = 0 rad = straight and level (U2). Pan positive = left (CCW seen
from above, like the steering servos and REP 103 yaw). Tilt positive = looking down (REP 103 pitch about +y), matching the home "forward and down"; the sign is a config
value (URDF axis plus the bridge `inverted` flag), documented with this default, and confirmed by the user (2026-10-18)
(orchestrator ruling R3; the user's "-30/+45" read as 30 deg up, 45 deg down). Defaults (U2): pan [-90, +90] deg, tilt [-30, +45]
deg, held in three places that a root test keeps equal: the URDF joint limits, the bridge `command_min/max_steps` (pan
1024..3072, tilt 1707..2560 before `inverted`), the web UI slider ranges. Servo IDs 20 (pan) and 21 (tilt): SET ON THE HARDWARE 2026-10-18 (both servos installed on the base bus
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00` next to swerve 32-39; arm 1-6 alone on its own adapter); configurable in client.yml, set on the servos with
`scripts/set_servo_id.py` before deploy. Serves G1, U2.

### D3: Per-joint speed and acceleration caps in the bridge

New optional per-joint keys for position joints: `max_speed_rad_s` and `max_acceleration_rad_s2` (validated > 0).
They are converted to the STS registers (`goal_speed` in steps/s, 4096 steps per turn; `acceleration` in 100 steps/s^2,
1..254), rounding so a small cap never becomes 0 (0 means unlimited on STS). A capped joint's goal goes out as one sync
write of the 7-byte block starting at `STS_ACC` (41: acceleration, goal_position, goal_time = 0, goal_speed), so every
goal and every goal refresh carries its caps and a servo reset (RAM cleared) never runs a move uncapped; uncapped joints
keep the 2-byte goal_position sync write unchanged (arm and steering bytes on the bus are unchanged). Defaults for the
head: 1.0 rad/s and 3.0 rad/s^2 (my call). Alternatives: one-off `set_register` writes (RAM, lost on brownout);
trajectory interpolation in a commander (more code, needs a node). Serves G1, U3.

### D4: Home pose: one value, four consumers, the bridge homes at startup

Default home: pan 0 deg, tilt +30 deg (looking forward and down; the optical axis meets the floor about 0.6 m ahead)
(my call within U4). Consumers: the bridge (new per-joint `startup_position_rad`: after the existing hold-present-position
and torque-on, the bridge queues it as the first pending target, so it goes out with the D3 caps), mcp_server
`head.home` (head_home, lease release), web_ui head tab `head_home_deg` (Home button), and the README. A root test pins
all of them equal and inside the limits. Alternatives: an Ansible variable templated into three config blocks (the config
blocks are parsed as YAML by the root tests, which `{{ }}` breaks); mcp_server homing at its own startup (a server restart
would move the head; the bridge owns the servos). Serves G2, U4.

### D5: One head URDF is the geometry source

`nodes/web_ui/urdf/head.urdf` (next to `so101_arm.urdf`, already read by web_ui and mcp_server), no meshes (boxes and
cylinders). Chain: `base_link` -fixed `head_mount_joint` (0.15, 0.10, 0.31)-> `head_pan_base` -revolute `head_pan`
(axis z, +-pi/2)-> `head_pan_link` -revolute `head_tilt` (origin z +0.04, so the tilt axis is ~0.35 m up; axis +y,
[-0.5236, 0.7854])-> `head_link` (rigid head frame) -fixed-> `head_camera_link` (x +0.02, y -0.025: RGB on the right)
-fixed rpy (-pi/2, 0, -pi/2)-> `head_camera_optical_frame`; `head_link` -fixed-> `tof_link` (x +0.02, y +0.025: ToF
on the left, MEMORY head cameras) -fixed rpy (-pi/2, 0, -pi/2)-> `tof_optical_frame`. The camera-to-ToF spacing is the
two fixed-joint origins (0.05 m default, my call), edited in the URDF (U2). The post visual comes up from behind and above
the lidar (lidar at (0.158, 0.06, 0.20)): it crosses the lidar plane only behind the lidar, inside the footprint box that
laser_filter removes, then runs above the lidar to the pan axis (U2). Joint velocity limits equal the D3 caps. Serves G3,
U2.

### D6: TF from robot_state_publisher, node type `head_state_publisher`

A launch file `nodes/head_state_publisher/launch/head_state_publisher.launch.py` starts `robot_state_publisher` (apt,
part of ros-base; `ros-jazzy-robot-state-publisher` listed explicitly) with `robot_description` read from the URDF file,
`joint_states` remapped to `/head/joint_states` and `robot_description` to `/head/robot_description`, node name
`head_state_publisher`. The fixed joints go out on `/tf_static` (configured geometry, not sensor data); the moving joints
only after real joint states arrive. Ansible: native node type (`ros2 launch`, `src_paths` = the launch dir and the URDF
so either change restarts it), ros2_nodes entry, playbook step, mid cgroup tier (my call). Alternatives: `-p
robot_description:="$(cat ...)"` on the command line (YAML-parses the XML; fragile); extending static_tf_publisher with
joint states (rebuilds robot_state_publisher). Serves G3.

### D7: static_tf_publisher honours roll and pitch

`FrameTransform` gains `roll` and `pitch` (default 0); the quaternion is the fixed-axis roll-pitch-yaw (ROS/URDF RPY)
product in pure `math` (no scipy: the unit has `memory_max: 96M`). The deployed frames have roll = pitch = 0, so TF is
unchanged; the fix makes the keys mean what client.yml says. Serves G3.

### D8: mcp_server head kinematics and look-at maths

`mcp_server/head_kinematics.py` loads the head URDF (ikpy chain from `base_link` to each optical frame, the library
`ik.py` already uses) and provides forward kinematics (`T_base_link_link(pan, tilt, link)`) and `look_at(point_base)`:
the (pan, tilt) inside the joint limits for which the RGB optical axis (+z of `head_camera_optical_frame`) passes through
the point, accounting for the mount, tilt-axis and camera offsets. Solved by `scipy.optimize.least_squares` on the angle
between the optical axis and the camera-to-point ray, seeded with the offset-free atan2 solution; accepted when the
residual is under 0.2 deg. A point outside the reachable cone (needs angles past the limits) or closer than 0.15 m to the
camera is refused with the angles it would need (never a partial aim) (my call). Serves G4, U5.

### D9: Head controller, tools and lease need "any"

`mcp_server/head.py` `HeadController` over a `HeadBackend` protocol (publish a JointState command with frame_id
`autonomy`, latest `/head/joint_states` sample): fresh-sample check (`head.state_max_age_s`), limit check, publish,
wait for convergence (`head.tolerance_rad` 0.03, `head.move_timeout_s` 4 s). `RosRobot` implements the backend;
`RobotApi.head` exposes it; config section `head` (urdf_path, joint names, topics, home, limits read from the URDF,
timeouts). Tools in `mcp_server/head_tools.py`: `set_head_angles(pan_deg, tilt_deg)`, `head_look_at(x, y, z,
frame='base_link'|'map')` (map converted with the current robot pose; refused when the pose is unknown), `head_home()`.
Out-of-range angles are a tool error naming the limits (no clamping). `TOOL_SUBSYSTEM` "any" (U6: either lease);
`MOTION_TOOLS` (battery cut-off refuses them); not in `RATE_SUBSYSTEMS` (head moves need no arm/base full-rate joint
states); `get_robot_state` gains the measured head angles (absent when stale, never zero-filled). Serves G4, U5, U6.

### D10: A release that leaves the agent with no lease homes the head

`ControlManager.release` (default reset) homes the head after the per-subsystem resets when the call released at least
one lease and the agent now holds neither the arm nor the base lease; it waits for convergence and reports
`reset.head`: `homed`, `failed: <why>`, `skipped: battery_cutoff`, or `kept` (keep_position=true). Releasing one lease
while the other is still held reports `kept: <other> lease held` and does not move the head (the "any" hand-over, as
`control.py:49` planned). A lease lost involuntarily (web UI take, generation change) does not move the head (my call).
claude_agent's job-end park (`release_control(subsystem='all')`) therefore homes the head with no agent change. Serves
G2, G4, U4, U6.

### D11: The front camera pose follows the head

`cameras.front.parent_frame` must be a link of the head URDF (default `head_link`; `FRONT_CAMERA_PARENT` base_link check
replaced), and `camera_scene.parent_transform('front')` returns the head link pose in base_link from the measured head
joints (an error when head joint states are stale, as the gripper camera does for arm joints). client.yml
`cameras.front`: `parent_frame: head_link`, `mount` = the URDF `head_link -> head_camera_link` origin, intrinsics still
null. Consumers updated: `camera_tools` calibration samples store the head parent pose, `lookup_tools.camera_pose_base`
composes the head pose, frontier facing uses the recorded camera pose (the "looks along the base heading" fallback stays
only for records without one), `approach_tools.camera_view_problem` evaluates with the current head pose. Serves G4.

### D12: claude_agent knows the head tools

The three head tools join `DEFAULT_EFFECTOR_TOOLS` (and client.yml) so effector + denied = mcp_server `MOTION_TOOLS`
(`tests/test_claude_agent_config.py`); the prompt and the camera note say the front (head) camera sits on a pan/tilt
head, can be aimed once with `head_look_at` (then zoomed), needs the arm or the base lease, and goes home when the job
parks. Serves G4, U5.

### D13: web_ui head commands and Head tab controls

Backend: head tab fields `head_command_topic` (`/head/joint_commands`), `head_joint_states_topic`
(`/head/joint_states`), `head_pan_range_deg` (-90, 90), `head_tilt_range_deg` (-30, 45), `head_home_deg` (0, 30); the
command topic joins `publish_topics()` and `frame_id_defaults()` (`web_ui`); the WS publish path refuses a head command
outside the configured ranges (after the battery cut-off check). Frontend: the disabled placeholder becomes two live
sliders (initialised from the measured angles, disabled until the first `/head/joint_states` sample), publishing a
`sensor_msgs/JointState` (`head_pan`, `head_tilt`, radians) while dragging at most 5 Hz and once on release, and a Home
button publishing `head_home_deg`. Arbitration (orchestrator ruling R1): web UI head commands are always allowed (the operator needs no lease); last
writer wins against the agent, whose head tools still need the arm or the base lease; the agent's next head command or
lease-release home overrides a web UI pose. Serves G5, U7.

### D14: The head in the 3D map view

map_nav tabs get `head_urdf` (`head.urdf`) and `head_joint_states_topic`; `MapScene`'s robot layer draws the head URDF at
the base origin (its root is base_link) with the measured joint values, under a new `head` layer toggle; nothing is
drawn before the first head joint state. Serves G5, U7.

### D15: Verification and a staged robot runbook

A final task runs every suite and a static check that no placeholder head text or base_link-only front camera
assumption remains, and hands over the runbook: install servos and set IDs, deploy the base bus node `base_servos` with the head group,
verify `/head/joint_states` and both bus rates, +-5 deg pan only, then tilt +-5 deg, then the full ranges and home,
TF echo, agent tools, web UI, lease release homing, lidar post shadow check. Serves G6, U3.

## Research

| Lens | Ran or skipped | Adopted ideas | Rejected ideas |
|---|---|---|---|
| discovery (codebase brief) | ran: `pantilt-brief.md` (scratch; findings copied here) plus planner reads of the files below | `extra_groups` per-namespace topics (`feetech_servos/config.py:274`); STS register map `STS_ACC` 41 / `goal_speed` 46 (`registers.py:37,40`); broadcast sync write (`sync_write.py:sync_write_register`); hold-then-torque startup (`bridge.py:205-228`); lease need `"any"` already modelled (`control.py:49-51`, `ControlManager.refusal`); `frame_id_defaults` publish pattern (`web_ui/config.py:frame_id_defaults`); URDF next to `so101_arm.urdf` read by mcp_server (`config.py:DEFAULT_URDF`); apt node pattern (`client.yml` `joint_state_throttle_follower`); launch-file node pattern (`robot_localization_ekf`) | a separate head bus (U1); a C++ head controller (non-goal arbitration/smoothing); `-p robot_description:="$(cat)"` (YAML-parses XML) |
| research-code-internal | skipped as a separate run: the brief's file map plus targeted reads cover reuse (rows below); no new shared helper beyond the head modules | reuse rows below | - |
| research-code-external | skipped: robot_state_publisher with a URDF is the standard ROS pan/tilt TF pattern; no open fork | - | - |
| research-docs | skipped as a separate run: the only new external surfaces are robot_state_publisher parameters/remaps (standard) and two STS RAM registers already mapped in `registers.py`; checked against the ROS 2 robot_state_publisher README conventions | `robot_description` as a parameter, `joint_states` remap | - |
| research-papers | skipped: two-joint look-at is closed-form-seeded least squares, no research core | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief 1 | `extra_groups` gives `/head/*` with zero bridge code | head-servo-group-config |
| brief 1 | speed limit gap: no goal_speed/acceleration for position joints | bridge-position-speed-caps |
| brief 1 | bus budget risk on a shared bus | head-servo-group-config (`test_bus_budget.py` with the real head group), head-suite-and-robot-runbook |
| brief 1 | startup hold-then-torque keeps the head still at boot | bridge-startup-goal (home queued after torque-on) |
| brief 1 | fakes `tests/sts_bus.py`, `tests/ros_stubs.py` | bridge-position-speed-caps, bridge-startup-goal |
| brief 2 | no robot_state_publisher; head URDF fits | head-urdf, head-state-publisher-node |
| brief 2 | static_tf_publisher yaw-only bug | static-tf-roll-and-pitch |
| brief 2 | `cameras.front` validated on base_link, `parent_transform` identity | mcp-front-camera-on-head |
| brief 2 | URDF XML consistency tests pattern (`test_robot_urdf.py`) | head-urdf, head-config-consistency |
| brief 3 | `register(ctx)` tool modules, `TOOL_SUBSYSTEM` "any", `MOTION_TOOLS`, `RATE_SUBSYSTEMS` | mcp-head-tools |
| brief 3 | `RobotApi` protocol + `RosRobot` + fakes change together | mcp-head-controller |
| brief 3 | camera geometry consumers (lookup view, frontier facing, approach) | mcp-front-camera-on-head, mcp-lookups-follow-head-pose |
| brief 3 | claude_agent tool lists in sync with MOTION_TOOLS | agent-head-tools-and-prompt |
| brief 3 | Head tab disabled sliders, publish allowlist, `frame_id_defaults` | web-ui-head-command-backend, head-tab-pan-tilt-controls |
| brief 3 | web_ui 3D draws URDFs from joint_states | head-in-3d-map-view |
| brief 4 | option A (extra group + robot_state_publisher + direct publishers) | adopted: D1, D6, D9, D13 |
| brief 4 | option B (separate bus) | rejected: U1 puts the head on the base bus |
| brief 4 | option C (C++ head controller) | rejected: arbitration and smoothing are non-goals |
| brief 5 | product questions 1-9 | answered by U1-U7 and orchestrator rulings R1-R3 (web UI arbitration, arm collision model, tilt sign default) |
| reuse | `sync_write_register` (`nodes/bridges/feetech_servos/sync_write.py`) | bridge-position-speed-caps |
| reuse | `hold_current_positions`, `set_startup_torque_state` (`startup_torque.py`) | bridge-startup-goal |
| reuse | `position_to_raw_steps` (`command_mapping.py`) | bridge-position-speed-caps, head-config-consistency |
| reuse | ikpy URDF chains (`mcp_server/ik.py:ArmKinematics`) | mcp-head-kinematics |
| reuse | `ControlManager.refusal` "any" (`mcp_server/control.py`) | mcp-head-tools |
| reuse | `offset_matrix`, `build_scene` (`mcp_server/camera_scene.py`) | mcp-front-camera-on-head |
| reuse | `RobotModel` (`web_ui/frontend/src/components3d/RobotModel.tsx`), robot-part layers (`map3d/layers.ts`) | head-in-3d-map-view |
| reuse | `frame_id_defaults`, `publish_topics` (`web_ui/web_ui/config.py`) | web-ui-head-command-backend |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Head group on the base bus node `base_servos` | head-servo-group-config |
| D2 Names, ranges, sign conventions, IDs | head-urdf, head-servo-group-config, head-config-consistency |
| D3 Speed and acceleration caps | bridge-position-speed-caps, head-servo-group-config |
| D4 Home pose and startup homing | bridge-startup-goal, head-servo-group-config, head-config-consistency |
| D5 Head URDF | head-urdf |
| D6 robot_state_publisher node type | head-state-publisher-node |
| D7 static_tf_publisher roll/pitch | static-tf-roll-and-pitch |
| D8 Head kinematics and look-at | mcp-head-kinematics |
| D9 Head controller, tools, lease "any" | mcp-head-controller, mcp-head-tools |
| D10 Release homes the head | mcp-lease-release-homes-head |
| D11 Front camera pose follows the head | mcp-front-camera-on-head, mcp-lookups-follow-head-pose |
| D12 claude_agent head tools | agent-head-tools-and-prompt |
| D13 web_ui head commands and Head tab | web-ui-head-command-backend, head-tab-pan-tilt-controls |
| D14 Head in the 3D view | head-in-3d-map-view |
| D15 Verification and runbook | head-config-consistency, head-suite-and-robot-runbook |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `head-urdf` | head-urdf | head-servo-group-config, head-state-publisher-node, mcp-head-kinematics, mcp-front-camera-on-head, web-ui-head-command-backend, head-in-3d-map-view, head-config-consistency | `nodes/web_ui/urdf/head.urdf` |
| `bridge-position-caps` | bridge-position-speed-caps | bridge-startup-goal, head-servo-group-config | `nodes/bridges/feetech_servos/config.py` |
| `bridge-startup-position` | bridge-startup-goal | head-servo-group-config | `nodes/bridges/feetech_servos/config.py` |
| `head-servo-group` | head-servo-group-config | head-state-publisher-node, head-config-consistency | `ansible/group_vars/client.yml` |
| `head-kinematics` | mcp-head-kinematics | mcp-head-controller, mcp-front-camera-on-head | `nodes/mcp_server/mcp_server/head_kinematics.py` |
| `head-controller` | mcp-head-controller | mcp-head-tools, mcp-lease-release-homes-head, mcp-front-camera-on-head, mcp-lookups-follow-head-pose | `nodes/mcp_server/mcp_server/head.py` |
| `head-tool-names` | mcp-head-tools | mcp-lease-release-homes-head, agent-head-tools-and-prompt | `nodes/mcp_server/mcp_server/head_tools.py` |
| `front-camera-head-scene` | mcp-front-camera-on-head | mcp-lookups-follow-head-pose | `nodes/mcp_server/mcp_server/camera_scene.py` |
| `web-head-fields` | web-ui-head-command-backend | head-tab-pan-tilt-controls, head-in-3d-map-view, head-config-consistency | `nodes/web_ui/web_ui/config.py` |
| `head-tab-types` | head-tab-pan-tilt-controls | head-in-3d-map-view | `nodes/web_ui/frontend/src/types.ts` |

## Review focus

- `bridge_cycle.py`: the 7-byte capped goal block (address 41, field order, little-endian, never a 0 speed or
  acceleration for a capped joint), uncapped joints byte-identical to today, refresh writes carry the caps.
- `bridge.py`: the startup home is queued only after torque-on succeeded for that servo, never for a servo left
  unheld, and goes out through the capped write.
- `head_kinematics.look_at`: offsets handled (camera not on the axes), refusal instead of clamping, map conversion.
- `control.py`: head homing only when no agent lease remains, the "any" hand-over keeps the head, keep_position,
  battery cut-off, involuntary lease loss.
- `camera_scene.parent_transform('front')`: stale head joints are an error, never an identity fallback.
- web_ui: range refusal on the server, sliders disabled before the first sample, 3D head hidden before it.
- client.yml: the head group lands on the base bus node `base_servos` (not lerobot_follower), its IDs/steps/caps/home agree with the
  URDF and the other configs.

## Left to the builder

- Internal names of private helpers, the exact URDF visual shapes and colours (within D5's post routing rule), metric
  names if any are added.
- Whether head kinematics uses ikpy FK or a small numpy chain parsed from the URDF, as long as it reads the same URDF
  and the tests pin the poses.
- Slider styling and layout within `frontend/src/layout/designRules` (tab design tests).
- Exact refusal message wording (each must name the limit or the missing lease).

## Assumptions and unknowns

- The split epic lands as its design says (`base_servos`, `test_bus_budget.py`); head-servo-group-config
  re-reads both before editing.
- The base bus has the byte budget for the head at 50 Hz: the split epic's projection gives 69 Hz capacity for base +
  head while driving; head-servo-group-config re-runs `test_bus_budget.py` with the real (capped, 7-byte goal) head
  group, and the runbook measures it on the robot.
- The tilt sign convention (positive = looking down) is decided (R3, confirmed by the user 2026-10-18); were it ever flipped, only the URDF axis, the bridge `inverted` flag and the default ranges would change.
- Servo IDs 20/21 are set on the hardware (2026-10-18); camera spacing 0.05 m and the mount offsets are estimates until measured on the built head:
  the runbook measures and updates the URDF and client.yml.
- The pan post does not shadow the lidar outside the laser_filter footprint box: checked in the runbook (`/scan` shows
  no return at the post bearing beyond the footprint).
- robot_state_publisher is installed with ros-base on the Pi; the node type still lists the apt package.
- The arm can reach the head post; the arm self-collision model does not include the head (R2, non-goal).

## Irreversible operations in scope

- Writing servo IDs to the head servos' EPROM with `scripts/set_servo_id.py` (reversible by writing again; a hardware
  step done or confirmed by the user before deploy, covered by U2 and G6).
- No data deletion, migration, secret rotation or force-push.
