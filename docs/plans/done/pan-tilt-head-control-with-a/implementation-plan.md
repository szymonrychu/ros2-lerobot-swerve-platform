# Pan tilt head control with agent and web UI implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Bridge position speed caps

- Status: done
- Files: `nodes/bridges/feetech_servos/config.py`, `nodes/bridges/feetech_servos/command_mapping.py`, `nodes/bridges/feetech_servos/bridge_cycle.py`, `nodes/bridges/feetech_servos/sync_write.py`, `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/tests/test_position_caps.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 9bc19ee739bd0e0f7711c8755795989e88289946

#### Requirement

- `config.py` (D3, U3): `JointEntry` gains `max_speed_rad_s: float | None = None` and `max_acceleration_rad_s2: float | None = None`, parsed per joint in the primary group and in `extra_groups`; allowed only on `mode: position` joints and only > 0 (otherwise `load_config` rejects the file with a logged reason, like other invalid keys).
- `command_mapping.py`: public `speed_cap_register(rad_s: float) -> int` (steps/s, 4096 steps per turn, rounded up, clamped to 1..32766) and `acceleration_cap_register(rad_s2: float) -> int` (units of 100 steps/s^2, rounded up, clamped to 1..254): a configured cap never encodes as 0, which STS treats as unlimited.
- `bridge_cycle.py`: in `write_goal_positions`, targets of capped joints go out in ONE sync write of the 7-byte block starting at `STS_ACC` (41): acceleration (1 byte), goal_position (2), goal_time (2, always 0), goal_speed (2), little-endian; uncapped joints keep the existing 2-byte goal_position sync write, byte-identical to today. Goal refresh (`goal_refresh_due`) rewrites capped joints with the full block, so caps are restored after a servo reset. A cycle with both kinds sends at most two goal packets.
- `sync_write.py`: a public helper writes a multi-register block (start address, per-servo byte lists) with `GroupSyncWrite`, reusing `sync_write_register`'s semantics.
- `bridge.py` wiring passes the caps through; nothing changes for configs without caps.
- `nodes/bridges/feetech_servos/README.md`: the two keys, units, register encoding and the brownout reason for writing them with every goal.
- Tests first in new `tests/test_position_caps.py` with `tests/sts_bus.py` (byte counters), seen failing: register conversions (1.0 rad/s -> 652, tiny values -> 1, large clamp); config validation (velocity joint with a cap rejected, <= 0 rejected, extra group parsed); capped targets produce one 7-byte block packet with the expected bytes; uncapped targets produce the unchanged 2-byte packet; mixed cycle = two packets; refresh of a capped joint rewrites the block.

#### Acceptance conditions

- a1: capped position joints are written as one 7-byte ACC..goal_speed block with non-zero caps [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q]
- a2: uncapped joints keep the byte-identical 2-byte goal_position sync write [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q]
- a3: caps are validated (position joints only, > 0) and parsed in extra_groups [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q]
- a4: register conversion never encodes a configured cap as 0 [evidence: symbol: speed_cap_register]
- a5: the README documents max_speed_rad_s and max_acceleration_rad_s2 [evidence: command: grep -q 'max_acceleration_rad_s2' nodes/bridges/feetech_servos/README.md]
- a6: feetech_servos tests and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/command_mapping.py`
- `nodes/bridges/feetech_servos/bridge_cycle.py`
- `nodes/bridges/feetech_servos/sync_write.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_position_caps.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- `config.py:JointEntry.max_speed_rad_s`, `.max_acceleration_rad_s2` (part `bridge-position-caps`).
- `command_mapping.py:speed_cap_register`, `acceleration_cap_register`.
- `sync_write.py` block-write helper; `bridge_cycle.py:write_goal_positions` capped path.

#### Expected outcome

Capped position joints always get their caps with every goal write; nothing changes on the bus for uncapped joints.

#### Steps

- Write `tests/test_position_caps.py` against `tests/sts_bus.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_position_caps.py -q`; expect failures.
- Implement; run the verify command; commit `feat(feetech_servos): per-joint speed and acceleration caps`.

#### Notes

- Register map: `registers.py` STS_ACC 41, goal_position 42, goal_time 44, STS_GOAL_SPEED_L 46.
- Existing byte-level tests: `tests/test_bus_io.py`, `tests/test_bridge_cycle.py` must stay green unchanged for uncapped configs.
- The split epic bus budget test (`tests/test_bus_budget.py`) must keep passing.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD (834 web_ui tests pass); the parking flag was a false positive, no parking word on README line 503.

### S2: Head URDF

- Status: done
- Files: `nodes/web_ui/urdf/head.urdf`, `nodes/web_ui/tests/test_head_urdf.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_urdf_scanner.py -q && uv run --frozen poe lint`
- Verified head: 606536d66d22b840deb2fce31bb3b8e0f849d646

#### Requirement

- New `nodes/web_ui/urdf/head.urdf` (D5, U2), no meshes (box/cylinder visuals), root link `base_link`. Chain: fixed `head_mount_joint` base_link -> `head_pan_base` at xyz (0.15, 0.10, 0.31); revolute `head_pan` (axis 0 0 1, limits -1.5708..1.5708, positive = left) -> `head_pan_link`; revolute `head_tilt` (origin 0 0 0.04, axis 0 1 0, limits -0.5236..0.7854, positive = looking down, REP 103 pitch: decided in ruling R3, confirmed by the user on 2026-10-18) -> `head_link`; fixed `head_camera_joint` head_link -> `head_camera_link` at (0.02, -0.025, 0) (RGB on the right); fixed `head_camera_optical_joint` rpy (-1.5708, 0, -1.5708) -> `head_camera_optical_frame`; fixed `tof_joint` head_link -> `tof_link` at (0.02, 0.025, 0) (ToF on the left); fixed `tof_optical_joint` rpy (-1.5708, 0, -1.5708) -> `tof_optical_frame`. Joint `velocity` limits 1.0 rad/s (the D3 caps), `effort` set.
- A comment block at the top states: the mount and spacing defaults are estimates to be measured on the built head; the camera-to-ToF spacing is the two fixed-joint y origins (0.05 m); servo centre 2048 = 0 rad = straight and level; sign conventions. Comments must not break XML parsing.
- Post visual: a link `head_post` (fixed to base_link) whose geometry comes up from behind the lidar (lidar at 0.158, 0.06, 0.20, radius about 0.035 m) and runs above it to the pan axis; at the lidar plane height (z 0.20) the post lies behind the lidar (x < 0.12) and inside the robot footprint (length 0.47 x width 0.386 centred on base_link).
- `nodes/web_ui/README.md`: a `head.urdf` paragraph in the URDF section (chain, frames, sign conventions, where to edit the spacing and mount, who reads it: robot_state_publisher, mcp_server, the 3D view).
- Tests first in new `nodes/web_ui/tests/test_head_urdf.py` (ElementTree, like `test_robot_urdf.py`), seen failing before the URDF exists: chain parent/child order; joint types, axes and limits; mount and camera origins; optical rpy; ToF y > RGB y (left vs right) and spacing 0.05; post at z 0.20 behind the lidar and inside the footprint; every visual uses primitive geometry; `/api/urdf/status` scan (`urdf_scanner.scan_urdf_directory`) reports head.urdf `ok`.

#### Acceptance conditions

- a1: head.urdf has the pan/tilt chain with the specified axes, limits and origins [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q]
- a2: both optical frames exist with optical rpy and ToF left of RGB at 0.05 m spacing [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q]
- a3: the post crosses the lidar plane only behind the lidar and inside the footprint [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q]
- a4: the web_ui README documents head.urdf [evidence: command: grep -q 'head.urdf' nodes/web_ui/README.md]
- a5: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/urdf/head.urdf`
- `nodes/web_ui/tests/test_head_urdf.py`
- `nodes/web_ui/README.md`

#### Interfaces

- `nodes/web_ui/urdf/head.urdf` (part `head-urdf`): links base_link, head_post, head_pan_base, head_pan_link, head_link, head_camera_link, head_camera_optical_frame, tof_link, tof_optical_frame; joints head_pan, head_tilt (revolute) and the fixed joints of the requirement.

#### Expected outcome

head.urdf parses, the urdf scanner reports it ok, and its tests pin the chain, limits, origins, optical frames and post routing.

#### Steps

- Write `nodes/web_ui/tests/test_head_urdf.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q`; expect failures (file missing).
- Write head.urdf and the README paragraph; rerun the verify command; expect all passed; commit `feat(web_ui): head URDF for the pan/tilt head`.

#### Notes

- Pattern: `nodes/web_ui/tests/test_robot_urdf.py` (ElementTree checks). No meshes: primitives only.
- Lidar pose: client.yml static_tf_publisher laser_frame (0.158, 0.06, 0.20). Footprint 0.47 x 0.386 (mcp_server FootprintSettings).
- XML comments must not contain `--`.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD (834 web_ui tests pass); the parking flag was a false positive, no parking word on README line 503.

### S3: lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head)

- Status: done
- Files: `nodes/mcp_server/mcp_server/lookup_tools.py:~133`
- Verify: none
- Verified head: 829b9a2825d5cbea6171dee1e4fdc7b21862b6fc

#### Requirement

lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head)

Where: nodes/mcp_server/mcp_server/lookup_tools.py:~133

Why: Lookups record a wrong front camera pose once the head moves. This belongs to task mcp-lookups-follow-head-pose, which should call camera_scene.parent_transform with head=robot.head.

#### Acceptance conditions

- AC1: lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head) at nodes/mcp_server/mcp_server/lookup_tools.py:~133 is resolved [evidence: judgement]

#### Files

- `nodes/mcp_server/mcp_server/lookup_tools.py:~133`

#### Interfaces

Keep the interfaces around nodes/mcp_server/mcp_server/lookup_tools.py:~133 unchanged unless resolving it requires otherwise: lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head).

#### Expected outcome

lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head) at nodes/mcp_server/mcp_server/lookup_tools.py:~133 is resolved, and Lookups record a wrong front camera pose once the head moves. This belongs to task mcp-lookups-follow-head-pose, which should call camera_scene.parent_transform with head=robot.head. no longer applies.

#### Steps

1. Write a failing test that shows: lookup_tools.camera_pose_base still uses np.eye(4) for the front parent pose instead of parent_transform(..., head=robot.head).
2. Fix it at nodes/mcp_server/mcp_server/lookup_tools.py:~133.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `mcp-front-camera-on-head` during run `pan-tilt-head-control-with-a`.


#### Ruling

Already delivered by mcp-lookups-follow-head-pose (lookup_tools.py:132 uses head_parent_transform with robot.head), merged into main. Closed by the orchestrator.

### S4: nodes/mcp_server/README.md module table and tests/README.md do not list head_kinematics.py / test_head_kinematics.py

- Status: done
- Files: `nodes/mcp_server/README.md`, `tests/README.md`
- Verify: none
- Verified head: 829b9a2825d5cbea6171dee1e4fdc7b21862b6fc

#### Requirement

nodes/mcp_server/README.md module table and tests/README.md do not list head_kinematics.py / test_head_kinematics.py

Where: nodes/mcp_server/README.md (module table near line 2218), tests/README.md

Why: CLAUDE.md requires READMEs to list modules and each test file; those files are outside this task's ownership

#### Acceptance conditions

- AC1: nodes/mcp_server/README.md module table and tests/README.md do not list head_kinematics.py / test_head_kinematics.py at nodes/mcp_server/README.md (module table near line 2218), tests/README.md is resolved [evidence: judgement]

#### Files

- nodes/mcp_server/README.md
- tests/README.md


#### Interfaces

Keep the interfaces around nodes/mcp_server/README.md (module table near line 2218), tests/README.md unchanged unless resolving it requires otherwise: nodes/mcp_server/README.md module table and tests/README.md do not list head_kinematics.py / test_head_kinematics.py.

#### Expected outcome

nodes/mcp_server/README.md module table and tests/README.md do not list head_kinematics.py / test_head_kinematics.py at nodes/mcp_server/README.md (module table near line 2218), tests/README.md is resolved, and CLAUDE.md requires READMEs to list modules and each test file; those files are outside this task's ownership no longer applies.

#### Steps

1. Write a failing test that shows: nodes/mcp_server/README.md module table and tests/README.md do not list head_kinematics.py / test_head_kinematics.py.
2. Fix it at nodes/mcp_server/README.md (module table near line 2218), tests/README.md.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `mcp-head-kinematics` during run `pan-tilt-head-control-with-a`.


#### Ruling

Folded into the open tasks of run 3 by the orchestrator (planText note): root test classification goes to agent-head-tools-and-prompt, README rows to head-suite-and-robot-runbook. Not a separate task.

### S5: Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home

- Status: done
- Files: `tests/test_claude_agent_config.py`, `tests/test_mcp_server_config.py`
- Verify: none
- Verified head: 829b9a2825d5cbea6171dee1e4fdc7b21862b6fc

#### Requirement

Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home

Where: tests/test_claude_agent_config.py, nodes/claude_agent

Why: Every registered mcp_server tool must be classified there; the agent-head-tools-and-prompt task owns it.

#### Acceptance conditions

- AC1: Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home at tests/test_claude_agent_config.py, nodes/claude_agent is resolved [evidence: judgement]

#### Files

- tests/test_claude_agent_config.py
- tests/test_mcp_server_config.py


#### Interfaces

Keep the interfaces around tests/test_claude_agent_config.py, nodes/claude_agent unchanged unless resolving it requires otherwise: Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home.

#### Expected outcome

Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home at tests/test_claude_agent_config.py, nodes/claude_agent is resolved, and Every registered mcp_server tool must be classified there; the agent-head-tools-and-prompt task owns it. no longer applies.

#### Steps

1. Write a failing test that shows: Root tests/test_claude_agent_config.py (3 tests) fails until claude_agent classifies set_head_angles, head_look_at and head_home.
2. Fix it at tests/test_claude_agent_config.py, nodes/claude_agent.
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `mcp-head-tools` during run `pan-tilt-head-control-with-a`.


#### Ruling

Folded into the open tasks of run 3 by the orchestrator (planText note): root test classification goes to agent-head-tools-and-prompt, README rows to head-suite-and-robot-runbook. Not a separate task.

### S6: Static TF roll and pitch

- Status: done
- Files: `nodes/static_tf_publisher/static_tf_publisher/node.py`, `nodes/static_tf_publisher/static_tf_publisher/config.py`, `nodes/static_tf_publisher/tests/test_transforms.py`, `nodes/static_tf_publisher/README.md`
- Verify: `cd nodes/static_tf_publisher && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 1b150f9754fc52199f5ab09a7c361894770acdff

#### Requirement

- `nodes/static_tf_publisher/static_tf_publisher/config.py`: `FrameTransform` gains `roll: float = 0.0` and `pitch: float = 0.0`, parsed from each frame entry (missing keys default to 0, non-numeric values rejected like the existing keys) (D7).
- `node.py`: the yaw-only `_yaw_to_quaternion` is replaced by a public `rpy_to_quaternion(roll, pitch, yaw) -> tuple[float, float, float, float]` (x, y, z, w) implementing the fixed-axis roll-pitch-yaw convention of ROS/URDF (q = qz(yaw) * qy(pitch) * qx(roll)) in pure `math` (no scipy or numpy: the unit has memory_max 96M); every published transform uses it.
- The deployed client.yml frames (imu_link, laser_frame with roll = pitch = 0) produce exactly the same quaternions as before (no TF change on the robot).
- `nodes/static_tf_publisher/README.md`: roll/pitch documented (radians, fixed-axis RPY like URDF).
- Tests first in new `nodes/static_tf_publisher/tests/test_transforms.py`, seen failing: yaw-only values unchanged (yaw pi gives (0, 0, 1, 0) within 1e-9); roll pi/2 alone; pitch pi/2 alone; the optical rotation rpy (-pi/2, 0, -pi/2) gives (-0.5, 0.5, -0.5, 0.5) within 1e-9; unit norm; config parsing of roll/pitch (present, absent, invalid).

#### Acceptance conditions

- a1: rpy_to_quaternion matches the ROS fixed-axis RPY convention including the optical rotation [evidence: command: cd nodes/static_tf_publisher && uv run --frozen pytest tests/test_transforms.py -q]
- a2: roll and pitch are parsed from config and used for every published transform [evidence: symbol: rpy_to_quaternion]
- a3: yaw-only frames keep their previous quaternion [evidence: command: cd nodes/static_tf_publisher && uv run --frozen pytest tests/test_transforms.py -q]
- a4: static_tf_publisher tests and lint pass [evidence: command: cd nodes/static_tf_publisher && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/static_tf_publisher/static_tf_publisher/node.py`
- `nodes/static_tf_publisher/static_tf_publisher/config.py`
- `nodes/static_tf_publisher/tests/test_transforms.py`
- `nodes/static_tf_publisher/README.md`

#### Interfaces

- `static_tf_publisher/node.py:rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]`; `config.py:FrameTransform.roll`, `.pitch`.

#### Expected outcome

Configured roll and pitch reach the published static transforms; deployed frames are unchanged.

#### Steps

- Write `tests/test_transforms.py`; run `cd nodes/static_tf_publisher && uv run --frozen pytest tests/test_transforms.py -q`; expect failures.
- Implement; run the verify command; commit `fix(static_tf_publisher): honour roll and pitch`.

#### Notes

- Keep pure `math`: the unit memory_max is 96M.
- `tests/ros_stubs.py` exists for node-level tests.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD (834 web_ui tests pass); the parking flag was a false positive, no parking word on README line 503.

### S7: Bridge startup goal

- Status: done
- Files: `nodes/bridges/feetech_servos/config.py`, `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/bridge_cycle.py`, `nodes/bridges/feetech_servos/tests/test_startup_goal.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: cd0233a22769b058807cf0ec716b916d6992f554

#### Requirement

- `config.py` (D4, U4): `JointEntry` gains `startup_position_rad: float | None = None`, allowed only on `mode: position` joints without a source range mapping, finite, and inside the joint's `command_min/max_steps` when those are set (else rejected).
- `bridge.py`: after the existing startup sequence (hold present position, torque on), each joint with `startup_position_rad` whose torque came on gets that position queued as its first pending target through `BridgeCycle` (the same path as a received command, `direct` semantics), so it goes out with the joint's caps (bridge-position-speed-caps) in the first write cycle. A servo left unheld or with torque off gets no startup goal (logged). Joints without the key are untouched (they hold their present position as today).
- The startup goal is logged once per joint (name, rad, steps).
- `nodes/bridges/feetech_servos/README.md`: `startup_position_rad` documented (the head uses it for its home pose; it moves the servo at every bridge start, with the caps).
- Tests first in new `tests/test_startup_goal.py` (`sts_bus.py`, `ros_stubs.py`), seen failing: a joint with the key gets exactly one goal write to the expected steps after torque-on, inside the capped block when capped; no write for joints without the key; no write for a servo whose torque failed; config rejection for velocity joints, source-mapped joints, non-finite and out-of-range values.

#### Acceptance conditions

- a1: a joint with startup_position_rad is driven there once after torque-on, with its caps [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q]
- a2: no startup goal for joints without the key or whose torque did not come on [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q]
- a3: invalid startup_position_rad configs are rejected [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q]
- a4: the README documents startup_position_rad [evidence: command: grep -q 'startup_position_rad' nodes/bridges/feetech_servos/README.md]
- a5: feetech_servos tests and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/bridge_cycle.py`
- `nodes/bridges/feetech_servos/tests/test_startup_goal.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- `config.py:JointEntry.startup_position_rad` (part `bridge-startup-position`); startup queueing in `bridge.py:run_bridge` through `BridgeCycle.handle_command` or an equivalent public method.

#### Expected outcome

A configured joint drives to its startup position once after torque-on, at its caps; other joints behave as today.

#### Steps

- Write `tests/test_startup_goal.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_startup_goal.py -q`; expect failures.
- Implement; run the verify command; commit `feat(feetech_servos): optional startup position per joint`.

#### Notes

- Reuse `startup_torque.py` results (unheld ids, failed torque ids) to decide which joints get the goal.
- `tests/test_bridge_startup_torque.py` shows how the startup sequence is tested.

#### Ruling

Accepted by the orchestrator: verified on committed HEAD (834 web_ui tests pass); the parking flag was a false positive, no parking word on README line 503.

### S8: MCP head kinematics

- Status: done
- Files: `nodes/mcp_server/mcp_server/head_kinematics.py`, `nodes/mcp_server/tests/test_head_kinematics.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q && uv run --frozen poe lint`
- Verified head: cbc8f61af7496c4f67430b5b0fc54b624d513bac

#### Requirement

- New `nodes/mcp_server/mcp_server/head_kinematics.py` (D8, U5): class `HeadKinematics(urdf_path: Path)` reading `nodes/web_ui/urdf/head.urdf` (ikpy chains as `ik.py` does, or a numpy chain parsed from the same URDF with ElementTree). Public API: `limits -> dict[str, tuple[float, float]]` (from the URDF); `link_pose(pan: float, tilt: float, link: str) -> np.ndarray` (4x4 pose of any head link or frame in base_link); `look_at(point_base: np.ndarray) -> tuple[float, float]` returning (pan, tilt) in radians for which the +z axis of `head_camera_optical_frame` passes through the point; raises `HeadAimError` (ValueError subclass) with the needed angles and the limits when the solution lies outside the limits, and when the point is closer than 0.15 m to the camera or the solve residual stays above 0.2 deg.
- The solve accounts for every offset (mount, tilt axis height, camera forward and lateral offset): `scipy.optimize.least_squares` over (pan, tilt) on the angle between the optical axis and the camera-to-point ray, bounded by the limits, seeded by the offset-free atan2 solution. Never returns clamped angles that miss the point.
- Docstrings document frames (base_link x forward, y left, z up, floor z = 0), units and sign conventions (pan + left, tilt + down).
- Tests first in new `nodes/mcp_server/tests/test_head_kinematics.py`, seen failing: at (0, 0) the optical axis is base_link +x; link_pose of head_camera_optical_frame and tof_optical_frame match the URDF origins; look_at residual < 0.2 deg for a grid of reachable floor and raised points (left, right, near, far, below the home view), and the forward kinematics of the returned angles puts the point on the axis within 2 mm at 2 m; a point straight behind the robot and a point needing tilt past +45 deg raise HeadAimError naming the angles; a point 0.1 m from the camera is refused; a point straight ahead on the floor at 0.6 m gives tilt near +30 deg (home view check).

#### Acceptance conditions

- a1: look_at aims the RGB optical axis through reachable points within 0.2 deg, offsets included [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q]
- a2: unreachable or too-close points raise HeadAimError with the needed angles and limits, never a clamped aim [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q]
- a3: link_pose matches the head URDF frames [evidence: symbol: HeadKinematics]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/head_kinematics.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`

#### Interfaces

- `mcp_server/head_kinematics.py:HeadKinematics` (`limits`, `link_pose`, `look_at`), `HeadAimError` (part `head-kinematics`).

#### Expected outcome

mcp_server can compute any head frame pose and the pan/tilt that aims the RGB camera at a point.

#### Steps

- Write `tests/test_head_kinematics.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): head kinematics and look-at`.

#### Notes

- `ik.py:ArmKinematics` shows the ikpy URDF loading used in this package; scipy and numpy are already dependencies.
- Frames: base_link floor z = 0 (camera_scene docstring).

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-urdf

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.

### S9: Head servo group config

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_head_servo_config.py`, `tests/README.md`, `nodes/bridges/feetech_servos/tests/test_bus_budget.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible && cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q`
- Verified head: affec8d01b18dfa9cc195216645b2336b72d448b

#### Requirement

- The split-bus epic leaves the base bus on `base_servos` (`node_type: feetech_servos`, primary namespace `swerve_drive`, 50 Hz, own `/dev/serial/by-id` adapter, `battery_interval_s: 0`, a client.yml comment marking where the head goes) and `nodes/bridges/feetech_servos/tests/test_bus_budget.py` (fake bus, configs loaded from client.yml, a stand-in `HEAD` constant). Re-read both first. The head group goes on `base_servos`, never on `lerobot_follower` (D1, U1).
- Its config gains `extra_groups` entry `namespace: head` with `head_pan` (id 20) and `head_tilt` (id 21) (D2 defaults; comment: set with `scripts/set_servo_id.py` before deploy, the node blocks at startup until every configured servo answers, so deploy only after the servos are installed), `command_min_steps`/`command_max_steps` from the ranges (pan 1024..3072 = +-90 deg; tilt 1707..2560 = -30..+45 deg, swapped consistently if `inverted` is set), `max_speed_rad_s: 1.0`, `max_acceleration_rad_s2: 3.0` (D3), `startup_position_rad` = the home pose (pan 0.0, tilt 0.5236) (D4). Its `direct_command_sources` includes `autonomy` and `web_ui`. A comment block states the sign conventions (pan + = left; tilt + = looking down, decided in ruling R3, confirmed by the user on 2026-10-18; flipped only via `inverted` and the URDF axis) and that the steps, the URDF limits and the web UI ranges are kept equal by tests.
- `test_bus_budget.py`: the base layout now takes the real head group from the loaded `base_servos` config (with its capped 7-byte goal block) instead of adding the stand-in `HEAD`, and still meets `BASE_HEADROOM` (0.8) at the base `control_loop_hz` of 50 Hz (split epic projection: base + head 14.4 ms per worst-case driving cycle vs the 20 ms period; the capped head block adds a few bytes); the arm + head option keeps using the stand-in. The feetech README `## Bus budget` table is recomputed from the test.
- Tests first in new root `tests/test_head_servo_config.py`, seen failing: the head group exists only on `base_servos` (not on lerobot_follower); joint names and ids unique on that bus; steps equal the URDF limits of `nodes/web_ui/urdf/head.urdf` converted with 4096 steps/turn around 2048 (+-1 step); caps present and > 0; startup positions inside the limits; `autonomy` and `web_ui` in direct_command_sources.
- `tests/README.md` lists the new test file and its tests.

#### Acceptance conditions

- a1: the head group is on the base bus node `base_servos` with ids, steps, caps and startup home, not on lerobot_follower [evidence: test: tests/test_head_servo_config.py]
- a2: the head steps equal the head URDF limits [evidence: test: tests/test_head_servo_config.py]
- a3: the bus budget test uses the real head group and base + head still fits the base rate [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a4: tests/README.md lists test_head_servo_config.py [evidence: command: grep -q 'test_head_servo_config.py' tests/README.md]
- a5: root tests, feetech tests and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible && cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_head_servo_config.py`
- `tests/README.md`
- `nodes/bridges/feetech_servos/tests/test_bus_budget.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- client.yml `base_servos` config `extra_groups[namespace=head]` (part `head-servo-group`): joints head_pan (20), head_tilt (21), steps, caps, startup_position_rad; topics /head/joint_states, /head/joint_commands.

#### Expected outcome

The base bus node `base_servos` config drives the head with its ranges, caps and home; the bus budget still holds with the real head.

#### Steps

- Write `tests/test_head_servo_config.py`; run `uv run pytest tests/test_head_servo_config.py -q`; expect failures.
- Edit client.yml and the bus budget test; run the verify command; commit `feat(ansible): head servo group on the base bus`.

#### Notes

- Do not deploy: the node blocks on missing servos; the orchestrator runbook deploys after the hardware gate.
- Steps from `command_mapping.position_to_raw_steps` (0 rad = 2048, 4096 steps per turn).

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-urdf

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.

### S10: MCP head controller

- Status: done
- Files: `nodes/mcp_server/mcp_server/head.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/tool_context.py`, `nodes/mcp_server/tests/fakes.py`, `nodes/mcp_server/tests/test_head_controller.py`, `nodes/mcp_server/tests/test_config.py`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py tests/test_config.py -q && uv run --frozen poe lint`
- Verified head: ae5ce9d56fd91b94c0df78c47a80378a4e8ae5f0

#### Requirement

- `config.py`: new `HeadSettings` (pydantic, `StrictModel`) under `McpServerConfig.head`: `urdf_path` (default `nodes/web_ui/urdf/head.urdf`, resolved like `arm.urdf_path`), `joint_names` (head_pan, head_tilt), `command_topic` `/head/joint_commands`, `state_topic` `/head/joint_states`, `home_pan_rad` 0.0, `home_tilt_rad` 0.5236, `tolerance_rad` 0.03, `move_timeout_s` 4.0, `state_max_age_s` 0.5; validation: home inside the URDF limits (D9, D4).
- New `mcp_server/head.py`: `HeadBackend` Protocol (`publish_head_command(positions: dict[str, float]) -> None`, `head_sample() -> JointSample | None`, `now()`, `sleep()`); `HeadController(backend, kinematics, settings)` with `measured() -> dict[str, float] | None` (None when missing or older than state_max_age_s, never zero-filled), `check(pan, tilt)` (raises `HeadError` naming the limits for out-of-range values, no clamping), `move(pan, tilt) -> HeadMoveResult` (publishes once with frame_id `autonomy`, waits until both joints are within tolerance or move_timeout_s; status `converged` | `timeout` | `no_feedback`, with measured angles), `home() -> HeadMoveResult`. `HeadMoveResult` model in head.py.
- `ros_iface.py`: `RosRobot` implements `HeadBackend` (subscription on `head.state_topic` with the sensor QoS used for joint states, publisher on `head.command_topic` with `COMMAND_QOS`, header.frame_id `autonomy`, JointState name/position only) and exposes `head: HeadController`.
- `tool_context.py`: `RobotApi` declares `head: HeadController`.
- `tests/fakes.py`: `FakeHeadBackend` (records commands, optional simulated follower moving at a set speed, settable stale/missing samples).
- Tests first in new `tests/test_head_controller.py` (and the config cases in `tests/test_config.py`), seen failing: defaults and home validation; measured None when stale or missing; out-of-range refused with limits named; move converges with a following fake, times out with a stuck one, reports no_feedback without samples; command carries frame_id autonomy and both joint names; home moves to the configured home.

#### Acceptance conditions

- a1: HeadController moves, waits for convergence and reports converged, timeout or no_feedback [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py -q]
- a2: out-of-range angles are refused naming the limits, and stale samples are never reported as measured [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py -q]
- a3: head config defaults and home validation [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py -q]
- a4: RosRobot implements the head backend on /head/joint_states and /head/joint_commands [evidence: symbol: HeadController]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/head.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/tests/fakes.py`
- `nodes/mcp_server/tests/test_head_controller.py`
- `nodes/mcp_server/tests/test_config.py`

#### Interfaces

- `mcp_server/head.py:HeadBackend`, `HeadController`, `HeadMoveResult`, `HeadError` (part `head-controller`); `config.py:HeadSettings`; `tool_context.py:RobotApi.head`; `tests/fakes.py:FakeHeadBackend`.

#### Expected outcome

mcp_server has a head controller wired to ROS that reports real arrival or a named failure.

#### Steps

- Write `tests/test_head_controller.py` and the config cases; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_controller.py tests/test_config.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): head controller`.

#### Notes

- Pattern: `arm.py:ArmController` over `FakeArmBackend` (tests/fakes.py).
- No placeholder: `measured()` returns None for stale samples.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-head-kinematics

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.

### S11: Head state publisher node

- Status: done
- Files: `nodes/head_state_publisher/launch/head_state_publisher.launch.py`, `nodes/head_state_publisher/README.md`, `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_head_state_publisher_config.py`, `tests/README.md`, `ansible/README.md`, `.claude/skills/ansible-deploy/SKILL.md`
- Verify: `uv run pytest tests/test_head_state_publisher_config.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible`
- Verified head: 456a20e614664ef5d27227f85c74b5783b25f015

#### Requirement

- New `nodes/head_state_publisher/launch/head_state_publisher.launch.py` (D6): starts `robot_state_publisher` (package `robot_state_publisher`, executable `robot_state_publisher`, node name `head_state_publisher`) with parameter `robot_description` = the text of `nodes/web_ui/urdf/head.urdf` (path resolved relative to the launch file, so it works from the deployed checkout), remappings `joint_states` -> `/head/joint_states` and `robot_description` -> `/head/robot_description`. New `nodes/head_state_publisher/README.md`: what it publishes (fixed joints on /tf_static at start; head_pan/head_tilt transforms only from real joint states), frames, how to check (`ros2 run tf2_ros tf2_echo base_link head_camera_optical_frame`).
- `ansible/group_vars/client.yml`: node type `head_state_publisher` (native, `node_launch_command: ros2 launch {{ ros2_repo_dest }}/nodes/head_state_publisher/launch/head_state_publisher.launch.py`, `node_src_dir: ""`, `src_paths: [nodes/head_state_publisher, nodes/web_ui/urdf/head.urdf]`, `apt_packages: [ros-jazzy-robot-state-publisher]`, cgroup mid tier: memory_low 32M, memory_max 128M, cpu_weight 500, io_weight 500, oom_score_adjust -500, comment "not measured yet"); ros2_nodes entry `head_state_publisher` present/enabled, placed after the base bus node `base_servos`.
- `ansible/playbooks/deploy_nodes_client.yml`: a `Deploy head_state_publisher` step in the existing pattern (tags include `head_state_publisher`, `apt`).
- `ansible/README.md` node table/section and `.claude/skills/ansible-deploy/SKILL.md` node list gain head_state_publisher.
- Tests first in new root `tests/test_head_state_publisher_config.py`, seen failing: node type fields and tier; entry present and enabled after the base bus node `base_servos`; the launch file (parsed with `ast`, no ROS import) names robot_state_publisher, reads head.urdf, remaps both topics; the playbook step exists with its tags; `tests/test_unit_cgroups.py` rules still pass.
- `tests/README.md` lists the new test file.

#### Acceptance conditions

- a1: the head_state_publisher node type and entry deploy robot_state_publisher on head.urdf remapped to /head/joint_states [evidence: test: tests/test_head_state_publisher_config.py]
- a2: the playbook deploys head_state_publisher [evidence: test: tests/test_head_state_publisher_config.py]
- a3: ansible README and the ansible-deploy skill name head_state_publisher [evidence: command: grep -q head_state_publisher ansible/README.md && grep -q head_state_publisher .claude/skills/ansible-deploy/SKILL.md]
- a4: tests/README.md lists test_head_state_publisher_config.py [evidence: command: grep -q 'test_head_state_publisher_config.py' tests/README.md]
- a5: root tests, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible]

#### Files

- `nodes/head_state_publisher/launch/head_state_publisher.launch.py`
- `nodes/head_state_publisher/README.md`
- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `tests/test_head_state_publisher_config.py`
- `tests/README.md`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`

#### Interfaces

- `nodes/head_state_publisher/launch/head_state_publisher.launch.py:generate_launch_description`; client.yml node type and entry `head_state_publisher`.

#### Expected outcome

Ansible can deploy, enable and disable head_state_publisher; it publishes head TF from /head/joint_states.

#### Steps

- Write `tests/test_head_state_publisher_config.py`; run `uv run pytest tests/test_head_state_publisher_config.py -q`; expect failures.
- Add the launch file, README, node type, entry, playbook step and docs; run the verify command; commit `feat(head_state_publisher): robot_state_publisher for the head`.

#### Notes

- Patterns: `robot_localization_ekf` (launch-file native node), `joint_state_throttle_follower` (apt node type, src_paths).
- The launcher runs `bash -c` in single quotes: no single quote in node_launch_command.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-servo-group-config

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.

### S12: MCP head tools

- Status: done
- Files: `nodes/mcp_server/mcp_server/head_tools.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/models.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/tests/test_head_tools.py`, `nodes/mcp_server/tests/test_tools.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 0497c6ccac6f2a6714cd30967f6b2c4ebb3d5594

#### Requirement

- New `mcp_server/head_tools.py` with `register(ctx: ToolContext)` added to `tools.TOOL_MODULES` and `TOOL_NAMES = ("set_head_angles", "head_look_at", "head_home")` (D9, U5, U6):
  - `set_head_angles(pan_deg: float, tilt_deg: float)`: moves the head (degrees in, degrees out); out-of-range is a tool error naming the limits in degrees.
  - `head_look_at(x: float, y: float, z: float, frame: Literal['base_link', 'map'] = 'base_link')`: map points are converted to base_link with the current robot map pose (tool error when the pose is unknown); aims once with `HeadKinematics.look_at` (no tracking) and moves; HeadAimError becomes a tool error with the needed angles.
  - `head_home()`: moves to the configured home.
  - Each returns status, commanded and measured pan/tilt in degrees and, for look_at, the target in base_link.
- `tools.py`: the three tools in `TOOL_SUBSYSTEM` as `"any"`, in `MOTION_TOOLS` (battery cut-off refuses them), absent from `RATE_SUBSYSTEMS` (explicit exclusion with a comment), and listed in the tool-name tuple; `INSTRUCTIONS` gains one sentence on the head tools and their lease need.
- `models.py` / `ros_iface.py`: `RobotState` gains optional `head: {pan_deg, tilt_deg}`, filled only from a fresh head sample (absent otherwise).
- `tests/test_tools.py`: `FakeRobot` gains a `head` backed by `FakeHeadBackend` so every existing test keeps passing.
- `nodes/mcp_server/README.md`: the three tools in the tool table (lease any, motion tool, degrees, aim once) and the head config section.
- Tests first in new `tests/test_head_tools.py`, seen failing: refused without either lease and allowed with the arm lease alone and with the base lease alone; refused under battery cut-off; out-of-range error names limits; look_at in base_link and map (pose used; unknown pose refused); look_at unreachable refused with angles and nothing published; head_home publishes the home; get_robot_state carries head angles only when fresh; head tools not in RATE_SUBSYSTEMS.

#### Acceptance conditions

- a1: the three head tools need either lease and are refused under battery cut-off [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q]
- a2: head_look_at aims once at a base_link or map point and refuses unreachable points without moving [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q]
- a3: get_robot_state reports head angles only from a fresh sample [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q]
- a4: the mcp_server README lists set_head_angles, head_look_at and head_home [evidence: command: grep -q head_look_at nodes/mcp_server/README.md && grep -q set_head_angles nodes/mcp_server/README.md && grep -q head_home nodes/mcp_server/README.md]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/head_tools.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_head_tools.py`
- `nodes/mcp_server/tests/test_tools.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- `mcp_server/head_tools.py:register`, `TOOL_NAMES` (part `head-tool-names`); `tools.py:TOOL_SUBSYSTEM`, `MOTION_TOOLS`, `RATE_SUBSYSTEMS`; `models.py:RobotState.head`.

#### Expected outcome

The agent can call set_head_angles, head_look_at and head_home under either lease.

#### Steps

- Write `tests/test_head_tools.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_tools.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): head tools`.

#### Notes

- `ControlManager.refusal` already supports "any".
- `RATE_SUBSYSTEMS` is derived from TOOL_SUBSYSTEM: exclude the head tools explicitly.
- Root `tests/test_claude_agent_config.py` classifies every registered tool: the agent task updates claude_agent; this task must not break mcp_server suites.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-head-controller

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.

### S13: MCP lease release homes head

- Status: done
- Files: `nodes/mcp_server/mcp_server/control.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/tests/test_head_lease.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: unavailable

#### Requirement

- `control.py` (D10, U4, U6): `ControlManager` takes an optional `head: HeadController | None` (constructed with `robot.head` in `tools.py`). In `release`, after the per-subsystem resets, the head outcome is decided and stored as `reset["head"]` (only when a head is configured):
  - keep_position=true: `kept`;
  - the agent still holds the arm or the base lease after this call: `kept: <arm|base> lease held` (the "any" hand-over: releasing base while holding arm, or the reverse, never moves the head);
  - this call released no lease (everything `skipped: not held`): no head entry and no motion;
  - otherwise (the call released a lease and none remains): under battery cut-off `skipped: battery_cutoff`; else `HeadController.home()` and `homed` when converged, else `failed: <status>`.
  A head failure never blocks the arm/base release (the release always happens, as for the arm reset).
- A lease lost involuntarily (web UI take, generation change) does not move the head; `stop` does not touch the head.
- The module docstring and `nodes/mcp_server/README.md` (control leases section) describe the head rule; the `control.py:49` comment about future head tools is updated to describe the delivered behaviour.
- Tests first in new `nodes/mcp_server/tests/test_head_lease.py` (FakeRobot / FakeHeadBackend, existing control fakes), seen failing: release('all') homes the head and reports homed; release('base') with the arm still held reports `kept: arm lease held` and publishes nothing; release('arm') then release('base') homes only at the second call; keep_position keeps; battery cut-off skips; a stuck head reports failed but both leases are released; a release of nothing publishes nothing; involuntary arm lease loss publishes nothing.

#### Acceptance conditions

- a1: a release that leaves the agent with no lease homes the head and reports reset.head homed [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_lease.py -q]
- a2: releasing one lease while the other is held keeps the head [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_lease.py -q]
- a3: keep_position, battery cut-off, head failure and involuntary loss behave as specified [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_head_lease.py -q]
- a4: the README control section describes the head release rule [evidence: judgement]
- a5: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/control.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/tests/test_head_lease.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- `control.py:ControlManager(head=...)`, `release` head outcome in `ReleaseResult.reset["head"]`; construction in `tools.py`.

#### Expected outcome

Releasing the last agent lease homes the head and reports it; hand-overs and keep_position never move it.

#### Steps

- Write `tests/test_head_lease.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_lease.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): release homes the head`.

#### Notes

- Existing lease tests: `tests/test_control.py`, `tests/test_lease_safety.py` stay green.
- Order: the base and arm resets run first (RELEASE_ORDER), the head last.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-head-tools

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.

### S14: Web UI head command backend

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/config/default.yaml`, `nodes/web_ui/tests/test_head_control.py`, `ansible/group_vars/client.yml`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: unknown

#### Requirement

- `nodes/web_ui/web_ui/config.py` (D13, D14): head tab fields `head_command_topic`, `head_joint_states_topic`, `head_pan_range_deg: tuple[float, float]`, `head_tilt_range_deg: tuple[float, float]`, `head_home_deg: tuple[float, float]` (pan, tilt) with defaults /head/joint_commands, /head/joint_states, (-90, 90), (-30, 45), (0, 30) filled for head tabs; validation: ranges ordered, home inside them. map_nav tabs gain `head_urdf` (default `head.urdf`) and `head_joint_states_topic` (default `/head/joint_states`); `head_joint_states_topic` joins `GENERIC_TOPIC_ATTRS` (subscribed as sensor_msgs/JointState). `publish_topics()` includes `head_command_topic`; `frame_id_defaults()` maps it to `web_ui`.
- Ruling R1: web UI head commands need no lease and are never refused because the agent holds a lease; only the battery cut-off and the range check below refuse them.
- `server.py`: the WS `publish` path refuses a message on a head command topic whose positions (head_pan, head_tilt, radians) fall outside the tab's ranges or are non-finite, with an error message to the client (after the existing battery cut-off refusal); valid ones publish as today.
- `config/default.yaml` and `ansible/group_vars/client.yml` web_ui config: the head tab and the map_nav tab carry the head fields.
- `nodes/web_ui/README.md`: Head tab control fields, range refusal, frame_id web_ui, head commands always allowed for the operator, last writer wins against the agent (ruling R1; no lease check on the web UI path).
- Tests first in new `nodes/web_ui/tests/test_head_control.py`, seen failing: defaults and validation; publish allowlist and frame_id default; in-range command published with frame_id web_ui; out-of-range and non-finite refused; battery cut-off still refuses first; map_nav head fields and the subscription type hint.

#### Acceptance conditions

- a1: head tab fields, defaults and validation exist and the command topic is allowlisted with frame_id web_ui [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_head_control.py -q]
- a2: out-of-range or non-finite head commands are refused server-side [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_head_control.py -q]
- a3: client.yml and default.yaml configure the head tab and map_nav head fields [evidence: command: grep -q head_command_topic nodes/web_ui/config/default.yaml && grep -q head_command_topic ansible/group_vars/client.yml]
- a4: web_ui tests and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_head_control.py`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/README.md`

#### Interfaces

- `web_ui/config.py` head tab fields and map_nav head fields (part `web-head-fields`), `publish_topics`, `frame_id_defaults`; `server.py` WS publish range check.

#### Expected outcome

The web UI backend publishes only valid head commands, tagged web_ui, and serves the head config to the frontend.

#### Steps

- Write `tests/test_head_control.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_head_control.py -q`; expect failures.
- Implement and update default.yaml, client.yml, README; run the verify command; commit `feat(web_ui): head command backend`.

#### Notes

- Battery cut-off refusal (`server.py`, type publish) stays first.
- Head joint states are a generic JointState subscription (GENERIC_TOPIC_ATTRS).

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-state-publisher-node

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD (web_ui 855 passed, mcp_server 2601 passed, lint clean); escalations were a verifier crash (--prev-head unavailable) and stale uncommitted edits in the worktree, discarded.

### S15: Head tab pan tilt controls

- Status: done
- Files: `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`, `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`, `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/hooks/subscription.ts`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 3b074abf93e0381772b4d50ce208b6c7776439de

#### Requirement

- `HeadTab.tsx` (D13, U7): the disabled placeholder (`pan/tilt: no head mount configured`) is replaced by live controls when the tab has `head_command_topic`: pan and tilt sliders with the configured ranges in degrees, initialised from the latest `head_joint_states_topic` sample and disabled (with a "waiting for /head/joint_states" line) until the first sample; dragging publishes `sensor_msgs/JointState` `{name: [head_pan, head_tilt], position: [rad, rad]}` at most every 200 ms and once on release (`onChangeCommitted`); a Home button publishes `head_home_deg`; the measured angles are shown as text. Without `head_command_topic` no controls are shown. The RGB click-to-zoom and ToF views from the blocking epics stay unchanged.
- `types.ts`: head tab fields (`head_command_topic`, `head_joint_states_topic`, `head_pan_range_deg`, `head_tilt_range_deg`, `head_home_deg`) and map_nav `head_urdf`, `head_joint_states_topic`.
- `hooks/subscription.ts`: head tabs subscribe `head_joint_states_topic`; map_nav tabs subscribe theirs.
- `nodes/web_ui/README.md`: Head tab controls section.
- Tests first in `HeadTab.test.tsx` (the "publishes nothing" assertions are replaced), seen failing: sliders disabled before the first sample and enabled after with the measured values; release publishes radians with both names; drag publishes no faster than 5 Hz; Home publishes the home pose; no controls without head_command_topic; tab design rules still pass.

#### Acceptance conditions

- a1: Head tab sliders publish head_pan/head_tilt radians on release and throttled while dragging [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a2: Home publishes the configured home pose and sliders stay disabled until the first head joint state [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a3: the placeholder text is gone [evidence: command: ! grep -rn 'no head mount configured' nodes/web_ui/frontend/src]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`
- `nodes/web_ui/README.md`

#### Interfaces

- `HeadTab.tsx` controls; `types.ts` head fields (part `head-tab-types`); `hooks/subscription.ts` head topics.

#### Expected outcome

The Head tab moves the head with sliders and Home and shows the measured angles.

#### Steps

- Update `HeadTab.test.tsx`; run `cd nodes/web_ui/frontend && npm test -- HeadTab`; expect failures.
- Implement; run `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`; commit `feat(web_ui): Head tab pan/tilt controls`.

#### Notes

- Keep the RGB click-to-zoom and ToF parts from the blocking epics untouched.
- Respect `layout/designRules` (tabsDesign.test.ts) and TOUCH_TARGET_PX.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on web-ui-head-command-backend

### S16: MCP front camera on head

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/camera_scene.py`, `nodes/mcp_server/mcp_server/camera_tools.py`, `nodes/mcp_server/tests/test_camera_scene_head.py`, `ansible/group_vars/client.yml`, `tests/test_mcp_camera_config.py`, `tests/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_camera_config.py tests/test_mcp_server_config.py -q && uv run poe lint`
- Verified head: 2b03f09c73784ef00c6ace24b595f71628f5a8a2

#### Requirement

- `config.py` (D11): `FRONT_CAMERA_PARENT = "base_link"` and its check are replaced: `cameras.front.parent_frame` defaults to `head_link` and must be a link of the head URDF (`head.urdf_path`); a mount, when given, must agree with it (existing rule).
- `camera_scene.py`: `parent_transform(cfg, camera, kin, joints, head=None)` (or an equivalent explicit head argument) returns, for `front`, the pose of `cameras.front.parent_frame` in base_link from `HeadKinematics.link_pose` at the measured head angles; stale or missing head joints raise a ValueError naming the head joint states (never an identity fallback). The module docstring states the front camera now moves with the head.
- `camera_tools.py`: `scene_for` and the calibration sample path pass the measured head angles for `front` (the stored sample's t_parent is the head parent pose).
- `ansible/group_vars/client.yml` mcp_server config: `cameras.front.parent_frame: head_link`, `mount` = the URDF `head_link -> head_camera_link` origin (x 0.02, y -0.025, z 0, rpy 0), intrinsics stay null (not calibrated); the `head:` section with the deploy values (same as the code defaults).
- Root `tests/test_mcp_camera_config.py`: the front pin becomes front = head_link with the URDF mount and null intrinsics; a new test checks `cameras.front.parent_frame` is a link of head.urdf and the mount equals the URDF joint origin. `tests/README.md` rows follow.
- Tests first in new `nodes/mcp_server/tests/test_camera_scene_head.py`, seen failing: front scene at pan 0/tilt 0 equals the URDF camera pose; at pan +30 deg the optical axis yaws left 30 deg; at tilt +30 deg a centre pixel hits the floor about 0.6 m ahead (ground point); stale head joints raise; config rejects a front parent that is not a head URDF link.

#### Acceptance conditions

- a1: the front camera scene follows the measured head pose and refuses stale head joints [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene_head.py -q]
- a2: cameras.front must be on a head URDF link; base_link-only validation is gone [evidence: command: ! grep -n 'FRONT_CAMERA_PARENT = "base_link"' nodes/mcp_server/mcp_server/config.py]
- a3: client.yml sets cameras.front on head_link with the URDF mount and a head section [evidence: test: tests/test_mcp_camera_config.py]
- a4: mcp_server and root camera config tests pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_camera_config.py tests/test_mcp_server_config.py -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/tests/test_camera_scene_head.py`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_camera_config.py`
- `tests/README.md`

#### Interfaces

- `config.py:CamerasSettings` front parent validation; `camera_scene.py:parent_transform` (part `front-camera-head-scene`); `camera_tools.py` scene/sample paths.

#### Expected outcome

Front camera geometry follows the measured head pose; client.yml mounts the camera on head_link.

#### Steps

- Write `tests/test_camera_scene_head.py` and update `tests/test_mcp_camera_config.py`; run them; expect failures.
- Implement; run the verify command; commit `feat(mcp_server): front camera pose follows the head`.

#### Notes

- The gripper camera path (`kin.link_frame`) is the model for a moving parent.
- Intrinsics stay null: tools keep saying not calibrated until a calibration exists.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-head-controller, web-ui-head-command-backend

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on web-ui-head-command-backend

#### Ruling

Accepted by the orchestrator: verified (mcp-lookups-follow-head-pose by hand: mcp_server 2620 passed, lint clean; its verifier agent died) and merged into main.

### S17: Agent head tools and prompt

- Status: done
- Files: `nodes/claude_agent/claude_agent/config.py`, `nodes/claude_agent/claude_agent/prompt.py`, `nodes/claude_agent/tests/test_config.py`, `nodes/claude_agent/tests/test_prompt.py`, `nodes/claude_agent/README.md`, `ansible/group_vars/client.yml`
- Verify: `cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q`
- Verified head: 380fef10e53758e1cf16adc60e93ad294a711df5

#### Requirement

- `nodes/claude_agent/claude_agent/config.py` (D12, U5): `set_head_angles`, `head_look_at`, `head_home` join `DEFAULT_EFFECTOR_TOOLS` (effector + denied = mcp_server `MOTION_TOOLS`); the camera note (as rewritten by the head-camera epic) adds that the front (head) camera sits on a pan/tilt head with the ToF, can be aimed once at a point with `head_look_at` (base_link or map) and then zoomed, and that the head returns home when the job parks.
- `prompt.py`: the control-lease paragraph says the head tools work under either the arm or the base lease and that a release leaving no lease homes the head (`reset.head`); the zoom paragraph says zoom never moves the head (look at first if needed).
- `ansible/group_vars/client.yml` claude_agent config: effector list equals the code defaults (root `tests/test_claude_agent_config.py`).
- `nodes/claude_agent/README.md`: head tools in the tool classification.
- Tests first in `nodes/claude_agent/tests/test_config.py` (head tools are effectors) and `tests/test_prompt.py` (prompt names head_look_at, the lease rule and the home on park), seen failing; root `tests/test_claude_agent_config.py` passes unchanged in its rules.

#### Acceptance conditions

- a1: the head tools are effector tools and the lists match mcp_server MOTION_TOOLS [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_config.py -q]
- a2: the prompt explains aiming with head_look_at, the either-lease rule and homing on park [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q]
- a3: client.yml claude_agent config equals the code defaults [evidence: test: tests/test_claude_agent_config.py]
- a4: claude_agent tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/README.md`
- `ansible/group_vars/client.yml`

#### Interfaces

- `claude_agent/config.py:DEFAULT_EFFECTOR_TOOLS`, `DEFAULT_CAMERA_NOTE`; `prompt.py` lease and zoom paragraphs.

#### Expected outcome

The agent sees the head tools as effectors and is told how to aim and that the head homes on park.

#### Steps

- Extend `tests/test_config.py` and `tests/test_prompt.py`; run `cd nodes/claude_agent && uv run --frozen pytest tests -q`; expect failures.
- Implement and update client.yml; run the verify command; commit `feat(claude_agent): head tools and prompt`.

#### Notes

- Root `tests/test_claude_agent_config.py`: effector + denied must equal mcp_server MOTION_TOOLS; client.yml must equal the code defaults.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-lease-release-homes-head, mcp-front-camera-on-head

#### Ruling

Accepted by the orchestrator: verified (mcp-lookups-follow-head-pose by hand: mcp_server 2620 passed, lint clean; its verifier agent died) and merged into main.

### S18: Head in 3D map view

- Status: done
- Files: `nodes/web_ui/frontend/src/map3d/headJoints.ts`, `nodes/web_ui/frontend/src/map3d/headJoints.test.ts`, `nodes/web_ui/frontend/src/map3d/layers.ts`, `nodes/web_ui/frontend/src/map3d/layers.test.ts`, `nodes/web_ui/frontend/src/map3d/MapScene.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 6a401f5a9ad7d5b1e54cb6bf8bd34c987194e0a0

#### Requirement

- New `frontend/src/map3d/headJoints.ts` (D14, U7): `headJointValues(msg)` maps a `/head/joint_states` message to URDF joint values for `head_pan`/`head_tilt`, returning undefined before a message or when either value is missing or non-finite (the head is then not drawn: no placeholder pose).
- `map3d/layers.ts`: a `head` robot-part layer toggle (default on), next to body/wheels/arm.
- `map3d/MapScene.tsx`: `RobotLayer` draws `RobotModel` for `headUrdf` at the base origin (its URDF root is base_link) with `headJointValues(...)`, visible when the head layer is on and values exist.
- `tabs/MapNavTab.tsx`: passes `tab.head_urdf` and the `head_joint_states_topic` data to `MapScene`.
- Tests first in new `map3d/headJoints.test.ts` and `map3d/layers.test.ts`, seen failing: values from a message; undefined before a message and for missing/non-finite entries; the head layer exists and toggles.

#### Acceptance conditions

- a1: head joint values come only from a real head joint state [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/headJoints.test.ts]
- a2: a head layer toggle exists [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/map3d/layers.test.ts]
- a3: MapScene draws the head URDF with the measured joints [evidence: symbol: headJointValues]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/map3d/headJoints.ts`
- `nodes/web_ui/frontend/src/map3d/headJoints.test.ts`
- `nodes/web_ui/frontend/src/map3d/layers.ts`
- `nodes/web_ui/frontend/src/map3d/layers.test.ts`
- `nodes/web_ui/frontend/src/map3d/MapScene.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`

#### Interfaces

- `map3d/headJoints.ts:headJointValues`; `map3d/layers.ts` head layer; `MapScene.tsx` RobotLayer head model.

#### Expected outcome

The map 3D view draws the head at its measured angles once a head joint state arrived.

#### Steps

- Write `map3d/headJoints.test.ts` and extend `map3d/layers.test.ts`; run `cd nodes/web_ui/frontend && npm test -- headJoints layers`; expect failures.
- Implement; run `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`; commit `feat(web_ui): head in the 3D map view`.

#### Notes

- `RobotModel` takes `jointStates` ({name, position}); the head URDF root is base_link, so it sits at the base origin like the base URDF.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-tab-pan-tilt-controls

### S19: MCP lookups follow head pose

- Status: done
- Files: `nodes/mcp_server/mcp_server/lookup_tools.py`, `nodes/mcp_server/mcp_server/frontiers.py`, `nodes/mcp_server/mcp_server/approach_tools.py`, `nodes/mcp_server/tests/test_lookup_head_pose.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 829b9a2825d5cbea6171dee1e4fdc7b21862b6fc

#### Requirement

- `lookup_tools.py` (D11): `camera_pose_base('front', ...)` composes the head parent pose (measured head angles) with the mount instead of identity; when head joint states are stale it returns None (the view then has no camera pose, never a base_link guess).
- `frontiers.py`: the facing of a front lookup comes from its recorded `camera_pose_base` (already supported); the base-heading fallback stays only for records without one, and its docstring says so (no "the front camera looks along the base heading" claim for new records).
- `approach_tools.py`: `camera_scene` / `camera_view_problem` build the front scene with the current measured head pose (the check assumes the head keeps its pose while the base moves; the docstring says so); stale head joints give the reason "head joint states stale".
- `nodes/mcp_server/README.md`: the Frontiers and camera geometry paragraphs describe the head-mounted front camera.
- Tests first in new `nodes/mcp_server/tests/test_lookup_head_pose.py`, seen failing: a front lookup records the head-composed camera pose (pan +45 deg yields a view yaw of base yaw + 45 deg); stale head joints record no camera pose; frontier facing uses the recorded pose; approach view problem names stale head joints.

#### Acceptance conditions

- a1: front lookups record the head-composed camera pose and none when head joints are stale [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_lookup_head_pose.py -q]
- a2: frontier facing and approach view checks use the head pose [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_lookup_head_pose.py -q]
- a3: the README describes the head-mounted front camera in the Frontiers section [evidence: judgement]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/lookup_tools.py`
- `nodes/mcp_server/mcp_server/frontiers.py`
- `nodes/mcp_server/mcp_server/approach_tools.py`
- `nodes/mcp_server/tests/test_lookup_head_pose.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- `lookup_tools.py:LookupRecorder.camera_pose_base` (or its current owner), `frontiers.py` facing helper, `approach_tools.py:camera_view_problem`.

#### Expected outcome

Recorded views, frontier facing and approach checks use the real head camera pose.

#### Steps

- Write `tests/test_lookup_head_pose.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_lookup_head_pose.py -q`; expect failures.
- Implement; run `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`; commit `feat(mcp_server): lookups use the head camera pose`.

#### Notes

- Persisted lookup records keep their schema (`camera_pose_base`); only the values change.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on mcp-front-camera-on-head, mcp-lease-release-homes-head

#### Ruling

Accepted by the orchestrator: verified (mcp-lookups-follow-head-pose by hand: mcp_server 2620 passed, lint clean; its verifier agent died) and merged into main.

### S20: Head config consistency

- Status: done
- Files: `tests/test_head_config_consistency.py`, `ansible/group_vars/client.yml`, `tests/README.md`, `nodes/README.md`, `MEMORY.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 7bd1de490bfaa44b35198be4028ebc7375786d80

#### Requirement

- New root `tests/test_head_config_consistency.py` (D2, D4, U2, U4), seen failing until the values agree: the home pose is equal (within 1e-3 rad) in the base bus node `base_servos`'s head `startup_position_rad`, mcp_server `head.home_*` (client.yml and the code default), and web_ui `head_home_deg` (client.yml and default.yaml), and lies inside the limits; the ranges agree across the head URDF limits, the bridge `command_min/max_steps` (4096 steps/turn around 2048, honouring `inverted`), the web_ui `head_*_range_deg` and the mcp_server limits (read from the URDF path it is configured with); the joint names agree (URDF, bridge group, mcp_server `head.joint_names`); the topics agree (`/head/joint_states`, `/head/joint_commands` in bridge namespace, mcp_server, web_ui, head_state_publisher launch remap). Fix any disagreement in `ansible/group_vars/client.yml`.
- `nodes/README.md`: head_state_publisher listed; the base bus node `base_servos`'s head group mentioned.
- `MEMORY.md`: a decision entry: head = `head` group on the base bus node `base_servos`, robot_state_publisher on `nodes/web_ui/urdf/head.urdf`, caps written with every goal, home pose (pan 0, tilt +30 deg down) in four places pinned by this test, lease "any" with home on the last release, sign conventions.
- `tests/README.md` lists the new test file and its tests.

#### Acceptance conditions

- a1: home pose, ranges, joint names and topics agree across URDF, bridge, mcp_server, web_ui and the TF node [evidence: test: tests/test_head_config_consistency.py]
- a2: MEMORY.md records the head decisions [evidence: command: grep -q 'head.urdf' MEMORY.md]
- a3: nodes/README.md lists head_state_publisher [evidence: command: grep -q head_state_publisher nodes/README.md]
- a4: root tests, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `tests/test_head_config_consistency.py`
- `ansible/group_vars/client.yml`
- `tests/README.md`
- `nodes/README.md`
- `MEMORY.md`

#### Interfaces

- `tests/test_head_config_consistency.py` (reads client.yml, default.yaml, head.urdf, mcp_server and web_ui code defaults).

#### Expected outcome

One test fails whenever the head home, ranges, names or topics disagree anywhere.

#### Steps

- Write the test; run `uv run pytest tests/test_head_config_consistency.py -q`; expect failures only where values disagree (or for the missing file); fix client.yml; run the verify command; commit `test: head config consistency`.
- Update nodes/README.md and MEMORY.md; commit `docs: head decisions`.

#### Notes

- Import the mcp_server and web_ui defaults by reading their config modules as the existing root tests do (see `tests/test_mcp_server_config.py`).

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on agent-head-tools-and-prompt, head-in-3d-map-view, mcp-lookups-follow-head-pose

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-in-3d-map-view, mcp-lookups-follow-head-pose

### S21: Head suite and robot runbook

- Status: done
- Files: `tests/test_head_live_refs.py`, `tests/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/static_tf_publisher && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`
- Verified head: unknown

#### Requirement

- New `tests/test_head_live_refs.py`, written first and seen failing if any hit remains: `test_no_head_placeholder_text` (`git grep` over tracked files outside `docs/plans/**` finds no `no head mount configured`, no `disabled pan/tilt`, no "future head mount" wording); `test_front_camera_not_pinned_to_base_link` (no `FRONT_CAMERA_PARENT = "base_link"`, no "front camera works in base_link" docstring claim in mcp_server). `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, test-ansible, feetech_servos, static_tf_publisher, mcp_server, claude_agent, web_ui, frontend).
- The staged runbook below is complete; the orchestrator executes it after merge (not the implementer).

Orchestrator runbook (after merge, robot reachable; every deploy through the ansible-deploy skill and `scripts/deploy-nodes.sh`, logged to `.logs/`; least dramatic motion first per CLAUDE.md; stop and ask the user before any step beyond the one described):
1. Hardware gate: ask the user to confirm the head is built and both servos are on the base bus with ID 20 (pan) and 21 (tilt) set with `scripts/set_servo_id.py`, centred so 2048 is straight and level, and the head turned by hand to straight and level before power-on. Until confirmed, do not deploy the base bus node `base_servos` (it blocks at startup on missing servos and the swerve drive would stop).
2. Check no agent session is running (claude_agent state API); if one is, ask before deploying mcp_server/claude_agent.
3. Before the first head deploy, temporarily narrow the head group in client.yml to pan `command_min/max_steps` 1991..2105 (+-5 deg), tilt 2020..2076 (about +-2.5 deg), and remove `startup_position_rad` (the bridge then holds the present position at startup), so the first startup moves nothing and no command can move more than 5 deg; deploy `./scripts/deploy-nodes.sh client base_servos head_state_publisher 2>&1 | tee .logs/deploy-client-head.log`.
4. Verify with topic_scraper_api (enable it in client.yml and deploy it if disabled): `/head/joint_states` publishes real positions near 0; `base_servos` runs at its 50 Hz target (the split epic projects base + head at 14.4 ms vs the 20 ms period) and `/follower/joint_states` stays >= 95 Hz; `/swerve_drive/joint_states` unchanged.
5. First live motion: pan only, +5 deg then -5 deg then 0 through `/head/joint_commands` (frame_id autonomy, via mcp_server `set_head_angles` with the base lease, or the web UI after its deploy); confirm the measured pan follows, at the capped speed, with no oscillation (topic_scraper oscillation rule if configured). Then tilt +-2.5 deg.
6. Restore the full ranges and the home `startup_position_rad` (pan 0, tilt 0.5236) and redeploy; widen in steps (pan +-30, then +-90; tilt -30..+45) only after the user agrees; the startup home moves the head down 30 deg at the capped speed.
7. TF: `ros2 run tf2_ros tf2_echo base_link head_camera_optical_frame` and `... tof_optical_frame` (via an ansible ad-hoc command, no manual SSH) follow pan/tilt; measure the real mount, tilt offset and camera spacing and update head.urdf and cameras.front.mount if they differ.
8. Deploy `web_ui mcp_server claude_agent` (after step 2): Head tab sliders move the head, Home returns it; 3D view shows the head; agent `head_look_at` at a floor point 1 m ahead in base_link centres it in the head camera image (`get_camera_image camera=front`); `release_control(subsystem='all')` reports `reset.head: homed`.
9. Lidar shadow: `/scan` shows no return at the post bearing beyond the laser_filter footprint box; if it does, report with the bearing and range.
10. Disable topic_scraper_api again if it was disabled before. If the robot is offline, report steps 1-10 as pending on the robot with everything else done.

#### Acceptance conditions

- a1: no placeholder head text remains [evidence: test: tests/test_head_live_refs.py::test_no_head_placeholder_text]
- a2: the front camera is no longer pinned to base_link [evidence: test: tests/test_head_live_refs.py::test_front_camera_not_pinned_to_base_link]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/static_tf_publisher && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]
- a4: the runbook stages servo IDs, +-5 deg pan first, tilt, full range, TF, agent, web UI, lease homing and lidar shadow checks [evidence: judgement]

#### Files

- `tests/test_head_live_refs.py`
- `tests/README.md`

#### Interfaces

- `tests/test_head_live_refs.py` (`test_no_head_placeholder_text`, `test_front_camera_not_pinned_to_base_link`).

#### Expected outcome

All suites green with the live-refs check; the orchestrator brings the head live in stages with the runbook in the requirement.

#### Steps

- Write `tests/test_head_live_refs.py`; run `uv run pytest tests/test_head_live_refs.py -q`; fix any real hit.
- Run the full verify command and quote each summary line; commit `test: no head placeholder or base_link front camera remains`.
- The runbook is executed by the orchestrator after merge.

#### Notes

- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- Servo safety (CLAUDE.md): +-5 deg pan first; ask before widening.

- 2026-10-18 run pan-tilt-head-control-with-a: dependency-failed on head-config-consistency

#### Ruling

Accepted by the orchestrator: verified by hand on the committed HEAD and merged into main (full suite green: root 1417, mcp_server 2678, frontend 1017); escalations were verifier/process gates. head-tab-pan-tilt-controls stale-command finding fixed test-first (609ed96f).

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
