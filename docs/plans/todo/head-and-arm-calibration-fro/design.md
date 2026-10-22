# Head and arm calibration from ChArUco board and gripper plate design

## Summary

Re-planned 2026-10-22. A "Head from arm" guided step on the web UI Calibrate page calibrates the pan/tilt head chain
against the follower arm, which the user trusts as calibrated. The existing 6x5 ChArUco board is clipped to the fixed
jaw; the head_rgbd_calibration node moves the arm through a scripted pose set (wrist roll, then wrist flex, then elbow
and shoulder only after a confirm), every pose and transit FK-checked against the floor, the robot body and the head,
and points the head at the board from a few pan/tilt offsets per arm pose. One robust least-squares fit solves mount
x/y/z, mount roll/pitch/yaw (the pan zero folded into the yaw), tilt zero and tilt roll, with the applied floor-fit
values as start values and cross-check, sigma, held-out views and limits that refuse Apply. Apply reuses the floor
fit's `applied/head_chain.json` path; mcp_server HeadKinematics and the front camera (reloaded without a restart) and
the web UI head URDF read it. The arm joint offset solve moved to epic arm-joint-offsets-from-the-g.

## Intent and why

The floor-plane fit (epic head-chain-floor-plane-fit-a, applied 2026-10-22 result r-20261022-065211: mount z 0.246 m,
tilt zero -6.50 deg, mount pitch 1.50 deg, mount roll -0.32 deg, tilt roll 0.92 deg; runbook
`docs/runbooks/head-floor-fit.md`) fixed the floor for ground estimation, but a plane is invariant to head yaw, mount
x/y and the pan zero, so `head_look_at`, grasping from head images and the 3D head view still use the CAD values
(`head_mount_joint` x 0.15, y 0.10, yaw 0). The user (2026-10-22): "calibrate head position/angles based on the arm
position/angles (it should be well calibrated now)". The arm is the yaw and x/y reference: its base pose
(`arm.base_in_base_link`) and joint offsets (`arm.joint_offsets_rad`, `client.yml` mcp_server) stay as they are.

## Goals

- G1: A guided "Head from arm" section on the Calibrate page runs base pose (confirm), board clip, tiers, Compute,
  result (applied, fitted, delta, sigma, floor cross-check, held-out numbers, failed limits) and Apply.
- G2: The fit recovers mount x/y/z, roll/pitch/yaw, tilt zero and tilt roll from synthetic scenes (0.3 px noise) within
  1.5 mm / 0.15 deg and refuses results over the RMS, held-out, sigma, coverage and floor cross-check limits.
- G3: No arm motion is commanded before its pose and transit pass the FK check (tool, board and arm links clear of the
  floor, the body and the head; roll rules; stretched-arm rule), and every arm motion goes through mcp_server.
- G4: Apply writes `applied/head_chain.json`; head_state_publisher (TF), mcp_server HeadKinematics and front camera,
  and the web UI head URDF use it; mcp_server without a restart.
- G5: The head tilt range is widened to the reprinted mount's recorded limit everywhere it is pinned.
- G6: Ansible in sync, READMEs and tests/README.md, and a robot runbook with real numbers.

## Non-goals

- Fitting arm joint offsets, link lengths, the arm base pose, or the tool point (epic arm-joint-offsets-from-the-g
  holds the moved offset solve).
- The camera intrinsics, the ToF-to-camera extrinsic and the head_camera_joint lever (applied RGB-D calibration), and
  the 0.04 m tilt offset (degenerate with mount z).
- Driving the base (the robot is charging; the old drive-straight yaw check is struck).
- The head as an obstacle in mcp_server's own self-collision body model (noted, not doing now).

## Anti-goals

- An arm motion with the board attached that was not FK-checked, or a parking move with the board on the gripper.
- Any elbow_flex, shoulder_lift or shoulder_pan motion without the user's OK in the UI.
- Placeholder data: a view without a fresh joint sample for all seven joints or without a board detection is not
  stored; nothing is zero-filled.
- An applied result that failed a limit, or a consumer that silently ignores the applied chain.
- Restarting mcp_server on apply (it kills a running agent session, MEMORY), rclpy probes on the client (MEMORY).

## Constraints

- House rules (CLAUDE.md): Python with type hints, pydantic config with every key in the shipped YAML and the
  client.yml block (`tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults` pins them
  equal, so every task that adds node config keys edits `ansible/group_vars/client.yml` too), constants at the top,
  test-first, ROS2 logging in nodes, servo safety (least dramatic joints first, ask before big joints), READMEs and
  tests/README.md, deploy only through `scripts/deploy-nodes.sh` with output in `.logs/`.
- Python stays in web_ui, mcp_server and the existing Python calibration node and core (user decision).
- Arm safety (MEMORY feedback_arm_motion_safety): wrist roll only with the gripper at most half open and the arm lifted
  and forward; negative elbow_flex stretches the arm; a stretched arm with shoulder_lift past about 1.85 rad stalls.
- Head camera mapping stream 576x324; intrinsics valid with the focus lock, which is held while the mapping stream has a
  subscriber (`capture.py`).
- Current head range pan -90..90 deg, tilt -30..+30 deg (+ is down, `head.urdf`); the pose set must work inside it.
- Arm FK: `urdf_angle = measured_angle + offset` (client.yml comment on `joint_offsets_rad`); the arm URDF root is
  joined to the robot base_link by `arm.base_in_base_link` {x 0.0592, y -0.05, z 0.100, yaw 0}.
- `ansible/group_vars/client.yml` is also edited by open tasks of the in-progress epic ground-detection-robustness, so
  that epic is this one's blocker (ticket.py orders cross-epic file overlaps through blockers).
- Tasks about an hour, at most 8 files each.

## Decisions

### D1: Calibrate the head chain against the trusted arm with the board on the gripper

The 6x5 board clipped to the fixed jaw is moved by the arm; the head camera sees it over many arm poses and head
angles. Arm FK (so101 URDF, measured joints plus `joint_offsets_rad`, `base_in_base_link`) gives the board in base_link
up to the board's fixed pose in `gripper_link`, which is fitted (6 DOF). The arm is not fitted. Alternatives: a floor
board at several spots (cannot see yaw/x/y against the board's own pose; the old plan), a joint head+arm fit (moved to
the follow-up). User decision 2026-10-22. Serves H1.

### D2: Free parameters and the degeneracies fixed

Free: `head_mount_joint` x, y, z, roll, pitch, yaw; `head_tilt` origin pitch (tilt zero) and roll (tilt roll); the
board pose in `gripper_link`. Names `mount_z_m`, `mount_roll_deg`, `mount_pitch_deg`, `tilt_zero_deg`,
`tilt_roll_deg` equal `floor_fit.PARAM_NAMES`; new `mount_x_m`, `mount_y_m`, `mount_yaw_deg`. Fixed at the applied
chain: `head_tilt` xyz (the 0.04 m offset is degenerate with mount z), `head_pan` origin (D3), `head_camera_joint` and
the optical joints (RGB-D calibration; the camera pitch is degenerate with the tilt zero), intrinsics; the whole arm.
(my call for the camera lever: not in the user's list.) Serves H1.

### D3: The pan zero is folded into the mount yaw

A pan zero offset and the mount yaw both rotate about the pan axis; with the mount tilted 1.5 deg they differ only to
second order (about 0.04 deg per deg), so they are one parameter. It is written as `head_mount_joint` rpy yaw in
`head_chain.json` (format unchanged): TF and look_at are exact, and a raw pan command of 0 in the Head tab points off
by the fitted yaw. Alternative: a servo pan zero in base_servos (re-zeroes the servo, needs a bridge redeploy and moves
every recorded pan angle). Open question Q1. Serves H1.

### D4: Floor-fit values as start values and cross-check

The fit starts from the applied chain (the floor-fit z, roll, pitch, tilt zero and tilt roll). For those five,
`floor_delta` = fitted minus the applied floor result; over `max_floor_delta_mm` 10 / `max_floor_delta_deg` 1.5 the
result fails. x, y and yaw get sanity limits against their start (`max_start_delta_mm` 30, `max_start_delta_deg` 5). An
optional prior (`floor_prior_weight`, default 0) adds one residual per floor parameter; off by default because the
floor fit carries the garage floor slope (normal RMS 0.83 deg). The floor values come from the robot's
`applied/applied.json` `head_chain` record: today's `apply_floor` (`floor_results.py`) already writes `{session,
result, applied_at}` there, so the 2026-10-22 apply recorded session floor-20261022-064337, result r-20261022-065211;
a record without the new `floor` key whose session matches `FLOOR_SESSION_ID_RE` is that floor result itself, and its
`floor_fit/<sid>/results/<rid>/result.json` `params` are the cross-check values. From now on both applies write
`source` and `floor`. Open question Q2. Serves H1, H3.

### D5: One robust least-squares fit with sigma, held-out arm poses and limits

Per view `cv2.solvePnP` seeds the board mount; one `scipy.optimize.least_squares` (Huber, `huber_f_scale_px`) over the
ChArUco corner reprojection of all fit views; sigma from `(J^T J)^-1 * s^2`; the largest parameter correlation is
reported. Every `holdout_every`-th arm pose is held out with all its views. Limits: `rms_px` 1.0, `holdout_rms_px`
1.5, `holdout_board_mm` 5.0 (board centre from the head vs from arm FK on held-out views), `max_sigma_mm` 2.0,
`max_sigma_deg` 0.2, `min_views` 40, `min_arm_poses` 16, `min_pan_span_deg` 30, `min_tilt_span_deg` 10, plus D4. A
result with any failed limit has `passed: false` and cannot be applied. Serves H3.

### D6: Extend the head_rgbd_calibration node, reuse the floor-fit machinery

Sessions gain kind `head_arm` in the existing store (RGB still poses with seven measured joints); compute runs in the
HTTP handler like `compute_floor` and writes `result.json`, `report.md` and, when passed, `head_chain.json`; apply goes
through one writer shared with `apply_floor` (copy into `applied/`, `head_chain` record in `applied.json`, the apply
path unit restarts head_state_publisher and the head consumers). Routes under `/api/head-arm/` beside `/api/floor/`.
Python because it extends an existing Python node (user decision). Serves H1, H4, H9.

### D7: Robot motion through mcp_server tools, head aimed by the node

An MCP client (`robot_client.py`, modelled on `nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool`, token
`/etc/ros2/mcp_server/token`, group `mcp-token`) calls `take_control`, `move_arm_joints`, `set_head_angles`,
`release_control`, `get_arm_state`, so the leases, the floor guard, the body self-collision check, the roll guard and
the battery gate apply. The node aims the head itself: nominal board centre from arm FK, pan/tilt solved on the applied
head chain (`aim_head`), plus the configured `head_offsets_deg` per arm pose so the board moves across the image and the
pan/tilt span grows; offsets outside the URDF limits are skipped. Serves H2.

### D8: Arm safety: confirms, tiers, checked poses, holding exits

Order: "Move to base pose" (confirm naming the joints that move; no board attached) -> the user clips the board and
presses "Board attached" -> tier 0 (wrist_roll only) -> tier 1 (wrist_flex and wrist_roll) -> a confirm naming
elbow_flex, shoulder_lift, shoulder_pan -> tier 2 -> back to the base pose -> hold. Declining the confirm skips tier 2.
The gripper target is sent once with the base pose (`gripper_rad`, at most the half-open `max_roll_gripper_rad`) and
never again. Before every command the path from the measured joints to the target is FK-checked (D9); a refusal stops
before anything moves. Every exit (end, stop, tool error, exception, confirm timeout) releases with
`keep_position: true`: the arm holds a checked pose and mcp_server keeps the head where it is (`reset.head: kept`,
`control.py::release_head`). "Park" (`release_control` with park) is a separate confirmed action after the user
removed the board; it is refused while `get_control_state` reports `arm.carrying`, and its release homes the head after
the arm (the board stays outside the head envelope anyway). The board is never a carried object: `move_arm_joints`
writes no carry record, the sequence never sends `carried_object_radius_m` (the keep release publishes the frozen
measured-arm footprint without a carried circle; the robot does not drive), and `gripper_rad` leaves the jaws without a
holding load on the clip. `move_arm_joints` targets are measured follower angles (`arm.py::move_joints`: "measured
(follower) joint space, as reported by get_arm_state"; `urdf = measured + offset`). speed_scale <= 0.3. User decision 2026-10-22 (supersedes the old gate Q4: elbow moves now need the confirm).
Open question Q3. Serves H2.

### D9: Pose check in the shared core

`arm_pose_check.py` checks a pose and a joint-linear path (samples every `path_step_rad`): tool point >= 0.10 m and
board outline points >= 0.05 m above the floor; board points and arm link capsules (upper arm, forearm, wrist) at least
`margin_m` from the body obstacles (a copy of mcp_server's `body` boxes and cylinders, pinned equal by the deploy test)
and from the head envelope (a vertical cylinder about the pan axis of the given head URDF containing the head at every
pan/tilt inside that URDF's joint limits, so it covers a widened tilt range too; pinned against the URDF visuals); board points clear of the upper arm and forearm; a wrist_roll change only
with the gripper at most half open and the tool >= 0.15 m high; shoulder_lift over 1.85 rad with a negative elbow_flex
refused; joints inside the arm URDF limits. mcp_server keeps its own checks on top. Serves H2.

### D10: Every head-chain consumer reads the applied override

head_state_publisher already applies it (delivered by the floor epic). mcp_server rebuilds HeadKinematics when
`applied/applied.json` changes (mtime checked per call), the front camera sits on `head_camera_link` with the applied
intrinsics, and web_ui serves the head URDF with the overrides applied per request. Serves H4.

### D11: One guided "Head from arm" section on the Calibrate page

Built like HeadFloorFit (MUI, step cards, polling). It replaces nothing; the "Arm calibration - deferred" line stays
for the follow-up epic. Serves H5.

### D12: Tilt range widened to the reprinted mount's recorded limit

Default -30..+60 deg (+ is down); when the hardware step records a smaller tilt-down limit, that value. The pose set does
not depend on it (it works within -30..+30). Kept from the old gate Q1. Serves H6.

### D13: Synthetic scenes first, robot runbook last

Each fit and check is written test-first against synthetic scenes (corners projected through a known perturbed chain,
0.3 px noise); the shipped pose set is pinned by a test that also fits a synthetic scene generated on it. The epic ends
with a runbook executed with the user on the robot. Serves H7.

### D14: The arm joint offset solve moves to a follow-up epic

arm-offsets-plate-solver, arm-offsets-compute-apply and arm-calibrate-ui-section moved verbatim to epic
arm-joint-offsets-from-the-g (blocked by this epic), to be re-planned there against this epic's parts. User decision
2026-10-22. Serves H8.

### D15: The existing 6x5 board is the gripper board

`models/charuco-6x5-40mm` (`scripts/markers_generate.yaml` `charuco:`, ids 0-14) on a clip on the fixed jaw; no new
plate. Kept from the old gate Q2. Serves H2.

### D17: The board mount is fitted; the measurement is only a start value

Standard hand-eye: the target-in-gripper pose is unknown and fitted (6 DOF, D2), so neither the solver nor the pose
set waits for the hardware step. The pose set ships with a nominal mount from the jaw geometry, checked with an extra
`provisional_margin_m` and a +-1 cm / +-5 deg mount band; the user's measurement replaces the nominal start value in
calib-robot-runbook, and the poses stay provisional until a dry run without the board there. Open question Q4 ("no"
puts arm-board-pose-set back behind board-on-gripper-hardware-st). (my call pending Q4) Serves H2.

### D16: Guided step-by-step process

Step cards for the physical steps (clip the board, remove it), the confirms and the result; automation only where it
cuts work without adding risk (scripted arm poses and head offsets). Kept from the old gate. Serves H5.

### D18: User 2026-10-22 approved (yes): fold pan zero into the head_
User 2026-10-22 approved (yes): fold pan zero into the head_mount_joint yaw (user decision)

### D19: User 2026-10-22 approved (yes): floor-fit values as start va
User 2026-10-22 approved (yes): floor-fit values as start values and pass/fail cross-check, weighted prior off (user decision)

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | ran (this re-plan read the delivered code: `shared/ros2_common/head_rgbd_calibration/{chain,floor_fit,urdf_text,charuco,config}.py`, `nodes/head_rgbd_calibration/head_rgbd_calibration/{floor_sweep,floor_results,floor_api,capture,store,__main__}.py`, `nodes/mcp_server/mcp_server/{head_kinematics,self_collision,body_model,tools}.py`, `nodes/web_ui/frontend/src/calibrate/`) | reuse `UrdfChain`, `arm_in_robot_base`, `head_chain_json`, `apply_overrides`, `parse_head_chain`, floor `apply_floor` writer, `base_urdf`, `rgb_qos`, `write_json_atomic`; mcp `call_robot_tool` pattern | a new node; mcp_server's C++ self-collision kernel from the node (not importable there) |
| research-code-external | skipped: ChArUco, solvePnP and least_squares are already used in the core | - | - |
| research-papers | skipped: hand-eye / kinematic calibration by bundle adjustment over a trusted arm is standard | - | - |
| research-docs | skipped: no new external API beyond the mcp SDK client claude_agent already uses (mcp==2.3.0) | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief 1 (dispatcher) | floor fit delivered: override format, FK core, head_state_publisher chain, floor sweep/compute/apply | rejected: delivered (urdf-chain-fk-core, head-chain-override-format, head-state-publisher-chain dropped) |
| brief 1 | reuse the floor-fit session/compute/apply machinery | head-arm-compute-and-apply, head-arm-sequence-http-api |
| brief 1 | mcp_server HeadKinematics, web_ui head URDF and front camera still ignore the override | mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| brief 2 | the floor cannot observe yaw, mount x/y, pan zero | head-from-arm-board-solver |
| user decision 2026-10-22 | arm trusted; fit the head chain from the board on the gripper; floor values as prior/cross-check | head-from-arm-board-solver, head-arm-compute-and-apply |
| user decision 2026-10-22 | arm offset solve to a follow-up epic | calib-docs-and-test-index (D14; the move itself done by ticket.py) |
| user decision (safety) | least dramatic joints first, UI confirm for elbow/shoulder, FK check vs body and head, roll rules | arm-pose-safety-check, arm-board-pose-set, head-arm-pose-capture-sequen, head-arm-calibrate-ui-sectio |
| user decision | held-out limits before Apply; applied through the override to every consumer | head-from-arm-board-solver, head-arm-compute-and-apply, mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| code: `nodes/mcp_server/mcp_server/body_model.py`, client.yml mcp_server `body` | mcp's body model has no head and does not know the board | arm-pose-safety-check |
| code: client.yml `test_ros2_nodes_entry_matches_defaults` | node config keys must land in client.yml in the same task | joint-state-still-capture, robot-mcp-client, arm-board-pose-set |
| reuse | `nodes/claude_agent/claude_agent/robot_stop.py:call_robot_tool` | robot-mcp-client |
| reuse | `nodes/head_rgbd_calibration/head_rgbd_calibration/floor_results.py:apply_floor` | head-arm-compute-and-apply |
| reuse | `shared/ros2_common/head_rgbd_calibration/chain.py:UrdfChain` | head-from-arm-board-solver, arm-pose-safety-check |
| reuse | `shared/ros2_common/head_rgbd_calibration/charuco.py:detect` | joint-state-still-capture |
| old plan | floor-board head grid, head_chain sessions, tof_joint recompute | rejected: the floor fit replaced the floor board; the camera joint is fixed so tof_joint is unchanged |
| old plan | drive-straight yaw check | rejected: the robot is charging, no driving (user) |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Calibrate the head chain against the trusted arm | head-from-arm-board-solver, head-arm-compute-and-apply |
| D2 Free parameters and the degeneracies fixed | calib-config-and-view-types, head-from-arm-board-solver |
| D3 The pan zero is folded into the mount yaw | head-from-arm-board-solver |
| D4 Floor-fit values as start values and cross-check | head-from-arm-board-solver, head-arm-compute-and-apply |
| D5 One robust least-squares fit with sigma, held-out arm poses and limits | calib-config-and-view-types, head-from-arm-board-solver |
| D6 Extend the node, reuse the floor-fit machinery | session-kinds-joint-poses, joint-state-still-capture, head-arm-compute-and-apply, head-arm-sequence-http-api |
| D7 Robot motion through mcp_server tools, head aimed by the node | robot-mcp-client, head-arm-pose-capture-sequen, calib-ansible-deploy-config |
| D8 Arm safety: confirms, tiers, checked poses, holding exits | arm-board-pose-set, head-arm-pose-capture-sequen, head-arm-sequence-http-api, head-arm-calibrate-ui-sectio |
| D9 Pose check in the shared core | arm-pose-safety-check, arm-board-pose-set, calib-ansible-deploy-config |
| D10 Every head-chain consumer reads the applied override | mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| D11 One guided Head from arm section | calib-frontend-api, head-arm-calibrate-ui-sectio |
| D12 Tilt range widened | board-on-gripper-hardware-st, head-tilt-range-widened |
| D13 Synthetic scenes first, robot runbook last | head-from-arm-board-solver, arm-board-pose-set, calib-docs-and-test-index, calib-robot-runbook |
| D14 The arm joint offset solve moves to a follow-up epic | calib-docs-and-test-index |
| D15 The existing 6x5 board is the gripper board | calib-config-and-view-types, board-on-gripper-hardware-st |
| D16 Guided step-by-step process | head-arm-calibrate-ui-sectio, head-arm-pose-capture-sequen |
| D17 The board mount is fitted; the measurement is only a start value | head-from-arm-board-solver, arm-board-pose-set, calib-robot-runbook |
| D18 User 2026-10-22 approved (yes): fold pan zero into the head_ | head-from-arm-board-solver, head-arm-compute-and-apply |
| D19 User 2026-10-22 approved (yes): floor-fit values as start va | head-from-arm-board-solver, head-arm-compute-and-apply |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `head-arm-calib-config` | calib-config-and-view-types | head-from-arm-board-solver, arm-pose-safety-check, joint-state-still-capture | `shared/ros2_common/head_rgbd_calibration/config.py` |
| `head-arm-view-types` | calib-config-and-view-types | head-from-arm-board-solver, session-kinds-joint-poses | `shared/ros2_common/head_rgbd_calibration/views.py` |
| `head-arm-fit` | head-from-arm-board-solver | head-arm-compute-and-apply, arm-board-pose-set | `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py` |
| `head-aim` | head-from-arm-board-solver | arm-board-pose-set, head-arm-pose-capture-sequen | `shared/ros2_common/head_rgbd_calibration/head_arm_fit.py` |
| `head-arm-synthetic-scenes` | head-from-arm-board-solver | head-arm-compute-and-apply, arm-board-pose-set | `tests/head_arm_synthetic.py` |
| `arm-pose-check` | arm-pose-safety-check | arm-board-pose-set, head-arm-pose-capture-sequen | `shared/ros2_common/head_rgbd_calibration/arm_pose_check.py` |
| `calib-session-kinds` | session-kinds-joint-poses | joint-state-still-capture, head-arm-compute-and-apply | `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py` |
| `calib-still-capture` | joint-state-still-capture | head-arm-pose-capture-sequen | `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py` |
| `calib-robot-client` | robot-mcp-client | head-arm-pose-capture-sequen, head-arm-sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py` |
| `arm-sequence-config` | arm-board-pose-set | head-arm-pose-capture-sequen, calib-ansible-deploy-config | `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` |
| `calib-sequencer` | head-arm-pose-capture-sequen | head-arm-sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py` |
| `calib-head-arm-results` | head-arm-compute-and-apply | head-arm-sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/head_arm_results.py` |
| `head-arm-calib-api` | head-arm-sequence-http-api | calib-frontend-api, head-arm-calibrate-ui-sectio | `nodes/head_rgbd_calibration/tests/fixtures/api/` |

Cross-epic (`docs/plans/contracts.json`): this epic uses `head-chain-override` (`urdf_text.py`), `urdf-chain-fk`
(`chain.py`) and `head-chain-applied-record` (`floor_results.py`; the record gains `source` and `floor`, compatible)
from head-chain-floor-plane-fit-a, and `head-rgbd-applied-calibration` (the `applied/` directory and its intrinsics).
It owns `head-arm-fit`, `head-arm-calib-api`, `calib-session-kinds` and `arm-pose-check` for the follow-up epic
arm-joint-offsets-from-the-g.

## Review focus

- The solver: the D2 parameter set (a free degenerate parameter makes sigma explode; a fixed one that should be free
  biases the result), the pan-zero folding (D3), held-out split by arm pose, sigma scaling, the floor cross-check.
- Frames: URDF rpy is fixed-axis xyz (`urdf_handoff.RPY_SEQUENCE`), head_tilt + is down, optical frames z forward; arm
  URDF root vs robot base_link via `base_in_base_link`; `urdf = measured + offset`; the pose set's joint convention
  equals what `move_arm_joints` takes.
- Arm safety: no command before its path check, the gripper sent once, the confirms only from the UI dialogs, every exit
  holds (keep_position true), Park only after the board-removed confirm, the head envelope really contains the head.
- Every consumer reading `head_chain.json`, and an unusable override falling back to the URDF with a warning.
- No placeholder data: a missing joint sample or detection means no view, never zeros.

## Left to the builder

- The exact base pose, pose values and head offsets of the shipped pose set, within D8/D9 and the tests that pin
  clearance, visibility, coverage and a passing synthetic fit.
- The internal parameter vector layout, `x_scale`, the Huber `f_scale` default (1 px) and `max_nfev`.
- UI layout inside the section, following HeadFloorFit.

## Assumptions and unknowns

- `move_arm_joints` targets use the `get_arm_state` convention; whether that is measured or URDF angles is probed by
  arm-board-pose-set in `nodes/mcp_server/mcp_server/arm.py` and pinned by a test.
- The board on the jaw decodes at 0.25-0.65 m on the 576 px stream and the arm carries it at speed_scale <= 0.3: checked
  in the runbook with tier 0 first.
- The node's service user can read `/etc/ros2/mcp_server/token` with group `mcp-token`: checked in
  calib-ansible-deploy-config and on the robot.
- Arm moves while docked and charging pass the mcp battery gate: checked in the runbook; a refusal is reported, not
  worked around.
- The front camera stream has the same aspect ratio as the mapping stream: probed in mcp-front-camera-applied.

## Open questions

- Q1: Fold the pan zero into the head_mount_joint yaw in head_chain.json (TF and head_look_at exact; a raw pan 0 in the Head tab stays off by the fitted yaw, expected under 3 deg) instead of re-zeroing the pan servo? [recommended: yes]
- Q2: Use the floor-fit values as start values and a pass/fail cross-check (10 mm on mount z, 1.5 deg on roll, pitch, tilt zero, tilt roll) with the weighted prior off by default, since the floor fit carries the garage floor slope? [recommended: yes]
- Q3: Clip the board only after the arm holds the base pose, hold the arm in place on every stop or error, and park only after you confirm the board is removed (no automatic park with the board on)? [recommended: yes]
- Q4: Let the solver fit the board-on-jaw pose (standard hand-eye: target-in-gripper unknown) with your measurement only as a start value, so the pose set and the solver do not wait for the hardware step and the poses stay provisional until a dry run without the board? (no: arm-board-pose-set waits for the hardware step) [recommended: yes]

## Irreversible operations in scope

none. Apply writes `applied/` files that the next apply or a re-seed replaces; a previous chain is kept in its result
directory.
