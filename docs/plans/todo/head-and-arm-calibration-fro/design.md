# Head and arm calibration from ChArUco board and gripper plate design

## Summary

Two buttons on the web UI Calibrate page measure the head kinematic chain and the follower arm joint offsets with the
head RGB camera. "Head calibrate" drives the pan/tilt head over a grid while a ChArUco board lies on the floor at 3-4
spots; one least-squares fit gives the pan-axis height, mount roll/pitch, tilt zero and the camera lever, applied as a
`head_chain.json` override that head_state_publisher, mcp_server (HeadKinematics and the front camera) and the web_ui
URDF view all read. "Arm calibrate" then moves the arm (small joints first, shoulder joints only after the user's OK)
with a small ChArUco plate on the fixed jaw in view of the head; a joint fit gives the shoulder_lift, elbow_flex,
wrist_flex and wrist_roll offsets plus the head yaw and x/y relative to the arm. The arm offsets are a
`joint_offsets_rad` snippet for review and commit; the head part is applied. Everything extends the existing Python
`head_rgbd_calibration` node and `shared/ros2_common/head_rgbd_calibration` core, and runs after the head mount
reprint (>= 60 deg tilt-down).

## Intent and why

The head mount (`head_mount_joint` 0.15 0.10 0.31), the 0.04 m tilt offset and the camera origins in
`nodes/web_ui/urdf/head.urdf` are CAD estimates, and the arm offsets in `client.yml` (`joint_offsets_rad`, fitted
2026-10-10 from a hand-laid grid sheet) could not be re-solved from the gripper camera (floor height, mount z, focal
length and offsets trade off, `docs/calibration/2026-10-10/README.md`). Grasping from head images and the RGB-D
mapping both need the head-to-arm geometry measured, and the user wants it done from the web UI with a board on the
ground, not with SSH and scripts.

## Goals

- G1: "Head calibrate" on the Calibrate page captures still frames of the floor board at 3-4 spots over a pan/tilt
  grid (head moved by the node), fits the head chain and shows RMS, held-out RMS, per-parameter sigma and pass/fail.
- G2: Applying a passing head result writes `applied/head_chain.json` and a recomputed `applied/tof_joint.json`;
  head_state_publisher (TF), mcp_server HeadKinematics and front camera, and the web_ui head URDF use it with no
  redeploy.
- G3: "Arm calibrate" (enabled once a head result is applied) moves the arm through a scripted pose set with the
  plate in view, small joints first, shoulder joints only after an explicit OK, tool point >= 10 cm above the floor;
  it fits shoulder_lift, elbow_flex, wrist_flex and wrist_roll offsets and head x/y/yaw, with sigma and a held-out
  tool-point error.
- G4: The arm result gives a `joint_offsets_rad` snippet (shoulder_pan unchanged) to copy or download for review and
  commit; applying it applies only its head chain (with x/y/yaw).
- G5: Results over any limit (RMS, sigma, held-out error, view counts) are refused for apply.
- G6: A second ChArUco plate with marker ids distinct from the 6x5 board comes out of the marker generator.
- G7: The head tilt range is widened to the reprinted mount's range in every place that pins it.
- G8: Synthetic-scene tests recover known parameters; READMEs, tests/README.md and a robot runbook with results.

## Non-goals

- Fitting link lengths (arm or head).
- Gripper-camera based arm calibration, and re-solving the gripper camera mount after new offsets (the runbook notes
  that the mount needs a re-solve).
- Leader arm calibration.
- A runtime override of `joint_offsets_rad` (the reach_map.npz cache key and the other mcp consumers read the
  committed config).
- Changing the ToF-to-RGB extrinsic or the camera intrinsics (they come from the applied RGB-D calibration).

## Anti-goals

- Any arm motion that is not through mcp_server's guarded tools, or a shoulder move without the user's OK in the UI.
- Placeholder data: no zero-filled joint angles, no detections or poses invented when a frame or joint sample is
  missing; a missing value is absent and the pose is not stored.
- An applied override that a consumer silently ignores (each consumer has a test reading it).
- Restarting mcp_server on apply (it kills a running agent session, MEMORY).
- A head result applied with mount x/y/yaw that came from nowhere: before an arm result they stay at the URDF values.

## Constraints

- House rules (CLAUDE.md): Python with type hints, pydantic config, constants at the top, test-first, ROS2 logging in
  nodes, no placeholder data on topics, servo safety (least dramatic joints first, ask before big joints), Ansible in
  sync, READMEs and tests/README.md updated, deploy only through `scripts/deploy-nodes.sh` with output in `.logs/`.
- The node is Python because it extends the existing Python `head_rgbd_calibration` node and shared core
  (MEMORY: C++ only for new non-web/mcp nodes; this is not a new node).
- Head camera: mapping stream 576x324, intrinsics valid at lens_position 1.0 with the focus lock engaged only while
  the mapping stream has a subscriber (`client.yml` head_camera_cpp, `capture.py`); the calibrator holds that
  subscription for the whole capture.
- Visibility: at the current +30 deg tilt-down the nearest floor seen is about 0.46 m away and the arm reaches
  x <= 0.35 m, so nothing works before the reprint allows >= 60 deg down.
- Floor = base_link z 0 (`robot.urdf`); the board top is at z 0.004 (4 mm board, `scripts/markers_generate.yaml`).
- Gauge fixed: `arm.base_in_base_link` {x 0.0592, y -0.05, z 0.100, yaw 0} and `joint_offsets_rad.shoulder_pan`
  0.0182 (`client.yml` mcp_server arm block).
- Files per task <= 8; tasks about an hour.

## Decisions

### D1: Floor board for the head, gripper plate for the arm, the arm as yaw reference

The head chain is fitted from a 6x5 ChArUco board flat on the floor; the arm offsets and the head x/y/yaw from a
small ChArUco plate on the fixed jaw seen by the head. The arm defines yaw and x/y: `base_in_base_link` and the
shoulder_pan offset are fixed (the gauge). Alternatives: tip touches (3 DOF per touch, coplanar, wrist_roll
unidentifiable, big joints near the floor, the head cannot see them: used only as held-out validation in the
runbook); the gripper camera (tried twice, degenerate, rejected); drive-straight or turn-in-place for yaw (only as
good as the steering zero; kept as a runbook check). User decision D-B1. Serves A1, A2.

### D2: Extend the head_rgbd_calibration node and the shared core, in Python

Sessions gain a `kind` (`rgbd`, `head_chain`, `arm`); the store, capture, compute, apply, jobs and HTTP API are
extended; the fits live in `shared/ros2_common/head_rgbd_calibration` next to the RGB-D maths. Python because it
extends an existing Python node. Alternative: a new node (duplicates the session store, thumbnails, jobs, apply path
and proxy). (my call) Serves A1, A2.

### D3: Head parameter set and the degeneracies fixed

Free in the head fit: `head_mount_joint` z, roll, pitch; `head_tilt` origin pitch (tilt zero) and roll (pan/tilt
skew, weak); `head_camera_joint` x, z, roll, yaw; per board spot x, y, yaw (board z fixed at 0.004, flat). Fixed:
mount x, y, yaw (URDF values until an arm result; pan zero is the mount yaw and is not observable against the board
yaw), the 0.04 m tilt offset (degenerate with mount z), camera pitch in head_link (degenerate with tilt zero), camera
y (degenerate with tilt-axis y), intrinsics (applied RGB-D calibration). Serves A1.

### D4: Plate rotation fixed to the clip's CAD pose

A constant wrist_roll offset is exactly absorbed by a free plate rotation in `gripper_link` (the plate rides on the
roll joint's child), the same reason the 2026-10-10 solve could not identify wrist_roll. So the plate sits on a keyed
clip whose plate rotation in `gripper_link` is a configured CAD value (`plate_mount.rpy`), and only the plate
translation is free. wrist_flex, elbow_flex and shoulder_lift are identifiable because wrist_roll varies across the
pose set. When no CAD rotation is configured, wrist_roll is not fitted and keeps its configured value, and the report
says so. (my call) Serves A2.

### D5: Output forms

Head: an applied `head_chain.json` override (`{joints: {head_mount_joint, head_tilt, head_camera_joint: {xyz, rpy}},
note, session, result}`) beside `tof_joint.json` in `applied/`, read by head_state_publisher, mcp_server
HeadKinematics and front camera, and web_ui `/api/urdf/<head_urdf>`; `tof_joint.json` is recomputed from the applied
ToF extrinsic on the new chain. Arm: a `joint_offsets_rad` YAML snippet for review and commit (not a runtime override:
a runtime override would desync the reach_map.npz guard and every other consumer of the committed config). The arm
result also carries a full head chain (x/y/yaw now measured), applied the same way. User decision (my call in the
request). Serves A1, A2.

### D6: Robot motion through mcp_server tools

The node moves the head (`set_head_angles`, `head_look_at`, `head_home`) and the arm (`take_control`,
`move_arm_joints`, `release_control`) as an MCP client of mcp_server (`http://127.0.0.1:18200/mcp`, bearer token
`/etc/ros2/mcp_server/token`), modelled on `nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool`. Leases,
self-collision, floor guard, battery gate and the web UI Home override all apply. Alternatives: publishing joint
commands directly (bypasses every guard), or driving the sequence from the browser (dies with the tab). (my call)
Serves A1, A2.

### D7: Arm safety tiers

The pose set has a small tier (only wrist_flex, wrist_roll and elbow_flex differ from a base pose, each within
`max_small_delta_rad`) and a big tier (shoulder_lift and shoulder_pan move). An arm run starts with the small tier;
the big tier runs only when the request carries `allow_big_joints: true`, which the UI sends only after a confirm
dialog naming the joints. Every pose's nominal tool point is checked >= 0.10 m above the floor before it is
commanded; speed_scale <= 0.3; the lease is always released (park home) at the end, on stop and on error. User
decision D-B1. Serves A2.

### D8: One robust least-squares fit with sigma, held-out views and limits

Per view solvePnP gives the initial board pose; one `scipy.optimize.least_squares` (Huber loss) on ChArUco corner
reprojection fits all free parameters; sigma from `(J^T J)^-1 * s^2`; every k-th view is held out and its RMS
reported; a result over a limit (`rms_px`, `holdout_rms_px`, per-group sigma, `min_views`, `min_spots`, arm
`holdout_tool_mm`) has `passed: false` and cannot be applied (like `LimitsConfig` today). Measured joint angles are
used (gravity sag drops out). Serves A1, A2, A5.

### D9: Two buttons on the Calibrate page

"Head calibrate" (with Next spot, Stop, Compute, result, Apply) and then "Arm calibrate" (enabled once a head_chain
result is applied; small tier, then a confirmed "Allow shoulder moves" run; result, snippet Copy and Download, Apply
head part). They replace the "Arm calibration - deferred" line. User decision. Serves A3.

### D10: Hardware first, then the tilt range

The calibration runs after the head mount reprint, which must allow >= 60 deg tilt-down; the tilt range is widened
in the URDF, the servo steps, the web_ui head tab and the tests once the reprint is mounted (widening before it could
drive the old mount into the body). The plate is printed from the generator with marker ids starting at 15 (the 6x5
board uses 0-14). The reprint and the print are a user task. User decision D-B1. Serves A4.

### D11: mcp_server reloads the applied head chain without a restart

mcp_server rebuilds its HeadKinematics when `applied/applied.json` changes (mtime checked before each head command,
aim and front camera pose), so an apply never restarts mcp_server and never kills an agent session. web_ui applies
the override per request. head_state_publisher keeps the existing restart through the apply path unit. (my call)
Serves A1.

### D12: Synthetic scenes first, robot runbook last

Each fit is written test-first against synthetic scenes (corners projected through a known perturbed chain, 0.3 px
noise) that must recover the parameters within stated tolerances; the epic ends with a runbook executed on the robot
(held-out tip touches, a 0.5 m drive-straight yaw check) with real numbers. Serves A5.

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | ran (discovery brief `head-arm-calib.md`, existing parts list) | extend head_rgbd_calibration node and core; `urdf_handoff.tof_joint_origin`; `urdf_text` rewrite; mcp `call_robot_tool` pattern (`nodes/claude_agent/claude_agent/robot_stop.py`); proxy `/api/calibration/rgbd/{path}` (`nodes/web_ui/web_ui/rgbd_calibration.py`) | a new node |
| architect-researcher (opus) feasibility | ran (brief section 1-4) | floor board + gripper plate, arm as yaw reference, fixed degeneracies, Huber LSQ with covariance | tip touches as the fit (validation only), gripper camera, lidar or drive yaw |
| research-code-external | skipped: ChArUco detection, solvePnP and least_squares are already used in the core | - | - |
| research-papers | skipped: hand-eye / kinematic calibration by joint bundle adjustment is standard and covered by the architect pass | - | - |
| research-docs | skipped: no new external API surface beyond the mcp SDK client already used by claude_agent (mcp==2.3.0) | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief 1 | floor views identify mount z/roll/pitch, tilt zero, camera lever; fix the listed degeneracies | head-chain-board-solver |
| brief 1 | arm as reference; base_in_base_link and shoulder_pan fixed | arm-offsets-plate-solver |
| brief 2c | plate on the fixed jaw, 6 DOF per view, 30-40 poses in the air | arm-offsets-plate-solver, arm-pose-capture-sequence |
| brief 2 | wrist_roll identifiable "marker off the roll axis" | rejected: a constant roll offset is absorbed by a free plate rotation; D4 fixes the rotation to the clip CAD pose instead |
| brief 2a | tip touches | rejected as fit input (validation only): calib-robot-runbook |
| brief 2b | gripper camera | rejected: degenerate (2026-10-10 README) |
| brief 3 | head grid per spot, angles from /head/joint_states | head-grid-capture-sequence, joint-state-still-capture |
| brief 3 | head tracks plate with look_at | arm-pose-capture-sequence |
| brief 3 | solvePnP init, Huber least_squares, covariance, held-out, refuse over limits | head-chain-board-solver, arm-offsets-plate-solver, calib-config-and-view-types |
| brief 3 | head_chain.json override, tof_joint recompute via urdf_handoff.tof_joint_origin | head-chain-override-format, head-chain-compute-apply |
| brief 3 | arm output joint_offsets_rad snippet, not runtime | arm-offsets-compute-apply |
| brief 4 | consumers ignore overrides (head_state_publisher launch, mcp head_kinematics.py:29, front mount client.yml, web_ui raw URDF) | head-state-publisher-chain, mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| brief 4 | focus lock: hold the mapping subscription | joint-state-still-capture |
| brief 4 | second charuco entry with distinct ids | gripper-plate-marker |
| brief 4 | reprint >= 60 deg, then widen tilt range | hardware-reprint-and-plate, head-tilt-range-widened |
| brief risk | head yaw only as good as shoulder_pan zero; drive-straight check | calib-robot-runbook |
| brief | gripper-camera mount needs a re-solve after offsets change | calib-docs-and-test-index (documented), non-goal |
| reuse | `nodes/claude_agent/claude_agent/robot_stop.py:call_robot_tool` | robot-mcp-client |
| reuse | `shared/ros2_common/head_rgbd_calibration/urdf_handoff.py:tof_joint_origin` | head-chain-compute-apply |
| reuse | `shared/ros2_common/head_rgbd_calibration/charuco.py:detect` | joint-state-still-capture |
| reuse | `tests/rgbd_synthetic.py` loaded by path from node tests (`nodes/head_rgbd_calibration/tests/pose_factory.py`) | session-kinds-joint-poses, head-chain-compute-apply |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Floor board for the head, gripper plate for the arm, the arm as yaw reference | head-chain-board-solver, arm-offsets-plate-solver |
| D2 Extend the head_rgbd_calibration node and the shared core, in Python | session-kinds-joint-poses, joint-state-still-capture, head-chain-compute-apply, arm-offsets-compute-apply, sequence-http-api |
| D3 Head parameter set and the degeneracies fixed | head-chain-board-solver |
| D4 Plate rotation fixed to the clip's CAD pose | calib-config-and-view-types, arm-offsets-plate-solver |
| D5 Output forms | head-chain-override-format, head-chain-compute-apply, arm-offsets-compute-apply, head-state-publisher-chain, mcp-head-kinematics-applied, mcp-front-camera-applied, web-ui-applied-head-urdf |
| D6 Robot motion through mcp_server tools | robot-mcp-client, head-grid-capture-sequence, arm-pose-capture-sequence, calib-ansible-deploy-config |
| D7 Arm safety tiers | arm-pose-capture-sequence, arm-calibrate-ui-section |
| D8 One robust least-squares fit with sigma, held-out views and limits | calib-config-and-view-types, head-chain-board-solver, arm-offsets-plate-solver |
| D9 Two buttons on the Calibrate page | calib-frontend-api, head-calibrate-ui-section, arm-calibrate-ui-section |
| D10 Hardware first, then the tilt range | gripper-plate-marker, hardware-reprint-and-plate, head-tilt-range-widened |
| D11 mcp_server reloads the applied head chain without a restart | mcp-head-kinematics-applied |
| D12 Synthetic scenes first, robot runbook last | head-chain-board-solver, arm-offsets-plate-solver, calib-docs-and-test-index, calib-robot-runbook |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `head-chain-override` | head-chain-override-format | head-chain-board-solver, head-chain-compute-apply, head-state-publisher-chain, mcp-head-kinematics-applied, web-ui-applied-head-urdf | `shared/ros2_common/head_rgbd_calibration/urdf_text.py` |
| `head-arm-view-types` | calib-config-and-view-types | head-chain-board-solver, arm-offsets-plate-solver, session-kinds-joint-poses | `shared/ros2_common/head_rgbd_calibration/views.py` |
| `head-arm-calib-config` | calib-config-and-view-types | gripper-plate-marker, head-chain-board-solver, arm-offsets-plate-solver, joint-state-still-capture | `shared/ros2_common/head_rgbd_calibration/config.py` |
| `urdf-chain-fk` | urdf-chain-fk-core | head-chain-board-solver, arm-offsets-plate-solver, arm-pose-capture-sequence | `shared/ros2_common/head_rgbd_calibration/chain.py` |
| `head-chain-fit` | head-chain-board-solver | arm-offsets-plate-solver, head-chain-compute-apply | `shared/ros2_common/head_rgbd_calibration/head_chain_fit.py` |
| `arm-offsets-fit` | arm-offsets-plate-solver | arm-offsets-compute-apply | `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py` |
| `head-arm-synthetic-scenes` | head-chain-board-solver | arm-offsets-plate-solver, head-chain-compute-apply, arm-offsets-compute-apply | `tests/head_arm_synthetic.py` |
| `calib-session-kinds` | session-kinds-joint-poses | joint-state-still-capture, head-chain-compute-apply, arm-offsets-compute-apply | `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py` |
| `calib-still-capture` | joint-state-still-capture | head-grid-capture-sequence, arm-pose-capture-sequence | `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py` |
| `calib-robot-client` | robot-mcp-client | head-grid-capture-sequence, arm-pose-capture-sequence, sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/robot_client.py` |
| `calib-sequencer` | head-grid-capture-sequence | arm-pose-capture-sequence, sequence-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py` |
| `head-arm-calib-api` | sequence-http-api | calib-frontend-api, head-calibrate-ui-section, arm-calibrate-ui-section | `nodes/head_rgbd_calibration/tests/fixtures/api/` |

Cross-epic: `head-chain-override` is registered in `docs/plans/contracts.json`; this epic uses
`head-rgbd-applied-calibration` (owner head-rgb-d-calibration-in-th, `applied/` directory and its seed).

## Review focus

- The solvers: parameter sets against D3/D4 (a free parameter that is degenerate makes sigma explode; a fixed one
  that should be free biases the result), Huber scale, covariance scaling, held-out split, limits.
- Frame conventions: URDF rpy is fixed-axis xyz (`urdf_handoff.RPY_SEQUENCE`), head_tilt + is down, optical frames
  z forward; arm `base_link` (arm URDF root) vs robot `base_link`, joined by `base_in_base_link`; `urdf = measured +
  offset`.
- Arm safety: the small-tier validation, the 0.10 m floor check before each command, lease release on every exit,
  `allow_big_joints` only from the confirm dialog.
- Every consumer actually reading `head_chain.json`, and an unusable override falling back to the URDF with a warning.
- No placeholder data: a missing joint sample or detection means no pose, never zeros.

## Left to the builder

- The exact base pose and pose values of the shipped arm pose set, within the D7 rules and the tests that pin
  visibility, floor clearance and limits.
- Default head grid values within pan +-60 deg and tilt 30-60 deg down, and the held-out stride.
- The internal parameter vector layout and the Huber `f_scale` (default 1 px).
- UI layout inside the two sections, following the RGB-D section's components.

## Assumptions and unknowns

- The reprinted mount reaches at least 60 deg tilt-down without collision: confirmed by the user in
  hardware-reprint-and-plate, then measured with a slow tilt test.
- The node's service user can read `/etc/ros2/mcp_server/token` (group `mcp-token`): checked in
  calib-ansible-deploy-config on the robot.
- The front camera stream (`/head_camera/image_raw/compressed`) has the same aspect ratio as the mapping stream, so
  the applied intrinsics scale to it: probed in mcp-front-camera-applied from the head_camera_cpp config; a different
  aspect ratio keeps the front camera uncalibrated.
- The plate (5x3 squares of 18 mm) decodes at 0.3-0.5 m on the 576 px mapping stream: probed in the runbook;
  if not, the arm poses bring it closer.
- Arm re-zero from the servo-torque-off-and-neutral epic (blocker) marks the old offsets stale; the new offsets
  committed from this epic's snippet are what replaces them.

## Open questions

none
- Q1: After the reprint, set the head tilt range to -30..+60 deg (+ is down) everywhere it is pinned? [recommended: yes]
- Q2: Gripper plate 5x3 squares of 18 mm (14 mm markers, ids 15-21, about 98 x 62 mm with margins) on a keyed clip on the fixed jaw? [recommended: yes]
- Q3: Will you give the plate's CAD rotation in gripper_link from the clip design, so wrist_roll can be fitted (without it wrist_roll keeps its current value)? [recommended: yes]

## Irreversible operations in scope

none. Applying writes `applied/` files that the next apply or a re-seed replaces; the arm snippet is committed only
after the user reviews it.
