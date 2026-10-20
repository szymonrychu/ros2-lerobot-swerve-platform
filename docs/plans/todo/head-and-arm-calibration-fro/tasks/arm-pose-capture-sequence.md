---
title: Arm pose capture sequence
status: todo
repo: ros2-lerobot-sverve-platform
value: "Arm calibrate moves the arm through safe poses with the plate in view, small joints first and shoulder joints only on the user's OK."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, nodes/head_rgbd_calibration/tests/test_sequencer_arm.py, nodes/head_rgbd_calibration/tests/test_arm_pose_set.py, nodes/head_rgbd_calibration/tests/test_config.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_arm.py tests/test_arm_pose_set.py tests/test_sequencer_head.py tests/test_config.py -q && uv run poe lint"
depends_on: [head-grid-capture-sequence, urdf-chain-fk-core]
owns: []
uses: [calib-sequencer, calib-still-capture, calib-robot-client, urdf-chain-fk]
model: ""
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Arm pose capture sequence

## Requirement

The Arm calibrate sequence: scripted arm poses with the plate in view, small joints first, shoulder joints only on an explicit OK (design D6, D7; CLAUDE.md servo safety).

- `config.py` and the shipped YAML: `arm_sequence: ArmSequenceConfig`:
  - `base_pose` (the five arm joints, rad), `small_poses: list[dict[str, float]]`, `big_poses: list[dict[str, float]]`;
  - `max_small_delta_rad` 0.6 and `max_small_roll_delta_rad` 1.6; `speed_scale` 0.2 (gt 0, le 0.3); `min_tool_height_m` 0.10; `settle_s` 1.0;
  - `joint_offsets_rad` (the five joints; default equal to the client.yml mcp_server `arm.joint_offsets_rad`: 0.0182, -0.0345, -0.1070, 0.0988, -0.0710); `arm_urdf_file` (default the deployed `nodes/web_ui/urdf/so101_arm.urdf`).
  - Validation at load: every pose names exactly the five arm joints; every small pose differs from `base_pose` only in wrist_flex, wrist_roll and elbow_flex, each within its limit; every pose is inside the arm URDF limits (read with `UrdfChain.limits`).
- Shipped default pose set: >= 12 small poses and >= 24 big poses (big ones vary shoulder_lift and shoulder_pan, within +-0.35 rad of the base pose). New `tests/test_arm_pose_set.py` pins, for every shipped pose: the nominal tool point (`UrdfChain` FK of `gripper_frame_link` + `fit.tool_offset_m` with `joint_offsets_rad`, mapped by `arm_in_robot_base(fit.base_in_base_link)`) is >= `min_tool_height_m` above the floor; with the head aimed at the plate point by the nominal head URDF (pan/tilt solved numerically, tilt within -30..+60 deg) the plate centre projects inside the 576x324 image with a 40 px margin; the camera-to-plate distance is 0.25-0.55 m; the small/big rules above hold.
- `sequencer.py`: `Sequencer.start_arm(session_id: str | None, allow_big_joints: bool) -> str`:
  - `capture.start_still_session("arm", session_id)`; `robot.take_arm()`; the small poses; then the big poses only when `allow_big_joints` is true.
  - Per pose: refuse before commanding anything when the nominal tool point is below `min_tool_height_m` (status names the pose index and height); `move_arm_joints(pose, speed_scale)`; a result that is an error or not converged stops the sequence quoting it; `head_look_at(plate point)` (nominal FK of `gripper_link` + `fit.plate_mount.xyz_init` in robot base_link); `sleep(settle_s)`; `capture_still(None)`.
  - `robot.release_arm()` always runs at the end, on stop, on a tool error and on an exception, then `head_home()`, then `capture.stop()`.
  - `state()` adds `tier` (`small` | `big`) and `allow_big_joints`.
- New `tests/test_sequencer_arm.py` (fakes): no big pose is commanded without `allow_big_joints`; with it big poses follow the small ones; release on end, stop, tool error and exception; the floor-clearance refusal happens before any command; `speed_scale` passed; the look_at target equals the FK plate point. `tests/test_config.py`: the validation errors name the pose and joint.

## Acceptance conditions

- a1: big poses never run without allow_big_joints [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_arm.py::test_no_big_pose_without_allow_big_joints]
- a2: the arm lease is released on end, stop, error and exception [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_arm.py::test_release_on_every_exit]
- a3: a pose below the floor clearance is refused before any command [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_arm.py::test_floor_clearance_refused_before_command]
- a4: every shipped pose keeps the tool 10 cm above the floor and the plate in view [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_shipped_poses_clear_floor_and_show_plate]
- a5: small poses move only wrist and elbow joints within the limits [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_small_poses_move_only_wrist_and_elbow]
- a6: sequencer and config tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_arm.py tests/test_sequencer_head.py tests/test_arm_pose_set.py tests/test_config.py -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/test_sequencer_arm.py`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

## Interfaces

- `sequencer.py::Sequencer.start_arm(session_id, allow_big_joints)`; config `arm_sequence` (pose set, `joint_offsets_rad`, `arm_urdf_file`).

## Expected outcome

Arm calibrate moves the arm through safe poses with the plate in view, small joints first and shoulder joints only on the user's OK.

## Steps

- Tests first (red), implement, verify. Choose the base pose so the plate is 0.3-0.45 m in front of the head camera; check every pose with the test before shipping it.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_arm.py tests/test_arm_pose_set.py tests/test_sequencer_head.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- least dramatic first (CLAUDE.md): only wrist and elbow in the small tier; the UI asks before the big tier.
- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

