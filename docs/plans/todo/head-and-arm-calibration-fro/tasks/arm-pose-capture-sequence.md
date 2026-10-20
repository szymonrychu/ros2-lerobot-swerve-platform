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

The Arm calibrate sequence: scripted arm poses with the plate in view, in three tiers of increasing motion, each beyond tier 0 only on an explicit confirm (design D6, D7; CLAUDE.md servo safety).

- `config.py` and the shipped YAML: `arm_sequence: ArmSequenceConfig`:
  - `base_pose` (the five arm joints, rad) and `gripper_rad` (the fixed gripper position that holds the plate clip closed; set once with the move to the base pose, never changed by any later pose);
  - `tier0_poses`, `tier1_poses`, `tier2_poses: list[dict[str, float]]`;
  - `max_tier1_delta_rad` 0.4 (wrist_flex, elbow_flex), `max_roll_delta_rad` 1.6 (wrist_roll); `speed_scale` 0.2 (gt 0, le 0.3); `min_tool_height_m` 0.10; `settle_s` 1.0;
  - `joint_offsets_rad` (the five joints; default equal to the client.yml mcp_server `arm.joint_offsets_rad`: 0.0182, -0.0345, -0.1070, 0.0988, -0.0710); `arm_urdf_file` (default the deployed `nodes/web_ui/urdf/so101_arm.urdf`).
  - Validation at load (errors name the tier, pose index and joint): every pose names exactly the five arm joints (no gripper); tier 0 poses differ from `base_pose` only in wrist_roll (within `max_roll_delta_rad`); tier 1 poses differ only in wrist_roll, wrist_flex and elbow_flex (flex/elbow within `max_tier1_delta_rad`); tier 2 poses may also move shoulder_lift and shoulder_pan (within +-0.35 rad of the base pose); every pose inside the arm URDF limits (`UrdfChain.limits`).
- Shipped default pose set: >= 8 tier 0, >= 12 tier 1 and >= 16 tier 2 poses. New `tests/test_arm_pose_set.py` pins, for every shipped pose: the nominal tool point (`UrdfChain` FK of `gripper_frame_link` + `fit.tool_offset_m` with `joint_offsets_rad`, mapped by `arm_in_robot_base(fit.base_in_base_link)`) is >= `min_tool_height_m` above the floor; with the head aimed at the plate point (pan/tilt solved numerically on the head URDF, with the pan/tilt limits read from that URDF's joint limits, never hard-coded) the plate centre projects inside the 576x324 image with a 40 px margin; the camera-to-plate distance is 0.25-0.55 m; the tier rules above hold. The test runs the solve on a fixture copy of `nodes/web_ui/urdf/head.urdf` with the tilt limit set to [-0.5236, 1.0472] in the test (the reprinted mount), so it does not depend on head-tilt-range-widened.
- `sequencer.py`: `Sequencer.start_arm(session_id: str | None, allow_wrist_elbow: bool, allow_big_joints: bool) -> str`:
  - `allow_big_joints` without tier 1 allowed now or already captured in the session raises ValueError.
  - `capture.start_still_session("arm", session_id)`; `robot.take_arm()` (also the lease the head tools need); first move: `move_arm_joints(base_pose + {"gripper": gripper_rad}, speed_scale)`; then tier 0, then tier 1 only when `allow_wrist_elbow`, then tier 2 only when `allow_big_joints`. A reopened session skips tiers whose poses it already holds (each stored pose records `tier`).
  - Per pose: refuse before commanding anything when the nominal tool point is below `min_tool_height_m` (status names the tier, pose and height); `move_arm_joints(pose, speed_scale)`; an error or not-converged result stops the sequence quoting it; `head_look_at(plate point)` (nominal FK of `gripper_link` + `fit.plate_mount.xyz_init` in robot base_link); `sleep(settle_s)`; `capture_still(None)` with the pose's tier.
  - End of the sequence: `robot.release_arm()` (park home; the release also homes the head, so there is no separate `head_home`), then `capture.stop()`. The release runs on every exit path: end, stop, tool error and exception.
  - `state()` adds `tier` (0, 1, 2), `allow_wrist_elbow`, `allow_big_joints`.
- New `tests/test_sequencer_arm.py` (fakes): only tier 0 without flags; tier 1 only with `allow_wrist_elbow`; tier 2 only with `allow_big_joints`; `allow_big_joints` alone refused; the gripper target is sent once with the base pose and in no later command; release (park) on end, stop, tool error and exception, and no `head_home` call; the floor-clearance refusal happens before any command; `speed_scale` passed; the look_at target equals the FK plate point; reopened session skips captured tiers. `tests/test_config.py`: validation errors name the tier, pose and joint.

Choose the base pose so the plate is 0.3-0.45 m in front of the head camera; check every pose with the test before shipping it.

## Acceptance conditions

- a1: tier 1 and tier 2 poses run only with their flags [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_arm.py::test_tiers_run_only_with_their_flags]
- a2: the arm lease is released on end, stop, error and exception with no separate head_home [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_arm.py::test_release_on_every_exit]
- a3: the gripper is set once with the base pose [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_arm.py::test_gripper_set_once_in_base_pose]
- a4: a pose below the floor clearance is refused before any command [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer_arm.py::test_floor_clearance_refused_before_command]
- a5: every shipped pose keeps the tool 10 cm above the floor and the plate in view within the URDF head limits [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_shipped_poses_clear_floor_and_show_plate]
- a6: each tier moves only its allowed joints within its limits [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_tiers_move_only_allowed_joints]
- a7: sequencer and config tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_arm.py tests/test_sequencer_head.py tests/test_arm_pose_set.py tests/test_config.py -q]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/test_sequencer_arm.py`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

## Interfaces

- `sequencer.py::Sequencer.start_arm(session_id, allow_wrist_elbow, allow_big_joints)`; config `arm_sequence` (`base_pose`, `gripper_rad`, `tier0_poses`, `tier1_poses`, `tier2_poses`, `joint_offsets_rad`, `arm_urdf_file`).

## Expected outcome

Arm calibrate moves the arm through safe poses with the plate in view, small joints first and shoulder joints only on the user's OK.

## Steps

- Tests first (red), implement, verify. Choose the base pose so the plate is 0.3-0.45 m in front of the head camera; check every pose with the test before shipping it.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer_arm.py tests/test_arm_pose_set.py tests/test_sequencer_head.py tests/test_config.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Least dramatic first (CLAUDE.md): tier 0 wrist roll only; tier 1 wrist flex and elbow behind a confirm checkbox; tier 2 shoulder joints behind a confirm dialog.
- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: builder check: head tools need a lease, three arm tiers with confirms, gripper set once, mcp-token via supplementary_groups, optional drive check
