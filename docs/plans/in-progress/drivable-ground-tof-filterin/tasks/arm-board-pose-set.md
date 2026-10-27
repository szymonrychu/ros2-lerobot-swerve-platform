---
title: Arm board pose set
status: done
repo: ros2-lerobot-sverve-platform
value: "A shipped arm pose set that is proven safe by FK, keeps the board in view and is known to give a passing fit before the arm ever moves."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/tests/test_arm_pose_set.py, nodes/head_rgbd_calibration/tests/test_config.py]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q"
depends_on: [robot-mcp-client, arm-pose-safety-check, head-from-arm-board-solver]
model: opus
review: full
verified_head: 3125be849dc861f09746ee23c16452545aed3d1c
points: 8
expected_minutes: 75
expected_tokens: 6000000
points_actual: ""
actual_minutes: 194
actual_tokens: 3617190
actual_roles: "impl 17, review 26"
actual_role_tokens: "impl 1941595, review 1675595"
---

# Arm board pose set

## Requirement

The arm sequence config and the shipped pose set, proven safe and sufficient by tests (owns `arm-sequence-config`;
design D37, D38, D39, D43; approvals H2, H7; CLAUDE.md servo safety: least dramatic joints first).

- `config.py`, the shipped YAML and the client.yml `head_rgbd_calibration` block: `arm_sequence: ArmSequenceConfig`:
  - `base_pose` (the five arm joints, rad) and `gripper_rad` (validated <= `pose_check.max_roll_gripper_rad`);
  - `tier0_poses`, `tier1_poses`, `tier2_poses: list[dict[str, float]]`;
  - `max_roll_delta_rad` 1.2, `max_flex_delta_rad` 0.4, `max_big_delta_rad` 0.35; `speed_scale` 0.2 (gt 0, le 0.3);
    `settle_s` 1.0; `confirm_timeout_s` 600.0; `head_offsets_deg: list[tuple[float, float]]` (pan, tilt; default
    `[[0, 0], [10, 0], [-10, 0], [0, 6]]`); `pose_check: PoseCheckConfig`; `provisional_margin_m` 0.05 (added to
    `pose_check.margin_m` while the board mount is the nominal one, design D47).
  - Board mount (design D47, open question Q8): this task does not wait for the hardware step. `fit.board_mount`
    (xyz_init, rpy_init, outline_margin_m) in the same YAML and block is a nominal pose of the board on the fixed jaw,
    derived from the jaw geometry of `nodes/web_ui/urdf/so101_arm.urdf` (`gripper_link`, `gripper_frame_link`) and the
    board outline, with a comment that it is provisional; the solver fits the real mount anyway and the measurement from
    board-on-gripper-hardware-st only replaces this start value (calib-robot-runbook). Every safety test runs with the
    nominal mount and `margin_m + provisional_margin_m`, and also with the mount shifted by +-1 cm and +-5 deg about
    each axis, so a measured mount within that band keeps every pose safe. The poses are provisional until the dry run
    in calib-robot-runbook.
  - Validation at load (errors name the tier, pose index and joint): every pose names exactly the five arm joints (no
    gripper); tier 0 poses differ from `base_pose` only in wrist_roll (within `max_roll_delta_rad`); tier 1 only in
    wrist_roll and wrist_flex (flex within `max_flex_delta_rad`); tier 2 may also move elbow_flex, shoulder_lift and
    shoulder_pan (each within `max_big_delta_rad` of the base pose).
- Joint convention (settled from the code): `move_arm_joints` targets are measured follower angles,
  `nodes/mcp_server/mcp_server/arm.py::move_joints` docstring "Joint name -> target rad in measured (follower) joint
  space, as reported by get_arm_state (clamped to URDF limits minus margin, applied in URDF space: urdf = measured +
  offset)". The poses and `base_pose` are stored as measured angles and every FK applies `urdf = measured + offset`
  with `fit.joint_offsets_rad`; the URDF limit check is done on `measured + offset`. A test pins this by asserting the
  docstring phrase in `arm.py`, so a change there fails the test.
- Shipped pose set: at least 8 tier 0, 8 tier 1 and 12 tier 2 poses, ordered so each transit is short; the base pose
  puts the board 0.3-0.5 m in front of the head camera, lifted and forward, clear of the body and the head.
- New `tests/test_arm_pose_set.py`, on `nodes/web_ui/urdf/so101_arm.urdf` and `nodes/web_ui/urdf/head.urdf` both
  as shipped and with the applied floor-fit chain of 2026-10-22 as overrides:
  - every pose passes `PoseChecker.check_pose` (rolling where wrist_roll differs from the previous pose) and every
    transit passes `check_path`: park/home -> base pose (the home pose read from the mcp_server config the builder
    finds; name it), base -> tier 0 -> tier 1 -> tier 2 -> base, with the shipped `gripper_rad`;
  - safety runs with the head envelope of whichever URDF is given (it reads the URDF joint limits; the test runs both
    the shipped -30..+30 deg URDF and a fixture with tilt -30..+60 deg, so head-tilt-range-widened cannot make a
    shipped pose unsafe);
  - visibility: with the head aimed by `aim_head` (limits read from the URDF, so the current -30..+30 deg tilt range)
    plus each head offset, at least 12 of the 20 board corners project inside the 576x324 image with a 10 px margin
    for at least 3 of the 4 offsets of every pose, and the camera-to-board distance is 0.25-0.65 m;
  - coverage: the aimed pan/tilt angles of the set span at least `fit.limits.min_pan_span_deg` and
    `min_tilt_span_deg`, and the set has at least `fit.limits.min_arm_poses` arm poses;
  - sufficiency: `arm_board_scene` on the shipped set (0.3 px noise, the synthetic truth of head-from-arm-board-solver)
    gives a `fit_head_from_arm` result that passes every limit;
  - the tier rules above hold for the shipped set. `tests/test_config.py`: defaults equal the YAML; each validation
    error names the tier, pose and joint.

## Acceptance conditions

- a1: every shipped pose and transit passes the FK safety check [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_every_pose_and_transit_is_safe]
- a2: the board is in view of the aimed head for every pose [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_board_visible_from_aimed_head]
- a3: the pose set covers the pan/tilt spans and pose count [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_coverage_limits]
- a4: a synthetic fit on the shipped set passes every limit [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_synthetic_fit_on_shipped_set_passes]
- a5: the move_arm_joints joint convention is pinned [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_target_joint_convention]
- a6: tier validation errors name the tier, pose and joint [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_arm_sequence_validation_names_pose]
- a7: the client.yml block still equals the shipped YAML [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a9: every pose stays safe with the provisional margin and a board mount off by 1 cm / 5 deg [evidence: test:nodes/head_rgbd_calibration/tests/test_arm_pose_set.py::test_safe_under_board_mount_uncertainty]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_arm_pose_set.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`

## Interfaces

- `config.py::ArmSequenceConfig`, `NodeConfig.arm_sequence` (part `arm-sequence-config`).

## Expected outcome

A shipped arm pose set that is proven safe by FK, keeps the board in view and is known to give a passing fit before the
arm ever moves.

## Steps

- Write `tests/test_arm_pose_set.py` and the config tests first; run them: red on the missing config.
- Derive the nominal board mount from the arm URDF jaw geometry; design the poses with the tests; update the YAML and the client.yml block; run
  `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`;
  expect every test passed and lint clean.

## Ruling

- 2026-10-22 accepted: gate-repeat on parking-marker false positives ('goes here as the start value', 'dry run checks them before the first capture' are design D47 wording, not deferred work); verified: 83 passed, lint clean on c474ae22

## Notes

- No robot motion in this task; poses are proven by FK only.
- The node tests load the root `tests/head_arm_synthetic.py` by path, as `tests/pose_factory.py` loads
  `tests/rgbd_synthetic.py`.
- Negative elbow_flex stretches the arm; keep shoulder_lift well under 1.85 rad when stretched (MEMORY).
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
