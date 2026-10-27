---
title: Head arm pose capture sequence
status: done
repo: ros2-lerobot-sverve-platform
value: "The node captures the board-on-gripper views by itself, moving only checked poses, asking before every big-joint motion and holding the arm on every exit."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py, nodes/head_rgbd_calibration/tests/test_sequencer.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer.py tests/test_arm_pose_set.py -q && uv run poe lint"
depends_on: [arm-board-pose-set]
model: opus
review: full
verified_head: bfc290f824ce18a6e3f69d82d010286736c30f22
points: 8
expected_minutes: 90
expected_tokens: 6000000
points_actual: ""
actual_minutes: 90
actual_tokens: 734570
actual_roles: "impl 3, review 12"
actual_role_tokens: "impl 367288, review 367282"
---

# Head arm pose capture sequence

## Requirement

The guided capture sequence (owns `calib-sequencer`; design D37, D38, D46; approvals H2, H5; CLAUDE.md servo safety).
Every motion goes through `RobotClient`; nothing is commanded before its path check passes.

- New `head_rgbd_calibration/sequencer.py`: `Sequencer(capture: CaptureController, robot: RobotClient, cfg:
  NodeConfig, head_urdf: Callable[[], str], arm_urdf: Callable[[], str], blocked: Callable[[], str | None], sleep:
  Callable[[float], None] = time.sleep, clock: Callable[[], float] = time.monotonic)`. One sequence at a time on a
  worker thread; `Busy` when a sequence runs or `blocked` names a reason (an RGB-D capture, a compute or a floor sweep).
  `head_urdf` returns the applied head URDF (`floor_results.base_urdf`), so the aim and the head envelope use the
  applied chain.
- Phases and calls (state `phase`): `move_to_base(confirmed: bool)`: refused without `confirmed` (ValueError, no tool
  call); `take_arm()`; `check_path(measured joints from arm_state(), base_pose)` (a refusal or a missing measured joint
  stops before any command); `move_arm_joints(base_pose + {"gripper": gripper_rad}, speed_scale)` -> phase
  `waiting_board` ("Clip the board to the fixed jaw, then press Board attached"). `board_attached()` ->
  `start_still_session(None)`, then tier 0 and tier 1 -> phase `waiting_big_joints` (status names elbow_flex,
  shoulder_lift, shoulder_pan). `allow_big_joints(confirmed: bool)`: True runs tier 2, False skips it; then
  `check_path` and the move back to `base_pose` -> phase `holding` ("Remove the board, then press Park"),
  `capture.stop()`, `release_arm(keep_position=True)`. `park(board_removed: bool)`: refused without `board_removed`;
  `take_arm()`, `release_arm(keep_position=False)` (mcp_server parks home) -> `idle`.
- Per pose (index `arm_pose` over the whole set, its tier): `check_path(measured, pose)`; refusal -> stop naming the
  tier, pose and rule; `move_arm_joints(pose, speed_scale)` (never a gripper key after the base move); an error or a
  not-converged result stops quoting it; nominal board centre by arm FK (measured joints after the move,
  `fit.board_mount`), `aim_head` on the applied chain; for each `head_offsets_deg` entry: the aimed angles plus the
  offset, skipped (counted with the reason) when outside the URDF head limits, else `set_head_angles`,
  `sleep(settle_s)`, `capture_still(arm_pose, tier, offset index)`; outcomes counted (`captured`, `skipped` with
  reasons).
- Exits: `stop()`, a tool error, an exception and a `confirm_timeout_s` wait in `waiting_board` or
  `waiting_big_joints` all end in `holding` when the board may be on (after `board_attached`), or `idle` before it:
  `capture.stop()` and `release_arm(keep_position=True)` (the arm holds a checked pose; mcp_server keeps the head where
  it is and reports `reset.head: kept`, `nodes/mcp_server/mcp_server/control.py::release_head`). The sequence never
  parks on its own and never sends `carried_object_radius_m`, so mcp_server publishes the frozen measured-arm footprint
  without a carried-object circle and writes no carry record (`move_arm_joints` never touches it).
- `park(board_removed=True)` first reads `control_state()`: when `arm.carrying.carrying` is true (a holding load on
  the gripper or a carry record) Park is refused quoting its reasons (mcp_server would refuse the park too); the park
  release homes the arm and then the head (`reset.head: homed`).
- `gripper_rad` is a position where the jaws do not squeeze the clip (no holding load, `holding_min_effort`), so the
  board never counts as a carried object.
- `state() -> {running, phase, session_id, tier, arm_pose, total_poses, captured, skipped, status, error,
  big_joints: [names], waiting_since}`.
- New `tests/test_sequencer.py` (fake RobotClient, fake capture, injected clock): `move_to_base` without confirm sends
  nothing; call order of a full run; tier 2 only after `allow_big_joints(True)`, skipped with False; the gripper key
  only in the base move; a failing path check (fake pose) stops before any command; release with
  `keep_position=True` on end, stop, tool error, exception and confirm timeout, and no park; `park` refused without
  `board_removed` and only then `release_arm(keep_position=False)`; Park refused while `control_state()` reports
  carrying; no call ever carries `carried_object_radius_m`; head commands equal aim plus offset and an
  out-of-limit offset is skipped with its reason; Busy; state keys.

## Acceptance conditions

- a1: no motion without the base-pose confirm [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_move_to_base_needs_confirm]
- a2: big-joint tier only after the confirm, skipped when declined [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_big_joints_need_confirm]
- a3: a failing path check stops before any command [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_path_check_refusal_sends_nothing]
- a4: every exit holds the arm and never parks [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_every_exit_holds_position]
- a5: park only after the board-removed confirm [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_park_needs_board_removed]
- a6: the gripper is commanded only with the base pose [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_gripper_only_in_base_move]
- a7: head commands are the aim plus the offsets within limits [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_head_aim_offsets]
- a9: Park is refused while mcp_server reports carrying, and no release carries a radius [evidence: test:nodes/head_rgbd_calibration/tests/test_sequencer.py::test_park_refused_while_carrying]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/sequencer.py`
- `nodes/head_rgbd_calibration/tests/test_sequencer.py`

## Interfaces

- `sequencer.py::Sequencer` (`move_to_base`, `board_attached`, `allow_big_joints`, `stop`, `park`, `state`), `Busy`
  (part `calib-sequencer`).

## Expected outcome

The node captures the board-on-gripper views by itself, moving only checked poses, asking before every big-joint motion
and holding the arm on every exit.

## Steps

- Write `tests/test_sequencer.py` first; run it: red on the missing module.
- Implement; run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_sequencer.py tests/test_arm_pose_set.py -q && uv run poe lint`;
  expect every test passed and lint clean.

## Ruling

- 2026-10-22 accepted: gate-repeat only on parking-marker false positives (design D47 wording 'before the first capture', 'a measured mount goes here as the start value', a config doc 'over the last this many seconds'); verified True on HEAD by the run's own verifier

## Notes

- Follow `floor_sweep.py`'s threading (lock for short updates, `stop_event`, cleanup in `finally`).
- The base move moves shoulder and elbow joints, hence its own confirm; the UI dialog names them.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on arm-board-pose-set
