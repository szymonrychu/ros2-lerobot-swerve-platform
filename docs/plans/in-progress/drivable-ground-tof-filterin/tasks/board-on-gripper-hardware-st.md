---
title: Board on gripper hardware step
status: done
repo: ros2-lerobot-sverve-platform
value: "The board clip, its pose on the jaw and the mount's tilt limit are recorded from the user, so the pose set and the safety check use real geometry."
files: [docs/runbooks/head-arm-calibration.md]
verify: "test -f docs/runbooks/head-arm-calibration.md && grep -q '^## Hardware' docs/runbooks/head-arm-calibration.md"
depends_on: []
model: ""
review: skip
verified_head: ""
points: 3
expected_minutes: 30
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Board on gripper hardware step

## Requirement

Hardware prerequisite only the user can do (design D38, D42, D45; approvals H2, H6). The pose set (arm-board-pose-set)
and the tilt range (head-tilt-range-widened) wait for it.

- Write `docs/runbooks/head-arm-calibration.md` with a `## Hardware` section listing what the user does and what is
  recorded:
  1. The 6x5 ChArUco board (`models/charuco-6x5-40mm.3mf`/`.pdf`, about 250 x 210 mm with margins, the board used for
     the RGB-D calibration) on a clip that holds it rigidly on the fixed jaw, so the board cannot slip during the
     sequence; the gripper closed on the clip at most half open (the roll rule). Record the clip, the board's outer
     size and margin, and its mass (the arm carries it at speed_scale <= 0.3).
  2. The approximate board frame pose in `gripper_link` (board origin = the first inner corner of the ChArUco grid, as
     `board_corners_3d`): translation in mm and rotation in deg, measured with a ruler against the jaw (the fit refines
     it; the safety check uses it with its margin, so it must be within about 1 cm and 5 deg).
  3. The tilt-down angle the reprinted mount allows without touching the body, the lidar or the ToF cable (checked by
     the user by hand with the head torque off, or the user's own slow test), and whether the camera/ToF bracket
     changed since the 2026-10-21 RGB-D calibration (if it changed, the RGB-D calibration is redone first).
  4. That the robot stays docked and charging (no driving) and the area around the arm is clear during the sequence.
- Ask the user for each item and record the answers (date, measured values) in the section. Until the user confirms,
  report the task `blocked` with `whoCanClear: user` and the list above; never fill in values the user did not give.

## Acceptance conditions

- a1: the runbook has a Hardware section with the user's recorded answers [evidence: doc-section:docs/runbooks/head-arm-calibration.md#Hardware]
- a2: the user confirmed the clip, the board pose on the jaw and the tilt-down limit [evidence: judgement]

## Files

- `docs/runbooks/head-arm-calibration.md`

## Interfaces

- `docs/runbooks/head-arm-calibration.md` `## Hardware` (read by arm-board-pose-set and head-tilt-range-widened).

## Expected outcome

The board clip, its pose on the jaw and the mount's tilt limit are recorded from the user, so the pose set and the
safety check use real geometry.

## Steps

- Write the section; ask the user; record the answers; commit `docs: head and arm calibration hardware prerequisites`.
- Run `test -f docs/runbooks/head-arm-calibration.md && grep -q '^## Hardware' docs/runbooks/head-arm-calibration.md`;
  expect exit 0.

## Ruling

- 2026-10-23 accepted: The numeric mount is not missing: the user described it (photo + 'up', 20 mm past the tip) and commit 4daf0e5d turned it into xyz_init [-0.1665, 0.1938, -0.1181] / rpy_init [3.141593, 0, 0.185886] in gripper_link; the live fit then solved the real mount (result r-20261023-112701 passed). Design D47: the measurement is only a start value. Runbook section written and verified (a6cf97c1).

## Notes

- No robot motion in this task.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).

## User answers (2026-10-22)

- Mount: the 6x5 board glued to a stick on its back (middle of the long edge, "square lollipop"), the stick held in the
  closed gripper and secured with a rubber band; the board is at a right angle to the gripper and its edge about 20 mm
  from the grip tip, touching the gripper.
- Mass: 214 g.
- Still open: which side the printed face points (gripper camera side or the other), the board rotation on the jaw,
  the tilt-down limit of the reprinted mount, and whether the camera/ToF bracket changed since 2026-10-21.

- User 2026-10-22: "mount is fragile, so please don't move too fast". The calibration run uses half the planned arm
  speed (speed_scale <= 0.15 instead of 0.3) and the gentlest acceleration the follower supports; the live step sets
  this in the arm sequence config before the first move, and the board is checked for slip after the dry run.
- User 2026-10-23: the board is mounted with its printed face on the gripper camera side; the camera/ToF bracket has
  not changed since the 2026-10-21 RGB-D calibration (no RGB-D re-calibration needed). Still open: the head tilt-down
  limit of the reprinted mount.
- User 2026-10-23: the reprinted mount allows 45 deg tilt down (not 60): head-tilt-range-widened uses +45 deg as the down limit.
