# Head-from-arm calibration runbook

Calibrates the head chain against the trusted arm with the 6x5 ChArUco board held in the gripper
(design D31-D51 of drivable-ground-tof-filterin). This file starts with the hardware facts only the
user can supply; the pose set (arm-board-pose-set) and the tilt range (head-tilt-range-widened)
read them.

## Hardware

Recorded 2026-10-23, all values given by the user. Anything not listed here was not given and must
not be filled in by an agent.

1. Board and clip
   - Board: the 6x5 ChArUco board (`models/charuco-6x5-40mm.3mf` / `.pdf`, about 250 x 210 mm with
     margins), the same board used for the RGB-D calibration.
   - Clip: the board is glued to a stick at the middle of its long edge. The stick is clamped in the
     gripper with a rubber band. The gripper stays at most half open (the roll rule).
   - Orientation: board at a right angle to the gripper, print toward the arm (camera side). The
     board extends UP from the stick when the arm is parked. Its lower edge is about 20 mm past the
     grip tip.
   - Mass: 214 g. The arm carries it at `speed_scale` 0.1 (the mount is fragile; the 0.3 ceiling
     of the plan is not used).
   - Board frame pose in `gripper_link` (origin = first inner corner of the grid, as
     `board_corners_3d`): the geometry above is the user's description; the numeric translation (mm)
     and rotation (deg) were NOT given as measured values. The pose set derives the starting estimate
     from the geometry above, the fit refines it, and the safety check applies its margin. If a ruler
     measurement is taken later, write it here with its date.
2. Head tilt
   - Tilt-down limit with the reprinted mount: 45 deg (+ is down). Gate Q1 (H6): the tilt range is
     -30..+60 deg everywhere it is pinned, or the smaller tilt-down limit the user records. The user
     recorded 45 deg, so the pinned range is -30..+45 deg.
   - Camera/ToF bracket: unchanged since the 2026-10-21 RGB-D calibration, so that calibration is
     not redone first.
3. Robot state during the sequence
   - The robot stays docked and charging (no driving) and the area around the arm is clear.

## Arm safety (H2)

- Least dramatic joints first. Any pose moving shoulder_pan, shoulder_lift or elbow needs the user's
  explicit OK at run time (UI confirmation).
- Every scripted pose is FK-checked against self-collision with the robot body and head.
- Wrist roll only half-open and lifted; negative elbow stretches.
- Arm motion only through mcp_server's guarded tools. No driving.

## Scene (for the ground tasks, not the arm sequence)

- Box: 5 x 25 x 18 cm.
- Ramp: 6 cm high; its other dimensions are the same as the board's.
