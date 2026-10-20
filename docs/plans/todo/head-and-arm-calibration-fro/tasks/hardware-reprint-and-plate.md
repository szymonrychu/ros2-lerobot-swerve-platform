---
title: Hardware reprint and plate
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head can see the floor next to the robot and the plate is on the jaw, without which neither calibration can run."
files: [docs/runbooks/head-arm-calibration.md]
verify: "test -f docs/runbooks/head-arm-calibration.md && grep -q '^## Hardware' docs/runbooks/head-arm-calibration.md"
depends_on: []
owns: []
uses: []
model: ""
estimate: 30
actual: ""
review: skip
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Hardware reprint and plate

## Requirement

Hardware prerequisite only the user can do (design D10; approval A4). The calibration cannot run before it: at +30 deg tilt-down the head sees no floor closer than about 0.46 m and the arm reaches x <= 0.35 m.

- Write `docs/runbooks/head-arm-calibration.md` with a `## Hardware` section listing what the user does and what is recorded:
  1. Reprint and mount the head mount so the head tilts at least 60 deg down without touching the body, the lidar or the ToF cable (checked by hand with the head torque off, or by the user's own slow test).
  2. The gripper plate is the existing 6x5 ChArUco board (`models/charuco-6x5-40mm.3mf`/`.pdf`, the same board used on the floor); no new plate is printed. Provide a clip that holds it on the fixed jaw with the gripper closed on it; record its mass (the arm carries it slowly) and an approximate board pose in `gripper_link` (`fit.plate_mount.xyz_init`/`rpy_init`). A CAD rotation of the board in `gripper_link` is optional: when given, wrist_roll is fitted instead of fixed at the recorded neutral.
  3. Record that the servo neutrals were recorded with the servo-torque-off-and-neutral feature (wrist_roll with the jaws aligned by eye), since the wrist_roll zero comes from it.
  4. Record the tilt-down angle the new mount allows and whether the camera/ToF bracket changed (if it changed, the RGB-D calibration must be redone before Head calibrate).
- Ask the user for each item and record the answers (date, measured values) in the section. Until the user confirms the head mount is mounted and the plate is attached, report the task `blocked` with `whoCanClear: user` and the list above; never fill in values the user did not give.

## Acceptance conditions

- a1: the runbook has a Hardware section with the user's recorded answers [evidence: doc-section:docs/runbooks/head-arm-calibration.md#Hardware]
- a2: the user confirmed the reprinted mount allows at least 60 deg tilt-down and the plate is attached with its CAD pose recorded [evidence: judgement]

## Files

- `docs/runbooks/head-arm-calibration.md`

## Interfaces

- `docs/runbooks/head-arm-calibration.md` `## Hardware`.

## Expected outcome

The head can see the floor next to the robot and the plate is on the jaw, without which neither calibration can run.

## Steps

- write the section; ask the user; record the answers; commit `docs: head and arm calibration hardware prerequisites`.
- Run `test -f docs/runbooks/head-arm-calibration.md && grep -q '^## Hardware' docs/runbooks/head-arm-calibration.md`; expect every test passed and lint clean.

## Notes

- no robot motion in this task.
- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
