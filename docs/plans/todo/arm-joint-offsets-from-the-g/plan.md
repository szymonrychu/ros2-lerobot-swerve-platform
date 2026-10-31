---
title: Arm joint offsets from the gripper-held ChArUco board
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-22
owner: ""
rung: ""
---

# Arm joint offsets from the gripper-held ChArUco board

## What and why

What: Arm joint offset solve from the 6x5 ChArUco board on the gripper, seen by the calibrated head camera
How: Moved tasks: plate solver, compute and apply with a joint_offsets_rad snippet, Arm UI section
Why: Moved from head-and-arm-calibration-fro so the arm offset solve is not lost while the head is calibrated against the trusted arm

Moved out of head-and-arm-calibration-fro on 2026-10-22 (user decision: "calibrate head position/angles based on the
arm position/angles (it should be well calibrated now)"). That epic now fits only the head chain against the trusted
arm. This epic keeps the arm joint offset solve so nothing is lost: with the board clipped to the fixed jaw and the head
chain applied, fit the shoulder_lift, elbow_flex, wrist_flex (and, with a CAD board rotation, wrist_roll) offsets,
give a joint_offsets_rad snippet for review and commit, and add an Arm section to the Calibrate page. The three task
files are the original plan moved verbatim; re-plan them with ticket.py amend against the parts the head epic
delivered (head_arm sessions, still capture, sequencer, pose check, head-from-arm fit) before running.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: Fit the arm joint offsets (shoulder_pan fixed as the gauge) from board-on-gripper views with the applied head chain,
  with sigma, held-out tool-point error and limits that refuse a bad result.
- 1.2: A joint_offsets_rad snippet for review and commit; never a runtime override.
- 1.3: An Arm section on the Calibrate page, every big-joint motion confirmed.

## Agreed scope

not agreed yet

## Non-goals

- Fitting the head chain (head-and-arm-calibration-fro).
- Link lengths, leader arm, gripper camera.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: Moved verbatim, re-planned before running

The three task files are the original head-and-arm plan; before this epic leaves todo, each is amended with
`ticket.py amend` against the parts head-and-arm-calibration-fro delivered. User decision 2026-10-22. Serves M1.

## Open questions

none

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | skipped: re-planned when the head epic has delivered | - | - |
| research-code-external | skipped: same maths as the head fit | - | - |
| research-papers | skipped: standard bundle adjustment | - | - |
| research-docs | skipped: no new external API | - | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| head-and-arm-calibration-fro (old plan) | arm offsets plate solver, compute/apply with snippet, Arm UI | arm-offsets-plate-solver, arm-offsets-compute-apply, arm-calibrate-ui-section |

## Irreversible operations

none. The snippet is committed only after the user reviews it.

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

### design.md: Summary

Holds the arm joint offset solve moved out of head-and-arm-calibration-fro on 2026-10-22. With the head chain applied
from that epic, the board-on-gripper views give the arm offsets (shoulder_pan fixed as the gauge), a joint_offsets_rad
snippet for review and commit, and an Arm section on the Calibrate page. The three tasks are the old plan verbatim and
are re-planned against the head epic's delivered parts before this epic runs.

### design.md: Intent and why

The user trusts the arm calibration now and wants the head calibrated against it first; the offset solve is kept so
nothing is lost and can be run when the arm is re-zeroed or drifts.

### design.md: Goals

- Arm offsets fitted with sigma, held-out tool-point error and limits; a reviewable snippet, never a runtime override.
- An Arm section on the Calibrate page with every big-joint motion confirmed.

### design.md: Non-goals

- The head chain (head-and-arm-calibration-fro); link lengths; leader arm; gripper camera.

### design.md: Anti-goals

- Applying offsets without the user's review; any big-joint motion without a confirm.

### design.md: Constraints

- House rules (CLAUDE.md); runs after head-and-arm-calibration-fro (blocker) and reuses its sessions, capture,
  sequencer, pose check and fit.

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 Moved verbatim, re-planned before running | arm-offsets-plate-solver, arm-offsets-compute-apply, arm-calibrate-ui-section |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `arm-offsets-fit` | arm-offsets-plate-solver | arm-offsets-compute-apply | `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py` |

Cross-epic: uses `head-arm-fit`, `head-arm-calib-api`, `calib-session-kinds`, `arm-pose-check` (owner
head-and-arm-calibration-fro) and `arm-calibration-stale-marker` (owner servo-torque-off-and-neutral).

### design.md: Review focus

- Re-plan first; then the solver's gauge (shoulder_pan fixed), the wrist_roll handling and the snippet.

### design.md: Left to the builder

- Settled at the re-plan.

### design.md: Assumptions and unknowns

- The head epic's parts are usable as they land; confirmed at the re-plan.

### approvals.json

- M1: User 2026-10-22: the arm joint offset solve (arm-offsets-plate-solver, arm-offsets-compute-apply, arm-calibrate-ui-section) moves here from head-and-arm-calibration-fro, not deleted, so nothing is lost. [decisions: D1] [tasks: arm-offsets-plate-solver, arm-offsets-compute-apply, arm-calibrate-ui-section] [status: approved]
