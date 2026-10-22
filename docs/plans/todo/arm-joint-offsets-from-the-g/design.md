# Arm joint offsets from the gripper-held ChArUco board design

## Summary

Holds the arm joint offset solve moved out of head-and-arm-calibration-fro on 2026-10-22. With the head chain applied
from that epic, the board-on-gripper views give the arm offsets (shoulder_pan fixed as the gauge), a joint_offsets_rad
snippet for review and commit, and an Arm section on the Calibrate page. The three tasks are the old plan verbatim and
are re-planned against the head epic's delivered parts before this epic runs.

## Intent and why

The user trusts the arm calibration now and wants the head calibrated against it first; the offset solve is kept so
nothing is lost and can be run when the arm is re-zeroed or drifts.

## Goals

- Arm offsets fitted with sigma, held-out tool-point error and limits; a reviewable snippet, never a runtime override.
- An Arm section on the Calibrate page with every big-joint motion confirmed.

## Non-goals

- The head chain (head-and-arm-calibration-fro); link lengths; leader arm; gripper camera.

## Anti-goals

- Applying offsets without the user's review; any big-joint motion without a confirm.

## Constraints

- House rules (CLAUDE.md); runs after head-and-arm-calibration-fro (blocker) and reuses its sessions, capture,
  sequencer, pose check and fit.

## Decisions

### D1: Moved verbatim, re-planned before running

The three task files are the original head-and-arm plan; before this epic leaves todo, each is amended with
`ticket.py amend` against the parts head-and-arm-calibration-fro delivered. User decision 2026-10-22. Serves M1.

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | skipped: re-planned when the head epic has delivered | - | - |
| research-code-external | skipped: same maths as the head fit | - | - |
| research-papers | skipped: standard bundle adjustment | - | - |
| research-docs | skipped: no new external API | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| head-and-arm-calibration-fro (old plan) | arm offsets plate solver, compute/apply with snippet, Arm UI | arm-offsets-plate-solver, arm-offsets-compute-apply, arm-calibrate-ui-section |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Moved verbatim, re-planned before running | arm-offsets-plate-solver, arm-offsets-compute-apply, arm-calibrate-ui-section |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `arm-offsets-fit` | arm-offsets-plate-solver | arm-offsets-compute-apply | `shared/ros2_common/head_rgbd_calibration/arm_offsets_fit.py` |

Cross-epic: uses `head-arm-fit`, `head-arm-calib-api`, `calib-session-kinds`, `arm-pose-check` (owner
head-and-arm-calibration-fro) and `arm-calibration-stale-marker` (owner servo-torque-off-and-neutral).

## Review focus

- Re-plan first; then the solver's gauge (shoulder_pan fixed), the wrist_roll handling and the snippet.

## Left to the builder

- Settled at the re-plan.

## Assumptions and unknowns

- The head epic's parts are usable as they land; confirmed at the re-plan.

## Open questions

none

## Irreversible operations in scope

none. The snippet is committed only after the user reviews it.
