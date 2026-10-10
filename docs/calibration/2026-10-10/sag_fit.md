# Gravity sag fit 2026-10-10

Gains of `arm.sag_compensation` (nodes/mcp_server/README.md, "Gravity sag compensation"), fitted by `sag_fit.py`
(inputs in `sag_fit.yaml`, full output in `sag_fit_results.json`).

## Data

The 2026-10-10 grid-sheet touch-downs (`session/touches.json`, 8 touches, gripper vertical: pitch 1.57, wrist_roll
-1.6). Each touch was commanded with `move_arm_cartesian` and the model deployed at the time (2026-10-08 joint offsets
and tool point), so the commanded joint targets are recomputed exactly with that model's IK, seeded along the same move
chain (`session/scripts/touch.sh`):

- hover (z -0.094, 8 poses): full deflection vector, measured - target, for shoulder_lift, elbow_flex and wrist_flex;
- lift (z -0.074 from the contact pose) and up (z 0.0), 14 moves: only the logged shoulder_lift residual (the other
  joints were inside their converge tolerance and are not used);
- not used: the transit moves (start pose of t4 unknown, residuals logged only above the tolerance), the last-free
  and contact poses (the tip touches the floor), the tool-call timing journal (no poses).

The deflection depends on the approach (gearbox friction band): every joint of every move is classed lifting (its last
motion went against gravity) or lowering. Torques are computed from the URDF inertials with the current joint offsets.
The mcp_server journal of 2026-10-09..10 has no per-joint residuals; from now on every arm move logs an `arm_settle`
record (target, commanded, settled pose) that `sag_fit.py` reads from saved journals.

## Result (rad per N m, held out = leave one touch out)

| Joint | Mode | Samples | k | Held-out RMS before | after | Reduction |
|---|---|---|---|---|---|---|
| shoulder_lift | lifting | 14 | 0.141 | 0.0838 | 0.0054 | 94% |
| shoulder_lift | lowering | 8 | 0.004 | 0.0039 | 0.0034 | 12% |
| shoulder_lift | pooled | 22 | | 0.0669 | 0.0048 | 93% |
| elbow_flex | lifting | 8 | 0.169 | 0.0416 | 0.0071 | 83% |
| wrist_flex | lifting | 6 | 1.061 | 0.0120 | 0.0031 | 74% |
| wrist_flex | lowering | 2 | 0.0 | 0.0063 | 0.0063 | 0% |
| wrist_flex | pooled | 8 | | 0.0109 | 0.0042 | 62% |

Hover tool point (held out, the arm landing at target + deflection - prediction): 12.8 mm RMS before, 3.4 mm after
(per touch 9.0-15.9 mm before, 2.5-5.0 mm after).

A single gain per joint (no approach modes) only reaches 40% on shoulder_lift (0.067 -> 0.040 rad): lowering moves
barely deflect while lifting ones deflect 0.06-0.10 rad, so the approach mode is part of the model.

## Decision

Enabled in `ansible/group_vars/client.yml`: `k {shoulder_lift: 0.141, elbow_flex: 0.169}`, `k_lowering
{shoulder_lift: 0.004}`, `max_rad 0.12`. Both joints improve far beyond the 50% bar on held-out moves.

wrist_flex is not compensated: its samples cover gravity torques of 0.009-0.012 N m only, while the wrist sees up to
0.117 N m (gripper horizontal); k 1.06 would command up to the 0.12 rad saturation there without any data, for a
residual (0.011 rad) already below the 0.03 rad converge tolerance. elbow_flex has no lowering sample (k_lowering 0: a
lowering elbow approach is not compensated).

Coverage: shoulder_lift torques 0.39-0.74 N m (of 0.86 max), elbow_flex 0.13-0.33 N m (of 0.45 max), all with the
gripper vertical and no payload. Re-fit once `arm_settle` records from other poses have accumulated.
