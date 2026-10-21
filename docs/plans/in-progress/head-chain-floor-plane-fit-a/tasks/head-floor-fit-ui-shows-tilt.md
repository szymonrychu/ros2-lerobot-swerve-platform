---
title: Head floor fit UI shows tilt roll and pose-mean RMS
status: todo
repo: ros2-lerobot-sverve-platform
value: The user sees every fitted head parameter and the RMS the limits check before Apply
files: [nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx, nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx, nodes/web_ui/frontend/src/calibrate/headFloorApi.ts, nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts, nodes/web_ui/README.md]
verify: "cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint"
depends_on: [head-floor-fit-ui-step, floor-fit-pose-mean-limits-p]
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head floor fit UI shows tilt roll and pose-mean RMS

## Requirement

The floor-fit result now carries params.tilt_roll_deg (value, sigma, urdf) and both pose-mean and per-observation RMS (task floor-fit-pose-mean-limits-p, shared/ros2_common/head_rgbd_calibration/floor_fit.py and the node result.json; read the real field names there). The web UI Head floor fit step (nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx, headFloorApi.ts) must show the tilt roll row in the parameter table (current, fitted, delta, sigma like the other rows), show the pose-mean RMS that the limits use next to the per-observation RMS (labelled so the user sees which one is checked), and show skipped poses with their reason (including "partly blocked: k of n clouds"). FloorResult type and its runtime guard gain the new keys (older results without them still render). Tests first in HeadFloorFit.test.tsx / headFloorApi.test.ts. Update nodes/web_ui/README.md (Head floor fit section) if it lists the table rows.

## Acceptance conditions

- a1: the parameter table shows the head_tilt roll row with current, fitted, delta and sigma [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx -t "shows the tilt roll row"]
- a2: the pose-mean RMS checked by the limits and the per-observation RMS are both shown and labelled [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/HeadFloorFit.test.tsx -t "labels pose-mean and per-observation RMS"]
- a3: results without the new keys still render [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/headFloorApi.test.ts -t "accepts results without the new keys"]
- a4: frontend type check, tests and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint]

## Files

- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.tsx`
- `nodes/web_ui/frontend/src/calibrate/HeadFloorFit.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.ts`
- `nodes/web_ui/frontend/src/calibrate/headFloorApi.test.ts`
- `nodes/web_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

The Head floor fit step lists tilt roll with the other parameters and shows which RMS the limits check.

What is true and observable when the task is done.

## Steps

1. Write the failing tests.
2. Extend the FloorResult type, guard and the table.
3. Run the verify command, update the README, commit.

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
