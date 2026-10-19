---
title: RGBD calib runbook and docs
status: todo
repo: ros2-lerobot-sverve-platform
value: "Operators follow one written procedure for calibrating on the robot, and the overlap with the fusion epic is explicit."
files: [docs/runbooks/head-rgbd-fusion.md, docs/head-rgbd-fusion-methodology.md, scripts/README.md, MEMORY.md]
verify: "grep -q 'Calibrate page' docs/runbooks/head-rgbd-fusion.md && grep -q 'head_rgbd_calibration/applied' MEMORY.md"
depends_on: [head-nodes-use-applied-calib, rgbd-sessions-and-results-ui, calib-core-retire-shims, head-urdf-tof-joint-override]
owns: []
uses: []
model: ""
estimate: 30
actual: ""
review: skip
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--rgbd-calib-runbook-and-docs
branch: head-rgb-d-calibration-in-th--rgbd-calib-runbook-and-docs
verified_head: ""
run: head-rgb-d-calibration-in-th
provides: []
acs_for_audit: []
process_gaps: []
---

# RGBD calib runbook and docs

## Requirement

Document the robot-side calibration flow and the decisions (design D11). Prose task.

- `docs/runbooks/head-rgbd-fusion.md` steps 2 and 3 describe the web UI flow: ToF warm-up and board check as today; open the hamburger menu, Calibrate; Start capture; 20-30 poses 0.25-0.45 m from the head, resting the board about 3 s each, tilts and image corners; watch the status line, the pose count and the RGB and ToF coverage grids; delete bad poses; Stop; Compute; read the report against the limits (RGB <= 0.5 px, ToF <= 0.3 px, ToF-to-RGB extrinsic RMS <= 1.0 px, plane bias <= 15 mm) and recapture or prune when it fails; Apply (restarts head_camera_cpp, head_rgbd_cpp, head_state_publisher; the section shows each unit restarted); confirm `/head_rgbd/depth/image_raw` publishes at the mapping rate with a subscriber and that the focus is continuous again once the capture stopped; optionally Download and commit the zip's files to `nodes/head_camera_cpp/config/head_camera_calibration.yaml`, `nodes/head_rgbd_cpp/config/tof_calibration.yaml`, `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml` and `nodes/web_ui/urdf/head.urdf` (they seed `applied/` on a reflashed robot). Only steps 2 and 3 are edited; the rest of the runbook (steps 1 and 4-8) belongs to the fusion epic's rgbd-final-suite-and-runbook and is left as is.
- `docs/head-rgbd-fusion-methodology.md`: the capture description points to the robot-side flow and the state dir.
- `scripts/README.md`: the CLI is for bags, validation captures and offline calibration; robot capture is the Calibrate page.
- `MEMORY.md`: one decision line: head RGB-D calibration is captured, computed and applied on the robot (`/var/lib/ros2/head_rgbd_calibration/applied`, Apply restarts the head nodes; repo files only seed it).

## Acceptance conditions

- a1: the runbook calibration steps use the Calibrate page and no longer the ssh live counter [evidence: command:grep -q "Calibrate page" docs/runbooks/head-rgbd-fusion.md && ! grep -q "live line" docs/runbooks/head-rgbd-fusion.md]
- a2: MEMORY.md records the robot-side delivery decision [evidence: command:grep -q "head_rgbd_calibration/applied" MEMORY.md]
- a3: the runbook still has its steps and results sections [evidence: doc-section:docs/runbooks/head-rgbd-fusion.md#Steps]
- a4: the documented flow matches the built UI and node behaviour (kept as a manual judgement check) [evidence: judgement:]

## Files

- `docs/runbooks/head-rgbd-fusion.md`
- `docs/head-rgbd-fusion-methodology.md`
- `scripts/README.md`
- `MEMORY.md`

## Interfaces

- `docs/runbooks/head-rgbd-fusion.md` steps 2-3; `MEMORY.md` decision line.

## Expected outcome

The runbook's calibration steps can be followed at the robot with the Calibrate page alone.

## Steps

- Edit the four documents; run the verify command (both greps succeed).

## Notes

- Prose task (review skip). The runbook belongs to the in-progress task rgbd-final-suite-and-runbook, which executes it later; keep its `## Results` section untouched.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on head-nodes-use-applied-calib, rgbd-sessions-and-results-ui, head-urdf-tof-joint-override
