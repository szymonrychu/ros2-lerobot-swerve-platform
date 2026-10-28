---
title: D435 recalibration runs
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The head, the D435 mount, the arm offsets and the tool point are all calibrated on the D435."
files: [ansible/group_vars/client.yml, docs/calibration/, docs/runbooks/d435-switchover.md, tests/test_d435_runbook.py, MEMORY.md]
verify: "uv run pytest tests/test_d435_runbook.py tests/test_head_rgbd_calibration_deploy.py tests/test_mcp_server_config.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [calibrate-head-and-arm-ui-se, calibration-apply-restart-un, head-chain-overrides-on-d435, mcp-front-camera-on-d435, d435-live-bring-up-and-topic]
model: sonnet
review: ""
verified_head: ""
points: 8
expected_minutes: 91
expected_tokens: 6000000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# D435 recalibration runs

## Requirement

LIVE task. Goal G8: "Recalibration runs on the D435: head mount extrinsics through the pan/tilt chain, floor fit, intrinsics check against factory; the web_ui calibration screen drives it." Goal G6: "The arm joint offset / tool point solve ... is re-done from the head D435 watching a gripper-held ChArUco board." Design D9, D10. Supersedes tof-depth-fix-and-ground-noi head-remount-floor-runbook (D15).
Safety (CLAUDE.md servo rule and memory arm-motion notes): the user watches every arm step. Run in order: head-only motion first (intrinsics capture with a hand-held board, floor sweep), then the arm tiers. Wrist and elbow tiers come before shoulder, and shoulder moves only after the confirm dialog. Wrist roll happens only with the gripper at most half open and the arm lifted. Every pose is FK-checked by the sequencer's PoseChecker. Stop on any stall.

1. Deploy `./scripts/deploy-nodes.sh client head_rgbd_calibration head_state_publisher web_ui 2>&1 | tee .logs/deploy-client-calib.log` (ansible-deploy skill; agent idle).
2. Calibrate page, D435 intrinsics check: >= 12 stills of the hand-held 6x5 board at 0.3-0.6 m covering the coverage grid; compute; record rms, focal diff, principal diff and plane bias; Accept when passed. If it fails, stop and return a `discovered` entry for the on-chip calibration health check (D10); do not apply anything.
3. Head floor fit (head moves only): sweep, compute, Apply; record the fitted chain and residuals.
4. Head from arm: board clipped to the fixed jaw; tiers 0-1, then shoulder after confirm; compute; Apply; record d435_joint and the held-out reprojection px.
5. Arm offsets + tool point (Q2): compute on the same board; accept only if passed with held-out views from >= 3 head pan/tilt settings. Commit the snippet to client.yml (`arm.joint_offsets_rad`, `tool_offset_m` when fitted, and the calibration node's `arm_sequence.joint_offsets_rad` mirror). Apply the head part.
6. `./scripts/deploy-nodes.sh client mcp_server 2>&1 | tee .logs/deploy-client-mcp.log` (check no agent session is running first; memory note). Check: the FK tool point projected into the D435 image (get_annotated_camera_image overlay `gripper`) lands on the jaw tip at 3 arm poses; record the pixel error.
7. Save each result's report.md and result.json under `docs/calibration/<date>-d435/`. Write the runbook section "Calibration" in `docs/runbooks/d435-switchover.md`; `tests/test_d435_runbook.py::test_calibration_section_is_recorded`. Add a MEMORY.md line: arm offsets and tool point re-solved from the head D435 on <date>.

## Acceptance conditions

- AC1: the D435 intrinsics check passed against factory and is recorded [evidence: test:tests/test_d435_runbook.py::test_calibration_section_is_recorded]
- AC2: the committed arm offsets equal in mcp_server and the calibration node [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_fit_joint_offsets_equal_mcp_server_arm]
- AC3: the calibration results are committed [evidence: command:test -n "$(git ls-files 'docs/calibration/*-d435/*')"]
- AC4: mcp_server is deployed with the new offsets [evidence: command:./scripts/deploy-nodes.sh client mcp_server]

## Files

- `ansible/group_vars/client.yml`
- `docs/calibration/`
- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `MEMORY.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add `test_calibration_section_is_recorded`; run `uv run pytest tests/test_d435_runbook.py -q`; expect a failure.
- Run the numbered live steps with the user present; save outputs under `.logs/`.
- Commit the results, client.yml offsets, runbook and MEMORY.md; rerun the tests; commit `feat(calibration): D435 head and arm recalibration`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28 (user): the board is the same 3D-printed 6x5 ChArUco (same dimensions as before), 4 mm thick; it is in front of the robot and the D435 head is mounted. Account for the 4 mm thickness where the board surface vs mount face matters (plate_mount / tool_in_board_m, board-plane depth bias).
