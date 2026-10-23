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

## Procedure

Mirrors `docs/runbooks/head-floor-fit.md`. Preconditions: Hardware section above recorded, full test
suite green, robot docked and charging, area around the arm clear, no agent session running (deploying
mcp_server kills it; ask first). Telemetry only through topic_scraper_api /
`scripts/topic_scraper_collect.py` and the web UI, never rclpy probes on the client. Logs go to `.logs/`.

1. Deploy (ansible-deploy skill): `./scripts/deploy-nodes.sh client head_rgbd_calibration
   head_state_publisher mcp_server web_ui base_servos 2>&1 | tee .logs/deploy-head-arm-calib.log`.
   Services must start without `ModuleNotFoundError`.
2. Tilt range: `set_head_angles` tilt 30, 40, 50 and the recorded limit at pan 0, slowly; nothing
   touches. Record.
2a. Board mount (D47): compare the board pose measured in the Hardware section with `fit.board_mount`.
   Within 1 cm / 5 deg: write the measurement into `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
   and the client.yml block (comment "measured <date>"). Outside it: same replacement, then rerun
   `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q`
   and `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`; a failing safety test stops
   the run (the pose set needs re-planning for that mount). Commit, redeploy head_rgbd_calibration.
2b. Dry run without the board: Move to base pose and every tier (press Board attached with nothing
   clipped), the user confirming each dialog and stopping at any pose close to the body, head or floor.
   Record each stop. Park. Then clip the board; check it for slip after the dry run.
3. Calibrate page, robot docked, user watching: Move to base pose, user clips the board, tiers 0 and 1,
   ask before "Allow elbow and shoulder moves", record views, arm poses, skipped reasons, RMS, held-out
   RMS, board error, each parameter with sigma and its change from the applied floor chain, and the
   floor cross-check. The user removes the board and confirms Park.
4. Apply a passed result; the units restart. Check TF with topic_scraper_api (enable it first) and the
   web UI head pose; `/ground/state` at home stays within 0.01 m height.
5. Live yaw and x/y check: arm at the base pose without the board, `head_look_at` on the tool point;
   the tool point's pixel offset from the image centre in the head camera image (web UI), converted to
   deg, before (CAD chain) and after. Record.
6. Note: the arm joint offset solve belongs to epic arm-joint-offsets-from-the-g.

Retry notes: speed_scale stays 0.1 (fragile mount). A pose refused by the path check is recorded as
skipped with its reason, not forced. After a long hold (about 4 h) the gripper servo can drop off the
bus under the board load: park with the board (D63) and unclip between sessions.

## Results

Run 2026-10-23 by the orchestrator with the user (session `calib-20261023-074418`). This section was
written from the run's record. Rows marked "BLOCKED" need a robot rerun with the user; no value was captured for them and none is guessed.

| Step | Result | Notes |
|---|---|---|
| Full suite green | pass 2026-10-23 | root pytest, lint, lint-nodes, test-ansible, and the head_rgbd_calibration, mcp_server and web_ui pytest suites all exit 0 (last web_ui run: 1080 passed); frontend tsc/npm checks not run here; log `.logs/full-suite.log` |
| 1 Deploy | pass | services started without ModuleNotFoundError (user-observed); the deploy log file was not kept, so this row rests on the user's observation |
| 2 Tilt range | BLOCKED: robot rerun needed | pinned limit is 45 deg (Hardware section) |
| 2a Board mount | done | `fit.board_mount` is the user's 2026-10-23 measurement (yaml comment "measured by the user 2026-10-23") |
| 2b Dry run | BLOCKED: robot rerun needed | per-pose stops were not captured |
| 3 Head from arm | pass | see numbers below |
| 4 Apply | pass, applied 11:52 | head_state_publisher, head_camera_cpp, head_rgbd_cpp restarted; `/ground/state` height delta BLOCKED: robot rerun needed |
| 5 Yaw and x/y pixel offset | BLOCKED: robot rerun needed | before/after offsets were not captured |
| 6 Arm joint offsets | note | belongs to epic arm-joint-offsets-from-the-g |

Calibration session `calib-20261023-074418`: speed_scale 0.1, board mass 214 g, robot docked.
- 113 views, 29 arm poses. Tier 2 pose 10 (arm pose 29) refused by the path check: board point 45.9 mm
  below the 50 mm `min_board_height_m` (the board sags about 4 mm under load).
- First compute `r-20261023-075643` FAILED (rms 12.2 px). Pan offsets of +-10 deg gave 10-13 px against
  2.5-3 px. Cause: the head_pan origin sits 38 mm back and 11 mm right of the URDF. Fix: fit
  `pan_axis_x/y` (commit b4aafbaf) and floor prior weight 10 (user decision D64).
- Recompute `r-20261023-112701` PASSED: rms 2.93 px, held-out 2.88 px / 2.86 mm, largest floor delta
  1.43 deg.
- Parameters: mount x 0.1655 m, y 0.1107 m, z 0.2456 m; roll 1.114 deg, pitch 1.452 deg, yaw 0.473 deg;
  tilt_zero -6.495 deg; tilt_roll 0.312 deg; pan_axis x -0.0367 m, y -0.0105 m. Per-parameter sigma: BLOCKED,
  read it from stored result `r-20261023-112701` on the robot.
- Applied 11:52; survives reboot.
- Incident: after about 4 h holding the board (load 200) gripper servo 6 dropped off the bus and the
  robot was power-cycled; the arm returned to rest. Park with the board (D63) was added and reviewed.

## Scene (for the ground tasks, not the arm sequence)

- Box: 5 x 25 x 18 cm.
- Ramp: 6 cm high; its other dimensions are the same as the board's.
