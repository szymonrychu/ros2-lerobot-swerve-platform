# Head-from-arm calibration runbook

Calibrates the head chain against the trusted arm with the ARM ChArUco board (6x5, 28 mm squares) held in the gripper
by its edge handle (design D31-D51 of drivable-ground-tof-filterin; board: arm-calib-board, T-77). This file starts with the hardware facts only the
user can supply; the pose set (arm-board-pose-set) and the tilt range (head-tilt-range-widened)
read them.

## Hardware

Recorded 2026-10-23, all values given by the user. Anything not listed here was not given and must
not be filled in by an agent.

1. Board and handle (2026-11-08, ARM board; geometry from the CAD, design choices by the user, decisions 260-297)
   - Board: the ARM ChArUco board `charuco-arm-6x5-28mm` (`models/charuco-arm-6x5-28mm.3mf` / `.pdf`, 178 x 150 mm
     plate, marker ids 15-29), printed in one piece, back down, with its edge grip handle (`docs/markers.md`,
     "Calibration boards"). The FULL board (`charuco-6x5-40mm`, ids 0-14) stays the intrinsics board.
   - Handle: a 10 mm tab off the middle of the long bottom edge, the grip post on the tab on the pattern side, one
     gusset. The gripper holds the post from the pattern side, the jaws closing along the plate edge; the pattern faces
     the wrist and the head camera.
   - Seat: insert the post between the half open jaws (`insert_gripper_rad` 0.4) with the moving finger in the channel,
     up to the shoulder (the finger tip stops there). The sequence then closes the gripper to `grip_rad` 0.050279 rad
     (contact 0.070279 minus squeeze 0.02) and refuses a jaw below 0.060279 rad as unseated.
   - Mass: 211.7 g (CAD). The arm carries it at `speed_scale` 0.1.
   - Board frame pose in `gripper_link`: the CAD seed `fit.board_mount` xyz (-0.0794, 0.187949, -0.118113) m, rpy
     (pi, 0, 0) from `markers/charuco-arm-6x5-28mm.json`; the fit refines it, and the pose check adds
     `provisional_margin_m` while the mount is the seed.
   - Earlier mount (2026-10-23, retired, the session `calib-20261023-074418` below): the 6x5 40 mm board glued at the
     middle of its long edge to a stick clamped in the gripper with a rubber band, print toward the arm, lower edge
     about 20 mm past the grip tip, 214 g.
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
2a. Board mount (D47): `fit.board_mount` is the CAD seed of the ARM board handle (Hardware section). If the
   generator's `markers/charuco-arm-6x5-28mm.json` changed, copy `board_mount`, `handle_in_board.tool_point_m`, the
   `grip` angles, `mass_g` and `handle_envelope_mm` into `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`,
   the node `config.py` constants, the mcp_server `calibration_board` defaults and the client.yml blocks, then rerun
   `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_arm_pose_set.py tests/test_config.py -q`
   and `uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`; a failing safety test stops
   the run (the pose set needs re-planning for that mount). Commit, redeploy head_rgbd_calibration.
2b. Dry run without the board: Move to base pose and press Board attached with nothing in the jaws: the grip
   close must stop the sequence with "the gripper closed past the handle contact: seat the handle up to the shoulder
   and start again" (the seat check), holding the arm (phase `holding`). Record the measured angle it quotes. Park.
2c. Grip close with the board (gripper only, decision 264): Move to base pose, seat the handle up to the shoulder,
   press Board attached. The gripper closes to `grip_rad`; the sequence must pass the seat check (measured at least
   `handle_contact_rad - handle_squeeze_rad / 2`) and the slack check (measured below `grip_rad +
   grip_converge_tolerance_rad`; a thicker handle stops it, then set `handle_contact_rad` from the measured contact) and
   start the still session. Stop. Record the measured gripper
   angle (from the state or `get_arm_state`) and take detection stills of the board at the base pose. The board must
   not slip or rotate in the jaws.
3. Calibrate page, robot docked, user watching: Move to base pose, user seats the ARM board handle in the jaws up to
   the shoulder and presses Board attached, the gripper closes on the handle (seat check), tiers 0 and 1,
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
bus under the board load: park with the board (D63) and take the board out of the jaws between sessions. A seat-check
refusal ("closed past the handle contact") means the post was not in the jaws up to the shoulder. The sequence ends in
phase `holding` (the board counts as attached): take the board out of the jaws, press Park (the checked board park),
then Move to base pose again (it opens the gripper to `insert_gripper_rad`), reseat the handle and retry.

Where errors show: a sequence or park that ends with an error shows it on the Calibrate page in the red banner
**Last head-arm error** (Head from arm section, and Arm calibration once opened) with the time and phase it happened
in, and the **Recent head-arm events** log below it lists the newest 50 events (start, each step and pose outcome,
errors, stops, park results), newest first. Each of these events is one `calib_event` line in the node journal (errors at
ERROR, stops at WARNING), next to the step telemetry the page does not show; read it through the deploy tooling, never
an ad-hoc probe (see "Reading a failed run from the journal" below).
Read the message before retrying; never re-record the servo neutral to make a failing Park converge (incident
2026-11-09: a neutral re-recorded in the folded pose shifted elbow_flex by +90.5 deg and Park could no longer converge;
the bridge now refuses such a save without the explicit large-shift confirm, `docs/runbooks/servo-torque-neutral.md`).

### Reading a failed run from the journal

Every calibration flow of the node logs structured `calib_event {json}` lines to the system journal of the unit
`ros2-head_rgbd_calibration` (keys `flow`, `run_id`, `step`, `outcome`, `reason`, `data`, `duration_s`; the full event
list per flow is in `nodes/head_rgbd_calibration/README.md`, section Telemetry). To find out where and why a head-arm
run went wrong:

1. List the runs and their ends: `journalctl -u ros2-head_rgbd_calibration -o cat --since today | grep -o
   'calib_event {.*' | cut -d' ' -f2- | jq -c 'select(.flow=="head_arm" and .step=="sequence")'`. The `error` event
   names `failed_step`, the `reason` (the same text as the red banner), the phase it ended in, the session id and the
   counts; a `park` event does the same for a park (`run_id` `park-...`).
2. Take that `run_id` and read the whole run in order: `... | jq -c 'select(.run_id=="seq-...")'`. The last `error`
   event of a timed step (`base_move`, `handle_grip`, `pose`, `return_to_base`, `park_move`, `park_release`, ...) carries
   the step's inputs and the exception.
3. Look at the step just before the failure:
   - `path_check` `refused`: `data.entries` names every violated rule with its value, limit and path fraction,
     `data.start` the measured joints and `data.target` the commanded pose (nothing moved).
   - `move_arm_joints` or `set_head_angles` with an outcome other than `converged`: `data.message` and `data.result`
     are mcp_server's answer (tracking error, positions); `tool_error` is a failed or unreachable tool call.
   - `gripper_measured` `unseated` or `slack`: `measured_rad` against `unseated_below_rad` and `slack_from_rad`.
   - `still` `skipped`: `reason` (no board, joints stale, not still) and `data.detail` (corner count, board ids, the
     window's colour MAD against `rgb_mad_limit`, the moving joint and its range, the joint stamp gaps).
   - `view` `skipped`: the head command and the violated view rule (min-Z, FOV, head limits).
   - `park_path_check` `refused`: the mount used (`fitted`, `recorded` or `configured`), the margins and the entries.
4. A compute that failed: `select(.flow=="head_arm_compute")` (or `arm_compute`, `floor_compute`, `d435_compute`) shows
   the `inputs` (views per tier and head offset, corner counts, intrinsics, floor reference) and the `result` with every
   limit check (`value`, `limit`, `ok`) and the worst views; an apply is `head_chain_apply` (files written, unit states).
5. The `api` flow lists every button press (POST) with its body and the refusal message of a 4xx answer.


Run 2026-10-23 by the orchestrator with the user (session `calib-20261023-074418`), with the retired stick-mounted
6x5 40 mm board (Hardware section, "Earlier mount"); the ARM board has no run recorded yet. This section was
written from the run's record. Every value below was read from the robot or the run record; none is guessed.

| Step | Result | Notes |
|---|---|---|
| Full suite green | pass 2026-10-23 | root pytest, lint, lint-nodes, test-ansible, and the head_rgbd_calibration, mcp_server and web_ui pytest suites all exit 0 (last web_ui run: 1080 passed); frontend tsc/npm checks not run here; log `.logs/full-suite.log` |
| 1 Deploy | pass | services started without ModuleNotFoundError (user-observed); the deploy log file was not kept, so this row rests on the user's observation |
| 2 Tilt range | moved to task head-tilt-range-widened | the user gave 45 deg down for the reprinted mount (Hardware section); the widened range ships and is checked in that task |
| 2a Board mount | done | `fit.board_mount` is the user's 2026-10-23 measurement (yaml comment "measured by the user 2026-10-23") |
| 2b Dry run | skipped by the user's choice | the board was already clipped (2026-10-23); instead every pose and path was re-checked by FK against the measured mount before any move (commit 4daf0e5d) |
| 3 Head from arm | pass | see numbers below |
| 4 Apply | pass, applied 11:52 | head_state_publisher, head_camera_cpp, head_rgbd_cpp restarted; `/ground/state` at home after apply: floor plane height median +0.006 m (p10 +0.001, p90 +0.013), tilt 1.40 deg, 40 states (`.logs/live/after_apply_home.ndjson`); within 0.01 m |
| 5 Yaw and x/y pixel offset | at the next live session with the user | needs an arm move to the base pose with the user present; recorded by retro-fixes-live-check |
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
  tilt_zero -6.495 deg; tilt_roll 0.312 deg; pan_axis x -0.0367 m, y -0.0105 m. Per-parameter sigma (stored result `r-20261023-112701`): mount x 0.25 mm, y 0.54 mm, z 0.53 mm;
  roll 0.056 deg, pitch 0.150 deg, yaw 0.107 deg; tilt_zero 0.160 deg; tilt_roll 0.051 deg; pan_axis x 0.25 mm,
  y 0.77 mm.
- Applied 11:52; survives reboot.
- Incident: after about 4 h holding the board (load 200) gripper servo 6 dropped off the bus and the
  robot was power-cycled; the arm returned to rest. Park with the board (D63) was added and reviewed.

## Scene (for the ground tasks, not the arm sequence)

- Box: 5 x 25 x 18 cm.
- Ramp: 6 cm high; its other dimensions are the same as the board's.
