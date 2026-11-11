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
     up to the shoulder (the finger tip stops there). The sequence then closes the gripper to `grip_rad` 0.0751 rad
     (`handle_contact_rad` 0.0951 minus squeeze 0.02), refuses a jaw below 0.0851 rad (contact - squeeze / 2) as
     unseated and a jaw at or above 0.1051 rad (grip + converge tolerance 0.03) as a slack grip. The contact 0.0951 rad
     was measured on the robot 2026-11-09 (CAD 0.070279): the printed handle is thicker than the CAD.
   - Mass: 211.7 g (CAD). The arm carries it at `speed_scale` 0.1.
   - Board frame pose in `gripper_link`: the CAD seed `fit.board_mount` xyz (-0.0794, 0.187949, -0.118113) m, rpy
     (pi, 0, 0) from `markers/charuco-arm-6x5-28mm.json`; the fit refines it, and the pose check adds
     `provisional_margin_m` (0, user decision 2026-11-10) while the mount is the seed.
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
2b. Dry run without the board: Open gripper (step 2) and press Board attached with nothing in the jaws: the grip
   close must stop the sequence with "the gripper closed past the handle contact: seat the handle up to the shoulder
   and start again" (the seat check), holding the arm (phase `holding`). Record the measured angle it quotes. Home (step 7).
2c. Grip close with the board (gripper only, decision 264): Open gripper (step 2), seat the handle up to the shoulder,
   press Board attached. The gripper closes to `grip_rad`; the sequence must pass the seat check (measured at least
   `handle_contact_rad - handle_squeeze_rad / 2`) and the slack check (measured below `grip_rad +
   grip_converge_tolerance_rad`; a thicker handle stops it, then set `handle_contact_rad` from the measured contact) and
   start the still session. Stop. Record the measured gripper
   angle (from the state or `get_arm_state`) and take detection stills of the board at the base pose. The board must
   not slip or rotate in the jaws.
3. Calibrate page, tab **Arm to camera** (deep link `#/calibrate/arm-camera`), robot docked, user watching. The
   precondition banner must not show "Not ready" (a camera-to-ground result, i.e. an applied head chain, is needed) or
   "Arm zero is stale" (after an accidental re-zero restore it in Servo calibration, Advanced / repair; after a servo
   calibration store the arm-to-camera calibration must be redone: an info line (not a warning) lists the per-joint
   zero shifts and every manual arm operation stays enabled, with no confirm: Home arm and Open gripper here, the Arm
   tab Arm home and Home, the Calibrate General Home arm and Move to neutral; with an unreadable stale marker the
   banner says so: fix the marker, never delete it before the redo; see the feetech_servos README). Steps: 1 **Home arm** (confirm), 2 **Open gripper**
   (confirm naming the joints: base pose, gripper open to `insert_gripper_rad`), 3 the user seats the ARM board handle
   in the jaws up to the shoulder and presses **Board attached**: the gripper closes on the handle (seat and slack
   checks in the events log), tiers 0 and 1 start, 4 **Capture**: ask before **Allow shoulder and elbow moves**
   (confirm) or **Skip them**; record views, arm poses and skipped reasons. The head-from-arm fit of this procedure is
   in step 5 under **More fits on this session** (**Compute head from arm**): record RMS, held-out RMS, board error,
   each parameter with sigma and its change from the applied floor chain, and the floor cross-check. **Abort
   calibration** in the General section stops the sequence at any time and holds the arm where it is (no Home move).
4. **Apply head from arm** on a passed result; the units restart. The arm-to-camera fit itself is step 5 **Compute**
   and step 6 **Check and accept** (Accept confirms and applies, Decline keeps the current calibration; Accept is off
   with the reasons when the report is not plausible); then step 7 **Home** (confirm: the arm moves to the stored home with the board), the user removes the board.
   Check TF with topic_scraper_api (enable it first) and the
   web UI head pose; `/ground/state` at home stays within 0.01 m height.
5. Live yaw and x/y check: arm at the base pose without the board, `head_look_at` on the tool point;
   the tool point's pixel offset from the image centre in the head camera image (web UI), converted to
   deg, before (CAD chain) and after. Record.
6. Note: the arm joint offset solve belongs to epic arm-joint-offsets-from-the-g.

Retry notes: speed_scale stays 0.1 (fragile mount). A pose refused by the path check is recorded as
skipped with its reason, not forced. After a long hold (about 4 h) the gripper servo can drop off the
bus under the board load: park with the board (D63) and take the board out of the jaws between sessions. A seat-check
refusal ("closed past the handle contact") means the post was not in the jaws up to the shoulder. The sequence ends in
phase `holding` (the board counts as attached): take the board out of the jaws, press Home (step 7, open whenever the
arm holds the board; the checked board Home),
then Open gripper again (step 2: base pose, gripper at `insert_gripper_rad`), reseat the handle and retry. A page that
lost the sequence (restart) offers **Home (board is on the gripper)** in step 1.

Board record: from Board attached until a completed Home the node keeps
`/var/lib/ros2/head_rgbd_calibration/head_arm_board.json`, and mcp_server refuses `/arm/home` (Home arm everywhere) while
it exists, whatever its age; the refusal names the file and both ways out. The Calibrate page shows the record in the
General section and above the Arm to camera steps, with its session (or "without a capture session"), attach time and
age, and disables Home arm while it exists. Board still on the gripper: **Home (board is on the gripper)** (step 1) or
**Home** (step 7 while holding). Board already off (taken out by hand without a Home): **The board is already off
the gripper: clear the record** and confirm. Nothing moves; the node removes the file, logs a `board_record_clear`
calib_event (run id `board-...`, the removed record in `data.record`) and a recent event, and a `holding` sequence
returns to idle. The clear is refused while a head-arm sequence or Home or an arm to camera compute runs. Never delete
the file by hand on the robot while the board may still be in the jaws.

Where errors show: a sequence or Home that ends with an error shows it on the Calibrate page in the red banner
**Last head-arm error** (Arm to camera flow, above its steps; a refused button shows its message on its own step) with the time and phase it happened
in, and the **Recent head-arm events** log below it lists the newest 50 events (start, each step and pose outcome,
errors, stops, Home results), newest first. Each of these events is one `calib_event` line in the node journal (errors at
ERROR, stops at WARNING), next to the step telemetry the page does not show; read it through the deploy tooling, never
an ad-hoc probe (see "Reading a failed run from the journal" below).
Read the message before retrying; never re-record the servo neutral to make a failing Home converge (incident
2026-11-09, history: a neutral re-recorded in the folded pose shifted elbow_flex by +90.5 deg and Park, now Home, could no longer converge;
the bridge now refuses such a save without the explicit large-shift confirm, `docs/runbooks/servo-torque-neutral.md`).

### Grip contact does not match the board

Board attached stops with "the gripper closed past the handle contact" (unseated) or "the gripper stopped short of the
commanded grip" (slack) although the handle is seated up to the shoulder: this board's handle is a little thinner or
thicker than the configured `handle_contact_rad`. Nothing needs editing in the config.

1. Seat the handle up to the shoulder in the jaws by hand and check it by eye. After these two errors the sequence is `holding` with the board on (the board was confirmed before the grip check); step 3 Grab board is locked then (Board attached works only in `waiting_board`), so the only reachable button is the one in the banner.
2. Press **Confirm correct grip** (in the red **Last head-arm error** banner for these two errors, where its refusal is also shown; step 3 Grab
   board and step 7 Home have it too, but step 3 only while its run is live) and confirm "The handle is seated correctly: record the measured gripper angle as the grip on
   this board (nothing moves)". The node measures the jaw, records it as the handle contact in
   `<state_dir>/head_arm_grip.json` (kept across restarts and deploys, never templated by Ansible) and answers with the
   new grip. It refuses a contact outside 0.03 to 0.30 rad and any call while something moves.
3. The line under Grab board and Home reads "Handle contact <rad> rad (recorded at <time>)"; "(config value)" means no
   record. After a confirm from the banner the red banner stays (the node keeps the error) and a green "Grip recorded:
   handle contact <rad> rad" line appears next to its button. To get a fresh grip, press Home, then Open gripper and
   Board attached. Resume capture (board on) keeps the jaw as it is and skips the seat and slack checks.
4. **Reset to config value** (shown only while a record exists, own confirm, nothing moves) deletes the record and goes
   back to the configured contact, e.g. for a different board. Both buttons stay disabled while the sequence runs or a
   call is busy.

A recorded contact is only as good as the seat: confirm only with the handle really seated, otherwise the unseated check
is blind for this board.

### Reading a failed run from the journal

Every calibration flow of the node logs structured `calib_event {json}` lines to the system journal of the unit
`ros2-head_rgbd_calibration` (keys `flow`, `run_id`, `step`, `outcome`, `reason`, `data`, `duration_s`; the full event
list per flow is in `nodes/head_rgbd_calibration/README.md`, section Telemetry). To find out where and why a head-arm
run went wrong:

1. List the runs and their ends: `journalctl -u ros2-head_rgbd_calibration -o cat --since today | grep -o
   'calib_event {.*' | cut -d' ' -f2- | jq -c 'select(.flow=="head_arm" and .step=="sequence")'`. The `error` event
   names `failed_step`, the `reason` (the same text as the red banner), the phase it ended in, the session id and the
   counts; a `home` event does the same for a Home (`run_id` `home-...`).
2. Take that `run_id` and read the whole run in order: `... | jq -c 'select(.run_id=="seq-...")'`. The last `error`
   event of a timed step (`base_move`, `handle_grip`, `pose`, `return_to_base`, `home_move`, `home_release`, ...) carries
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
   - `home_path_check` `refused`: the mount used (`fitted`, `recorded` or `configured`), the margins and the entries.
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
  robot was power-cycled; the arm returned to rest. Park with the board (D63) was added and reviewed (history; the flow button is now Home).

## Arm to camera calibration (T-99)

Calibrates the arm in the head camera's frame with the head chain held fixed: arm joint offsets (shoulder_pan
included), the arm base x, y, z in base_link, the board seat in the gripper and the board sag. API, report fields and
gauges: `nodes/head_rgbd_calibration/README.md` ("Arm to camera calibration").

1. Precondition: a head calibration is applied (floor fit or a passed head-from-arm result). `GET
   /api/arm-camera/state` must say `ready: true`; `head_chain_source` names which chain is held fixed. Without one the
   compute is refused (409) with "apply a head calibration first".
2. Capture: the same head-arm sequence as steps 3 above (Move to base pose, seat the ARM board, Board attached, tiers,
   ask before "Allow elbow and shoulder moves", Home). No extra arm moves: the fit only reads the stored views.
3. Compute: `POST /api/arm-camera/sessions/{sid}/compute` (202 `{result_id}`), then poll `GET
   .../results/{rid}` until `state` is `done` or `failed` (a few seconds on a laptop, longer on the Pi 5; the child is
   killed after `compute_timeout_s`).
4. Read the report: `plausible` false means a refusal (an offset change over 15 deg, the base moved over 100 mm or
   turned over 10 deg, the board seat over 10 mm / 10 deg, the sag over 5 deg): the result cannot be accepted; check
   the board seating and the applied head chain, then capture again. `warnings` are today's limits as advice (RMS,
   held-out RMS, held-out tool error, view and pose counts). Compare `score` with `comparison.current_score` (the
   offsets and base the robot runs with, the accepted calibration or else the configured one, on the same views) and look at `accuracy_examples` (1 sigma and 95 % at 30 cm reach,
   full reach and 50 cm in front of the head camera) and `comparison.shift_examples` (how far the tool point moves).
5. Accept (`POST .../accept`) applies the calibration to the robot; decline (`POST .../decline`) keeps the one in use.
   Accept writes `applied/arm_calibration.json` and then applied.json (`arm_calibration`); mcp_server picks it up on its
   next monitor tick and swaps the joint offsets, arm base and mount height into the running arm model as soon as no
   arm motion runs (no restart, no client.yml edit, no redeploy). The accept is refused (409) when the arm zero may
   have changed at or after the session's first still: the follower stale marker lists `joint_offsets` for a re-zero
   then, `/var/lib/ros2/arm/zero_changed.json` names a neutral save, zero restore or `calibrate_servos.py` zero change then,
   or either file is unreadable.
   Capture a new session in the current zero.
   After an arm servo calibration store the marker carries `servo_calibration_at` and `joint_offsets` stays listed
   until this accept; the accept clears `joint_offsets` and the field. Zero restore stays refused afterwards (the
   `zero_changed.json` record still names the servo calibration) until the next neutral save or CLI zero change.
6. Confirm the runtime picked it up: `get_arm_state` shows `arm_calibration` with the accepted session and result, and
   the mcp_server journal has a `calib_event` flow `arm_calibration` step `load` ok with `old` and `new` values
   (`apply` start first if a motion was running; `stale_marker` ok if joint_offsets was cleared; `load` failed means
   the values in use were kept, the reason says why):

   ```bash
   journalctl -u ros2-mcp_server -o cat | grep -o 'calib_event {.*' | cut -d' ' -f2- \
     | jq -c 'select(.flow=="arm_calibration")'
   ```

7. Verify: a live tool point check as step 5 above (head_look_at on the tool point, pixel offset before and after).
8. Follow-ups the runtime apply does not cover: the approach tools refuse with "reach map is stale" until the reach map
   is rebuilt for the new arm. filter_node's teleop body check and the web_ui arm views follow the accepted calibration
   at runtime and fall back to the client.yml baseline when the record is removed. To make the accepted values the
   baseline, copy `calibration.snippet` (`joint_offsets_rad`, `arm_base_height_m`, `base_in_base_link`) into the
   client.yml mcp_server `arm:` block, filter_node's `self_collision_guard.arm` and web_ui's `arm_calibration`
   (`tests/test_arm_guard_config.py` pins the three equal), the head_rgbd_calibration `fit.joint_offsets_rad` and
   `fit.base_in_base_link` (and its shipped YAML), rebuild the reach map (`nodes/mcp_server/README.md`, "Reach map"), run the config tests
   (`tests/test_head_arm_calib_config.py`, the node `tests/test_config.py`), commit and redeploy (ansible-deploy skill):
   `./scripts/deploy-nodes.sh client mcp_server head_rgbd_calibration web_ui filter_node 2>&1 | tee .logs/deploy-arm-camera.log`.

Journal (through the deploy tooling): the flows `arm_camera_compute` (inputs, solver, one `refusal` / `warning` event
each with value and threshold, result) and `arm_camera_accept` (accept, decline), with the result id as run id:

```bash
journalctl -u ros2-head_rgbd_calibration -o cat | grep -o 'calib_event {.*' | cut -d' ' -f2- \
  | jq -c 'select(.flow=="arm_camera_compute" or .flow=="arm_camera_accept")'
```

## Scene (for the ground tasks, not the arm sequence)

- Box: 5 x 25 x 18 cm.
- Ramp: 6 cm high; its other dimensions are the same as the board's.
