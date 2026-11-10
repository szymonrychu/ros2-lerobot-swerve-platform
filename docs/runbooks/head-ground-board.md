# Runbook: head ground board calibration (camera to ground)

The head chain (camera to ground: head mount z, roll, pitch, tilt zero and roll, the D435 mount x, z, roll, yaw and the
pan axis x, y) solved from the big 40 mm FULL ChArUco board (`charuco-6x5-40mm`) lying flat on the floor (T-99 part
D). The head sweeps around the board at one to three board placements; the colour views fit the chain, the depth floor
planes of the same poses cross-check it, and the user accepts or declines the result on its score and estimated
accuracy. After an accept a verification visits three head poses and reads `/ground/state`. Model, API and report
fields: `nodes/head_rgbd_calibration/README.md` ("Ground board calibration"); fit internals:
`shared/README.md` ("Ground board fit").

Run from the dev machine against the client (Pi 5, `client.ros2.lan`). Deploys only through
`scripts/deploy-nodes.sh` (never `ansible-playbook`). Related: `docs/runbooks/head-floor-fit.md` (the acceptance
checks reused by the verification), `docs/runbooks/ground-estimation.md`, `docs/runbooks/head-arm-calibration.md`.

Servo safety: only the head moves (pan and tilt, at most `ground_board.max_tilt_deg` 45 deg down); no arm or wheel joint
moves. Every exit (done, stop, error) commands the head back to where it was at start.

## Preconditions

- The robot is on a flat floor and not on blocks; the board lies flat (no curl, no wrinkle) on that floor.
- The RGB-D calibration is applied with an accepted D435 intrinsics check (an accept without it is refused with 409).
- Nothing else of the calibration node runs: an RGB-D capture or compute, a head-arm sequence, a floor sweep, an arm to
  camera compute (the API answers 409 naming what runs).
- `head_rgbd_calibration` with the `ground_board` config block is deployed (Deploy below).
- For the verification: `ground_estimator_cpp` runs and publishes `/ground/state`.

## Board placement

The fit assumes nothing about where the board lies, only that it is:

- flat on the floor, printed side up;
- close: within about 0.4-1.0 m in front of the head (a board farther than `bounds.max_board_distance_m` 2.5 m refuses
  the result);
- fully visible in the camera preview with the head homed (pan 0, tilt 30 deg down; at least `home_min_corners` 16 of
  the 20 corners must be detected there, else the placement fails with `board not detected at home ...`).

Use two or three placements with clearly different positions and turns (e.g. left and turned 20 deg, right and turned
-35 deg, farther and turned 70 deg): one placement fits but warns `min_placements`, and different turns separate the
head roll and the camera roll better.

## Deploy

Invoke the `ansible-deploy` skill for the exact command first, then (all output to `.logs/`):

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client head_rgbd_calibration web_ui 2>&1 | tee .logs/deploy-client-head-ground-board.log
```

Expect `failed=0` and the units `active`.

## Steps in the web UI

Calibrate page (hamburger menu, `#/calibrate/camera-ground`), tab **Camera to ground**: 1 D435 intrinsics check
(accept one first if none is accepted: the ground board accept needs it), 3 **Home camera**, 4 lay the board while
watching the head camera preview, **Board is down**, 5 **Start sweep**, after each placement move the board and press
**Next placement** (or **Stop**), 6 **Compute**, 7 the quality gate (refusals, warnings, accuracy at 1 sigma and 95 %,
the parameters with the held head mount gauge; **Accept** or **Decline**), 8 **Start verification** (per pose pass or
fail with the medians against the tolerances). The depth floor fit stays under **More fits**. The same steps through
the API follow.

Step 2 (Front/back cameras, CSI intrinsics, optional) offers only the CSI cameras whose node is enabled
(`GET /api/calibration/rgbd/csi`; `csi_cameras.<name>.enabled` mirrors the `<name>_camera` ros2_nodes entry, so the
back camera is not offered while back_camera is disabled). When a capture gets no frame within `csi_frame_timeout_s`
(10 s) the node stops it and the step says "Camera not detected: the Front camera published no frames. Check the ribbon
cable and reboot the robot, then start the capture again." (journal: `front_camera` logs "camera ... not found" and
the kernel "imx708 ... probe ... failed" when the sensor does not answer on the CSI bus). Reseat the ribbon with the
robot powered off; no software step brings the sensor back without a reboot.

## Steps (API through the web_ui proxy)

`API=http://client.ros2.lan:8080/api/calibration/rgbd/ground-board` (web_ui forwards to the node's `/api/ground-board`).

1. Lay the board (placement 1) and check the camera preview at the home pose.
2. `curl -X POST $API/sweep/start`. Poll `curl $API/state | jq .sweep`: `homing`, `locating` (the board located on
   the floor through the applied chain, `board` shows x, y, yaw), `sweeping` (`pose` of `total`, `views`, `planes`),
   then `awaiting_placement` with the head back home. A failed placement shows its reason in `message`; fix it (move
   the board closer or into view) and continue.
3. Move the board to placement 2, `curl -X POST $API/sweep/next-placement`; repeat for placement 3. After
   `max_placements` (3) the session ends by itself (`done`); `curl -X POST $API/sweep/stop` ends it earlier (an awaited
   placement is not taken). Without a next placement within `placement_wait_s` (30 min) the session ends.
4. `curl $API/sessions | jq '.[0]'` gives the session id (`ground-YYYYmmdd-HHMMSS`), its views, planes and skipped poses.
   `curl -X POST $API/sessions/<sid>/compute` answers 202 `{result_id}`; poll
   `curl $API/sessions/<sid>/results/<rid> | jq .state` until `done` (a minute or more on the Pi 5 for a full three-placement session).
5. Read the report (`jq .report`):
   - `plausible` false: the result is refused outright (`refusals`: a mount move over 100 mm, a mount, tilt or camera
     turn over 10 deg, a camera move over 30 mm, a pan axis move over 50 mm, a board over 2.5 m away, a non-finite
     number). It cannot be accepted: check the placements and capture again.
   - `warnings` are advisory (RMS, hold-out RMS, sigmas, view, pose and placement counts, pan and tilt spans, the depth
     cross-check). The decision is the user's.
   - `score` (`rms_px`, `heldout_rms_px`, the depth cross-check `floor_height_rms_mm` and `floor_normal_rms_deg`) and
     `comparison.current_score` (the applied chain on the same views): a fit clearly better than the applied chain and
     a hold-out RMS close to the fit RMS are the good signs.
   - `accuracy_examples`: the floor height 1 m and 2 m ahead (mm), an obstacle 2 m ahead (lateral and range, mm; the lateral one excludes the head heading, which this fit keeps as applied), the
     head pitch and roll to the floor (deg), each at 1 sigma and 95 %; `comparison.shift_examples` says how much the
     new chain moves each of them against the applied one.
6. Accept: `curl -X POST $API/sessions/<sid>/results/<rid>/accept` (applies the head chain, restarts the
   `restart_units`), or Decline: `curl -X POST $API/sessions/<sid>/results/<rid>/decline` (nothing applied changes).
7. Verification after an accept: `curl -X POST $API/verify/start`, poll `curl $API/verify/state`. It waits for the
   restarted units, then at home, pan 40 / tilt 20 and pan -40 / tilt 25 collects `/ground/state` while the head is
   still. A pose passes with the median plane height within 1 cm of 0, `plane_vs_base_deg` under 0.6, under 10
   obstacle points and no drop points (the floor fit acceptance checks). The floor around the robot must be clear for
   this step (take the board away): the board itself is a 4 mm step but its edges can count as obstacle points. The
   summary is stored with the accepted result (`accepted.verification` in `GET $API/state`).

## Telemetry

Every step is a `calib_event` line in the node journal (flows `ground_sweep`, `ground_compute`, `ground_accept`,
`ground_verify`):

```bash
journalctl -u ros2-head_rgbd_calibration -o cat --since today | grep -o 'calib_event {.*' | cut -d' ' -f2- \
  | jq -c 'select(.flow | startswith("ground_"))'
```

## Undo

The next accept of any head chain source (a floor fit, a head-arm or arm offsets fit, another ground board result)
replaces the applied chain. A ground board result can only be accepted while the chain it was fitted against is still
applied, so to undo, run the source of the previous chain again (e.g. a new floor fit or a new ground board session)
rather than re-accepting an old ground result.
