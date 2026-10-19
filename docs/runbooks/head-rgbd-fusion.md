# Runbook: head RGB-D fusion on the robot

Run from the dev machine against the client (Pi 5, `client.ros2.lan`). Deploys only through `scripts/deploy-nodes.sh`
(ansible-deploy skill), logs in `.logs/`. ROS commands on the client need
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Head motion: only the pan sweep (+-20 deg at
about 20 deg/s, inside the pan/tilt epic's verified range). CPU is read in Grafana, never with `top` over SSH.
Limits and metric definitions: `docs/head-rgbd-fusion-methodology.md` (`### Results` is filled from steps 2-7).

## Steps

1. Deploy `./scripts/deploy-nodes.sh client arducam_tof head_camera_cpp head_rgbd_cpp`. head_rgbd_cpp has no ToF
   calibration yet, so `ros2 topic hz /head_rgbd/depth/image_raw` and `/head_rgbd/points` must print nothing and
   `head_rgbd_pairs_published_total` stays 0 (placeholder check, part 1).
2. Warm up the ToF 20 minutes. Print the ChArUco board (6x5, 40 mm squares): `uv run python -m scripts.markers_generate`,
   PDF at 100 % on matte laser paper on a rigid plate (or the two-colour STL/3MF); one square 40 +- 0.2 mm with a caliper.
   Capture and compute run on the robot from the web UI: open the hamburger menu, the Calibrate page, its RGB-D section, and
   Start capture. Hold 20-30 poses 0.25-0.45 m from the head (the 240x180 ToF only resolves the markers this close),
   with tilts and in the image corners; rest the board on a support for about 3 s per pose (the mapping stream is 2 Hz
   and a hand-held board never passes the stillness check). Watch the status line, the pose count and the RGB and ToF
   coverage grids (fill the empty cells, corners first); delete bad poses in the list. Stop, then Compute. Read the
   report against the limits: RGB <= 0.5 px, ToF <= 0.3 px, ToF-to-RGB extrinsic RMS <= 1.0 px, plane bias <= 15 mm;
   when one fails, recapture or prune the worst poses and Compute again. Nothing is applied until step 3.
3. Apply in the same section: it writes the result to `/var/lib/ros2/head_rgbd_calibration/applied` and restarts
   head_camera_cpp, head_rgbd_cpp and head_state_publisher; the section shows each unit restarted (`tof_joint` in the
   URDF is rewritten by the same Apply). `camera_info_file` and `tof_calibration_file` already point at the applied
   state files, so no group_vars edit or redeploy is needed. Confirm `/head_rgbd/depth/image_raw` publishes at the mapping
   rate with a subscriber, and that the head camera's focus is continuous again once the capture stopped and no
   subscriber is left (U4). Optionally Download the zip and commit its files to
   `nodes/head_camera_cpp/config/head_camera_calibration.yaml`, `nodes/head_rgbd_cpp/config/tof_calibration.yaml`,
   `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml` and `nodes/web_ui/urdf/head.urdf`: the repo files only seed
   `applied/` on a reflashed robot. The CLI (`scripts/head_rgbd_calibrate.py`) stays for bags, validation captures
   and offline calibration.
4. Time offset: `capture-validation sweep` while panning +-20 deg at about 20 deg/s; `validate` gives the ToF-RGB offset;
   write it into `arducam_tof.stamp_latency_ms` in client.yml, redeploy arducam_tof, repeat the sweep: |offset| <= 10 ms.
5. Depth accuracy: matte target perpendicular at 0.5, 1, 2, 3 m (tape reference), `capture-validation 0.5m|1.0m|2.0m|3.0m`
   each; `validate`: raw ToF bias/RMSE and registration bias. A residual bias > 1 cm is recorded as a finding with numbers
   (the design's correction rule).
6. Alignment and coverage: board 1 m in front of a wall >= 2.5 m (`capture-validation wall`): median edge alignment <= 1.0 px
   at 288x162; valid ratio >= 0.85 on a flat wall at 1.5 m; motion: the sweep with compensation on gives median alignment
   <= 1.5 px and the dropped-pair counter rises only above the rate limit.
7. CPU: `mapping.rate_hz` temporarily 10, a subscriber on `/head_rgbd/depth/image_raw`, 10 minutes in Grafana (cAdvisor
   `container_cpu_usage_seconds_total` for `ros2-head_rgbd_cpp.service`; `rate(process_cpu_seconds_total)` of the node):
   p95 <= 0.2 core with upsampling off; repeat with upsampling on; record head_camera_cpp's CPU delta; set `memory_max` to
   >= 2x the measured peak; restore `mapping.rate_hz` 2.
8. Placeholder check part 2: stop arducam_tof, then stop head_state_publisher: `/head_rgbd/*` publishes nothing and the drop
   counters name the reason (`no_tf`, `bad_input`); restart both.

## Results

Executed 2026-10-19 from branch `head-rgb-d-fusion-of-camera--rgbd-final-suite-and-runbook` (deployed revision: origin/main).
Status legend: PASS, FAIL, NOT RUN (needs a person at the robot).

| Step | Result | Evidence |
|---|---|---|
| Full suite (verify command) | PASS: 1571 pytest passed, ruff/vulture clean, ansible lint+syntax clean, C++ tests 126 -> 0 failures after the node-test race fix, lint-cpp, arducam_tof and web_ui tests/lint clean | `.logs/suite.log`, `.logs/suite2.log`, `.logs/cpp.log` |
| 1 Deploy arducam_tof head_camera_cpp head_rgbd_cpp | PASS: ok=160 changed=22 failed=0, 8 min 10 s; the three services active | `.logs/deploy-client-step1.log` |
| 1 Placeholder check part 1 | PASS: `ros2 topic hz` on `/head_rgbd/depth/image_raw` and `/head_rgbd/points` printed nothing in 10 s; `head_rgbd_pairs_published_total 0`, all drop counters 0 (no ToF calibration, no subscriber) | `.logs/step1-placeholder.log` |
| 2 Warm-up, board print, capture, calibrate | NOT RUN: needs a person to print and mount the board, caliper a square and hold 20-30 poses | - |
| 3 URDF, redeploy, mapping rate, focus (U4) | NOT RUN: depends on step 2 calibration files | - |
| 4 Time offset sweep | NOT RUN: depends on step 3 | - |
| 5 Depth accuracy 0.5/1/2/3 m | NOT RUN: needs a target and tape reference placed by a person | - |
| 6 Alignment, coverage, motion | NOT RUN: needs board and wall setup | - |
| 7 CPU in Grafana | NOT RUN: no calibration, so the node does no work to measure | - |
| 8 Placeholder check part 2 | NOT RUN: meaningful only once calibrated (otherwise `no_calibration` masks the reasons) | - |

Blocker: steps 2-8 need a person at the robot (printed board, caliper, targets at tape-measured distances, a wall).
No agent can clear that. The methodology `### Results` table stays empty until those numbers exist.
