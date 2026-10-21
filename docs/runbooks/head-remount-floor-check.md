# Runbook: head remount floor check

After the pan/tilt mount reprint the head ToF floor measurement is made trustworthy again: z-depth instead of radial
distance (arducam_tof, calibrated K), `inverted: true` on head_pan, a per-cell temporal median in ground_estimator_cpp
and a 10 cm ToF obstacle band in Nav2. This runbook deploys that, re-zeros the head, re-runs the head RGB-D calibration
and checks the floor numbers. Run from the dev machine against the client (Pi 5, `client.ros2.lan`); the agent drives the
checks, the user confirms the pan direction and runs the Calibrate page steps. Deploys only through
`scripts/deploy-nodes.sh` (never `ansible-playbook`). ROS commands on the client need
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Related: `docs/runbooks/ground-estimation.md`,
`docs/runbooks/head-rgbd-fusion.md`, `docs/runbooks/servo-torque-neutral.md`.

Servo safety: only the head moves, at most +0.3 rad pan, minimal amplitude; no arm joint moves.

## Preconditions

- The robot is free to move and not on blocks.
- A flat floor patch of at least 3 m x 2 m, nothing else in view of the head.
- The ChArUco board lies on the floor in front of the robot (the calibration target, also the costmap obstacle check).
- No agent session is running before mcp_server is deployed (deploying claude_agent/mcp_server kills a running session,
  exit 143): check the claude_agent session state, ask when one runs.
- `topic_scraper_api` is enabled for the client in `ansible/group_vars/client.yml` (`enabled: true`) and deployed, else
  port 18100 refuses connections (CLAUDE.md).

## Deploy

Invoke the `ansible-deploy` skill for the exact command, then (all output to `.logs/`):

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client arducam_tof head_rgbd_calibration base_servos ground_estimator_cpp nav2_bringup 2>&1 | tee .logs/deploy-client-remount.log
./scripts/deploy-nodes.sh client mcp_server 2>&1 | tee .logs/deploy-client-remount-mcp.log
```

head_rgbd_calibration is the apply unit (it restarts the head nodes on Apply). Expect `failed=0` and every unit
`active`. The second command runs only after the agent session check from Preconditions.

## Head pan direction

Command head pan +0.3 rad (head only, tilt unchanged at 30 deg down). The head must turn to the robot's left, CCW seen
from above. The user confirms the direction by looking at the robot; the camera image shifts to the right. Return the
pan to 0. If the head turns right, stop: head_pan `inverted: true` did not reach `base_servos`.

## Head re-zero

Calibrate page, Servo torque and neutral section, per `docs/runbooks/servo-torque-neutral.md` (head group: torque off,
turn the head straight by hand, Record neutral, Re-enable torque + save). Pass: `head_pan` and `head_tilt` within
0.005 rad of 0 in `/head/joint_states`, limits in `/swerve_drive/servo_registers` unchanged, torque 1.

## Head RGB-D recalibration

Calibrate page, per `docs/runbooks/head-rgbd-fusion.md` (capture the board poses, compute, Apply). With z-depth
(`/tof/depth/image_raw` = radial SDK distance divided by the ray norm of the calibrated K) the plane bias no longer
absorbs the radial-to-z factor: `plane_bias_mm_median` should be near 0, within +-10 mm. The `LimitsConfig`
`plane_max_bias_mm` limit of 40 still applies and rejects the calibration above it. Record RMS values and the bias.

## Floor numbers

Robot still on the flat floor, head at pan 0 and tilt 30 down, 60 s per collection with the topic_scraper_api workflow
(not raw `ros2 topic echo`):

```bash
python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /ground/state:.plane --interval 0.1
```

Run one collection each for `/ground/state`, `/ground/obstacle_points` and `/ground/drop_points`, and read
`ground_temporal_cells` and `ground_plane_vs_base_deg` on the metrics port 19119. Defaults in effect (from
`nodes/ground_estimator_cpp/config/ground_estimator.yaml`): cell_size_m 0.10, window_frames 8, min_frames 4.

- Plane height 0 +-1 cm.
- plane_vs_base < 0.6 deg.
- Zero drop points and zero obstacle points on the flat floor over 60 s.
- `ground_temporal_cells` non-zero.

## Costmap

The local costmap `tof_voxel_layer` marks points from 0.10 m (10 cm) above the floor (`tof_mark` in `nav2_params.yaml`,
the collision monitor `tof_obstacles` band is the same).

- Empty floor: the local costmap stays clear for 30 s (web UI local costmap or `/local_costmap/costmap`).
- The ChArUco board and a basket still get marked where taller than 10 cm; a board lying flat under 10 cm may stay unmarked.

## Results

Record the robot run here: one row per check, the measured number and pass or fail.

Date:

| Check | Number | Result |
|---|---|---|
| Deploy (failed=0, units active) | | |
| Pan direction (+0.3 rad turns left) | | |
| Head re-zero (pan and tilt within 0.005 rad) | | |
| Recalibration plane_bias_mm_median (+-10 mm) | | |
| Plane height (0 +-1 cm) | | |
| plane_vs_base (< 0.6 deg) | | |
| Drop and obstacle points over 60 s (0 and 0) | | |
| ground_temporal_cells (non-zero) | | |
| Costmap clear on empty floor for 30 s | | |
| Costmap marks board and basket above 10 cm | | |
