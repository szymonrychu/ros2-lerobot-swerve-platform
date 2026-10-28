# Runbook: ground estimation on the robot

Executed by the orchestrator on the robot after the merge, not by the implementer. Run from the dev machine against the
client (Pi 5, `client.ros2.lan`). Deploys only through `scripts/deploy-nodes.sh` (ansible-deploy skill), logs in `.logs/`.
ROS commands on the client need `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. CPU and memory
are read in Grafana, never with `top` over SSH. Design: `docs/plans/in-progress/rgb-d-ground-estimation-and/design.md`
(D11, D12); node: `nodes/ground_estimator_cpp/README.md`; Nav2 side: `nodes/nav2_bringup/README.md`.

Obstacle rule under test: a step, threshold, drop or ramp is an obstacle (lethal) when its step exceeds one third of the
wheel diameter (0.04 m) or its slope exceeds 15 deg; otherwise it is a warning (slow-down, passable). Exactly 0.04 m and
15 deg are warnings. Servo safety: scenes drive at the lowest speed first; nothing here moves an arm joint.

After a head remount run `docs/runbooks/head-remount-floor-check.md` first (z-depth, head pan sign, re-zero,
recalibration, floor numbers).

## Preconditions

- head_rgbd_cpp and laser_filter are running (`systemctl is-active ros2-head_rgbd_cpp ros2-laser_filter` on the client);
  `/tof/points_filtered` and `/head_rgbd/points` publish.
- The head is at the home tilt, 30 deg down (check the head joint_states).
- No agent session is running before mcp_server is deployed (MEMORY "deploy during agent session": deploying mcp_server
  kills a running agent session, exit 143). Check the claude_agent session state first; ask when one is running.
- `topic_scraper_api` is enabled for the client in `ansible/group_vars/client.yml` (`enabled: true`) and deployed, else port
  18100 refuses connections (CLAUDE.md).
- A level, empty floor patch of at least 3 m x 2 m, the sill and drop props (3 cm and 5 cm), a box, a 13 deg and a steeper
  ramp, a red sheet, a tape measure.

## Deploy

1. Invoke the `ansible-deploy` skill for the command, then:
   `./scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones nav2_bringup web_ui 2>&1 | tee .logs/deploy-client-ground.log`
   Expect `failed=0` and the PCL build of ground_estimator_cpp done with `-j2` (recorded under Build and CI validation).
2. After the session check from Preconditions: `./scripts/deploy-nodes.sh client mcp_server 2>&1 | tee .logs/deploy-client-mcp.log`.
3. `systemctl is-active ros2-ground_estimator_cpp ros2-costmap_zones ros2-nav2_bringup ros2-web_ui ros2-mcp_server` all
   `active`; the metrics endpoint answers on port 19119.

## Topic checks

Collect with `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /ground/state:.plane --interval 0.1`
(one run per topic) over 60 s with the robot stationary on the level floor. Not raw `ros2 topic echo`.

| Topic | Expected rate |
|---|---|
| `/ground/obstacle_points` | ~10 Hz |
| `/ground/drop_points` | ~10 Hz |
| `/ground/labelled_points` | ~1 Hz |
| `/ground/state` | 2 Hz |

Also record: the TF drop counter (`ground_clouds_dropped_total` on port 19119, by reason; it must stay flat while the head is still and
rise only while the head moves) and the head joint_states rate (the head TF depends on it).

## Level floor

Robot still on the level floor, 60 s of `/ground/state`:

- plane height within 1 cm of the tape-measured sensor height above the floor;
- slope below 1 deg (roll and pitch) and plane_vs_base below 1 deg;
- `prior` `imu` (with `imu_ok` true), `slope_level` `ok`.

## Scenes

For each scene place the prop 1 m ahead, read `/ground/state` regions, `/ground/warning_areas`, the web UI Ground layer and
the local costmap, then drive at the prop with a nav goal. Obstacle rule: max step = one third of the wheel diameter =
0.04 m, max slope 15 deg (bounded by the tilt_guard error limits).

- 3 cm sill: a warning area, `/speed_limit` at the warning percent while crossing, non-lethal ground cost cells (about 101,
  below INSCRIBED 253), the robot drives over it.
- 5 cm sill: a lethal ground mark (`ground_obstacle_mark` cells), the planner routes around.
- 3 cm drop: a warning area, slow-down while crossing, non-lethal cost cells, the robot drives over it.
- 5 cm drop: lethal `ground_drop_mark` cells, the planner routes around.
- 10 cm drop at 1 m: a drop region with height within 2 cm and edge distance within 5 cm of the tape measure,
  `ground_drop_mark` cells, CliffStop behaves unchanged (still driven by `depth_cliff`).
- box: obstacle points and `ground_obstacle_mark` cells.
- 13 deg ramp: a warning with slope within 1 deg of the measured angle, driven over slowly.
- ramp steeper than 15 deg: an obstacle, lethal mark, planner routes around.
- red sheet flat on the floor: `floor_colour_mismatch` labels in the web UI Ground layer and a colour warning area
  (slow-down, passable). Compare the fixed and the ground marks in the web UI local costmap with the layers toggled at
  runtime: `ros2 param set /local_costmap/local_costmap ground_voxel_layer.enabled false|true` and
  `ros2 param set /local_costmap/local_costmap depth_voxel_layer.enabled false|true`; restore both to `true` afterwards.

## CPU and memory

Pi 5, 10 min of driving in the scenes' area:

- ground_estimator_cpp: `rate(container_cpu_usage_seconds_total{host="client",id="/system.slice/ros2-ground_estimator_cpp.service"}[1m])`
  p95 <= 0.10 core.
- nav2_bringup before/after: `uv run python scripts/node_cpu_compare.py` with the `nav2_bringup_ground_layer` entry of
  `scripts/node_cpu_compare.yaml`; delta <= +0.05 core.
- Memory p50 and peak of the unit. Write `memory_low` = p50 and `memory_max` >= 2x peak into the `ground_estimator_cpp` type
  in `ansible/group_vars/client.yml`, commit `chore(ansible): ground_estimator_cpp measured memory`, then redeploy
  ground_estimator_cpp with the command from Deploy.

## Build and CI validation

CI and the robot run validate these checks. Record each result line in Results.

- The cpp-node-test container suite of ground_estimator_cpp in the CI matrix (`.github/workflows/ci.yml`).
- The frontend suite: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`.
- The PCL build of the deploy on the Pi 5 with `-j2`, within memory (no OOM kill in the deploy log).

## Threshold validation

Each default is confirmed by the scenes above or corrected in `ansible/group_vars/client.yml` with the measured reason.
The values equal `nodes/ground_estimator_cpp/config/ground_estimator.yaml` and the costmap_zones block of client.yml.

| Key | Default | Confirmed by |
|---|---|---|
| `fit.distance_m` | 0.02 | level floor plane height and slope |
| `fit.max_normal_angle_deg` | 10 | level floor, fit never rejected on the floor |
| `fit.max_offset_m` | 0.08 | level floor, fit never rejected on the floor |
| `traversability.wheel_radius_m` | 0.06 | equals swerve_drive_controller; max step 1/3 of the diameter (`max_step_fraction`) = 0.04 m |
| `traversability.max_slope_deg` | 15 | 13 deg ramp warning, ramp steeper than 15 deg obstacle |
| `edges.step_max_height_m` | 0.2 | 3 cm and 5 cm sills and drops, box |
| `colour.max_mahalanobis` | 3.0 | red sheet flagged, floor not flagged |
| `ground_warning_speed_pct` | 40 | `/speed_limit` while crossing the 3 cm sill |
| `ground_warning_mask_value` | 40 | ground cost cells about 101, passable |

## Switch-over checklist

The criteria of the nav2_bringup README ("Switch-over criteria"), each with its measured result in Results. The switch-over
itself (disabling `depth_mark` and `depth_cliff_mark` in `depth_voxel_layer`) is its own change after every result passes.

1. Level-floor check passes: plane height within 1 cm, slope and plane_vs_base below 1 deg.
2. The 3 cm sill, 5 cm sill, 10 cm drop at 1 m, box and ramp scenes mark as expected with `ground_voxel_layer` enabled and
   `depth_voxel_layer` disabled at runtime.
3. No false marks on the floor over a 5 min drive.
4. CPU within the budget of the ground estimator (0.10 core) and Nav2 (+0.05 core) on the Pi 5.

## Results

The robot run fills this table. Status values: PASS, FAIL, NOT RUN.

| Check | Expected | Measured | Pass |
|---|---|---|---|
| Deploy ground_estimator_cpp costmap_zones nav2_bringup web_ui | failed=0 | | |
| Deploy mcp_server after the session check | failed=0 | | |
| `/ground/obstacle_points` rate | ~10 Hz | | |
| `/ground/drop_points` rate | ~10 Hz | | |
| `/ground/labelled_points` rate | ~1 Hz | | |
| `/ground/state` rate | 2 Hz | | |
| TF drop counter, head joint_states rate | recorded | | |
| Level floor plane height | within 1 cm | | |
| Level floor slope and plane_vs_base | below 1 deg | | |
| Level floor prior, slope_level | imu, ok | | |
| 3 cm sill | warning, slow-down, drives over | | |
| 5 cm sill | lethal mark, routed around | | |
| 3 cm drop | warning, slow-down, drives over | | |
| 5 cm drop | lethal mark, routed around | | |
| 10 cm drop at 1 m | height within 2 cm, edge within 5 cm, CliffStop unchanged | | |
| Box | obstacle | | |
| 13 deg ramp | warning, slope within 1 deg, driven slowly | | |
| Ramp steeper than 15 deg | obstacle | | |
| Red sheet | colour mismatch labels, warning area, passable | | |
| ground_estimator_cpp CPU p95 | <= 0.10 core | | |
| nav2_bringup_ground_layer delta | <= +0.05 core | | |
| Memory p50 / peak, client.yml committed | memory_low = p50, memory_max >= 2x peak | | |
| cpp-node-test suite in CI | green | | |
| Frontend suite | green | | |
| PCL build on the Pi 5 (-j2) | within memory | | |
| Threshold validation | defaults confirmed or corrected | | |
| Switch-over criteria 1-4 | all hold | | |
