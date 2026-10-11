# map_guard

Protects the SLAM map (slam_toolbox, online async mapping, see `nodes/slam_toolbox/README.md`) from being corrupted
when the robot is moved in ways the odometry does not explain.

- **Trigger 1, map mismatch:** the current scan no longer agrees with the cached `/map`.
- **Trigger 2, disturbance:** the robot is LIFTED, CARRIED / PUSHED or ROTATED by hand. Bumps are not a trigger on
  their own: every detector needs its condition to hold for its own duration, so short spikes are ignored (a bump that
  really shifts the pose is caught by trigger 1).
- **Response:** pause slam_toolbox's new measurements while disturbed; once the robot has settled, resume if the scan
  agrees with the map again, otherwise back up (serialize) the current map, reset SLAM and tell web_ui to drop its GPS
  anchor.
- **Agent:** only while claude_agent is processing an instruction (`busy`), an event goes on `/robot_events`;
  otherwise only logs and metrics.
- **Starts in dry run:** everything is evaluated, logged and exported, but no slam_toolbox service is called.

Client (robot) only. Pure ROS-free logic in `agreement.py`, `detectors.py`, `guard.py`, `agent.py`, `backup.py`,
`timing.py`; `node.py` only wires ROS (subscriptions, TF, service clients, worker thread).

## Signals

| Direction | Name | Type | Use |
|---|---|---|---|
| in | `/scan_filtered` | `sensor_msgs/LaserScan` | scored against the map |
| in | `/map` | `nav_msgs/OccupancyGrid` (reliable, transient local) | cached map |
| in | TF `map` <- laser frame | | projects the scan (slam_toolbox `map->odom` + EKF `odom->base_link` + static) |
| in | `/imu/data` | `sensor_msgs/Imu` (BNO055 NDOF) | tilt, yaw rate, vertical acceleration |
| in | `/odom` | `nav_msgs/Odometry` | wheel odometry (swerve_controller) |
| in | `/odom_rf2o` | `nav_msgs/Odometry` | lidar odometry (rf2o) |
| in | `/cmd_vel` | `geometry_msgs/Twist` | drive command the swerve controller follows (end of the Nav2 chain and teleop) |
| in | `GET http://127.0.0.1:18300/api/state` | HTTP JSON `{"busy": bool}` | claude_agent busy flag |
| out | `/robot_events` | `std_msgs/String` JSON | agent events (only while busy) |
| out | `/map_guard/map_reset` | `std_msgs/Empty` | after a real reset (never in dry run); web_ui drops the GPS anchor and map |
| call | `/slam_toolbox/pause_new_measurements` | `slam_toolbox/srv/Pause` | toggle (request empty, response `status`) |
| call | `/slam_toolbox/get_parameters` | `rcl_interfaces/srv/GetParameters` | reads `paused_new_measurements` |
| call | `/slam_toolbox/serialize_map` | `slam_toolbox/srv/SerializePoseGraph` | backup (`filename`, `result`) |
| call | `/slam_toolbox/reset` | `slam_toolbox/srv/Reset` | `pause_new_measurements: false`, `result` |
| out | `127.0.0.1:19110/metrics` | Prometheus | see Metrics |

## How it works

### Agreement score (`agreement.py`)

At most `agreement_hz` times per second a scan is projected into the map: endpoints of the finite ranges within
`[range_min, min(range_max, max_range_m)]`, transformed through the latest `map` <- laser frame TF (full 3D rotation,
so a rotated or upside-down lidar mount is handled). An endpoint is **evaluated** if it falls inside the grid and a
known cell is within `hit_radius_cells`; it **agrees** if a cell within `hit_radius_cells` is occupied
(`>= occupied_threshold`). Points in unknown space are not evaluated: driving into an unexplored area is not a
mismatch. `ratio = agreeing / evaluated`; no sample when there is no map, the map has no known cell or fewer than
`min_points` points were evaluated. The dilated masks are computed once per `/map` message (numpy, vectorised).

**Mismatch:** the rolling median of the last `median_window` ratios stays below `mismatch_ratio` for `mismatch_s`,
while the map has at least `min_map_cells_known` known cells and the median had been at least `healthy_ratio` on
it before. A fresh or empty map therefore never triggers.

### Disturbance detectors (`detectors.py`)

- **LIFTED, tilt:** roll or pitch from the IMU orientation differs from the level baseline by more than
  `lift_tilt_deg` for `lift_s`. The baseline is the median attitude of the last level period (`level_window_s` of
  samples within `level_tolerance_deg` of each other). A steady attitude beyond `lift_tilt_deg` is adopted as the
  new baseline only after `level_rebaseline_s` (the robot was put down on a slope).
- **LIFTED, vertical acceleration:** `bno055_imu` publishes the fusion's gravity-free `linear_acceleration` (it
  falls back to the raw, gravity-included `acceleration` only when the linear one is unreadable). The acceleration is
  rotated into the world frame with the orientation; `|a_z| > lift_accel_z` for `lift_accel_s` is lift evidence,
  which triggers LIFTED only if, within `lift_confirm_s`, the tilt exceeds `lift_confirm_tilt_deg` or the lidar
  odometry moves faster than `carried_speed_mps`. Samples whose norm is within `gravity_band` of g are the
  gravity-included fallback and carry no evidence.
- **CARRIED / PUSHED:** lidar odometry speed above `carried_speed_mps` while wheel odometry speed is below
  `wheel_still_mps` and nothing is commanded (no `/cmd_vel` newer than `cmd_timeout_s` with a magnitude above
  `cmd_eps`), for `carried_s`. The lidar speed is the larger of the rf2o twist and the pose difference between two
  messages (rf2o's twist is laser-frame x only, so a sideways carry shows only in the pose).
- **ROTATED:** IMU yaw rate (world z) above `rotate_rate_rad_s` while the wheel odometry yaw rate is below
  `wheel_still_rad_s` and no angular command is active, for `rotate_s`.

Streams older than `input_timeout_s` are ignored (no data, no trigger).

### State machine (`guard.py`)

```
OK --lifted/carried/rotated--> DISTURBED   pause new measurements, event "disturbed"
OK --mismatch---------------> EVALUATING   pause, event "disturbed" (no settling: nobody handles the robot)
DISTURBED --no detector fires--> SETTLING --settle_s quiet--> EVALUATING
SETTLING / EVALUATING --any lifted/carried/rotated--> DISTURBED
EVALUATING --after evaluate_s, median >= healthy_ratio--> RESUMED (unpause, event "resumed") --> OK
EVALUATING --after evaluate_s, median <  healthy_ratio--> RESET --> OK (+ cooldown_s)
```

- A mismatch alone pauses too (so no bad scan is integrated while it is evaluated) and skips the settling step.
- No agreement sample at all during the evaluation (no map, too few points): the guard resumes; it never drops a
  map without evidence.
- **RESET:** `serialize_map` to `<backup_dir>/<backup_prefix>_<UTC YYYYMMDDTHHMMSSZ>` (slam_toolbox writes
  `.posegraph` and `.data`), delete all but the newest `keep_backups` backups (the main `slam_map.*` is never
  touched), call `/slam_toolbox/reset`, unpause (Reset with `pause_new_measurements: false` leaves the pause state
  alone, so the guard unpauses itself), publish `/map_guard/map_reset`, event "reset". A failed backup skips the
  reset (the map is never dropped without a copy) and unpauses. During `cooldown_s` after a reset a mismatch cannot
  trigger (the new map is also disarmed until it is healthy).
- **Pause toggle tracking:** `Pause` is a toggle. Before toggling, the guard reads slam_toolbox's
  `paused_new_measurements` parameter (falling back to its own tracked state when the parameter cannot be read) and
  toggles only when it differs from the wanted state; after the toggle it checks `status` and re-reads the
  parameter, logging a warning if the state did not change.
- Every transition is logged at warn with the reasons and the numbers (rolling median ratio, evaluation median).

### Agent events (`agent.py`)

Contract of `/robot_events` (`nodes/claude_agent/claude_agent/robot_events.py`):
`{seq, ts, type, severity, source: "map_guard", message, data: {reason, ratio, dry_run, ...}}`.

| type | severity |
|---|---|
| `map_guard_disturbed` | `critical` for lifted / carried (the agent stops driving), `warning` for rotated / mismatch |
| `map_guard_resumed` | `warning` |
| `map_guard_reset` | `critical` (`data.backup` is the backup path) |

Events are published **only while the agent is busy**: `GET agent_state_url` returns `busy: true`. The flag is
fetched at most every `agent_poll_s` with an `agent_timeout_s` timeout, from the guard worker thread (never the
executor). An unreachable agent, a timeout or a malformed answer counts as not busy. In dry run the events are still
sent while busy, with the message prefixed `[dry run] `. Dry run only suppresses the slam_toolbox calls and the reset topic: lifted / carried
`map_guard_disturbed` events stay `critical` so a lift interrupts the agent; every other event is `warning` in dry run
(including the would-have-reset event).

## Metrics

Port `metrics_port` (19110) on 127.0.0.1, through `shared/ros2_metrics`.

| Metric | Type | Meaning |
|---|---|---|
| `map_guard_agreement_ratio` | gauge | latest agreement sample |
| `map_guard_state{state}` | gauge | 1 for the current state (ok, disturbed, settling, evaluating, resumed, reset), 0 for the others |
| `map_guard_triggers_total{reason}` | counter | triggers acted on (mismatch, lifted, carried, rotated) |
| `map_guard_pauses_total` | counter | pauses requested (dry run included) |
| `map_guard_resets_total{dry_run}` | counter | resets (`dry_run="true"`: would have reset) |
| `map_guard_resumes_total` | counter | resumes |
| `map_guard_tilt_deg` | gauge | roll/pitch difference from the level baseline |
| `map_guard_unexplained_speed_mps` | gauge | lidar minus wheel odometry speed while nothing is commanded (0 while commanded) |
| `map_guard_dry_run` | gauge | 1 in dry run |

## Configuration

`/etc/ros2/map_guard/config.yaml` (env `MAP_GUARD_CONFIG`), written from the `config: |` block of the `map_guard`
entry in `ansible/group_vars/client.yml`. Validated by pydantic (`config.py`): unknown keys are rejected, durations
and rates must be positive, ratios in 0..1, `healthy_ratio > mismatch_ratio`, `reasons` only from
`mismatch, lifted, carried, rotated`.

| Key | Default | Meaning |
|---|---|---|
| `dry_run` | `true` | log + metrics only; no slam_toolbox call, no `/map_guard/map_reset` |
| `reasons` | `[mismatch, lifted, carried, rotated]` | triggers acted on |
| `scan_topic` | `/scan_filtered` | scan scored against the map |
| `map_topic` | `/map` | SLAM map |
| `imu_topic` | `/imu/data` | IMU |
| `wheel_odom_topic` | `/odom` | wheel odometry |
| `lidar_odom_topic` | `/odom_rf2o` | lidar odometry |
| `cmd_vel_topic` | `/cmd_vel` | drive command |
| `map_frame` | `map` | map frame |
| `reset_topic` | `/map_guard/map_reset` | reset notification |
| `events_topic` | `/robot_events` | agent events |
| `pause_service` | `/slam_toolbox/pause_new_measurements` | Pause toggle |
| `reset_service` | `/slam_toolbox/reset` | Reset |
| `serialize_service` | `/slam_toolbox/serialize_map` | SerializePoseGraph |
| `parameters_service` | `/slam_toolbox/get_parameters` | reads `paused_new_measurements` |
| `service_timeout_s` | `5.0` | service response timeout, s |
| `agent_state_url` | `http://127.0.0.1:18300/api/state` | claude_agent state |
| `agent_poll_s` | `2.0` | minimum time between agent requests, s |
| `agent_timeout_s` | `0.5` | agent HTTP timeout, s |
| `metrics_port` | `19110` | Prometheus port (null: env `METRICS_PORT`, else off) |
| `tick_hz` | `10.0` | state machine rate |
| `max_range_m` | `8.0` | scan range limit for scoring, m |
| `hit_radius_cells` | `1` | hit / known radius, cells |
| `occupied_threshold` | `65` | occupied from this occupancy |
| `min_points` | `40` | minimum evaluated points per sample |
| `agreement_hz` | `2.0` | maximum agreement rate |
| `median_window` | `5` | samples in the rolling median |
| `mismatch_ratio` | `0.3` | mismatch below this median... |
| `mismatch_s` | `5.0` | ...held this long, s |
| `healthy_ratio` | `0.6` | arms the mismatch; resume bar |
| `min_map_cells_known` | `2000` | known cells before a mismatch can trigger (5 m^2 at 5 cm) |
| `lift_tilt_deg` | `12.0` | tilt from baseline that means lifted, deg... |
| `lift_s` | `1.0` | ...held this long, s |
| `level_window_s` | `3.0` | level period length, s |
| `level_tolerance_deg` | `2.0` | attitude spread in a level period, deg |
| `level_rebaseline_s` | `30.0` | steady tilted attitude adopted as the baseline after this, s |
| `lift_accel_z` | `1.5` | vertical gravity-free acceleration, m/s^2... |
| `lift_accel_s` | `0.4` | ...held this long, s... |
| `lift_confirm_s` | `2.0` | ...then confirmed within this, s... |
| `lift_confirm_tilt_deg` | `4.0` | ...by this tilt (or lidar motion), deg |
| `gravity_band` | `2.0` | norm within this of g = gravity-included sample, m/s^2 |
| `carried_speed_mps` | `0.08` | lidar odometry speed, m/s... |
| `wheel_still_mps` | `0.02` | ...while wheels slower than this, m/s... |
| `cmd_timeout_s` | `0.5` | ...and no command newer than this, s... |
| `cmd_eps` | `0.01` | ...above this magnitude... |
| `carried_s` | `1.0` | ...for this long, s |
| `input_timeout_s` | `0.5` | older stream samples are ignored, s |
| `rotate_rate_rad_s` | `0.3` | IMU yaw rate, rad/s... |
| `wheel_still_rad_s` | `0.05` | ...while wheel yaw rate below this... |
| `rotate_s` | `0.8` | ...for this long, s |
| `settle_s` | `3.0` | quiet time before the evaluation, s |
| `evaluate_s` | `4.0` | evaluation window, s |
| `cooldown_s` | `30.0` | no mismatch trigger after a reset, s |
| `backup_dir` | `/var/lib/ros2/maps` | backup directory (created by the slam_toolbox deploy, owned by the node user) |
| `backup_prefix` | `map_guard_backup` | backup stem prefix |
| `keep_backups` | `5` | newest backups kept |

## Dry run and enabling

Deployed with `dry_run: true`. Watch the warn log lines (`journalctl -u ros2-map_guard`, lines starting with
`[dry run] map_guard`) and the metrics while using the robot normally and while lifting / carrying / turning it by
hand. When the triggers match reality, set `dry_run: false` in the `map_guard` config block and redeploy:

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client map_guard 2>&1 | tee .logs/deploy-client-map_guard.log
```

To act on only some triggers, narrow `reasons` (e.g. `[lifted, carried]`).

## Tuning from Grafana

- `map_guard_agreement_ratio` during normal driving gives the healthy level: keep `healthy_ratio` clearly below its
  usual value and `mismatch_ratio` clearly below the dips seen in cluttered or glassy areas.
- `map_guard_tilt_deg` while driving over thresholds and ramps sets `lift_tilt_deg` (keep it above the largest
  driving tilt); lift the robot to see the lifted value.
- `map_guard_unexplained_speed_mps` while driving (should stay near 0, it is 0 while commanded) and while the robot
  coasts after a command sets `carried_speed_mps` and `cmd_timeout_s`.
- `map_guard_triggers_total{reason}` and `map_guard_resets_total{dry_run="true"}` rates show false positives in dry
  run; `map_guard_state` shows how long episodes last (`settle_s`, `evaluate_s`).

## Failure modes

- **No TF `map` <- laser** (SLAM not running, EKF down): no agreement sample, so no mismatch and evaluations resume.
- **No `/map` yet / fresh map:** no sample or disarmed monitor; no mismatch.
- **IMU stale or without orientation** (`orientation_covariance[0] == -1` or a zero quaternion): lifted/rotated
  cannot fire; carried still works.
- **rf2o or wheel odometry stale:** carried/rotated cannot fire.
- **slam_toolbox service unavailable or timing out:** logged at warn; pause/unpause keep the last known state, a
  failed backup skips the reset, a failed reset publishes nothing.
- **claude_agent unreachable:** treated as idle, no events.
- **Steep ramps (> lift_tilt_deg):** read as lifted until `level_rebaseline_s` of steady attitude; raise
  `lift_tilt_deg` or drop `lifted` from `reasons` if the robot drives on such slopes.
- **Teleop through a topic other than `/cmd_vel`:** the robot would look carried; `cmd_vel_topic` must be the topic
  the swerve controller follows.

## Development

```bash
cd nodes/map_guard
uv sync
uv run pytest tests/ -v
uv run poe lint
```

Tests: `test_agreement.py`, `test_detectors.py`, `test_guard.py`, `test_agent.py`, `test_backup.py`,
`test_config.py`, `test_timing.py`, `test_node.py` (see `tests/README.md`).
