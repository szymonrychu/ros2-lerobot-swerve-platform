# BNO055 IMU Node

ROS2 node that reads a BNO055 IMU over I2C and publishes `sensor_msgs/Imu` on `/imu/data` (configurable), with full covariance matrices and correct units for the Navigation stack (Nav2).

## Features

- **Topic**: `sensor_msgs/Imu` on `/imu/data` by default.
- **Frame**: `header.frame_id` default `imu_link` (configurable).
- **Data**: Orientation (quaternion), angular velocity (rad/s), linear acceleration (m/s²).
- **Covariances**: Configurable orientation, angular velocity, and linear acceleration covariance matrices (row-major 9 values; use `-1` in first element for "unknown").
- **Rate**: Configurable publish rate (default 100 Hz).

## Configuration

YAML config path: `BNO055_IMU_CONFIG` or `/etc/ros2/bno055_imu/config.yaml`.

| Key | Default | Description |
|-----|---------|-------------|
| `topic` | `/imu/data` | ROS2 topic for Imu messages |
| `frame_id` | `imu_link` | Header frame_id |
| `publish_hz` | `100` | Publish rate (1–1000) |
| `i2c_bus` | `1` | I2C bus number (e.g. 1 → /dev/i2c-1) |
| `i2c_address` | `0x28` | BNO055 I2C address (`0x28` or `0x29` typical) |
| `orientation_covariance` | `0.01` or list of 9 | Diagonal variance or full 9-element row-major |
| `angular_velocity_covariance` | `0.01` or list of 9 | Same format |
| `linear_acceleration_covariance` | `0.04` or list of 9 | Same format |
| `operation_mode` | `IMUPLUS` | `IMUPLUS` (gyro+accel, relative heading), `NDOF` or `NDOF_FMC_OFF` (adds the magnetometer: absolute heading) |
| `calibration_topic` | `/imu/calibration` | `std_msgs/String` JSON `{sys, gyro, accel, mag}` (0-3 each) at 1 Hz; empty disables |
| `calibration_file` | `/var/lib/ros2/bno055_imu/calibration.json` | Persisted sensor offsets, restored at init; empty disables persistence |
| `calibration_save_interval_s` | `60` | Minimum seconds between calibration saves (min 1) |
| `max_soft_restores` | `3` | Consecutive soft mode restores without a published sample before a full re-init (min 0) |
| `reinit_after_s` | `10.0` | Seconds without a published sample before a full re-init, whatever the failure path (min 1) |
| `odom_topic` | `/odom` | `nav_msgs/Odometry` (wheel odometry) that says when the robot is stationary; empty disables gyro bias learning |
| `odom_timeout_s` | `1.0` | Odometry older than this counts as not stationary (min 0.1) |
| `stationary_linear_mps` | `0.01` | Odometry `abs(vx)`, `abs(vy)` below this counts as still |
| `stationary_angular_rad_s` | `0.01` | Odometry `abs(wz)` below this counts as still |
| `gyro_bias_stillness_s` | `2.0` | Seconds of continuous stillness before the bias is learned |
| `gyro_bias_tau_s` | `60` | Time constant of the bias moving average (min 1) |
| `gyro_bias_max_rad_s` | `0.02` | Clamp of the bias magnitude per axis |
| `gyro_bias_sanity_rad_s` | `0.05` | Samples whose gyro norm minus the current bias exceeds this are not learned |
| `gyro_bias_file` | `/var/lib/ros2/bno055_imu/gyro_bias.json` | Persisted learned bias, loaded at start; empty disables persistence |
| `gyro_bias_save_interval_s` | `60` | Minimum seconds between bias saves (min 1); only a changed, settled bias is written |
| `gyro_bias_mean_s` | `30` | Seconds of learning that use a running mean instead of the EMA (min 0) |
| `gyro_bias_topic` | `/imu/gyro_bias` | `std_msgs/String` JSON bias diagnostic at 1 Hz; empty disables it |
| `save_calibration_service` | `/imu/calibration/save` | `std_srvs/Trigger`: save the chip calibration profile now (see Calibration); empty disables it |
| `reset_calibration_service` | `/imu/calibration/reset` | `std_srvs/Trigger`: delete the saved profile, reset the chip, start uncalibrated; empty disables it |
| `metrics_port` | unset | Prometheus `/metrics` port on 127.0.0.1 (robot: `19103`); unset falls back to env `METRICS_PORT`, then disabled |

Example:

```yaml
topic: /imu/data
frame_id: imu_link
publish_hz: 100
i2c_bus: 1
i2c_address: 0x28
orientation_covariance: 0.01
angular_velocity_covariance: 0.01
linear_acceleration_covariance: 0.04
```

## Metrics

Served on `127.0.0.1:<metrics_port>` (`19103` on the robot) via `ros2-metrics`, node name `bno055_imu`.

| Metric | Type | Meaning |
|--------|------|---------|
| `imu_read_errors_total` | counter | I2C read errors in the publish loop |
| `imu_soft_restores_total` | counter | Soft mode restores issued by the recovery policy |
| `imu_reinits_total{reason}` | counter | Full re-inits; reason `watchdog`, `i2c_hard_errors` or `soft_exhausted` |
| `imu_init_attempts_total` | counter | Driver creation attempts (start-up and re-init) |
| `imu_calibration_level{subsystem}` | gauge | 0-3 for `sys`, `gyro`, `accel`, `mag` |
| `imu_published_total` | counter | Imu samples published |
| `imu_mode` | gauge | OPR_MODE register value (0x08 IMUPLUS, 0x0b NDOF_FMC_OFF, 0x0c NDOF) |
| `imu_seconds_since_publish` | gauge | Seconds since the last published sample |
| `imu_gyro_bias_rad_s{axis}` | gauge | Learned gyro bias subtracted from `angular_velocity` (`x`, `y`, `z`); 0 until learned |
| `imu_gyro_bias_rejected_samples_total` | counter | Stationary gyro samples rejected by the sanity gate (`gyro_bias_sanity_rad_s`); a bias this large is never learned |
| `imu_gyro_bias_learning` | gauge | 1 while the last sample updated the bias (robot stationary), else 0 |
| `imu_cycle_seconds` | histogram | Work time of one cycle (burst read, decode, publish, periodic calibration); buckets 0.002 to 0.1 s |
| `imu_cycle_overruns_total` | counter | Cycles that finished after their next slot; the missed slots were skipped |

## Read and pacing

- **Burst read** (`bno055_imu/burst.py`): the data registers gyro (0x14), euler (0x1A), quaternion (0x20) and linear
  acceleration (0x28) are contiguous, so each cycle does ONE I2C transaction (write register 0x14, read 26 bytes) instead
  of the library's separate property reads (each also reads the mode register first). `decode_burst` applies the same
  scale factors as `adafruit_bno055.BNO055_I2C` (gyro 0.001090830782496456 rad/s per LSB, euler 1/16 deg, quaternion
  1/2^14, linear acceleration 1/100 m/s^2) so published values are identical to the old ones. An all-zero block (what the
  chip returns in CONFIG mode) is an invalid sample: it is retried, then the recovery policy takes over, and nothing is
  published.
- **Fixed-rate pacing** (`bno055_imu/pacing.py`): cycles start on a fixed grid (`next_tick`, period `1 / publish_hz`), and
  only the time left until the next slot is spent in the ROS executor (`spin_until`, so callbacks still run). A cycle that
  overruns its slot skips the missed slots without bursting to catch up and increments `imu_cycle_overruns_total`.
- **Calibration status** is read every 1 s inside the same cycle as a publish, it never costs an extra cycle.
- **CPU note**: the BNO055 sits on the kernel bit-banged `i2c-gpio` bus, so the time an I2C transaction takes is system
  CPU of this process. The unit has no CPU cap and runs in the critical `CPUWeight=1000` tier, so under contention
  other work yields first; fewer transactions per cycle is still what lets the node reach `publish_hz`. Watch
  `imu_cycle_seconds` and `imu_cycle_overruns_total` when tuning `publish_hz`.

## Publish rate and consumers

The client config (`ansible/group_vars/client.yml`) publishes `/imu/data` at `publish_hz: 25` (was 50): the software
`i2c-gpio` bus busy-waits, so the 50 Hz loop cost 0.37-0.40 core (two thirds kernel time, 18k of 120k cycles overran);
25 Hz frees about 0.18 core. The period is 40 ms, so a staleness limit must be at least 3 periods (0.12 s). The
`covariance_window` is 50 samples (still ~2 s; `compute_covariance` is off anyway). `tests/test_client_cpu_load.py`
pins the rate, the window span, every IMU staleness limit below and the rescaled `imu_baseline_alpha`.

| Consumer | Uses | At 25 Hz |
|----------|------|----------|
| robot_localization_ekf (`imu0`) | gyro yaw rate only (`imu0_config` index 11), EKF at 30 Hz, `imu0_queue_size` 10 | Fine: the EKF already interpolates between inputs slower than its 30 Hz tick; no staleness limit. |
| swerve_controller tilt guard | roll/pitch, `persistence_s` 0.5, `imu_timeout_s` 0.5 | Fine: time-based debounce (12 samples at 25 Hz), timeout is 12 periods. |
| ground_estimator_cpp IMU prior | latest orientation, `imu.timeout_s` 0.5 | Fine: 12 periods, and it uses the newest sample, not a window. |
| map_guard and map_guard_cpp | tilt, yaw rate, `input_timeout_s` 0.5, `level_window_s` 3, `lift_accel_s` 0.4 | Fine: all windows are in seconds (10 samples in the shortest, `lift_accel_s`); timeout is 12 periods. |
| mcp_server monitor | bump: `bump_min_samples` 2 (= 80 ms at 25 Hz, a glitch filter), `imu_baseline_alpha` EMA per sample | `imu_baseline_alpha` is per sample, so halving the rate would double the baseline time constant (1 s to 2 s); rescaled to 0.04 in client.yml. `bump_min_samples` stays 2 (80 ms still rejects a single glitchy sample, a real bump lasts longer). |
| mcp_server floor guard | tilt for the slow zone, `imu_max_age_s` 1.0 | Fine: 25 periods. |
| web_ui | GPS compass anchor (`COMPASS_IMU_MAX_AGE_S` 1.0 between IMU stamp and fix), IMU calibration status (`imu_stale_s` 3.0, status is 1 Hz) | Fine: both far above 40 ms. |
| head_rgbd_cpp | Does not subscribe to `/imu/data` (no IMU code or config) | Not affected. |
| master2master | Relays `/imu/data` to `/controller/imu/data` | Disabled on the client; not affected. |

## Recovery

`bno055_imu/recovery.py` (`RecoveryPolicy`, pure Python, injectable clock) decides between continue, soft mode restore
and full re-initialisation:

- **Soft restore**: re-writes the operation mode (no crystal switch). It only counts as successful once a valid sample is
  published, so restores do not clear the escalation state. After `max_soft_restores` in a row without a publish the node
  escalates. Log: `BNO055 soft mode restore N/M`.
- **Watchdog**: no published sample for `reinit_after_s` (`time.monotonic`) forces a full re-init regardless of which
  failure path the node is in.
- **Full re-init**: waits the backoff `min(30, 2**n)` s (n = re-inits since the last publish, capped at 5; a publish
  resets it), issues a guarded chip reset (`SYS_TRIGGER` 0x3F = 0x20 `RST_SYS`, ~650 ms; a failure there is ignored),
  recreates the I2C bus object and driver (crystal, saved calibration profile, operation mode set and verified), resets
  the covariance estimators and warms up. Logged at warn: `BNO055 full re-init (reinit #N): <reason>` with reason
  `soft restores exhausted (...)`, `watchdog: N s without data`, `I2C hard errors, soft restore not possible` or
  `calibration reset requested` (the reset service; no backoff, no second chip reset, no profile restored; metric code
  `calibration_reset`).
- Nothing is published while reads are invalid (no placeholder data).

## Gyro bias estimation

The gyro reads a small constant offset at rest: measured -5.83e-4 rad/s on z (-2 deg/min). The EKF fuses the gyro yaw
rate, so while parked the heading drifted 26 deg/h. The node therefore learns the bias and publishes
`angular_velocity = raw - bias`. The orientation from the BNO055 fusion is not altered.

- **Estimate**: per axis (x, y, z) exponential moving average of the raw gyro reading with time constant
  `gyro_bias_tau_s`, clamped to `gyro_bias_max_rad_s`.
- **Learn only when stationary**: the wheel odometry on `odom_topic` (best-effort QoS) must report `abs(vx)`, `abs(vy)` below
  `stationary_linear_mps` and `abs(wz)` below `stationary_angular_rad_s` continuously for `gyro_bias_stillness_s`, the
  odometry must be fresher than `odom_timeout_s` (no data means no learning), and the gyro norm minus the current bias must
  stay below `gyro_bias_sanity_rad_s` (rejects bumps and being carried and rotated).
- **Before anything is learned** the raw reading is published (bias 0), never placeholder data. An empty `odom_topic`
  disables the whole feature.
- **Running mean first**: during the first `gyro_bias_mean_s` (30 s) of learning in a run the estimate is the running mean of the accepted stationary samples (a stored bias counts as one sample), so it converges in about half a minute of stillness instead of several time constants; after that the EMA tracks slow drift.
- **Persisted**: once the mean window completed the bias is written to `gyro_bias_file` (atomic JSON `{"bias": [x, y, z]}`, next to `calibration.json`, same Ansible state dir) at most every `gyro_bias_save_interval_s` and only when it changed. It is loaded at start and applied from the first sample, so a restart no longer publishes raw gyro for minutes. A bias saved from a window shorter than `gyro_bias_mean_s` is never written. A missing file means zero bias; a corrupt or non-finite file is ignored with a warning. Delete the file to reset it.
- **Rejected samples**: while the robot is stationary but the gyro is more than `gyro_bias_sanity_rad_s` (0.05) from the current bias, the sample is counted in `imu_gyro_bias_rejected_samples_total` and the node warns at most every 30 s. With the clamp at 0.02 a true bias above about 0.07 rad/s is never learned, and this is how you see it.
- **Audit topic**: `gyro_bias_topic` (`/imu/gyro_bias`, 1 Hz, std_msgs/String JSON) carries the real last raw gyro sample (`raw`, rad/s, before correction), the `bias`, `learning`, `settled` (mean window done), `rejected_samples` and `learned_s`. Published only from real samples, never a placeholder.
- **Watch it**: `imu_gyro_bias_rad_s{axis="z"}` should settle near -5.8e-4 and `imu_gyro_bias_learning` is 1 while parked.

## Calibration persistence

The BNO055 forgets its calibration on every reset, so each node restart used to drop `/imu/calibration` to 0,0,0,0.
The node now keeps the sensor offsets in `calibration_file` (JSON: `accel_offset`, `gyro_offset`, `mag_offset`,
`accel_radius`, `mag_radius`).

- **Restore**: on every chip init (start and I2C reconnect) the offsets are written in CONFIG mode before the chip is
  switched to `operation_mode`; the log shows `restored calibration profile from <file>`. A missing file means an
  uncalibrated start. A corrupt or implausible file (missing keys, values outside int16, radius outside 1-2000) logs a
  warning and the node starts uncalibrated.
- **Save**: once per calibration episode (after `mag` reaches 3, again only after it dropped below 3 and returned),
  checked every `calibration_save_interval_s` and only while the robot is stationary per wheel odometry (needs the gyro
  bias estimator; without it the stillness check is skipped). Only the magnetometer is gated: a wheeled robot cannot
  do the accelerometer 6-orientation motion, and the compass heading needs only the mag offsets. The node briefly switches to CONFIG mode, reads the offsets, switches back and writes the file atomically (temp file
  plus `os.replace`) only if it differs from what is on disk; the log shows `saved calibration profile`. That cycle
  publishes nothing while fusion restarts (about 1.5 s), which the tilt guard reports as a stale IMU; reading the chip
  every interval used to blank `/imu/data` once a minute for as long as `mag` stayed 3. The adafruit_bno055 offset properties do not switch modes
  themselves (library 5.4.22), so the node does it explicitly.
- **Restored flag**: `/imu/calibration` JSON carries `restored: true` once a saved profile is on the chip (restored at
  init, or just saved). The chip itself reports 0 after a restart until it re-checks, so consumers (web_ui compass
  anchor) trust `restored` instead of waiting for `mag` to climb again.
- **Reset**: call the reset service (below); deleting the file by hand and restarting the node still works. Ansible
  creates `/var/lib/ros2/bno055_imu` owned by the node user.

### Save and reset services

Two `std_srvs/Trigger` services (logic in `bno055_imu/calibration_service.py`, ROS-free and unit-tested on a fake
driver). The usual way to call them is the web UI **Calibrate** page (IMU section), which shows `/imu/calibration` live
and has Save and Reset buttons; from a shell: `ros2 service call /imu/calibration/save std_srvs/srv/Trigger`.

- **Save** (`save_calibration_service`, `/imu/calibration/save`): reads the current status and, when `mag` is 3 and the
  robot is stationary, reads the chip profile (CONFIG-mode round trip, about 1.5 s without `/imu/data`), validates it
  and writes `calibration_file` atomically even if unchanged. Response `success: true`, message
  `saved calibration (sys S, gyro G, accel A, mag M) to <file>`. It refuses with `success: false` and writes nothing
  when: `calibration_file disabled`; `calibration status unavailable`; `mag is N, needs 3`; `robot is moving` (wheel
  odometry says moving or is stale; with `odom_topic` empty stillness is unknown and the save is allowed); `calibration
  reset in progress`; or a read/validate/write error (the message carries it). Once the chip was read, the calibration
  episode counts as read, so the automatic save does not blank `/imu/data` again for the same episode.
- **Reset** (`reset_calibration_service`, `/imu/calibration/reset`): refused with `robot is moving` while moving.
  Otherwise it deletes `calibration_file` (a missing file is fine), resets the chip (`SYS_TRIGGER` register 0x3F,
  `RST_SYS` bit 0x20, datasheet section 4.3.63) and the read loop runs the full re-init path without restoring a
  profile, before any further read. The status restarts from 0 and `/imu/calibration` reports `restored: false`. A
  failed delete or chip write answers `success: false` with the error. During the re-init nothing is published; the
  usual warm-up rules apply (no placeholder data).
- Both callbacks run in the node's single-threaded executor, spun only from the read loop (`spin_once` between cycles),
  so no service I2C access ever overlaps a read.

What to do while the status climbs (the Calibrate page shows the same guidance):

- **gyro**: keep the robot still for a few seconds.
- **accel**: hold several different static poses (tilted on each side, nose up and down); a wheeled robot only
  reaches them by lifting it. Only `mag` gates the save, so accel may stay below 3.
- **mag**: drive or rotate the robot through slow figure-eights, away from motors, steel and power cables.

## Hardware

- **Sensor**: BNO055 (Bosch 9-DOF) over I2C.
- **Default address**: `0x28`; alternate `0x29` depending on ADR pin.
- **Fallback behavior**: Node tries configured `i2c_address` first, then falls back to alternate BNO055 addresses.
- **Host**: On Raspberry Pi, the I2C device must be accessible (udev rules set mode 0666). On the Pi 5 client the BNO055 runs on the software i2c-gpio bus `/dev/i2c-8` (GPIO2/3), because the hardware controller does not reliably honour the BNO055's clock stretching; see `ansible/README.md`, "BNO055 I2C bus".
- **Mode**: set by `operation_mode`. `IMUPLUS` (default) fuses accel+gyro only, so heading has an arbitrary zero. `NDOF` / `NDOF_FMC_OFF` also use the magnetometer: orientation yaw is then absolute, referenced to **magnetic** north (add the local declination for true north). The magnetometer needs calibration by moving the sensor through varied orientations (watch `mag` on `/imu/calibration`, 3 = calibrated) and is disturbed by motors and servos near the sensor. The quaternion is published as the chip reports it (no axis remap): flat and facing magnetic north it is the identity, yaw increases counter-clockwise (x north, y west, z up). When fusion fails, falls back to raw acceleration; orientation then published as identity with covariance -1. After mode switch, the node waits 1.5 s and runs a warm-up phase (up to 3 s) until the sensor returns valid gyro+accel; this addresses BNO055 needing time for fusion to stabilize after power-on or service restart.

## Build and run

From repo root:

```bash
cd nodes/bridges/bno055_imu
uv sync
uv run poe lint
uv run poe test
```

Deploy to target via Ansible:

```bash
./scripts/deploy-nodes.sh client bno055_imu
```

## Dependencies

- `adafruit-circuitpython-bno055`: BNO055 driver.
- `adafruit-blinka`: Hardware abstraction (I2C).
- `adafruit-extended-bus`: Optional; used on Linux to select I2C bus by number.
