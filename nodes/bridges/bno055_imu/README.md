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
  CPU of this process and counts against the unit's `cpu_quota`. Fewer transactions per cycle is what lets the node reach
  `publish_hz`; watch `imu_cycle_seconds` and `imu_cycle_overruns_total` when tuning `publish_hz` or the quota.

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
  `soft restores exhausted (...)`, `watchdog: N s without data` or `I2C hard errors, soft restore not possible`.
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
- **Not persisted**: the bias is relearned after each restart (about one time constant of stillness). Unlike the
  sensor offsets in `calibration.json` it is a software estimate that drifts with temperature, so a stored value would go
  stale and would need its own invalidation; relearning is simpler and always current.
- **Watch it**: `imu_gyro_bias_rad_s{axis="z"}` should settle near -5.8e-4 and `imu_gyro_bias_learning` is 1 while parked.

## Calibration persistence

The BNO055 forgets its calibration on every reset, so each node restart used to drop `/imu/calibration` to 0,0,0,0.
The node now keeps the sensor offsets in `calibration_file` (JSON: `accel_offset`, `gyro_offset`, `mag_offset`,
`accel_radius`, `mag_radius`).

- **Restore**: on every chip init (start and I2C reconnect) the offsets are written in CONFIG mode before the chip is
  switched to `operation_mode`; the log shows `restored calibration profile from <file>`. A missing file means an
  uncalibrated start. A corrupt or implausible file (missing keys, values outside int16, radius outside 1-2000) logs a
  warning and the node starts uncalibrated.
- **Save**: at most every `calibration_save_interval_s`, when `mag` reports 3 (only the magnetometer is gated: a wheeled
  robot cannot do the accelerometer 6-orientation motion, and the compass heading needs only the mag offsets),
  the node briefly switches to CONFIG mode, reads the offsets, switches back and writes the file atomically (temp file
  plus `os.replace`) only if it differs from what is on disk; the log shows `saved calibration profile`. That cycle
  publishes nothing while fusion restarts (about 1.5 s). The adafruit_bno055 offset properties do not switch modes
  themselves (library 5.4.22), so the node does it explicitly.
- **Restored flag**: `/imu/calibration` JSON carries `restored: true` once a saved profile is on the chip (restored at
  init, or just saved). The chip itself reports 0 after a restart until it re-checks, so consumers (web_ui compass
  anchor) trust `restored` instead of waiting for `mag` to climb again.
- **Reset**: delete the file and restart the node (`sudo rm /var/lib/ros2/bno055_imu/calibration.json`), then redo the
  calibration motion. Ansible creates `/var/lib/ros2/bno055_imu` owned by the node user.

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
