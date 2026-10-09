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
