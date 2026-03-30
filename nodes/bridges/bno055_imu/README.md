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

## Hardware

- **Sensor**: BNO055 (Bosch 9-DOF) over I2C.
- **Default address**: `0x28`; alternate `0x29` depending on ADR pin.
- **Fallback behavior**: Node tries configured `i2c_address` first, then falls back to alternate BNO055 addresses.
- **Host**: On Raspberry Pi, enable I2C and ensure `/dev/i2c-1` is accessible (udev rules set mode 0666).
- **Mode**: The node uses **IMUPLUS** fusion: publishes orientation (quaternion), angular velocity, and linear acceleration. When fusion fails, falls back to raw acceleration; orientation then published as identity with covariance -1. After mode switch, the node waits 1.5 s and runs a warm-up phase (up to 3 s) until the sensor returns valid gyro+accel; this addresses BNO055 needing time for fusion to stabilize after power-on or service restart.

## Build and run

From repo root:

```bash
cd nodes/bridges/bno055_imu
poetry install
poetry run poe lint
poetry run poe test
```

Deploy to target via Ansible:

```bash
./scripts/deploy-nodes.sh client bno055_imu
```

## Dependencies

- `adafruit-circuitpython-bno055`: BNO055 driver.
- `adafruit-blinka`: Hardware abstraction (I2C).
- `adafruit-extended-bus`: Optional; used on Linux to select I2C bus by number.
