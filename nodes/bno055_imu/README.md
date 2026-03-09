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
- **Host**: Map the I2C device into the container, e.g. `--device=/dev/i2c-1:/dev/i2c-1`. On Raspberry Pi, enable I2C and use the correct bus (typically `i2c_bus: 1`).
- **Mode**: The node uses **ACCGYRO** (raw accel+gyro, no fusion) for reliable output. When still, `linear_acceleration` shows gravity (~9.8 m/s²); angular velocity and linear acceleration change when the module moves. Orientation is published as identity with covariance -1 (unknown).

## Build and run

From repo root:

```bash
cd nodes/bno055_imu
poetry install
poetry run poe lint
poetry run poe test
```

Docker (from repo root, build context `nodes/bno055_imu`):

```bash
docker build -t bno055_imu:latest nodes/bno055_imu
docker run --rm --device=/dev/i2c-1:/dev/i2c-1 \
  -v /etc/ros2/bno055_imu:/etc/ros2/bno055_imu:ro \
  -e BNO055_IMU_CONFIG=/etc/ros2/bno055_imu/config.yaml \
  --network host \
  bno055_imu:latest
```

(Ensure ROS2 is sourced inside the image; the Dockerfile uses `source /opt/ros/jazzy/setup.bash`.)

## Dependencies

- `adafruit-circuitpython-bno055`: BNO055 driver.
- `adafruit-blinka`: Hardware abstraction (I2C).
- `adafruit-extended-bus`: Optional; used on Linux to select I2C bus by number.
