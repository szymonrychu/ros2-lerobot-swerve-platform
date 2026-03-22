---
name: bno055-diagnostics
description: Use when checking BNO055 IMU status, debugging I2C issues, verifying sensor data on /imu/data, or after deploying the bno055_imu node to the client RPi
---

# BNO055 IMU Diagnostics

Run `scripts/bno055_diag.sh` for a complete one-command IMU diagnostic. Never SSH manually to check IMU status — the script handles everything and requires no approvals.

## Quick Reference

```bash
./scripts/bno055_diag.sh              # Full one-shot diagnostic
./scripts/bno055_diag.sh --watch      # Repeat every 10s (Ctrl+C to stop)
./scripts/bno055_diag.sh --watch --interval 5
./scripts/bno055_diag.sh --logs-only  # Only recent service logs
./scripts/bno055_diag.sh --lines 30   # More log lines (default: 20)
./scripts/bno055_diag.sh --no-scraper # Skip topic_scraper query
```

## Output Sections

| Section | What to look for |
|---------|-----------------|
| Service | `active` = container running. `inactive` = check logs for crash reason. |
| I2C scan | `BNO055 found at 0x28` (or 0x29). If NOT found, sensor not wired or not powered. |
| IMU logs | `warm-up complete` = sensor ready. `warm-up timed out` = I2C unreliable. `consecutive failures` = persistent I2C errors triggering reconnect. |
| Topic scraper | Non-zero accel/gyro + `orientation_covariance[0] != -1` = sensor calibrated and publishing. |

## Interpreting Results — Common Patterns

| Symptom | Cause | Fix |
|---------|-------|-----|
| Service `inactive` | Container crashed or device not mounted | Check logs for import error or `PermissionError`; verify `--device=/dev/i2c-1` in container args |
| `BNO055 NOT found` in I2C scan | Wrong address, wiring, or module not powered | Check physical connection; try `i2c_address: 0x29` in config |
| `warm-up timed out` | Sensor not returning valid gyro/accel within 3 s | Keep sensor still for a few seconds; power-cycle module; check I2C baudrate |
| `consecutive failures — attempting I2C reconnect` | Intermittent I2C bus errors | Check wiring integrity; verify `i2c_baudrate` Ansible var is ≤ 400000 |
| `reconnect failed` | Hardware-level I2C failure | Power-cycle client RPi; check `/dev/i2c-1` is accessible inside container |
| `orientation_covariance[0]=-1` in scraper | Orientation unknown — quaternion invalid or sensor uncalibrated | Move sensor slowly in figure-8; wait for sys/gyro/accel calibration ≥ 1 |
| All accel/gyro zero in scraper | Data valid but all readings zero | Check sensor not on perfectly flat surface; verify it's not publishing placeholder data |
| No topic_scraper data | `topic_scraper_api` not running or wrong port | Check `ros2-topic_scraper_api` service on client; default port 18100 |
| `i2cdetect unavailable` | `i2c-tools` not installed on client | `sudo apt install i2c-tools` on client RPi |

## Service and Node Details

- **Service:** `ros2-bno055_imu.service` on `client.ros2.lan`
- **Topic:** `/imu/data` (type: `sensor_msgs/Imu`)
- **I2C:** bus 1 (`/dev/i2c-1`), default address 0x28 (fallback 0x29)
- **Config:** `/etc/ros2/bno055_imu/config.yaml`
- **Key log markers:** `BNO055 IMU: publishing` (startup), `warm-up complete`, `calibration (sys, gyro, accel, mag):`

## Calibration Info

BNO055 calibration status is logged at startup:
```
BNO055 calibration (sys, gyro, accel, mag): 0, 1, 1, 0  (0=uncal, 3=full)
```
- `sys=3` = fully calibrated; lower values = partial
- Calibration is lost on power cycle; the sensor re-calibrates automatically during use
- `orientation_covariance[0]=-1` means quaternion is not trusted yet (keep sensor still or move slowly)
