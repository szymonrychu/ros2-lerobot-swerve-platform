# Scripts

Helper scripts for deployment, telemetry, and GPS RTK.

## GPS RTK

Run these from your computer (or from a host with SSH and network access to server/client).

### 1. Calibration (base station survey-in)

**One-time** survey-in of the LC29H(BS) base on the server. Takes about 1–2 hours.

```bash
# From repo root: run calibration on server via SSH (stops gps_rtk_base, runs survey-in, then instructs next steps)
./scripts/rtk_calibrate.sh

# Optional: custom samples and accuracy (default: 3600 samples, 15 m)
./scripts/rtk_calibrate.sh --samples 3600 --accuracy 15

# Skip factory restore (if module already reset)
./scripts/rtk_calibrate.sh --no-restore

# Run locally (you must be on the server with /dev/ttyAMA0)
./scripts/rtk_calibrate.sh --local
```

**Env:** `RTK_SERVER_HOST` (default `server.ros2.lan`), `RTK_SSH_USER` (from Ansible inventory or `$USER`).

After calibration, the script tells you to **power cycle** the base (unplug/plug HAT or reboot) and start the service again.

### 2. Verify fixes (topic scraper)

Requires **topic_scraper_api** running on server and client (port 18100) and **jq** installed.

```bash
# One-shot: print latest /server/gps/fix and /client/gps/fix (status, lat, lon, alt)
./scripts/rtk_verify.sh

# Poll every 5 s until Ctrl+C
./scripts/rtk_verify.sh --watch

# Capture for 60 s, then show client status distribution and last fixes
./scripts/rtk_verify.sh --capture 60
```

**Fix status:** `-1` = no fix, `0` = GPS fix, `1` = SBAS, `2` = **RTK fix** (goal for rover).

**Env:** `RTK_SERVER_HOST`, `RTK_CLIENT_HOST` (defaults `server.ros2.lan`, `client.ros2.lan`), `SCRAPER_PORT` (default `18100`).

### 3. Quick status (services + RTCM port)

```bash
# Service and RTCM port check only
./scripts/rtk_status.sh

# Include one-shot fix from topic scraper
./scripts/rtk_status.sh --fix
```

**Env:** `RTK_SERVER_HOST`, `RTK_CLIENT_HOST`, `RTK_SSH_USER`.

### 4. Post-calibration (base survey-in done + power cycle)

After the base is calibrated and the server RPi has been power-cycled:

1. **Start the base** (if it didn’t auto-start):  
   `ssh $USER@<server> 'sudo systemctl start ros2-gps_rtk_base'`
2. **Check stack**:  
   `./scripts/rtk_status.sh` → both services active, server TCP 5016 listening.
3. **Check fix quality**:  
   `./scripts/rtk_verify.sh` or `./scripts/rtk_verify.sh --fix` → client should show `status=2` (RTK fix) once corrections are flowing and the rover has converged.
4. **Optional capture**:  
   `./scripts/rtk_verify.sh --capture 60` → client status distribution and last fixes (confirm sustained RTK fix and position).

Run these from a machine that can reach both server and client (e.g. your laptop on the same network). Topic scraper must be running on both hosts (port 18100) for verify steps.

### Swerve navigation debug (relative goal)

Send a Nav2 goal relative to the current robot pose (e.g. "1 m forward") for testing the swerve + Nav2 stack. Requires ROS2 Jazzy sourced and Nav2 running (e.g. on client).

```bash
# From a machine that can reach the client ROS2 graph (e.g. with ROS_DOMAIN_ID and DDS discovery, or SSH + X11 for local):
source /opt/ros/jazzy/setup.bash
python scripts/swerve_goal_relative.py --dx 1.0 --dy 0
python scripts/swerve_goal_relative.py --dx 0.5 --dy 0.3 --dtheta 0.2
```

- **--dx, --dy, --dtheta**: Offset in robot frame (m, m, rad). Goal = current pose + offset.
- **--odom-topic**: Odometry topic (default `/odom` or `SWERVE_GOAL_ODOM_TOPIC`).
- **--action**: NavigateToPose action name (default `/navigate_to_pose`).
- **--frame-id**: Frame for goal pose (default `odom` for odom-relative goals).

### IMU (BNO055) verification

The client runs the BNO055 IMU node publishing `sensor_msgs/Imu` on `/imu/data`. The **topic_scraper_api** (client, port 18100) allows type `sensor_msgs/msg/Imu`; use it to confirm the IMU is publishing.

```bash
# From a host that can reach the client (e.g. client.ros2.lan:18100):
# List topics (should include /imu/data with has_sample: true when node is healthy)
curl -s http://<client>:18100/topics | jq '.topics[] | select(.topic == "/imu/data")'

# Latest IMU sample (orientation, angular_velocity, linear_acceleration)
curl -s http://<client>:18100/topics/imu/data | jq .

# Stream IMU orientation quaternion (x,y,z,w) at 0.2 s interval
python scripts/topic_scraper_collect.py \
  --source client=http://<client>:18100 \
  --select '/imu/data:.orientation' \
  --interval 0.2
```

If `/imu/data` has no sample, check `ros2-bno055_imu` service and logs. The node publishes only when it has valid gyro and accel data from the sensor; it does not publish placeholder data.

### Post-deploy verification (topic scraper)

After deploying ROS2 nodes, verify cross-host connectivity and topic flow using `topic_scraper_collect.py`. Run from a host that can reach both Pis (e.g. dev machine with `/etc/hosts` entries for `server.ros2.lan` and `client.ros2.lan`):

```bash
python scripts/topic_scraper_collect.py \
  --source client=http://client.ros2.lan:18100 \
  --source server=http://server.ros2.lan:18100 \
  --select '/leader/joint_states:.position' \
  --select '/follower/joint_states:.position' \
  --interval 0.1 \
  --once
```

**Validation checklist:**

- Topics return non-empty payloads (no timeouts/404).
- Values are not all zeros (e.g. `position` has non-zero entries when arms are connected).
- Both `client` and `server` sources emit data.
- Save a short capture and inspect with `jq` for structure and non-zero values:  
  `... > scrape_capture.ndjson` then `jq . scrape_capture.ndjson | head`

**Suggested test matrix:** `/leader/joint_states` (server), `/follower/joint_states` (client), `/filter/input_joint_updates` (client), `/odom` or `/imu/data` if available, `/server/gps/fix` and `/client/gps/fix` (expect zeros until RTK fix).

### Other

- **calibrate_rtk_base.py** — Low-level calibration script (run on the machine that has the base serial port). Used by `rtk_calibrate.sh` when not using `--local`.
- **topic_scraper_collect.py** — Generic NDJSON collector for topic_scraper_api; used by `rtk_verify.sh`.
