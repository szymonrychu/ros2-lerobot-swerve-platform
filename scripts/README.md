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

Requires **topic_scraper_api** running on server and client (port 18100) and **jq** installed. topic_scraper_api is disabled by default (`enabled: false`): set `enabled: true` for it in `ansible/group_vars/<host>.yml` and run `scripts/deploy-nodes.sh <host> topic_scraper_api` before using the scraper workflow; otherwise connections to port 18100 are refused.

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
- **topic_scraper_collect.py** — Generic NDJSON collector for topic_scraper_api (disabled by default, `enabled: true` to use it; prints a hint on connection refused); used by `rtk_verify.sh`.

## swerve_diag.sh

Read-only health check of the swerve drive on the client: service states (`base_servos` hosts the swerve servos on the second adapter, `swerve_controller`), rates of `/swerve_drive/joint_states`, `/swerve_drive/joint_commands`, `/odom`, `/cmd_vel`, one joint-state / odom sample, and recent warnings from both services.

```bash
./scripts/swerve_diag.sh
./scripts/swerve_diag.sh --logs-only --lines 40
```

## costmap_zones_verify.sh

Motion-free robot check of the keepout / slow zone chain (POI area zones -> costmap_zones masks -> Nav2 KeepoutFilter and SpeedFilter). Safe while the robot is charging: it only adds and deletes `E7-verify-*` POIs on `/poi/command` and asks the planner for paths with `compute_path_to_pose`; it never sends a velocity command, and the verifier refuses to run while `/navigate_to_pose` has an active goal.

- **Step 0** (in the script, over ssh): `ros-jazzy-nav2-costmap-2d` >= 1.3.10 (`dpkg --compare-versions`) and `systemctl is-active` for `ros2-costmap_zones`, `ros2-poi_store`, `ros2-nav2_bringup`.
- **Steps 1-7** (`python -m costmap_zones.verify` in the node venv on the client, `ROS_ENV` as `swerve_diag.sh`, geometry from `nodes/costmap_zones/verify.yaml`): masks on the `/map` grid with valid values; a control path through a 0.3 x 1.0 m rectangle 0.6 m ahead; a keepout on it (global costmap 99/100, planner fails or avoids it); a keepout around the robot (a path out still succeeds); a slow zone on it (keepout mask holds the slow value, global costmap 59, never 99/100, planner still succeeds); a 40 % slow zone around the robot (`/speed_limit` 40 percent, then 0); and, always, the deletion of every `E7-verify-*` POI. Details in `nodes/costmap_zones/README.md`.

One `PASS`/`FAIL` line per step, then `RESULT:` and `OVERALL:`; the exit code is non-zero on any failure.

```bash
mkdir -p .logs
./scripts/costmap_zones_verify.sh 2>&1 | tee .logs/costmap-zones-verify.log
```

Env: `COSTMAP_ZONES_CLIENT_HOST` (default `client.ros2.lan`), `COSTMAP_ZONES_SSH_USER` (default from `ansible/inventory` or `$USER`), `COSTMAP_ZONES_REMOTE_REPO` (default `/opt/ros2-lerobot-swerve-platform`), `COSTMAP_ZONES_REMOTE_VENV` (default `/opt/ros2-nodes/costmap_zones/venv`).

## mppi_overlay_diag.sh

Read-only check that the patched `nav2_mppi_controller` overlay is built and loaded on the client: the `/opt/ros2-ws/.built-nav2_mppi_controller-*` build stamp, the dpkg version of `ros-jazzy-nav2-mppi-controller` against the tag pinned in `ansible/group_vars/client.yml` (prints `WARN` when they differ, i.e. time to bump the pin), the `libmppi_controller.so` / `libmppi_critics.so` paths mapped by the nav2 process (`/proc/<pid>/maps`; `/opt/ros2-ws/install/...` is the overlay, `/opt/ros/jazzy/...` the apt library), `ros2 param get /controller_server FollowPath.model_delay_vx/vy/wz`, and recent `ros2-nav2_bringup` journal lines matching "missed its desired rate". Env: `MPPI_CLIENT_HOST`, `MPPI_SSH_USER`.

```bash
./scripts/mppi_overlay_diag.sh
./scripts/mppi_overlay_diag.sh --lines 40
```

## d435_diag.sh

Read-only RealSense D435 diagnostic for the client (`ros2-realsense_d435`). It never starts a ROS participant or Python on the client (no rclpy probes there): SSH runs only `systemctl`, `journalctl`, `lsusb`, sysfs and device-tree reads, `vcgencmd`, `curl` to the local topic_scraper_api, and with `--device` `rs-enumerate-devices`. How to read the output: `.claude/skills/d435-diagnostics/SKILL.md`.

- **default**: unit state; `lsusb -d 8086:0b07` and the sysfs `speed` of the camera (expect `5000`, USB 3); the latest `Device FW version` / `Device USB type` startup lines of the unit's journal; PSU and USB current (`vcgencmd get_config usb_max_current_enable`, the PSU max current and USB over-current flag from `/proc/device-tree/chosen/power`, any `5A` PSU line of this boot's journal); counts of `VIDIOC_QBUF`, `No such device`, `Frame didn't arrive` over `--since` (default `30 min ago`); the last `--lines` journal lines.
- **--topics**: every `/head/d435/` topic with its type from topic_scraper_api `GET /topics`.
- **--rates**: the rate of every `/head/d435/*/camera_info` topic from its header stamps. The scraper keeps only the latest sample per topic (its `sample_seq` is global), so the script polls those endpoints with curl on the client for `--interval` seconds and computes (distinct stamps - 1) / stamp span; when every poll saw a new frame it marks the rate as a lower bound.
- **--device**: `rs-enumerate-devices -s` and `-c` (serial, firmware, intrinsics, Depth-to-Color extrinsic), only while the unit is inactive or failed; it refuses (exit 1) while the unit runs, since the driver holds the camera.
- `--watch` repeats the mode every `--interval` seconds (default 10). `--topics` and `--rates` exit 2 with a hint when topic_scraper_api is unreachable (it is disabled by default; `enabled: true` in `ansible/group_vars/client.yml`, then `scripts/deploy-nodes.sh client topic_scraper_api`). Its `allowed_types` include `sensor_msgs/msg/CameraInfo` for `--rates`.

Env: `D435_CLIENT_HOST` (default `client.ros2.lan`), `D435_SSH_USER` (default from `ansible/inventory` or `$USER`), `SCRAPER_PORT` (default 18100).

```bash
./scripts/d435_diag.sh
./scripts/d435_diag.sh --since "2 hours ago" --lines 40
./scripts/d435_diag.sh --topics
./scripts/d435_diag.sh --rates --interval 5
./scripts/d435_diag.sh --device
```

## mppi_cpu_compare.py

Compares the CPU of `ros2-nav2_bringup.service` before and after the overlay deploy from the server's Mimir (`prometheus_url: http://server.ros2.lan:9009/prometheus`, `host: client`). Queries `rate(container_cpu_usage_seconds_total{host, id="/system.slice/<unit>"}[1m])` over `[switch - window_h, switch]` and `[switch + settle_min, switch + settle_min + window_h]` (the settle period after the restart is left out) and prints mean and p95 of both windows. PASS when after <= `max(before * (1 + rel_limit), before + abs_limit_cores)` for both statistics; exit 1 on FAIL, exit 2 when a series is empty or Mimir is unreachable (never a PASS). `switch_time: auto` takes the latest `container_start_time_seconds` of the unit (the deploy restart), or set an epoch in seconds. Configured by `scripts/mppi_cpu_compare.yaml` (pydantic-validated: defaults `window_h` 6, `settle_min` 10, `rel_limit` 0.05, `abs_limit_cores` 0.01). Mimir keeps 1 d of metrics, so the before window must still be in retention. See `nodes/nav2_bringup/README.md` (Verifying the MPPI overlay).

```bash
uv run python scripts/mppi_cpu_compare.py
```

## node_cpu_compare.py

Per-node CPU before/after table from the server's Mimir, for the C++ cut-over (map_guard -> map_guard_cpp) and the cheap Python fixes (web_ui, gripper_uvc_camera, mcp_server). For each entry `{name, before_unit, after_unit, switch_time}` of `scripts/node_cpu_compare.yaml` (pydantic-validated, with `prometheus_url`, `host`, `window_h` 6, `settle_min` 10) it queries `rate(container_cpu_usage_seconds_total[1m])` of `/system.slice/<before_unit>` over `[switch - window_h, switch]` and of `<after_unit>` over `[switch + settle_min, switch + settle_min + window_h]`, and prints mean and p95 in cores, plus a `total` row (the sum of the rows' statistics). The query and statistics functions are imported from `mppi_cpu_compare.py`. An empty series or an unreachable Mimir exits 2, never a result. Set each `switch_time` to the epoch of that node's deploy (`date +%s` right after it; the shipped `1` fails the run on purpose); Mimir keeps 1 d, so the before window must still be in retention. Run from the repo root after the C++ cut-over deploy:

```bash
uv run python -m scripts.node_cpu_compare
```

## probe-mppi-overlay-docker.sh

Docker probe (arm64 `ros:jazzy-ros-base`, runs natively on an Apple Silicon Mac) for the nav2 MPPI overlay patch
`nodes/nav2_bringup/patches/nav2_mppi_controller-0001-model-delay-pr6154.patch` (Nav2 PR #6154, `model_delay_vx/vy/wz`,
ported to xtensor for Jazzy 1.3.13). It clones navigation2 at the constant `PIN`
(`f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`, tag 1.3.13 = the apt version), runs `git apply --check` and applies the
patch, checks it touches only `nav2_mppi_controller/`, `rosdep install`s the package deps, `colcon build`s
`nav2_mppi_controller` with `-DBUILD_TESTING=ON`, runs `colcon test` with
`ctest -R 'motion_model_tests|optimizer_unit_tests|optimizer_smoke_test'` and `colcon test-result --verbose`. Prints
`PASS`/`FAIL` per step and `ALL CHECKS PASSED` at the end; exits non-zero on any failure. The first run downloads the
image and the build deps (several minutes). `PROBE_JOBS` (default 4) sets the make parallelism. `PIN` must equal the
overlay source commit in `ansible/group_vars/client.yml`; run the probe after every pin bump, before deploying (see
`nodes/nav2_bringup/README.md`, MPPI overlay (model_delay)).

```bash
mkdir -p .logs && ./scripts/probe-mppi-overlay-docker.sh 2>&1 | tee .logs/probe-mppi-overlay.log | tail -40
```

## cpp-node-test.sh

Builds and tests one in-repo C++ (ament_cmake) package in docker, identically on the Mac and in CI. Usage:
`scripts/cpp-node-test.sh <pkg> [--lint-only]`, where `<pkg>` is `ros2_common_cpp` or the name of a `nodes/<name>_cpp`
package (the directory name equals the `package.xml` name).

1. Builds, or reuses, the local image `cpp-node-test:<hash>` from `scripts/cpp-node-test.Dockerfile`: `FROM
   ros:jazzy-ros-base` on the native platform (arm64 on an Apple Silicon Mac, the Pi's architecture; amd64 in CI),
   plus colcon and the rosdep keys of every in-repo C++ package. The tag is a hash of the Dockerfile and every
   `package.xml`, so the image is rebuilt only when a dependency changes. The Dockerfile also maps the rosdep key
   `prometheus-cpp` (no upstream rule) to apt `prometheus-cpp-dev`.
2. Mounts the repo read-only at `/repo` and copies `shared/ros2_common_cpp`, `nodes/*_cpp` and `.clang-format` into
   `/ws/src`, keeping the repo layout.
3. `rosdep install`, then `colcon build --packages-up-to <pkg> --cmake-args -DBUILD_TESTING=ON
   -DCMAKE_BUILD_TYPE=Release`.
4. `colcon test --packages-select <pkg>` (with `--lint-only` only the ament linters: `ctest -L linter`), then
   `colcon test-result --verbose`, which must report at least one test and no failure. `cppcheck` really runs
   (`AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1`; ament skips cppcheck 2.x otherwise).

Exits non-zero on any build, test or lint failure, and always ends with one line `cpp-node-test: <pkg> PASS` or
`cpp-node-test: <pkg> FAIL`. An unknown package exits 2 and lists the known ones. It generalizes the container
pattern of `probe-mppi-overlay-docker.sh` (`docker run --rm -i`, repo mounted `:ro`, inline script on stdin). The root
poe tasks `test-cpp` and `lint-cpp` run it for every C++ package, and the `cpp` CI job runs it per package.

```bash
mkdir -p .logs && ./scripts/cpp-node-test.sh ros2_common_cpp 2>&1 | tee .logs/cpp-node-test.log | tail -20
uv run poe test-cpp      # every C++ package, full build + gtests + lint
uv run poe lint-cpp      # every C++ package, linters only
```

To fix a clang-format finding, reformat in the same image:
`docker run --rm -v "$PWD:/repo" -w /repo cpp-node-test:<hash> bash -c 'source /opt/ros/jazzy/setup.bash &&
ament_clang_format --config .clang-format --reformat shared/ros2_common_cpp'` (`docker images cpp-node-test` shows
the tag).

## propose_unit_limits.py

Prints proposed `MemoryLow`, `MemoryMax`, `CPUWeight` and `IOWeight` for the `ros2-*` systemd units in the robot-headroom model (no `CPUQuota`, no `MemoryHigh`) from metrics history (p99 CPU, typical (p50) and peak memory working set) next to the values configured in `ansible/group_vars/client.yml`: `MemoryLow` = typical working set rounded up to 8 MiB, `MemoryMax` = 2x the peak (a larger current cap kept), weights = the configured tier (else 500), plus the sum of the proposed `MemoryLow`. Flags: `oom risk`, `tight max`, `unprotected`, `throttle risk` (only where a `CPUQuota` is still set). It queries the Prometheus API of the server's Mimir (`prometheus_url: http://server.ros2.lan:9009/prometheus`, 1 d retention), which stores both hosts' series, and selects one host with `host: client` (the `host` label set by Alloy; use `server` together with `group_vars_path: ansible/group_vars/server.yml` for the server's units). Configured by `scripts/propose_unit_limits.yaml` (URL, host, window, percentiles, `memory_max_factor`, minimums, default weights, `format: table|yaml`); read-only, it never edits `client.yml`. See `ansible/README.md` (Node resource model).

```bash
uv run python scripts/propose_unit_limits.py
```

## arm_zero_restore.py

One-off repair of the follower arm zero after a neutral save (DefineMiddle) recorded in the wrong pose: with the arm resting folded and torque off, it rewrites each STS3215 offset register (address 31, sign-magnitude, sign bit 11, +-2047) so the present reading, mapped to joint radians exactly as the feetech bridge publishes `/follower/joint_states` (`feetech_mapping.command_mapping.position_to_raw_steps` / `steps_to_radians` with the joint's `inverted` flag), equals the reference reading taken before the incident. Nothing moves and torque is never written.

Sign convention: present = (encoder - offset) mod 4096. A neutral save logged as `neutral recorded: <joint> +N steps` (present - 2048, `neutral.py`) raised that offset by N; the restore changes the offset by present - target, the inverse of the logged shift (shoulder_lift about +901, elbow_flex about -1029).

Flow: refuse while `bridge_service` (`ros2-lerobot_follower`) is active; refuse unless every listed servo reads `torque_enable` 0; read present, offset and angle limits; plan per joint the target, offset change and new offset; refuse with nothing written when any change differs from `expected_offset_delta_steps` by more than `tolerance_deg` (5 deg = 56.9 steps), a new offset exceeds +-2047 or a read fails. Dry run (default) prints the table and stops. `--apply` per joint in config order: re-read torque, `UnLockEprom`, write the offset, `LockEprom` (as `neutral.py`), settle 0.5 s, verify present within `verify_tolerance_steps` (3) of the target (else rewrite the old offset and stop), and write the angle limit registers back if they read differently (as `neutral.py` after DefineMiddle). Every run that reached the bus writes a JSON record (`record_path`, default `/var/lib/ros2/arm/zero_restore-<UTC timestamp>.json`) with the before/after per joint. Exit 0 when the plan is valid (dry run) or every joint is applied and verified, else 1.

Configured by `scripts/arm_zero_restore.yaml` (pydantic, works with Ubuntu's pydantic 1.10 and pydantic 2): `device`, `bridge_service`, `record_path`, `tolerance_deg`, `verify_tolerance_steps`, and per joint `name`, `id`, `inverted`, `reference_rad`, `expected_offset_delta_steps`. Device, names, ids and inverted flags are pinned to the `lerobot_follower` entry of `ansible/group_vars/client.yml` by `tests/test_arm_zero_restore.py`. Flags: `--config <path>` and `--apply`.

Runs on the client RPi in the follower's deployed venv (st3215, feetech_mapping; pydantic and pyyaml from the system site packages), from the repo checkout, with the bridge stopped. Procedure: `docs/runbooks/servo-torque-neutral.md` "Arm zero restore".

```bash
sudo systemctl stop ros2-lerobot_follower
cd /opt/ros2-lerobot-swerve-platform
PYTHONPATH=nodes/bridges /opt/ros2-nodes/lerobot_follower/venv/bin/python3 scripts/arm_zero_restore.py            # dry run
PYTHONPATH=nodes/bridges /opt/ros2-nodes/lerobot_follower/venv/bin/python3 scripts/arm_zero_restore.py --apply    # write
sudo systemctl start ros2-lerobot_follower
```

## calibrate_retry_threshold.py

Calibrates `retry_success_threshold` (the cut-off the claude_agent runner compares the model's `p_retry_success` with) from the retry ledger. It reads the records from `GET /api/retry_outcomes` (through the web_ui proxy, `source_url`, 10 s timeout) or from the ledger JSONL (`source_file`, exactly one of the two), skips records with an unknown `schema_version` (counted), and optionally filters by `since_ts` and `model`. It prints record counts (total / observed / unobserved, overall and per bin), the Brier score of the observed records next to the base-rate reference Brier, a reliability table (n, mean p, success rate, Wilson CI via `scipy.stats.binomtest(...).proportion_ci(method='wilson')`, low-n flag), ECE and the success rate per `eai_error_type`. A record counts as a success when its next outcome is `done`; `unobserved` records are excluded from every score.

The proposal is the smallest threshold on a 0.05 grid whose observed `p >= t` subset has at least `min_samples` records and a Wilson lower bound (at `confidence`) of at least `target_success_rate`, never below the minimum observed p. Otherwise it prints `insufficient data, keep <current>` (current = the most recent record's threshold). A proposal is printed as a paste-ready line, for example `retry_success_threshold: 0.62  # calibrate_retry_threshold.py 2026-10-20 n=84 Brier 0.18`; applying it to `ansible/group_vars/client.yml` is a manual edit and a deploy. Censoring: at threshold 0.5 low-p predictions are only observed through loop re-entries, so the calibration covers about p >= 0.5; to explore lower p set `retry_success_threshold: 0.0` in group_vars for an evaluation session and restore it afterwards. Calibration trusts the model's self-reported outcomes; if the evaluation harness shows low agreement with human labels, do not trust the proposal.

Configured by `scripts/calibrate_retry_threshold.yaml` (pydantic `Settings`: `source_url` / `source_file`, `since_ts`, `model`, `bins`, `target_success_rate`, `confidence`, `min_samples`, `min_bin_samples`). The config path is the positional argument; the only flag is `--json <path>`, which writes the machine-readable result. It never writes any other file.

```bash
uv run python scripts/calibrate_retry_threshold.py scripts/calibrate_retry_threshold.yaml
uv run python scripts/calibrate_retry_threshold.py scripts/calibrate_retry_threshold.yaml --json .logs/calibration.json
```

## eval/ (evaluation harness)

Python package `scripts.eval` for per-stage evaluation of the claude_agent on a fixed pick-and-place test set. The test set, stage derivation and report (`testset.py`, `stages.py`, `report.py`) are driven by `run.py` with the HTTP client (`client.py`), the labelling prompts (`labels.py`) and the run config (`run.yaml`).

- `testset.py` / `testset_v1.yaml`: pydantic `TestSet` with `version`, `place_target`, exactly 10 objects and exactly 3 placements (unique ids, at least one `arm_only: true`), `prompt_template` (placeholders `{object_name}`, `{placement_description}`, `{place_description}` only) and `arm_only_suffix`. `cases(mode)` expands object-major: `full` gives all 30 cases, `arm_only` only the arm-only placements (v1: 10 x 2 = 20). `render_prompt(case, mode)` fills the template and appends the arm-only suffix in arm-only mode. v1 has 10 household and plush objects, two placements within arm reach (`front_center`, `front_left`) and one that needs driving (`far_room`). A test set file is never edited in place: a change gets a new version file.
- `stages.py`: `derive(events)` turns one trial's claude_agent events (user_message through turn_end) into the agent-claimed outcome (`ok`, `fail`, `not_reached`) of each stage, only from tool_call / tool_result / phase_completed, never from phase names. navigate: navigate_to_pose / move_relative / drive, ok when the last result is not an error. locate: perception calls (camera images, pixel_to_ground, pixel_to_point, camera_point_to_arm, mark_candidate_points, resolve_candidate, plan_grasp, look_around) before the first grasp_object, ok when a grasp_object or a `planned` plan_grasp follows. grasp: grasp_object calls (attempts = their number), ok when the last outcome is `grasped` or `unverified`. verify: a gripper image or get_arm_state after the last grasp followed by a phase_completed, ok when that phase closed done with no release_object before it outside a place-stage phase. place: the last release_object after the last grasp, ok when it released and its phase closed done. Each phase maps to the stage whose tools it called most (a tie goes to the later stage); its attempts are the highest phase_completed attempt, and every failed close keeps its `failure` record. Runner-made calls (events with `source`) are ignored. It also returns lookup ids, the instruction id and the turn_end payload.
- `workflow_checks.py`: `derive_workflow(events)` returns a `WorkflowDerivation` (workflow from the agent's start_workflow call, skipped mandatory ids from the turn_end `job_end.workflow`, retrospective author, survey POI writes, loaded skills); `report.py` stores it in `TrialRecord.workflow` (optional, old lines load) and adds a workflow section (trials per workflow, share without skipped mandatory steps, share with an agent retrospective, mean survey POI writes); `run.py` prints it per trial.
- `report.py`: `TrialRecord` (one line of `trials.jsonl`: run, test set version, mode, case, trial, instruction id, timestamps, turn_end status, `timed_out` (the harness hit `trial_timeout_s` and POSTed stop; the report counts these as `timeouts`, apart from interrupts), cost, turns, effector calls, claimed and human stage labels, human success and first failed stage, phases, lookup ids, note) and `trial_record_from_derivation`. `build(trials, k, cases_planned)` returns a JSON-ready dict: coverage (cases run of 30, planned cases, modes), success@1 (trial 1 is a human success), success@k (a human success within trials 1..k; a case with fewer than k trials counts only when one succeeded, which matches stop_on_success), per stage attempted / success conditional on reaching it / share of first failures (human labels), mean attempts per phase overall and per stage, mean automatic retries per trial, agent-vs-human agreement and false "ok" rate per stage (stages labelled n/a are not compared), failure_type and eai_error_type histograms over every failed close, and cost / time per trial. `render_text(report)` prints it as plain-text tables with the coverage on the first line.
- `client.py`: `AgentClient` over the web_ui agent proxy (`<base_url>/state|history|message|stop|reset`), httpx with 5 s connect and 15 s read timeouts. `reset()` retries once after 5 s on a 409 (busy), `message(text, deny_tools)` sends `deny_tools` only when given and raises `BatteryBlockError` (503, with the agent's message) or `AgentBusyError` (409), `history_since(seq)` pages `/history` backwards until it reaches the seq, `stop()` interrupts.
- `labels.py`: `ask(stages_claimed, mode, input_fn)` asks whether the object is in the target now (`y`/`n`, the trial success), then one key per stage in pipeline order, then an optional note. Stage keys: `y` success, `n` fail, `r` not reached, `x` n/a. In arm-only mode navigate defaults to `x` (Enter accepts it). An invalid key is refused and asked again. The first stage labelled `n` is the trial's first failed stage.
- `run.py` / `run.yaml`: the driver, config `RunConfig` (pydantic: `base_url`, `testset`, `mode` arm_only by default, `arm_only_deny_tools`, `k`, `stop_on_success`, `reset_between_trials`, `poll_interval_s`, `trial_timeout_s`, `output_dir` `.logs/eval`, optional `cases` subset such as `[tomato_plush/front_center]`).

### Running an arm-only evaluation

The robot must be charging (no driving) and the objects within arm reach. Every run needs the agent idle and, with `reset_between_trials`, loses its session history (the retry ledger is kept).

```bash
uv run python -m scripts.eval.run scripts/eval/run.yaml
uv run python -m scripts.eval.run scripts/eval/run.yaml --resume 20261020-143000   # continue, skipping recorded trials
```

Per trial (cases are object-major, up to `k` trials each): the setup prompt says what to place (Enter starts, `s` skips the trial, `q` quits and reports); the driver waits until the agent is idle (at most `trial_timeout_s`, then aborts, it never resets a busy agent), resets the session, POSTs the instruction (with `deny_tools` = `arm_only_deny_tools` only in `arm_only` mode) and polls `/history` to the `turn_end` of that instruction. On `trial_timeout_s` it POSTs `/stop` and waits 60 s more. The raw events are saved to `<output_dir>/<run_id>/events/<object>_<placement>_<trial>.json`, the agent-claimed stages and the lookup ids are printed (open the lookup ids in the web UI agent tab to see the images), then you label the trial:

| Prompt | Keys |
|--------|------|
| Is the object in the target now? | `y` / `n` (this is the trial success) |
| navigate, locate, grasp, verify, place | `y` success, `n` fail, `r` not reached, `x` n/a (navigate defaults to `x` in arm-only mode) |
| Note | free text or Enter |

One `TrialRecord` line is appended and flushed to `<output_dir>/<run_id>/trials.jsonl`, `stop_on_success` skips the rest of a case after a labelled success, and the report is printed and written to `report.json` at the end, on `q` and on an abort. A 503 battery block, a reset that is still busy after the retry, a busy agent past `trial_timeout_s` or an unreachable agent aborts the run with the message; recorded trials stay resumable. `instruction_id` in each record joins the trials to the retry ledger records.

Calibration exploration: the retry ledger only observes retries below `retry_success_threshold` through loop re-entries, so for an exploration session set `retry_success_threshold: 0.0` in `ansible/group_vars/client.yml`, deploy the claude_agent, run the evaluation, then restore the real threshold and redeploy. Then run `calibrate_retry_threshold.py` on the collected records.

## gen_nav_footprint.py

Single source of the navigation footprint geometry. Builds the robot's top-view H outline (side frames, crossbar,
steering wheel envelopes, lidar column) from the body model - the mcp_server `body:` block in
`ansible/group_vars/client.yml` when present, else the `body:` in `scripts/gen_nav_footprint.yaml` - and writes the
Nav2 costmap footprint, inflation radius and `cost_scaling_factor`, the collision_monitor StopBox sub-polygons and the
laser_filter self polygon into the marked regions (`# >>> gen_nav_footprint: <name>` ... `# <<< ...`) of
`nodes/nav2_bringup/config/nav2_params.yaml` and `nodes/laser_filter/config/footprint_filter.yaml`. Configured by
`scripts/gen_nav_footprint.yaml` (pydantic-validated: costmap resolution, circle vertices, gap closing, filter and
StopBox margins, falloff, decimals). Uses shapely for the polygon union, offsets and sweeps.
`tests/test_nav_footprint.py` fails while the committed files differ from its output. See `nodes/nav2_bringup/README.md`
(Footprint).

```bash
uv run python scripts/gen_nav_footprint.py
```

## gen_reach_map.py

Builds or checks the committed arm reach map `nodes/mcp_server/mcp_server/data/reach_map.npz` (inverse reachability of
the arm with the robot body, used by the approach planner). Thin wrapper: it runs `python -m mcp_server.reach_map
build|check` in the mcp_server uv environment (ikpy, numpy) with the mcp_server config of
`ansible/group_vars/client.yml`, from the repository root. The robot runs from the git checkout and never builds the
map. `--check` exits 1 when the artifact is missing or stale (its key differs from the key of the current config).
`tests/test_mcp_server_config.py` fails while the artifact does not match `client.yml`.

Regenerate and commit the file after any change to the body, the arm mount (`base_in_base_link`, `arm_base_height_m`),
joint limits or offsets, the tool point, the jaw, the body check margins or the `reach_map` grid in `client.yml`
(about 1.5 min on the M4). See `nodes/mcp_server/README.md` (Reach map).

```bash
uv run python scripts/gen_reach_map.py          # rebuild
uv run python scripts/gen_reach_map.py --check  # exit 1 when stale
```

## markers_generate.py

Writes the QR metadata markers (payload `P1/K:<kind>/N:<name>/S:<mm>`, see `shared/ros2_common/marker_payload.py`) and the ChArUco calibration boards (FULL for the intrinsics checks, ARM with its gripper handle for the head-from-arm and arm offsets fits; `docs/markers.md`, "Calibration boards") as print-ready files. Libraries: segno (QR), reportlab (PDF), trimesh + manifold3d (meshes), OpenCV (board). Code: `scripts/markers/` (`charuco.py` board, PDF, meshes and JSON; `handle.py` the edge handle; `jaw_fit.py` the SO-101 jaw geometry from the URDF meshes; `mesh.py` the print check).

```bash
uv run python -m scripts.markers_generate [config]   # default scripts/markers_generate.yaml
```

The only argument is an optional config path. Exit 1 on an invalid config (pydantic error naming the key, nothing written) or when a marker or board cannot be produced (`FAILED marker ...` / `FAILED board <name>: ...`, its files are not written, the others are still produced; the last line is `wrote <n> files to <dir>; <k> failed`). A board whose printed solid is not watertight or has a face overhanging more than 45 deg (`mesh.print_violations`) fails with `board <name> is not printable: ...` before any of its files is written.

Config keys (`scripts/markers_generate.yaml`):

- `output_dir`: relative paths resolve against the repo root (default `.logs/markers`, gitignored). The calibration board files are the exception that is committed: copy a changed board to `markers/` (all outputs and the JSON) and `models/` (PDF and 3MF, served by web_ui for download).
- `arm_urdf_file`: the SO-101 arm URDF whose gripper meshes seat a board handle (default `nodes/web_ui/urdf/so101_arm.urdf`).
- `defaults`: `ecc` (m/q), `min_version`/`max_version` (2..4), `quiet_zone_modules` (>= 2), `page` (`size` A4/A3/LETTER or `width_mm`+`height_mm`, `orientation`).
- `stl`: `base_thickness_mm`, `relief_mm`, `inlay_mm` (0 = raised modules, > 0 = flush inlay), `margin_mm`, `min_module_mm` (refuse below, default 1.5), `warn_module_mm` (warn below, default 2.0).
- `markers[]`: `kind_code`, `name`, `size_mm`, optional `params`, `outputs` (subset of `pdf`, `stl`, `3mf`), `page` override.
- `boards[]`: the ChArUco boards (`CharucoSpec`): `name` (file stem and board id, lower case, digits, hyphens; unique), `squares_x`, `squares_y` (odd), `square_mm`, `marker_mm`, `dictionary`, `ids_offset` (first marker id; boards sharing a dictionary must use disjoint ids), `margin_mm`, `thickness_mm` (>= 4), `relief_mm`, `inlay_mm` (same meaning as in `stl`; shipped: 4 mm plate, 1.2 mm flush black inlay), `bed_mm` (the plate, with the handle, must fit), `page`, `outputs`, `pattern_face` (`top`: printed back down with the inlays in the top layers; `bottom`: face down on the bed), optional `handle`. Empty: no boards. Shipped: `charuco-6x5-40mm` (FULL, ids 0-14) and `charuco-arm-6x5-28mm` (ARM, ids 15-29, with the handle). `scripts.markers.charuco.board_config(spec)` gives the calibration-side `BoardConfig`.
- `boards[].handle` (`HandleSpec`, `scripts/markers/handle.py`; mm unless noted): the edge grip handle. Grip post: `length_mm` (grip behind the shoulder, 25..40), `neck_mm` (shoulder block from the tab top to the shoulder face), `thickness_tip_mm` / `thickness_end_mm` (nominal grip at the shoulder and at the free end), `channel_tip_mm` / `channel_end_mm` (finger channel widths), `clearance_mm` (channel side walls, 0.2..0.4), `clamp_clearance_mm` (print allowance taken off each clamp face, the bed face and the channel floor, 0..0.3, default 0.15; `thickness_*` are the nominal surfaces), `rail_width_mm`, `rail_height_mm`, `squeeze_rad` (grip = contact - squeeze, < 0.03 rad). Tab: `tab_width_mm` (far end, at least the post footprint), `tab_root_width_mm` (at the plate edge, at least `tab_width_mm`, at most the plate width), `tab_thickness_mm` (thicker than the plate), `post_offset_mm` (plate edge to the post axis). Gusset: `gusset_height_mm` (at most the post length), `gusset_reach_mm` (at most the height: top face within 45 deg; ends on the tab), `gusset_thickness_mm`. Mass and tool: `density_g_per_cm3`, `tool_offset_m` (the mcp_server `arm.tool_offset_m`). A board with a handle needs `pattern_face: top` and the plate plus tab within `bed_mm`.

Outputs per marker, stem `<kind>-<name>` in lower case: `charger-dock-1.pdf`, `charger-dock-1-base.stl`, `charger-dock-1-modules.stl`, `charger-dock-1.3mf`. Each board uses its `name` as the stem with the same four suffixes (`charuco-6x5-40mm`: 6x5 squares of 40 mm with 30 mm markers, plate 250 x 210 mm; `charuco-arm-6x5-28mm`: 28 mm squares with 21 mm markers, plate 178 x 150 mm plus the handle). The PDF is drawn at an exact raster (every square, marker, marker cell and gap a whole number of pixels: 1.4 px/mm FULL, 2.0 px/mm ARM). A board with a handle is printed in one piece (the handle unioned into the base body) and also writes `<name>.json`, whatever its `outputs`:

- `name`, `board` (the `BoardConfig` fields), `plate_mm`, `volume_cm3`, `mass_g` (volume x density), `centre_of_mass_m` (board frame, m).
- `board_mount` (`xyz_init`, `rpy_init`): the board frame in `gripper_link` with the handle seated (m, rad), the CAD seed of head_rgbd_calibration's `fit.board_mount`.
- `handle_in_board`: `gripper_link_xyz_m`, `gripper_link_rpy_rad` (gripper_link in the board frame), `tool_point_m` (the `tool_offset_m` point in the board frame, the `plate_mount.tool_in_board_m` seed), `tool_axis` and `open_axis` (gripper_link TOOL_AXIS and OPEN_AXIS in the board frame: the post axis and the jaw-closing direction).
- `grip`: `contact_rad` (both jaws touch the NOMINAL grip; not a measured contact), `squeeze_rad`, `grip_rad` (= contact - squeeze, the commanded grip), `clearance_clamp_mm` (the print allowance), `modelled_contact_rad` (the contact on the printed, thinner solid pressed on the fixed jaw; `grip_rad` < `modelled_contact_rad` < `contact_rad`).
- `handle_envelope_mm`: `height_mm` (post top above the pattern face) and the `post`, `gusset` and `tab` boxes, `[[x, y, z] min, [x, y, z] max]` mm in the board frame (pattern at z = 0, back at z = 4; the handle lies at z < 0 past the plate's bottom edge): the collision boxes (`handle_boxes_mm`) of the head_rgbd_calibration pose check and the mcp_server board park.
- `stiffness`: `modulus_mpa`, `tab_deflection_mm`, `plate_deflection_mm`, `centre_deflection_mm` (board centre sag under 1 g normal to the plate) and `root_stress_mpa` (tab root under a 10 g jerk).

The JSON values are copied (6 decimals) into head_rgbd_calibration (`config.py` CAD constants, node yaml), mcp_server (`calibration_board` defaults) and `ansible/group_vars/client.yml`; tests pin each copy to the JSON.

Size definition: `size_mm` is the symbol edge, finder outer edge to outer edge, quiet zone excluded. The module edge is `size_mm / (17 + 4 * version)`; the STL path refuses a module under `min_module_mm` (1.5 mm) and warns under `warn_module_mm` (2 mm). A 20 mm marker is therefore too small for STL, 100 mm marker modules are 3.4-4 mm.

Printing:

- PDF: print at 100 % (actual size, no fit to page) on the media in the config; the page carries a 100 mm ruler to check the scale with calipers. Mount on a rigid plate and keep the white quiet zone.
- STL: load both STLs together (base and modules) or the 3MF, which holds the two parts as separate objects. In Bambu Studio assign white filament to the base and black to the modules (3MF extruders 1 and 2); the modules are raised by `relief_mm` (shipped config: `relief_mm: 0`, `inlay_mm: 1.2`, so the black sits flush in a 4 mm white plate).
- ARM board: one piece, back down (pattern face up, the handle tab flat on the bed, the post rising from it), no supports. See `docs/runbooks/printable-markers.md`, "Printing the ARM calibration board".

## markers_render.py

Offscreen renders of a board with a handle: eight views of the print bodies (`+x`, `-x`, `+y`, `-y`, `+z`, `-z`, `iso_front`, `iso_back`; base white, modules black, in print coordinates) and `grip_fit`, the board seated in the posed SO-101 gripper at `grip_rad` (fixed jaw grey, moving jaw orange), seen along the finger width. It reads the written STLs and JSON, never regenerates them. pyvista/VTK render through OSMesa (no display, no root).

```bash
eval "$(bash scripts/markers/setup_render_libs.sh | grep '^export')"   # once per shell
uv sync --group render
uv run --group render python -m scripts.markers_render [config]       # default scripts/markers_render.yaml
```

- `scripts/markers/setup_render_libs.sh`: downloads the pinned Debian bookworm OSMesa/LLVM debs into `.cache/render-libs` (gitignored) with curl and `dpkg-deb -x` (no apt, no root; idempotent), exits 1 naming any package missing from the index or whose version differs from its pin, and prints the two exports (`LD_LIBRARY_PATH`, `VTK_DEFAULT_OPENGL_WINDOW=vtkOSOpenGLRenderWindow`).
- The `render` dependency group of the root `pyproject.toml` holds pyvista (and VTK); it is opt-in. `scripts/markers_render.py` imports pyvista at the top (through `scripts/markers/render.py`), so it runs only with the group; the config model (`RenderConfig`, `load_render_config`) lives in `scripts/markers/config.py` and the view maths in `scripts/markers/views.py` (no pyvista), so their tests run in the default environment.
- Config keys (`scripts/markers_render.yaml`, `RenderConfig`, unknown keys rejected, relative paths against the repo root): `markers_dir` (written board files, default `markers`), `board` (the stem, default `charuco-arm-6x5-28mm`), `arm_urdf_file`, `out_dir` (default `.logs/arm-calib-board/renders`), `size` (width, height px, default 1200 x 900).
- Output: nine PNGs `<board>-<view>.png` in `out_dir`, their paths printed one per line; exit 1 on an invalid config.

## marker_bench.sh

Times marker decode and pose on the client RPi 5 with `python -m marker_detector.bench` in the deployed node venv. Streams `nodes/marker_detector/config/bench.yaml` (or `--config FILE`) over ssh, writes the JSON to `.logs/marker_bench.json` and prints p50/p95/max per scene and stage. Budget: decode + pose p95 <= 100 ms per 640x360 D435 colour frame (design D14).

```bash
./scripts/marker_bench.sh            # default scenes
./scripts/marker_bench.sh --help
```

Env: `MARKER_BENCH_CLIENT_HOST` (default `client.ros2.lan`), `MARKER_BENCH_SSH_USER` (default from `ansible/inventory`), `MARKER_BENCH_REMOTE_VENV` (default `/opt/ros2-nodes/marker_detector/venv`). Needs the node deployed with this bench module.
