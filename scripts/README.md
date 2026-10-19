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
- `stages.py`: `derive(events)` turns one trial's claude_agent events (user_message through turn_end) into the agent-claimed outcome (`ok`, `fail`, `not_reached`) of each stage, only from tool_call / tool_result / phase_completed, never from phase names. navigate: navigate_to_pose / move_relative / drive, ok when the last result is not an error. locate: perception calls (camera images, pixel_to_ground, mark_candidate_points, resolve_candidate, plan_grasp, look_around) before the first grasp_object, ok when a grasp_object or a `planned` plan_grasp follows. grasp: grasp_object calls (attempts = their number), ok when the last outcome is `grasped` or `unverified`. verify: a gripper image or get_arm_state after the last grasp followed by a phase_completed, ok when that phase closed done with no release_object before it outside a place-stage phase. place: the last release_object after the last grasp, ok when it released and its phase closed done. Each phase maps to the stage whose tools it called most (a tie goes to the later stage); its attempts are the highest phase_completed attempt, and every failed close keeps its `failure` record. Runner-made calls (events with `source`) are ignored. It also returns lookup ids, the instruction id and the turn_end payload.
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

Writes the QR metadata markers (payload `P1/K:<kind>/N:<name>/S:<mm>`, see `shared/ros2_common/marker_payload.py`) and the ChArUco calibration board of the head RGB-D calibration as print-ready files. Libraries: segno (QR), reportlab (PDF), trimesh + manifold3d (meshes), OpenCV (board).

```bash
uv run python -m scripts.markers_generate [config]   # default scripts/markers_generate.yaml
```

The only argument is an optional config path. Exit 1 on an invalid config (pydantic error naming the key, nothing written) or when a marker cannot be produced (named in the error, its files are not written, the others are still produced).

Config keys (`scripts/markers_generate.yaml`):

- `output_dir`: relative paths resolve against the repo root (default `.logs/markers`, gitignored; never commit print files).
- `defaults`: `ecc` (m/q), `min_version`/`max_version` (2..4), `quiet_zone_modules` (>= 2), `page` (`size` A4/A3/LETTER or `width_mm`+`height_mm`, `orientation`).
- `stl`: `base_thickness_mm`, `relief_mm`, `inlay_mm` (0 = raised modules, > 0 = flush inlay), `margin_mm`, `min_module_mm` (refuse below, default 1.5), `warn_module_mm` (warn below, default 2.0).
- `markers[]`: `kind_code`, `name`, `size_mm`, optional `params`, `outputs` (subset of `pdf`, `stl`, `3mf`), `page` override.
- `charuco`: `squares_x`, `squares_y` (odd), `square_mm`, `marker_mm`, `dictionary`, `margin_mm`, `thickness_mm` (>= 4), `relief_mm`, `inlay_mm` (same meaning as in `stl`; shipped config: 4 mm plate, 1.2 mm flush black inlay), `bed_mm`, `page`, `outputs`. Omit to skip the board.

Outputs per marker, stem `<kind>-<name>` in lower case: `charger-dock-1.pdf`, `charger-dock-1-base.stl`, `charger-dock-1-modules.stl`, `charger-dock-1.3mf`. The board uses the stem `charuco-6x5-40mm` with the same four suffixes (6x5 squares of 40 mm with 30 mm markers, plate 250 x 210 mm, fits a 256 mm bed).

Size definition: `size_mm` is the symbol edge, finder outer edge to outer edge, quiet zone excluded. The module edge is `size_mm / (17 + 4 * version)`; the STL path refuses a module under `min_module_mm` (1.5 mm) and warns under `warn_module_mm` (2 mm). A 20 mm marker is therefore too small for STL, 100 mm marker modules are 3.4-4 mm.

Printing:

- PDF: print at 100 % (actual size, no fit to page) on the media in the config; the page carries a 100 mm ruler to check the scale with calipers. Mount on a rigid plate and keep the white quiet zone.
- STL: load both STLs together (base and modules) or the 3MF, which holds the two parts as separate objects. In Bambu Studio assign white filament to the base and black to the modules (3MF extruders 1 and 2); the modules are raised by `relief_mm` (shipped config: `relief_mm: 0`, `inlay_mm: 1.2`, so the black sits flush in a 4 mm white plate).

## marker_bench.sh

Times marker decode and pose on the client RPi 5 with `python -m marker_detector.bench` in the deployed node venv. Streams `nodes/marker_detector/config/bench.yaml` (or `--config FILE`) over ssh, writes the JSON to `.logs/marker_bench.json` and prints p50/p95/max per scene and stage. Budget: decode + pose p95 <= 100 ms per 1152x648 frame (design D14).

```bash
./scripts/marker_bench.sh            # default scenes
./scripts/marker_bench.sh --help
```

Env: `MARKER_BENCH_CLIENT_HOST` (default `client.ros2.lan`), `MARKER_BENCH_SSH_USER` (default from `ansible/inventory`), `MARKER_BENCH_REMOTE_VENV` (default `/opt/ros2-nodes/marker_detector/venv`). Needs the node deployed with this bench module.

## head_rgbd_calibrate.py

Calibrates the head Camera Module 3 and the ToF sensor against each other and hands the result to the nodes and the URDF. One script, one config (`scripts/head_rgbd_calibrate.yaml`); the only arguments are the mode, an optional `--config FILE` and, for `capture-validation`, a suffix. `session` in the yaml names the bag every mode uses (default: the newest calibration bag under `capture.local_dir`).

```bash
uv run python -m scripts.head_rgbd_calibrate capture            # prints the local bag path
uv run python -m scripts.head_rgbd_calibrate calibrate
uv run python -m scripts.head_rgbd_calibrate urdf [--check]
uv run python -m scripts.head_rgbd_calibrate capture-validation 1.0m   # 0.5m 1.0m 2.0m 3.0m sweep wall
uv run python -m scripts.head_rgbd_calibrate validate
```

**Board.** Print the ChArUco board of the yaml `board` block (it must equal the `charuco` block of `markers_generate.yaml`): `uv run python -m scripts.markers_generate` (marker-and-calibration-board) writes the print-ready file. Mount it flat and rigid.

**Capture protocol.** `capture` records `capture.duration_s` (default 120 s) of `capture.topics` on the robot over ssh and copies the bag back. It subscribes `/head_camera/mapping/image_raw/compressed`, which engages the head camera focus lock (`lens_position`), so the RGB intrinsics are valid at that one focus; changing `lens_position` invalidates the calibration. Let the ToF warm up for several minutes first (its depth drifts while it heats). Hold the board still for about 2 s in each of 20 or more poses, between 0.3 and 1.5 m, tilted up to about 40 degrees, covering the periphery of both images and moving between poses; the first 2 s of the bag are ignored (focus settling). For validation record one bag per `capture-validation` suffix: a flat wall at the tape-measured 0.5, 1.0, 2.0 and 3.0 m (`0.5m` ... `3.0m`, camera facing the wall square-on), a `wall` bag with the board in front of a farther background while `head_rgbd_cpp` runs, and a `sweep` bag panning the head across the board. They are named `<session>-<suffix>` and record `capture.validation_topics` (the fused `/head_rgbd/*` topics too).

**Modes.**
- `capture` / `capture-validation <suffix>`: described above.
- `calibrate`: picks still RGB and ToF pairs, fits RGB intrinsics, ToF intrinsics (compared with the SDK `/tof/camera_info`) and the ToF-to-RGB extrinsic, checks the board plane depth, then writes the three files and `<report_dir>/<session>-calibration.md` (RMS values, view counts, SDK differences, plane bias, lens position read from the head camera node config named in the yaml `head_camera` block). Any exceeded `limits.*` exits 1 naming it and writes nothing.
- `urdf`: prints the drift between the head URDF and the extrinsic, then rewrites only the `tof_joint` origin of `outputs.urdf`; `urdf --check` only prints the drift.
- `validate`: depth accuracy per `validation.distances_m` from the `<session>-<d>m` bags, edge alignment, registration bias and valid ratio from the `wall` bag, time offset from the `sweep` bag, plus the calibration RMS values and plane bias from the extrinsics file. It prints and writes `<report_dir>/<session>-validation.md` and exits 1 when a target fails. Targets with no input (CPU p95, placeholder messages, motion edge: measured elsewhere; wall or sweep bag or calibration files absent) show SKIP and do not gate; a missing distance bag is an error.

**Outputs and consumers.** `outputs.rgb_calibration` is read by `head_camera_cpp` (`camera_info_file`), `outputs.tof_calibration` and `outputs.extrinsics` by `head_rgbd_cpp` (which takes the transform itself from TF), and `outputs.urdf` (`nodes/web_ui/urdf/head.urdf`, the `tof_joint` origin) feeds the TF tree that `robot_state_publisher` publishes. Redeploy those nodes after calibrating.

## head_rgbd_calibration/ (frames and config)

Head RGB-D calibration tool, configured by `scripts/head_rgbd_calibrate.yaml` (pydantic `CalibrationConfig` in `head_rgbd_calibration/config.py`: `capture`, `board`, `selection`, `limits`, `outputs`, `validation`; unknown keys are rejected and `capture.topics` must include the mapping image topic so the focus lock engages). `board` must equal the `charuco` block of `markers_generate.yaml` (a test pins this).

`frames.py`: `capture_bag` records on the robot over ssh (`timeout -s INT` around `ros2 bag record -s sqlite3`), `scp -r`s the bag to `capture.local_dir/<name>` and removes the remote copy; `read_frames` decodes a bag with `rosbags`; `still_pairs` picks one RGB and ToF pair per still window (after `skip_start_s`, pair dt <= `max_pair_dt_s`); `normalise_amplitude` percentile-stretches ToF amplitude to uint8 (None for an empty frame).

`charuco.py`: `make_board` builds the OpenCV ChArUco board from `board`; `detect` returns sub-pixel corner ids and points (None below 6 corners) for the RGB image or the normalised ToF amplitude.

`intrinsics.py`: `calibrate` (plumb_bob, refuses fewer than `limits.min_views` views), `calibrate_tof` (fits with distortion fixed at zero and free, keeps the zero-D model when its RMS <= `limits.tof_zero_distortion_rms_px`, records `zero_distortion_rms`), `compare_sdk` (fx/fy/cx/cy difference in % against `/tof/camera_info`) and `write_camera_calibration_yaml` (ROS camera_calibration format; raises and writes nothing when RMS exceeds `rgb_max_rms_px` / `tof_max_rms_px`). RGB is calibrated at the 576x324 mapping size. Synthetic scenes for all calibration tests live in `tests/rgbd_synthetic.py`.

`pipeline.py`: the steps behind `head_rgbd_calibrate.py` (session lookup, calibration run, limit checks, output and report writers, validation session, wall and sweep metrics).
