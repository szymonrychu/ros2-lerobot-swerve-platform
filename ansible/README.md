# Ansible

Ansible layout for provisioning Raspberry Pis (Server and Client) and deploying ROS2 nodes as systemd services. Target reader: mid-level Python dev with ROS2; some Ansible experience is enough.

## Layout

- **`inventory`** — Host groups `server`, `client`, and `controller`. Edit with your hostnames or IPs. `all:vars` can set `ansible_user`, `ansible_python_interpreter`. The default inventory uses hostnames `server.ros2.lan` and `client.ros2.lan`; when running Ansible from a dev machine, add to `/etc/hosts`: `192.168.1.33 server.ros2.lan` and `192.168.1.34 client.ros2.lan`.
- **`group_vars/`** — `all.yml`, `server.yml`, `client.yml` for group-specific variables.
- **`site.yml`** — Full site: provision all hosts, then deploy ROS2 nodes on server and client (includes playbooks below). Run `ansible-playbook -i inventory site.yml`.
- **`playbooks/`**
  - **`server.yml`**, **`client.yml`** — Provision: bootstrap Ubuntu 24.04, optional network (netplan) and hostname, and system optimization (debloat + tuning). Run once per host (or when changing base setup). Set `network_address`, `network_gateway`, and optionally `hostname`, `network_nameservers` in group_vars or host_vars to apply static IP and hostname.
  - **`optimize.yml`** — System optimization only: debloat, performance tuning, resilience. Can be run standalone on all hosts.
  - **`controller.yml`** — Full provisioning of the SteamDeck: hostname role + steamdeck_ui role. Run once per SteamDeck.
  - **`docker_cleanup.yml`** — Runs the `docker_cleanup` role on both server and client. Use before migrating to native ROS2 node installs to remove all Docker artifacts. Run: `ansible-playbook -i inventory playbooks/docker_cleanup.yml`.
  - **`deploy_topic_scraper_client_config.yml`** — One-off playbook that pushes an updated `topic_scraper_api` config to the client (including `sensor_msgs/msg/Imu` in `allowed_types` and observation rules for leader-vs-follower comparison and oscillation detection) and restarts the service.
  - **`deploy_steamdeck_ui.yml`** — Update-only: re-clones repo, re-runs `npm ci`, re-deploys config. Use for UI-only updates without re-provisioning.
  - **`deploy_nodes_server.yml`**, **`deploy_nodes_client.yml`** — Deploy all nodes on a target. Each node is listed **explicitly** (no loops) so the order and set of deployments is always clear. Runs repo sync once, deploys every node, then verifies all services are active.
  - There are no per-node playbooks: `deploy_nodes_client.yml` / `deploy_nodes_server.yml` are the only node deploys and every node's steps carry its name as an Ansible tag, so `scripts/deploy-nodes.sh client web_ui mcp_server` is ONE run of the client playbook with `--tags web_ui,mcp_server` (see [Deploy tags](#deploy-tags)).
  - **`tasks/repo_sync.yml`** — Shared include: ensures the Git repo is cloned and up-to-date on the target host.
  - **`tasks/resolve_and_deploy.yml`** — Shared include: looks up a node by name from `ros2_nodes`, resolves all vars, and calls the `ros2_node_deploy` role. Accepts `_deploy_node_name` and optional `_extra_env`.
- **`roles/`**
  - **`common`** — Minimal bootstrap: Python3, git, sudo, basic packages.
  - **`network`** — Netplan: primary interface gets static IP (ethernet or wlan, auto-detected); other interfaces DHCP; IPv6 disabled. Runs when `network_address` and `network_gateway` are set; for primary WiFi set `network_wifi_ssid` (and optionally `network_wifi_password`).
  - **`hostname`** — Set system hostname (hostnamectl, `/etc/hostname`, `127.0.1.1` in `/etc/hosts`). Runs only when `hostname` is set.
  - **`ros2_node_deploy`** — For each node: create the node venv (`python3 -m venv --system-site-packages`) and `uv sync --frozen --no-dev` from the repo (`build_context` path), create config dir, write config file, systemd unit, enable/start; or uninstall (stop, disable, remove unit and config dir). Handlers reload systemd and restart the node when config or unit changes.
  - **`ros2_node_verify`** — Runs as the last step of the play, after the end-of-play restarts: waits for services to settle, checks each present+enabled node’s systemd unit is active, waits again, then re-checks (stability). Used by `deploy_nodes_server.yml` and `deploy_nodes_client.yml`. Variables: `ros2_node_verify_settle_seconds` (default 10), `ros2_node_verify_stable_seconds` (default 5).

  - **`system_optimize`** — Ubuntu 24.04 debloating, performance tuning, and resilience hardening for Raspberry Pi. See [System optimization](#system-optimization) below.
  - **`docker_cleanup`** — Full Docker removal for migration to native ROS2 nodes. Stops all running containers, prunes images/volumes/networks, stops and disables Docker/containerd services, purges Docker CE packages (`docker-ce`, `docker-ce-cli`, `containerd.io`, plugins), removes data dirs (`/var/lib/docker`, `/etc/docker`, `/var/lib/containerd`), removes systemd overrides, APT repo and keyring files, removes the user from the `docker` group, reloads systemd, and autoremoving unused packages. All steps are skipped if Docker is not installed.
  - **`monitoring`** — Empty role scaffold (directories for defaults, handlers, meta, tasks, templates exist but contain no files yet). Reserved for future host monitoring.
  - **`steamdeck_ui`** — Provisions the SteamDeck controller (controller.ros2.lan / 192.168.1.35): installs ROS2 Jazzy base, Node.js 20, Python bridge deps (websockets, pydantic, opencv), Electron system deps, clones the repo, runs `npm ci`, deploys `/etc/steamdeck-ui/config.yaml` (rendered from Jinja2 template), and installs a `.desktop` shortcut.

## System optimization

The `system_optimize` role strips unnecessary packages and services from Ubuntu 24.04, tunes kernel/VM/network parameters for ROS2 workloads, reduces SD card wear, and adds resilience features. Designed for headless Raspberry Pi running from SD cards over WiFi. It runs as part of provisioning (`server.yml`, `client.yml`, `site.yml`) or standalone via `playbooks/optimize.yml`.

### What it does

**Debloat (packages removed and services masked):**
- snapd (and all snap data), cloud-init, ModemManager, open-vm-tools, vgauth, open-iscsi, multipath-tools, udisks2, apport, pollinate, unattended-upgrades, ubuntu-advantage/pro, secureboot-db, avahi-daemon, bluetooth
- APT pinning prevents snapd and cloud-init from being reinstalled
- WiFi (wpa_supplicant) is kept enabled by default

**Performance tuning:**
- CPU governor set to `performance` (configurable)
- `vm.swappiness=0` (no swap), `vm.vfs_cache_pressure=50`, `vm.dirty_ratio=10`
- `vm.min_free_kbytes=65536` to prevent OOM stalls
- ROS2 DDS UDP buffer sizes: `net.core.rmem_max/wmem_max=8MB`
- `fs.inotify` limits raised
- WiFi power management disabled for low-latency ROS2 DDS

**SD card wear reduction:**
- Swap fully disabled (removed from fstab, file deleted)
- Root filesystem mounted with `noatime` and `commit=600` (10-minute ext4 commit)
- Dirty page writeback interval raised to 15 s (`dirty_writeback_centisecs=1500`)
- `/tmp` and `/var/tmp` mounted as tmpfs
- Journal set to volatile (RAM-only, `/var/log/journal` removed)
- APT daily update/upgrade timers disabled
- Core dumps disabled

**Raspberry Pi specific:**
- GPU memory reduced to 16 MB (headless)
- Bluetooth disabled via device tree overlay and service masking
- HDMI output blanked to save power (~30 mA per port)
- I2C clock speed set via `dtparam=i2c_arm_baudrate` when `rpi_i2c_baudrate` is defined (unset on the client: the BNO055 uses a software i2c-gpio bus, see "BNO055 I2C bus")

**Resilience:**
- Hardware watchdog (`bcm2835_wdt`) with systemd `RuntimeWatchdogSec` — auto-reboots on kernel hang
- `kernel.panic=10` and `kernel.panic_on_oops=1` — auto-reboots on panic
- Journald log rotation (max 64 MB in RAM when volatile)

### Configuration

All defaults are in `roles/system_optimize/defaults/main.yml`. Override in `group_vars` or `host_vars`:

| Variable | Default | Description |
|----------|---------|-------------|
| `cpu_governor` | `performance` | CPU frequency governor |
| `swap_enabled` | `false` | Enable swap (disabled to protect SD card) |
| `watchdog_enabled` | `true` | Enable hardware watchdog |
| `watchdog_timeout_s` | `15` | Watchdog timeout (seconds) |
| `journal_max_use` | `64M` | Max journal size (in RAM when volatile) |
| `sdcard_journal_volatile` | `true` | Journal to RAM only (no SD writes) |
| `sdcard_ext4_commit_s` | `600` | ext4 commit interval (seconds) |
| `tmpfs_tmp_enabled` | `true` | Mount /tmp as tmpfs |
| `debloat_disable_wpa_supplicant` | `false` | Keep WiFi enabled |
| `rpi_gpu_mem` | `16` | GPU memory allocation (MB) |
| `rpi_disable_bluetooth` | `true` | Disable Bluetooth |
| `rpi_disable_hdmi` | `true` | Blank HDMI output |
| `rpi_wifi_power_save_off` | `true` | Disable WiFi power saving |
| `rpi_i2c_baudrate` | _(undefined)_ | Hardware I2C clock speed in Hz (unset; the client BNO055 uses i2c-gpio) |
| `rpi_bno055_i2c_gpio_bus` | `8` (client) | Bus number of the software i2c-gpio bus on GPIO2/3 used by the BNO055 (`/dev/i2c-8`) |

### Running standalone

```bash
cd ansible
ansible-playbook -i inventory playbooks/optimize.yml
```

## Node list and config (ros2_nodes)

Node list and per-type defaults live in **`group_vars/client.yml`** and **`group_vars/server.yml`**.

### Repo (group_vars/all.yml)

Deploy playbooks clone the repo on each node for local builds:

- **`ros2_repo_url`** — e.g. `https://github.com/szymonrychu/ros2-lerobot-swerve-platform`
- **`ros2_repo_revision`** — branch, tag, or commit (default `main`)
- **`ros2_repo_dest`** — path on the node (default `/opt/ros2-lerobot-swerve-platform`)
- **`ros2_repo_root`** — (default `{{ playbook_dir }}/../..`) Path to repo root on the controller.

### ros2_node_type_defaults

Maps each **node_type** to **build_context** (path relative to repo root), and optional config path and env.

```yaml
ros2_node_type_defaults:
  ros2_master:
    build_context: nodes/ros2_master
  feetech_servos:
    build_context: nodes/bridges/feetech_servos
    config_path: /etc/ros2/feetech_servos
    env:
      - FEETECH_SERVOS_CONFIG=/etc/ros2/feetech_servos/config.yaml
```

Optional `environment_file` adds `EnvironmentFile=<path>` to the native unit (via `node_environment_file` in the
`ros2_node_deploy` role) for secrets that must not appear in the unit's `Environment=` lines. `mcp_server` uses it for
its bearer token (`/etc/ros2/mcp_server/token`, see "MCP server token" below).

### ros2_nodes

List of nodes to deploy. Each entry:

| Key         | Required | Default | Description |
|------------|----------|---------|-------------|
| `name`     | yes      | —       | Logical node name; systemd unit is `ros2-{{ name }}.service`. |
| `node_type`| yes      | —       | Key in `ros2_node_type_defaults` (image, build_context, config_path, env). |
| `present`  | no       | `true`  | If `false`, the node is uninstalled (unit and config dir removed). |
| `enabled`  | no       | `true`  | If `true`, service is enabled and started; if `false`, stopped and disabled. |
| `config`   | no       | —       | Config file content (string) in the node’s expected format; used when type has `config_path`. |
| `env`      | no       | `[]`    | Extra env vars (list of `KEY=VAL`), appended to type’s `env`. |
| `extra_args` | no     | `''`    | Extra arguments passed to the service. |

Example:

```yaml
ros2_nodes:
  - name: lerobot_follower
    node_type: feetech_servos
    present: true
    enabled: true
    config: |
      namespace: follower
      joint_names:
        - name: joint_1
          id: 1
        - name: joint_2
          id: 2
        - name: joint_3
          id: 3
        - name: joint_4
          id: 4
        - name: joint_5
          id: 5
        - name: joint_6
          id: 6
  - name: gripper_uvc_camera
    node_type: uvc_camera
    enabled: true
    env:
      - UVC_DEVICE=/dev/video0
      - UVC_TOPIC=/camera_0/image_raw
```

Battery voltage wiring (client): `lerobot_follower` sets `battery_topic: /battery_state`, `battery_interval_s: 1.0`, `battery_cells: 3` and publishes the pack voltage read from the servos; the `web_ui` config has a `battery:` block (same topic and cells, `cutoff_cell_v: 2.8`, `resume_cell_v: 2.9`, `stale_s: 5.0`) that shows the voltage and rejects web UI commands below 3 x 2.8 = 8.4 V. The server's `lerobot_leader` sets `battery_interval_s: 0` (its bus is not the robot pack) and the server runs no web_ui, so it gets no `battery:` block. `tests/test_battery_config.py` pins this wiring.

When you add, remove, or reconfigure ROS2 nodes, update these vars and re-run the deploy playbook.

### Joint command topic flow (client)

Leader joint states are relayed and filtered before reaching the follower feetech bridge:

- **Server:** `lerobot_leader` (feetech) publishes `/leader/joint_states`.
- **Client:** `master2master` subscribes to `/leader/joint_states` and republishes to **`/filter/input_joint_updates`**.
- **Client:** `test_joint_api` can also publish to `/filter/input_joint_updates` (same path as master2master; use for testing, gripper-only).
- **Client:** `filter_node` subscribes to `/filter/input_joint_updates`, runs the configured algorithm (e.g. Kalman), and publishes to **`/follower/joint_commands`**.
- **Client:** `lerobot_follower` (feetech) subscribes to `/follower/joint_commands` and drives the servos.

So both master2master and the test API feed the same filter → feetech chain.

### Topic scraper debug API (client + server)

The **topic_scraper_api** node runs on both hosts and exposes:

- `GET /topics` for topic metadata (topic, endpoint, type, sample status)
- `GET /topics/<topic-path>` for latest sample payload plus timing fields (`header_stamp_ns`, `received_at_ns`)

Default port is `18100` and it runs with host networking. Example:

```bash
curl http://client.ros2.lan:18100/topics
curl http://server.ros2.lan:18100/topics/leader/joint_states
```

Use `scripts/topic_scraper_collect.py` to poll both hosts and emit merged NDJSON for dynamic comparisons.

### BNO055 I2C bus (client)

The Pi 5 hardware I2C controller (RP1 DesignWare) does not reliably honour the BNO055's clock stretching: reads come
back corrupted, and in NDOF mode the chip locked up with `Errno 110` (2026-10-09). `deploy_nodes_client.yml` therefore
includes `tasks/bno055_i2c_boot_config.yml` (tags `boot`, `bno055_imu`), which sets `dtparam=i2c_arm=off`, removes the
old 10 kHz `dtparam=i2c_arm_baudrate` workaround and adds
`dtoverlay=i2c-gpio,bus=8,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=2` (bit-banged, ~100 kHz, waits for
stretched SCL). The bno055_imu config uses `i2c_bus: 8` (kept equal to `rpi_bno055_i2c_gpio_bus`). The client reboots only when a
line changed.

### IMU (client)

The **bno055_imu** node runs on the client and publishes `sensor_msgs/Imu` on `/imu/data` (configurable) with orientation, angular velocity, linear acceleration, and full covariance matrices for use with the Navigation stack (Nav2). It reads a BNO055 over I2C; the node reads `/dev/i2c-1` directly. Config: topic, frame_id, publish_hz, i2c_bus, i2c_address (default 0x28), and covariance values (see `nodes/bridges/bno055_imu/README.md`).

### Raspberry Pi GPIO/I2C tooling (all users)

The `common` role installs:

- `gpiod` and `i2c-tools`
- compatibility commands: `pinctrl` and `raspi-gpio` under `/usr/local/bin`
- udev access rules for all users on Raspberry Pi hosts:
  - `/dev/gpiochip*` -> mode `0666`
  - `/dev/i2c-*` -> mode `0666`

This allows non-root users to run GPIO and I2C checks directly.

#### BNO055 `RST` / `INT` pin usage

Assuming:

- `RST` on GPIO17
- `INT` on GPIO4
- I2C on GPIO2/3 (software i2c-gpio bus, `/dev/i2c-8`)

Use this reset-and-probe sequence:

```bash
# Hold reset low (active-low reset), then release high
pinctrl set 17 op dl
sleep 0.05
pinctrl set 17 op dh

# Keep INT as input and read its current level
pinctrl set 4 ip
pinctrl get 4

# Probe I2C (as normal user)
i2cdetect -y -r 1
```

Equivalent with `raspi-gpio`:

```bash
raspi-gpio set 17 op dl
sleep 0.05
raspi-gpio set 17 op dh
raspi-gpio set 4 ip
raspi-gpio get 4
i2cdetect -y -r 1
```

## Network and hostname (provision)

The **network** role sets a static IP on the primary interface (ethernet or WiFi, auto-detected), sets all other interfaces to DHCP, and disables IPv6 in netplan.

In **`group_vars/server.yml`** or **`group_vars/client.yml`** (or host_vars), set:

- **`network_address`** — Static IP in CIDR (e.g. `192.168.1.10/24`). Required.
- **`network_gateway`** — Default gateway (e.g. `192.168.1.1`). Required.
- **`network_nameservers`** — Optional list. By default derived from `primary_dns_server` (e.g. `192.168.1.1`) and `secondary_dns_server` (e.g. `1.1.1.1`).
- **`network_interface`** — Optional. Default: primary IPv4 interface (ethernet or wlan).
- When the primary interface is **WiFi** (e.g. `wlan0`): **`network_wifi_ssid`** (required), **`network_wifi_password`** (optional).
- **`hostname`** — Short hostname (e.g. `server-rpi4`). Optional; when set, the hostname role runs.
- **`ros2_extend_etc_hosts`** — (default `false`) When true, hostname role adds `ros2_hosts_entries` (server.ros2.lan, client.ros2.lan) to `/etc/hosts`. Default false: rely on primary DNS.

The network role writes a netplan file under `/etc/netplan/` and runs `netplan apply`. The **hostname** role runs `hostnamectl set-hostname` and updates `/etc/hostname` and `/etc/hosts`.

## ROS2 network setup (DDS discovery)

- **All ROS2 nodes:** the unit template emits `ros2_dds_env` (`group_vars/all.yml`) first: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (simple discovery restricted to the host; FastDDS uses shared memory for same-host data). Node `env` entries come after it and override.
- **Cross-host:** only the client `master2master` adds `ROS_STATIC_PEERS={{ ros2_server_hostname }}`, so it is the single process exchanging DDS traffic with the server. Server nodes list no static peers: when they listed the client, every client node meshed with the server over Wi-Fi.
- **Steam Deck UI:** SUBNET discovery with the client as static peer (`steamdeck_ros2_static_peers`).
- **Shells:** `/etc/profile.d/ros2_dds.sh` (from `playbooks/tasks/dds_host_setup.yml`) sets the same localhost discovery for SSH sessions, so the `ros2` CLI and `scripts/*_diag.sh` see the graph. Non-login SSH commands should `source /etc/profile.d/ros2_dds.sh`.
- **Shared memory:** nodes run as a regular user, and logind's default `RemoveIPC=yes` deletes that user's POSIX shared memory (the FastDDS SHM segments) on every SSH logout. Data then silently stops for processes started afterwards. `dds_host_setup.yml` installs `/etc/systemd/logind.conf.d/ros2-keep-ipc.conf` (`RemoveIPC=no`) and restarts running `ros2-*` services once when it is first applied.
- **History:** a FastDDS discovery server per host was tried (2026-10-03) and removed: launch_ros' one-shot lifecycle and component-loading service calls (slam_toolbox configure, Nav2 composable nodes) hung intermittently through it on the busy client. The `fastdds_discovery_server` node stays `present: false` so deploys uninstall it.

## Boot network wait

`--all` deploys install `/etc/systemd/system/systemd-networkd-wait-online.service.d/any-interface.conf` (`playbooks/tasks/network_wait_online.yml`): boot waits for any one interface for at most 30 s. The netplan template also marks secondary Ethernet ports `optional: true` (applied when the network role runs). Without this, an unplugged `eth0` held every ROS node for the 2-minute wait-online timeout.

## Deploy load management

A deploy no longer stops the whole stack up front. The role's `stop_for_build.yml` stops all running `ros2-*` services, once per play, only right before a heavy step that will actually run: the uv sync of a node whose dependency hash changed, the web_ui `npm ci` / `npm run build`, or a colcon source build whose stamp is missing. So installs and builds still run on an otherwise idle Pi (the client overheated and froze on 2026-10-03), while a no-change deploy stops nothing. Nothing is restarted while nodes are deployed either: a changed config, unit, launcher, source, dependency set or build output queues a restart by appending the node to the host file `/var/lib/ros2-deploy/pending-restart` (`ros2_restart_queue`, one name per line), and `playbooks/tasks/start_ros_nodes.yml` carries out all of them once, at the end, in `ros2_nodes` order (dependencies first), one node at a time with `ros2_node_start_interval_s` (2 s, `group_vars/all.yml`) between them. It fails the run if a restart fails and clears the queue only after all succeeded, so a failed run keeps its queued restarts for the next one. The same step starts enabled nodes in the run's scope that are not running (stopped for a build); with a node filter (`--tags web_ui`) the scope is the selected nodes, queued nodes are restarted whatever the scope, and verify checks the scope plus the restarted nodes. The node tasks of both deploy playbooks run inside a `block` whose `rescue` runs the same start step (errors ignored) and then fails the run with the original error, so a failure after a build stopped the nodes never leaves the robot down.

**Failure safety.** (1) The restart intent is written to the host queue by a task right after each change is applied (uv sync, npm build and static copy, config, launcher, unit, colcon build, source stamp), not only at the late handler flush; the source stamp is written last, after config, launcher and unit succeeded. A failure at any point therefore still leaves the restart queued, and the start step runs `systemctl daemon-reload` first so a changed unit takes effect. (2) Before a heavy step stops the running units, they are appended to `/var/lib/ros2-deploy/stopped-for-build`; the start step starts every listed unit whatever the run's scope (also with `--tags <node>`) and clears the file after success. (3) `playbooks/tasks/deploy_guard.yml` installs `ros2-deploy-recover.timer` (every 2 minutes): when the stopped list is older than `ros2_recover_after_min` (15) and no deploy lock younger than `ros2_deploy_lock_max_age_min` (240) exists, it starts the listed units and clears the file. This covers a lost controller or SSH session, where the rescue path cannot run on an unreachable host. The lock `/var/lib/ros2-deploy/deploy.lock` is taken as the last pre-task and released after verify or in the rescue; a lock left by a dead controller stops blocking after its max age. The pre-tasks (repo sync, apt, firmware config) run before any node is stopped, so a failure there needs no rescue. The rescue message carries the failed task, its message, rc and stderr.

**What counts as a changed source.** Every node gets a source stamp (`/var/lib/ros2-deploy/stamps/<node>.src`) holding the hash of the git tree hashes of its source paths on the synced checkout: `node_src_dir` for Python nodes, `src_paths` for launch/config-only node types (rplidar_a1, overview_camera, laser_filter, rf2o_laser_odometry, robot_localization_ekf, slam_toolbox, nav2_bringup, ros2_master), plus the type's `src_extra_paths` (mcp_server also reads `nodes/web_ui/urdf`) and `shared/` when the type sets `src_shared: true` (mcp_server, web_ui; a root test checks every pyproject with a `../../shared` dependency is declared). A different hash queues the restart. Running nodes nothing changed for are left alone and cost no sleep. `ros2_node_verify` runs last, after those restarts, and checks all units with one `systemctl is-active` per round.

## web_ui frontend build

The web_ui frontend is built on the client during deploy (`npm ci`, `npm run build`), but only when needed: `npm ci` when the hash of `package.json` + `package-lock.json` differs from `frontend/.npm-ci-stamp` (or `node_modules` is gone), `npm run build` when the git tree hash of `nodes/web_ui/frontend` differs from `frontend/.build-stamp`, after an `npm ci`, or when `dist/index.html` / `web_ui/static/index.html` is missing. The stamps are written after the step succeeded; the node restarts only when a build ran. Both steps run in a transient systemd scope (`systemd-run --scope -p CPUQuota={{ ros2_build_cpu_quota }} -p IOWeight=10`, 400% = all four cores since the Pi got active cooling; was 100% while it overheated) under `nice -n 19 ionice -c 3`, which keeps the running ROS nodes first in line. On 2026-10-03 a full-priority build next to Nav2, SLAM and the bridges on the then-uncooled Pi 5 overheated it until it stopped responding: lower `ros2_build_cpu_quota` again if temperatures climb.

### Source-built ROS packages (colcon)

Packages missing from the Jazzy apt distribution are built from a pinned git commit. A node type with a `colcon_source` block gets `roles/ros2_node_deploy/tasks/colcon_source_build.yml`, which takes ONE source dict (`rf2o_laser_odometry`) or a LIST of sources (`overview_camera`) and builds them in list order, so dependencies come first (libcamera before camera_ros). Each source is `{repo, commit, package, workspace}` plus optional `patches`, `cmake_args` (default `-DCMAKE_BUILD_TYPE=Release`) and `meson_args` (builds that package with colcon-meson, `--meson-args`). `commit` is a SHA or a tag. `colcon_source_package.yml` builds one source: the commit is cloned to `<workspace>/src/<package>` and built with `colcon build --merge-install --parallel-workers 1 --packages-select <package>` (`MAKEFLAGS=-j{{ ros2_build_jobs }}`; meson's ninja uses all cores the cgroup allows) inside the same `systemd-run --scope -p CPUQuota={{ ros2_build_cpu_quota }} -p IOWeight=10 nice -n 19 ionice -c 3` wrapper as the web_ui build, after sourcing `/opt/ros/jazzy/setup.bash` and the workspace's own `install/setup.bash` (so camera_ros finds the fresh libcamera). Optional `patches` (a list of unified diffs in the repo, applied with `ansible.builtin.patch` after the checkout; the git task uses `force` so the clone is reset before they are re-applied) can fix upstream bugs. A stamp file `<workspace>/.built-<package>-<hash>` makes it idempotent; the hash covers the pinned commit, the content of every patch, the build args and (for the second and later sources) the stamps of the sources before it, so the build only reruns, and the node restarts, when one of them changes, or when `<workspace>/install/setup.bash` is missing (the stamp is then removed). For rf2o (single source, no build args) the hash is the original `sha1(commit + patch checksums)`, so existing stamps stay valid. The launcher script sources `<workspace>/install/setup.bash` (of the first source) after `/opt/ros/jazzy/setup.bash`. Used by `rf2o_laser_odometry` and `overview_camera` (workspace `/opt/ros2-ws`); the first deploy compiles on the Pi, so expect a slow, low-priority build (libcamera needs network once for its libpisp meson wrap). Build dependencies come from the node type's `apt_packages`.

**Camera boot overlay.** `playbooks/tasks/overview_camera_boot_config.yml` (included from `pre_tasks` of `deploy_nodes_client.yml`, tags `boot` and `overview_camera`) ensures `camera_auto_detect=0`, `dtoverlay=imx708,cam0` and `dtoverlay=arducam-pivariety,cam1` (Arducam ToF camera on the second CSI port; the Pivariety driver and overlay ship with the Ubuntu raspi kernel) in `/boot/firmware/config.txt` (same file and `lineinfile` pattern as the UART overlay), removes stale `dtoverlay=imx219...` lines left by the retired stereo pair, and reboots the client only when a line was added, changed or removed. The `overview_camera` node type installs no apt `ros-jazzy-camera-ros` / `ros-jazzy-libcamera`: they would shadow the source-built fork. `realsense_d435i` is `present: false` (uninstalled on deploy); no `camera_link` frame is published by `static_tf_publisher` until the overview camera mount pose is measured. See `nodes/overview_camera/README.md`.

## ROS package sync

`--all` deploys (`deploy_nodes_client.yml`, `deploy_nodes_server.yml`) run `playbooks/tasks/ros_packages_sync.yml` right after the repo sync. It refreshes the apt index (at most hourly), upgrades every installed `ros-jazzy-*` package that has an update, and restarts the running `ros2-*` services when anything was upgraded. Mixing packages from different packages.ros.org syncs breaks ABI: on 2026-10-03 a freshly installed `laser_filters` failed with an undefined `diagnostic_updater` symbol.

## Node Resource Limits

Systemd `CPUQuota` and `MemoryMax` are set per node in `group_vars/client.yml` and `group_vars/server.yml`.

### Server

| Node | CPUQuota | MemoryMax |
|---|---|---|
| ros2-master | 20% | 128M |
| lerobot_leader | 50% | 128M |
| topic_scraper_api | 25% | 128M |
| gps_rtk_base | 25% | 64M |

### Client

| Node | CPUQuota | MemoryMax |
|---|---|---|
| ros2-master | 20% | 128M |
| master2master | 25% | 128M |
| filter_node | 25% | 128M |
| test_joint_api | 15% | 64M |
| topic_scraper_api | 25% | 128M |
| bno055_imu | 15% | 64M |
| gps_rtk_rover | 25% | 64M |
| haptic_controller | 25% | 64M |
| gripper_uvc_camera | 30% | 256M |
| rplidar_a1 | 30% | 128M |
| realsense_d435i | 50% | 512M (retired, `present: false`) |
| overview_camera | 100% (Nice=5) | 256M |
| lerobot_follower | 50% | 128M |
| swerve_controller | 30% | 128M |
| static_tf_publisher | 10% | 64M |
| rf2o_laser_odometry | 25% | 128M |
| rf2o_odom_relay | 10% | 64M |
| robot_localization_ekf | 25% | 128M |
| slam_toolbox | 75% | 512M |
| nav2_bringup | 75% | 512M |
| web_ui | 30% | 256M |
| mcp_server | 25% | 256M |
| poi_store | 10% | 128M |
| claude_agent | 50% (Nice=10) | 1G |

### POI store directory

`playbooks/tasks/poi_store_dir.yml` creates `/var/lib/ros2/poi` (owner `ansible_user`, mode `0755`) before `poi_store` is
deployed, in `deploy_nodes_client.yml` (tags `setup`, `poi_store`). The node keeps `poi.json` there.

### SLAM maps directory

`playbooks/tasks/slam_maps_dir.yml` creates `/var/lib/ros2/maps` (owner `ansible_user`, mode `0755`) before
`slam_toolbox` is deployed, in `deploy_nodes_client.yml` (tags `setup`, `slam_toolbox`).
slam_toolbox saves and reloads its posegraph there (`slam_map.posegraph` / `slam_map.data`).

### bno055_imu calibration directory

`playbooks/tasks/bno055_state_dir.yml` creates `/var/lib/ros2/bno055_imu` (owner `ansible_user`, mode `0755`) before
`bno055_imu` is deployed, in `deploy_nodes_client.yml` (tags `setup`, `bno055_imu`). The node saves and restores the
BNO055 calibration offsets there (`calibration.json`); delete the file to reset them.

### web_ui tile cache directory

`playbooks/tasks/web_ui_tile_cache_dir.yml` creates `/var/cache/web_ui/tiles` (and its parent, owner `ansible_user`,
mode `0755`) before `web_ui` is deployed, in `deploy_nodes_client.yml` (tags `setup`, `web_ui`). The
map tab's `/api/tiles` proxy caches map tiles there (`tile_cache_dir` default) and serves them when offline.
It also deletes the stale numeric `z` directories directly under `/var/cache/web_ui/tiles` (the pre-key cache layout,
full of "API KEY REQUIRED" placeholder tiles; tiles now live in a fingerprint subdirectory) and is idempotent.

### web_ui map tile API key

`playbooks/tasks/web_ui_tile_key.yml` runs before `web_ui` is deployed (tags `setup`, `web_ui`). CARTO basemaps need an
API key (https://carto.com/basemaps/apikey). It is read on the controller with `lookup('env', 'CARTO_API_KEY')` and
written to `/etc/ros2/web_ui/env` as `WEB_UI_TILE_API_KEY=<key>` (mode `0600`, owner `ansible_user`, `no_log: true`);
the web_ui unit reads it through `EnvironmentFile=` and the map tab's `tile_api_key_env` names the variable. With
`CARTO_API_KEY` unset an existing file is kept; with neither the deploy fails with a hint. A changed file queues a
web_ui restart. Deploy with:

```bash
export CARTO_API_KEY=...   # from https://carto.com/basemaps/apikey
./scripts/deploy-nodes.sh client web_ui
```

### MCP server token and arm home directory

`playbooks/tasks/mcp_server_setup.yml` runs before `mcp_server` is deployed (in `deploy_nodes_client.yml` and in
tags `setup`, `mcp_server`). It creates `/etc/ros2/mcp_server/token` once, containing
`MCP_SERVER_TOKEN=<48 random letters/digits>` (mode `0600`, owner `ansible_user`, `force: false` so redeploys keep the
token, `no_log: true`), and `/var/lib/ros2/arm` (owner `ansible_user`) for the arm home pose (`home.yaml`),
`/var/lib/ros2/camera_calibration` (owner `ansible_user`) for the camera calibration samples of the mcp_server camera
tools, and `/var/lib/ros2/objects` (owner `ansible_user`) for the legacy object memory file (`objects.json`, imported once into poi_store as object POIs and renamed `objects.json.migrated`). The unit
reads the token through `EnvironmentFile=`; the token never enters git. Fetch it on the dev machine with
`eval "$(./scripts/robot_mcp_token.sh)"` (see `nodes/mcp_server/README.md`).

### claude_agent user and OAuth token

`playbooks/tasks/claude_agent_setup.yml` runs before `claude_agent` is deployed (in `deploy_nodes_client.yml` and in
tags `setup`, `claude_agent`, after `mcp_server_setup.yml`). It creates the system user `claude_agent` (no login
shell, home `/var/lib/claude_agent`), the directories `/etc/ros2/claude_agent`, `/var/lib/claude_agent` (HOME, session log) and `/var/lib/claude_agent/workspace` (the agent's persistent volume: its `NOTES.md` and notes; `0750`, owner `claude_agent`, created if missing and never deleted or emptied by a deploy), and writes
`/etc/ros2/claude_agent/env` containing `CLAUDE_CODE_OAUTH_TOKEN=<token>` (mode `0600`, owner `claude_agent`,
`no_log: true`). The token is read on the controller with `lookup('env', 'CLAUDE_CODE_OAUTH_TOKEN')`, so deploy with:

```bash
export CLAUDE_CODE_OAUTH_TOKEN=...   # from `claude setup-token`
./scripts/deploy-nodes.sh client claude_agent
```

With the variable unset an existing env file on the robot is kept; with neither the deploy fails with that hint. The
unit reads the file through `EnvironmentFile=` (never `Environment=`). The service runs as `claude_agent` through the
optional per-node role variables `node_user` (`user:` in `ros2_node_type_defaults`, default `ansible_user`),
`node_supplementary_groups` (`supplementary_groups:`) and `node_nice` (`nice:`); other nodes are unchanged. Because the MCP
bearer token is in the CLI child's argv, the claude_agent unit is also hardened through the optional variables
`node_protect_proc`, `node_proc_subset`, `node_no_new_privileges`, `node_private_tmp` (`protect_proc: invisible`,
`proc_subset: pid`, `no_new_privileges: true`, `private_tmp: true`; defaults off, so other units are unchanged).
`mcp_server_setup.yml` now creates the system group `mcp-token` and makes the MCP token group-readable
(`0640`, owner `ansible_user`, group `mcp-token`); `claude_agent` joins that group through `SupplementaryGroups=`.
The Claude Code CLI is the native binary bundled in the pinned `claude-agent-sdk` wheel (installed by uv), so no
npm install is needed; `DISABLE_AUTOUPDATER=1` is set in the unit environment.

## Connection tuning

The `ansible.cfg` `[ssh_connection]` section tunes SSH for the Raspberry Pis on flaky WiFi:

- **`ControlMaster=auto`** / **`ControlPersist=120s`** - one SSH connection is reused by all tasks of a run (ansible adds the `ControlPath` itself). Before 2026-10 multiplexing was off because of stale sockets after network drops; the keepalives below make a dead master exit, and `retries` re-connects. If a stale socket ever blocks a run, `rm ~/.ansible/cp/*`.
- **`pipelining = True`** - modules run over the open SSH session instead of being copied to a temp file first (needs no `requiretty` in sudoers, the Ubuntu 24.04 default), saving about two round trips per task.
- **`ConnectTimeout=30`** - fails fast on unreachable hosts (30 s).
- **`ServerAliveInterval=10`** / **`ServerAliveCountMax=6`** - sends a keepalive every 10 s; drops the connection after 60 s of silence.
- **`timeout=120`** - per-task SSH timeout (2 min).
- **`retries=5`** - retries failed SSH connections up to 5 times.

## Node venvs (uv)

How a node's Python environment is built on the Pi (`ros2_base` and `ros2_node_deploy` roles):

- **uv install** (`ros2_base`): the pinned `uv` (`ros2_uv_version`, currently 0.11.29) is installed system-wide with `pipx install --force "uv==<version>"` into `/usr/local/bin`. A matching `uv --version` makes the task a no-op; any other version is replaced.
- **Venv recipe** (`ros2_node_deploy`, only for nodes with a `build_context` source dir): `python3 -m venv --system-site-packages /opt/ros2-nodes/<node>/venv` using the system Python 3.12 (so `rclpy` and apt `python3-*` libraries stay visible), then `uv sync --frozen --no-dev` run in `<repo>/<build_context>` with `UV_PROJECT_ENVIRONMENT=<venv>`, `UV_PYTHON=<venv>/bin/python3` and `UV_PYTHON_DOWNLOADS=never`. `--frozen` installs exactly the node's committed `uv.lock`; `--no-dev` skips the dev group.
- **Stamp** `<venv>/.uv-deps`: sha256 of the node's `pyproject.toml` + `uv.lock` (plus `shared/pyproject.toml` for nodes with `src_shared`). The sync runs only when it differs from the stamp or the venv is new; the stamp is written only after `uv sync` succeeded. A stamp from the previous installer has another file name, so the first uv deploy always syncs once.
- **Changing dependencies**: edit the node's `pyproject.toml`, run `uv lock` in the node directory, commit both files, redeploy the node.

## Deploy tags

Every task of the deploy playbooks, the `ros2_node_deploy` / `ros2_node_verify` roles and `playbooks/tasks/*.yml` carries at least one of these phase tags (a root test enforces it). Each node's deploy step and node-specific setup carries the node name as well (`web_ui`, `mcp_server`, `overview_camera`, ...).

| Tag | What it covers |
|-----|----------------|
| `sync` | Repo clone/update on the target (also `always`) |
| `apt` | ROS package sync and ONE batched install of the apt packages of all nodes in the run |
| `python` | uv venv, dependency sync (only when the hash changed) and the deployed-source stamp |
| `build` | web_ui npm steps and colcon source builds (stamp-gated), and the stop-before-build |
| `config` | Node config files, launcher scripts, systemd units, enable/disable, uninstall, DDS host setup |
| `boot` | Firmware overlays (`/boot/firmware/config.txt`), reboot, boot network wait |
| `setup` | Node prerequisites: directories, tokens, users (also tagged with their node) |
| `restart` | The final gradual restart/start of changed and stopped nodes (also `always`) |
| `verify` | The final active-and-stable check of all node units (also `always`) |
| `always` | Runs under every `--tags` filter: run selection, repo sync, restart, verify, handler flush, asserts |

`scripts/deploy-nodes.sh <target> node1 node2` runs the target's playbook ONCE with `--tags node1,node2` (so one git sync, no parallel runs); `--all` has no node filter. `--tags` / `--skip-tags` pass through with `--all`:

```bash
./scripts/deploy-nodes.sh client web_ui mcp_server               # only those nodes, all their phases
./scripts/deploy-nodes.sh client --all --tags config,restart     # config/unit/launcher changes only, then restart
./scripts/deploy-nodes.sh client --all --tags python             # uv/source change detection for every node
./scripts/deploy-nodes.sh client --all --skip-tags build,verify  # everything but builds and the final check
./scripts/deploy-nodes.sh client --all --tags boot               # firmware overlays and boot wait only
```

`--tags build` and `--tags config` do not install apt packages (only a full run, `--tags apt` or a node tag do): use the node tag for a node that is new or whose packages changed. With a node filter the apt steps take part (the batch installs only the selected nodes' packages), firmware (`boot`) steps only for nodes that need them (`gps_rtk_rover`, `overview_camera`), and the always-tagged restart and verify steps run last. Note that `always` tasks also run under `--tags config`; skip them with `--skip-tags restart,verify`.

## Deploy performance

Goal: a no-change `./scripts/deploy-nodes.sh client --all` (was over 20 minutes) should only look, not act. `ansible.cfg` enables `ansible.posix.profile_tasks` and `ansible.posix.timer` (`ansible-galaxy collection install -r requirements.yml` if the collection is missing), so every deploy log ends with the slowest tasks and the total time. What changed and the expected saving on a no-change deploy of the client:

| Change | Expected saving |
|--------|-----------------|
| web_ui `npm ci` + `npm run build` only when their stamp changed (was every deploy, both ~5-10 min on the Pi) | the bulk of the 20 minutes |
| uv sync per node only when `pyproject.toml` / `uv.lock` (and `shared/pyproject.toml` for nodes depending on it) changed; `shared/` is an editable path source, so source changes need no sync | ~10-20 s per node, 27 nodes |
| No stop-all and no per-node restart: only changed nodes restart, once, at the end; nodes are stopped only before a heavy step. Restart cause detection is per node: git tree hash of the node's source dir (plus `shared/` for dependents) vs `/opt/ros2-nodes/<node>/.deployed-src`, config, launcher, unit, deps, builds | the restart + 2 s start interval of every node (about 2-4 minutes), and no downtime for unchanged nodes |
| Colcon sources: key, stamp and `install/setup.bash` are checked first; clone, patch and build are skipped when the stamp exists (was a git fetch + patch reset per source on every deploy) | a few network round trips per source |
| One batched apt task for all nodes instead of one per node | about 25 apt calls |
| SSH `ControlPersist` + pipelining; facts limited to `gather_subset=min` and cached 24 h | 2-3x faster per-task overhead over WiFi (roughly 300 remote tasks) |
| Node verify: one `systemctl is-active` for all units per round (was one SSH task per node, two rounds) | about 50 SSH tasks |
| Removed the servo "stop before deploy" pre-tasks: a changed servo node is restarted by the queue, an unchanged one keeps running | an unneeded servo restart |

First deploy after this change: all stamps are missing, so every node's uv sync, `.deployed-src` and the web_ui build run once and every node restarts once; later deploys are the quick path. The probe and stamp logic is covered by `tests/test_ansible_deploy_speed.py` (the probe scripts are run against temp git repos); the effect on the real Pis is read off the `profile_tasks` recap of the next deploy.

## Linting and testing

- **ansible-lint**: Run from the `ansible/` directory so `roles_path` resolves: `cd ansible && ansible-lint .`. From repo root: `uv run poe lint-ansible`.
- **test-ansible**: Lint plus playbook syntax-check for all playbooks: `uv run poe test-ansible` (runs `ansible-lint .` and `ansible-playbook -i inventory playbooks/<name>.yml --syntax-check` for each playbook).
- **Config**: `ansible/.ansible-lint` (profile, skip_list, warn_list). Pre-commit runs ansible-lint on staged `ansible/*.yml` files via a local hook that runs from `ansible/`. Install ansible-lint (e.g. `pip install ansible-lint` or `pipx install ansible-lint`) for the hook to work.

## SteamDeck provisioning

The SteamDeck (`controller.ros2.lan`) is provisioned natively. It runs as a touch-friendly dashboard for the client RPi.

```bash
# Full provisioning (first time):
ansible-playbook -i inventory playbooks/controller.yml -l controller

# Update UI only (after code changes):
ansible-playbook -i inventory playbooks/deploy_steamdeck_ui.yml -l controller
```

See [nodes/steamdeck_ui/README.md](../nodes/steamdeck_ui/README.md) for architecture, config schema, and development docs.

## Deploying Nodes

**Always use `scripts/deploy-nodes.sh`** — never run `ansible-playbook` directly for node deploys.

```bash
# Single node
./scripts/deploy-nodes.sh client web_ui
./scripts/deploy-nodes.sh server lerobot_leader

# Several nodes: ONE playbook run with --tags web_ui,filter_node,bno055_imu (one repo sync)
./scripts/deploy-nodes.sh client web_ui filter_node bno055_imu

# All nodes on a target (includes the full verify step), optionally limited to phases
./scripts/deploy-nodes.sh client --all
./scripts/deploy-nodes.sh server --all --tags config,restart
```

Node names match the `name` field in `group_vars/client.yml` or `group_vars/server.yml` under `ros2_nodes`. The script rejects names that are not `ros2_nodes` entries of the target; each node's steps carry its name as tag (see [Deploy tags](#deploy-tags)).

## Running playbooks

From the **`ansible/`** directory (so `ansible.cfg` and `inventory` are used):

**Full site (provision + deploy nodes):**
```bash
ansible-playbook -i inventory site.yml
```

**Provision only (bootstrap):**
```bash
ansible-playbook -i inventory playbooks/server.yml -l server
ansible-playbook -i inventory playbooks/client.yml -l client
```

**Deploy nodes only (via script from repo root):**
```bash
./scripts/deploy-nodes.sh server --all
./scripts/deploy-nodes.sh client --all
```

## After changing playbooks or roles

When you change playbooks, roles, or templates, re-run the relevant playbook so targets get the updates. When you change node **source code**, re-run the deploy playbook to update the uv venv and restart the service.
