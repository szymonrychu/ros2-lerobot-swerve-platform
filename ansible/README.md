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
  - **`nodes/client/<node>.yml`**, **`nodes/server/<node>.yml`** — Per-node standalone playbooks. Each syncs the repo, deploys exactly one node, and exits. Use these directly or via `scripts/deploy-nodes.sh`.
  - **`tasks/repo_sync.yml`** — Shared include: ensures the Git repo is cloned and up-to-date on the target host.
  - **`tasks/resolve_and_deploy.yml`** — Shared include: looks up a node by name from `ros2_nodes`, resolves all vars, and calls the `ros2_node_deploy` role. Accepts `_deploy_node_name` and optional `_extra_env`.
- **`roles/`**
  - **`common`** — Minimal bootstrap: Python3, git, sudo, basic packages.
  - **`network`** — Netplan: primary interface gets static IP (ethernet or wlan, auto-detected); other interfaces DHCP; IPv6 disabled. Runs when `network_address` and `network_gateway` are set; for primary WiFi set `network_wifi_ssid` (and optionally `network_wifi_password`).
  - **`hostname`** — Set system hostname (hostnamectl, `/etc/hostname`, `127.0.1.1` in `/etc/hosts`). Runs only when `hostname` is set.
  - **`ros2_node_deploy`** — For each node: install Poetry venv from repo (`build_context` path), create config dir, write config file, systemd unit, enable/start; or uninstall (stop, disable, remove unit and config dir). Handlers reload systemd and restart the node when config or unit changes.
  - **`ros2_node_verify`** — Runs after all nodes are deployed: waits for services to settle, checks each present+enabled node’s systemd unit is active, waits again, then re-checks (stability). Used by `deploy_nodes_server.yml` and `deploy_nodes_client.yml`. Variables: `ros2_node_verify_settle_seconds` (default 10), `ros2_node_verify_stable_seconds` (default 5).

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
- I2C clock speed set to 400 kHz (`dtparam=i2c_arm_baudrate`) when `rpi_i2c_baudrate` is defined (client only, requires reboot)

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
| `rpi_i2c_baudrate` | _(undefined)_ | I2C clock speed in Hz; set to `10000` in `group_vars/client.yml` for BNO055 reliability |

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

### I2C Baudrate (BNO055 Reliability)

The `system_optimize` role configures I2C clock speed on the client RPi via
`dtparam=i2c_arm_baudrate` in `/boot/firmware/config.txt`.
Set `rpi_i2c_baudrate: 10000` in `group_vars/client.yml` (current value).
**Requires reboot** to take effect.

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
- I2C on GPIO2/3 (`/dev/i2c-1`)

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

## ROS2 network setup (bind and localhost)

Systemd service env vars control DDS discovery:

- **All nodes:** `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — restricts DDS discovery to localhost, preventing multicast announcements from flooding WiFi. FastDDS uses shared memory (SHM) for same-machine data transport and unicast UDP for discovery.
- **Cross-host (client → server):** `master2master` adds `ROS_STATIC_PEERS=192.168.1.33` for unicast discovery to the server.
- **Cross-host (server → client):** `ros2-master` and `lerobot_leader` add `ROS_STATIC_PEERS=192.168.1.34` so server topics are discoverable from the client.
- **Scraper/GPS on server:** `topic_scraper_api` and `gps_rtk_base` use `LOCALHOST` only — no cross-host peering needed.

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
| realsense_d435i | 50% | 512M |
| lerobot_follower | 50% | 128M |
| swerve_drive_servos | 50% | 128M |
| swerve_controller | 30% | 128M |
| static_tf_publisher | 10% | 64M |
| robot_localization_ekf | 25% | 128M |
| nav2_bringup | 75% | 512M |
| web_ui | 30% | 256M |

## Connection tuning

The `ansible.cfg` `[ssh_connection]` section hardens SSH for flaky WiFi links to the Raspberry Pis:

- **`ControlMaster=no`** / **`ControlPath=none`** — disables SSH multiplexing to avoid stale sockets after network drops.
- **`ConnectTimeout=30`** — fails fast on unreachable hosts (30 s).
- **`ServerAliveInterval=10`** / **`ServerAliveCountMax=6`** — sends a keepalive every 10 s; drops the connection after 60 s of silence.
- **`timeout=120`** — per-task SSH timeout (2 min).
- **`retries=5`** — retries failed SSH connections up to 5 times.

## Linting and testing

- **ansible-lint**: Run from the `ansible/` directory so `roles_path` resolves: `cd ansible && ansible-lint .`. From repo root: `poetry run poe lint-ansible`.
- **test-ansible**: Lint plus playbook syntax-check for all playbooks: `poetry run poe test-ansible` (runs `ansible-lint .` and `ansible-playbook -i inventory playbooks/<name>.yml --syntax-check` for each playbook).
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

# Multiple nodes in parallel (independent nodes run simultaneously)
./scripts/deploy-nodes.sh client web_ui filter_node bno055_imu
./scripts/deploy-nodes.sh client web_ui & ./scripts/deploy-nodes.sh server topic_scraper_api &

# All nodes on a target (sequential, includes full verify step)
./scripts/deploy-nodes.sh client --all
./scripts/deploy-nodes.sh server --all
```

Node names match the `name` field in `group_vars/client.yml` or `group_vars/server.yml` under `ros2_nodes`. Each node has a matching playbook under `playbooks/nodes/<target>/<node>.yml`.

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

When you change playbooks, roles, or templates, re-run the relevant playbook so targets get the updates. When you change node **source code**, re-run the deploy playbook to update the Poetry venv and restart the service.
