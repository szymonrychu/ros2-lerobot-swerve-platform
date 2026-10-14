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
  - **`tasks/cgroup_headroom.yml`** — Client deploys: `system.slice` and `user.slice` drop-ins (templates in `playbooks/templates/`) that guarantee robot headroom; see "Node resource model".
- **`roles/`**
  - **`common`** — Minimal bootstrap: Python3, git, sudo, basic packages.
  - **`network`** — Netplan: primary interface gets static IP (ethernet or wlan, auto-detected); other interfaces DHCP; IPv6 disabled. Runs when `network_address` and `network_gateway` are set; for primary WiFi set `network_wifi_ssid` (and optionally `network_wifi_password`).
  - **`hostname`** — Set system hostname (hostnamectl, `/etc/hostname`, `127.0.1.1` in `/etc/hosts`). Runs only when `hostname` is set.
  - **`ros2_node_deploy`** — For each node: create the node venv (`python3 -m venv --system-site-packages`) and `uv sync --frozen --no-dev` from the repo (`build_context` path), create config dir, write config file, systemd unit, enable/start; or uninstall (stop, disable, remove unit and config dir). Handlers reload systemd and restart the node when config or unit changes.
  - **`ros2_node_verify`** — Runs as the last step of the play, after the end-of-play restarts: waits for services to settle, checks each present+enabled node’s systemd unit is active, waits again, then re-checks (stability). Used by `deploy_nodes_server.yml` and `deploy_nodes_client.yml`. Variables: `ros2_node_verify_settle_seconds` (default 10), `ros2_node_verify_stable_seconds` (default 5).

  - **`system_optimize`** — Ubuntu 24.04 debloating, performance tuning, and resilience hardening for Raspberry Pi. See [System optimization](#system-optimization) below.
  - **`docker_cleanup`** — Full Docker removal for migration to native ROS2 nodes. Stops all running containers, prunes images/volumes/networks, stops and disables Docker/containerd services, purges Docker CE packages (`docker-ce`, `docker-ce-cli`, `containerd.io`, plugins), removes data dirs (`/var/lib/docker`, `/etc/docker`, `/var/lib/containerd`), removes systemd overrides, APT repo and keyring files, removes the user from the `docker` group, reloads systemd, and autoremoving unused packages. All steps are skipped if Docker is not installed.
  - **`monitoring_agent`** - Grafana Alloy on both Pis (metrics -> the server's Mimir, journal -> its Loki), `monitoring.slice`, the RPi throttling timer, and on the robot the removal of its former Prometheus and Grafana. **`web_proxy`** - nginx on port 80 of both (robot: web UI, `/grafana/` redirected to the server; server: Grafana). **`monitoring`** - Grafana Mimir + Loki + Grafana on the server RPi4. Included by `deploy_nodes_client.yml` (agent, proxy) and `deploy_nodes_server.yml` (agent, proxy, stack) with tag `monitoring` only. See [Monitoring](#monitoring).
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

The **topic_scraper_api** node is installed on both hosts but disabled (`enabled: false`, 2026-10-11: about 0.4 cores of always-on subscriptions on the robot); set `enabled: true` in `group_vars/<host>.yml` and deploy to use it for a debugging session. When running it exposes:

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

## Jetson (vision coprocessor)

The Jetson Nano computes stereo depth and runs no ROS and no Pi roles.

- **Group:** `[jetson]` in the inventory (`jetson-nano ansible_host=192.168.1.32`), vars in `group_vars/jetson.yml` (`ansible_user: jetson`, Python 3.8, hostname `nano`, `192.168.1.32/24`).
- **Kept out of the Pi plays:** `site.yml` (provision) and `playbooks/optimize.yml` target `server:client`, so `common`, `network`, `hostname` and `system_optimize` never run on it. `tests/test_jetson_ansible.py` pins this.
- **Deploy targets:** `./scripts/deploy-nodes.sh jetson base build_deps depthd` (or `jetson --all`) runs `playbooks/deploy_nodes_jetson.yml -l jetson`; it has no `ros2_nodes` entries, only those three targets. The Pi side of the cable is `./scripts/deploy-nodes.sh client jetson_link`.
- **WiFi kept:** the Jetson stays on WiFi (192.168.1.32) for development; only the depth stream uses the cabled link.

### Time sync and NAT for the Jetson (role `jetson_link`)

The Pi is the Jetson's time server and its way to the internet, so the Jetson's frame timestamps share the Pi's clock.

- **Apply:** `./scripts/deploy-nodes.sh client jetson_link` (the `jetson_link` tag: the `network` role with the client's `network_link_ethernets`, then `jetson_link` with `jetson_link_side: server`). Node and phase tags never run it; an untagged client run does. The Jetson side (`jetson_link_side: client`) is applied by `playbooks/deploy_nodes_jetson.yml`.
- **Time:** both sides install chrony and disable `systemd-timesyncd`. The Pi keeps the internet pool as upstream, serves `allow 10.77.0.0/30` and `local stratum 10`, so the Jetson syncs even when the Pi is offline. The Jetson uses `server 10.77.0.1 iburst prefer` with the pool as fallback over WiFi, and `makestep 1.0 3`. Check with `chronyc sources` on the Jetson and `chronyc clients` on the Pi.
- **NAT (Pi only):** `net.ipv4.ip_forward = 1` in `/etc/sysctl.d/90-jetson-link-forward.conf`, and the nftables table `jetson_link` in `/etc/nftables.d/jetson-nat.nft` (included from `/etc/nftables.conf`, loaded on boot). It masquerades `10.77.0.0/30` out of the Pi's primary interface and forwards only between the link interface and the primary interface; all other forwarding is untouched.
- **Names:** `/etc/hosts` gets `10.77.0.2 jetson.ros2.lan` on the Pi and `10.77.0.1 client.ros2.lan` on the Jetson.
- **Idempotent:** every change task registers; chrony is restarted and the ruleset reloaded only when its file changed.

### OS base of the Jetson (role `jetson_base`)

`./scripts/deploy-nodes.sh jetson base` (tag `base` of `playbooks/deploy_nodes_jetson.yml`) applies, in order, the `network` role (Jetson link vars, WiFi stanza unchanged), `jetson_link` with `jetson_link_side: client`, and `jetson_base`. `jetson_base` codifies the end state of the 2026-10-14 cleanup (`docs/plans/2026-10-14-jetson-depth-offload-briefs/jetson-cleanup-log.md`).

- **Masked units:** `jetson_base_masked_units` (defaults, each with its reason, copied from the log) are linked to `/dev/null` in `/etc/systemd/system`; default target `multi-user.target`.
- **Journal:** `/etc/systemd/journald.conf.d/persistent.conf` (`Storage=persistent`, `SystemMaxUse=200M`) and `/var/log/journal`; journald restarts only when the file changed.
- **Clocks:** `nvpmodel -q` is read and `nvpmodel -m 0` (MAXN) runs only when it differs; `/etc/systemd/system/jetson-clocks.service` (oneshot, `After=nvpmodel.service`, `jetson_clocks`) is enabled.
- **Keep-list:** the role fails when any `jetson_base_keep_packages` (the 27 `nvidia-l4t-*`, CUDA 10.2 runtime and nvcc, `libcudnn8`, `libnvinfer8`, `libnvvpi1`) or `/usr/lib/python3/dist-packages/cv2` (OpenCV 4.8 with CUDA) is missing. The role never purges a package.
- **ISP override:** `files/camera_overrides.isp` (Waveshare IMX219 stereo wiki, `Camera_overrides.tar.gz`; URL and sha256 in the defaults) goes to `/var/nvidia/nvcam/settings/` mode 0664; `nvargus-daemon` restarts only when it changed (fixes the pink cast).
- **Controller:** ansible-core >= 2.19 refuses the Jetson's Python 3.8, so `deploy-nodes.sh jetson ...` runs ansible-core 2.18 through `uvx` when the installed one is newer. The Jetson's chrony 3.5 has no `ntsdumpdir`, so that line is Pi-only; check the source with `chronyc -n sources` (without `-n` the Pi shows as `client.ros2.lan`).
- **WiFi safety:** after the network role, `/etc/netplan/00-wifi.yaml` is removed by the `network` role (`network_netplan_superseded_files`), before netplan applies and only when `01-ansible-netplan.yaml` contains the `wlan0` stanza (netplan rejects the same access point in two files).

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

- **`network_link_ethernets`** — Optional list of static point-to-point ethernets (default `[]`). Each entry has `iface`, `address` (CIDR) and optionally `default_route_via` and `route_metric`. Entries render with `dhcp4: false`, `ignore-carrier: true` (the address exists before the peer is up) and `optional: true`, and are left out of the DHCP list. Before writing netplan the role probes each link interface and fails, naming the address, if it holds an IPv4 address other than the planned one.

**Direct Pi-Jetson link (design D8).** `group_vars/all.yml` defines `jetson_link_*` (interface `eth0`, Pi `10.77.0.1/30`, Jetson `10.77.0.2/30`) and sets `network_link_ethernets` by group: the client gets `10.77.0.1/30`; the Jetson gets `10.77.0.2/30` plus a default route via `10.77.0.1` at metric 1000, so the Jetson's WiFi route keeps winning while it is up. The WiFi stanza is unchanged. The Pi picks this up only in a deploy of its own.

The network role writes a netplan file under `/etc/netplan/` and runs `netplan apply`. The **hostname** role runs `hostnamectl set-hostname` and updates `/etc/hostname` and `/etc/hosts`.

## ROS2 network setup (DDS discovery)

- **All ROS2 nodes:** the unit template emits `ros2_dds_env` (`group_vars/all.yml`) first: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (simple discovery restricted to the host) and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (the shared-memory transport is off: a SIGKILLed participant's SHM port is reused by its restart while peers keep writing into the deleted segment, which left web_ui deaf after an OOM restart on 2026-10-11; trade-off: same-host image topics are copied through loopback UDP, costing CPU). Node `env` entries come after it and override. Raw `sensor_msgs/Image` (640x480) over loopback UDP is unsupported: `rmem_default` is 1 MiB and no Fast DDS socket-buffer is configured, so a raw subscriber on the Pi can drop fragments; use the compressed topics.
- **Allocator env:** `MALLOC_MMAP_THRESHOLD_` (client.yml, e.g. plan_grasp's node) disables glibc's dynamic mmap/trim threshold, so every allocation of 1 MiB or more is mmap'd and zero-faulted on each use. Do not change or copy it blindly: measure RSS and CPU on the robot first (and consider `MALLOC_TRIM_THRESHOLD_` only if the cost is visible).
- **Cross-host:** only the client `master2master` adds `ROS_STATIC_PEERS={{ ros2_server_hostname }}`, so it is the single process exchanging DDS traffic with the server. Server nodes list no static peers: when they listed the client, every client node meshed with the server over Wi-Fi.
- **Steam Deck UI:** SUBNET discovery with the client as static peer (`steamdeck_ros2_static_peers`).
- **Shells:** `/etc/profile.d/ros2_dds.sh` (from `playbooks/tasks/dds_host_setup.yml`) renders the full `ros2_dds_env` (localhost discovery and `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`) for SSH sessions, so the `ros2` CLI and `scripts/*_diag.sh` see the graph. Non-login SSH commands should `source /etc/profile.d/ros2_dds.sh`.
- **logind IPC:** nodes run as a regular user, and logind's default `RemoveIPC=yes` deletes that user's POSIX shared memory on every SSH logout (the nodes themselves use UDPv4, but other shared-memory users of that account would lose it too). `dds_host_setup.yml` installs `/etc/systemd/logind.conf.d/ros2-keep-ipc.conf` (`RemoveIPC=no`) and restarts running `ros2-*` services once when it is first applied.
- **History:** a FastDDS discovery server per host was tried (2026-10-03) and removed: launch_ros' one-shot lifecycle and component-loading service calls (slam_toolbox configure, Nav2 composable nodes) hung intermittently through it on the busy client. The `fastdds_discovery_server` node stays `present: false` so deploys uninstall it.

## Boot network wait

`--all` deploys install `/etc/systemd/system/systemd-networkd-wait-online.service.d/any-interface.conf` (`playbooks/tasks/network_wait_online.yml`): boot waits for any one interface for at most 30 s. The netplan template also marks secondary Ethernet ports `optional: true` (applied when the network role runs). Without this, an unplugged `eth0` held every ROS node for the 2-minute wait-online timeout.

## Deploy load management

A deploy no longer stops the whole stack up front. The role's `stop_for_build.yml` stops all running `ros2-*` services, once per play, only right before a heavy step that will actually run: the uv sync of a node whose dependency hash changed, the web_ui `npm ci` / `npm run build`, or a pinned upstream colcon source build whose stamp is missing (a local in-repo C++ package builds without stopping anything, see [Source-built ROS packages](#source-built-ros-packages-colcon)). So installs and builds still run on an otherwise idle Pi (the client overheated and froze on 2026-10-03), while a no-change deploy stops nothing. Nothing is restarted while nodes are deployed either: a changed config, unit, launcher, source, dependency set or build output queues a restart by appending the node to the host file `/var/lib/ros2-deploy/pending-restart` (`ros2_restart_queue`, one name per line), and `playbooks/tasks/start_ros_nodes.yml` carries out all of them once, at the end, in `ros2_nodes` order (dependencies first), one node at a time with `ros2_node_start_interval_s` (2 s, `group_vars/all.yml`) between them. It fails the run if a restart fails and clears the queue only after all succeeded, so a failed run keeps its queued restarts for the next one. The same step starts enabled nodes in the run's scope that are not running (stopped for a build); with a node filter (`--tags web_ui`) the scope is the selected nodes, queued nodes are restarted whatever the scope, and verify checks the scope plus the restarted nodes. The node tasks of both deploy playbooks run inside a `block` whose `rescue` runs the same start step (errors ignored) and then fails the run with the original error, so a failure after a build stopped the nodes never leaves the robot down.

**Failure safety.** (1) The restart intent is written to the host queue by a task right after each change is applied (uv sync, npm build and static copy, config, launcher, unit, colcon build, source stamp), not only at the late handler flush; the source stamp is written last, after config, launcher and unit succeeded. A failure at any point therefore still leaves the restart queued, and the start step runs `systemctl daemon-reload` first so a changed unit takes effect. (2) Before a heavy step stops the running units, they are appended to `/var/lib/ros2-deploy/stopped-for-build`; the start step starts every listed unit whatever the run's scope (also with `--tags <node>`) and clears the file after success. (3) `playbooks/tasks/deploy_guard.yml` installs `ros2-deploy-recover.timer` (every 2 minutes): when the stopped list is older than `ros2_recover_after_min` (15) and no deploy lock younger than `ros2_deploy_lock_max_age_min` (240) exists, it starts the listed units and clears the file. This covers a lost controller or SSH session, where the rescue path cannot run on an unreachable host. The lock `/var/lib/ros2-deploy/deploy.lock` is taken as the last pre-task and released after verify or in the rescue; a lock left by a dead controller stops blocking after its max age. The pre-tasks (repo sync, apt, firmware config) run before any node is stopped, so a failure there needs no rescue. The rescue message carries the failed task, its message, rc and stderr.

**What counts as a changed source.** Every node gets a source stamp (`/var/lib/ros2-deploy/stamps/<node>.src`) holding the hash of the git tree hashes of its source paths on the synced checkout: `node_src_dir` for Python nodes, `src_paths` for launch/config-only node types (rplidar_a1, overview_camera, rf2o_laser_odometry, robot_localization_ekf, slam_toolbox, nav2_bringup, ros2_master), plus the type's `src_extra_paths` (mcp_server also reads `nodes/web_ui/urdf`) and `shared/` when the type sets `src_shared: true` (every node type that imports `shared/ros2_metrics`: feetech_servos, filter_node, swerve_controller, rf2o_odom_relay, bno055_imu, gps_rtk, uvc_camera, arducam_tof, stereo_depth, poi_store, costmap_zones, claude_agent, test_joint_api, topic_scraper_api, mcp_server, web_ui; a root test checks every pyproject with a `../../shared` dependency is declared). `rplidar_a1` has no venv: it lists `shared/ros2_metrics` in `src_extra_paths`, gets `python3-prometheus-client` from apt and `PYTHONPATH=<ros2_repo_dest>/shared/ros2_metrics` in its unit env. A different hash queues the restart. Running nodes nothing changed for are left alone and cost no sleep. `ros2_node_verify` runs last, after those restarts, and checks all units with one `systemctl is-active` per round.

## web_ui frontend build

The web_ui frontend is built on the client during deploy (`npm ci`, `npm run build`), but only when needed: `npm ci` when the hash of `package.json` + `package-lock.json` differs from `frontend/.npm-ci-stamp` (or `node_modules` is gone), `npm run build` when the git tree hash of `nodes/web_ui/frontend` differs from `frontend/.build-stamp`, after an `npm ci`, or when `dist/index.html` / `web_ui/static/index.html` is missing. The stamps are written after the step succeeded; the node restarts only when a build ran. Both steps run in a transient systemd scope (`systemd-run --scope -p CPUQuota={{ ros2_build_cpu_quota }} -p CPUWeight={{ ros2_build_cpu_weight }} -p IOWeight=10`, 400% = all four cores since the Pi got active cooling; was 100% while it overheated) under `nice -n 19 ionice -c 3`, which keeps the running ROS nodes first in line. On 2026-10-03 a full-priority build next to Nav2, SLAM and the bridges on the then-uncooled Pi 5 overheated it until it stopped responding: lower `ros2_build_cpu_quota` again if temperatures climb.


### Build dependencies of depthd (role `jetson_depthd`, tag `build_deps`)

`./scripts/deploy-nodes.sh jetson build_deps` applies `jetson_depthd` (tags `build_deps` and `depthd`; the `depthd` tasks arrive later). `tasks/main.yml` imports `tasks/deps.yml` under `build_deps`.

- **cmake:** the Kitware APT repository for focal (key in `/usr/share/keyrings/kitware-archive-keyring.asc`, `signed-by`) and `cmake`/`cmake-data` at `state: latest` replace the distro 3.16 with >= 3.20 (`ctest --test-dir`, libSGM); the role asserts the version. This is the only upgrade the role makes.
- **Packages:** `libzmq3-dev`, `libzstd-dev`, `nlohmann-json3-dev`, `libyaml-cpp-dev`, `libgtest-dev`, `pkg-config` (`jetson_depthd_build_packages`).
- **CUDA stack:** the cleanup purge removed the `cuda-toolkit-10-2` and `tensorrt` metapackages. The role installs the leaf dev packages (`cuda-compiler-10-2`, `cuda-cudart-dev-10-2`, `cuda-libraries-dev-10-2`, `libcudnn8-dev`, `libnvinfer-dev`, `libnvonnxparsers-dev`, `libnvinfer-plugin-dev`, `vpi1-dev`) from the L4T r32.6 repository (added only when missing) and points `/usr/local/cuda` at cuda-10.2 with `update-alternatives` only when it does not already resolve there. The metapackages are not reinstalled because they pull the docs and samples the cleanup removed.
- **Never touched:** no OpenCV package is installed (OpenCV 4.8 CUDA stays) and nothing is removed.
- **Directories:** `/var/lib/depthd` (`jetson:jetson` 0755) and `/etc/depthd` (`root` 0755). A second run reports no changes.
### Source-built ROS packages (colcon)

Packages missing from the Jazzy apt distribution are built from a pinned git commit. A node type with a `colcon_source` block gets `roles/ros2_node_deploy/tasks/colcon_source_build.yml`, which takes ONE source dict (`rf2o_laser_odometry`) or a LIST of sources (`overview_camera`) and builds them in list order, so dependencies come first (libcamera before camera_ros). Each source is `{repo, commit, package, workspace}` plus optional `patches`, `cmake_args` (default `-DCMAKE_BUILD_TYPE=Release`), `meson_args` (builds that package with colcon-meson, `--meson-args`), `base_paths` (directories inside the checkout passed to `colcon build --base-paths`, for a multi-package repo such as navigation2 so its unbuilt sibling packages do not shadow the apt ones; unset for single-package repos), `stamp_apt_packages` (apt packages whose installed version, read with `dpkg-query -W -f='${Version}'` also in check mode, is appended to the stamp key; only when the list is non-empty, so rf2o/libcamera/camera_ros keys are unchanged) and `jobs` (make `-j`, default `ros2_build_jobs`). `commit` is a SHA or a tag. `colcon_source_package.yml` builds one source: the commit is cloned to `<workspace>/src/<package>` and built with `colcon build --merge-install --parallel-workers 1 --packages-select <package>` (`MAKEFLAGS=-j{{ colcon_src.jobs | default(ros2_build_jobs) }}`; meson's ninja uses all cores the cgroup allows) inside the same `systemd-run --scope -p CPUQuota={{ ros2_build_cpu_quota }} -p CPUWeight={{ ros2_build_cpu_weight }} -p IOWeight=10 nice -n 19 ionice -c 3` wrapper as the web_ui build, after sourcing `/opt/ros/jazzy/setup.bash` and the workspace's own `install/setup.bash` (so camera_ros finds the fresh libcamera). Optional `patches` (a list of unified diffs in the repo, applied with `ansible.builtin.patch` after the checkout; the git task uses `force` so the clone is reset before they are re-applied) can fix upstream bugs. A stamp file `<workspace>/.built-<package>-<hash>` makes it idempotent; the hash covers the pinned commit, the content of every patch, the build args and (for the second and later sources) the stamps of the sources before it, so the build only reruns, and the node restarts, when one of them changes, or when `<workspace>/install/setup.bash` is missing (the stamp is then removed). For rf2o (single source, no build args) the hash is the original `sha1(commit + patch checksums)`, so existing stamps stay valid. The launcher script sources `<workspace>/install/setup.bash` (of the first source) after `/opt/ros/jazzy/setup.bash`. Used by `rf2o_laser_odometry`, `overview_camera` and `nav2_bringup` (workspace `/opt/ros2-ws`). `nav2_bringup` builds the patched `nav2_mppi_controller` (client.yml `nav2_mppi_overlay_source`, navigation2 tag 1.3.13 = apt version, `model_delay` patch, `jobs: 2`, `stamp_apt_packages: [ros-jazzy-nav2-mppi-controller]` so a ros-jazzy sync that moves the apt MPPI rebuilds the overlay); its `colcon_source` is `{}` when `nav2_mppi_overlay` is `false`, which is the rollback: the launcher stops sourcing the workspace and the apt plugin is used again (see `nodes/nav2_bringup/README.md`). The build is synchronous: `stop_for_build.yml` stops every `ros2-*` unit (including claude_agent and mcp_server) for its duration, and the first build compiles xtensor at `-O3`, so expect a long deploy; the first deploy compiles on the Pi, so expect a slow, low-priority build (libcamera needs network once for its libpisp meson wrap). Build dependencies come from the node type's `apt_packages`.

**Local in-repo packages (`colcon_source.local`).** The C++ node types build their ament packages from the repo checkout instead of a pinned upstream commit. A source item `{local: <repo path>, package, workspace, stamp_apt_packages}` (for example `{local: nodes/stereo_depth_cpp, package: stereo_depth_cpp, workspace: /opt/ros2-ws}`) is rsynced on every deploy from `<ros2_repo_dest>/<local>/` into `<workspace>/src/<package>/` (`rsync -a --checksum --delete --delete-excluded`, Python caches excluded; a no-op when nothing changed). Its stamp key is a hash of the synced tree (`find . -type f` in byte order, `sha256sum` of every file, so both paths and contents count) plus the installed versions of its `stamp_apt_packages`, its build args and the chain key of the sources before it, kept as `<workspace>/.built-<package>-<key>`: an unchanged deploy builds nothing, a changed, added, removed or renamed source file or an apt upgrade rebuilds. The shared library package is chained in front: both `_cpp` node types list `{local: shared/ros2_common_cpp, package: ros2_common_cpp}` first, so a change there rebuilds the node package too (the second node type finds the shared package's stamp and skips it). A local build discovers only its own tree (`--base-paths <workspace>/src/<package>/.`), runs in the same low-priority scope (`systemd-run --scope -p CPUWeight={{ ros2_build_cpu_weight }}` (20) `-p IOWeight=10 nice -n 19 ionice -c 3`, `--parallel-workers 1`) with `-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF` and `MAKEFLAGS=-j2` by default (`cmake_args` and `jobs` override), and **never includes `stop_for_build.yml`**: an in-repo package is small, so it builds next to the running robot, and only the node's own unit restarts afterwards (the restart queue). No clone and no patch run for a local source. The pinned upstream builds (rf2o, libcamera/camera_ros, MPPI) keep their stamp keys, their `make -j` and the stop-all. Tests: `tests/test_cpp_node_deploy.py` renders the role's sync, hash and key expressions on a temporary tree.

**Camera boot overlay.** `playbooks/tasks/overview_camera_boot_config.yml` (included from `pre_tasks` of `deploy_nodes_client.yml`, tags `boot` and `overview_camera`) sets up two IMX219 cameras on the Pi 5 CSI ports (left cam0, right cam1). It first removes `dtoverlay=imx708,cam0` and `dtoverlay=arducam-pivariety,cam1` (`state: absent`; the Arducam ToF is left without a CSI port and stays disabled), then ensures `camera_auto_detect=0`, `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1` in `/boot/firmware/config.txt` (same file and `lineinfile` pattern as the UART overlay). No task removes an imx219 line. Every task runs on aarch64 only and registers its result, and the client reboots only when a line was added, changed or removed. The camera_ros source carries `nodes/overview_camera/patches/camera_ros-0001-expose-sync-controls.patch`, which exposes the libcamera `rpi::SyncMode` and `rpi::SyncFrames` controls as parameters so the two cameras can run as sync server and client. The `overview_camera` node type installs no apt `ros-jazzy-camera-ros` / `ros-jazzy-libcamera`: they would shadow the source-built fork. `realsense_d435i` is `present: false` (uninstalled on deploy); no `camera_link` frame is published by `static_tf_publisher` until the overview camera mount pose is measured. See `nodes/overview_camera/README.md`.

**Arducam ToF camera (`arducam_tof`).** Native node type in `group_vars/client.yml` (`python3 -m arducam_tof` from `nodes/bridges/arducam_tof`, `src_shared: true` for `shared/ros2_metrics`, config in `/etc/ros2/arducam_tof/config.yaml` via `ARDUCAM_TOF_CONFIG`, normal cgroup tier, `memory_low` 64M / `memory_max` 192M). Its `ros2_nodes` entry is `present: true`, `enabled: false` until the camera is plugged in on CSI cam1 (`camera_index: 1`; the `dtoverlay=arducam-pivariety,cam1` line above). The `config: |` block holds only keys the driver accepts (`metrics_port: 19112`, `camera_index`, `range_mode` near or far, `confidence_threshold`, `frame_id: tof_optical_frame`, `publish_rate_hz`); the driver rejects any other key. Topics are fixed in the driver: `/tof/depth/image_raw`, `/tof/amplitude/image_raw`, `/tof/camera_info` (read by the web UI head tab). To enable: set `enabled: true`, add its `monitoring_agent_node_targets` entry (`127.0.0.1:19112`) and run `./scripts/deploy-nodes.sh client arducam_tof`. See `nodes/bridges/arducam_tof/README.md`.

**Stereo depth (`stereo_depth`).** Native node type in `group_vars/client.yml` (`python3 -m stereo_depth` from `nodes/stereo_depth`, `src_shared: true` for `shared/ros2_metrics`, config in `/etc/ros2/stereo_depth/config.yaml` via `STEREO_DEPTH_CONFIG`, apt `ros-jazzy-message-filters` and `ros-jazzy-stereo-msgs`, background cgroup tier, `memory_low` 96M / `memory_max` 384M estimate). Its `ros2_nodes` entry is present and enabled; the `config: |` block makes every `DepthConfig` default explicit with `metrics_port: 19113`, scraped by Alloy (`monitoring_agent_node_targets`). Deployed by the tagged `Deploy stereo_depth` block right after `overview_camera`: `./scripts/deploy-nodes.sh client stereo_depth`. It publishes `/stereo/depth/image_raw` (16UC1 mm, read by the web UI head tab) only once `/var/lib/ros2/camera_calibration` holds a valid stereo calibration. See `nodes/stereo_depth/README.md`.

**C++ ports (`stereo_depth_cpp`, `map_guard_cpp`).** Side-by-side native node types in `group_vars/client.yml` for the rclcpp ports in `nodes/stereo_depth_cpp` and `nodes/map_guard_cpp`: `node_src_dir: ""`, `src_paths: []` (the build stamp restarts them), `colcon_source` = the local `shared/ros2_common_cpp` package then the node's own local package (workspace `/opt/ros2-ws`, see above), launch `ros2 run stereo_depth_cpp stereo_depth_node` / `ros2 run map_guard_cpp map_guard_node`, config in `/etc/ros2/<name>/config.yaml` through the Python node's variable (`STEREO_DEPTH_CONFIG`, `MAP_GUARD_CONFIG`) and the C++ build dependencies in `apt_packages` (build-essential, cmake, rsync, colcon, ament_cmake, rclcpp, rclcpp_components, yaml-cpp, prometheus-cpp, plus OpenCV/cv_bridge/message_filters or tf2_ros/slam_toolbox/cpp-httplib/nlohmann-json). `stereo_depth_cpp` is in the background tier exactly like `stereo_depth`, `map_guard_cpp` in the normal tier with map_guard's memory. Their `ros2_nodes` entries sit right after the Python node's, with a `config: |` block identical to the Python node's, so they share its metrics port (19113, 19110): only one of a pair may run (`tests/test_node_metrics_wiring.py`). Deployed by the tagged `Deploy stereo_depth_cpp` / `Deploy map_guard_cpp` blocks right after the Python counterparts. Both are live since the cut-over, see [Cut-over and rollback](#cut-over-and-rollback).

**`jetson_bridge_cpp`.** The Pi side of the Jetson depth offload (`nodes/jetson_bridge_cpp`): a native C++ node type built like the ports above (local `shared/ros2_common_cpp` then `nodes/jetson_bridge_cpp`, `/opt/ros2-ws`, `jobs: 2`, stamped apt packages incl. `libzmq3-dev`, `libzstd-dev`, `prometheus-cpp-dev`), launched `ros2 run jetson_bridge_cpp jetson_bridge_node` with `JETSON_BRIDGE_CONFIG`. It has no Python counterpart, sits in the normal robot tier (500/500/-500, like the camera it replaces) with `memory_low` 32M and `memory_max` 256M (estimate). Its `ros2_nodes` entry is present and enabled with every config key explicit; the endpoint is `tcp://{{ jetson_link_jetson_ip }}:5550`. Deployed by the tagged `Deploy jetson_bridge_cpp` block (`./scripts/deploy-nodes.sh client jetson_bridge_cpp`). `tests/test_jetson_bridge_config.py` pins it.

**Arducam ToF SDK apt task.** `playbooks/tasks/arducam_tof_sdk.yml` (included from `pre_tasks` of `deploy_nodes_client.yml`, tags `arducam_tof` and `apt`, aarch64 only) adds the Arducam arm64 apt repo (`deb [arch=arm64 trusted=yes] https://arducam.github.io/arducam_ppa linux main`, file `arducam`; `trusted=yes` because Arducam publishes no signing key the playbook could pin) and installs `arducam-tof-sdk-dev`. It never fails a deploy: the repo targets Raspberry Pi OS and is unproven on Ubuntu noble, so when the repo or the package is unavailable a `rescue` removes the repo file again (a dead repo would break the batched node apt run) and prints a warning; the node then stays down until the SDK installs. It is idempotent (no change once the repo and package are present). The node itself is deployed by the tagged `Deploy arducam_tof` block of the same playbook.

## ROS package sync

`--all` deploys (`deploy_nodes_client.yml`, `deploy_nodes_server.yml`) run `playbooks/tasks/ros_packages_sync.yml` right after the repo sync. It refreshes the apt index (at most hourly), upgrades every installed `ros-jazzy-*` package that has an update, and restarts the running `ros2-*` services when anything was upgraded. Mixing packages from different packages.ros.org syncs breaks ABI: on 2026-10-03 a freshly installed `laser_filters` failed with an undefined `diagnostic_updater` symbol.

## Cut-over and rollback

`stereo_depth_cpp` and `map_guard_cpp` are `present: true`, `enabled: true`; `stereo_depth` and `map_guard` stay `present: true`, `enabled: false` (installed, stopped, disabled) so the rollback is one line. The `_cpp` config blocks equal the Python ones (`map_guard_cpp` keeps `dry_run: true`), and `monitoring_agent_node_targets` scrapes the `_cpp` names on the shared ports (19113, 19110). `tests/test_cpp_cutover.py` pins all of this.

Deploy the cut-over (first deploy builds the packages on the Pi at nice 19 without stopping the robot):

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client stereo_depth stereo_depth_cpp map_guard map_guard_cpp 2>&1 | tee .logs/deploy-client-cutover.log
```

Rollback, per node: in `group_vars/client.yml` set `enabled: true` on the Python entry (`stereo_depth` / `map_guard`) and `enabled: false` on its `_cpp` entry, rename the matching `monitoring_agent_node_targets` node back, then run the same `scripts/deploy-nodes.sh client <python> <cpp>` command. Never enable both of a pair (one metrics port, duplicate publishers).

Measure the saving with `scripts/node_cpu_compare.py` (per-node mean and p95 cores before and after, from Mimir; see `scripts/README.md`).

## Node resource model: robot headroom guaranteed, background constrained

The robot (client) Pi exists to run the robot. Its cgroups therefore guarantee the ROS2 node units stable headroom
instead of squeezing them to a minimum, and constrain everything else. Until 2026-10-11 every robot unit had a hard
`CPUQuota` and a `MemoryHigh` of 7/8 `MemoryMax`: the follower was throttled in 87 of 105 periods, web_ui, the scraper and
map_guard in every period, and mcp_server sat above `MemoryHigh` (PSI some 83-85%, 57k memory.high events) until it was
OOM-killed. The model now is:

- **No CPU quota, no reclaim throttling for robot units.** Client node types set no `cpu_quota` and no `memory_high`;
  the template renders `CPUQuota=` / `MemoryHigh=` only when a type sets them (the server's types still do).
- **Priority from weights.** `CPUWeight` / `IOWeight` tiers far above systemd's default 100, so robot units win every
  contention inside `system.slice` while idle cores stay usable by anyone.
- **Memory protection from `MemoryLow`** at about each type's measured typical working set: under memory pressure the
  kernel reclaims from everything else first.
- **`MemoryMax` only as a safety net**, at least twice the 24 h peak (mcp_server 1G while its 383 MB spike is being
  investigated), so a leak is contained without ever clipping normal operation.
- **`OOMScoreAdjust` negative for robot units** (critical most negative); monitoring units are +500, so the OOM killer
  takes background work first.
- **`Restart=always`** (`RestartSec=15`): a unit comes back after any exit, a clean one included. On 2026-10-11 09:45
  nav2_bringup's launch exited 0 after its component container segfaulted and stayed down under `Restart=on-failure`.
  A unit stopped through systemd (the deploy's `systemctl stop` before a build in `stop_for_build.yml`, `state: stopped`
  for `present: false` and `enabled: false` nodes) is not an exit and stays stopped.

### Unit directives

The native unit template (`roles/ros2_node_deploy/templates/ros2-node-native.service.j2`) renders each optional
directive only when its `ros2_node_type_defaults` key is set; `playbooks/tasks/resolve_and_deploy.yml` maps every key
straight through (empty when unset, nothing is derived). A unit change queues the usual daemon-reload + restart.

| `ros2_node_type_defaults` key | Template variable | Unit directive |
|---|---|---|
| `cpu_quota` | `node_cpu_quota` | `CPUQuota=` (server types only) |
| `memory_low` | `node_memory_low` | `MemoryLow=` |
| `memory_max` | `node_memory_max` | `MemoryMax=` (always; 256M when unset) |
| `memory_high` | `node_memory_high` | `MemoryHigh=` (no type sets it) |
| `cpu_weight` | `node_cpu_weight` | `CPUWeight=` |
| `io_weight` | `node_io_weight` | `IOWeight=` |
| `oom_score_adjust` | `node_oom_score_adjust` | `OOMScoreAdjust=` |

### Client weight tiers

| Tier | cpu_weight | io_weight | oom_score_adjust | Node types |
|---|---|---|---|---|
| critical | 1000 | 1000 | -900 | `feetech_servos` (lerobot_follower, swerve_drive_servos), `swerve_controller`, `ros2_master`, `filter_node`, `static_tf_publisher`, `bno055_imu` |
| normal robot | 500 | 500 | -500 | `rplidar_a1`, `rf2o_laser_odometry`, `rf2o_odom_relay`, `robot_localization_ekf`, `nav2_bringup`, `slam_toolbox`, `laser_filter`, `gps_rtk`, `poi_store`, `costmap_zones`, `overview_camera`, `uvc_camera`, `arducam_tof`, `realsense_d435i`, `test_joint_api`, `map_guard`, `map_guard_cpp`, `jetson_bridge_cpp` |
| agent-facing | 300 | 300 | -200 | `mcp_server`, `claude_agent`, `web_ui`, `topic_scraper_api`, `master2master`, `haptic_controller` |
| background | 100 | 100 | 0 | `stereo_depth`, `stereo_depth_cpp`, `joint_state_throttle_follower`, `joint_state_throttle_swerve` (work the robot can lose: under contention the depth rate drops, and it is killed before any robot node) |

Background for comparison: other `system.slice` services 100, `user.slice` 50, deploy build scopes 20,
`monitoring.slice` 20. Weights are shares under contention, not limits: a critical unit gets twice a normal one's share
and ten times a default service's. `overview_camera` keeps `Nice=5` and `claude_agent` `Nice=10` inside their cgroups.

### Joint-state throttles (client)

Idle consumers (mcp_server) do not need the 95 Hz joint states. Two apt-binary nodes (`ros-jazzy-topic-tools`, installed through the type's `apt_packages`) publish 10 Hz copies, same message type and QoS as the source:

| Node (= node type) | Command | Copy |
|---|---|---|
| `joint_state_throttle_follower` | `ros2 run topic_tools throttle messages /follower/joint_states 10.0 /follower/joint_states/throttled` | `/follower/joint_states/throttled` |
| `joint_state_throttle_swerve` | `ros2 run topic_tools throttle messages /swerve_drive/joint_states 10.0 /swerve_drive/joint_states/throttled` | `/swerve_drive/joint_states/throttled` |

These are exactly the topics mcp_server's config subscribes to (`topics.follower_joint_states`, `topics.swerve_joint_states`). Both are `present` and `enabled` in `group_vars/client.yml`, have a playbook step each (tags: node name, `apt`, `python`, `build`, `config`), run in the background weight tier and carry no `/metrics` (no repo code; health is the systemd collector and the unit's cgroup series). They run no repo file, so only a package or command change restarts them. Test: `tests/test_joint_state_throttle_config.py`.

### Client memory (measured 2026-10-11 08:23 UTC, Mimir, 24 h window)

`memory_low` is the p50 working set rounded up to 8 MiB, `memory_max` at least twice the 24 h peak. Types with no
series that day (`master2master`, `haptic_controller` are deployed disabled, `realsense_d435i` is retired) carry an
estimate.

| Node (type) | p50 / peak MiB | MemoryLow | MemoryMax |
|---|---|---|---|
| ros2-master (`ros2_master`) | 42.6 / 46.9 | 48M | 128M |
| lerobot_follower (`feetech_servos`) | 49.7 / 50.4 | 56M | 128M |
| swerve_controller | 49.8 / 50.5 | 56M | 128M |
| filter_node | 47.6 / 48.2 | 48M | 128M |
| static_tf_publisher | 44.5 / 45.0 | 48M | 96M |
| bno055_imu | 53.6 / 57.6 | 56M | 128M |
| rplidar_a1 | 91.6 / 92.9 | 96M | 192M |
| rf2o_laser_odometry | 38.6 / 39.3 | 40M | 128M |
| rf2o_odom_relay | 47.6 / 48.2 | 48M | 128M |
| robot_localization_ekf | 46.9 / 47.7 | 48M | 128M |
| laser_filter | not measured (Python node since 2026-10-11; the C++ chain was 35.7 / 37.6) | 64M | 256M |
| slam_toolbox | 84.1 / 90.5 | 88M | 512M |
| nav2_bringup | 168.7 / 198.9 | 176M | 512M |
| map_guard | 63.7 / 66.4 | 64M | 160M |
| map_guard_cpp (not installed) | no data (same values as map_guard until measured) | 64M | 160M |
| gps_rtk_rover (`gps_rtk`) | 55.0 / 55.8 | 56M | 128M |
| poi_store | 53.8 / 54.3 | 56M | 128M |
| costmap_zones | not measured (initial values; re-measure in Mimir after 24 h) | 96M | 256M |
| overview_camera | estimate for two IMX219 (one camera measured 94.6 / 95.6; re-measure once both run) | 128M | 384M |
| gripper_uvc_camera (`uvc_camera`) | 66.7 / 68.2 | 72M | 256M |
| arducam_tof (disabled) | no data (estimate from uvc_camera plus the SDK and numpy; re-measure once enabled) | 64M | 192M |
| stereo_depth | no data (estimate: rclpy, numpy, OpenCV and the 480x360 SGBM buffers; re-measure once it runs) | 96M | 384M |
| stereo_depth_cpp | no data (same values as stereo_depth until measured) | 96M | 384M |
| jetson_bridge_cpp | no data (estimate: a ZeroMQ SUB thread, zstd decode of 16UC1 depth, five publishers; re-measure once it runs) | 32M | 256M |
| test_joint_api | 57.6 / 58.2 | 64M | 128M |
| mcp_server | 177.9 / 383.4 | 184M | 1G |
| claude_agent | 93.4 / 386.5 | 96M | 1G |
| web_ui | 106.4 / 253.1 | 112M | 512M |
| topic_scraper_api | 119.6 / 128.0 (the old 128M cap) | 120M | 256M |
| master2master (disabled) | no data | 64M | 128M |
| haptic_controller (disabled) | no data | 48M | 128M |
| realsense_d435i (retired) | no data | 128M | 512M |

The enabled nodes protect **1672 MiB** in total: under a quarter of the 8 GB Pi 5, which leaves about 6.3 GB for the
OS, page cache, monitoring and unprotected work.

### Host slices (`playbooks/tasks/cgroup_headroom.yml`)

Every client deploy (tags `config`, `always`) installs two slice drop-ins named `50-robot-headroom.conf` and reloads
systemd when either changed:

- `/etc/systemd/system/system.slice.d/`: `MemoryLow=` the sum of `memory_low` over the present and enabled nodes
  (1672M). A unit's `MemoryLow` only protects as far as every ancestor allocates protection, so without it the units'
  values would protect nothing.
- `/etc/systemd/system/user.slice.d/`: `CPUWeight=` / `IOWeight=` `ros2_user_slice_cpu_weight` /
  `ros2_user_slice_io_weight` (50). SSH sessions and the deploy's `become` steps (`uv sync`, git, apt) run in session
  scopes under `user.slice` (sudo keeps the caller's cgroup), so they lose every contention to `system.slice` (100).

`ros2_cgroup_headroom_enabled: false` in `group_vars/client.yml` removes both drop-ins again. The heavy builds (`npm`,
colcon) run in their own transient scope with `-p CPUWeight={{ ros2_build_cpu_weight }}` (20, `group_vars/all.yml`)
next to the `CPUQuota` cap, `IOWeight=10`, nice 19 and idle IO; the deploy also stops the nodes before them.
`monitoring.slice` (`roles/monitoring_agent`, see Monitoring) stays at CPU/IO weight 20 with its CPU and memory caps.

### Server

The server shares the template but keeps its quotas and caps (no tiers, no `memory_low`, no `MemoryHigh`):

| Node | CPUQuota | MemoryMax |
|---|---|---|
| ros2-master | 20% | 128M |
| lerobot_leader | 50% | 128M |
| topic_scraper_api (disabled) | 25% | 192M |
| gps_rtk_base | 25% | 64M |

### Proposing values from Prometheus

`scripts/propose_unit_limits.py` queries the Prometheus API of the server's Mimir (`http://server.ros2.lan:9009/prometheus`;
`container_cpu_usage_seconds_total`, `container_memory_working_set_bytes` for `/system.slice/ros2-*.service`, selected by
`host="client"`) and prints, per unit, the configured `CPUWeight` / `MemoryLow` / `MemoryMax`, the observed p99 CPU,
typical (p50) and peak working set, and the proposed `MemoryLow` (typical rounded up to 8 MiB), `MemoryMax` (2x the peak
rounded up to 16 MiB, a larger current cap kept) and `CPUWeight` / `IOWeight` (the configured tier, else the normal tier
500), followed by the sum of the proposed `MemoryLow` (what `system.slice` must allocate). Flags: `oom risk` (peak above
`MemoryMax`), `tight max` (`MemoryMax` under 2x the peak), `unprotected` (no `MemoryLow`, or one under the typical working
set), `throttle risk` (a `CPUQuota` is still set and p99 CPU reaches 90% of it). Options (URL, `host` label (client or
server, with the matching `group_vars_path`), window (default 24h, the whole Mimir retention), percentiles, the
`MemoryMax` factor, minimums, default weights, output `format: table|yaml`) live in `scripts/propose_unit_limits.yaml`;
there are no CLI flags.

```bash
uv run python scripts/propose_unit_limits.py
```

The script never edits `client.yml`: applying a proposal is a manual, reviewed edit of `memory_low` / `memory_max` /
the weights.

### POI store directory

`playbooks/tasks/poi_store_dir.yml` creates `/var/lib/ros2/poi` (owner `ansible_user`, mode `0755`) before `poi_store` is
deployed, in `deploy_nodes_client.yml` (tags `setup`, `poi_store`). It also ensures `/var/lib/ros2/poi/lookups` (same owner and mode, after the parent): the photo store, which poi_store
owns (the `photos:` block of its config in `group_vars/client.yml`: `store_dir`, retention limits `max_records` /
`max_bytes` / `max_age_days`, `max_image_bytes`, `prune_interval_s`, latched `summary_topic` `/poi/photos`). The node
keeps `poi.json` and the photo index there.

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
tools, and `/var/lib/ros2/objects` (owner `ansible_user`) for the legacy object memory file (`objects.json`, imported once into poi_store as object POIs and renamed `objects.json.migrated`), and `/var/lib/ros2/poi` plus
`/var/lib/ros2/poi/lookups` (owner `ansible_user`, `0755`) for the camera lookup store (every camera image the agent
received, `<id>.jpg` + `index.jsonl`; the retention limits are the `lookups:` block of the mcp_server config in
`group_vars/client.yml`). The unit
reads the token through `EnvironmentFile=`; the token never enters git. Fetch it on the dev machine with
`eval "$(./scripts/robot_mcp_token.sh)"` (see `nodes/mcp_server/README.md`).

### mcp_server guard rules

The `guard_rules:` block of the mcp_server `config:` in `group_vars/client.yml` holds the deployed safety rules (R1
`base_motion_arm_outside_footprint`, R2 `base_motion_carrying_without_object_footprint`, R3 `no_arm_retake_from_human`,
R4 `wrist_roll_needs_lifted_arm`, R5 `no_stretched_shoulder_overreach`); each carries a one-line hazard comment. Schema
and semantics: `nodes/mcp_server/README.md` "Guard rules". Next to it, `motion_queue.watch_rate_hz: 10` and
`watch_max_per_call: 8` configure `watch_constraints`. A node test (`test_guard_rules_deployed.py`) validates the block
with the node's own pydantic models and a root test (`test_mcp_server_config.py`) pins the five names and `dry_run`.

**dry_run -> enforce.** Every rule ships as `mode: dry_run`: a hit is logged (`guard_rule_would_refuse`) and counted in
`mcp_guard_rule_hits_total{rule,mode,phase,basis}`, nothing is refused. After about a week of counters, review each rule
(hits, `basis="unknown"` share, whether the hits were real hazards), then flip one rule at a time:

1. Edit that rule's `mode: dry_run` to `mode: enforce` in `group_vars/client.yml` (and update the root test if it
   should no longer be dry_run).
2. `uv run poe test-ansible`, commit, push.
3. `./scripts/deploy-nodes.sh client mcp_server 2>&1 | tee .logs/deploy-client.log` (restarts mcp_server; check the
   agent is idle first, a deploy kills a running agent session).
4. Watch the counters (`mode="enforce"`) and the agent's refusals; roll back by setting the rule back to `dry_run`.

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

## Monitoring

Metrics and logs of both Pis end up on the server, which stores them for 1 d and shows them in Grafana at **`http://server.ros2.lan/grafana/`** (folder **Robot**, no login). Three roles, all deployed only by the `monitoring` target:

| Role | Hosts | What it runs |
|---|---|---|
| `roles/monitoring_agent` | client and server | Grafana Alloy (metrics -> Mimir, journal -> Loki), `monitoring.slice`, the RPi throttling timer; on the robot it also removes the former local Prometheus and Grafana |
| `roles/web_proxy` | client and server | nginx on port 80: the robot site (web UI) or the server site (Grafana) |
| `roles/monitoring` | server | Grafana Mimir, Grafana Loki, Grafana and the dashboards |

**What runs where.**

| Service | Host | Package | Port | Role |
|---|---|---|---|---|
| `alloy.service` | both | `alloy` (apt.grafana.com) | 12345 (0.0.0.0: UI, `/metrics`) | Collects host (`prometheus.exporter.unix` incl. hwmon, thermal_zone, the textfile and the systemd collector), per-systemd-unit cgroups (`prometheus.exporter.cadvisor`, `docker_only = false`), itself, our ROS2 nodes' `/metrics` (robot only, job `ros2_nodes`) and the stack's own `/metrics` (server only: jobs `mimir`, `loki`, `grafana`), every 5 s; reads the journal. Remote-writes to Mimir, pushes logs to Loki, every series and stream labelled `host=client` or `host=server` |
| `rpi-throttled.timer` | both | `libraspberrypi-bin` (`vcgencmd`) | - | Every 5 s writes `vcgencmd get_throttled` to the textfile collector |
| `nginx.service` | both | `nginx` (Ubuntu) | 80 | Robot: `/` -> web UI (8080, WebSocket upgrade), `/grafana/...` -> `302` to `http://server.ros2.lan/grafana/...` (path and query kept, old bookmarks work). Server: `/` -> `302 /grafana/`, `/grafana/` -> Grafana (3000, WebSocket upgrade for Grafana Live). The distro default site is removed and `nginx -t` runs before every reload, on both |
| `mimir.service` | server | `mimir` 3.x (apt.grafana.com arm64 deb) | 9009 (0.0.0.0: push and Prometheus API), gRPC 9095 on 127.0.0.1 | Metrics store: monolithic `-target=all`, filesystem blocks under `/var/lib/mimir`, no tenants (`multitenancy_enabled: false`), one replica, in-memory rings, every internal address 127.0.0.1 |
| `loki.service` | server | `loki` 3.x (apt.grafana.com arm64 deb) | 3100 (0.0.0.0: push and query), gRPC 9096 on 127.0.0.1 | Log store: monolithic, filesystem chunks and tsdb v13 index under `/var/lib/loki`, `auth_enabled: false`, in-memory ring |
| `grafana-server.service` | server | `grafana` (apt.grafana.com) | 3000 (127.0.0.1) | Dashboards at `/grafana/` behind nginx (`serve_from_sub_path`); anonymous visitors are Editors, the admin account stays for settings |

Packages: apt.grafana.com publishes arm64 debs of `mimir` and `loki` (checked in `dists/stable/main/binary-arm64/Packages`: mimir 3.2.x, loki 3.7.x), so all four come from the same deb822 repo (`roles/monitoring_agent/tasks/grafana_repo.yml`, also included by `roles/monitoring`); the debs bring their own systemd units and users (`mimir`, `loki`). No release binaries are downloaded.

URLs: Grafana `http://server.ros2.lan/grafana/` (`http://client.ros2.lan/grafana/` redirects there), Mimir's Prometheus API `http://server.ros2.lan:9009/prometheus` (e.g. `/prometheus/api/v1/query`), Loki `http://server.ros2.lan:3100`, Alloy pipeline UIs `http://client.ros2.lan:12345` and `http://server.ros2.lan:12345`, the web UI `http://client.ros2.lan/` (8080 still works). Mimir and Loki have no auth: the LAN has no public address.

**Deploy (server first).** `./scripts/deploy-nodes.sh server monitoring`, then `./scripts/deploy-nodes.sh client monitoring` (each = `--tags monitoring`). The server goes first so the robot's Alloy has a destination (if not, it buffers until the server is up). `monitoring` is a non-node deploy target of both hosts (`non_node_targets` in `scripts/deploy-nodes.sh`), not a `ros2_nodes` entry. A node or phase deploy (`client web_ui`, `--all --tags config`) never runs the roles; an untagged `--all` does. A monitoring-only run starts and verifies no ROS node (`select_run.yml`, shared by both playbooks, gives it an empty node scope); on the client the agent idle guard still runs (`-e ros2_deploy_ignore_agent=true` skips it, the roles restart no robot node). Order in the server playbook: `monitoring_agent` (installs the slice), `web_proxy`, `monitoring` (its readiness check goes through nginx). A second run reports no changes.

**Settings.** Role defaults hold the agent-only host (the robot) and the stack; the per-host keys are monitoring-only keys in `group_vars/<host>.yml`: `monitoring_host` (the `host` label, required), on the client `monitoring_agent_remove_local_stack: true` and `monitoring_agent_node_targets`, on the server the 127.0.0.1 push URLs, `monitoring_agent_stack_targets` and the bigger slice. `web_proxy_site` (`robot` or `server`) picks the nginx site. `monitoring_agent_enabled` / `monitoring_enabled` false stop and disable Alloy / the stack (packages, data and config stay; nginx stays, it also fronts the web UI).

**Robot cleanup.** The robot ran Prometheus and Grafana itself until 2026-10-10. With `monitoring_agent_remove_local_stack` the agent role stops and disables `prometheus` and `grafana-server` (only while their units exist), purges both packages and removes `/etc/systemd/system/{prometheus,grafana-server}.service.d`, `/etc/grafana`, `/etc/prometheus`, `/etc/default/prometheus`, `/var/lib/prometheus` (the old 1 d of samples) and `/var/lib/grafana`. All state-based: the second run changes nothing. It runs before the slice shrinks.

**Retention.** Metrics 1 d: Mimir's compactor deletes blocks older than `compactor_blocks_retention_period: 24h`. Logs 1 d: Loki's `retention_period: 24h`, applied by its compactor (`retention_enabled: true`, `delete_request_store: filesystem`). Every target is collected every 5 s with a 4 s scrape timeout (the timeout must stay below the interval). Alloy reads up to 12 h of journal on its first start (`max_age`) and drops debug-level lines.

**Mimir sizing.** About 5-10k active series per host at 5 s, ~20k series and ~4k samples/s in all. `max_global_series_per_user` keeps the default 150k; `ingestion_rate` is 50k samples/s (default 10k) with a 500k burst so a WAL replay after an outage is not throttled into 429s. For the Pi4's RAM the ingester refuses series past 100k (`instance_limits.max_series`) and the querier runs 4 queries at a time (default 8).

**Buffering during a server outage.** Alloy writes every sample to its remote-write WAL (`/var/lib/alloy/data`) before sending and replays it when Mimir answers again; `loki.write` retries each log batch with backoff (10 retries, up to 5 min apart; after that the batch is dropped), and after an Alloy restart the journal is read on from the saved position. Mimir and Loki keep their own WALs (`/var/lib/mimir/tsdb`, `/var/lib/loki/wal`) across restarts.

**SD card.** The server's root (and so `/var/lib/mimir`, `/var/lib/loki`) is an SD card; the writes of Mimir's TSDB/WAL and compaction and of Loki's chunks wear it. Accepted by the user for the 1 d retention; the data stays under `/var/lib/{mimir,loki}`.

**Why Alloy pushes (remote write).** Alloy's embedded exporters are not ordinary HTTP targets, so remote write is the supported way to get them out; all collection, relabelling and cardinality cuts live in one file per host (`/etc/alloy/config.alloy`), and Mimir scrapes nothing.

**Cardinality cuts.** cAdvisor collects only CPU, memory, block IO and OOM events (`disabled_metrics` drops network, disk, percpu, process and the rest: network stats of host-netns services repeat the host interfaces per unit). A relabel keep rule keeps cgroup `id`s `/`, `/system.slice`, `/user.slice`, `/monitoring.slice`, `/system.slice/<unit>.service`, `/monitoring.slice/<unit>.service` and `/user.slice/user-<uid>.slice`, dropping scopes and sessions; `veth*`/`docker*`/`br-*` interfaces are dropped (cAdvisor) and excluded (netdev).

**Logs.** `loki.source.journal` labels each line with `unit` (`__journal__systemd_unit`, absent for kernel lines), `level` (`__journal_priority_keyword`: emerg, alert, crit, err, warning, notice, info, debug), `transport` (kernel lines carry OOM kills and USB resets) and `job="systemd-journal"`; `loki.process` drops `level="debug"`; `loki.write` adds `host`. Upstream ROS nodes (slam_toolbox, nav2, ...) have no in-process metrics but their logs are here. Example: `{host="client", unit="ros2-web_ui.service"}` or `sum by (unit) (rate({host="client", level=~"emerg|alert|crit|err"}[5m]))`.

**Admin password.** Generated on the server on the first deploy (`openssl rand -hex 24`, never on the controller or in git) into `/etc/grafana/admin-password` (0600 root:grafana). systemd reads it as root (`LoadCredential=admin_password:/etc/grafana/admin-password`) and Grafana gets it through `GF_SECURITY_ADMIN_PASSWORD__FILE=%d/admin_password`. Read it with `sudo cat /etc/grafana/admin-password` on the server; user `admin`. Grafana applies it when it creates its database (first start), so a later change of the file needs `grafana cli admin reset-admin-password`. Analytics, update checks, news and gravatar are off. Datasources (provisioned, read-only): `prometheus-robot` = Mimir at `http://127.0.0.1:9009/prometheus` (type prometheus, `prometheusType: Mimir`, default; the uid every dashboard uses) and `loki-robot` = Loki at `http://127.0.0.1:3100`. Only the Prometheus and Loki backend plugins run: the 11 other bundled datasources are in `[plugins] disable_plugins` (each runs its own backend process; 13 of them used 302 MB and got Grafana OOM-killed on the robot) and `preinstall_disabled` is on.

**Lost restarts.** A run that fails after writing a config loses its handlers, and the mimir and loki debs start their services with the package configs on install. After starting, each role therefore restarts any of its services whose config, slice drop-in or `monitoring.slice` is newer than the unit's start time (agent: alloy; stack: mimir, loki, grafana-server). The Loki config is checked with `loki -verify-config` before it is installed. The stack waits for Mimir `/ready`, Loki `/ready`, Grafana `/api/health` and `/grafana/api/health` through nginx; the agent for Alloy `/-/ready`; the proxy for its redirect (robot: `/grafana/d/robot-health?orgId=1` -> `302 http://server.ros2.lan/grafana/d/robot-health?orgId=1`, server: `/` -> `302 /grafana/`).

**Resource caps.** `/etc/systemd/system/monitoring.slice` (agent role, both hosts): `CPUWeight=20`, `IOWeight=20` (ROS units in `system.slice` have 100, so monitoring loses every contention), then per host:

| Host | Units in the slice | CPUQuota | MemoryHigh / MemoryMax | Per-unit MemoryMax |
|---|---|---|---|---|
| client | alloy, rpi-throttled | 40% (of one core) | 280M / 300M | alloy 250M |
| server | mimir, loki, grafana-server, alloy, rpi-throttled | 150% (of four cores) | 2000M / 2200M | mimir 700M, loki 400M, grafana-server 768M, alloy 250M |

The server's per-unit caps add up to 2118M, under the slice's hard total, leaving ~1.1 GB of the 3.8 GB for the ROS units and the page cache. Drop-ins `/etc/systemd/system/<unit>.service.d/10-monitoring-slice.conf` set `Slice=monitoring.slice`, `Nice=10`, `OOMScoreAdjust=500` (the OOM killer takes monitoring before ROS units), `Restart=on-failure` and the `MemoryMax`. Grafana keeps 768M (OOM-killed at 250M by first-start migrations and at 450M while browsing). Alloy runs as root (`User=root`) so cAdvisor can read every cgroup and `/proc` and the journal source every journal.

**Metric names produced** (Mimir's Prometheus API, every series with `host`):

- Host: `node_*` from the unix exporter, e.g. `node_cpu_seconds_total`, `node_load1/5/15`, `node_memory_MemAvailable_bytes`, `node_filesystem_avail_bytes`, `node_disk_read_bytes_total`/`node_disk_written_bytes_total`, `node_network_receive_bytes_total`/`node_network_transmit_bytes_total`, `node_thermal_zone_temp`, `node_hwmon_temp_celsius`.
- Per systemd unit (cAdvisor raw cgroup v2 handler; label `id="/system.slice/<unit>.service"`, monitoring units `id="/monitoring.slice/<unit>.service"`): `container_cpu_usage_seconds_total` (`cpu="total"`), `container_memory_working_set_bytes`, `container_memory_rss`, `container_fs_reads_bytes_total`, `container_fs_writes_bytes_total`, `container_spec_memory_limit_bytes` (0 when no MemoryMax), `container_spec_cpu_quota` / `container_spec_cpu_period` (quota only for units with a CPUQuota), `container_oom_events_total`.
- Throttling (textfile `rpi_throttled.prom`): `rpi_throttled_flags` (raw `get_throttled` value) and `rpi_throttled{bit=...}` (0/1) for `under_voltage_now`, `freq_capped_now`, `throttled_now`, `soft_temp_limit_now` (bits 0-3) and `under_voltage_occurred`, `freq_capped_occurred`, `throttled_occurred`, `soft_temp_limit_occurred` (bits 16-19). When `vcgencmd` fails the file is removed (no stale value).
- Stack (server): `alloy_*`, `cortex_*`/`mimir_*` (job `mimir`), `loki_*` (job `loki`), `grafana_*` (job `grafana`); the robot sends `alloy_*` only.

**Node targets.** The robot's Alloy scrapes our ROS2 nodes as one `prometheus.scrape "nodes"` (job `ros2_nodes`, 5 s interval, 4 s timeout, path `/metrics`) from `monitoring_agent_node_targets` in `group_vars/client.yml` (the server's list is empty). Every target carries a `node` label; `honor_labels = true` keeps the `node` label of the helper's `robot_node_info{node}` instead of renaming it to `exported_node`. A stopped node is `up{job="ros2_nodes"} == 0`. Nodes without an HTTP server serve `/metrics` through `shared/ros2_metrics` on 127.0.0.1 (`metrics_port` at the top level of the node's `config: |` block in `group_vars/client.yml`, or `METRICS_PORT` in the env of env-configured nodes); nodes with an HTTP server add `GET /metrics` to their existing port. `tests/test_node_metrics_wiring.py` checks that every enabled node below has a target on its configured port. Server nodes are not scraped (their units are in the cgroup, systemd and journal data).

| Node | Port | Configured by |
|---|---|---|
| `lerobot_follower` | 19101 | `metrics_port` |
| `filter_node` | 19102 | `metrics_port` |
| `bno055_imu` | 19103 | `metrics_port` |
| `gps_rtk_rover` | 19104 | `metrics_port` |
| `gripper_uvc_camera` | 19105 | env `METRICS_PORT` |
| `rplidar_a1` (scan supervisor) | 19106 | env `METRICS_PORT` |
| `swerve_controller` | 19107 | `metrics_port` |
| `rf2o_odom_relay` | 19108 | `metrics_port` |
| `poi_store` | 19109 | `metrics_port` |
| `map_guard` | 19110 | `metrics_port` |
| `map_guard_cpp` (live; `map_guard` is the disabled rollback, never both enabled) | 19110 | `metrics_port` |
| `costmap_zones` | 19111 | `metrics_port` |
| `arducam_tof` (disabled: add its target when enabling) | 19112 | `metrics_port` |
| `stereo_depth` | 19113 | `metrics_port` |
| `stereo_depth_cpp` (live; `stereo_depth` is the disabled rollback, never both enabled) | 19113 | `metrics_port` |
| `jetson_bridge_cpp` | 19114 | `metrics_port` |
| `jetson_depthd` (not a ROS node: depthd on the Jetson at `{{ jetson_link_jetson_ip }}:8090`, scraped by the Pi's Alloy because the Jetson has no route to the server) | 8090 | own `/metrics` (`depthd` HTTP API) |
| `web_ui` | 8080 | `http_port` (own server) |
| `test_joint_api` | 18080 | `port` (own server) |
| `topic_scraper_api` | 18100 | `port` (own server) |
| `mcp_server` | 18200 | `server.port` (own server) |
| `claude_agent` | 18300 | `http_port` (own server) |

Adding a node: give it a port (config `metrics_port` or env `METRICS_PORT`), add `{node, address, path}` to `monitoring_agent_node_targets` in `group_vars/client.yml`, deploy the node and `deploy-nodes.sh client monitoring`.

**Unit health (systemd collector).** The unix exporter adds `enable_collectors = ["systemd"]` with `enable_restarts` and `start_time`, limited by `unit_include` (`monitoring_agent_systemd_unit_include`) to `ros2-*.service`, `alloy`, `mimir`, `loki`, `grafana-server` and `nginx`: `node_systemd_unit_state{name,state}`, `node_systemd_service_restart_total{name}`, `node_systemd_unit_start_time_seconds{name}`. Upstream nodes (slam_toolbox, nav2, robot_localization, rf2o) and laser_filter have no in-process metrics: their health is this collector plus their cgroup `container_*` series and their journal lines.

**Validated with the real binaries** (`tests/test_monitoring_real_binaries.py`, runs when the images are present locally, never pulls): the rendered Mimir config starts `grafana/mimir:3.2.2` with only a loopback interface and reaches `/ready`; the Loki config passes `grafana/loki:3.7.8 -verify-config` and reaches `/ready`; both Alloy configs run in `grafana/alloy:latest` ("now listening", no "failed to evaluate"); both nginx sites pass `nginx:stable nginx -t` and the robot redirect keeps path and query; every PromQL query passes `promtool check rules` (`prom/prometheus:v2.45.3`) and every LogQL query Loki's `/loki/api/v1/format_query`.

### Dashboards

Every `*.json` in `roles/monitoring/files/` is copied (one `with_fileglob` task) into `/etc/grafana/dashboards` on the server and provisioned read-only into folder **Robot**. All use datasource uid `prometheus-robot` (the Loki panels `loki-robot`), refresh 5s, default range last 1 h, tag `robot`, and a link row to every dashboard tagged `robot`. Expressions use only the node metric contract, `node_*`, `container_*`, `rpi_throttled*`, `robot_node_*` and `up` (root tests). Every selector names a host: Robot resources and Robot health have a **Host** picker (`label_values(up, host)`, default `client`) and use `host="$host"`; the agent, SLAM and grasping dashboards show robot-only metrics and use `host="client"`.

| File | Title | Shows |
|---|---|---|
| `robot-resources.json` | Robot resources | (per host) Host CPU % by mode, load, memory, CPU temperature, throttling, root disk usage and IO, network, per-unit CPU vs CPUQuota and working set vs MemoryMax, per-unit RSS and disk IO, top 10 units by CPU, the monitoring units' own CPU, memory and disk writes |
| `robot-agent.json` | Robot agent | Agent busy timeline, instructions by status, instruction duration p50/p95, turns and tool uses by tool, seconds since activity, watchdog fires and session resets, tokens and cost, MCP tool call rate, latency p50/p95 and errors per tool, motion queue depth, motion step statuses, robot events by severity, claude_agent and mcp_server CPU/memory |
| `robot-slam.json` | Robot SLAM | slam_toolbox, nav2_bringup, robot_localization_ekf, rf2o_laser_odometry, laser_filter, rplidar_a1 unit CPU/memory; lidar scan rate, max gap and driver restarts; rf2o relay rate and rejections; swerve slip residual and odom rate; map updates and age; robot pose TF ok; nav goals by result and duration; IMU publish health and calibration; GPS fix quality, satellites, corrections and NTRIP throughput |
| `robot-grasping.json` | Robot grasping | Grasp plans by mode/outcome, attempts by mode/result, success ratio, gripper effort, grip profile uses, floor guard slowdowns, arm motion steps by status, tracking error p50/p95, grasp tool latencies (plan_grasp, grasp_object, release_object, move_arm_cartesian, set_gripper), servo load, temperature, torque and voltage per arm joint, servo bus failures |
| `robot-health.json` | Robot health | (per host) State of every ros2-* and monitoring unit, unit restarts in the window, node up/down and time since start, error counters per node (servo bus, IMU errors/reinits, GPS serial errors/NTRIP reconnects, camera drops/reopens, lidar restarts, web UI ws disconnects/stale topics, agent watchdog, MCP tool errors), throttling, temperature, disk and memory pressure; Loki: error-or-worse journal lines per unit (rate) and the recent error lines |

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

Every task of the deploy playbooks, the `ros2_node_deploy` / `ros2_node_verify` roles and `playbooks/tasks/*.yml` carries at least one of these phase tags (a root test enforces it). Each node's deploy step and node-specific setup carries the node name as well (`web_ui`, `mcp_server`, `overview_camera`, ...). A non-node deploy target carries only its own name: the `monitoring` roles of both hosts (see [Monitoring](#monitoring)) run for `--tags monitoring` or an untagged run, never for a phase tag.

| Tag | What it covers |
|-----|----------------|
| `sync` | Repo clone/update on the target (also `always`) |
| `apt` | ROS package sync and ONE batched install of the apt packages of all nodes in the run |
| `python` | uv venv, dependency sync (only when the hash changed) and the deployed-source stamp |
| `build` | web_ui npm steps and colcon source builds (stamp-gated, local sources rsynced and hashed), and the stop-before-build (not for local sources) |
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

**Agent idle guard (client).** A client deploy restarts nodes, which kills a running robot-agent session. The first pre-task, `playbooks/tasks/agent_idle_guard.yml` (tag `always`, so it runs for every `--tags` filter), reads `GET http://127.0.0.1:<http_port>/api/state` of `claude_agent` on the client (port from the node's config, default 18300) before anything can stop or restart a unit. It waits, polling every `ros2_agent_idle_poll_s` (15 s), until `busy` is false and `last_activity_at` is at least `ros2_agent_quiet_min` (5) minutes before the robot's own `now`, for at most `ros2_agent_idle_wait_min` (15) minutes, then fails with a message naming the override. It proceeds at once when `claude_agent` is absent or disabled, or the API does not answer; an older agent without `last_activity_at` is judged by `busy` alone. Override: `./scripts/deploy-nodes.sh client web_ui -e ros2_deploy_ignore_agent=true`. Defaults are in `group_vars/all.yml`.

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
