# Ansible

Ansible layout for provisioning Raspberry Pis (Server and Client) and deploying ROS2 nodes as systemd services. Target reader: mid-level Python dev with ROS2; some Ansible experience is enough.

## Layout

- **`inventory`** — Host groups `server` and `client`. Edit with your hostnames or IPs. `all:vars` can set `ansible_user`, `ansible_python_interpreter`.
- **`group_vars/`** — `all.yml`, `server.yml`, `client.yml` for group-specific variables.
- **`site.yml`** — Full site: provision all hosts, then deploy ROS2 nodes on server and client (includes playbooks below). Run `ansible-playbook -i inventory site.yml`.
- **`playbooks/`**
  - **`server.yml`**, **`client.yml`** — Provision: bootstrap Ubuntu 24.04, optional network (netplan) and hostname, then Docker (and Compose plugin). Run once per host (or when changing base setup). Set `network_address`, `network_gateway`, and optionally `hostname`, `network_nameservers` in group_vars or host_vars to apply static IP and hostname.
  - **`deploy_monitoring.yml`** — Deploy only the monitoring stack (Alloy, Mimir, Loki, Grafana) on selected hosts. Run `ansible-playbook -i inventory playbooks/deploy_monitoring.yml`; use `-l server` or `-l client` to limit to a host group.
  - **`deploy_nodes_server.yml`**, **`deploy_nodes_client.yml`** — Deploy: clone repo from GitHub (URL and revision in `group_vars/all.yml`), build or pull each node’s container, deploy config, install systemd unit, enable/start or disable/stop. Containers are built on each Pi from the cloned repo (no CI/registry by default). Set `node_build_on_controller: true` when you have a registry to build on the controller and pull on nodes.
- **`roles/`**
  - **`common`** — Minimal bootstrap: Python3, git, sudo, basic packages.
  - **`network`** — Netplan: primary interface gets static IP (ethernet or wlan, auto-detected); other interfaces DHCP; IPv6 disabled. Runs when `network_address` and `network_gateway` are set; for primary WiFi set `network_wifi_ssid` (and optionally `network_wifi_password`).
  - **`hostname`** — Set system hostname (hostnamectl, `/etc/hostname`, `127.0.1.1` in `/etc/hosts`). Runs only when `hostname` is set.
  - **`docker`** — Docker CE + Docker Compose plugin on Ubuntu 24.04.
  - **`monitoring`** — Alloy, Mimir, Loki, Grafana in one role: docker-compose + systemd. Alloy collects Docker container logs and cAdvisor metrics (scrape interval configurable, default 1s), sends metrics to Mimir and logs to Loki. Mimir and Loki use local filesystem with configurable retention (default 8h). Grafana listens on a configurable port (default 8080), admin/admin, with Mimir and Loki as datasources. Applied to **all hosts** (server and client); each host runs its own stack (see `site.yml`). Variables: `monitoring_mimir_retention`, `monitoring_loki_retention`, `monitoring_alloy_scrape_interval`, `monitoring_grafana_http_port`, `monitoring_data_dir`; image tags in role defaults.
  - **`ros2_node_deploy`** — For each node: build image from repo (`build_context` path), create config dir, write config file, systemd unit, enable/start; or uninstall (stop, disable, remove unit and config dir). Handlers reload systemd and restart the node when config or unit changes.
  - **`ros2_node_verify`** — Runs after all nodes are deployed: waits for services to settle, checks each present+enabled node’s systemd unit is active, waits again, then re-checks (stability). Used by `deploy_nodes_server.yml` and `deploy_nodes_client.yml`. Variables: `ros2_node_verify_settle_seconds` (default 10), `ros2_node_verify_stable_seconds` (default 5).

## Node list and config (ros2_nodes)

Node list and per-type defaults live in **`group_vars/client.yml`** and **`group_vars/server.yml`**.

### Repo (group_vars/all.yml)

Deploy playbooks clone the repo on each node for local builds:

- **`ros2_repo_url`** — e.g. `https://github.com/szymonrychu/ros2-lerobot-swerve-platform`
- **`ros2_repo_revision`** — branch, tag, or commit (default `main`)
- **`ros2_repo_dest`** — path on the node (default `/opt/ros2-lerobot-swerve-platform`)
- **`node_build_on_controller`** — (default `false`) When false, images are built on each Raspberry Pi from the cloned repo. Set `true` when you have a registry/CI: controller builds and pushes, nodes pull (requires `docker_registry_username` / `docker_registry_password` on nodes and `docker login` on controller).
- **`ros2_repo_root`** — (default `{{ playbook_dir }}/../..`) Path to repo root on the controller when building images; used as `docker build` context (playbook lives in `ansible/playbooks/`).

### ros2_node_type_defaults

Maps each **node_type** to image (registry path), **build_context** (path relative to repo root for `docker build`), and optional config path and env. Images use the registry `https://harbor.szymonrichert.pl/containers/<name>`.

```yaml
ros2_node_type_defaults:
  ros2_master:
    image: harbor.szymonrichert.pl/containers/client-ros2-master:latest
    build_context: nodes/ros2_master
  feetech_servos:
    image: harbor.szymonrichert.pl/containers/client-feetech-servos-follower:latest
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
| `extra_args` | no     | `''`    | Extra arguments passed to `docker run`. |

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

When you add, remove, or reconfigure ROS2 nodes (including in docker-compose or node source), update these vars and re-run the deploy playbook.

### Joint command topic flow (client)

Leader joint states are relayed and filtered before reaching the follower feetech bridge:

- **Server:** `lerobot_leader` (feetech) publishes `/leader/joint_states`.
- **Client:** `master2master` subscribes to `/leader/joint_states` and republishes to **`/filter/input_joint_updates`**.
- **Client:** `test_joint_api` can also publish to `/filter/input_joint_updates` (same path as master2master; use for testing, gripper-only).
- **Client:** `filter_node` subscribes to `/filter/input_joint_updates`, runs the configured algorithm (e.g. Kalman), and publishes to **`/follower/joint_commands`**.
- **Client:** `lerobot_follower` (feetech) subscribes to `/follower/joint_commands` and drives the servos.

So both master2master and the test API feed the same filter → feetech chain.

### IMU (client)

The **bno095_imu** node runs on the client and publishes `sensor_msgs/Imu` on `/imu/data` (configurable) with orientation, angular velocity, linear acceleration, and full covariance matrices for use with the Navigation stack (Nav2). It reads a BNO085/BNO095 over I2C; the container is given access to the I2C device (e.g. `--device=/dev/i2c-1:/dev/i2c-1`). Config: topic, frame_id, publish_hz, i2c_bus, and covariance values (see `nodes/bno095_imu/README.md`).

## Network and hostname (provision)

The **network** role sets a static IP on the primary interface (ethernet or WiFi, auto-detected), sets all other interfaces to DHCP, and disables IPv6 in netplan.

In **`group_vars/server.yml`** or **`group_vars/client.yml`** (or host_vars), set:

- **`network_address`** — Static IP in CIDR (e.g. `192.168.1.10/24`). Required.
- **`network_gateway`** — Default gateway (e.g. `192.168.1.1`). Required.
- **`network_nameservers`** — Optional list (e.g. `["8.8.8.8", "8.8.4.4"]`).
- **`network_interface`** — Optional. Default: primary IPv4 interface (ethernet or wlan).
- When the primary interface is **WiFi** (e.g. `wlan0`): **`network_wifi_ssid`** (required), **`network_wifi_password`** (optional).
- **`hostname`** — Short hostname (e.g. `server-rpi4`). Optional; when set, the hostname role runs.

The network role writes a netplan file under `/etc/netplan/` and runs `netplan apply`. The **hostname** role runs `hostnamectl set-hostname` and updates `/etc/hostname` and `/etc/hosts`.

## ROS2 network setup (bind and localhost)

Containers get env vars so that:

- **Server:** `ros2-master` and `lerobot_leader` use **`ROS_LOCALHOST_ONLY=0`** and run with `--network host` so leader topics are discoverable to the Client for cross-host relay.
- **Client:** `master2master` uses **`ROS_LOCALHOST_ONLY=0`**, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, and `ROS_STATIC_PEERS=<server-ip>` (from `group_vars/client.yml`) so it can discover and relay server topics.
- **Local-only nodes:** Other client nodes (e.g. follower, UVC) can stay localhost-oriented while still using `--network host` for consistent ROS graph visibility and device access.

## Linting and testing

- **ansible-lint**: Run from the `ansible/` directory so `roles_path` resolves: `cd ansible && ansible-lint .`. From repo root: `poetry run poe lint-ansible`.
- **test-ansible**: Lint plus playbook syntax-check for all playbooks: `poetry run poe test-ansible` (runs `ansible-lint .` and `ansible-playbook -i inventory playbooks/<name>.yml --syntax-check` for each playbook).
- **Config**: `ansible/.ansible-lint` (profile, skip_list, warn_list). Pre-commit runs ansible-lint on staged `ansible/*.yml` files via a local hook that runs from `ansible/`. Install ansible-lint (e.g. `pip install ansible-lint` or `pipx install ansible-lint`) for the hook to work.

## Running playbooks

From the **`ansible/`** directory (so `ansible.cfg` and `inventory` are used):

**Full site (provision + deploy nodes):**
```bash
ansible-playbook -i inventory site.yml
```

**Provision only (bootstrap + Docker):**
```bash
ansible-playbook -i inventory playbooks/server.yml -l server
ansible-playbook -i inventory playbooks/client.yml -l client
```

**Deploy nodes only (systemd services):**
```bash
ansible-playbook -i inventory playbooks/deploy_nodes_server.yml -l server
ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client
```

## After changing playbooks or roles

When you change playbooks, roles, or templates, re-run the relevant playbook so targets get the updates. There is no “container rebuild” inside Ansible; image builds are done via Docker (see repo [AGENTS.md](../AGENTS.md) and [README.md](../README.md)). When you change node **source code**, rebuild the container image and then re-run the deploy playbook if you need to pull/restart the service.
