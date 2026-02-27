# Ansible

Ansible layout for provisioning Raspberry Pis (Server and Client) and deploying ROS2 nodes as systemd services. Target reader: mid-level Python dev with ROS2; some Ansible experience is enough.

## Layout

- **`inventory`** — Host groups `server` and `client`. Edit with your hostnames or IPs. `all:vars` can set `ansible_user`, `ansible_python_interpreter`.
- **`group_vars/`** — `all.yml`, `server.yml`, `client.yml` for group-specific variables.
- **`playbooks/`**
  - **`server.yml`**, **`client.yml`** — Provision: bootstrap Ubuntu 24.04 and install Docker (and Compose plugin). Run once per host (or when changing base setup).
  - **`deploy_nodes_server.yml`**, **`deploy_nodes_client.yml`** — Deploy: clone repo from GitHub (URL and revision in `group_vars/all.yml`), build each node’s container locally from the repo, deploy config, install systemd unit, enable/start or disable/stop. Containers are built on the node from the cloned repo (no pre-built image pull).
- **`roles/`**
  - **`common`** — Minimal bootstrap: Python3, git, sudo, basic packages.
  - **`docker`** — Docker CE + Docker Compose plugin on Ubuntu 24.04.
  - **`ros2_node_deploy`** — For each node: build image from repo (`build_context` path), create config dir, write config file, systemd unit, enable/start; or uninstall (stop, disable, remove unit and config dir). Handlers reload systemd and restart the node when config or unit changes.

## Node list and config (ros2_nodes)

Node list and per-type defaults live in **`group_vars/client.yml`** and **`group_vars/server.yml`**.

### Repo (group_vars/all.yml)

Deploy playbooks clone the repo on each node for local builds:

- **`ros2_repo_url`** — e.g. `https://github.com/szymonrychu/ros2-lerobot-swerve-platform`
- **`ros2_repo_revision`** — branch, tag, or commit (default `main`)
- **`ros2_repo_dest`** — path on the node (default `/opt/ros2-lerobot-swerve-platform`)

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
      joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
  - name: gripper_uvc_camera
    node_type: uvc_camera
    enabled: true
    env:
      - UVC_DEVICE=/dev/video0
      - UVC_TOPIC=/camera_0/image_raw
```

When you add, remove, or reconfigure ROS2 nodes (including in docker-compose or node source), update these vars and re-run the deploy playbook.

## Linting and testing

- **ansible-lint**: Run from the `ansible/` directory so `roles_path` resolves: `cd ansible && ansible-lint .`. From repo root: `poetry run poe lint-ansible`.
- **test-ansible**: Lint plus playbook syntax-check for all playbooks: `poetry run poe test-ansible` (runs `ansible-lint .` and `ansible-playbook -i inventory playbooks/<name>.yml --syntax-check` for each playbook).
- **Config**: `ansible/.ansible-lint` (profile, skip_list, warn_list). Pre-commit runs ansible-lint on staged `ansible/*.yml` files via a local hook that runs from `ansible/`. Install ansible-lint (e.g. `pip install ansible-lint` or `pipx install ansible-lint`) for the hook to work.

## Running playbooks

From the **`ansible/`** directory (so `ansible.cfg` and `inventory` are used):

**Provision (bootstrap + Docker):**
```bash
ansible-playbook -i inventory playbooks/server.yml -l server
ansible-playbook -i inventory playbooks/client.yml -l client
```

**Deploy nodes (systemd services):**
Build (or pull) the node images on the target (or push to a registry and pull there). Then:

```bash
ansible-playbook -i inventory playbooks/deploy_nodes_server.yml -l server
ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client
```

## After changing playbooks or roles

When you change playbooks, roles, or templates, re-run the relevant playbook so targets get the updates. There is no “container rebuild” inside Ansible; image builds are done via Docker (see repo [AGENTS.md](../AGENTS.md) and [README.md](../README.md)). When you change node **source code**, rebuild the container image and then re-run the deploy playbook if you need to pull/restart the service.
