# ROS2 Lerobot Swerve-Drive Platform

## Development setup

* **Python**: Managed with [mise](https://mise.jdx.dev/). Run `mise install` in the repo to install Python 3.12, uv, and Poetry.
* **Dependencies**: Root package (tests, shared): `mise exec -- poetry install`. Each Python node under `nodes/` has its own Poetry project: run `poetry install` (or `mise exec -- poetry install`) inside `nodes/master2master`, `nodes/bridges/uvc_camera`, `nodes/bridges/lerobot_joints`, `nodes/lerobot_teleop`.
* **Linters**: Root (tests, shared): `poetry run poe lint` / `poetry run poe lint-fix`. All Python nodes: `poetry run poe lint-nodes` or `./scripts/lint-all-nodes.sh`. Each node can be linted on its own: `cd nodes/<node> && poetry run poe lint`. Docker builds for nodes use Poetry (no requirements.txt).
* **Pre-commit**: Run `pre-commit install` once so hooks run on every commit. To run all hooks on the whole repo (full check or before first commit): `./scripts/run-pre-commit.sh` — it stages all files if none are tracked so that hooks have files to check. Manual run: `pre-commit run --all-files` (requires tracked files).
* **Tests**: `poetry run pytest` and `poetry run coverage run -m pytest` for the root package.

## Monorepo layout

* **`client/`** — Client Raspberry Pi: **only** `docker-compose.yml` (and optional config). No node source here; builds reference `../nodes/`.
* **`server/`** — Server Raspberry Pi: **only** `docker-compose.yml` (and optional config). No node source here; builds reference `../nodes/`.
* **`nodes/`** — All ROS2 node source and Dockerfiles (ros2_master, master2master, bridges, lerobot_teleop, etc.).
* **`shared/`** — Shared Python libraries used by multiple nodes.
* **`ansible/`** — Provisioning and deployment: playbooks and roles for Ubuntu 24.04 on Raspberry Pis, Docker install, and deploying nodes as systemd services.

## Docker (build and run)

* **Server**: `cd server && docker compose build && docker compose up -d ros2-master`. Optional profiles: `--profile gps`, `--profile lerobot-leader`.
* **Client**: `cd client && docker compose build && docker compose up -d ros2-master master2master`. Optional profiles: `--profile uvc`, `--profile realsense`, `--profile lidar`, `--profile imu`, `--profile gps`, `--profile lerobot`, `--profile swerve`. Map devices in `docker-compose.yml` (e.g. `device: /dev/video0`) when running on hardware.

## Ansible

* **Provision** (bootstrap + Docker on Ubuntu 24.04): from `ansible/`, run `ansible-playbook -i inventory playbooks/server.yml -l server` or `playbooks/client.yml -l client`. Edit `inventory` with your hostnames or IPs.
* **Deploy nodes** (systemd services): build images on the target or push to a registry, then run `ansible-playbook -i inventory playbooks/deploy_nodes_server.yml -l server` or `playbooks/deploy_nodes_client.yml -l client`. Node config (e.g. master2master topics) is deployed to `/etc/ros2-nodes/<node_name>/` and mounted into the container; services restart when config or image changes.

## Development roadmap

See [ROADMAP.md](ROADMAP.md) for detailed roadmap plans.

## Hardware components

### Server Raspberry PI 4b

* Lerobot SO-101 leader arm
* GPS-RTK LC29H(BS)

### Client Raspberry PI 5b

* Lerobot SO-101 follower arm
* Swerve platform
  * separate serial servo bus adapter
  * 4 pairs of Feetech ST3215 12V servos (one for rotating wheel, second for steering wheel)
* GPS-RTK LC29H(DA)
* BNO095 IMU
* RPLidar-A1
* Intel RealSense D435i
* 2x Arducam B0454 - 5MP OV5648 USB Camera

### Server offloading AI tasks

Yet to be determined x64 based system with LLM/VLA capabilty.

### On-Site controller

SteamDeck running Ubuntu, equipped with ROS2 and Proton based GUI controlling robot.

## Documentation

* [README.md](README.md) — architecture and component list (this file)
* [ROADMAP.md](ROADMAP.md) — MVP scope and roadmap streams
* [AGENTS.md](AGENTS.md) — Cursor/agent rules and conventions (**read when resuming work, when joining the project, or when context is summarized**)
* [MEMORY.md](MEMORY.md) — key decisions and agent notes for this project
* Per-node and key-directory READMEs: `server/README.md`, `client/README.md`, `nodes/README.md`, `nodes/master2master/README.md`, `nodes/bridges/README.md`, `shared/README.md`, `ansible/README.md`, `tests/README.md`, etc.

Repository: `ros2-lerobot-sverve-platform`
