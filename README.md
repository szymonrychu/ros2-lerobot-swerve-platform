# ROS2 Lerobot Swerve-Drive Platform

## Development setup

* **Python**: Managed with [mise](https://mise.jdx.dev/). Run `mise install` in the repo to install Python 3.12, uv, and Poetry.
* **Dependencies**: Root package (tests, shared): `mise exec -- poetry install`. Each Python node under `nodes/` has its own Poetry project: run `poetry install` (or `mise exec -- poetry install`) inside `nodes/master2master`, `nodes/bridges/uvc_camera`, `nodes/bridges/feetech_servos`, `nodes/lerobot_teleop`.
* **Linters**: Root (tests, shared): `poetry run poe lint` / `poetry run poe lint-fix`. All Python nodes: `poetry run poe lint-nodes` or `./scripts/lint-all-nodes.sh`. Ansible: `poetry run poe lint-ansible` or `./scripts/lint-ansible.sh` (requires ansible-lint installed, e.g. `pip install ansible-lint`). Each node can be linted on its own: `cd nodes/<node> && poetry run poe lint`. If `poetry` is not on your PATH (e.g. managed by mise), run commands as `mise exec -- poetry run poe lint` (and similarly for other targets). Docker builds for nodes use Poetry (no requirements.txt).
* **Pre-commit**: Run `pre-commit install` once so hooks run on every commit. To run all hooks on the whole repo (full check or before first commit): `./scripts/run-pre-commit.sh` — it stages all files if none are tracked so that hooks have files to check. Manual run: `pre-commit run --all-files` (requires tracked files).
* **Tests**: `poetry run pytest` and `poetry run coverage run -m pytest` for the root package.

## Monorepo layout

* **`nodes/`** — All ROS2 node source and Dockerfiles (ros2_master, master2master, bridges, lerobot_teleop, etc.).
* **`shared/`** — Shared Python libraries used by multiple nodes.
* **`ansible/`** — Provisioning and deployment: playbooks and roles for Ubuntu 24.04 on Raspberry Pis, Docker install, clone of this repo on each node, local container build, and deploying nodes as systemd services.

## Ansible

* **Provision** (bootstrap + Docker on Ubuntu 24.04): from `ansible/`, run `ansible-playbook -i inventory playbooks/server.yml -l server` or `playbooks/client.yml -l client`. Edit `inventory` with your hostnames or IPs.
* **Deploy nodes** (clone repo, build containers locally, systemd services): run `ansible-playbook -i inventory playbooks/deploy_nodes_server.yml -l server` or `deploy_nodes_client.yml -l client`. Ansible clones the repo from GitHub (revision configurable, default `main`), builds each node's container from the repo on the node, deploys config to `/etc/ros2-nodes/<node_name>/`, and manages systemd services. Container images use the registry `https://harbor.szymonrichert.pl/containers/<node name>`.

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
* Per-node and key-directory READMEs: `nodes/README.md`, `nodes/master2master/README.md`, `nodes/bridges/README.md`, `shared/README.md`, `ansible/README.md`, `tests/README.md`, etc.

Repository: `ros2-lerobot-swerve-platform`
