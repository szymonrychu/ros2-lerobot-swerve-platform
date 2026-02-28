# Agent rules (Cursor / IDE)

**When to read this file (mandatory):** Read [AGENTS.md](AGENTS.md) and [MEMORY.md](MEMORY.md) when resuming work on this project, when a new agent joins the project, and when the agent's context is summarized. These files are the single source of truth for conventions and mandatory behaviour.

Use this file and [MEMORY.md](MEMORY.md) when working in this repo.

## Tech stack

* ROS2 (distro TBD — e.g. Humble/Jazzy)
* Docker and docker-compose
* Ansible
* Ubuntu 24.04
* Client: Raspberry Pi 5
* Server / dev: Intel x64 or MacBook M4

## Structure

* Monorepo with **`ansible/`**, **`nodes/`**, and **`shared/`**
* All node code and Dockerfiles live under **`nodes/`**. **`shared/`** holds shared Python libraries used by multiple nodes
* Ansible role(s) and runbook for provisioning and deployment

## Conventions

* Add new node source under **`nodes/`**; add the node type and build context to Ansible `group_vars` and the deploy role so containers are built from the repo on each node
* Device access: use docker-compose device mappings for `/dev/` where needed
* Prefer one container per node
* **Always use Python 3 type hints** for all new and modified Python code (functions, method parameters and returns, class attributes where appropriate)
* **Extend unit tests as the project grows** — add or update tests when adding functionality, refactoring, or fixing bugs; keep test coverage and test quality in step with the codebase

### Python conventions

* **Use the right libraries**: Prefer established libraries even when they seem more complex than the minimum (e.g. numpy for non-trivial computation; ROS2 logging in ROS2 nodes; standard `logging` for non-ROS2 code; an existing Feetech servo library over a custom protocol implementation; **pydantic** for config loading/validation such as master2master settings instead of ad-hoc dict/YAML parsing). Implement only functions that are missing in the chosen library.
* **Prefer file configuration over CLI**: Use config files (YAML/JSON) for node and script options unless the user explicitly asks for command-line arguments.
* **Fewer private functions**: Prefer module-level or non-underscore-prefixed functions; use "private" (leading-underscore) helpers only when there is a clear need.
* **Constants**: Group constant variables at the upper portion of each Python file; name them in `SCREAMING_SNAKE_CASE`.
* **Imports**: Keep all imports at the top of the file; do not wrap imports in try/except—let import errors fail fast.
* **Docstrings**: Document input and output (parameters and return) in docstrings, including their types (aligned with type hints).
* **Unit tests**: Write unit tests for each new function; extend the test suite as the codebase grows.

## Pointers

* Full architecture and component list: [README.md](README.md)
* MVP scope and streams: [ROADMAP.md](ROADMAP.md)

## Scope (MVP)

* **Client stream**: All robot nodes (ROS2 server, UVC cameras, RealSense D435i, RPLidar-A1, Lerobot SO-101 follower, BNO095 IMU, GPS-RTK LC29H(DA), Nav2 + SLAM), Ansible + systemd
* **Server stream**: ROS2 server, GPS-RTK LC29H(BS), Lerobot SO-101 leader
* **Shared stream**: Leader–follower teleop over ROS2/local network; ROS2 master-to-master on all MVP topics
* OpenVLA and other non-MVP features are out of scope for MVP

## AI rules (mandatory)

* Write basic unit tests for new functionality.
* Run linters and unit tests before considering work done. Root lint: `poetry run poe lint` (tests, shared). Lint all Python nodes: `poetry run poe lint-nodes` or `./scripts/lint-all-nodes.sh`. Each node has its own Poetry and `poe lint` in its directory. **Ansible:** run `poetry run poe lint-ansible` (or `cd ansible && ansible-lint .`) and `poetry run poe test-ansible` (ansible-lint + playbook `--syntax-check`). When changing Ansible code, run these and fix any reported issues.
* **Always commit and push:** Use semantic commit messages (e.g. `feat:`, `fix:`, `docs:`). **Create a commit after each phase/iteration that passes**; the agent performs these commits and follows the semantic-commits convention. **Always push to the upstream repository after each such commit.** When only one agent works on the codebase, do not use feature branches—push only to `main`.
* **Ansible and ROS2 nodes:** Whenever ROS2 nodes or their configuration change, update the Ansible code so it stays in sync. Ansible must support deploying, enabling, and disabling nodes by name and configuring them via the `ros2_nodes` schema (name, node_type, present, enabled, config). See `group_vars/client.yml` and `group_vars/server.yml` and the `ros2_node_deploy` role. Keep Ansible lint and tests passing: run `poe lint-ansible` and `poe test-ansible`; fix or document any skip/warn in `ansible/.ansible-lint`. **When changing Ansible or ROS2 node deployment, verify with:** `ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client` and `ansible-playbook -i inventory playbooks/deploy_nodes_server.yml -l server` (run from `ansible/`).
* Prefer working on multiple branches when possible.
* Ask the developer very detailed questions about everything when unclear.
* Never hallucinate — prefer asking or citing sources over inventing.
* **Containers and source changes:** Whenever source code that is used inside a container is edited, rebuild that container so the image reflects the change. Remember each container's purpose and its requirements (env, devices, config). Iterate with fixes until all mandatory functionality for that node works; do not leave broken or half-working behaviour.
* **Documentation:** Document work in README.md files: one per node under `nodes/` (e.g. `nodes/master2master/README.md`, `nodes/bridges/feetech_servos/README.md`), one for [ansible/](ansible/README.md), and one for other key directories (`nodes/README.md`, `shared/README.md`, **[tests/README.md](tests/README.md)**). Target audience: mid-level Python developer with ROS/ROS2 experience. Keep READMEs up to date whenever something in that area changes.
* **Tests README:** Maintain [tests/README.md](tests/README.md): list each test file, briefly describe what it covers, and document each test (or test group). Update this README when adding, removing, or changing tests so the test suite stays easy to navigate.

## Memory and decisions

Use [MEMORY.md](MEMORY.md) to record key project decisions and Cursor's (LLM's) notes about the project. When you make or learn important decisions, conventions, or context, append or update MEMORY.md so future sessions and humans have a single place to look.
