# Key decisions and agent notes for this project

(Record important decisions, conventions, and Cursor/LLM context here.)

**Agents:** Always read [AGENTS.md](AGENTS.md) when resuming work, when joining the project, or when context is summarized; mandatory rules and "when to read" are defined there.

## Preparation phase

* **Tooling**: Python 3 via mise; dependencies via uv and Poetry (ROADMAP had "UVC" — corrected to uv). Linting and testing setup follows [hikvision-doorbell](https://github.com/szymonrychu/hikvision-doorbell): pre-commit, flake8, black, isort, vulture, autoflake, pytest, coverage.
* **ROS2 distro**: TBD (e.g. Humble on 22.04 or Jazzy on 24.04). To be recorded when chosen; Docker base images and Ansible will use the same.
* **Layout**: All nodes live under **`nodes/`**; **`shared/`** holds shared Python libraries. No `client/` or `server/` directories; deployment is via Ansible (clone repo on node, build containers locally).
* **Container registry**: Images are built and pushed to `https://harbor.szymonrichert.pl/containers/<node name>` (e.g. `harbor.szymonrichert.pl/containers/client-ros2-master:latest`). Ansible builds containers locally on each node from the cloned repo; image names use this registry path for consistency.
* **Commits**: After each phase/iteration commit, push to upstream. When only one agent works on the codebase, push only to `main` (no feature branches).
* **Leader/follower arms**: The **feetech_servos** node (configurable via YAML: namespace, joint_names as list of `{ name, id }` per joint—explicit servo ID, no assumption IDs start at 1) publishes `/<namespace>/joint_states` and subscribes `/<namespace>/joint_commands`. Use example configs `config/leader.yaml` and `config/follower.yaml` for leader (Server) and follower (Client). Topic layout remains `/leader/...` and `/follower/...` when so configured. Optional lerobot_teleop node on Client: subscribes `/leader/joint_states` (proxied by master2master), publishes `/follower/joint_commands`. master2master supports `type: JointState` for sensor_msgs/JointState relay.

## Linting and Python nodes

* **Per-node Poetry**: Each Python node has its own `pyproject.toml` and `poetry.lock` (master2master, uvc_camera, feetech_servos, lerobot_teleop). Run `poetry install` and `poetry run poe lint` / `poetry run poe lint-fix` inside each node directory. From repo root: `poetry run poe lint-nodes` or `./scripts/lint-all-nodes.sh` to lint all nodes. Root `poe lint` covers only `tests`, `shared` (no `src` package).
* **Docker**: Node images use Poetry (no requirements.txt); Dockerfiles install poetry in a venv, then `poetry install --no-dev`.
* **Pre-commit**: Pre-commit only runs on files known to git. If nothing is tracked yet (e.g. before first commit), run `./scripts/run-pre-commit.sh` — it stages all files and runs all hooks. Then run `pre-commit install` for hooks on every commit.
* **Python conventions**: Prefer established libraries (e.g. Feetech servo library, pydantic for config); prefer file config over CLI; write unit tests. Per senior review: constants at top of file in SCREAMING_SNAKE_CASE; fewer private functions; imports at top, no try/except on imports; docstrings must include in/out variables with their types.
* **Commits**: Create a commit after each phase/iteration that passes; use semantic commit messages (feat:, fix:, docs:, etc.); the agent performs these commits.
* **Ansible and ROS2 nodes**: When ROS2 nodes or their configuration change, update Ansible so it stays in sync. Node list and config live in `group_vars/client.yml` and `group_vars/server.yml` under `ros2_nodes` and `ros2_node_type_defaults`. Schema: each node has `name`, `node_type` (maps to image/config via type defaults), `present` (default true; false triggers uninstall), `enabled` (default true; systemd enable/start vs disable/stop), optional `config` (string, node’s config file content), optional `env` (list of KEY=VAL). Ansible clones the repo from https://github.com/szymonrychu/ros2-lerobot-swerve-platform (revision configurable, default main) to a node directory, builds each node container from the repo locally (build_context per type, e.g. nodes/ros2_master), then installs or uninstalls the systemd unit and config dir per node.
* **Ansible lint and tests**: ansible-lint (run from `ansible/`: `ansible-lint .`) and playbook `--syntax-check` are part of the workflow. From repo root: `poetry run poe lint-ansible`, `poetry run poe test-ansible`. Config: `ansible/.ansible-lint` (profile, skip_list, warn_list). Pre-commit runs ansible-lint via a local hook that `cd`s into `ansible/` so roles_path resolves. Install ansible-lint (e.g. `pip install ansible-lint` or `pipx install ansible-lint`) for the hook to work.
* **Ansible network and hostname**: Roles `network` (netplan static IP: address, gateway, optional nameservers; detects primary interface) and `hostname` (hostnamectl + /etc/hostname, /etc/hosts). Run during provision when `network_address`, `network_gateway`, and/or `hostname` are set in group_vars or host_vars. ROS2 units: Server ros2_master uses `ROS_LOCALHOST_ONLY=0` (bind all); Server other nodes and all Client nodes use `ROS_LOCALHOST_ONLY=1`. Client master2master receives `ROS2_SERVER_HOST` (set `ros2_server_host` in client.yml) for connecting to Server’s ROS2 master.

## Repository upgrade (runtime and deploy)

* **master2master**: Single-process relay (one rclpy node, one MultiThreadedExecutor); no per-thread init/shutdown. Config validation: invalid direction/type raise `ConfigError`; topic names normalized (leading slash). Run from repo root with `mise exec --` when poetry is not on PATH (see README).
* **UVC camera bridge**: Real frame capture via OpenCV; publishes `sensor_msgs/Image` (bgr8). Env: `UVC_DEVICE`, `UVC_TOPIC`, `UVC_FRAME_ID`. Dependency: `opencv-python-headless`.
* **Teleop**: Configurable topics via `TELEOP_LEADER_JOINT_STATES_TOPIC`, `TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC`; explicit QoS profile.
* **Feetech**: Config rejects namespace containing `/` and empty joint names; set_servo_id catches OSError/ValueError/RuntimeError from ListServos.
* **Ansible**: `ros2_node_deploy` validates required vars (node_name, node_image, node_build_context, repo_dest); systemd unit has RestartSec=5, TimeoutStopSec=30, docker stop -t 25. Server node name for leader arm: `lerobot_leader` (group_vars/server.yml).

## Rules alignment and testability (post-review)

* **master2master**: Imports (`signal`, `rclpy`) at top of `proxy.py`; docstring Args fixed for `run_all_relays`. Extra tests: `normalize_topic` edge cases, `load_config` from file, `parse_rule_entry` invalid type.
* **UVC camera**: Config split to `config.py` (no ROS/OpenCV) for testability; `get_config` strips topic/frame_id and falls back to defaults. Tests in `nodes/bridges/uvc_camera/tests/test_config.py`; pytest `pythonpath = ["."]` so `config` resolves.
* **Lerobot teleop**: Config split to `config.py` (no ROS deps); tests in `nodes/lerobot_teleop/tests/test_config.py`; same pythonpath pattern.
* **Feetech**: Tests for `load_config_from_env`; `BridgeConfig` and master2master `TopicRule` docstrings (Attributes). Shared `clamp` tests: equal bounds, at bounds.
* **Node tests**: uvc_camera and lerobot_teleop need `poetry install --no-root` (or `poetry lock` then install) before first run; run from node dir with `poetry run poe test`.
