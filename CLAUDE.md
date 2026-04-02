# CLAUDE.md — Project Conventions

---

## Tech Stack

- ROS2 Jazzy (Ubuntu 24.04)
- Ansible
- Ubuntu 24.04
- Client: Raspberry Pi 5
- Server: Raspberry Pi 4b
- Dev: MacBook M4

---

## Repository Structure

- Monorepo with **`ansible/`**, **`nodes/`**, **`shared/`**, **`tests/`**, **`scripts/`**, **`docs/`**, **`client/`**, and **`server/`** at root level.
- All node code lives under **`nodes/`**. **`shared/`** holds shared Python libraries used by multiple nodes. **`client/`** and **`server/`** contain host-specific non-node config. **`scripts/`** has operational tooling. **`tests/`** has root-level tests. **`docs/`** has project documentation.

## Python Conventions

- **Always use Python 3 type hints** for all new and modified Python code (functions, method parameters and returns, class attributes where appropriate).

### Libraries

- **Use the right libraries**: Prefer established libraries even when they seem more complex than the minimum (e.g. numpy for non-trivial computation; ROS2 logging in ROS2 nodes; standard `logging` for non-ROS2 code; an existing Feetech servo library over a custom protocol implementation; **pydantic** for config loading/validation such as master2master settings instead of ad-hoc dict/YAML parsing). Implement only functions that are missing in the chosen library.
- Use established libraries and frameworks for non-trivial logic (parsing, crypto, networking, protocols, math, etc.) instead of writing your own.
- Only implement custom code when no suitable library exists or when the library cannot meet the requirement.
- When in doubt, search for an existing solution (PyPI, ROS2 packages, stdlib) before implementing from scratch.

### Config files

- **Prefer file configuration over CLI**: Use config files (YAML/JSON) for node and script options unless the user explicitly asks for command-line arguments.

### Code style

- **Fewer private functions**: Prefer module-level or non-underscore-prefixed functions; use "private" (leading-underscore) helpers only when there is a clear need.
- **Constants**: Group constant variables at the upper portion of each Python file; name them in `SCREAMING_SNAKE_CASE`.
- **Imports**: Keep all imports at the top of the file; do not wrap imports in try/except — let import errors fail fast.
- **Docstrings**: Document input and output (parameters and return) in docstrings, including their types (aligned with type hints).

### Unit tests

- Write unit tests for each new function; extend the test suite as the codebase grows.
- Add or update tests when adding functionality, refactoring, or fixing bugs.

### numpy dependency guard (mandatory)

Before finalising any ROS2 Python node, check imported ROS2 message packages (especially `sensor_msgs`) for transitive Python runtime dependencies and **explicitly add `numpy` to the node's Poetry dependencies when needed**. Validate by starting the service and confirming it does not fail with `ModuleNotFoundError: numpy`.

---

## ROS2 Rules

### No Placeholder Data on Topics

Any data published on ROS2 topics **must not be placeholder data**. Only real, valid data from sensors or actual state may be published.

**Do NOT publish:**

- Zeroed or default values when sensor reads fail or are invalid
- Synthetic/coerced data (e.g. filling with zeros to "keep the topic alive")
- Fake or dummy data for testing purposes (use mocks in tests instead)

**If a node cannot obtain valid data** (e.g. sensor unreachable, calibration incomplete):

- Do **not** publish placeholder messages
- Log the condition and skip publishing for that cycle
- The topic may have "no sample yet" until real data is available — that is acceptable

This applies to all ROS2 publishers in `nodes/`.

---

## Servo Rules

### Safety: Least-Dramatic Movement First

When testing new functionality that involves moving servos, limit tests to servos that do the **least dramatic** moves:

- **Arms:** e.g. gripper joints (open/close, rotate) — not base, shoulder, elbow, or wrist.
- **Wheels / drivetrain:** e.g. wheel yaw axes — avoid full-speed or large travel unless explicitly required.

Use minimal amplitude first. If a test needs motion on more dramatic joints, stop and ask the user before proceeding.

### Debug Telemetry Workflow

For servo debugging and post-deploy verification, use **topic_scraper_api** plus **scripts/topic_scraper_collect.py** as the default telemetry/debug workflow (not raw `ros2 topic echo` or ad-hoc scripts).

**Leader–follower oscillation checks:**

- Collect **merged NDJSON** from both **client** and **server** scrapers (e.g. `--source client=http://<client>:18100 --source server=http://<server>:18100`).
- Verify **timing/skew** (e.g. `received_at_ns`, `header_stamp_ns` across sources) and **command stability** (position/effort deltas, no runaway oscillation).
- Use `/rules` and `/rules/{name}` on the scraper API when observation rules are configured (e.g. compare, oscillation).

Example:

```bash
python scripts/topic_scraper_collect.py \
  --source client=http://client.ros2.lan:18100 \
  --source server=http://server.ros2.lan:18100 \
  --select /filter/input_joint_updates:.position \
  --select /follower/joint_states:.position \
  --interval 0.1
```

---

## Mandatory Workflow: Test → Commit → Push → Deploy

**Before considering any change done, complete this sequence in full. No exceptions.**

1. **Test** — Run the relevant test/lint for what you changed:
   - Node tests: e.g. `cd nodes/bridges/feetech_servos && poetry run pytest tests/ -v`
   - Root lint: `poetry run poe lint`
   - All nodes: `poetry run poe lint-nodes`
   - Ansible: `poetry run poe lint-ansible` and `poetry run poe test-ansible`
   - Fix any failures before continuing.

2. **Commit** — Stage and commit with a semantic message (`feat:`, `fix:`, `docs:`, etc.). One logical change per commit. Create a commit after each phase/iteration that passes.

3. **Push** — Push to the upstream repository. When only one agent works on the codebase, push to `main` (no feature branches).

4. **Deploy** — When Ansible or any ROS2 node code changed, use `scripts/deploy-nodes.sh`.
   **Never run `ansible-playbook` directly — always use the script.**
   Always invoke the `ansible-deploy` skill before deploying to get the correct command.
   **Log all deploy output** to `.logs/` (gitignored) instead of system temp dirs:

   ```bash
   mkdir -p .logs

   # Single or multiple changed nodes (parallel)
   ./scripts/deploy-nodes.sh client web_ui filter_node 2>&1 | tee .logs/deploy-client.log
   ./scripts/deploy-nodes.sh server lerobot_leader 2>&1 | tee .logs/deploy-server.log

   # All nodes on a target
   ./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-all.log
   ./scripts/deploy-nodes.sh server --all 2>&1 | tee .logs/deploy-server-all.log
   ```

   so deployed services reflect the latest code.

   > **Note:** The Deploy step is skipped when RPi targets are offline/unreachable. Still complete Test, Commit, and Push.

**Do not skip any step.** If deploy fails, fix issues and redeploy until it succeeds.

### Additional mandatory rules

- If a request conflicts with these conventions (e.g. out-of-scope feature, different convention), ask the user to confirm before overriding.
- **Ansible and ROS2 nodes:** Whenever ROS2 nodes or their configuration change, update the Ansible code so it stays in sync. Ansible must support deploying, enabling, and disabling nodes by name and configuring them via the `ros2_nodes` schema (`name`, `node_type`, `present`, `enabled`, `config`). See `group_vars/client.yml` and `group_vars/server.yml` and the `ros2_node_deploy` role. Keep Ansible lint and tests passing: run `poe lint-ansible` and `poe test-ansible`; fix or document any skip/warn in `ansible/.ansible-lint`.

---

## Linting and Testing Commands

Run from the **repository root** unless noted:

| Command | What it does |
|---|---|
| `poetry run poe lint` | Lint `tests/` and `shared/` (root level) |
| `poetry run poe lint-nodes` | Lint all Python nodes (all 14 node directories with `pyproject.toml`) |
| `./scripts/lint-all-nodes.sh` | Alternative: lint all nodes via shell script (same coverage) |
| `poetry run poe lint-ansible` | Run ansible-lint |
| `poetry run poe test-ansible` | ansible-lint + playbook `--syntax-check` |
| `poetry run pytest tests/ -v` | Run root-level tests |

Each node has its own Poetry environment and `poe lint` / `poe lint-fix` — run these inside the node directory (e.g. `cd nodes/bridges/feetech_servos && poetry run poe lint`).

**Per-node Poetry:** Each Python node has its own `pyproject.toml` and `poetry.lock`. Run `poetry install` and `poetry run poe lint` / `poetry run poe lint-fix` inside each node directory. Root `poe lint` covers only `tests`, `shared`.

**Ansible lint:** Run `ansible-lint .` from `ansible/` or `poetry run poe lint-ansible` from the root. Config: `ansible/.ansible-lint`.

---

## Native Node Source Changes

Whenever source code used by a native ROS2 node is edited, **redeploy that node** so the Poetry venv and service reflect the change. Use `scripts/deploy-nodes.sh <target> <node_name>` to redeploy individual nodes.

---

## Documentation Expectations

Document work in README files:

- One per node under `nodes/` (e.g. `nodes/master2master/README.md`, `nodes/bridges/feetech_servos/README.md`)
- `ansible/README.md`
- `nodes/README.md`, `shared/README.md`
- `tests/README.md` — list each test file, briefly describe what it covers, and document each test (or test group). Update whenever tests are added, removed, or changed.

Target audience: mid-level Python developer with ROS/ROS2 experience. Keep READMEs up to date whenever something in that area changes.

---

## BNO055 IMU Diagnostics

**Use `scripts/bno055_diag.sh` for all BNO055 health checks — never SSH manually.** The skill `bno055-diagnostics` (in `.claude/skills/bno055-diagnostics/SKILL.md`) describes all modes and how to interpret output.

```bash
./scripts/bno055_diag.sh              # Full one-shot diagnostic
./scripts/bno055_diag.sh --watch      # Repeat every 10s
./scripts/bno055_diag.sh --logs-only  # Only service logs
```

---

## GPS RTK Diagnostics

**Use `scripts/rtk_diag.sh` for all RTK health checks — never SSH manually.** The skill `rtk-diagnostics` (in `.claude/skills/rtk-diagnostics/SKILL.md`) describes all modes and how to interpret output.

```bash
./scripts/rtk_diag.sh              # Full one-shot diagnostic
./scripts/rtk_diag.sh --watch      # Repeat every 10s
./scripts/rtk_diag.sh --capture 60 # 60s capture + fix distribution
./scripts/rtk_diag.sh --logs-only  # Only journalctl [diag] lines
```

For re-calibration (survey-in): `./scripts/rtk_calibrate.sh --accuracy 20`

---

## Memory and Decisions

[`MEMORY.md`](MEMORY.md) records key project decisions and LLM context. Update it when making important decisions. Do not modify it during refactoring unless content is genuinely stale.
