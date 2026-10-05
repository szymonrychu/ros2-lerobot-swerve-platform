# Shared libraries

Python code shared by multiple nodes. `shared/` is one installable Poetry package, `ros2-common` (import name
`ros2_common`, see [pyproject.toml](pyproject.toml)). Node code is in [nodes/](../nodes/).

See [CLAUDE.md](../CLAUDE.md): use Python 3 type hints and extend unit tests as the project grows.

## Using it from a node

Add a Poetry path dependency in develop mode, relative to the node directory:

```toml
[tool.poetry.dependencies]
ros2-common = { path = "../../shared", develop = true }
```

Then run `poetry lock` once. On the Pi the whole repo is cloned to `ros2_repo_dest`, and the `ros2_node_deploy` role
runs `poetry install --only main` inside `<ros2_repo_dest>/nodes/<node>` into `/opt/ros2-nodes/<name>/venv`, so the
same `../../shared` path resolves there; nothing extra is needed in Ansible. A node one level deeper (e.g.
`nodes/bridges/<node>`) uses `../../../shared`.

## Modules

| Module | Contents |
|---|---|
| `ros2_common._utils` | `clamp(value, low, high)` |
| `ros2_common.battery` | `BatteryConfig` (pydantic: `topic` `/battery_state`, `cells` 3, `cutoff_cell_v` 2.8, `resume_cell_v` 2.9, `stale_s` 5.0; `resume_cell_v >= cutoff_cell_v`, `cells >= 1`) and `BatteryGuard` (thread-safe cut-off hysteresis: enter below `cells * cutoff_cell_v`, leave only above `cells * resume_cell_v`; no reading or older than `stale_s` = unknown = not blocked) |

Used by: `nodes/mcp_server` (motion tools refused in cut-off). `nodes/web_ui` still has its own copy of the guard.

## Tests

Run from the repo root with the root pytest (`poetry run pytest tests -v`): `tests/test_shared_utils.py`,
`tests/test_shared_battery.py` (config and guard logic) and `tests/test_shared_package.py` (packaging and the node
path dependency). They import via `shared.ros2_common`. Documented in [tests/README.md](../tests/README.md).
