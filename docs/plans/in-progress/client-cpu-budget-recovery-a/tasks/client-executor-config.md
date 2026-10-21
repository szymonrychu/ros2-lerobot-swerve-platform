---
title: Client executor config
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The client actually runs the five nodes on the EventsExecutor (about 0.3 core), with a one-line rollback per node."
files: [ansible/group_vars/client.yml, tests/test_client_cpu_load.py, tests/README.md, ansible/README.md]
verify: "uv run pytest tests/test_client_cpu_load.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [head-camera-15-fps, swerve-events-executor, laser-filter-events-executor, marker-events-executor, filter-node-events-executor, feetech-events-executor]
owns: []
uses: []
model: ""
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Client executor config

## Requirement

Item 2 of the brief, deployment half (design D2; approval A2): the client runs the five high-rate single-threaded Python nodes on the EventsExecutor, switchable back by config.

- `ansible/group_vars/client.yml`: `executor: events` at the top level of the `config: |` block of swerve_controller, laser_filter, marker_detector, filter_node, lerobot_follower and base_servos, each with a one-line comment naming the rollback (`single_threaded`).
- mcp_server and web_ui get no `executor` key (they keep their MultiThreadedExecutor / own threads; EventsExecutor is experimental and ignores callback groups). `ansible/group_vars/server.yml` is not changed (the server's feetech leader bridge keeps the default).
- Tests first (red seen) in `tests/test_client_cpu_load.py`: `test_high_rate_python_nodes_use_events_executor` (the six entries above parse to `executor: events`) and `test_mcp_server_and_web_ui_keep_their_executor` (no `executor` key in either config block).
- `ansible/README.md` (one paragraph: which nodes run the EventsExecutor and how to roll back) and `tests/README.md` are updated.

## Acceptance conditions

- a1: the six client entries select the events executor [evidence: test:tests/test_client_cpu_load.py::test_high_rate_python_nodes_use_events_executor]
- a2: mcp_server and web_ui have no executor key [evidence: test:tests/test_client_cpu_load.py::test_mcp_server_and_web_ui_keep_their_executor]
- a3: every node config still loads with its node's own loader (the existing config-render tests stay green) [evidence: command:uv run pytest tests -q -k "config or cpu_load"]
- a4: root lint and ansible lint pass [evidence: command:uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`
- `ansible/README.md`

## Interfaces

Uses the `executor` key built by swerve-events-executor, laser-filter-events-executor, marker-events-executor, filter-node-events-executor and feetech-events-executor (all dependencies). Values only `events` or `single_threaded`.

## Expected outcome

The six client node configs select `executor: events`; mcp_server and web_ui do not; `uv run pytest tests/test_client_cpu_load.py -q` passes.

## Steps

- Add the two tests; `uv run pytest tests/test_client_cpu_load.py -q` FAILS.
- Add `executor: events` with the rollback comment to the six config blocks; rerun, PASS.
- Load each rendered config block with its node's own loader in a quick check through the existing config tests (`uv run pytest tests -q -k "config or cpu_load"`).
- `uv run poe lint && uv run poe test-ansible`; update ansible/README.md and tests/README.md.
- Commit `perf(ansible): run the high-rate Python nodes on the EventsExecutor`.

## Notes

The feetech bridge config is read from the `config: |` block via FEETECH_SERVOS_CONFIG (lerobot_follower, base_servos); server.yml's feetech leader keeps the default. Rollback per node: `executor: single_threaded` and `./scripts/deploy-nodes.sh client <node>`. Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.
