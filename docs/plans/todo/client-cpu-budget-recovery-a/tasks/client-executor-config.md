---
title: Client executor config
status: todo
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
