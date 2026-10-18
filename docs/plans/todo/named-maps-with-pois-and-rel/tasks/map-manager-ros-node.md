---
title: Map manager ROS node
status: todo
repo: ros2-lerobot-sverve-platform
value: "map_manager runs on the robot: it answers named-map commands over topics, publishes a truthful status, and costs no CPU while idle."
files: [nodes/map_manager/map_manager/node.py, nodes/map_manager/map_manager/__main__.py, nodes/map_manager/map_manager/metrics.py, nodes/map_manager/tests/ros_stubs.py, nodes/map_manager/tests/test_node.py, nodes/map_manager/README.md]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q && uv run --frozen pytest -q && uv run --frozen poe lint"
depends_on: [map-manager-operations]
owns: []
uses: [map-manager-core, map-manager-contract, scan-agreement]
model: sonnet
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map manager ROS node

## Requirement

The rclpy layer of map_manager (design D4, D10), the only rclpy module, tested with stubs like
`nodes/poi_store/tests/ros_stubs.py`.

- `__main__.py`: load `MapManagerConfig`, exit 78 on a config error (as the other nodes), start metrics, run the node.
- Idle: subscriptions to `/map_manager/command` (reliable), `/poi/list` (latched), `/poi/result`; publishers
  `/map_manager/result` (volatile), `/map_manager/status` (latched), `/map_manager/localised` (std_msgs/Bool,
  latched); service clients for serialize (`slam_toolbox/srv/SerializePoseGraph`), deserialize
  (`DeserializePoseGraph`), pause (`Pause`) and get_parameters. No TF listener, no `/map`, `/scan_filtered` or `/pose`
  subscription while idle.
- `start_operation` creates the TF buffer/listener and the `/map` (latched), `/scan_filtered` (sensor QoS) and `/pose`
  subscriptions; `end_operation` destroys them. Agreement samples are computed at `agreement_hz` from scans through TF
  `map <- laser` with the shared `agreement` functions against the latest `/map`.
- Commands run on a worker thread (a MultiThreadedExecutor or a dedicated thread with futures) so service calls and
  waits never block the executor; JSON errors answer `ok: false` with the reason.
- Status published at start (real maps list, `localisation.state` `unknown`, `load_seq` 0) and after every change;
  `/map_manager/localised` is published only once a load reached `loading` or a verdict, never at start.
- POI port: replace sends `{"op": "replace"}` on `/poi/command` and matches `/poi/result` by request_id with
  `poi_timeout_s`.
- `metrics.py`: `map_manager_operations_total{op,ok}`, `map_manager_localisation_ratio` (set only after a measured
  verdict), `map_manager_load_seconds` histogram, plus `robot_node_info{node="map_manager"}` via ros2_metrics.
- `cleanup_temp()` at start.
- README `nodes/map_manager/README.md`: purpose, the topic table, bundle layout, load sequence with the verified
  slam_toolbox facts (design D5), config keys, metrics, tests.
- `tests/test_node.py` (test first): idle subscriptions only; operation creates and destroys the heavy
  subscriptions; a command produces a result; status at start; localised not published at start; invalid JSON;
  numpy import guard (the module imports sensor_msgs-dependent code with numpy declared).

## Acceptance conditions

- a1: while idle only the command, POI list and POI result subscriptions exist [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k idle]
- a2: an operation creates the TF, map, scan and pose subscriptions and destroys them afterwards [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k operation_subscriptions]
- a3: status is published at start with state unknown and localised is not published at start [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k start]
- a4: a JSON command is answered on the result topic, invalid JSON with ok false [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_node.py -q -k command]
- a5: README documents topics, bundle layout and the load sequence [evidence: doc-section: nodes/map_manager/README.md#Load sequence]
- a6: the whole map_manager suite and lint pass [evidence: command: cd nodes/map_manager && uv run --frozen pytest -q && uv run --frozen poe lint]

## Files

- `nodes/map_manager/map_manager/node.py`
- `nodes/map_manager/map_manager/__main__.py`
- `nodes/map_manager/map_manager/metrics.py`
- `nodes/map_manager/tests/ros_stubs.py`
- `nodes/map_manager/tests/test_node.py`
- `nodes/map_manager/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
