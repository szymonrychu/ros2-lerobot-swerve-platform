---
title: Bridge instance identity
status: todo
repo: ros2-lerobot-sverve-platform
value: "Two feetech bridges on one robot get distinct ROS node names and metrics labels, and only one publishes the battery."
files: [nodes/bridges/feetech_servos/config.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/tests/test_instance_identity.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint"
depends_on: []
owns: [BridgeConfig.node_name]
uses: []
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Bridge instance identity

## Requirement

- `BridgeConfig` (`config.py`) gains `node_name: str | None = None`, parsed by `load_config` from an optional top-level `node_name` key: stripped string; missing or empty gives None; a value that is not a valid ROS node name (`^[A-Za-z][A-Za-z0-9_]*$`, module constant `NODE_NAME_PATTERN`) makes `load_config` return None, like every other invalid config. Docstring Attributes entry added.
- `bridge.py`: the hard-coded `NODE_NAME = "lerobot_follower"` and `Node("feetech_servos_bridge")` become module constants `DEFAULT_METRICS_NODE_NAME = "lerobot_follower"` and `DEFAULT_ROS_NODE_NAME = "feetech_servos_bridge"`; `run_bridge` creates `Node(config.node_name or DEFAULT_ROS_NODE_NAME)` and calls `start_metrics_server(port, config.node_name or DEFAULT_METRICS_NODE_NAME)`. With `node_name` unset both names are exactly today's (the server's `lerobot_leader` is untouched).
- `serve_bridge`: when `config.battery_interval_s == 0` no `BatteryState` publisher is created and nothing is ever published on `battery_topic`; with an interval > 0 behaviour is unchanged.
- Tests first, seen failing before the code exists: new `tests/test_instance_identity.py`, reusing the stub pattern of `tests/test_shutdown.py` (`install_ros_stubs`, `Node`, `SingleThreadedExecutor` and `start_metrics_server` mocked): `node_name` parsed from YAML, default None, invalid names (`"9bus"`, `"a/b"`, `"a-b"`) make `load_config` return None; `run_bridge` passes `node_name` to `Node` and to `start_metrics_server`; unset keeps `feetech_servos_bridge` / `lerobot_follower`; no `create_publisher` call for `BatteryState` at `battery_interval_s: 0`, one at `1.0`.
- `README.md`: a `node_name` row in the config reference, the battery row says interval 0 creates no publisher, and a new section `## Several bridges on one host` (distinct `node_name` and `metrics_port`, battery read by one instance only, one serial adapter per instance, `extra_groups` share one adapter); the `extra_groups` bullet no longer says the swerve servos share the follower arm bus (they run in their own instance `swerve_drive_servos`; the pan/tilt head joins that instance as a group).

## Acceptance conditions

- a1: node_name is parsed, defaults to None and invalid ROS names make load_config return None [evidence: test: nodes/bridges/feetech_servos/tests/test_instance_identity.py]
- a2: run_bridge names the rclpy node and the metrics node info from node_name and keeps today's names when unset [evidence: test: nodes/bridges/feetech_servos/tests/test_instance_identity.py]
- a3: no BatteryState publisher exists at battery_interval_s 0, one exists at 1.0 [evidence: test: nodes/bridges/feetech_servos/tests/test_instance_identity.py]
- a4: BridgeConfig carries node_name [evidence: symbol: nodes/bridges/feetech_servos/config.py:BridgeConfig]
- a5: README documents running several bridges on one host [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Several bridges on one host]
- a6: feetech_servos suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_instance_identity.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
