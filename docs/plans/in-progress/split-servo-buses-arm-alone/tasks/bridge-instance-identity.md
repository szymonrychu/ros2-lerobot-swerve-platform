---
title: Bridge instance identity
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/split-servo-buses-arm-alone--bridge-instance-identity
branch: ""
verified_head: 336a2cb67ff414e05f7d0a74618b8d31cba545cd
run: split-servo-buses-arm-alone
provides: []
acs_for_audit: []
process_gaps: []
---

# Bridge instance identity

## Requirement

- `BridgeConfig` (`config.py`) gains `node_name: str | None = None`, parsed by `load_config` from an optional top-level `node_name` key: stripped string; missing or empty gives None; a value that is not a valid ROS node name (`^[A-Za-z][A-Za-z0-9_]*$`, module constant `NODE_NAME_PATTERN`) makes `load_config` return None, like every other invalid config. Docstring Attributes entry added.
- `bridge.py`: the hard-coded `NODE_NAME = "lerobot_follower"` and `Node("feetech_servos_bridge")` become module constants `DEFAULT_METRICS_NODE_NAME = "lerobot_follower"` and `DEFAULT_ROS_NODE_NAME = "feetech_servos_bridge"`; `run_bridge` creates `Node(config.node_name or DEFAULT_ROS_NODE_NAME)` and calls `start_metrics_server(port, config.node_name or DEFAULT_METRICS_NODE_NAME)`. With `node_name` unset both names are exactly today's (the server's `lerobot_leader` is untouched).
- `serve_bridge`: when `config.battery_interval_s == 0` no `BatteryState` publisher is created and nothing is ever published on `battery_topic`; with an interval > 0 behaviour is unchanged.
- Tests first, seen failing before the code exists: new `tests/test_instance_identity.py`, reusing the stub pattern of `tests/test_shutdown.py` (`install_ros_stubs`, `Node`, `SingleThreadedExecutor` and `start_metrics_server` mocked): `node_name` parsed from YAML, default None, invalid names (`"9bus"`, `"a/b"`, `"a-b"`) make `load_config` return None; `run_bridge` passes `node_name` to `Node` and to `start_metrics_server`; unset keeps `feetech_servos_bridge` / `lerobot_follower`; no `create_publisher` call for `BatteryState` at `battery_interval_s: 0`, one at `1.0`.
- `README.md`: a `node_name` row in the config reference, the battery row says interval 0 creates no publisher, and a new section `## Several bridges on one host` (distinct `node_name` and `metrics_port`, battery read by one instance only, one serial adapter per instance, `extra_groups` share one adapter); the `extra_groups` bullet no longer says the swerve servos share the follower arm bus (they run in their own instance `base_servos`; the pan/tilt head joins that instance as a group).

## Acceptance conditions

- a1: node_name is parsed, defaults to None and invalid ROS names make load_config return None [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q]
- a2: run_bridge names the rclpy node and the metrics node info from node_name and keeps today's names when unset [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q]
- a3: no BatteryState publisher exists at battery_interval_s 0, one exists at 1.0 [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q]
- a4: BridgeConfig carries node_name [evidence: symbol: nodes/bridges/feetech_servos/config.py:BridgeConfig]
- a5: README documents running several bridges on one host [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Several bridges on one host]
- a6: feetech_servos suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_instance_identity.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- `feetech_servos.config.BridgeConfig.node_name: str | None` (owned contract `BridgeConfig.node_name`, consumed by split-bus-group-vars through the client.yml `node_name` key).
- `feetech_servos.config.NODE_NAME_PATTERN`; `feetech_servos.bridge.DEFAULT_ROS_NODE_NAME`, `DEFAULT_METRICS_NODE_NAME`.
- Unchanged: `run_bridge(config: BridgeConfig) -> None`, `serve_bridge(config: BridgeConfig, node: Node) -> None`.

## Expected outcome

A feetech bridge configured with `node_name: base_servos` and `battery_interval_s: 0` runs as ROS node `base_servos`, exports `robot_node_info{node="base_servos"}` and advertises no `/battery_state`; an unconfigured bridge behaves exactly as before.

## Steps

- Write `tests/test_instance_identity.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q`; expect failures on the missing `node_name` attribute and the unconditional battery publisher.
- Implement the config field and the bridge changes; rerun the file until green.
- Update README; run the verify command; commit `feat(feetech_servos): node_name config and no battery publisher when disabled`.

## Notes

- Design D3. The server's `lerobot_leader` config sets no `node_name` and no `metrics_port`: its behaviour must not change.
- Keep constants at the top, type hints and typed docstrings (CLAUDE.md). Do not wrap imports in try/except.
- `tests/test_shutdown.py::bridge_module` is the harness to reuse (import it or copy the three monkeypatches; no new fake framework).

## Ruling

Accepted by the orchestrator: verified and merged (feetech 286, mcp_server 2465, root 1149 passed); mcp-base-servo-registers gate-repeat findings refuted by the reviewer.
