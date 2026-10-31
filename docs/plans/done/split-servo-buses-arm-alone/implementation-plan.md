# Split servo buses arm alone base plus head implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Bridge instance identity

- Status: done
- Files: `nodes/bridges/feetech_servos/config.py`, `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/tests/test_instance_identity.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint`
- Verified head: 336a2cb67ff414e05f7d0a74618b8d31cba545cd

#### Requirement

- `BridgeConfig` (`config.py`) gains `node_name: str | None = None`, parsed by `load_config` from an optional top-level `node_name` key: stripped string; missing or empty gives None; a value that is not a valid ROS node name (`^[A-Za-z][A-Za-z0-9_]*$`, module constant `NODE_NAME_PATTERN`) makes `load_config` return None, like every other invalid config. Docstring Attributes entry added.
- `bridge.py`: the hard-coded `NODE_NAME = "lerobot_follower"` and `Node("feetech_servos_bridge")` become module constants `DEFAULT_METRICS_NODE_NAME = "lerobot_follower"` and `DEFAULT_ROS_NODE_NAME = "feetech_servos_bridge"`; `run_bridge` creates `Node(config.node_name or DEFAULT_ROS_NODE_NAME)` and calls `start_metrics_server(port, config.node_name or DEFAULT_METRICS_NODE_NAME)`. With `node_name` unset both names are exactly today's (the server's `lerobot_leader` is untouched).
- `serve_bridge`: when `config.battery_interval_s == 0` no `BatteryState` publisher is created and nothing is ever published on `battery_topic`; with an interval > 0 behaviour is unchanged.
- Tests first, seen failing before the code exists: new `tests/test_instance_identity.py`, reusing the stub pattern of `tests/test_shutdown.py` (`install_ros_stubs`, `Node`, `SingleThreadedExecutor` and `start_metrics_server` mocked): `node_name` parsed from YAML, default None, invalid names (`"9bus"`, `"a/b"`, `"a-b"`) make `load_config` return None; `run_bridge` passes `node_name` to `Node` and to `start_metrics_server`; unset keeps `feetech_servos_bridge` / `lerobot_follower`; no `create_publisher` call for `BatteryState` at `battery_interval_s: 0`, one at `1.0`.
- `README.md`: a `node_name` row in the config reference, the battery row says interval 0 creates no publisher, and a new section `## Several bridges on one host` (distinct `node_name` and `metrics_port`, battery read by one instance only, one serial adapter per instance, `extra_groups` share one adapter); the `extra_groups` bullet no longer says the swerve servos share the follower arm bus (they run in their own instance `base_servos`; the pan/tilt head joins that instance as a group).

#### Acceptance conditions

- a1: node_name is parsed, defaults to None and invalid ROS names make load_config return None [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q]
- a2: run_bridge names the rclpy node and the metrics node info from node_name and keeps today's names when unset [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q]
- a3: no BatteryState publisher exists at battery_interval_s 0, one exists at 1.0 [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q]
- a4: BridgeConfig carries node_name [evidence: symbol: nodes/bridges/feetech_servos/config.py:BridgeConfig]
- a5: README documents running several bridges on one host [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Several bridges on one host]
- a6: feetech_servos suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_instance_identity.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- `feetech_servos.config.BridgeConfig.node_name: str | None` (owned contract `BridgeConfig.node_name`, consumed by split-bus-group-vars through the client.yml `node_name` key).
- `feetech_servos.config.NODE_NAME_PATTERN`; `feetech_servos.bridge.DEFAULT_ROS_NODE_NAME`, `DEFAULT_METRICS_NODE_NAME`.
- Unchanged: `run_bridge(config: BridgeConfig) -> None`, `serve_bridge(config: BridgeConfig, node: Node) -> None`.

#### Expected outcome

A feetech bridge configured with `node_name: base_servos` and `battery_interval_s: 0` runs as ROS node `base_servos`, exports `robot_node_info{node="base_servos"}` and advertises no `/battery_state`; an unconfigured bridge behaves exactly as before.

#### Steps

- Write `tests/test_instance_identity.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_instance_identity.py -q`; expect failures on the missing `node_name` attribute and the unconditional battery publisher.
- Implement the config field and the bridge changes; rerun the file until green.
- Update README; run the verify command; commit `feat(feetech_servos): node_name config and no battery publisher when disabled`.

#### Notes

- Design D3. The server's `lerobot_leader` config sets no `node_name` and no `metrics_port`: its behaviour must not change.
- Keep constants at the top, type hints and typed docstrings (CLAUDE.md). Do not wrap imports in try/except.
- `tests/test_shutdown.py::bridge_module` is the harness to reuse (import it or copy the three monkeypatches; no new fake framework).

#### Ruling

Accepted by the orchestrator: verified and merged (feetech 286, mcp_server 2465, root 1149 passed); mcp-base-servo-registers gate-repeat findings refuted by the reviewer.

### S2: MCP base servo registers

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/tests/test_on_demand_subscriptions.py`, `nodes/mcp_server/README.md`, `ansible/group_vars/client.yml`, `tests/test_mcp_server_config.py`, `tests/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py tests/test_monitor.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint`
- Verified head: 3eb7eea753291d2698ff5cf088c3dff31724c3ed

#### Requirement

- `TopicSettings` (`mcp_server/config.py`): new field `base_servo_registers: str = "/swerve_drive/servo_registers"` next to `servo_registers`, commented as the base bus bridge's JSON register dump (swerve servos; head joints join it when the pan/tilt epic adds them to that bus); the `servo_registers` comment says arm servos (follower bus).
- `ros_iface.py`: `RosRobot` subscribes `t.base_servo_registers` at start with the same `self.on_servo_registers` callback as `t.servo_registers`; `"base_servo_registers"` is added to `MOTION_CRITICAL_TOPICS`, so it is never on demand.
- `ansible/group_vars/client.yml` mcp_server `topics:` gains `base_servo_registers: /swerve_drive/servo_registers` with a comment; the `servo_registers` comment says arm servos.
- Tests first, seen failing: `tests/test_on_demand_subscriptions.py::test_motion_critical_list_names_the_monitor_and_motion_feeds` expects `base_servo_registers`; a new test in the same file delivers a JSON dump `{"fl_drive": {"present_temperature": 70, "status": 0}}` on the base topic through the rig and asserts it reaches `RobotMonitor` (`fl_drive` in `monitor.servos` and a critical `overheat` level for `fl_drive`). Root `tests/test_mcp_server_config.py::test_mcp_server_monitor_topics_match_their_producers` asserts `topics["base_servo_registers"] == "/swerve_drive/servo_registers"` (the namespace consistency with `base_servos` is asserted by split-bus-group-vars).
- `nodes/mcp_server/README.md` lists both register dumps wherever `/follower/servo_registers` is described as the body monitor feed; `tests/README.md` rows for the changed and new tests.

#### Acceptance conditions

- a1: the base register dump topic is a config field with the base bus default [evidence: symbol: nodes/mcp_server/mcp_server/config.py:TopicSettings]
- a2: base_servo_registers is motion-critical and subscribed at start [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py::test_motion_critical_list_names_the_monitor_and_motion_feeds -q]
- a3: a dump on the base topic reaches the robot monitor and raises the swerve servo overheat [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py -q]
- a4: client.yml wires the base dump topic [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_monitor_topics_match_their_producers]
- a5: mcp_server targeted tests, root config test and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py tests/test_monitor.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

#### Interfaces

- `mcp_server.config.TopicSettings.base_servo_registers: str` (owned contract `topics.base_servo_registers`; split-bus-group-vars asserts it equals `/<base_servos namespace>/servo_registers`).
- `mcp_server.ros_iface.MOTION_CRITICAL_TOPICS` gains `"base_servo_registers"`; `RosRobot.on_servo_registers(msg: String) -> None` unchanged and shared.

#### Expected outcome

mcp_server reads both register dumps from start; swerve servo temperatures and status errors keep producing robot events once they arrive on `/swerve_drive/servo_registers`.

#### Steps

- Edit the tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_on_demand_subscriptions.py -q` and `uv run pytest tests/test_mcp_server_config.py -q` from the root; expect failures (unknown field, missing subscription; the strict model also rejects the client.yml key until the field exists).
- Implement config, ros_iface, client.yml, docs; run the verify command; commit `feat(mcp_server): subscribe the base bus servo register dump`.

#### Notes

- Design D6. `TopicSettings` is a `StrictModel` (`extra="forbid"`): the client.yml key and the field must land in this one task.
- `follower_set_register` stays `/follower/set_register` (gripper grip profiles are arm writes).
- Until the split is deployed nothing publishes the base topic: the subscription waits, no placeholder data.
- Deploying mcp_server ends a running agent session (memory); the deploy itself is the orchestrator's runbook step.

#### Ruling

Accepted by the orchestrator: verified and merged (feetech 286, mcp_server 2465, root 1149 passed); mcp-base-servo-registers gate-repeat findings refuted by the reviewer.

### S3: tests/README.md (repo root, around line 425) describes the feetech_servos tests but does not list test_bus_budget.py

- Status: done
- Files: `tests/README.md`
- Verify: none
- Verified head: 211c42d7a5cab8c57434f4c2a3823913616b8f80

#### Requirement

tests/README.md (repo root, around line 425) describes the feetech_servos tests but does not list test_bus_budget.py

Where: tests/README.md

Why: CLAUDE.md requires tests/README.md to list each test file; it is outside this task's named files

#### Acceptance conditions

- a1: resolved without a code change in this task [evidence: judgement]

#### Files

- `tests/README.md`

#### Interfaces

- None.

#### Expected outcome

- Resolved.

#### Steps

- None.

#### Notes


Discovered in task `bus-budget-test` during run `split-servo-buses-arm-alone`.

Blocked by the dedup check:
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'deploy-head-camera-playbook' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'final-suite-and-runbook' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'head-camera-docs' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'head-camera-node-type' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'mcp-head-camera-topic' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'retire-pi-cameras' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'web-ui-head-camera-topics' of in-progress epic 'front-camera-30-fps-stream-w'
- task 'tests-readme-md-repo-root-ar': file 'tests/README.md' is also in task 'named-maps-docs-and-memory' of in-progress epic 'named-maps-with-pois-and-rel'


#### Resolution

Folded into final-suite-and-runbook via the run instructions (add the tests/README.md row for test_bus_budget.py).

### S4: The worktree's ansible/ has no inventory file (gitignored; only inventory.example), so ad-hoc ansible commands from a task worktree need -i pointing at the main checkout's ansible/inventory.

- Status: done
- Files: none
- Verify: none
- Verified head: 211c42d7a5cab8c57434f4c2a3823913616b8f80

#### Requirement

The worktree's ansible/ has no inventory file (gitignored; only inventory.example), so ad-hoc ansible commands from a task worktree need -i pointing at the main checkout's ansible/inventory.

Where: ansible/ansible.cfg (inventory = inventory)

Why: Runbook stage 0 commands fail with 'Could not match supplied host pattern' when run from a worktree.

#### Acceptance conditions

- a1: the run planText tells agents to pass -i <main checkout>/ansible/inventory for ad-hoc ansible from a worktree [evidence: judgement]


#### Files

- `ansible/ansible.cfg` (no change)

#### Interfaces

- None.

#### Expected outcome

- Agents know to point ad-hoc ansible at the main checkout's inventory.

#### Steps

- No code change; handled in the run instructions (orchestrator ruling 2026-10-18).

#### Notes


Discovered in task `split-bus-group-vars` during run `split-servo-buses-arm-alone`.


#### Resolution

No code change (orchestrator ruling 2026-10-18): the inventory is gitignored by design; run instructions now name -i <main>/ansible/inventory.

### S5: Tilt guard torque topic

- Status: done
- Files: `nodes/swerve_drive_controller/swerve_drive_controller/tilt_guard.py`, `nodes/swerve_drive_controller/tests/test_tilt_guard.py`, `nodes/swerve_drive_controller/tests/test_node_tilt_guard.py`, `nodes/swerve_drive_controller/README.md`
- Verify: `cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q && uv run --frozen poe lint`
- Verified head: 83af294455d8aa6986d54f93927266765dc6b374

#### Requirement

- `tilt_guard.py`: module constant `DEFAULT_TORQUE_TOPIC = "/swerve_drive/set_register"` (the `base_servos` feetech bridge's set_register topic, design D6); `TiltGuardConfig.torque_topic` defaults to it; the field docstring says it is the base bus bridge.
- Tests first, seen failing: `tests/test_tilt_guard.py` asserts the default is `/swerve_drive/set_register`; `tests/test_node_tilt_guard.py` reads the published torque payloads from `DEFAULT_TORQUE_TOPIC` (imported, no `/follower/set_register` literal left) and still sees `torque_enable` 0 then 1 for the four drive joints with `error_action: torque_off`.
- `README.md`: the overview bullet that says the 8 servos share the follower-arm serial bus is rewritten: they have their own bus (second USB adapter) served by the `base_servos` feetech bridge at 50 Hz, namespace `swerve_drive`; the Publishes line, the torque_off text and the `torque_topic` table row name `/swerve_drive/set_register`.

#### Acceptance conditions

- a1: the tilt guard torque topic defaults to the base bus bridge [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_tilt_guard.py -q]
- a2: torque_off payloads for the four drive joints are published on the default topic [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_node_tilt_guard.py -q]
- a3: nothing in the swerve controller tree names the follower set_register or the follower-arm bus [evidence: command: ! git grep -n -E 'follower-arm serial bus|/follower/set_register|follower_set_register' -- nodes/swerve_drive_controller]
- a4: swerve_drive_controller suite and lint pass [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

#### Files

- `nodes/swerve_drive_controller/swerve_drive_controller/tilt_guard.py`
- `nodes/swerve_drive_controller/tests/test_tilt_guard.py`
- `nodes/swerve_drive_controller/tests/test_node_tilt_guard.py`
- `nodes/swerve_drive_controller/README.md`

#### Interfaces

- `swerve_drive_controller.tilt_guard.DEFAULT_TORQUE_TOPIC: str`, `TiltGuardConfig.torque_topic` (owned contract `TiltGuardConfig.torque_topic default`; split-bus-group-vars sets the same value in client.yml).

#### Expected outcome

Without any config, the tilt guard's torque_off writes go to `/swerve_drive/set_register`, where the base bridge knows the drive joints.

#### Steps

- Edit the two tests; run `cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_tilt_guard.py tests/test_node_tilt_guard.py -q`; expect the default assertion to fail.
- Change the default and the README; run the verify command; commit `fix(swerve_controller): tilt guard torque_off targets the base bus bridge`.

#### Notes

- Design D6. The deployed `error_action` stays `brake`; client.yml's `torque_topic` is changed by split-bus-group-vars, not here.
- No other code change: the node already publishes on `tilt_cfg.torque_topic` (`node.py:162`).

#### Ruling

Accepted by the orchestrator: verified and merged (feetech 286, mcp_server 2465, root 1149 passed); mcp-base-servo-registers gate-repeat findings refuted by the reviewer.

### S6: Split bus group vars

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_servo_bus_split_config.py`, `tests/test_node_metrics_wiring.py`, `tests/test_battery_config.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py tests/test_mcp_server_config.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: b0aae1373e120f3b36cd3d74332c996a8bbcbb7c

#### Requirement

- `lerobot_follower` config (client.yml): only the six arm joints (IDs 1-6, gripper source/command steps unchanged); `extra_groups`, `velocity_command_timeout_s` and the swerve comments removed; `node_name: lerobot_follower` added; `device` stays `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A059004-if00`; control_loop_hz stays the default 100; battery, `publish_effort_joints`, `direct_command_sources`, goal refresh and read failure keys unchanged; the `publish_current_joints` cost comment is restated for the 6-servo bus (+54 bytes per cycle) with the value still `[]`.
- The retired `swerve_drive_servos` entry (`present: false` since `03b43eb5`, already uninstalled on the robot) is deleted, and the playbook step `Deploy swerve_drive_servos` in `ansible/playbooks/deploy_nodes_client.yml` becomes `Deploy base_servos` (`_deploy_node_name: base_servos`, tags `[base_servos, apt, python, build, config]`, apply tag `base_servos`), keeping its position after `Deploy lerobot_follower` (design D1).
- New `base_servos` entry: `present: true`, `enabled: true`, `env: [FEETECH_SERVOS_CONFIG=/etc/ros2/base_servos/config.yaml]`, config: `metrics_port: 19115`, `node_name: base_servos`, `device:` the second adapter's by-id path recorded in design.md "Assumptions and unknowns" A1, `namespace: swerve_drive`, `log_joint_updates: false`, `enable_torque_on_start: true`, `register_publish_interval_s: 10`, `control_loop_hz: 50`, `battery_interval_s: 0`, `velocity_command_timeout_s: 0.3`, `goal_refresh_interval_s: 1.0`, `read_failure_limit: 3`, `read_retry_interval_s: 5.0`, and the eight joints moved verbatim with their comments: fl_drive 32 velocity inverted, fl_steer 33, fr_drive 35 velocity, fr_steer 34, rl_drive 39 velocity inverted, rl_steer 38, rr_drive 36 velocity, rr_steer 37. A comment marks where the pan/tilt epic adds `extra_groups: [{namespace: head, joint_names: [...]}]` (design D7); no head joints are configured. The old "moved onto the lerobot_follower bus" comment is replaced.
- `swerve_controller` `tilt_guard.torque_topic: /swerve_drive/set_register`.
- `monitoring_agent_node_targets` gains `{node: base_servos, address: "127.0.0.1:19115", path: /metrics}`.
- Tests first, seen failing: new `tests/test_servo_bus_split_config.py` asserting: the follower serves exactly IDs 1-6, has no `extra_groups`, `node_name` lerobot_follower, the arm adapter path, control rate 100 (unset or 100); `base_servos` is present and enabled with namespace `swerve_drive`, the exact eight joints above (ID, mode, inverted), rate 50, battery 0, metrics 19115, velocity timeout 0.3, torque on start; no `ros2_nodes` entry and no playbook step is named `swerve_drive_servos`; both devices are `/dev/serial/by-id/` paths and differ; the client's present feetech_servos entries are exactly these two, with distinct `node_name` and `metrics_port` and disjoint servo IDs; `swerve_controller` `joint_names` equal the base joint names and its `joint_states_topic` / `joint_commands_topic` are `/swerve_drive/joint_states` / `/swerve_drive/joint_commands`; `tilt_guard.torque_topic == "/swerve_drive/set_register"`; mcp_server `topics.base_servo_registers == "/swerve_drive/servo_registers"`, `topics.servo_registers == "/follower/servo_registers"`, `topics.follower_set_register == "/follower/set_register"`; the type defaults give the entry the critical cgroup tier (no per-entry `cpu_weight`/`io_weight`/`oom_score_adjust` override). `tests/test_node_metrics_wiring.py` `CONFIG_METRICS_PORTS` gains `"base_servos": 19115`. `tests/test_battery_config.py` gains a test that exactly one client feetech_servos entry has `battery_interval_s > 0` (lerobot_follower) and `base_servos` has 0. `tests/README.md` rows for all of them.

#### Acceptance conditions

- a1: the follower config serves only the arm on its adapter [evidence: test: tests/test_servo_bus_split_config.py]
- a2: base_servos carries the eight swerve joints verbatim at 50 Hz on its own by-id device with battery off and metrics 19115 [evidence: test: tests/test_servo_bus_split_config.py]
- a3: swerve_controller, the tilt guard and mcp_server point at the base bridge topics [evidence: test: tests/test_servo_bus_split_config.py]
- a4: the base bridge metrics endpoint has an Alloy target [evidence: test: tests/test_node_metrics_wiring.py::test_every_enabled_contract_node_has_a_matching_scrape_target]
- a8: the base bridge has a tagged deploy step and the retired name is gone [evidence: test: tests/test_ansible_deploy_speed.py::test_every_node_has_a_tagged_deploy_step]
- a5: exactly one feetech instance publishes the battery [evidence: test: tests/test_battery_config.py]
- a6: client.yml has no placeholder device value [evidence: command: ! git grep -n -E 'PENDING|by-id/REPLACE|CHANGEME' -- ansible/group_vars/client.yml]
- a7: root targeted tests, lint and ansible syntax pass [evidence: command: uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py tests/test_mcp_server_config.py tests/test_nav_stack_config.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_servo_bus_split_config.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_battery_config.py`
- `tests/README.md`

#### Interfaces

- Owned contract `base_servos entry` in `ansible/group_vars/client.yml` (`ros2_nodes`), read by serial-device-preflight, bus-budget-test and docs-and-diagnostics.
- Uses `BridgeConfig.node_name` (key `node_name`), `TiltGuardConfig.torque_topic` (same value as its default) and `TopicSettings.base_servo_registers`.
- Playbook step `Deploy base_servos` (renamed from `Deploy swerve_drive_servos`); `tests/test_ansible_deploy_speed.py::test_every_node_has_a_tagged_deploy_step` keeps entries and steps in sync.

#### Expected outcome

One deploy of `lerobot_follower base_servos swerve_controller mcp_server monitoring` gives an arm-only follower and a base bridge on the second adapter, with all base consumers on `/swerve_drive/*`.

#### Steps

- Confirm design.md A1 holds a real `/dev/serial/by-id/...` path (not `PENDING`); if not, stop as blocked: the orchestrator fills it from the user or runbook stage 0.
- Write `tests/test_servo_bus_split_config.py` and the two test edits; run `uv run pytest tests/test_servo_bus_split_config.py tests/test_node_metrics_wiring.py tests/test_battery_config.py -q`; expect failures on the follower's extra group and the absent base entry.
- Edit client.yml and the playbook step; run the verify command; commit `feat(ansible): swerve servos on their own bus via base_servos`.

#### Notes

- Design D1, D3, D4, D7, D8. The left-wheel inversion (32, 39) was verified on the bench 2026-10-03: carry it, never change IDs, modes or inversion.
- The head is not configured here (non-goal); only the comment.
- The feetech schema assert in `ros2_node_deploy` already validates the new entry's joints.
- If a pre-existing suite failure blocks the verify command, clear it per the boy-scout rule as its own commit.

#### Ruling

Accepted by the orchestrator: verified and merged (feetech 294, mcp_server 2499); docs-and-diagnostics failed only on a verifier crash, its checks pass by hand.

### S7: Bus budget test

- Status: done
- Files: `nodes/bridges/feetech_servos/tests/test_bus_budget.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint`
- Verified head: 4aee585dd897a6addb36627e31229fd77cf33ac8

#### Requirement

- New `tests/test_bus_budget.py` (the test is the deliverable, design D5). It loads the `lerobot_follower` and `base_servos` config blocks from `ansible/group_vars/client.yml` (repo root located from the test file) through `feetech_servos.config.load_config` (block written to `tmp_path`), builds emulated buses with `tests/sts_bus.py` (`make_bus`, `register_table`, `put_word`) holding exactly those servos, and runs the worst-case cycle of `tests/test_bus_io.py::test_follower_cycle_bus_time_keeps_the_control_rate`: every position goal due for refresh, every velocity joint's target changed, then one state sync read of all servos.
- Constants at the top: `MEASURED_SHARED_BUS_HZ = 95.7` (robot, 2026-10-11, shared 14-servo bus, the client.yml comment), the old shared layout (arm + the eight swerve joints, imported from `tests/test_bus_io.py`, not copied), `HEAD` = two position joints `head_pan`, `head_tilt` with IDs above every configured ID (used only while the base config has no `head` extra group; once the pan/tilt epic configures one, the test uses that real group instead), `ARM_HEADROOM = 0.95`, `BASE_HEADROOM = 0.8`.
- Projection: remainder = 1 / MEASURED_SHARED_BUS_HZ - fake wire time of the old shared layout with idle wheels; projected cycle of a layout = remainder + its worst-case fake wire time.
- Tests: arm-only (follower config) projected cycle <= ARM_HEADROOM / follower `control_loop_hz`; base + HEAD (base config plus HEAD as an extra group) projected cycle <= BASE_HEADROOM / base `control_loop_hz`; arm + HEAD projected cycle < 1 / follower `control_loop_hz` (U4: the head can also go on the arm bus); fake wire time alone < half the period on both buses; the loaded configs are the deployed shape (follower 6 joints at 100 Hz, base 8 joints at 50 Hz). Red evidence (the test guards config written by split-bus-group-vars): run the arm assertion once against the pre-split follower block (`git show <split commit>~1:ansible/group_vars/client.yml`, 14 servos, projected 10.45 ms > 9.5 ms) and report that it fails.
- `README.md` section `## Bus budget`: the anchored projection rule, the table of D5 (layout, fake wire time, projected cycle, capacity, target) as computed by the test, and how to re-anchor after a new robot measurement; the `publish_current_joints` cost sentence is restated for the 6-servo arm bus (+54 bytes per cycle).

#### Acceptance conditions

- a1: the arm-only bus keeps at least 5 percent headroom at its control rate [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a2: base plus a 2-servo head fit the base control rate with 20 percent headroom while driving [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a3: arm plus head would fit the arm control rate [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a4: the test reads the deployed configs from client.yml, not copies [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a5: README documents the bus budget [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Bus budget]
- a6: feetech_servos suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/tests/test_bus_budget.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- Reads contract `base_servos entry` and the `lerobot_follower` entry from client.yml.
- Reuses `tests/sts_bus.py` (`FakeStsPort.wire_time_s`, `make_bus`), `feetech_servos.bridge_cycle.BridgeCycle`, `feetech_servos.config.load_config`, and `ARM`, `STEER`, `DRIVE` from `tests/test_bus_io.py`.

#### Expected outcome

`uv run pytest tests/test_bus_budget.py` passes and its README table matches the planning probe (arm ~9.3 ms projected, base + head ~14.4 ms while driving).

#### Steps

- Write `tests/test_bus_budget.py`; show the arm assertion red against the pre-split follower block (14 servos project to 10.45 ms > 9.5 ms) and green on the split config: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q`.
- Write the README section with the numbers the test computes; run the verify command; commit `test(feetech_servos): bus budget of the arm and base buses`.

#### Notes

- Design D4, D5, D7. Planning probe (real st3215 handler, fake bus): shared idle 4.28 ms / driving 8.88 ms; arm 3.12; arm + head 3.44; base + head driving 8.24 (wire time).
- The projection keeps the non-bus remainder fixed: conservative, per-servo CPU savings are not credited. Do not tune the headroom constants to make a red test pass; a real shortfall is reported.
- No production code changes in this task.

#### Ruling

Accepted by the orchestrator: verified and merged (feetech 294, mcp_server 2499); docs-and-diagnostics failed only on a verifier crash, its checks pass by hand.

### S8: Serial device preflight

- Status: done
- Files: `ansible/roles/ros2_node_deploy/tasks/main.yml`, `ansible/group_vars/client.yml`, `tests/test_serial_devices.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_serial_devices.py tests/test_unit_cgroups.py tests/test_servo_bus_split_config.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible`
- Verified head: b3749383da29f5b9e5d25fecd5a73bb707bb56cb

#### Requirement

- `ansible/roles/ros2_node_deploy/tasks/main.yml`: right after `Validate feetech_servos node config schema (name+id per joint)` and before the uninstall and native deploy blocks, tasks tagged `[config]` that run when `node_present`, `node_enabled` and `node_type == "feetech_servos"` and the parsed config has `device`: `ansible.builtin.stat` the device path (following the by-id symlink); when it does not exist, list `/dev/serial/by-id/` (`changed_when: false`, tolerate a missing directory) and fail with a message naming the node, the configured path and the listing. Disabled, absent or device-less feetech nodes skip it. ansible-lint clean (no new skip).
- `ansible/group_vars/client.yml` `rplidar_a1` env gains `RPLIDAR_SERIAL_PORT=` the lidar's by-id path recorded in design.md "Assumptions and unknowns" A2, with a comment: a second USB-serial adapter can take `/dev/ttyUSB0` at boot.
- Tests first, seen failing: new `tests/test_serial_devices.py`: (1) the preflight tasks exist in main.yml, sit after the schema assert and before the first `include_tasks`/deploy block, are conditioned on `feetech_servos`, presence and enabled state, use `ansible.builtin.stat` on the config's `device` and fail with text containing `/dev/serial/by-id`; (2) every serial device named in client.yml (each feetech `device`, `RPLIDAR_SERIAL_PORT`) is a `/dev/serial/by-id/` path and they are pairwise distinct; (3) no client `ros2_nodes` config or env names `/dev/ttyUSB` or `/dev/ttyACM`.
- `ansible/README.md`: new section `## Serial devices` (by-id naming for every USB-serial adapter, how to find a path with `ls -l /dev/serial/by-id/`, the preflight failure message and the fix); `tests/README.md` rows.

#### Acceptance conditions

- a1: the preflight is ordered before any deploy step, guarded to enabled feetech nodes with a device, and fails listing /dev/serial/by-id [evidence: test: tests/test_serial_devices.py]
- a2: every client USB-serial device is a distinct by-id path, including the lidar [evidence: test: tests/test_serial_devices.py]
- a3: no client node names a ttyUSB or ttyACM device [evidence: command: ! git grep -n -E '/dev/tty(USB|ACM)' -- ansible/group_vars/client.yml]
- a4: ansible README documents serial device naming and the preflight [evidence: doc-section: ansible/README.md#Serial devices]
- a5: root tests, lint, ansible-lint and syntax check pass [evidence: command: uv run pytest tests/test_serial_devices.py tests/test_unit_cgroups.py tests/test_servo_bus_split_config.py -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible]

#### Files

- `ansible/roles/ros2_node_deploy/tasks/main.yml`
- `ansible/group_vars/client.yml`
- `tests/test_serial_devices.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces

- New tasks in `ansible/roles/ros2_node_deploy/tasks/main.yml` reusing the `_feetech_cfg` parse of the schema assert.
- Uses contract `base_servos entry` (its `device`).
- `RPLIDAR_SERIAL_PORT` env, read by `nodes/bridges/rplidar_a1/launch/rplidar_a1.launch.py` (`SERIAL_PORT_ENV`), no launch change.

#### Expected outcome

Deploying with an unplugged or renamed adapter stops with the by-id listing before any restart; the lidar opens its own adapter by id.

#### Steps

- Confirm design.md A2 holds the lidar's real by-id path; if not, stop as blocked (orchestrator fills it from runbook stage 0).
- Write `tests/test_serial_devices.py`; run `uv run pytest tests/test_serial_devices.py -q`; expect failures (no preflight task, lidar on the default ttyUSB0).
- Add the tasks, the env line and the README section; run the verify command; commit `feat(ansible): serial device preflight and by-id lidar port`.

#### Notes

- Design D2. Restarts are queued to the end of the play (`playbooks/tasks/start_ros_nodes.yml`), so failing here leaves every running unit as it was.
- The server's `lerobot_leader` device is already by-id; the preflight applies there too and must pass unchanged.
- Do not add a udev rule (D2 rejected it).

#### Ruling

Accepted by the orchestrator: verified and merged (feetech 294, mcp_server 2499); docs-and-diagnostics failed only on a verifier crash, its checks pass by hand.

### S9: Docs and diagnostics

- Status: done
- Files: `README.md`, `ansible/README.md`, `.claude/skills/ansible-deploy/SKILL.md`, `.claude/skills/teleop-diagnostics/SKILL.md`, `scripts/swerve_diag.sh`, `scripts/README.md`, `MEMORY.md`
- Verify: `bash -n scripts/swerve_diag.sh && ! git grep -n -E 'swerve servos (now )?run inside|shared with the swerve servos|on the arm bus|share the follower-arm serial bus|expected to be absent|swerve_drive_servos' -- README.md ansible/README.md .claude/skills scripts nodes && uv run poe lint`
- Verified head: unknown

#### Requirement

- Root `README.md` node table: `lerobot_follower` is the SO-101 arm alone on its USB adapter; the `lerobot_follower (group swerve_drive)` row becomes a `base_servos` row (feetech_servos, `/swerve_drive/joint_states` pub, `/swerve_drive/joint_commands` sub, 8x ST3215 IDs 32-39 on the second USB adapter, 50 Hz).
- `ansible/README.md`: wherever node instances or tiers are listed (the critical tier line names `swerve_drive_servos` today), `base_servos` is the running feetech_servos instance of the base bus (critical tier, metrics 19115, own adapter by id); the retired name is gone.
- `.claude/skills/ansible-deploy/SKILL.md`: rows `lerobot_follower` (SO-101 follower arm, its own USB bus) and `base_servos` replacing the retired `swerve_drive_servos` row (swerve servos 32-39, base bus on the second adapter; deploy together with `lerobot_follower` and `swerve_controller` when bus config changes).
- `.claude/skills/teleop-diagnostics/SKILL.md`: lerobot_follower drives the arm alone on its own bus; expected `/follower/joint_states` >= 98 Hz (shared-bus gate was 95 Hz) and how to check it (Grafana `rate(servo_cycle_duration_seconds_count{host="client",node="lerobot_follower"}[5m])`, or the topic_scraper workflow with `sample_seq` deltas); a symptoms row: base bus problems no longer stall teleop, check `scripts/swerve_diag.sh`.
- `scripts/swerve_diag.sh`: header data path names the `base_servos` feetech bridge on the second adapter; both service loops check `ros2-base_servos` and `ros2-swerve_controller` (not `lerobot_follower`); the "expected to be absent" line is gone. `scripts/README.md` swerve_diag paragraph matches.
- `MEMORY.md`: the `swerve_drive_servos` bullet under "Swerve drive and Nav2" is rewritten for `base_servos` and describes the current state (own adapter by id, 50 Hz, `node_name`, battery off, metrics 19115) and a new "Servo bus split" decision entry records D4/D5 (arm 100 Hz, base 50 Hz, anchored projection numbers, head room on either bus).
- TDD exception: documentation and diagnostic-message edits only; verified by `bash -n` and the grep in the verify command.

#### Acceptance conditions

- a1: no live doc or script says the swerve servos share the arm bus [evidence: command: ! git grep -n -E 'swerve servos (now )?run inside|shared with the swerve servos|on the arm bus|share the follower-arm serial bus|expected to be absent|swerve_drive_servos' -- README.md ansible/README.md .claude/skills scripts nodes]
- a2: swerve_diag.sh checks the base bridge service and parses [evidence: command: bash -n scripts/swerve_diag.sh && grep -q 'ros2-base_servos ros2-swerve_controller' scripts/swerve_diag.sh]
- a3: the teleop-diagnostics skill states the arm-only bus rate expectation [evidence: command: grep -q '98 Hz' .claude/skills/teleop-diagnostics/SKILL.md]
- a4: MEMORY.md records the bus split decision [evidence: doc-section: MEMORY.md#Servo bus split]
- a5: root lint passes [evidence: command: uv run poe lint]

#### Files

- `README.md`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `.claude/skills/teleop-diagnostics/SKILL.md`
- `scripts/swerve_diag.sh`
- `scripts/README.md`
- `MEMORY.md`

#### Interfaces

- Reads contract `base_servos entry` (names, rate, port) for the docs; no code interfaces.

#### Expected outcome

Every operator-facing text describes two buses: the arm alone on `lerobot_follower`, the base (and later the head) on `base_servos`.

#### Steps

- Edit the seven files; run the verify command; expect `bash -n` silent and no grep hits; commit `docs: arm and base on separate servo buses`.

#### Notes

- Design D9. The swerve controller, feetech and mcp_server READMEs are updated by their own tasks; docs/plans archives are never rewritten.
- MEMORY.md: update stale lines, do not rewrite unrelated history.

#### Ruling

Accepted by the orchestrator: verified and merged (feetech 294, mcp_server 2499); docs-and-diagnostics failed only on a verifier crash, its checks pass by hand.

### S10: Final suite and runbook

- Status: done
- Files: `tests/test_servo_bus_live_refs.py`, `tests/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe lint-ansible && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)`
- Verified head: 202f5160a100b489feaf7fa2288e7c9717983c01

#### Requirement

- New `tests/test_servo_bus_live_refs.py`, written first and seen failing if any hit remains: `test_no_live_text_puts_swerve_servos_on_the_arm_bus` runs `git grep -n -I -E` for `swerve servos (now )?run inside|shared with the swerve servos|on the arm bus|share the follower-arm serial bus|group .swerve_drive.|all servos incl. the swerve ones` over tracked files and fails on every hit outside `docs/plans/**`, `graphify-out/**` and this test file; `test_retired_swerve_drive_servos_name_is_gone` fails on any `swerve_drive_servos` outside `docs/plans/**`, `graphify-out/**`, `tests/**` and `MEMORY.md`; `test_no_feetech_entry_but_the_base_serves_swerve_ids` asserts that in client.yml only `base_servos` lists servo IDs 32-39 and only `lerobot_follower` lists IDs 1-6. `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, lint-ansible, test-ansible, feetech_servos, swerve_drive_controller, mcp_server).
- The staged runbook in this task's Steps is complete; the orchestrator executes it after merge (the implementer does not touch the robot).

#### Acceptance conditions

- a1: no live text puts the swerve servos on the arm bus [evidence: test: tests/test_servo_bus_live_refs.py::test_no_live_text_puts_swerve_servos_on_the_arm_bus]
- a5: the retired swerve_drive_servos name is gone from live files [evidence: test: tests/test_servo_bus_live_refs.py::test_retired_swerve_drive_servos_name_is_gone]
- a2: only the base entry serves the swerve IDs and only the follower the arm IDs [evidence: test: tests/test_servo_bus_live_refs.py::test_no_feetech_entry_but_the_base_serves_swerve_ids]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe lint-ansible && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)]
- a4: the runbook covers device identification, deploy order keeping teleop configured, least-dramatic motion first and both bus rate gates [evidence: judgement]

#### Files

- `tests/test_servo_bus_live_refs.py`
- `tests/README.md`

#### Interfaces

- None new; `tests/test_servo_bus_live_refs.py`.

#### Expected outcome

All suites green with a consistency check; the orchestrator rolls the split out on the robot with the runbook below.

#### Steps

Implementer:
- Write `tests/test_servo_bus_live_refs.py`; run `uv run pytest tests/test_servo_bus_live_refs.py -q`; fix any real hit it reports; run the full verify command and quote each summary line; commit `test: no live text puts the swerve servos on the arm bus`.

Orchestrator runbook (after merge, robot reachable; every deploy through the ansible-deploy skill, logged to `.logs/`; no manual SSH, read-only Ansible ad-hoc commands only where noted):
0. Devices (before plan-execute when design.md A1/A2 are still `PENDING`, else before step 1). The user plugs the second adapter into the client Pi; the base servo chain is NOT moved yet. Identify read-only: `cd ansible && ansible client -m ansible.builtin.command -a 'ls -l /dev/serial/by-id/'`. Expect three entries: `usb-1a86_USB_Single_Serial_5A7A059004-if00` (arm, unchanged), one new adapter entry (A1) and the lidar's (A2, the one linking to `ttyUSB*`, Silicon Labs CP210x). Write A1/A2 into design.md. If the new adapter shows no unique by-id entry, stop and ask the user (A3). Baseline for the record: Grafana MCP `query_prometheus` `rate(servo_cycle_duration_seconds_count{host="client",node="lerobot_follower"}[5m])` (about 95.7) and `/battery_state` voltage.
1. Software first, base chain still on the arm adapter. Agent idle check is built into the client deploy. `mkdir -p .logs && ./scripts/deploy-nodes.sh client lerobot_follower base_servos swerve_controller mcp_server rplidar_a1 monitoring 2>&1 | tee .logs/deploy-client-bus-split.log` (one Ansible run; the serial preflight fails it before any restart if a device is missing). Expect: verify green; `./scripts/teleop_diag.sh` shows all three teleop topics PRESENT (arm teleop configured throughout); `base_servos` logs `Waiting for servos: [32, 33, 34, 35, 36, 37, 38, 39]` (read with `ansible client -m ansible.builtin.command -a 'journalctl -u ros2-base_servos -n 30 --no-pager'`); no `/swerve_drive/joint_states` yet (no placeholder data); `/scan` still published (`./scripts/swerve_diag.sh` and the lidar unit active).
2. Rewire (user, physical). Ask the user to power the servos off, move the base servo chain (IDs 32-39) from the arm adapter to the second adapter, power on, and confirm. Nothing is deployed in this step.
3. Base bus up. `base_servos` log shows all eight servos found and torque enabled, no repeated read failures; `./scripts/swerve_diag.sh`: `ros2-base_servos` and `ros2-swerve_controller` active, `/swerve_drive/joint_states` about 50 Hz, `/odom` present.
4. Bus rate gates, idle. Grafana: `rate(servo_cycle_duration_seconds_count{host="client",node="lerobot_follower"}[5m])` >= 98 and `{node="base_servos"}` >= 48; p99 `histogram_quantile(0.99, rate(servo_cycle_duration_seconds_bucket{host="client",node="lerobot_follower"}[5m]))` <= 0.01. topic_scraper workflow: set `enabled: true` for `topic_scraper_api` in `ansible/group_vars/client.yml` (local edit, not committed), `./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-client-scraper.log`, then `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select '/follower/joint_states:.header.stamp' --select '/swerve_drive/joint_states:.header.stamp' --interval 1.0 > .logs/bus-rates-idle.ndjson` for 60 s; rate per topic = delta `sample_seq` / delta `received_at_ns`: follower >= 98 Hz, base >= 48 Hz.
5. Least-dramatic motion. Arm first: a gripper-only move of about 0.1 rad through test_joint_api (`python scripts/joint_api_client.py post --joint gripper <value>`), then the user drives leader teleop briefly; `/rules/leader_vs_follower` on the scraper shows tracking. Base: with the user's go-ahead and a clear floor, robot MCP `move_relative` 0.03 m forward (steering stays centred, about 3 cm), then 0.10 m while collecting `.logs/bus-rates-driving.ndjson` as in step 4: follower stays >= 98 Hz while the base drives (the shared bus projected 66 Hz), base >= 48 Hz; `/odom` advances about 0.10 m.
6. Consumers. `ansible client -m ansible.builtin.shell -a 'source /opt/ros/jazzy/setup.bash && source /etc/profile.d/ros2_dds.sh && ros2 topic info -v /swerve_drive/set_register && ros2 topic info /battery_state' -e ansible_shell_executable=/bin/bash`: `/swerve_drive/set_register` subscribed by `base_servos` and published by swerve_controller; `/battery_state` has exactly one publisher, voltage within 0.2 V of the baseline. Robot MCP `get_body_state` reports swerve servo temperatures (fl_drive ... rr_steer) within 2 minutes.
7. Restore `topic_scraper_api` to `enabled: false` (`git checkout ansible/group_vars/client.yml`) and redeploy it.
Rollback (only if step 3-6 fail and cannot be fixed): power off, move the base chain back to the arm adapter, `git revert` the epic's commits, push, redeploy the same node list.

#### Notes

- Design D10. A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.
- If the robot is offline, runbook steps 0-7 are reported as pending on the robot (each named), with everything else done.
- Servo safety (CLAUDE.md): gripper before any other arm joint; wheels at minimal speed and travel; stop and ask before anything larger.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
